"""Panel de ajuste en vivo del Follow Line (se ejecuta en el PC, no en el contenedor).

    tools/panel.sh            → abre http://localhost:8050

1. Se conecta al controlador (src/follow_line.py, puerto 6089 del contenedor)
   y se reconecta solo cada vez que pulsas play en la web.
2. Reenvía al navegador cada fotograma (gráficas) y los cambios de los sliders
   al controlador.
3. Graba en logs/:
     <fecha>_panel.csv          un fotograma por fila, con los parámetros vigentes
     <fecha>_panel_vueltas.csv  una fila por vuelta: tiempo, métricas y parámetros
   Cada play (o el botón «Nueva prueba») abre ficheros nuevos.
4. Cronometra vueltas con la odometría: vuelta = alejarse > 30 m del punto de
   partida y volver a < 2.5 m (como tools/vuelta.sh).
"""
import argparse
import asyncio
import csv
import json
import math
import time
from pathlib import Path

from aiohttp import WSMsgType, web

RAIZ = Path(__file__).resolve().parents[2]
ESTATICOS = Path(__file__).parent / "static"
PRESETS = Path(__file__).parent / "presets"
LOGS = RAIZ / "logs"
HISTORIAL = 2400  # fotogramas que recibe una pestaña nueva (2 min a 20 Hz)

COLS_DATO = ["t", "dt", "e", "kappa", "kappa_lejos", "y0", "rumbo", "validos", "z_max", "t_perc",
             "v", "w", "v_ref", "p", "i", "d", "ff", "x", "y", "yaw", "vx", "wz"]
COLS_METRICAS = ["vuelta", "tiempo_s", "tiempo_reloj_s", "distancia_m", "v_media", "e_abs_medio",
                 "e_rms", "e_max", "perdidos", "cambios_w_s", "w_abs_medio", "params_cambiados"]
R_LEJOS, R_CERCA = 30.0, 2.5


class Vueltas:
    """Cronómetro y métricas por vuelta a partir de la odometría."""

    def __init__(self):
        self.origen = None
        self.n = 0
        self._nueva(None)

    def _nueva(self, t):
        self.t0 = t
        self.t0_reloj = time.time()
        self.lejos = False
        self.muestras = 0
        self.s_e = self.s_e2 = self.e_max = self.s_w = self.s_v = 0.0
        self.perdidos = self.cambios = 0
        self.dist = 0.0
        self.w_ant = None
        self.xy_ant = None
        self.cambio_params = False

    def actual(self, t):
        return None if self.t0 is None or t is None else t - self.t0

    def dato(self, d, params):
        """Procesa un fotograma; devuelve la fila de la vuelta si se acaba de cerrar."""
        t, x, y = d.get("t"), d.get("x"), d.get("y")
        if self.t0 is None:
            self._nueva(t)
        if d.get("e") is None:
            self.perdidos += 1
        else:
            e = abs(d["e"])
            self.s_e += e
            self.s_e2 += e * e
            self.e_max = max(self.e_max, e)
            self.muestras += 1
        w = d.get("w") or 0.0
        if self.w_ant is not None and w * self.w_ant < 0:
            self.cambios += 1
        self.w_ant = w
        self.s_w += abs(w)
        self.s_v += d.get("v") or 0.0
        if x is None:
            return None
        if self.origen is None:
            self.origen = (x, y)
        if self.xy_ant:
            self.dist += math.hypot(x - self.xy_ant[0], y - self.xy_ant[1])
        self.xy_ant = (x, y)
        r = math.hypot(x - self.origen[0], y - self.origen[1])
        if r > R_LEJOS:
            self.lejos = True
        if not (self.lejos and r < R_CERCA):
            return None
        self.n += 1
        dur = t - self.t0
        total = self.muestras + self.perdidos
        fila = {
            "vuelta": self.n, "tiempo_s": round(dur, 2),
            "tiempo_reloj_s": round(time.time() - self.t0_reloj, 2),
            "distancia_m": round(self.dist, 1),
            "v_media": round(self.s_v / max(total, 1), 2),
            "e_abs_medio": round(self.s_e / max(self.muestras, 1), 1),
            "e_rms": round(math.sqrt(self.s_e2 / max(self.muestras, 1)), 1),
            "e_max": round(self.e_max, 0), "perdidos": self.perdidos,
            "cambios_w_s": round(self.cambios / max(dur, 1e-6), 2),
            "w_abs_medio": round(self.s_w / max(total, 1), 3),
            "params_cambiados": int(self.cambio_params),
        }
        fila.update(params)
        self._nueva(t)
        return fila


class Panel:
    def __init__(self, host, puerto):
        self.host, self.puerto = host, puerto
        self.navegadores = set()
        self.esquema, self.valores = [], {}
        self.historial = []
        self.vueltas_tabla = []
        self.conectado = False
        self.escritor = None  # writer TCP hacia el controlador
        self.marca = ""
        self.ult_t = None
        # Parámetros elegidos en el panel (sliders, presets, tabla). Se reaplican
        # solos cada vez que el controlador se conecta (cada play en la web), y si
        # se eligen sin controlador quedan esperando a que se conecte.
        self.deseados = {}
        self.f_csv = self.f_vueltas = None

    # --- ficheros ---------------------------------------------------------
    def nueva_prueba(self, motivo):
        for f in (self.f_csv, self.f_vueltas):
            if f:
                f.close()
        LOGS.mkdir(exist_ok=True)
        base = time.strftime("%Y%m%d-%H%M%S") + "_panel"
        nombres = [p["nombre"] for p in self.esquema]
        self.f_csv = open(LOGS / f"{base}.csv", "w", newline="", buffering=1)
        self.csv = csv.DictWriter(self.f_csv, COLS_DATO + nombres + ["vuelta", "marca"],
                                  extrasaction="ignore")
        self.csv.writeheader()
        self.f_vueltas = open(LOGS / f"{base}_vueltas.csv", "w", newline="", buffering=1)
        self.csv_vueltas = csv.DictWriter(self.f_vueltas, COLS_METRICAS + nombres, extrasaction="ignore")
        self.csv_vueltas.writeheader()
        self.base = base
        self.vueltas = Vueltas()
        self.vueltas_tabla = []
        self.historial = []
        print(f"[panel] {motivo}: grabando en logs/{base}.csv")

    # --- navegador --------------------------------------------------------
    async def difundir(self, msg):
        txt = json.dumps(msg)
        for ws in list(self.navegadores):
            try:
                await ws.send_str(txt)
            except ConnectionError:
                self.navegadores.discard(ws)

    def msg_estado(self):
        return {"tipo": "estado", "conectado": self.conectado, "fichero": getattr(self, "base", None),
                "presets": sorted(p.stem for p in PRESETS.glob("*.json"))}

    async def ws(self, request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self.navegadores.add(ws)
        await ws.send_str(json.dumps({"tipo": "hola", "params": self.esquema, "valores": self.valores,
                                      "historial": self.historial, "vueltas": self.vueltas_tabla}))
        await ws.send_str(json.dumps(self.msg_estado()))
        try:
            async for m in ws:
                if m.type == WSMsgType.TEXT:
                    await self.desde_navegador(json.loads(m.data))
        finally:
            self.navegadores.discard(ws)
        return ws

    async def desde_navegador(self, m):
        tipo = m.get("tipo")
        if tipo == "set":
            await self.pedir_params(m.get("valores", {}), "")
        elif tipo == "reset_pid":
            await self.al_controlador(m)
        elif tipo == "marca":
            self.marca = m.get("texto", "marca")
        elif tipo == "nueva_prueba" and self.esquema:
            self.nueva_prueba("nueva prueba")
            await self.difundir({"tipo": "hola", "params": self.esquema, "valores": self.valores,
                                 "historial": [], "vueltas": []})
            await self.difundir(self.msg_estado())
        elif tipo == "preset_guardar":
            PRESETS.mkdir(exist_ok=True)
            nombre = "".join(c for c in m.get("nombre", "") if c.isalnum() or c in "-_") or "preset"
            (PRESETS / f"{nombre}.json").write_text(json.dumps(self.valores, indent=2))
            await self.difundir(self.msg_estado())
        elif tipo == "preset_cargar":
            f = PRESETS / f"{m.get('nombre')}.json"
            if f.exists():
                await self.pedir_params(json.loads(f.read_text()), f"preset «{f.stem}» cargado")
            else:
                await self.difundir({"tipo": "aviso", "texto": f"no existe el preset «{m.get('nombre')}»"})

    async def pedir_params(self, cambios, texto):
        """Guarda los parámetros elegidos y se los manda al controlador si está conectado."""
        self.deseados.update(cambios)
        if self.conectado:
            await self.al_controlador({"tipo": "set", "valores": cambios})
            if texto:
                await self.difundir({"tipo": "aviso", "texto": texto})
        else:  # sin controlador: se ven en los sliders y se aplican al pulsar play
            self.valores = {**self.valores, **cambios}
            await self.difundir({"tipo": "params", "valores": self.valores, "pendiente": True})
            await self.difundir({"tipo": "aviso", "texto": (texto + ". " if texto else "")
                                 + "Sin controlador: se aplicarán al pulsar play"})

    async def al_controlador(self, m):
        if self.escritor:
            self.escritor.write((json.dumps(m) + "\n").encode())
            await self.escritor.drain()

    # --- controlador ------------------------------------------------------
    async def bucle_controlador(self):
        while True:
            try:
                lector, self.escritor = await asyncio.open_connection(self.host, self.puerto)
                async for linea in lector:
                    await self.desde_controlador(json.loads(linea))
            except (OSError, ValueError, asyncio.IncompleteReadError):
                pass
            if self.conectado:
                print("[panel] controlador desconectado")
            self.conectado, self.escritor = False, None
            await self.difundir(self.msg_estado())
            await asyncio.sleep(1.0)

    async def desde_controlador(self, m):
        tipo = m.get("tipo")
        if tipo == "hola":
            self.esquema, self.valores = m["params"], m["valores"]
            self.conectado = True
            self.ult_t = None
            self.nueva_prueba("controlador conectado")
            await self.difundir({"tipo": "hola", "params": self.esquema, "valores": self.valores,
                                 "historial": [], "vueltas": []})
            await self.difundir(self.msg_estado())
            distintos = {k: v for k, v in self.deseados.items() if self.valores.get(k) != v}
            if distintos:  # el controlador arranca con sus valores por defecto
                await self.al_controlador({"tipo": "set", "valores": distintos})
                await self.difundir({"tipo": "aviso", "texto": "parámetros del panel reaplicados: "
                                     + " ".join(f"{k}={v}" for k, v in distintos.items())})
        elif tipo == "params":
            if m["valores"] != self.valores:
                self.vueltas.cambio_params = True
            self.valores = m["valores"]
            await self.difundir(m)
        elif tipo == "dato":
            if self.ult_t is not None and m["t"] < self.ult_t - 0.5:  # simulación reiniciada
                self.nueva_prueba("simulación reiniciada")
                await self.difundir({"tipo": "hola", "params": self.esquema, "valores": self.valores,
                                     "historial": [], "vueltas": []})
                await self.difundir(self.msg_estado())
            self.ult_t = m["t"]
            fila_vuelta = self.vueltas.dato(m, self.valores)
            m["vuelta"] = self.vueltas.n + 1
            m["t_vuelta"] = self.vueltas.actual(m["t"])
            if self.marca:
                m["marca"] = self.marca
                self.marca = ""
            self.csv.writerow({**m, **self.valores})
            self.historial.append(m)
            del self.historial[:-HISTORIAL]
            await self.difundir(m)
            if fila_vuelta:
                self.vueltas_tabla.append(fila_vuelta)
                self.csv_vueltas.writerow(fila_vuelta)
                print(f"[panel] vuelta {fila_vuelta['vuelta']}: {fila_vuelta['tiempo_s']} s")
                await self.difundir({"tipo": "vuelta", "fila": fila_vuelta})


async def principal(args):
    panel = Panel(args.host, args.puerto)
    app = web.Application()
    app.router.add_get("/ws", panel.ws)
    app.router.add_get("/", lambda r: web.FileResponse(ESTATICOS / "index.html"))
    app.router.add_static("/static", ESTATICOS)
    corredor = web.AppRunner(app)
    await corredor.setup()
    await web.TCPSite(corredor, "127.0.0.1", args.web).start()
    print(f"[panel] abre http://localhost:{args.web}  (controlador en {args.host}:{args.puerto})")
    await panel.bucle_controlador()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--puerto", type=int, default=6089, help="puerto de telemetría del controlador")
    ap.add_argument("--web", type=int, default=8050, help="puerto del panel en el navegador")
    try:
        asyncio.run(principal(ap.parse_args()))
    except KeyboardInterrupt:
        pass
