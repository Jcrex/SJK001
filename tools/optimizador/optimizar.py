"""Optimizador automático de los parámetros del Follow Line (optimización bayesiana).

    tools/optimizar.sh [config.json]

Necesita el panel (tools/panel.sh) en marcha y el controlador corriendo (play en
la web). Funciona como un cliente más del panel, así que en el navegador se ven
las gráficas, el mapa y la tabla de cada ensayo mientras optimiza.

Cada ensayo es una vuelta desde parado:
  1. el modelo (Optuna, TPE) propone parámetros;
  2. se mandan al controlador y se devuelve el coche a la salida;
  3. se espera a que acabe la vuelta, choque o se agote el tiempo;
  4. se calcula J (ver config.json) y el modelo aprende de ese resultado.

Todo se guarda en logs/optuna/<estudio>.db: se puede parar con Ctrl+C y reanudar
con el mismo comando. El mejor resultado se guarda también como preset del panel
(tools/panel/presets/optuna_<estudio>.json). Informe: tools/optimizador/informe.py.

Por qué no deep learning: cada evaluación es una vuelta de ~20 s, así que en una
noche hay unos pocos cientos de datos. La optimización bayesiana está hecha para
eso (pocas evaluaciones caras); el aprendizaje por refuerzo profundo necesitaría
millones de pasos de simulación.
"""
import asyncio
import csv
import json
import math
import sys
import time
from pathlib import Path

import aiohttp
import optuna

RAIZ = Path(__file__).resolve().parents[2]
PRESETS = RAIZ / "tools" / "panel" / "presets"
SALIDA = RAIZ / "logs" / "optuna"


def cargar_config(ruta):
    cfg = json.loads(Path(ruta).read_text())
    return {k: v for k, v in cfg.items() if not k.startswith("_")}


def objetivo(fila, cfg):
    """J a minimizar a partir de la fila de la vuelta (o None si no hubo fila)."""
    o = cfg["objetivo"]
    if fila and fila.get("completada", 1):
        return (fila["tiempo_s"] + o["peso_e_rms"] * fila["e_rms"] / 100.0
                + o["peso_cambios_w"] * fila["cambios_w_s"])
    hecho = min((fila or {}).get("distancia_m", 0.0) / o["distancia_vuelta"], 1.0)
    return o["penal_fallo"] * (2.0 - hecho)


def recortar(valores, espacio):
    """Valores de un preset recortados al espacio de búsqueda (para las semillas)."""
    return {k: min(max(float(valores.get(k, lo)), lo), hi) for k, (lo, hi) in espacio.items()}


class Ensayador:
    """Habla con el panel por WebSocket y ejecuta una vuelta por ensayo."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.ws = None
        self.cola = asyncio.Queue()
        self.conectado = False
        self.valores = {}
        self.esquema = {}

    async def conectar(self, sesion):
        while True:
            try:
                self.ws = await sesion.ws_connect(self.cfg["panel"], heartbeat=20)
                asyncio.create_task(self._leer())
                return
            except aiohttp.ClientError:
                print("[opt] esperando al panel (tools/panel.sh)...", flush=True)
                await asyncio.sleep(3)

    async def _leer(self):
        async for m in self.ws:
            if m.type != aiohttp.WSMsgType.TEXT:
                continue
            d = json.loads(m.data)
            t = d.get("tipo")
            if t == "hola" and d.get("params"):
                self.esquema = {p["nombre"]: (p["min"], p["max"]) for p in d["params"]}
            if t == "estado":
                self.conectado = d["conectado"]
            elif t in ("hola", "params"):
                self.valores = d.get("valores") or self.valores
            if t in ("params", "reinicio", "vuelta", "hola"):
                await self.cola.put(d)
        self.ws = None
        await self.cola.put({"tipo": "panel_caido"})

    async def enviar(self, m):
        await self.ws.send_str(json.dumps(m))

    async def esperar(self, condicion, plazo):
        """Primer mensaje que cumple la condición, o None si se agota el plazo."""
        fin = time.time() + plazo
        while (resto := fin - time.time()) > 0:
            try:
                m = await asyncio.wait_for(self.cola.get(), resto)
            except asyncio.TimeoutError:
                return None
            if m["tipo"] == "panel_caido":
                raise ConnectionError("el panel se ha cerrado")
            if condicion(m):
                return m
        return None

    async def listo(self):
        """Espera a que haya un controlador conectado al panel."""
        aviso = 0.0
        while not self.conectado:
            if time.time() - aviso > 30:
                print("[opt] esperando al controlador (play en la web)...", flush=True)
                aviso = time.time()
            await asyncio.sleep(1)

    async def vuelta(self, params):
        """Aplica los parámetros, vuelve a la salida y corre una vuelta. Devuelve
        (fila de la vuelta o None, motivo)."""
        await self.listo()
        while not self.cola.empty():
            self.cola.get_nowait()
        await self.enviar({"tipo": "set", "valores": params})
        ack = await self.esperar(lambda m: m["tipo"] == "params", 5)
        if not ack:
            return None, "sin_respuesta"
        recortados = {k: ack["valores"].get(k) for k, v in params.items()
                      if k in ack["valores"] and abs(ack["valores"][k] - v) > 1e-9}
        if recortados:  # fuera del rango del controlador: se usa el valor recortado
            print(f"[opt] aviso: el controlador recortó {recortados}", flush=True)
        await self.enviar({"tipo": "reiniciar"})
        r = await self.esperar(lambda m: m["tipo"] == "reinicio" and m["motivo"] == "pedido", 10)
        if r is None:
            return None, "sin_respuesta"
        if not r.get("ok"):
            return None, "reinicio_fallido"
        m = await self.esperar(lambda m: m["tipo"] in ("vuelta", "reinicio", "hola"),
                               self.cfg["t_max_ensayo"])
        if m is None:
            await self.enviar({"tipo": "reiniciar"})
            return None, "timeout"
        if m["tipo"] == "hola":  # el controlador se reinició (nuevo play o recarga de la web)
            return None, "interrumpido"
        if m["tipo"] == "vuelta":
            return m["fila"], "" if m["fila"]["completada"] else m["fila"]["motivo"]
        # reinicio por choque: la fila del intento llega justo después (si llegó a moverse)
        f = await self.esperar(lambda x: x["tipo"] == "vuelta", 1.0)
        return (f["fila"] if f else None), m["motivo"]

    async def estado(self, **kw):
        if self.ws:
            await self.enviar(dict(tipo="opt_estado", **kw))


async def principal(ruta_cfg):
    cfg = cargar_config(ruta_cfg)
    SALIDA.mkdir(parents=True, exist_ok=True)
    nombre = cfg["estudio"]
    estudio = optuna.create_study(
        study_name=nombre, storage=f"sqlite:///{SALIDA / (nombre + '.db')}", load_if_exists=True,
        direction="minimize",
        sampler=optuna.samplers.TPESampler(multivariate=True, n_startup_trials=cfg["arranque_aleatorio"],
                                           seed=cfg["semilla_aleatoria"]))
    espacio, fijos = cfg["espacio"], cfg["fijos"]
    for t in estudio.trials:  # ensayos a medias de una ejecución parada con Ctrl+C
        if t.state == optuna.trial.TrialState.RUNNING:
            estudio.tell(t.number, state=optuna.trial.TrialState.FAIL)
    if not estudio.trials:  # estudio nuevo: empezar por los presets conocidos
        for s in cfg["semillas"]:
            f = PRESETS / f"{s}.json"
            if f.exists():
                estudio.enqueue_trial(recortar(json.loads(f.read_text()), espacio))
                print(f"[opt] semilla: {s}")
    f_csv = SALIDA / f"{nombre}_ensayos.csv"
    nuevo = not f_csv.exists()
    salida_csv = open(f_csv, "a", newline="", buffering=1)
    cols = ["ensayo", "J", "completada", "motivo", "tiempo_s", "e_rms", "e_max", "cambios_w_s",
            "perdidos", "distancia_m", "fecha"] + list(espacio) + list(fijos)
    escritor = csv.DictWriter(salida_csv, cols, extrasaction="ignore")
    if nuevo:
        escritor.writeheader()

    ens = Ensayador(cfg)
    async with aiohttp.ClientSession() as sesion:
        await ens.conectar(sesion)
        previos = sum(t.state.is_finished() for t in estudio.trials)
        print(f"[opt] estudio «{nombre}»: {previos} ensayos previos. Ctrl+C para parar.", flush=True)
        await asyncio.sleep(1.0)  # recibir el esquema de parámetros del panel
        for k, (lo, hi) in espacio.items():
            if k not in ens.esquema:
                if ens.esquema:
                    print(f"[opt] AVISO: «{k}» no es un parámetro del controlador; se ignorará", flush=True)
            elif lo < ens.esquema[k][0] or hi > ens.esquema[k][1]:
                print(f"[opt] AVISO: rango de «{k}» {lo}…{hi} fuera del del controlador "
                      f"{ens.esquema[k][0]}…{ens.esquema[k][1]}; se recortará", flush=True)
        hechos = confirmado = 0
        while not cfg["max_ensayos"] or hechos < cfg["max_ensayos"]:
            completos = [t for t in estudio.trials if t.state == optuna.trial.TrialState.COMPLETE]
            if (cfg["confirmar_cada"] and hechos and hechos % cfg["confirmar_cada"] == 0
                    and completos and confirmado != hechos):
                confirmado = hechos
                estudio.enqueue_trial(estudio.best_trial.params, user_attrs={"confirmacion": 1})
            trial = estudio.ask()
            params = {k: trial.suggest_float(k, lo, hi) for k, (lo, hi) in espacio.items()}
            params = {k: round(v, 5) for k, v in params.items()}
            envio = {**fijos, **params}
            mejor = None
            if completos:
                b = estudio.best_trial
                mejor = {"valor": b.value, "ensayo": b.number, "tiempo_s": b.user_attrs.get("tiempo_s")}
            await ens.estado(activo=True, estudio=nombre, ensayo=trial.number, fase="corriendo",
                             mejor=mejor, ultimo=getattr(ens, "ultimo", None))
            try:
                fila, motivo = await ens.vuelta(envio)
            except ConnectionError:
                estudio.tell(trial, state=optuna.trial.TrialState.FAIL)
                print("[opt] el panel se cerró; reconectando...", flush=True)
                await ens.conectar(sesion)
                continue
            if motivo in ("sin_respuesta", "reinicio_fallido", "interrumpido"):
                # no es culpa de los parámetros: se descarta y se repite más tarde
                estudio.tell(trial, state=optuna.trial.TrialState.FAIL)
                print(f"[opt] ensayo {trial.number} descartado ({motivo}); reintento en 5 s", flush=True)
                await asyncio.sleep(5)
                continue
            J = objetivo(fila, cfg)
            datos = dict(fila or {}, motivo=motivo, completada=int(bool(fila and fila.get("completada"))))
            for k in ("tiempo_s", "e_rms", "e_max", "cambios_w_s", "perdidos", "distancia_m"):
                if k in datos:
                    trial.set_user_attr(k, datos[k])
            trial.set_user_attr("completada", datos["completada"])
            trial.set_user_attr("motivo", motivo)
            estudio.tell(trial, J)
            hechos += 1
            escritor.writerow({**datos, **envio, "ensayo": trial.number, "J": round(J, 3),
                               "fecha": time.strftime("%Y-%m-%d %H:%M:%S")})
            res = (f"{datos.get('tiempo_s', '–')} s, e_rms {datos.get('e_rms', '–')}"
                   if datos["completada"] else f"FALLO ({motivo})")
            ens.ultimo = f"#{trial.number}: J={J:.2f} · {res}"
            print(f"[opt] {ens.ultimo}  v_max={params.get('v_max')} kp={params.get('kp')} "
                  f"td={params.get('td')} k_ff={params.get('k_ff')} t_mirar={params.get('t_mirar')}",
                  flush=True)
            if datos["completada"] and estudio.best_trial.number == trial.number:
                PRESETS.mkdir(exist_ok=True)
                (PRESETS / f"optuna_{nombre}.json").write_text(json.dumps(envio, indent=2))
                print(f"[opt] ★ nuevo mejor: J={J:.2f} ({datos['tiempo_s']} s) → preset optuna_{nombre}",
                      flush=True)
        await ens.estado(activo=False, estudio=nombre, ensayo=len(estudio.trials), fase="terminado",
                         mejor=None, ultimo=getattr(ens, "ultimo", None))


if __name__ == "__main__":
    ruta = sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent / "config.json"
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    try:
        asyncio.run(principal(ruta))
    except KeyboardInterrupt:
        print("\n[opt] parado. Se reanuda con el mismo comando.")
