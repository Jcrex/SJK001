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


def importar(estudio, cfg, de, espacio, previos=None):
    """Copia al estudio (nuevo) los ensayos de otros estudios, para no perder lo aprendido.

    Un parámetro del espacio que no existía en el estudio viejo toma su valor en
    `previos` (p. ej. k_hor = 0: las vueltas viejas se corrieron sin él, que equivale
    a 0). J se recalcula con el objetivo actual a partir de las métricas guardadas,
    así que también vale si se han cambiado los pesos. Se saltan los ensayos que
    caen fuera de los rangos nuevos. Los ensayos en cola del viejo pasan al nuevo.
    """
    previos = previos or {}
    dist = {k: optuna.distributions.FloatDistribution(lo, hi) for k, (lo, hi) in espacio.items()}
    total = 0
    for nombre in de:
        db = SALIDA / f"{nombre}.db"
        if not db.exists():
            print(f"[opt] AVISO: no existe {db}; no se importa", flush=True)
            continue
        viejo = optuna.load_study(study_name=nombre, storage=f"sqlite:///{db}")
        n = fuera = 0
        sobrantes = set()
        for t in viejo.trials:
            if t.state == optuna.trial.TrialState.WAITING:  # pedidos aún sin ensayar
                fijados = t.system_attrs.get("fixed_params") or t.params
                estudio.enqueue_trial(recortar({**fijados}, espacio), user_attrs=dict(t.user_attrs))
                continue
            if t.state != optuna.trial.TrialState.COMPLETE:
                continue
            params = {k: t.params.get(k, previos.get(k)) for k in dist}
            if any(v is None or not dist[k].low <= v <= dist[k].high for k, v in params.items()):
                fuera += 1
                continue
            sobrantes |= set(t.params) - set(dist)
            ua = dict(t.user_attrs)
            ua.setdefault("importado_de", f"{nombre}#{t.number}")
            fila = None if "tiempo_s" not in ua and "distancia_m" not in ua else ua
            estudio.add_trial(optuna.trial.create_trial(
                params=params, distributions=dist, value=objetivo(fila, cfg), user_attrs=ua))
            n += 1
        total += n
        print(f"[opt] importados {n} ensayos de «{nombre}»"
              + (f" ({fuera} fuera de la ventana, saltados)" if fuera else ""), flush=True)
        if sobrantes:
            print(f"[opt] AVISO: {sorted(sobrantes)} variaba en «{nombre}» y ahora está fijo", flush=True)
    return total


# --- Ventana móvil -------------------------------------------------------------
# El rango de cada parámetro es una ventana que se desplaza: si los mejores ensayos se
# pegan a un borde, o se pide desde el panel un valor de fuera, la ventana se mueve
# hacia allí. Optuna no admite cambiar el rango dentro de un estudio sin que el modelo
# multivariante deje fuera ese parámetro, así que cada movimiento crea una generación
# nueva del estudio (<estudio>_g<n>) que importa todos los ensayos que caen dentro.
# El estado (generación actual y ventanas) se guarda en <estudio>_ventana.json.

def _redondear(x):
    return float(f"{x:.6g}")


def _desplazar(lo, hi, nuevo_hi=None, nuevo_lo=None, vals=(), u=0.1, piso=None):
    """Desplaza [lo, hi] hasta nuevo_hi (o nuevo_lo) moviendo el otro borde lo mismo,
    pero sin dejar ningún valor de `vals` a menos de u·ancho del borde contrario
    (si no, la revisión siguiente devolvería la ventana: rebote)."""
    if nuevo_hi is not None:
        hi2 = nuevo_hi
        lo2 = lo + (hi2 - hi)
        if vals:
            lo2 = min(lo2, (min(vals) - u * hi2) / (1 - u))
        lo2 = lo2 if piso is None else max(lo2, piso)
    else:
        lo2 = nuevo_lo
        hi2 = hi - (lo - lo2)
        if vals:
            hi2 = max(hi2, (max(vals) - u * lo2) / (1 - u))
    return _redondear(lo2), _redondear(hi2)


def mover_ventana(espacio, top, pisos, topes, umbral, paso):
    """Ventanas nuevas si al menos la mitad de los mejores están a menos de
    umbral·ancho de un borde: se desplaza paso·ancho hacia ese lado. Devuelve
    {k: (lo, hi)} solo de las que cambian. Nunca baja del mínimo del controlador
    ni pasa de un tope duro."""
    cambios = {}
    for k, (lo, hi) in espacio.items():
        vals = [t[k] for t in top if k in t]
        if not vals:
            continue
        w, n = hi - lo, len(vals)
        piso, tope = pisos.get(k), topes.get(k)
        if 2 * sum(v >= hi - umbral * w for v in vals) >= n and (tope is None or hi < tope):
            hi2 = hi + paso * w if tope is None else min(hi + paso * w, tope)
            cambios[k] = _desplazar(lo, hi, nuevo_hi=hi2, vals=vals, u=umbral, piso=piso)
        elif 2 * sum(v <= lo + umbral * w for v in vals) >= n and (piso is None or lo > piso):
            lo2 = lo - paso * w if piso is None else max(lo - paso * w, piso)
            cambios[k] = _desplazar(lo, hi, nuevo_lo=lo2, vals=vals, u=umbral, piso=piso)
    return cambios


def incluir_valores(espacio, valores, top, pisos, topes, umbral=0.1):
    """Ventanas desplazadas para que entren los valores pedidos, con un margen de
    umbral·ancho. Como mover_ventana, deja a los mejores lejos del borde contrario."""
    cambios = {}
    for k, v in valores.items():
        if k not in espacio:
            continue
        lo, hi = espacio[k]
        w = hi - lo
        piso, tope = pisos.get(k), topes.get(k)
        v = max(v, piso) if piso is not None else v
        v = min(v, tope) if tope is not None else v
        vals = [t[k] for t in top if k in t] + [v]
        if v > hi:
            hi2 = v + umbral * w if tope is None else min(v + umbral * w, tope)
            cambios[k] = _desplazar(lo, hi, nuevo_hi=hi2, vals=vals, u=umbral, piso=piso)
        elif v < lo:
            lo2 = v - umbral * w if piso is None else max(v - umbral * w, piso)
            cambios[k] = _desplazar(lo, hi, nuevo_lo=lo2, vals=vals, u=umbral, piso=piso)
    return cambios


def clasificacion(estudio):
    """Parámetros agrupados por repeticiones y ordenados por J media.

    Cada repetición de los mismos parámetros es un ensayo aparte. La mejor vuelta
    suelta engaña: unos parámetros que completan 2 de 24 vueltas tienen un mínimo
    muy bueno y una media pésima (cada choque vale ~75). La media de J de todas las
    repeticiones es el coste esperado de una vuelta con esos parámetros.
    Devuelve [{"params", "J", "n", "ok", "tiempo_s", "ensayo"}], el mejor primero;
    solo grupos con al menos una vuelta completa. "ensayo" es el de mejor J.
    """
    grupos = {}
    for t in estudio.trials:
        if t.state == optuna.trial.TrialState.COMPLETE:
            grupos.setdefault(tuple(sorted((k, round(v, 5)) for k, v in t.params.items())), []).append(t)
    res = []
    for g in grupos.values():
        ok = [t for t in g if t.user_attrs.get("completada")]
        if ok:
            res.append({"params": g[0].params, "J": sum(t.value for t in g) / len(g), "n": len(g),
                        "ok": len(ok), "tiempo_s": round(sum(t.user_attrs["tiempo_s"] for t in ok) / len(ok), 2),
                        "ensayo": min(g, key=lambda t: t.value).number})
    return sorted(res, key=lambda r: (r["J"], -r["n"]))


def mejores(estudio, n):
    return [r["params"] for r in clasificacion(estudio)[:n]]


def a_confirmar(clas, hasta, top=5):
    """Parámetros que conviene repetir: el primero de los `top` mejores que lleva
    menos de `hasta` repeticiones (un campeón ya confirmado no se repite más)."""
    for r in clas[:top]:
        if r["n"] < hasta:
            return r
    return None


class Ventana:
    """Generación actual del estudio y sus rangos, guardados en <estudio>_ventana.json."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.base = cfg["estudio"]
        self.ruta = SALIDA / f"{self.base}_ventana.json"
        e = json.loads(self.ruta.read_text()) if self.ruta.exists() else {}
        self.gen = e.get("gen", 0)
        self.actual = e.get("actual", self.base)
        self.espacio = {k: tuple(v) for k, v in e.get("espacio", {}).items()}
        self.config_vista = {k: tuple(v) for k, v in e.get("config", {}).items()}
        self.historial = e.get("historial", [])
        self.config_orig = {k: tuple(v) for k, v in cfg["espacio"].items()}  # tal cual config.json

    def deseado(self):
        """Ventanas a usar al arrancar: las guardadas, salvo los parámetros cuyo rango
        se ha cambiado a mano en config.json desde la última vez (o que son nuevos)."""
        d = {}
        for k, r in self.config_orig.items():
            d[k] = self.espacio[k] if k in self.espacio and self.config_vista.get(k) == r else r
        return d

    def guardar(self):
        self.ruta.write_text(json.dumps({
            "actual": self.actual, "gen": self.gen, "espacio": self.espacio,
            "config": self.config_orig,
            "historial": self.historial}, indent=1))

    def abrir(self, nombre):
        return optuna.create_study(
            study_name=nombre, storage=f"sqlite:///{SALIDA / (nombre + '.db')}", load_if_exists=True,
            direction="minimize",
            sampler=optuna.samplers.TPESampler(multivariate=True, n_startup_trials=self.cfg["arranque_aleatorio"],
                                               seed=self.cfg["semilla_aleatoria"] + self.gen))

    def nueva_generacion(self, espacio, motivo):
        """Crea <base>_g<n+1> con estas ventanas e importa todo lo de la generación actual."""
        antes = dict(self.espacio)
        viejo = self.actual
        self.gen += 1
        self.actual = f"{self.base}_g{self.gen}"
        self.espacio = dict(espacio)
        estudio = self.abrir(self.actual)
        importar(estudio, self.cfg, [viejo], espacio,
                 (self.cfg.get("importar") or {}).get("valores_previos"))
        cambios = {k: (antes.get(k), v) for k, v in espacio.items() if antes.get(k) != v}
        self.historial.append({"fecha": time.strftime("%Y-%m-%d %H:%M:%S"), "gen": self.gen,
                               "motivo": motivo, "cambios": cambios})
        self.guardar()
        texto = "; ".join(f"{k} {a[0]:.4g}–{a[1]:.4g} → {b[0]:.4g}–{b[1]:.4g}" if a else f"{k} → {b[0]:.4g}–{b[1]:.4g}"
                          for k, (a, b) in cambios.items())
        print(f"[opt] ventana movida ({motivo}), generación {self.actual}: {texto}", flush=True)
        return estudio, texto


class Ensayador:
    """Habla con el panel por WebSocket y ejecuta una vuelta por ensayo."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.ws = None
        self.cola = asyncio.Queue()
        self.conectado = False
        self.valores = {}
        self.esquema = {}
        self.estudio = None  # para encolar los ensayos que se piden desde el panel
        self.vigilados = None  # valores del ensayo en curso; si cambian, el ensayo no vale
        self.pedidos = []  # ensayos del panel que necesitan mover la ventana (se hacen entre vueltas)
        self.pisos, self.topes = {}, {}
        self.interferido = None

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
                self.pisos = {p["nombre"]: p["min"] for p in d["params"]}
                self.topes = {p["nombre"]: p["max"] for p in d["params"] if p.get("tope_duro")}
            if t == "estado":
                self.conectado = d["conectado"]
            elif t in ("hola", "params"):
                self.valores = d.get("valores") or self.valores
            if t == "params" and self.vigilados is not None and not self.interferido:
                cambios = {k: v for k, v in d.get("valores", {}).items()
                           if k in self.vigilados and abs(v - self.vigilados[k]) > 1e-9}
                if cambios:
                    self.interferido = cambios
            if t == "encolar":
                await self.encolar(d.get("valores") or {})
            if t in ("params", "reinicio", "vuelta", "hola"):
                await self.cola.put(d)
        self.ws = None
        await self.cola.put({"tipo": "panel_caido"})

    async def encolar(self, valores):
        """Ensayo pedido desde el panel: va a la cola del estudio y el modelo aprende de él
        como de cualquier otro. Solo cuentan los parámetros del espacio de búsqueda. Si un
        valor cae fuera de la ventana, la ventana se desplaza para incluirlo (entre vueltas)."""
        espacio, fijos = self.cfg["espacio"], self.cfg["fijos"]
        valores = {k: float(v) for k, v in valores.items()}
        notas = []
        ignor = [k for k, v in valores.items() if k not in espacio and k in self.esquema
                 and abs(v - float(fijos.get(k, self.valores.get(k, v)))) > 1e-9]
        if ignor:
            notas.append("ignorados (no los optimiza este estudio): " + ", ".join(ignor))
        fuera = {k: valores[k] for k, (lo, hi) in espacio.items()
                 if k in valores and not lo <= valores[k] <= hi}
        params = {k: round(v, 5) for k, v in recortar(valores, espacio).items()}
        dif = [f"{k}={valores.get(k, v):.4g}" for k, v in params.items()
               if k in self.valores and abs(valores.get(k, v) - self.valores[k]) > 1e-9]
        if fuera:
            self.pedidos.append({k: valores.get(k, v) for k, v in params.items()})
            texto = ("Fuera de la ventana (" + ", ".join(f"{k}={v:.4g}" for k, v in fuera.items())
                     + "): la ventana se desplazará al acabar la vuelta en curso y se ensayará después")
        else:
            self.estudio.enqueue_trial(params, user_attrs={"manual": 1})
            en_cola = sum(t.state == optuna.trial.TrialState.WAITING for t in self.estudio.trials)
            texto = f"En cola del optimizador (posición {en_cola}): se ensaya tras la vuelta en curso"
        texto += ". " + ("Cambia " + ", ".join(dif) if dif else "Mismos valores que el ensayo en curso")
        texto += (". " + "; ".join(notas) if notas else "")
        print("[opt] " + texto, flush=True)
        await self.enviar({"tipo": "opt_aviso", "texto": texto})

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
        self.vigilados, self.interferido = dict(ack["valores"]), None
        try:
            return await self._correr()
        finally:
            self.vigilados = None

    async def _correr(self):
        await self.enviar({"tipo": "reiniciar"})
        r = await self.esperar(lambda m: m["tipo"] == "reinicio" and m["motivo"] == "pedido", 10)
        if r is None:
            return None, "sin_respuesta"
        if not r.get("ok"):
            return None, "reinicio_fallido"
        m = await self.esperar(lambda m: m["tipo"] in ("vuelta", "reinicio", "hola"),
                               self.cfg["t_max_ensayo"])
        if self.interferido:  # alguien cambió parámetros a mitad de vuelta
            print(f"[opt] parámetros cambiados durante el ensayo: {self.interferido}", flush=True)
            return None, "interferencia"
        if m is not None and m["tipo"] == "reinicio" and m["motivo"] == "pedido":
            return None, "interferencia"  # «Volver a la salida» pulsado a mano
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
    base = cfg["estudio"]
    vm = cfg.get("ventana_movil") or {}
    ven = Ventana(cfg)
    estudio = ven.abrir(ven.actual)
    for t in estudio.trials:  # ensayos a medias de una ejecución parada con Ctrl+C
        if t.state == optuna.trial.TrialState.RUNNING:
            estudio.tell(t.number, state=optuna.trial.TrialState.FAIL)
    if not estudio.trials:  # estudio nuevo
        ven.espacio = {k: tuple(v) for k, v in cfg["espacio"].items()}
        imp = cfg.get("importar") or {}
        if importar(estudio, cfg, imp.get("de", []), ven.espacio, imp.get("valores_previos")):
            # Probar a propósito los parámetros nuevos sobre el mejor importado: si todos los
            # ensayos viejos tienen el mismo valor, el modelo no sabría que merece la pena moverlos.
            mejor = clasificacion(estudio)[0]["params"] if clasificacion(estudio) else estudio.best_trial.params
            for k, valores in imp.get("probar_nuevos", {}).items():
                for v in valores:
                    estudio.enqueue_trial({**mejor, k: v})
                    print(f"[opt] en cola: mejor importado con {k} = {v}", flush=True)
        else:  # desde cero: empezar por los presets conocidos
            for s in cfg["semillas"]:
                f = PRESETS / f"{s}.json"
                if f.exists():
                    estudio.enqueue_trial(recortar(json.loads(f.read_text()), ven.espacio))
                    print(f"[opt] semilla: {s}")
        ven.guardar()
    elif not ven.espacio:  # estudio de antes de la ventana móvil: se adopta tal cual
        ven.espacio = ven.deseado()
        ven.guardar()
    elif ven.deseado() != ven.espacio:  # rangos cambiados a mano en config.json
        estudio, _ = ven.nueva_generacion(ven.deseado(), "config.json cambiado")
    cfg["espacio"] = espacio = dict(ven.espacio)
    fijos = cfg["fijos"]

    f_csv = SALIDA / f"{base}_ensayos.csv"
    nuevo = not f_csv.exists()
    salida_csv = open(f_csv, "a", newline="", buffering=1)
    cols = ["ensayo", "J", "completada", "motivo", "tiempo_s", "e_rms", "e_max", "cambios_w_s",
            "perdidos", "distancia_m", "fecha"] + list(espacio) + list(fijos)
    escritor = csv.DictWriter(salida_csv, cols + ["estudio"], extrasaction="ignore")
    if nuevo:
        escritor.writeheader()

    ens = Ensayador(cfg)
    ens.estudio = estudio
    async with aiohttp.ClientSession() as sesion:
        await ens.conectar(sesion)
        previos = sum(t.state.is_finished() for t in estudio.trials)
        print(f"[opt] estudio «{ven.actual}»: {previos} ensayos previos. Ctrl+C para parar.", flush=True)
        print("[opt] ventanas: " + "  ".join(f"{k} {lo:.4g}–{hi:.4g}" for k, (lo, hi) in espacio.items()),
              flush=True)
        await asyncio.sleep(1.0)  # recibir el esquema de parámetros del panel
        for k, (lo, hi) in espacio.items():
            if k not in ens.esquema:
                if ens.esquema:
                    print(f"[opt] AVISO: «{k}» no es un parámetro del controlador; se ignorará", flush=True)
            elif lo < ens.pisos[k] or (k in ens.topes and hi > ens.topes[k]):
                print(f"[opt] AVISO: ventana de «{k}» {lo}…{hi} fuera de lo que admite el controlador "
                      f"(mín. {ens.pisos[k]}); se recortará", flush=True)

        def cambiar_estudio(nuevas, motivo):
            nonlocal estudio, espacio
            estudio, texto = ven.nueva_generacion({**espacio, **nuevas}, motivo)
            cfg["espacio"] = espacio = dict(ven.espacio)
            ens.estudio = estudio
            return texto

        hechos = confirmado = 0
        clas = clasificacion(estudio)
        campeon = clas[0]["ensayo"] if clas else None
        while not cfg["max_ensayos"] or hechos < cfg["max_ensayos"]:
            # Cambios de ventana: solo entre vueltas
            if ens.pedidos:
                pedidos, ens.pedidos = ens.pedidos, []
                nuevas = {}
                for v in pedidos:
                    nuevas.update(incluir_valores({**espacio, **nuevas}, v, mejores(estudio, vm.get("top", 10)),
                                                  ens.pisos, ens.topes, vm.get("umbral", 0.1)))
                texto = cambiar_estudio(nuevas, "pedido desde el panel") if nuevas else ""
                for v in pedidos:
                    estudio.enqueue_trial({k: round(x, 5) for k, x in recortar(v, espacio).items()},
                                          user_attrs={"manual": 1})
                await ens.enviar({"tipo": "opt_aviso", "texto": "Ventana desplazada: " + texto
                                  + ". Tu ensayo va el siguiente" if texto else "Tu ensayo va el siguiente"})
            elif (vm.get("activa") and hechos and hechos % vm.get("cada", 10) == 0
                  and getattr(ens, "revisado", -1) != hechos):
                ens.revisado = hechos
                nuevas = mover_ventana(espacio, mejores(estudio, vm.get("top", 10)), ens.pisos, ens.topes,
                                       vm.get("umbral", 0.1), vm.get("paso", 0.25))
                if nuevas:
                    texto = cambiar_estudio(nuevas, "los mejores están en el borde")
                    await ens.enviar({"tipo": "opt_aviso", "texto": "Ventana desplazada: " + texto})
            clas = clasificacion(estudio)
            if (cfg["confirmar_cada"] and hechos and hechos % cfg["confirmar_cada"] == 0
                    and confirmado != hechos):
                confirmado = hechos
                c = a_confirmar(clas, cfg.get("confirmar_hasta", 3))
                if c:
                    estudio.enqueue_trial(c["params"], user_attrs={"confirmacion": 1})
                    print(f"[opt] confirmación: ensayo #{c['ensayo']} ({c['ok']}/{c['n']} completas)", flush=True)
            trial = estudio.ask()
            params = {k: trial.suggest_float(k, lo, hi) for k, (lo, hi) in espacio.items()}
            params = {k: round(v, 5) for k, v in params.items()}
            envio = {**fijos, **params}
            mejor = None
            if clas:
                b = clas[0]
                mejor = {"valor": b["J"], "ensayo": b["ensayo"], "tiempo_s": b["tiempo_s"],
                         "repes": f"{b['ok']}/{b['n']}"}
            await ens.estado(activo=True, estudio=ven.actual, ensayo=trial.number, fase="corriendo",
                             mejor=mejor, ultimo=getattr(ens, "ultimo", None),
                             ventanas={k: list(v) for k, v in espacio.items()})
            try:
                fila, motivo = await ens.vuelta(envio)
            except ConnectionError:
                estudio.tell(trial, state=optuna.trial.TrialState.FAIL)
                print("[opt] el panel se cerró; reconectando...", flush=True)
                await ens.conectar(sesion)
                continue
            if motivo in ("sin_respuesta", "reinicio_fallido", "interrumpido", "interferencia"):
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
                               "fecha": time.strftime("%Y-%m-%d %H:%M:%S"), "estudio": ven.actual})
            res = (f"{datos.get('tiempo_s', '–')} s, e_rms {datos.get('e_rms', '–')}"
                   if datos["completada"] else f"FALLO ({motivo})")
            ens.ultimo = f"#{trial.number}: J={J:.2f} · {res}"
            print(f"[opt] {ens.ultimo}  v_max={params.get('v_max')} kp={params.get('kp')} "
                  f"td={params.get('td')} k_ff={params.get('k_ff')} k_hor={params.get('k_hor')}",
                  flush=True)
            clas = clasificacion(estudio)
            if clas and clas[0]["ensayo"] != campeon:
                b = clas[0]
                campeon = b["ensayo"]
                PRESETS.mkdir(exist_ok=True)
                (PRESETS / f"optuna_{base}.json").write_text(json.dumps({**fijos, **b["params"]}, indent=2))
                print(f"[opt] ★ nuevo mejor: #{b['ensayo']} J media={b['J']:.2f} ({b['tiempo_s']} s, "
                      f"{b['ok']}/{b['n']} completas) → preset optuna_{base}", flush=True)
        await ens.estado(activo=False, estudio=ven.actual, ensayo=len(estudio.trials), fase="terminado",
                         mejor=None, ultimo=getattr(ens, "ultimo", None))


if __name__ == "__main__":
    ruta = sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent / "config.json"
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    try:
        asyncio.run(principal(ruta))
    except KeyboardInterrupt:
        print("\n[opt] parado. Se reanuda con el mismo comando.")
