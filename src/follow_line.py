"""Follow Line F1 — percepción por scanlines + PID del T03 ajustable en vivo.

Fichero único: se pega tal cual en el editor del ejercicio. Las funciones de
percepción y control no dependen del simulador, así que las pruebas
(tests/) las importan sin HAL.

Ajuste en vivo: abre un servidor de telemetría en el puerto 6089. El panel
(tools/panel.sh, en el PC) se conecta, dibuja las gráficas, cambia los
parámetros de PARAMS sin reiniciar y graba todo en logs/. Sin panel, el
controlador funciona igual. Los valores por defecto son la combinación elegida
en el macroanálisis final (docs/ESTUDIO_PID.md §18): 5.29 s de mediana y 63 de 65
vueltas completas a v_max = 93 m/s. FASE1 guarda el control P de las fases 1–2.

Geometría (docs/PLAN.md §5.1 y §5.3): cámara pinhole de 640×480 y FOV horizontal
de 90°, con H y CY calibrados en el simulador. Un píxel (x, y) por debajo del
horizonte corresponde a un punto del suelo a Z m por delante y Y m a la izquierda:
    Z = F·H / (y − CY)        Y = −H·(x − CX) / (y − CY)
"""
import os
import time

import cv2
import numpy as np

# --- Cámara -----------------------------------------------------------------
ANCHO, ALTO = 640, 480
# CY y H calibrados en el simulador (tools/sim/calibrar_h.py, desplazamientos
# laterales conocidos): el coche apoya ~0.5° morro abajo, así que el horizonte
# sube de la fila 239.5 teórica a la 237.45. Con 239.5 las filas lejanas se
# situaban casi al doble de distancia y κ salía ~3 veces más pequeña.
CX, CY = 320.0, 237.45  # punto principal (CY = horizonte efectivo)
F = 320.0  # px: (640/2) / tan(90°/2)
H = 0.1015  # m: altura efectiva de la cámara
CX_IMG = CX  # compatibilidad con la fase 1

# Filas de scanline, de cerca a lejos. Entre paréntesis, la distancia por delante (Z):
# 470 (0.14 m) 400 (0.20) 340 (0.32) 300 (0.52) 280 (0.76) 268 (1.06) 260 (1.43)
# 255 (1.85) 251 (2.4) 248 (3.1) 246 (3.8) 245 (4.3) 244 (5.0) 243 (5.9) 242 (7.1)
# 241 (9.1) 240 (12.7). La cámara corta a 15 m (fila ≈ 239.6).
FILAS = np.array([470, 400, 340, 300, 280, 268, 260, 255, 251, 248, 246, 245, 244, 243, 242, 241, 240])
FILAS_CERCA = FILAS >= 390  # filas que dan el error de posición actual
Z_LEJOS = 1.5  # m: a partir de aquí los puntos cuentan para κ_lejos
K_RECTA = 0.02  # 1/m: |κ| por debajo → recta (radio > 50 m)

# Banda del horizonte: n filas seguidas desde la primera en la que se ve el suelo
# (la cámara corta a 15 m, fila ≈ 239.6). Ahí la curva se ve antes que en ningún
# otro sitio: la columna media de la línea en la banda dice hacia dónde gira la pista.
SALTO_HOR = 50  # px: salto máx. de la línea entre filas de la banda (evita otros tramos)

ROJO_BAJO = ((0, 100, 60), (10, 255, 255))
ROJO_ALTO = ((170, 100, 60), (180, 255, 255))

# --- Parámetros ajustables en vivo -------------------------------------------
# (nombre, defecto, mín, máx, paso, grupo, descripción). El panel (tools/panel)
# genera un slider por fila y los cambios llegan al bucle sin reiniciar.
# Un parámetro se puede fijar también con la variable de entorno SJK_<NOMBRE>.
# Los valores por defecto son los elegidos en el macroanálisis final (preset w14_v93_v6).
PARAMS = [
    ("v_max", 93.0, 1.0, 120.0, 0.5, "Velocidad", "m/s en recta"),
    ("k_v_curva", 1.69696, 0.0, 200.0, 1.0, "Velocidad",
     "v_ref = v_max / (1 + k·|κ_lejos|). 0 = velocidad constante"),
    ("v_min", 7.79264, 1.0, 20.0, 0.5, "Velocidad", "m/s mínima del perfil de velocidad"),
    ("a_sub", 0.0, 0.0, 50.0, 0.5, "Velocidad", "m/s² máx. al acelerar. 0 = sin límite"),
    ("a_fren", 0.0, 0.0, 100.0, 1.0, "Velocidad", "m/s² máx. al frenar. 0 = sin límite"),
    ("kp", 0.01139, 0.0, 0.04, 0.0005, "Giro (PID)", "rad/s por píxel de error"),
    ("ti", 0.0, 0.0, 10.0, 0.05, "Giro (PID)", "s, tiempo integral. 0 = sin I"),
    ("td", 0.06318, 0.0, 0.5, 0.005, "Giro (PID)", "s, tiempo derivativo. 0 = sin D"),
    ("n_filtro", 7.41661, 1.0, 30.0, 1.0, "Giro (PID)", "N del filtro de D: τd = Td/N"),
    ("w_max", 14.0, 0.5, 20.0, 0.1, "Giro (PID)", "rad/s, saturación de w (anti-windup)"),
    ("kv_kp", 0.00054, 0.0, 0.5, 0.005, "Giro (PID)", "gain scheduling: Kp_ef = Kp / (1 + kv·v)"),
    ("k_ff", 0.64259, 0.0, 3.0, 0.05, "Prealimentación", "w_ff = k_ff · v · κ"),
    ("t_mirar", 0.11597, 0.0, 1.5, 0.02, "Mirada adelantada",
     "s: el error se mide a L = v·t m por delante (pure pursuit). 0 = filas cercanas (fases 1–2)"),
    ("z_mirar_min", 0.36997, 0.2, 6.0, 0.1, "Mirada adelantada", "m, L mínima (a poca velocidad)"),
    ("k_hor", 0.0357, 0.0, 0.08, 0.0005, "Horizonte",
     "rad/s por px: w_hor = −k_hor·e_hor, gira hacia donde va la línea en el horizonte. 0 = desactivado"),
    ("fila_hor", 240.0, 236.0, 300.0, 1.0, "Horizonte", "primera fila de la banda (la del horizonte)"),
    ("n_filas_hor", 10.0, 1.0, 40.0, 1.0, "Horizonte", "filas de la banda (azul en la imagen)"),
    ("v_perdida", 4.0, 0.0, 10.0, 0.5, "Otros", "m/s cuando no se ve la línea"),
    ("reinicio_auto", 1.0, 0.0, 1.0, 1.0, "Recuperación",
     "1 = si pierde la línea o se atasca, vuelve solo a la salida"),
    ("t_perdida_max", 0.8, 0.2, 5.0, 0.1, "Recuperación", "s seguidos sin línea → reinicio"),
    ("t_atasco", 0.7, 0.3, 5.0, 0.1, "Recuperación",
     "s con velocidad real < 40 % de la mandada (choque) → reinicio"),
]
_ALIAS_ENTORNO = {"v_max": "SJK_V"}  # nombres de las fases 1–2
# Control P de las fases 1–2 (los valores por defecto de entonces), para comparar y para las pruebas
FASE1 = {"v_max": 10.0, "k_v_curva": 0.0, "v_min": 4.0, "kp": 0.008, "td": 0.0, "n_filtro": 10.0, "w_max": 6.0,
         "kv_kp": 0.0, "k_ff": 0.0, "t_mirar": 0.0, "z_mirar_min": 1.0, "k_hor": 0.0, "v_perdida": 1.0,
         "t_perdida_max": 1.0, "t_atasco": 1.0}
# El máximo de PARAMS es solo el recorrido inicial del slider: no se recorta por arriba,
# para poder explorar sin techo (el optimizador desplaza su ventana). Solo estos
# parámetros tienen un máximo de verdad. El mínimo sí se respeta siempre.
TOPE_DURO = {"reinicio_auto"}


def params_por_defecto():
    d = {}
    for nombre, defecto, *_ in PARAMS:
        txt = os.environ.get(_ALIAS_ENTORNO.get(nombre, ""), os.environ.get("SJK_" + nombre.upper()))
        d[nombre] = float(txt) if txt else defecto
    return d


def params_fase1():
    return dict(params_por_defecto(), **FASE1)


def ajustar_params(actual, cambios):
    """Aplica cambios {nombre: valor}: recorta al mínimo (y al máximo solo los de
    TOPE_DURO); ignora nombres desconocidos."""
    rangos = {p[0]: (p[2], p[3]) for p in PARAMS}
    nuevo = dict(actual)
    for k, v in cambios.items():
        if k in rangos:
            v = max(float(v), rangos[k][0])
            nuevo[k] = min(v, rangos[k][1]) if k in TOPE_DURO else v
    return nuevo


KP = FASE1["kp"]  # compatibilidad con control_p de la fase 1
LOG = os.environ.get("SJK_LOG")  # ruta del CSV; sin definir no se guarda nada
PUERTO_PANEL = int(os.environ.get("SJK_PANEL_PORT", 6089))  # mapeado por docker (6080–6090)


# --- Geometría --------------------------------------------------------------
def imagen_a_suelo(x, y):
    """Píxeles (x, y) → suelo (Z adelante, Y izquierda), en metros."""
    dy = np.asarray(y, float) - CY
    return F * H / dy, -H * (np.asarray(x, float) - CX) / dy


def suelo_a_imagen(z, y):
    """Suelo (Z, Y) → píxeles, como array N×2 de (x, y)."""
    z, y = np.asarray(z, float), np.asarray(y, float)
    return np.column_stack([CX - F * y / z, CY + F * H / z])


# --- Percepción -------------------------------------------------------------
def mascara_rojo(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    return cv2.bitwise_or(cv2.inRange(hsv, *ROJO_BAJO), cv2.inRange(hsv, *ROJO_ALTO))


def centro_linea(img_bgr, franja=(250, 480)):
    """Fase 1: columna del centroide del rojo en la franja, o None si no hay rojo."""
    m = cv2.moments(mascara_rojo(img_bgr[franja[0]:franja[1]]), binaryImage=True)
    return None if m["m00"] == 0 else m["m10"] / m["m00"]


def _tramos(fila_mask):
    """Centros de los tramos contiguos de rojo de una fila."""
    b = np.concatenate(([0], (fila_mask > 0).view(np.int8), [0]))
    d = np.flatnonzero(np.diff(b))
    ini, fin = d[0::2], d[1::2]
    return (ini + fin - 1) / 2.0


def centros_filas(img_bgr, filas=FILAS):
    """Centro de la línea en cada fila (NaN si no hay rojo).

    Se recorre de cerca a lejos y, si una fila tiene varios tramos, se elige el
    más cercano al último punto válido: así se sigue la línea y no otro trozo
    del circuito que asome por el horizonte.
    """
    mask = mascara_rojo(img_bgr[filas])  # solo len(filas)×640 píxeles
    xs = np.full(len(filas), np.nan)
    esperado = CX
    for i in range(len(filas)):
        c = _tramos(mask[i])
        if c.size:
            xs[i] = c[np.argmin(np.abs(c - esperado))]
            esperado = xs[i]
    return xs


def banda_horizonte(img_bgr, xs, fila, n):
    """Columna de la línea en cada fila de la banda [fila, fila + n).

    Se sigue la línea de abajo arriba desde el último punto de las scanlines que
    queda por debajo de la banda, como en centros_filas, y se descarta un tramo si
    salta más de SALTO_HOR px respecto a la fila anterior: así no se coge otro trozo
    del circuito que asome por el horizonte. Devuelve (filas, xs_banda).
    """
    filas = np.arange(int(fila), min(int(fila) + int(n), ALTO))
    xb = np.full(len(filas), np.nan)
    if filas.size == 0:
        return filas, xb
    debajo = np.isfinite(xs) & (FILAS >= filas[-1])
    if not debajo.any():
        return filas, xb
    esperado = xs[debajo][-1]  # FILAS va de cerca a lejos: el último es el más alto
    mask = mascara_rojo(img_bgr[filas])
    for i in range(len(filas) - 1, -1, -1):
        c = _tramos(mask[i])
        if c.size:
            j = np.argmin(np.abs(c - esperado))
            if abs(c[j] - esperado) <= SALTO_HOR:
                xb[i] = esperado = c[j]
    return filas, xb


def _ajuste(z, y, w):
    """Parábola Y = c0 + c1·Z + c2·Z² → (c0, c1, c2). Da posición y rumbo."""
    c2, c1, c0 = np.polyfit(z, y, 2, w=w)
    return c0, c1, c2


def _curvatura(z, y, w, signo):
    """Curvatura por ajuste algebraico de círculo A(Z²+Y²) + D·Z + E·Y + G = 0.

    Una parábola sobrestima κ cuando los puntos llegan lejos respecto al radio;
    el círculo no. Con A ≈ 0 (recta) sale κ ≈ 0 sin dividir por cero. El signo
    (+ izquierda) lo da la parábola, que no se equivoca en eso.
    """
    s = max(np.abs(z).max(), 1e-6)  # normalizar para que el SVD esté bien condicionado
    zn, yn = z / s, y / s
    M = np.column_stack([zn * zn + yn * yn, zn, yn, np.ones_like(zn)]) * w[:, None]
    A, D, E, G = np.linalg.svd(M)[2][-1]
    disc = D * D + E * E - 4 * A * G
    if disc <= 0:
        return 0.0
    return float(np.sign(signo) * abs(2 * A) / np.sqrt(disc) / s)


def percepcion(img_bgr, fila_hor=240, n_filas_hor=10):
    """Todo lo que el control necesita de un fotograma."""
    xs = centros_filas(img_bgr)
    ok = np.isfinite(xs)
    filas_hor, xs_hor = banda_horizonte(img_bgr, xs, fila_hor, n_filas_hor)
    ok_hor = np.isfinite(xs_hor)
    p = {"xs": xs, "validos": int(ok.sum()), "e_px": None, "y0": None, "rumbo": None,
         "kappa": 0.0, "kappa_lejos": None, "recta": True, "z_max": 0.0,
         "filas_hor": filas_hor, "xs_hor": xs_hor, "n_hor": int(ok_hor.sum()),
         "e_hor": float(xs_hor[ok_hor].mean() - CX) if ok_hor.any() else None}
    if not ok.any():
        return p

    cerca = ok & FILAS_CERCA
    x_cerca = xs[cerca].mean() if cerca.any() else xs[ok][0]
    p["e_px"] = float(x_cerca - CX)

    z, y = imagen_a_suelo(xs[ok], FILAS[ok])
    p["z_max"] = float(z.max())
    w = FILAS[ok] - CY  # error de Y ∝ 1/(y − CY): pesar más las filas cercanas
    if ok.sum() >= 3:
        c0, c1, c2 = _ajuste(z, y, w)
        p.update(y0=float(c0), rumbo=float(np.arctan(c1)), kappa=_curvatura(z, y, w, c2),
                 coef=(float(c0), float(c1), float(c2)))
        lejos = z >= Z_LEJOS
        if lejos.sum() >= 3:
            c2l = _ajuste(z[lejos], y[lejos], w[lejos])[2]
            p["kappa_lejos"] = _curvatura(z[lejos], y[lejos], w[lejos], c2l)
    p["recta"] = abs(p["kappa"]) < K_RECTA
    return p


# --- Control ----------------------------------------------------------------
def control_p(cx, kp=KP):
    """w > 0 gira a la izquierda (ROS). Línea a la derecha → error > 0 → w < 0."""
    return -kp * (cx - CX)


class PID:
    """PID discreto del T03: filtro en D, ponderación de referencia (b, c),
    anti-windup por integración condicional y prealimentación.

        I  = I + qi·e                       qi = Kp·T/Ti
        ef = a·ef1 + (1−a)·(c·r − y)        a  = τd/(T+τd),  τd = Td/N
        D  = qd·(ef − ef1)                  qd = Kp·Td/T
        u  = Kp·(b·r − y) + I + D + uFF,    saturada a [−u_max, u_max]

    T (dt) se pasa en cada paso, porque se mide. Ti = 0 o Td = 0 anulan I o D.
    Aquí y es la posición de la línea en píxeles (e_px) y u es w, así que con
    r = 0 se reproduce control_p: w = −Kp·e_px.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.i = 0.0
        self.ef1 = None

    def paso(self, r, y, dt, kp, ti=0.0, td=0.0, n=10.0, u_max=1e9, uff=0.0, b=1.0, c=0.0):
        e = r - y
        qi = kp * dt / ti if ti > 0 else 0.0
        efn = c * r - y
        if td > 0 and dt > 0:
            tau = td / max(n, 1e-6)
            a = tau / (dt + tau)
            ef = efn if self.ef1 is None else a * self.ef1 + (1 - a) * efn
            d = 0.0 if self.ef1 is None else kp * td / dt * (ef - self.ef1)
        else:
            ef, d = efn, 0.0
        if ti <= 0:
            self.i = 0.0
        self.i += qi * e
        p = kp * (b * r - y)
        u = p + self.i + d + uff
        if (u < -u_max and qi * e < 0) or (u > u_max and qi * e > 0):
            self.i -= qi * e  # anti-windup: no integrar hacia la saturación
            u = p + self.i + d + uff
        self.ef1 = ef
        return min(max(u, -u_max), u_max), {"p": p, "i": self.i, "d": d, "ff": uff}


def v_referencia(kappa_lejos, kappa, prm):
    """Perfil de velocidad: frena según la curvatura que se ve por delante."""
    k = kappa_lejos if kappa_lejos is not None else kappa
    v = prm["v_max"] / (1.0 + prm["k_v_curva"] * abs(k))
    return min(max(v, min(prm["v_min"], prm["v_max"])), prm["v_max"])


def rampa(v_ant, v_obj, dt, a_sub, a_fren):
    """Limita la aceleración (a_sub) y la frenada (a_fren). 0 = sin límite."""
    dv = v_obj - v_ant
    if dv > 0 and a_sub > 0:
        dv = min(dv, a_sub * dt)
    elif dv < 0 and a_fren > 0:
        dv = max(dv, -a_fren * dt)
    return v_ant + dv


def error_mirada(p, v, t_mirar, z_min):
    """Error en píxeles medido a L = v·t_mirar m por delante (pure pursuit).

    Es la columna (respecto al centro) donde se ve la línea a esa distancia,
    sacada de la parábola Y(Z): e = −F·Y(L)/L. Usa las mismas unidades que e_px,
    así que Kp sirve igual. A diferencia del error de las filas cercanas (casi un
    doble integrador desde w), este incluye el rumbo: e ≈ (Y + L·ψ)·F/L, lo que
    añade un cero en s = −v/L = −1/t_mirar, es decir, adelanto de fase sin derivar.
    L se recorta a [z_min, z_max visible]. Devuelve (e, L), o (None, 0) sin ajuste.
    """
    if t_mirar <= 0 or p.get("coef") is None:
        return None, 0.0
    L = min(max(v * t_mirar, z_min), max(p["z_max"], 0.2))
    c0, c1, c2 = p["coef"]
    return -F * (c0 + c1 * L + c2 * L * L) / L, L


class Controlador:
    """Un paso de control por fotograma nuevo. Sin dependencias del simulador."""

    def __init__(self, prm=None):
        self.prm = prm or params_por_defecto()
        self.pid = PID()
        self.v = 0.0
        self.w = 0.0

    def paso(self, p, dt):
        prm = self.prm
        if p["e_px"] is None:  # línea perdida: despacio y con el último giro
            self.v = prm["v_perdida"]
            self.pid.reset()
            return self.v, self.w, {"v_ref": self.v, "p": 0.0, "i": 0.0, "d": 0.0, "ff": 0.0, "hor": 0.0,
                                    "e_ctl": None, "L": 0.0}
        v_ref = v_referencia(p["kappa_lejos"], p["kappa"], prm)
        self.v = rampa(self.v if self.v > 0 else v_ref, v_ref, dt, prm["a_sub"], prm["a_fren"])
        kp = prm["kp"] / (1.0 + prm["kv_kp"] * self.v)
        uff = prm["k_ff"] * self.v * p["kappa"]
        e_hor = p.get("e_hor")
        u_hor = 0.0 if e_hor is None else -prm["k_hor"] * e_hor
        e, L = error_mirada(p, self.v, prm["t_mirar"], prm["z_mirar_min"])
        if e is None:
            e = p["e_px"]
        self.w, partes = self.pid.paso(0.0, e, dt, kp, prm["ti"], prm["td"],
                                       prm["n_filtro"], prm["w_max"], uff + u_hor)
        return self.v, self.w, dict(partes, ff=uff, hor=u_hor, v_ref=v_ref, e_ctl=e, L=L)

    def reiniciar(self):
        self.pid.reset()
        self.v = self.w = 0.0


class Vigilante:
    """Detecta que el coche se ha salido (sin línea) o se ha atascado (choque).

    Atasco: la velocidad real (odometría) es < 40 % de la mandada durante t_atasco,
    algo que en este coche cinemático solo pasa si hay contacto con la pared.
    """

    def __init__(self):
        self.desde_perdida = None
        self.desde_atasco = None

    def comprobar(self, t, p, v_cmd, vx_real, prm):
        if prm["reinicio_auto"] < 0.5:
            return None
        if p["validos"] == 0:
            self.desde_perdida = t if self.desde_perdida is None else self.desde_perdida
            if t - self.desde_perdida > prm["t_perdida_max"]:
                return "sin_linea"
        else:
            self.desde_perdida = None
        if vx_real is not None and v_cmd > 3.0 and vx_real < 0.4 * v_cmd:
            self.desde_atasco = t if self.desde_atasco is None else self.desde_atasco
            if t - self.desde_atasco > prm["t_atasco"]:
                return "choque"
        else:
            self.desde_atasco = None
        return None

    def limpiar(self):
        self.desde_perdida = self.desde_atasco = None


# Salidas conocidas (x, y, z, yaw): si el coche arranca cerca de una, se usa exacta.
SALIDAS = {"simple": (53.462, -10.734, 0.004, -1.57)}
MUNDO_GZ = "default"


def elegir_salida(pose_inicial):
    """Pose a la que volver tras un choque: la salida conocida más cercana (< 3 m)
    o, si no hay ninguna, la pose en la que arrancó el controlador."""
    if pose_inicial is None:
        return SALIDAS["simple"]
    x, y, yaw = pose_inicial[:3]
    for s in SALIDAS.values():
        if (x - s[0]) ** 2 + (y - s[1]) ** 2 < 9.0:
            return s
    return (x, y, 0.004, yaw)


def volver_a_salida(salida):
    """Teletransporta el coche con el servicio de Gazebo. Devuelve True si funcionó."""
    import math
    import subprocess

    x, y, z, yaw = salida
    req = (f'name: "f1", position: {{x: {x}, y: {y}, z: {z}}}, '
           f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}")
    try:
        r = subprocess.run(["gz", "service", "-s", f"/world/{MUNDO_GZ}/set_pose",
                            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                            "--timeout", "2000", "--req", req],
                           capture_output=True, text=True, timeout=5)
        return r.returncode == 0 and "true" in r.stdout
    except (OSError, subprocess.SubprocessError):
        return False


# --- Telemetría y ajuste en vivo (panel en tools/panel) ----------------------
class Telemetria:
    """Servidor TCP en un hilo aparte: manda una línea JSON por fotograma al panel
    y recibe cambios de parámetros. Sin panel conectado no hace nada, y nunca
    bloquea el bucle de control (si el panel va lento, se descartan datos).

    Mensajes al panel:   {"tipo": "hola", "params": [...]}, {"tipo": "dato", ...},
                         {"tipo": "params", "valores": {...}}
                         {"tipo": "reinicio", "motivo": ..., "ok": ...}
    Mensajes del panel:  {"tipo": "set", "valores": {...}}, {"tipo": "reset_pid"},
                         {"tipo": "reiniciar"} (volver a la salida)
    """

    def __init__(self, puerto, prm):
        import queue
        import socket
        import threading

        self.prm = prm  # se sustituye entero (asignación atómica) al cambiar
        self.cola = queue.Queue(maxsize=400)
        self.pedir_reset_pid = False
        self.pedir_reinicio = False
        self.conectado = False
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("0.0.0.0", puerto))
        self.srv.listen(1)
        threading.Thread(target=self._bucle, daemon=True).start()

    def enviar(self, dato):
        if self.conectado:
            try:
                self.cola.put_nowait(dato)
            except Exception:  # cola llena: el panel no da abasto, se pierde este dato
                pass

    def _hola(self):
        esquema = [dict(nombre=n, defecto=d, min=lo, max=hi, paso=st, grupo=g, desc=ds,
                        tope_duro=n in TOPE_DURO)
                   for n, d, lo, hi, st, g, ds in PARAMS]
        return {"tipo": "hola", "params": esquema, "valores": self.prm}

    def _bucle(self):
        import json
        import queue

        while True:
            cli, _ = self.srv.accept()
            cli.settimeout(0.02)
            buf = b""
            try:
                cli.sendall((json.dumps(self._hola()) + "\n").encode())
                while not self.cola.empty():
                    self.cola.get_nowait()
                self.conectado = True
                while True:
                    lote = []
                    try:
                        lote.append(self.cola.get(timeout=0.02))
                        while len(lote) < 50:
                            lote.append(self.cola.get_nowait())
                    except queue.Empty:
                        pass
                    if lote:
                        cli.sendall("".join(json.dumps(d) + "\n" for d in lote).encode())
                    try:
                        trozo = cli.recv(65536)
                        if not trozo:
                            break  # el panel cerró
                        buf += trozo
                    except OSError as ex:
                        if "timed out" not in str(ex):
                            raise
                    while b"\n" in buf:
                        linea, buf = buf.split(b"\n", 1)
                        self._mensaje(json.loads(linea), cli)
            except (OSError, ValueError):
                pass
            finally:
                self.conectado = False
                cli.close()

    def _mensaje(self, m, cli):
        import json

        if m.get("tipo") == "set":
            self.prm = ajustar_params(self.prm, m.get("valores", {}))
            cli.sendall((json.dumps({"tipo": "params", "valores": self.prm}) + "\n").encode())
        elif m.get("tipo") == "reset_pid":
            self.pedir_reset_pid = True
        elif m.get("tipo") == "reiniciar":
            self.pedir_reinicio = True


class Odometria:
    """Posición real del coche (/f1/odom) para el mapa y el cronómetro del panel.
    Si no se puede suscribir, devuelve None y el resto sigue funcionando."""

    def __init__(self):
        self.ultimo = None
        try:
            import threading

            import rclpy
            from nav_msgs.msg import Odometry
            from rclpy.executors import SingleThreadedExecutor

            if not rclpy.ok():
                rclpy.init()
            self.nodo = rclpy.create_node("sjk_odometria")
            self.nodo.create_subscription(Odometry, "/f1/odom", self._cb, 10)
            ex = SingleThreadedExecutor()
            ex.add_node(self.nodo)
            threading.Thread(target=ex.spin, daemon=True).start()
        except Exception as ex:  # sin ROS (pruebas) o fallo al suscribirse
            print("odometría no disponible:", ex)

    def _cb(self, m):
        import math

        q, p, t = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.ultimo = (p.x, p.y, yaw, t.linear.x, t.angular.z)


# --- Visualización ----------------------------------------------------------
def dibujar(img, p, v, w):
    vis = img.copy()
    f = p.get("filas_hor")
    if f is not None and f.size:  # banda del horizonte en azul transparente
        tramo = vis[f[0]:f[-1] + 1]
        azul = np.zeros_like(tramo)
        azul[:] = (255, 0, 0)
        vis[f[0]:f[-1] + 1] = cv2.addWeighted(tramo, 0.55, azul, 0.45, 0)
        for x, y in zip(p["xs_hor"], f):
            if np.isfinite(x):
                cv2.circle(vis, (int(x), int(y)), 2, (255, 255, 0), -1)
        if p["e_hor"] is not None:  # columna media: hacia dónde va la pista
            xm = int(CX + p["e_hor"])
            cv2.line(vis, (xm, f[0] - 12), (xm, f[-1] + 12), (255, 255, 0), 2)
    for x, y in zip(p["xs"], FILAS):
        if np.isfinite(x):
            cv2.circle(vis, (int(x), int(y)), 3 if y < 300 else 6, (0, 255, 0), -1)
    cv2.line(vis, (int(CX), 390), (int(CX), ALTO - 1), (255, 0, 0), 1)
    estado = "RECTA" if p["recta"] else ("CURVA IZQ" if p["kappa"] > 0 else "CURVA DER")
    kl = "-" if p["kappa_lejos"] is None else f"{p['kappa_lejos']:+.3f}"
    cv2.putText(vis, f"{estado}  k={p['kappa']:+.3f} kl={kl} 1/m", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(vis, f"v={v:.1f} w={w:+.2f}  puntos={p['validos']} zmax={p['z_max']:.1f} m", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    eh = "-" if p.get("e_hor") is None else f"{p['e_hor']:+.0f} px ({p['n_hor']} filas)"
    cv2.putText(vis, f"horizonte: {eh}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    return vis


# --- Bucle principal --------------------------------------------------------
def _ros_vivo():
    try:
        import rclpy

        return rclpy.ok()
    except ImportError:
        return True


def sello_camara(HAL):
    s = HAL.camera_node.last_img_.header.stamp
    return s.sec + s.nanosec * 1e-9


def main():
    import HAL
    import WebGUI

    log = open(LOG, "w", buffering=1) if LOG else None
    if log:
        log.write("t_sim,t_reloj,cx,e,v,w,kappa,kappa_lejos,y0,rumbo,validos,z_max,t_perc_ms,"
                  "v_ref,p,i,d,ff\n")

    ctl = Controlador()
    try:
        tel = Telemetria(PUERTO_PANEL, ctl.prm)
    except OSError as ex:  # puerto ocupado: se sigue sin panel
        print("telemetría desactivada:", ex)
        tel = None
    odom = Odometria()
    vig = Vigilante()
    salida = None
    quieto_hasta = 0.0  # tras un reinicio, unos fotogramas parado mientras llega la imagen nueva

    sello_ant = None
    n = n_espera = 0
    while True:
        st = sello_camara(HAL)
        if st == 0 or st == sello_ant:  # aún sin imagen o sin fotograma nuevo
            time.sleep(0.002)
            n_espera += 1
            if n_espera % 250 == 0 and not _ros_vivo():  # rclpy apaga ROS con SIGTERM
                return
            continue
        n_espera = 0
        dt = 0.05 if sello_ant is None else st - sello_ant
        sello_ant = st
        if tel:
            ctl.prm = tel.prm
            if tel.pedir_reset_pid:
                ctl.pid.reset()
                tel.pedir_reset_pid = False
        if salida is None and odom.ultimo is not None:
            salida = elegir_salida(odom.ultimo)

        img = HAL.getImage()
        t0 = time.perf_counter()
        p = percepcion(img, ctl.prm["fila_hor"], ctl.prm["n_filas_hor"])
        t_perc = (time.perf_counter() - t0) * 1000
        o = odom.ultimo
        motivo = vig.comprobar(st, p, ctl.v, None if o is None else o[3], ctl.prm)
        if tel and tel.pedir_reinicio:
            motivo, tel.pedir_reinicio = "pedido", False
        if motivo:
            HAL.setV(0.0)
            HAL.setW(0.0)
            ok = volver_a_salida(salida or elegir_salida(None))
            ctl.reiniciar()
            vig.limpiar()
            quieto_hasta = st + 0.3
            print(f"reinicio ({motivo}): {'ok' if ok else 'FALLÓ el servicio de Gazebo'}")
            if tel:
                tel.enviar({"tipo": "reinicio", "motivo": motivo, "ok": ok, "t": round(st, 3)})
            continue
        if st < quieto_hasta:
            HAL.setV(0.0)
            HAL.setW(0.0)
            continue
        v, w, partes = ctl.paso(p, dt)
        HAL.setV(v)
        HAL.setW(w)

        if tel and tel.conectado:
            tel.enviar({
                "tipo": "dato", "t": round(st, 3), "dt": round(dt, 4),
                "e": None if p["e_px"] is None else round(p["e_px"], 1),
                "kappa": round(p["kappa"], 4),
                "kappa_lejos": None if p["kappa_lejos"] is None else round(p["kappa_lejos"], 4),
                "y0": None if p["y0"] is None else round(p["y0"], 4),
                "rumbo": None if p["rumbo"] is None else round(p["rumbo"], 4),
                "validos": p["validos"], "z_max": round(p["z_max"], 2), "t_perc": round(t_perc, 3),
                "v": round(v, 3), "w": round(w, 4), "v_ref": round(partes["v_ref"], 3),
                "p": round(partes["p"], 4), "i": round(partes["i"], 4),
                "d": round(partes["d"], 4), "ff": round(partes["ff"], 4), "hor": round(partes["hor"], 4),
                "e_hor": None if p["e_hor"] is None else round(p["e_hor"], 1), "n_hor": p["n_hor"],
                "e_ctl": None if partes["e_ctl"] is None else round(partes["e_ctl"], 1),
                "L": round(partes["L"], 2),
                "x": None if o is None else round(o[0], 3), "y": None if o is None else round(o[1], 3),
                "yaw": None if o is None else round(o[2], 4),
                "vx": None if o is None else round(o[3], 3), "wz": None if o is None else round(o[4], 4),
            })

        if log:
            f = lambda a: "" if a is None else f"{a:.4f}"  # noqa: E731
            cx = "" if p["e_px"] is None else f"{CX + p['e_px']:.1f}"
            e = "" if p["e_px"] is None else f"{p['e_px']:.1f}"
            log.write(f"{st:.3f},{time.time():.3f},{cx},{e},{v:.3f},{w:.4f},{p['kappa']:.4f},"
                      f"{f(p['kappa_lejos'])},{f(p['y0'])},{f(p['rumbo'])},{p['validos']},"
                      f"{p['z_max']:.2f},{t_perc:.3f},{partes['v_ref']:.3f},{partes['p']:.4f},"
                      f"{partes['i']:.4f},{partes['d']:.4f},{partes['ff']:.4f}\n")

        n += 1
        if n % 2 == 0:  # la web codifica JPEG en otro hilo: basta con 10 Hz
            WebGUI.showImage(dibujar(img, p, v, w))


if __name__ == "__main__":
    main()
