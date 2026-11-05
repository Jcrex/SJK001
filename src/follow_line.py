"""Follow Line F1 — percepción por scanlines + PID del T03 ajustable en vivo.

Fichero único: se pega tal cual en el editor del ejercicio. Las funciones de
percepción y control no dependen del simulador, así que las pruebas
(tests/) las importan sin HAL.

Ajuste en vivo: abre un servidor de telemetría en el puerto 6089. El panel
(tools/panel.sh, en el PC) se conecta, dibuja las gráficas, cambia los
parámetros de PARAMS sin reiniciar y graba todo en logs/. Sin panel, el
controlador funciona igual. Con los valores por defecto es el control P de
las fases 1–2 (v = 10 m/s, Kp = 0.008).

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

ROJO_BAJO = ((0, 100, 60), (10, 255, 255))
ROJO_ALTO = ((170, 100, 60), (180, 255, 255))

# --- Parámetros ajustables en vivo -------------------------------------------
# (nombre, defecto, mín, máx, paso, grupo, descripción). El panel (tools/panel)
# genera un slider por fila y los cambios llegan al bucle sin reiniciar.
# Un parámetro se puede fijar también con la variable de entorno SJK_<NOMBRE>.
# Con los valores por defecto el controlador es el P de las fases 1–2.
PARAMS = [
    ("v_max", 10.0, 1.0, 30.0, 0.5, "Velocidad", "m/s en recta"),
    ("k_v_curva", 0.0, 0.0, 200.0, 1.0, "Velocidad",
     "v_ref = v_max / (1 + k·|κ_lejos|). 0 = velocidad constante"),
    ("v_min", 4.0, 1.0, 20.0, 0.5, "Velocidad", "m/s mínima del perfil de velocidad"),
    ("a_sub", 0.0, 0.0, 50.0, 0.5, "Velocidad", "m/s² máx. al acelerar. 0 = sin límite"),
    ("a_fren", 0.0, 0.0, 100.0, 1.0, "Velocidad", "m/s² máx. al frenar. 0 = sin límite"),
    ("kp", 0.008, 0.0, 0.04, 0.0005, "Giro (PID)", "rad/s por píxel de error"),
    ("ti", 0.0, 0.0, 10.0, 0.05, "Giro (PID)", "s, tiempo integral. 0 = sin I"),
    ("td", 0.0, 0.0, 0.5, 0.005, "Giro (PID)", "s, tiempo derivativo. 0 = sin D"),
    ("n_filtro", 10.0, 1.0, 30.0, 1.0, "Giro (PID)", "N del filtro de D: τd = Td/N"),
    ("w_max", 6.0, 0.5, 10.0, 0.1, "Giro (PID)", "rad/s, saturación de w (anti-windup)"),
    ("kv_kp", 0.0, 0.0, 0.5, 0.005, "Giro (PID)", "gain scheduling: Kp_ef = Kp / (1 + kv·v)"),
    ("k_ff", 0.0, 0.0, 3.0, 0.05, "Prealimentación", "w_ff = k_ff · v · κ"),
    ("v_perdida", 1.0, 0.0, 10.0, 0.5, "Otros", "m/s cuando no se ve la línea"),
]
_ALIAS_ENTORNO = {"v_max": "SJK_V"}  # nombres de las fases 1–2


def params_por_defecto():
    d = {}
    for nombre, defecto, *_ in PARAMS:
        txt = os.environ.get(_ALIAS_ENTORNO.get(nombre, ""), os.environ.get("SJK_" + nombre.upper()))
        d[nombre] = float(txt) if txt else defecto
    return d


def ajustar_params(actual, cambios):
    """Aplica cambios {nombre: valor} recortados a su rango; ignora nombres desconocidos."""
    rangos = {p[0]: (p[2], p[3]) for p in PARAMS}
    nuevo = dict(actual)
    for k, v in cambios.items():
        if k in rangos:
            nuevo[k] = min(max(float(v), rangos[k][0]), rangos[k][1])
    return nuevo


KP = params_por_defecto()["kp"]  # compatibilidad con control_p de la fase 1
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


def percepcion(img_bgr):
    """Todo lo que el control necesita de un fotograma."""
    xs = centros_filas(img_bgr)
    ok = np.isfinite(xs)
    p = {"xs": xs, "validos": int(ok.sum()), "e_px": None, "y0": None, "rumbo": None,
         "kappa": 0.0, "kappa_lejos": None, "recta": True, "z_max": 0.0}
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
        p.update(y0=float(c0), rumbo=float(np.arctan(c1)), kappa=_curvatura(z, y, w, c2))
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
            return self.v, self.w, {"v_ref": self.v, "p": 0.0, "i": 0.0, "d": 0.0, "ff": 0.0}
        v_ref = v_referencia(p["kappa_lejos"], p["kappa"], prm)
        self.v = rampa(self.v if self.v > 0 else v_ref, v_ref, dt, prm["a_sub"], prm["a_fren"])
        kp = prm["kp"] / (1.0 + prm["kv_kp"] * self.v)
        uff = prm["k_ff"] * self.v * p["kappa"]
        self.w, partes = self.pid.paso(0.0, p["e_px"], dt, kp, prm["ti"], prm["td"],
                                       prm["n_filtro"], prm["w_max"], uff)
        return self.v, self.w, dict(partes, v_ref=v_ref)


# --- Telemetría y ajuste en vivo (panel en tools/panel) ----------------------
class Telemetria:
    """Servidor TCP en un hilo aparte: manda una línea JSON por fotograma al panel
    y recibe cambios de parámetros. Sin panel conectado no hace nada, y nunca
    bloquea el bucle de control (si el panel va lento, se descartan datos).

    Mensajes al panel:   {"tipo": "hola", "params": [...]}, {"tipo": "dato", ...},
                         {"tipo": "params", "valores": {...}}
    Mensajes del panel:  {"tipo": "set", "valores": {...}}, {"tipo": "reset_pid"}
    """

    def __init__(self, puerto, prm):
        import queue
        import socket
        import threading

        self.prm = prm  # se sustituye entero (asignación atómica) al cambiar
        self.cola = queue.Queue(maxsize=400)
        self.pedir_reset_pid = False
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
        esquema = [dict(nombre=n, defecto=d, min=lo, max=hi, paso=st, grupo=g, desc=ds)
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

        img = HAL.getImage()
        t0 = time.perf_counter()
        p = percepcion(img)
        t_perc = (time.perf_counter() - t0) * 1000
        v, w, partes = ctl.paso(p, dt)
        HAL.setV(v)
        HAL.setW(w)

        if tel and tel.conectado:
            o = odom.ultimo
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
                "d": round(partes["d"], 4), "ff": round(partes["ff"], 4),
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
