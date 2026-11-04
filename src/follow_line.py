"""Follow Line F1 — fase 2: percepción por scanlines + control P a velocidad constante.

Fichero único: se pega tal cual en el editor del ejercicio. Las funciones de
percepción y control no dependen del simulador, así que las pruebas
(tests/) las importan sin HAL.

Geometría (docs/PLAN.md §5.1): cámara pinhole de 640×480, FOV horizontal de 90°,
cabeceo 0 y a H ≈ 0.104 m del suelo. Un píxel (x, y) por debajo del horizonte
corresponde a un punto del suelo a Z m por delante y Y m a la izquierda:
    Z = F·H / (y − CY)        Y = −H·(x − CX) / (y − CY)
"""
import os
import time

import cv2
import numpy as np

# --- Cámara -----------------------------------------------------------------
ANCHO, ALTO = 640, 480
CX, CY = 320.0, 239.5  # punto principal; el horizonte cae entre las filas 239 y 240
F = 320.0  # px: (640/2) / tan(90°/2)
H = 0.104  # m: altura de la cámara (0.1 sobre la base + 0.004 de la salida)
CX_IMG = CX  # compatibilidad con la fase 1

# Filas de scanline, de cerca a lejos. Entre paréntesis, la distancia por delante (Z):
# 470 (0.14 m) 400 (0.21) 340 (0.33) 300 (0.55) 280 (0.82) 268 (1.17) 260 (1.62)
# 255 (2.15) 251 (2.9) 248 (3.9) 246 (5.1) 245 (6.0) 244 (7.3) 243 (9.5) 242 (13.3)
FILAS = np.array([470, 400, 340, 300, 280, 268, 260, 255, 251, 248, 246, 245, 244, 243, 242])
FILAS_CERCA = FILAS >= 390  # filas que dan el error de posición actual
Z_LEJOS = 1.5  # m: a partir de aquí los puntos cuentan para κ_lejos
K_RECTA = 0.02  # 1/m: |κ| por debajo → recta (radio > 50 m)

ROJO_BAJO = ((0, 100, 60), (10, 255, 255))
ROJO_ALTO = ((170, 100, 60), (180, 255, 255))

# --- Control ----------------------------------------------------------------
V = float(os.environ.get("SJK_V", 10.0))  # m/s, constante en las fases 1–2
KP = float(os.environ.get("SJK_KP", 0.008))  # rad/s por píxel de error
V_PERDIDA = 1.0  # m/s cuando no se ve la línea
LOG = os.environ.get("SJK_LOG")  # ruta del CSV; sin definir no se guarda nada


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
def sello_camara(HAL):
    s = HAL.camera_node.last_img_.header.stamp
    return s.sec + s.nanosec * 1e-9


def main():
    import HAL
    import WebGUI

    log = open(LOG, "w", buffering=1) if LOG else None
    if log:
        log.write("t_sim,t_reloj,cx,e,v,w,kappa,kappa_lejos,y0,rumbo,validos,z_max,t_perc_ms\n")

    sello_ant = None
    w = 0.0
    n = 0
    while True:
        st = sello_camara(HAL)
        if st == 0 or st == sello_ant:  # aún sin imagen o sin fotograma nuevo
            time.sleep(0.002)
            continue
        sello_ant = st

        img = HAL.getImage()
        t0 = time.perf_counter()
        p = percepcion(img)
        t_perc = (time.perf_counter() - t0) * 1000
        if p["e_px"] is None:
            v = V_PERDIDA  # mantener el último giro para recuperar la línea
        else:
            v = V
            w = control_p(CX + p["e_px"])
        HAL.setV(v)
        HAL.setW(w)

        if log:
            f = lambda a: "" if a is None else f"{a:.4f}"  # noqa: E731
            cx = "" if p["e_px"] is None else f"{CX + p['e_px']:.1f}"
            e = "" if p["e_px"] is None else f"{p['e_px']:.1f}"
            log.write(f"{st:.3f},{time.time():.3f},{cx},{e},{v:.3f},{w:.4f},{p['kappa']:.4f},"
                      f"{f(p['kappa_lejos'])},{f(p['y0'])},{f(p['rumbo'])},{p['validos']},"
                      f"{p['z_max']:.2f},{t_perc:.3f}\n")

        n += 1
        if n % 2 == 0:  # la web codifica JPEG en otro hilo: basta con 10 Hz
            WebGUI.showImage(dibujar(img, p, v, w))


if __name__ == "__main__":
    main()
