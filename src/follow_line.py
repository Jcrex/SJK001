"""Follow Line F1 — fase 1: línea base con control P a velocidad constante.

Fichero único: se pega tal cual en el editor del ejercicio. Las funciones de
percepción y control no dependen del simulador, así que las pruebas
(tests/test_follow_line.py) las importan sin HAL.
"""
import os
import time

import cv2
import numpy as np

# --- Parámetros -------------------------------------------------------------
ANCHO = 640
CX_IMG = ANCHO / 2
FRANJA = (250, 480)  # filas usadas para el centroide (por debajo del horizonte, fila 239)
ROJO_BAJO = ((0, 100, 60), (10, 255, 255))
ROJO_ALTO = ((170, 100, 60), (180, 255, 255))

V = float(os.environ.get("SJK_V", 10.0))  # m/s, constante en la fase 1
KP = float(os.environ.get("SJK_KP", 0.008))  # rad/s por píxel de error
V_PERDIDA = 1.0  # m/s cuando no se ve la línea
LOG = os.environ.get("SJK_LOG")  # ruta del CSV; sin definir no se guarda nada


# --- Percepción -------------------------------------------------------------
def mascara_rojo(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    return cv2.bitwise_or(cv2.inRange(hsv, *ROJO_BAJO), cv2.inRange(hsv, *ROJO_ALTO))


def centro_linea(img_bgr, franja=FRANJA):
    """Columna del centroide del rojo en la franja, o None si no hay rojo."""
    mask = mascara_rojo(img_bgr[franja[0]:franja[1]])
    m = cv2.moments(mask, binaryImage=True)
    if m["m00"] == 0:
        return None
    return m["m10"] / m["m00"]


# --- Control ----------------------------------------------------------------
def control_p(cx, kp=KP):
    """w > 0 gira a la izquierda (ROS). Línea a la derecha → error > 0 → w < 0."""
    return -kp * (cx - CX_IMG)


# --- Bucle principal --------------------------------------------------------
def sello_camara(HAL):
    s = HAL.camera_node.last_img_.header.stamp
    return s.sec + s.nanosec * 1e-9


def main():
    import HAL
    import WebGUI

    log = open(LOG, "w", buffering=1) if LOG else None
    if log:
        log.write("t_sim,t_reloj,cx,e,v,w\n")

    sello_ant = None
    w = 0.0
    while True:
        st = sello_camara(HAL)
        if st == 0 or st == sello_ant:  # aún sin imagen o sin fotograma nuevo
            time.sleep(0.002)
            continue
        sello_ant = st

        img = HAL.getImage()
        cx = centro_linea(img)
        if cx is None:
            v = V_PERDIDA  # mantener el último giro para recuperar la línea
        else:
            v = V
            w = control_p(cx)
        HAL.setV(v)
        HAL.setW(w)

        if log:
            e = "" if cx is None else f"{cx - CX_IMG:.1f}"
            log.write(f"{st:.3f},{time.time():.3f},{'' if cx is None else f'{cx:.1f}'},{e},{v:.3f},{w:.4f}\n")

        vis = img.copy()
        if cx is not None:
            cv2.circle(vis, (int(cx), (FRANJA[0] + FRANJA[1]) // 2), 8, (0, 255, 0), -1)
        cv2.line(vis, (int(CX_IMG), FRANJA[0]), (int(CX_IMG), FRANJA[1]), (255, 0, 0), 1)
        WebGUI.showImage(vis)


if __name__ == "__main__":
    main()
