"""Fase 0 — comprobación del entorno (pegar en el editor del ejercicio Follow Line).

Con el coche parado:
- Mide a qué frecuencia llegan fotogramas nuevos (sello de tiempo de la cámara)
  y a qué frecuencia gira el bucle.
- Mide cuánto cuestan HAL.getImage() y la máscara del rojo.
- Mide el factor de tiempo real (tiempo simulado / tiempo de reloj). El
  cronómetro de vuelta usa la hora de reloj, así que importa.
- Muestra la máscara del rojo con las filas de scanline y su centro.
Cada ~2 s imprime un resumen en la consola del ejercicio.

API real (leída de /workspace/code en el contenedor):
- HAL.getImage() devuelve un array BGR 480×640×3. Convierte el mensaje ROS
  en cada llamada.
- HAL.setV(v), HAL.setW(w): Twist en /f1/cmd_vel, sin límites.
- WebGUI.showImage(img). El módulo se llama WebGUI, no GUI.
- HAL.camera_node.last_img_.header.stamp: sello del último fotograma.
"""
import time

import cv2
import numpy as np
import HAL
import WebGUI

ROJO_BAJO = ((0, 100, 60), (10, 255, 255))
ROJO_ALTO = ((170, 100, 60), (180, 255, 255))
FILAS_SCAN = [470, 440, 400, 350, 300, 260]


def mascara_rojo(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    return cv2.bitwise_or(cv2.inRange(hsv, *ROJO_BAJO), cv2.inRange(hsv, *ROJO_ALTO))


def sello():
    s = HAL.camera_node.last_img_.header.stamp
    return s.sec + s.nanosec * 1e-9


HAL.setV(0)
HAL.setW(0)

t_resumen = time.time()
sim_resumen = sello()
iteraciones = nuevos = 0
sello_ant = None
t_get, t_mask = [], []

while True:
    t0 = time.perf_counter()
    img = HAL.getImage()
    t_get.append((time.perf_counter() - t0) * 1000)

    st = sello()
    if st != sello_ant:
        nuevos += 1
        sello_ant = st
    iteraciones += 1

    t0 = time.perf_counter()
    mask = mascara_rojo(img)
    t_mask.append((time.perf_counter() - t0) * 1000)

    vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    for y in FILAS_SCAN:
        cv2.line(vis, (0, y), (vis.shape[1] - 1, y), (0, 255, 0), 1)
        xs = np.flatnonzero(mask[y])
        if xs.size:
            cv2.circle(vis, (int(xs.mean()), y), 5, (255, 0, 0), -1)
    WebGUI.showImage(vis)

    ahora = time.time()
    if ahora - t_resumen >= 2.0:
        dt = ahora - t_resumen
        rtf = (st - sim_resumen) / dt
        filas = np.flatnonzero(mask.any(axis=1))
        centros = []
        for y in FILAS_SCAN:
            xs = np.flatnonzero(mask[y])
            centros.append(f"{y}:{xs.mean():.0f}" if xs.size else f"{y}:-")
        print(
            f"shape={img.shape} | bucle={iteraciones / dt:.0f} Hz, "
            f"fotogramas nuevos={nuevos / dt:.1f} Hz, RTF={rtf:.2f} | "
            f"getImage={np.mean(t_get):.2f} ms, máscara={np.mean(t_mask):.2f} ms | "
            f"filas con rojo {filas.min() if filas.size else '-'}–{filas.max() if filas.size else '-'} | "
            f"centros {' '.join(centros)}"
        )
        t_resumen, sim_resumen = ahora, st
        iteraciones = nuevos = 0
        t_get, t_mask = [], []
