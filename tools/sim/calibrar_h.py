"""Calibra la proyección de la cámara (H y fila del horizonte CY).

Uso: python3 calibrar_h.py [d1 d2 ...]   (m a la izquierda de la línea; por defecto ±0.5, ±1.0)
Pone el coche parado y paralelo a la recta de salida (yaw −π/2), desplazado d
metros a su izquierda, y toma un fotograma nuevo. La línea está entonces a
Y = −d en todas las distancias (medido: con d = 0 se ve centrada, el desfase
de 4 cm de la cámara no se nota), así que en cada fila la distancia real es
Z = F·|d| / |x − CX|. Ajusta 1/Z = (y − CY)/(F·H) por mínimos cuadrados.
"""
import math
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import Image

sys.path.insert(0, "/tmp/sjk_sim")
sys.path.insert(0, "/tmp/sjk_sim/ctrl")
from run_vuelta import SALIDA, gz, pausa  # noqa: E402
import follow_line as fl  # noqa: E402

CAM_Y = 0.0
ds = [float(a) for a in sys.argv[1:]] or [-1.0, -0.5, 0.5, 1.0]
rclpy.init()
n = rclpy.create_node("calibrar_h")
st = {}
n.create_subscription(Image, "/f1/camera/image_raw", lambda m: st.update(img=m), 5)
x0, y0, z0, yaw = SALIDA
izq = (math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2))
ratios = []
puntos = []  # (fila, 1/Z real)
for d in ds:
    pausa(True)
    gz("/world/default/set_pose", "gz.msgs.Pose",
       f'name: "f1", position: {{x: {x0 + d * izq[0]}, y: {y0 + d * izq[1]}, z: {z0}}}, '
       f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}")
    pausa(False); st.clear()
    t_fin = time.time() + 1.0  # deja pasar varios fotogramas tras recolocar
    while time.time() < t_fin or "img" not in st:
        rclpy.spin_once(n, timeout_sec=0.05)
    pausa(True)
    m = st["img"]
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)[:, :, ::-1].copy()
    xs = fl.centros_filas(img)
    y_real = -(d + CAM_Y)
    print(f"\n## d = {d:+.2f} m → Y real = {y_real:+.3f} m")
    for fila, x in zip(fl.FILAS, xs):
        if np.isnan(x):
            continue
        z, y = fl.imagen_a_suelo(x, fila)
        r = y_real / y if abs(y) > 1e-3 else float("nan")
        print(f"fila {fila}: x={x:6.1f} px  Z={float(z):5.2f} m  Y_med={float(y):+.3f} m  Y_real/Y_med={r:.3f}")
        if abs(y_real) > 0.2 and np.isfinite(r):
            ratios.append(r)
        if abs(y_real) > 0.2 and abs(x - fl.CX) > 15 and 3 < x < 636:
            puntos.append((fila, abs(x - fl.CX) / (fl.F * abs(y_real))))
if len(puntos) >= 3:
    f_, iz = np.array(puntos).T
    a, b = np.polyfit(f_, iz, 1)
    res = iz - (a * f_ + b)
    print(f"\nAjuste con {len(puntos)} puntos: H = {1 / (a * fl.F):.4f} m, CY = {-b / a:.2f} px "
          f"(modelo actual: H = {fl.H}, CY = {fl.CY}); residuo máx 1/Z = {np.abs(res).max():.4f} 1/m")
pausa(True)
