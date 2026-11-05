"""Pone el coche parado en (x, y, yaw), toma un fotograma y lo guarda en .npy (BGR).

Uso: python3 foto_pose.py x y yaw salida.npy
"""
import math
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import Image

sys.path.insert(0, "/tmp/sjk_sim")
from run_vuelta import gz, pausa  # noqa: E402

x, y, yaw = map(float, sys.argv[1:4])
rclpy.init()
n = rclpy.create_node("foto_pose")
st = {}
n.create_subscription(Image, "/f1/camera/image_raw", lambda m: st.update(img=m), 5)
pausa(True)
gz("/world/default/set_pose", "gz.msgs.Pose",
   f'name: "f1", position: {{x: {x}, y: {y}, z: 0.004}}, '
   f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}")
pausa(False); st.clear()
t_fin = time.time() + 1.0
while time.time() < t_fin or "img" not in st:
    rclpy.spin_once(n, timeout_sec=0.05)
pausa(True)
m = st["img"]
np.save(sys.argv[4], np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)[:, :, ::-1].copy())
print("guardado", sys.argv[4])
