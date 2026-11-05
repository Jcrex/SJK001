"""Prueba de giro desde una pose arbitraria, lejos de la salida.

Uso: python3 prueba_giro.py x y yaw v w segundos [v w segundos ...]
Coloca el coche en (x, y, yaw), manda cada (v, w) durante su tiempo y, por
cada tramo, imprime el giro medido (Δyaw, wz medio) frente al mandado.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.insert(0, "/tmp/sjk_sim")
from run_vuelta import gz, pausa  # noqa: E402


def poner(x, y, yaw):
    gz("/world/default/set_pose", "gz.msgs.Pose",
       f'name: "f1", position: {{x: {x}, y: {y}, z: 0.004}}, '
       f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}")


def yaw_de(m):
    q = m.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


x, y, yaw0 = map(float, sys.argv[1:4])
tramos = [tuple(map(float, sys.argv[i:i + 3])) for i in range(4, len(sys.argv), 3)]
rclpy.init()
n = rclpy.create_node("prueba_giro")
pub = n.create_publisher(Twist, "/f1/cmd_vel", 10)
ult = {}
n.create_subscription(Odometry, "/f1/odom", lambda m: ult.update(m=m), 50)
pausa(True); poner(x, y, yaw0); pub.publish(Twist()); pausa(False)
t_fin = time.time() + 0.3
while time.time() < t_fin:
    rclpy.spin_once(n, timeout_sec=0.02)
for v, w, dur in tramos:
    cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
    m0 = ult["m"]; s0 = m0.header.stamp.sec + m0.header.stamp.nanosec * 1e-9
    acum, prev = 0.0, yaw_de(m0)
    t0 = time.time()
    while time.time() - t0 < dur:
        pub.publish(cmd); rclpy.spin_once(n, timeout_sec=0.02)
        yy = yaw_de(ult["m"]); acum += math.remainder(yy - prev, 2 * math.pi); prev = yy
    m1 = ult["m"]; s1 = m1.header.stamp.sec + m1.header.stamp.nanosec * 1e-9
    p = m1.pose.pose.position
    print(f"v={v:5.1f} w={w:+.2f}  Δyaw={acum:+.3f} rad en {s1 - s0:.2f} s sim  "
          f"wz_medio={acum / max(s1 - s0, 1e-6):+.3f} ({100 * acum / max(s1 - s0, 1e-6) / w if w else 0:5.0f} %)  "
          f"fin x={p.x:6.2f} y={p.y:6.2f} yaw={yaw_de(m1):+.3f}", flush=True)
pub.publish(Twist()); pausa(True)
