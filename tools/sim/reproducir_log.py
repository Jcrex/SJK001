"""Reproduce en bucle abierto los (v, w) de un log de vuelta y compara el giro.

Uso: python3 reproducir_log.py LOG.csv [t_max_s [v w segundos ...]]
Pone el coche en la salida y manda cada (v, w) del log en el mismo instante
de tiempo simulado (relativo al primer comando). Imprime cada 0.2 s la w
mandada frente a la wz de la odometría, y el yaw. Los (v, w, segundos) extra
se mandan después de t_max, uno tras otro.
"""
import csv
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

sys.path.insert(0, "/tmp/sjk_sim")
from run_vuelta import a_la_salida, pausa  # noqa: E402

filas = list(csv.DictReader(open(sys.argv[1])))
t_max = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
t0_log = float(filas[0]["t_sim"])
cmds = [(float(r["t_sim"]) - t0_log, float(r["v"]), float(r["w"])) for r in filas]
cmds = [c for c in cmds if c[0] <= t_max]
extra = [tuple(map(float, sys.argv[k:k + 3])) for k in range(3, len(sys.argv), 3)]
t_c = cmds[-1][0] if cmds else 0.0
for v, w, dur in extra:
    for k in range(int(round(dur / 0.05))):
        t_c += 0.05; cmds.append((t_c, v, w))

rclpy.init()
n = rclpy.create_node("reproducir_log")
pub = n.create_publisher(Twist, "/f1/cmd_vel", 10)
st = {}
n.create_subscription(Odometry, "/f1/odom", lambda m: st.update(odom=m), 50)
n.create_subscription(Clock, "/clock", lambda m: st.update(t=m.clock.sec + m.clock.nanosec * 1e-9), 50)
pausa(True); a_la_salida(); pub.publish(Twist()); pausa(False)
while "t" not in st or "odom" not in st:
    rclpy.spin_once(n, timeout_sec=0.05)
t0 = st["t"]
i, t_imp = 0, -1.0
while i < len(cmds):
    rclpy.spin_once(n, timeout_sec=0.005)
    t = st["t"] - t0
    while i < len(cmds) and cmds[i][0] <= t:
        c = Twist(); c.linear.x = cmds[i][1]; c.angular.z = cmds[i][2]; pub.publish(c); i += 1
    if t - t_imp >= 0.2 and i:
        m = st["odom"]; q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = m.pose.pose.position
        print(f"t={t:5.2f} w_cmd={cmds[i-1][2]:+.3f} wz={m.twist.twist.angular.z:+.3f} "
              f"yaw={yaw:+.4f} x={p.x:6.2f} y={p.y:6.2f}", flush=True)
        t_imp = t
pub.publish(Twist()); pausa(True); a_la_salida()
