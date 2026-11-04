"""¿Hace el coche lo que se le manda? Envía (v, w) fijos y mide con /f1/odom.
Uso (en el contenedor): python3 prueba_cinematica.py v w segundos"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.insert(0, "/tmp/sjk_sim")
from run_vuelta import a_la_salida, pausa  # noqa: E402

v, w, dur = map(float, sys.argv[1:4])
rclpy.init()
n = rclpy.create_node("prueba_cinematica")
pub = n.create_publisher(Twist, "/f1/cmd_vel", 10)
muestras = []
n.create_subscription(Odometry, "/f1/odom", lambda m: muestras.append(m), 50)
pausa(True); a_la_salida(); pausa(False)
cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
t0 = time.time()
while time.time() - t0 < dur:
    pub.publish(cmd); rclpy.spin_once(n, timeout_sec=0.02)
pub.publish(Twist()); pausa(True); a_la_salida()

def yaw(m):
    q = m.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

m = muestras[len(muestras) // 3:]  # quitar el arranque
ts = [x.header.stamp.sec + x.header.stamp.nanosec * 1e-9 for x in m]
yaws = [yaw(x) for x in m]
giro = sum(math.remainder(b - a, 2 * math.pi) for a, b in zip(yaws, yaws[1:]))
dist = sum(math.hypot(b.pose.pose.position.x - a.pose.pose.position.x,
                      b.pose.pose.position.y - a.pose.pose.position.y) for a, b in zip(m, m[1:]))
T = ts[-1] - ts[0]
print(f"mandado: v={v} w={w} → curvatura {w / v if v else float('nan'):.3f} 1/m")
print(f"medido : v={dist / T:.3f} w={giro / T:.3f} → curvatura {giro / dist if dist else float('nan'):.3f} 1/m"
      f"  (twist odom: vx={m[-1].twist.twist.linear.x:.3f} wz={m[-1].twist.twist.angular.z:.3f})")
