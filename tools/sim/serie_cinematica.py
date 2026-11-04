"""Serie temporal de la odometría con (v, w) fijos. Uso: python3 serie_cinematica.py v w segundos"""
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
n = rclpy.create_node("serie_cinematica")
pub = n.create_publisher(Twist, "/f1/cmd_vel", 10)
ult = {}
n.create_subscription(Odometry, "/f1/odom", lambda m: ult.update(m=m), 50)
pausa(True); a_la_salida(); pausa(False)
cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
t0 = time.time(); t_imp = 0
while time.time() - t0 < dur:
    pub.publish(cmd); rclpy.spin_once(n, timeout_sec=0.02)
    if "m" in ult and time.time() - t_imp > 0.5:
        m = ult["m"]; q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = m.pose.pose.position
        print(f"t={time.time()-t0:4.1f} x={p.x:7.2f} y={p.y:7.2f} z={p.z:6.3f} yaw={yaw:+.3f} "
              f"vx={m.twist.twist.linear.x:+.2f} vy={m.twist.twist.linear.y:+.2f} wz={m.twist.twist.angular.z:+.3f}")
        t_imp = time.time()
pub.publish(Twist()); pausa(True); a_la_salida()
