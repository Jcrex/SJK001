"""HAL mínimo de respaldo para el banco de pruebas, por si la web aún no ha
copiado el HAL real a /workspace/code. Imita lo que usa follow_line.py:
camera_node.last_img_ (con su header.stamp), getImage() en BGR, setV() y setW().
"""
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image

if not rclpy.ok():
    rclpy.init()


class _Camara:
    def __init__(self, nodo):
        self.last_img_ = Image()  # stamp = 0 hasta el primer fotograma
        nodo.create_subscription(Image, "/f1/camera/image_raw", self._cb, 2)

    def _cb(self, m):
        self.last_img_ = m


_nodo = rclpy.create_node("hal_respaldo")
camera_node = _Camara(_nodo)
_pub = _nodo.create_publisher(Twist, "/f1/cmd_vel", 10)
_cmd = Twist()
_ex = SingleThreadedExecutor()
_ex.add_node(_nodo)
threading.Thread(target=_ex.spin, daemon=True).start()


def getImage():
    m = camera_node.last_img_
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
    return img[:, :, ::-1].copy()  # rgb8 → BGR, como el HAL real


def setV(v):
    _cmd.linear.x = float(v)
    _pub.publish(_cmd)


def setW(w):
    _cmd.angular.z = float(w)
    _pub.publish(_cmd)
