"""Banco de pruebas: cronometra una vuelta de un controlador (se ejecuta DENTRO del contenedor).

Uso: python3 run_vuelta.py CONTROLADOR.py LOG.csv [timeout_s]

1. Pone el coche parado en la salida y quita la pausa.
2. Lanza el controlador con el HAL real del ejercicio (/workspace/code) y un
   WebGUI vacío (para no interferir con la web).
3. Con /f1/odom detecta la vuelta: se ha alejado más de 30 m del punto de
   salida y vuelve a pasar a menos de 2.5 m.
4. Falla si la línea se pierde más de 3 s seguidos o si se agota el tiempo.
5. Para el coche, lo devuelve a la salida y pausa la simulación.
Imprime una línea final "RESULTADO {json}".
"""
import ctypes
import json
import math
import os
import subprocess
import sys
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SALIDA = (53.462, -10.734, 0.004, -1.57)  # x, y, z, yaw del launcher del circuito simple
STUB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stub")


def _morir_con_el_padre():
    """El controlador recibe SIGTERM si el banco muere (evita controladores huérfanos)."""
    ctypes.CDLL("libc.so.6").prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG


def gz(servicio, reqtype, req):
    subprocess.run(
        ["gz", "service", "-s", servicio, "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
         "--timeout", "3000", "--req", req],
        check=True, capture_output=True,
    )


def pausa(p):
    gz("/world/default/control", "gz.msgs.WorldControl", f"pause: {'true' if p else 'false'}")


def a_la_salida():
    x, y, z, yaw = SALIDA
    gz("/world/default/set_pose", "gz.msgs.Pose",
       f'name: "f1", position: {{x: {x}, y: {y}, z: {z}}}, '
       f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}")


def perdidas_seguidas(log_path):
    """Fotogramas seguidos al final del log sin línea (cx vacío)."""
    try:
        with open(log_path) as f:
            lineas = f.readlines()[-80:]
    except FileNotFoundError:
        return 0
    n = 0
    for ln in reversed(lineas[1:] if lineas and lineas[0].startswith("t_sim") else lineas):
        if ln.split(",")[2] == "":
            n += 1
        else:
            break
    return n


def main():
    controlador, log_path = sys.argv[1], sys.argv[2]
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0

    rclpy.init()
    nodo = rclpy.create_node("banco_vuelta")
    pub = nodo.create_publisher(Twist, "/f1/cmd_vel", 10)
    odo = {"msg": None, "grabar": False}
    log_odo = open(log_path.replace(".csv", "_odom.csv"), "w", buffering=1)
    log_odo.write("t_sim,x,y,yaw,vx,wz\n")

    def al_recibir_odom(m):
        odo["msg"] = m
        if odo["grabar"]:  # trayectoria real, para validar la percepción
            q, p, tw = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            log_odo.write(f"{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.3f},"
                          f"{p.x:.4f},{p.y:.4f},{yaw:.5f},{tw.linear.x:.3f},{tw.angular.z:.4f}\n")

    nodo.create_subscription(Odometry, "/f1/odom", al_recibir_odom, 50)

    def parar():
        for _ in range(5):
            pub.publish(Twist())
            rclpy.spin_once(nodo, timeout_sec=0.02)

    pausa(True)
    parar()
    a_la_salida()
    pausa(False)
    time.sleep(0.5)
    parar()
    odo["grabar"] = True

    entorno = dict(os.environ, SJK_LOG=log_path,
                   PYTHONPATH=f"{STUB_DIR}:/workspace/code:" + os.environ.get("PYTHONPATH", ""))
    proc = subprocess.Popen([sys.executable, controlador], env=entorno,
                            stdout=subprocess.DEVNULL, stderr=open(log_path + ".err", "w"),
                            preexec_fn=_morir_con_el_padre)

    inicio = None  # (t_sim, t_reloj, x, y)
    recorrido, ultimo = 0.0, None
    lejos = False
    resultado = {"ok": False, "motivo": "timeout"}
    t_limite = time.time() + timeout
    while time.time() < t_limite:
        rclpy.spin_once(nodo, timeout_sec=0.05)
        if proc.poll() is not None:
            resultado = {"ok": False, "motivo": f"el controlador terminó (código {proc.returncode})"}
            break
        m = odo["msg"]
        if m is None:
            continue
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if inicio is None:
            if math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) > 0.05:
                inicio = (ts, time.time(), x, y)
                ultimo = (x, y)
            continue
        recorrido += math.hypot(x - ultimo[0], y - ultimo[1])
        ultimo = (x, y)
        d0 = math.hypot(x - inicio[2], y - inicio[3])
        if d0 > 30:
            lejos = True
        if lejos and d0 < 2.5:
            resultado = {"ok": True, "t_sim": round(ts - inicio[0], 2),
                         "t_reloj": round(time.time() - inicio[1], 2),
                         "distancia_m": round(recorrido, 1)}
            break
        if perdidas_seguidas(log_path) >= 60:
            resultado = {"ok": False, "motivo": "línea perdida > 3 s",
                         "t_sim": round(ts - inicio[0], 2), "distancia_m": round(recorrido, 1)}
            break

    odo["grabar"] = False
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
    parar()
    pausa(True)
    a_la_salida()
    if resultado["ok"]:
        resultado["v_media"] = round(resultado["distancia_m"] / resultado["t_sim"], 2)
        resultado["rtf"] = round(resultado["t_sim"] / resultado["t_reloj"], 2)
    print("RESULTADO " + json.dumps(resultado, ensure_ascii=False), flush=True)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
