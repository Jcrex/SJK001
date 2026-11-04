"""Pruebas de src/follow_line.py con imágenes sintéticas (sin simulador)."""
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import follow_line as fl  # noqa: E402

ROJO_BGR = (0, 0, 216)  # color medido en el simulador


def imagen_con_linea(x_abajo, x_horizonte=320, alto=480, ancho=640):
    """Pista vacía con una línea roja triangular, como la del simulador."""
    img = np.full((alto, ancho, 3), 50, np.uint8)
    pts = np.array([[x_abajo - 120, alto - 1], [x_abajo + 120, alto - 1], [x_horizonte, 239]])
    cv2.fillPoly(img, [pts], ROJO_BGR)
    return img


def test_mascara_detecta_rojo_y_no_el_gris():
    mask = fl.mascara_rojo(imagen_con_linea(320))
    assert mask[470, 320] == 255
    assert mask[470, 10] == 0
    assert mask[100, 320] == 0  # cielo


def test_centro_linea_centrada():
    cx = fl.centro_linea(imagen_con_linea(320))
    assert cx == pytest.approx(320, abs=3)


def test_centro_linea_desplazada_a_la_derecha():
    cx = fl.centro_linea(imagen_con_linea(450))
    assert cx > 340


def test_centro_linea_sin_rojo_devuelve_none():
    assert fl.centro_linea(np.full((480, 640, 3), 50, np.uint8)) is None


def test_control_p_gira_a_la_derecha_si_la_linea_esta_a_la_derecha():
    # ROS: w > 0 gira a la izquierda; línea a la derecha → w < 0
    assert fl.control_p(cx=450, kp=0.01) < 0
    assert fl.control_p(cx=200, kp=0.01) > 0
    assert fl.control_p(cx=320, kp=0.01) == pytest.approx(0)
