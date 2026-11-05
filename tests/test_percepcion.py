"""Pruebas de la percepción por scanlines (fase 2) con imágenes sintéticas.

Las curvas se generan proyectando un arco de radio conocido sobre el suelo con
el mismo modelo de cámara que usa follow_line (pinhole, cabeceo 0, altura H).
"""
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import follow_line as fl  # noqa: E402

ROJO_BGR = (0, 0, 216)


def pista_sintetica(radio=None, desplazamiento=0.0, rumbo=0.0, ancho_linea=0.11):
    """Imagen con la línea en el suelo.

    radio: m, > 0 curva a la izquierda, < 0 a la derecha, None recta.
    desplazamiento: m, posición lateral de la línea respecto a la cámara (+ izquierda).
    rumbo: rad, ángulo de la línea respecto al eje del coche (+ izquierda).
    """
    img = np.full((480, 640, 3), 50, np.uint8)
    zs = np.linspace(0.12, 14.0, 600)
    if radio is None:
        ys = np.zeros_like(zs)
    else:
        ys = radio - np.sign(radio) * np.sqrt(np.maximum(radio**2 - zs**2, 0))
        zs, ys = zs[np.abs(zs) < abs(radio)], ys[np.abs(zs) < abs(radio)]
    ys = ys + desplazamiento + np.tan(rumbo) * zs
    izq = fl.suelo_a_imagen(zs, ys + ancho_linea / 2)
    der = fl.suelo_a_imagen(zs, ys - ancho_linea / 2)
    poli = np.vstack([izq, der[::-1]]).round().astype(np.int32)
    cv2.fillPoly(img, [poli], ROJO_BGR)
    return img


# --- geometría --------------------------------------------------------------
def test_suelo_e_imagen_son_inversas():
    z, y = np.array([0.5, 2.0, 6.0]), np.array([0.1, -0.3, 0.0])
    px = fl.suelo_a_imagen(z, y)
    z2, y2 = fl.imagen_a_suelo(px[:, 0], px[:, 1])
    assert z2 == pytest.approx(z, rel=1e-6)
    assert y2 == pytest.approx(y, abs=1e-6)


def test_punto_de_la_izquierda_tiene_y_positiva():
    _, y = fl.imagen_a_suelo(np.array([200.0]), np.array([400.0]))
    assert y[0] > 0


# --- centros por fila -------------------------------------------------------
def test_centros_recta_centrada():
    xs = fl.centros_filas(pista_sintetica())
    assert np.all(np.isfinite(xs))
    assert xs == pytest.approx(np.full(len(fl.FILAS), fl.CX), abs=1.5)


def test_fila_sin_rojo_es_nan():
    xs = fl.centros_filas(np.full((480, 640, 3), 50, np.uint8))
    assert np.all(np.isnan(xs))


def test_con_dos_tramos_en_una_fila_sigue_el_mas_cercano():
    img = pista_sintetica()
    fila = fl.FILAS[-1]  # la más lejana
    img[fila, 20:60] = ROJO_BGR  # trozo de otra parte del circuito
    xs = fl.centros_filas(img)
    assert xs[-1] == pytest.approx(fl.CX, abs=3)


# --- ajuste y curvatura ------------------------------------------------------
@pytest.mark.parametrize("radio", [4.0, 8.0, 15.0, 30.0, -8.0, -15.0])
def test_curvatura_de_un_arco_conocido(radio):
    p = fl.percepcion(pista_sintetica(radio=radio))
    assert p["kappa"] == pytest.approx(1 / radio, rel=0.10)


def test_recta_tiene_curvatura_casi_nula_y_es_recta():
    p = fl.percepcion(pista_sintetica(desplazamiento=0.05))
    assert abs(p["kappa"]) < 0.02
    assert p["recta"]


def test_curva_cerrada_no_es_recta():
    assert not fl.percepcion(pista_sintetica(radio=6.0))["recta"]


def test_desplazamiento_y_rumbo():
    p = fl.percepcion(pista_sintetica(desplazamiento=0.05, rumbo=0.1))
    assert p["y0"] == pytest.approx(0.05, abs=0.02)
    assert p["rumbo"] == pytest.approx(0.1, abs=0.03)


def test_error_en_pixeles_compatible_con_fase_1():
    # línea desplazada a la derecha → error positivo, como (cx - 320) en la fase 1
    p = fl.percepcion(pista_sintetica(desplazamiento=-0.05))
    assert p["e_px"] > 10


def test_sin_linea():
    p = fl.percepcion(np.full((480, 640, 3), 50, np.uint8))
    assert p["e_px"] is None
    assert p["validos"] == 0


# --- banda del horizonte ------------------------------------------------------
def test_banda_horizonte_ve_la_curva_antes():
    p = fl.percepcion(pista_sintetica(radio=20.0))  # curva a la izquierda que empieza lejos
    assert p["n_hor"] >= 5 and p["e_hor"] < -20  # línea a la izquierda en el horizonte
    assert abs(p["e_px"]) < 5  # cerca aún va recta: solo el horizonte la anticipa
    p = fl.percepcion(pista_sintetica(radio=-20.0))
    assert p["e_hor"] > 20
    assert fl.percepcion(pista_sintetica())["e_hor"] == pytest.approx(0.0, abs=2)


def test_banda_horizonte_ignora_otro_tramo_y_se_puede_mover():
    img = pista_sintetica()
    img[242, 20:60] = ROJO_BGR  # otro trozo del circuito asomando
    p = fl.percepcion(img)
    assert p["e_hor"] == pytest.approx(0.0, abs=2)
    p = fl.percepcion(img, fila_hor=245, n_filas_hor=3)
    assert list(p["filas_hor"]) == [245, 246, 247]
    assert fl.percepcion(np.full((480, 640, 3), 50, np.uint8))["e_hor"] is None
