"""Cronómetro de vueltas del panel (tools/panel/panel.py). Necesita aiohttp:
    uv run --with aiohttp --with pytest pytest tests/test_vueltas.py"""
import math
import os
import sys

import pytest

pytest.importorskip("aiohttp")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools", "panel"))
import panel  # noqa: E402

T = 0.05


def correr(v, radio=40.0, desfase=0.0, lateral=0.0, vueltas=1.2):
    """Coche en un círculo de `radio` que empieza en la salida a velocidad v (m/s), a 20 Hz.
    `desfase` mueve el primer fotograma tras la salida; `lateral` desplaza la trayectoria
    respecto al punto de salida en la meta. Devuelve las filas cerradas."""
    c = panel.Vueltas()
    filas = []
    c.dato({"t": 0.0, "x": 0.0, "y": 0.0, "e": 0.0, "w": 0.0, "v": v}, {})  # origen
    w = v / radio
    k = 1
    while k * T * w < 2 * math.pi * vueltas:
        a = (k * T + desfase) * w
        r = radio + lateral
        x, y = r * math.sin(a), radio - r * math.cos(a)
        f = c.dato({"t": k * T, "x": x, "y": y, "e": 0.0, "w": 0.1, "v": v}, {})
        if f:
            filas.append(f)
        k += 1
    return filas


def test_a_baja_velocidad_el_tiempo_es_el_de_una_vuelta():
    f = correr(10.0)
    assert len(f) == 1 and f[0]["completada"] == 1
    assert f[0]["tiempo_s"] == pytest.approx(2 * math.pi * 40 / 10, abs=0.01)
    assert f[0]["distancia_m"] == pytest.approx(2 * math.pi * 40, rel=0.002)


@pytest.mark.parametrize("v", [76.0, 95.0, 120.0])
@pytest.mark.parametrize("desfase", [0.0, 0.013, 0.026, 0.041])
def test_no_se_salta_la_meta_aunque_avance_mas_de_un_circulo_por_fotograma(v, desfase):
    # a 120 m/s avanza 6 m por fotograma: con el punto (r < 2.5 m) la meta se perdería
    f = correr(v, desfase=desfase, lateral=1.5)
    assert len(f) == 1
    assert f[0]["tiempo_s"] == pytest.approx(2 * math.pi * 40 / v - desfase, abs=0.01)


def test_la_meta_lejos_de_la_trayectoria_no_cuenta():
    assert correr(76.0, lateral=3.0) == []


def test_el_teletransporte_del_reinicio_no_cierra_una_vuelta():
    # Tras un choque, el primer fotograma aún trae la posición del choque (a 66 m de la
    # salida) y el siguiente ya está en la salida: ese salto no es un paso por meta.
    c = panel.Vueltas()
    c.dato({"t": 0.0, "x": 0.0, "y": 0.0, "e": 0.0, "w": 0.0, "v": 90.0}, {})  # origen
    c.fallo("choque", 0.0, {})
    filas = [c.dato({"t": 0.40, "x": -61.8, "y": -22.9, "e": 0.0, "w": 0.0, "v": 88.0}, {}),
             c.dato({"t": 0.45, "x": 0.1, "y": -2.8, "e": 0.0, "w": 0.0, "v": 93.0}, {})]
    for k in range(1, 6):  # y sigue recto alejándose de la salida
        filas.append(c.dato({"t": 0.45 + k * T, "x": 0.1, "y": -2.8 - 4.65 * k, "e": 0.0, "w": 0.0, "v": 93.0}, {}))
    assert [f for f in filas if f] == []
    assert c.dist == pytest.approx(5 * 4.65, abs=0.01)


def circulo(c, t0, v, radio=40.0, a0=0.0, vueltas=1.2):
    """Frames de un coche en un círculo que empieza en el ángulo a0 (0 = la salida)."""
    filas, w, k = [], v / radio, 0
    while k * T * w < 2 * math.pi * vueltas:
        a = a0 + k * T * w
        f = c.dato({"t": t0 + k * T, "x": radio * math.sin(a), "y": radio - radio * math.cos(a),
                    "e": 0.0, "w": 0.1, "v": v}, {})
        if f:
            filas.append(f)
        k += 1
    return filas


def test_panel_arrancado_a_mitad_de_vuelta_toma_la_meta_del_primer_reinicio():
    c = panel.Vueltas()
    circulo(c, 0.0, 90.0, a0=2.5, vueltas=0.3)  # el panel ve el coche a mitad de circuito
    c.fallo("pedido", 10.0, {})
    f = circulo(c, 10.0, 90.0)
    assert len(f) == 1 and f[0]["completada"] == 1
    assert f[0]["distancia_m"] == pytest.approx(2 * math.pi * 40, rel=0.01)


def test_primer_reinicio_con_fotograma_viejo_corrige_la_meta():
    c = panel.Vueltas()
    circulo(c, 0.0, 90.0, a0=2.5, vueltas=0.3)
    c.fallo("choque", 10.0, {})
    c.dato({"t": 10.0, "x": -61.8, "y": -22.9, "e": 0.0, "w": 0.0, "v": 88.0}, {})  # posición del choque
    f = circulo(c, 10.05, 90.0)
    assert len(f) == 1
    assert f[0]["distancia_m"] == pytest.approx(2 * math.pi * 40, rel=0.01)


def test_un_reinicio_fallido_no_mueve_la_meta():
    c = panel.Vueltas()
    circulo(c, 0.0, 90.0, a0=0.0, vueltas=0.3)
    origen = c.origen
    c.fallo("pedido", 10.0, {}, ok=False)
    assert c.origen == origen
