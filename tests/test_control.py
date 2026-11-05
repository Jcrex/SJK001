"""Pruebas del PID, el perfil de velocidad, los parámetros y la telemetría."""
import json
import os
import socket
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import follow_line as fl  # noqa: E402


def percep(e_px, kappa=0.0, kappa_lejos=None):
    return {"e_px": e_px, "kappa": kappa, "kappa_lejos": kappa_lejos}


def test_por_defecto_es_el_control_p_de_la_fase_1():
    ctl = fl.Controlador(dict(fl.params_por_defecto(), v_max=10.0, kp=0.008))
    for e in (40.0, -120.0, 0.0, 300.0):
        v, w, _ = ctl.paso(percep(e), 0.05)
        assert v == pytest.approx(10.0)
        assert w == pytest.approx(fl.control_p(fl.CX + e, kp=0.008))


def test_integral_elimina_el_error_constante_y_d_es_cero_en_regimen():
    pid = fl.PID()
    u = [pid.paso(0.0, 10.0, 0.05, kp=0.01, ti=1.0)[0] for _ in range(20)]
    assert u[-1] < u[0] < 0  # la I sigue sumando con error constante
    pid = fl.PID()
    for _ in range(5):
        _, partes = pid.paso(0.0, 10.0, 0.05, kp=0.01, td=0.1)
    assert partes["d"] == pytest.approx(0.0, abs=1e-12)


def test_derivada_filtrada_reacciona_al_cambio_y_sin_pico_infinito():
    pid = fl.PID()
    pid.paso(0.0, 0.0, 0.05, kp=0.01, td=0.1, n=10)
    _, partes = pid.paso(0.0, 50.0, 0.05, kp=0.01, td=0.1, n=10)
    sin_filtro = -0.01 * 0.1 / 0.05 * 50.0
    assert sin_filtro < partes["d"] < 0  # mismo signo, pero suavizada


def test_anti_windup_no_integra_en_saturacion():
    pid = fl.PID()
    for _ in range(200):
        u, partes = pid.paso(0.0, 300.0, 0.05, kp=0.01, ti=0.5, u_max=1.0)
    assert u == pytest.approx(-1.0)
    assert abs(partes["i"]) < 1.0  # sin anti-windup llegaría a ~ -60


def test_ff_gira_hacia_la_curva():
    ctl = fl.Controlador(dict(fl.params_por_defecto(), k_ff=1.0, kp=0.0))
    _, w, partes = ctl.paso(percep(0.0, kappa=0.1), 0.05)
    assert w == pytest.approx(10.0 * 0.1) and partes["ff"] > 0  # κ > 0 izquierda → w > 0


def test_perfil_de_velocidad_y_rampa():
    prm = dict(fl.params_por_defecto(), v_max=20.0, k_v_curva=50.0, v_min=5.0)
    assert fl.v_referencia(0.0, 0.0, prm) == pytest.approx(20.0)
    assert fl.v_referencia(0.02, 0.0, prm) == pytest.approx(10.0)  # 20 / (1 + 50·0.02)
    assert fl.v_referencia(None, -0.02, prm) == pytest.approx(10.0)  # sin κ_lejos usa κ
    assert fl.v_referencia(1.0, 0.0, prm) == pytest.approx(5.0)  # recorta a v_min
    assert fl.rampa(10.0, 20.0, 0.05, a_sub=10.0, a_fren=0.0) == pytest.approx(10.5)
    assert fl.rampa(10.0, 2.0, 0.05, a_sub=0.0, a_fren=40.0) == pytest.approx(8.0)
    assert fl.rampa(10.0, 2.0, 0.05, a_sub=0.0, a_fren=0.0) == pytest.approx(2.0)


def test_linea_perdida_frena_y_mantiene_el_giro():
    ctl = fl.Controlador()
    _, w0, _ = ctl.paso(percep(100.0), 0.05)
    v, w, _ = ctl.paso({"e_px": None, "kappa": 0.0, "kappa_lejos": None}, 0.05)
    assert v == ctl.prm["v_perdida"] and w == w0


def test_ajustar_params_recorta_al_rango_e_ignora_desconocidos():
    prm = fl.ajustar_params(fl.params_por_defecto(), {"kp": 99.0, "no_existe": 1, "ti": "0.5"})
    assert prm["kp"] == 0.04 and prm["ti"] == 0.5 and "no_existe" not in prm


def test_params_desde_entorno(monkeypatch):
    monkeypatch.setenv("SJK_V", "12")
    monkeypatch.setenv("SJK_KP", "0.01")
    prm = fl.params_por_defecto()
    assert prm["v_max"] == 12.0 and prm["kp"] == 0.01


def test_telemetria_manda_datos_y_recibe_parametros():
    tel = fl.Telemetria(0, fl.params_por_defecto())  # puerto 0: lo elige el sistema
    puerto = tel.srv.getsockname()[1]
    cli = socket.create_connection(("127.0.0.1", puerto), timeout=2)
    f = cli.makefile("r")
    hola = json.loads(f.readline())
    assert hola["tipo"] == "hola" and {p["nombre"] for p in hola["params"]} == {p[0] for p in fl.PARAMS}
    for _ in range(100):
        if tel.conectado:
            break
        time.sleep(0.01)
    tel.enviar({"tipo": "dato", "t": 1.0})
    assert json.loads(f.readline()) == {"tipo": "dato", "t": 1.0}
    cli.sendall(b'{"tipo": "set", "valores": {"kp": 0.02}}\n')
    assert json.loads(f.readline())["valores"]["kp"] == 0.02
    assert tel.prm["kp"] == 0.02
    cli.close()
