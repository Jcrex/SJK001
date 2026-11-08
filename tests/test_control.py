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


def test_fase1_es_el_control_p():
    ctl = fl.Controlador(dict(fl.params_fase1(), v_max=10.0, kp=0.008))
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
    ctl = fl.Controlador(dict(fl.params_fase1(), k_ff=1.0, kp=0.0))
    _, w, partes = ctl.paso(percep(0.0, kappa=0.1), 0.05)
    assert w == pytest.approx(10.0 * 0.1) and partes["ff"] > 0  # κ > 0 izquierda → w > 0


def test_perfil_de_velocidad_y_rampa():
    prm = dict(fl.params_fase1(), v_max=20.0, k_v_curva=50.0, v_min=5.0)
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


def test_ajustar_params_solo_recorta_el_minimo_e_ignora_desconocidos():
    prm = fl.ajustar_params(fl.params_fase1(),
                            {"kp": 99.0, "v_max": 60.0, "td": -1.0, "no_existe": 1, "ti": "0.5"})
    assert prm["kp"] == 99.0 and prm["v_max"] == 60.0  # sin techo
    assert prm["td"] == 0.0 and prm["ti"] == 0.5 and "no_existe" not in prm
    assert fl.ajustar_params(prm, {"reinicio_auto": 5})["reinicio_auto"] == 1.0  # tope duro


def test_params_desde_entorno(monkeypatch):
    monkeypatch.setenv("SJK_V", "12")
    monkeypatch.setenv("SJK_KP", "0.01")
    prm = fl.params_por_defecto()
    assert prm["v_max"] == 12.0 and prm["kp"] == 0.01


def test_telemetria_manda_datos_y_recibe_parametros():
    tel = fl.Telemetria(0, fl.params_fase1())  # puerto 0: lo elige el sistema
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


def test_error_mirada_es_la_columna_de_la_linea_a_distancia_L():
    p = {"coef": (-0.5, 0.0, 0.0), "z_max": 10.0}  # línea recta 0.5 m a la derecha
    e, L = fl.error_mirada(p, v=10.0, t_mirar=0.2, z_min=1.0)
    assert L == pytest.approx(2.0) and e == pytest.approx(fl.F * 0.5 / 2.0)  # 80 px a la derecha
    _, L = fl.error_mirada(p, v=50.0, t_mirar=1.0, z_min=1.0)
    assert L == pytest.approx(10.0)  # recortada a lo visible
    _, L = fl.error_mirada(p, v=1.0, t_mirar=0.1, z_min=1.5)
    assert L == pytest.approx(1.5)  # mínimo
    assert fl.error_mirada(p, 10.0, 0.0, 1.0) == (None, 0.0)  # desactivada
    # Con rumbo: línea centrada ahora pero el coche apunta a la izquierda de ella (c1 < 0)
    e, _ = fl.error_mirada({"coef": (0.0, -0.1, 0.0), "z_max": 10.0}, 10.0, 0.3, 1.0)
    assert e > 0  # → girar a la derecha antes de que aparezca error cerca


def test_controlador_usa_la_mirada_si_esta_activa():
    prm = dict(fl.params_fase1(), t_mirar=0.2, kp=0.01)
    ctl = fl.Controlador(prm)
    p = {"e_px": 0.0, "kappa": 0.0, "kappa_lejos": None, "coef": (-0.5, 0.0, 0.0), "z_max": 10.0}
    _, w, partes = ctl.paso(p, 0.05)
    assert partes["e_ctl"] == pytest.approx(80.0) and w == pytest.approx(-0.8)


def test_vigilante_sin_linea_y_choque():
    prm = dict(fl.params_fase1(), t_perdida_max=1.0, t_atasco=0.5)
    vig = fl.Vigilante()
    sin = {"validos": 0}
    con = {"validos": 10}
    assert vig.comprobar(0.0, sin, 10.0, 10.0, prm) is None
    assert vig.comprobar(0.9, sin, 10.0, 10.0, prm) is None
    assert vig.comprobar(1.1, sin, 10.0, 10.0, prm) == "sin_linea"
    vig.limpiar()
    assert vig.comprobar(2.0, con, 20.0, 3.0, prm) is None  # empieza el atasco
    assert vig.comprobar(2.6, con, 20.0, 3.0, prm) == "choque"
    vig.limpiar()
    assert vig.comprobar(3.0, con, 20.0, 19.0, prm) is None  # va bien
    assert vig.comprobar(3.0, sin, 20.0, 3.0, dict(prm, reinicio_auto=0.0)) is None


def test_elegir_salida():
    s = fl.SALIDAS["simple"]
    assert fl.elegir_salida((s[0] + 1.0, s[1], -1.5, 0, 0)) == s  # cerca de la conocida → exacta
    assert fl.elegir_salida((0.0, 0.0, 1.0, 0, 0)) == (0.0, 0.0, 0.004, 1.0)


def test_horizonte_gira_hacia_la_curva_y_cero_lo_desactiva():
    p = {"e_px": 0.0, "kappa": 0.0, "kappa_lejos": None, "e_hor": -50.0}  # curva a la izquierda
    _, w, partes = fl.Controlador(dict(fl.params_fase1(), k_hor=0.01)).paso(p, 0.05)
    assert w == pytest.approx(0.5) and partes["hor"] == pytest.approx(0.5) and partes["ff"] == 0.0
    _, w, _ = fl.Controlador(fl.params_fase1()).paso(p, 0.05)  # k_hor = 0
    assert w == 0.0


def test_por_defecto_son_los_parametros_elegidos():
    # macroanálisis final (ESTUDIO_PID §18): preset w14_v93_v6, 63/65 vueltas a 5.29 s
    prm = fl.params_por_defecto()
    assert prm["v_max"] == 93.0 and prm["w_max"] == 14.0 and prm["kp"] == 0.01139
    assert prm["k_hor"] == 0.0357 and prm["t_mirar"] == 0.11597
