"""Ventana móvil del optimizador (tools/optimizador/optimizar.py). Necesita optuna:
    uv run --with optuna --with aiohttp --with pytest pytest tests/test_ventana.py"""
import os
import sys

import pytest

pytest.importorskip("optuna")
pytest.importorskip("aiohttp")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools", "optimizador"))
import optimizar as o  # noqa: E402


def test_se_desplaza_hacia_el_borde_sin_dejar_fuera_a_los_mejores():
    top = [{"v": 29.5 + 0.05 * i, "k": 0.5} for i in range(10)]
    c = o.mover_ventana({"v": (16.0, 30.0), "k": (0.0, 1.0)}, top, {"v": 1.0}, {}, 0.1, 0.25)
    assert c == {"v": (19.5, 33.5)}  # ancho 14 → +3.5 por los dos lados; k en el centro no se mueve


def test_hacia_abajo_respeta_el_minimo_del_controlador():
    top = [{"z": 0.21} for _ in range(10)]
    assert o.mover_ventana({"z": (0.2, 4.0)}, top, {"z": 0.2}, {}, 0.1, 0.25) == {}  # ya en el mínimo
    lo, hi = o.mover_ventana({"z": (0.5, 4.0)}, top, {"z": 0.2}, {}, 0.1, 0.25)["z"]
    assert lo == 0.2 and hi == pytest.approx(3.7)


def test_mitad_en_el_borde_basta_y_menos_no():
    mitad = [{"v": 29.9}] * 5 + [{"v": 20.0}] * 5
    assert "v" in o.mover_ventana({"v": (16.0, 30.0)}, mitad, {}, {}, 0.1, 0.25)
    pocos = [{"v": 29.9}] * 4 + [{"v": 20.0}] * 6
    assert o.mover_ventana({"v": (16.0, 30.0)}, pocos, {}, {}, 0.1, 0.25) == {}
    # con mejores en 20, el mínimo sube 3.5 (16 → 19.5), que sigue por debajo de 20
    lo, hi = o.mover_ventana({"v": (16.0, 30.0)}, mitad, {}, {}, 0.1, 0.25)["v"]
    assert hi == 33.5 and lo == pytest.approx(18.5)  # no sube más: 20 queda a 0.1·ancho del borde
    assert 20.0 - lo >= 0.1 * (hi - lo) - 1e-6


def test_sin_rebote_tras_desplazar():
    top = [{"v": 21.0}, {"v": 21.5}, {"v": 22.0}]
    esp = {"v": (20.0, 24.0)}
    esp.update(o.incluir_valores(esp, {"v": 33.0}, top, {}, {}, 0.15))
    assert esp["v"][0] <= 21.0 <= 33.0 <= esp["v"][1]
    assert o.mover_ventana(esp, top, {}, {}, 0.15, 0.25) == {}  # la revisión siguiente no la devuelve


def test_incluir_valor_pedido_desde_el_panel():
    top = [{"v": 29.0}, {"v": 28.0}]
    c = o.incluir_valores({"v": (16.0, 30.0), "k": (0.0, 1.0)}, {"v": 35.0, "k": 0.5}, top, {}, {})
    lo, hi = c["v"]
    assert hi == pytest.approx(36.4) and lo <= 28.0 and list(c) == ["v"]
    assert 28.0 - lo >= 0.1 * (hi - lo) - 1e-6
    c = o.incluir_valores({"t": (0.5, 1.0)}, {"t": -3.0}, [], {"t": 0.0}, {})
    assert c["t"][0] == 0.0  # no baja del mínimo del controlador
    assert o.incluir_valores({"r": (0.0, 1.0)}, {"r": 5.0}, [], {}, {"r": 1.0}) == {}  # tope duro


def _estudio_con(vueltas):
    import optuna
    """Estudio en memoria con ensayos [(params, J, completada)]."""
    est = optuna.create_study()
    dist = {"v": optuna.distributions.FloatDistribution(0, 100)}
    for params, J, ok in vueltas:
        est.add_trial(optuna.trial.create_trial(params=params, distributions=dist, value=J,
                                                user_attrs={"completada": ok, "tiempo_s": J if ok else 5.0}))
    return est


def test_clasificacion_ordena_por_la_media_de_las_repeticiones():
    suerte = [({"v": 50.0}, 9.0, 1)] + [({"v": 50.0}, 75.0, 0)] * 3  # 1 de 4: media 58.5
    fiable = [({"v": 40.0}, 10.0, 1)] * 3
    solo_fallos = [({"v": 60.0}, 80.0, 0)]
    clas = o.clasificacion(_estudio_con(suerte + fiable + solo_fallos))
    assert [r["params"]["v"] for r in clas] == [40.0, 50.0]  # sin vuelta completa no entra
    assert (clas[0]["n"], clas[0]["ok"], clas[0]["J"]) == (3, 3, 10.0)
    assert (clas[1]["n"], clas[1]["ok"]) == (4, 1)
    assert o.mejores(_estudio_con(suerte + fiable), 1) == [{"v": 40.0}]


def test_confirmar_salta_los_ya_repetidos():
    clas = o.clasificacion(_estudio_con([({"v": 40.0}, 10.0, 1)] * 3 + [({"v": 45.0}, 10.5, 1)]))
    assert o.a_confirmar(clas, hasta=3)["params"] == {"v": 45.0}
    assert o.a_confirmar(clas, hasta=5)["params"] == {"v": 40.0}
    assert o.a_confirmar(clas[:1], hasta=3) is None
