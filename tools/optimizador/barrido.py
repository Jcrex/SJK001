"""Barrido de un parámetro con el resto fijo: N vueltas por valor, para medir su efecto
sin que el optimizador mueva nada más.

    tools/barrido.sh                                  w_max = 6…13, 20 vueltas cada uno
    tools/barrido.sh --param w_max --valores 7,8,9 --repes 10
    tools/barrido.sh --base optuna_simple_v3          parámetros base de un preset del panel
    tools/barrido.sh --base optuna_simple_v4 --param v_max --valores 76,78,80 --fijar w_max=12
    tools/barrido.sh --resumen logs/optuna/barrido_w_max_<fecha>.csv

Necesita el panel y el controlador en marcha, igual que el optimizador, y **el optimizador
parado** (los dos mandarían parámetros al mismo coche).

- Base: por defecto, el grupo de parámetros con más vueltas completas de todas sus
  repeticiones (mínimo 3) y menor tiempo medio en logs/optuna/simple_v3_ensayos.csv. Se le
  añaden los «fijos» de config.json y se cambia solo el parámetro barrido.
- Orden intercalado: cada ronda prueba todos los valores una vez, en un orden barajado
  distinto. Así, si el simulador se degrada con el tiempo, afecta a todos los valores
  por igual.
- Reanudable: con --salida <csv> existente, continúa hasta completar N vueltas por valor.
- Las vueltas que no son culpa de los parámetros (interferencia, panel caído…) se repiten.
"""
import argparse
import asyncio
import csv
import json
import random
import subprocess
import time
from collections import defaultdict
from pathlib import Path

import aiohttp

from optimizar import PRESETS, SALIDA, Ensayador, cargar_config, objetivo

PARAMS = ["v_max", "k_v_curva", "v_min", "kp", "td", "n_filtro", "k_ff", "kv_kp",
          "t_mirar", "z_mirar_min", "k_hor"]
COLS = ["ronda", "valor", "completada", "motivo", "tiempo_s", "J", "e_rms", "e_max", "cambios_w_s",
        "perdidos", "distancia_m", "fecha"]


def base_mejor(ruta):
    """Grupo de parámetros más fiable (≥ 3 repeticiones) y, a igualdad, más rápido."""
    grupos = defaultdict(list)
    with open(ruta) as f:
        for fila in csv.DictReader(f):
            clave = tuple(round(float(fila[p]), 5) for p in PARAMS)
            grupos[clave].append(fila)
    mejores = []
    for clave, filas in grupos.items():
        if len(filas) < 3:
            continue
        ok = [float(x["tiempo_s"]) for x in filas if x["completada"] == "1"]
        if ok:
            mejores.append((-len(ok) / len(filas), sum(ok) / len(ok), -len(filas), clave, len(ok), len(filas)))
    mejores.sort()
    fiab, t, _, clave, ok, n = mejores[0]
    print(f"[barrido] base: grupo con {ok}/{n} completas, {t:.2f} s de media")
    return dict(zip(PARAMS, clave))


def resumen(ruta):
    por_valor = defaultdict(list)
    with open(ruta) as f:
        for fila in csv.DictReader(f):
            por_valor[float(fila["valor"])].append(fila)
    print(f"\n{'valor':>7} {'n':>3} {'completas':>10} {'t_min':>6} {'t_med':>6} {'t_medio':>7} "
          f"{'J_medio':>7} {'w±/s':>5} {'fallos (m recorridos)':<30}")
    for v in sorted(por_valor):
        fs = por_valor[v]
        ok = sorted(float(x["tiempo_s"]) for x in fs if x["completada"] == "1")
        osc = [float(x["cambios_w_s"]) for x in fs if x["completada"] == "1" and x["cambios_w_s"]]
        J = [float(x["J"]) for x in fs]
        dist = [x["distancia_m"] for x in fs if x["completada"] != "1"]
        med = ok[len(ok) // 2] if ok else float("nan")
        print(f"{v:>7g} {len(fs):>3} {len(ok):>4}/{len(fs):<2} ({len(ok) / len(fs):>4.0%})"
              f" {min(ok) if ok else float('nan'):>6.2f} {med:>6.2f}"
              f" {sum(ok) / len(ok) if ok else float('nan'):>7.3f} {sum(J) / len(J):>7.2f}"
              f" {sum(osc) / len(osc) if osc else float('nan'):>5.2f} {', '.join(dist)[:30]}")


async def correr(a, cfg, base, valores, ruta):
    hechas = defaultdict(int)
    nueva = not ruta.exists()
    if not nueva:
        with open(ruta) as f:
            for fila in csv.DictReader(f):
                hechas[float(fila["valor"])] += 1
        print(f"[barrido] reanudando {ruta.name}: " + ", ".join(f"{v:g}→{hechas[v]}" for v in valores))
    envio_base = {**cfg["fijos"], **base, **a.fijos}
    cols = COLS + list(envio_base) + ([a.param] if a.param not in envio_base else [])
    salida = open(ruta, "a", newline="", buffering=1)
    esc = csv.DictWriter(salida, cols, extrasaction="ignore")
    if nueva:
        esc.writeheader()
    azar = random.Random(a.semilla)
    ens = Ensayador(cfg)
    async with aiohttp.ClientSession() as sesion:
        await ens.conectar(sesion)
        await asyncio.sleep(1.0)  # recibir el esquema de parámetros del panel
        if ens.esquema and a.param not in ens.esquema:
            raise SystemExit(f"«{a.param}» no es un parámetro del controlador")
        ronda = min(hechas[v] for v in valores)
        while ronda < a.repes:
            orden = [v for v in valores if hechas[v] <= ronda]
            azar.shuffle(orden)
            for v in orden:
                envio = {**envio_base, a.param: v}
                while True:
                    try:
                        fila, motivo = await ens.vuelta(envio)
                    except ConnectionError:
                        print("[barrido] el panel se cerró; reconectando...", flush=True)
                        await ens.conectar(sesion)
                        continue
                    # timeout: el controlador no mandó nada en t_max_ensayo (play cortado o web recargada)
                    if motivo in ("sin_respuesta", "reinicio_fallido", "interrumpido", "interferencia", "timeout", "vuelta_corta"):
                        print(f"[barrido] {a.param}={v:g} descartada ({motivo}); se repite en 5 s", flush=True)
                        await asyncio.sleep(5)
                        continue
                    break
                datos = dict(fila or {}, motivo=motivo, completada=int(bool(fila and fila.get("completada"))))
                J = objetivo(fila, cfg)
                hechas[v] += 1
                esc.writerow({**datos, **envio, "ronda": hechas[v], "valor": v, "J": round(J, 3),
                              "fecha": time.strftime("%Y-%m-%d %H:%M:%S")})
                res = f"{datos.get('tiempo_s')} s" if datos["completada"] else \
                    f"FALLO ({motivo}, {datos.get('distancia_m', '?')} m)"
                print(f"[barrido] ronda {hechas[v]}/{a.repes}  {a.param}={v:g}: {res}", flush=True)
            ronda += 1
            resumen(ruta)
    salida.close()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--param", default="w_max")
    p.add_argument("--valores", default="6,7,8,9,10,11,12,13",
                   help="lista separada por comas (6 = referencia actual)")
    p.add_argument("--repes", type=int, default=20, help="vueltas por valor")
    p.add_argument("--base", default="mejor",
                   help="«mejor» (del CSV de ensayos) o nombre de un preset de tools/panel/presets")
    p.add_argument("--ensayos", default=str(SALIDA / "simple_v3_ensayos.csv"))
    p.add_argument("--salida", help="CSV de resultados (si existe, se reanuda)")
    p.add_argument("--semilla", type=int, default=1)
    p.add_argument("--config", default=str(Path(__file__).parent / "config.json"),
                   help="config del optimizador de la que se toman los «fijos» (config_v4.json → w_max = 12)")
    p.add_argument("--fijar", default="", help="k=v,k=v: valores fijos que pisan a la base y a la config")
    p.add_argument("--resumen", metavar="CSV", help="solo imprime el resumen de un barrido")
    a = p.parse_args()
    if a.resumen:
        resumen(Path(a.resumen))
        return
    if subprocess.run(["pgrep", "-f", "optimizador/optimizar.py"], capture_output=True).returncode == 0:
        raise SystemExit("El optimizador está en marcha: páralo antes (Ctrl+C); los dos moverían el mismo coche.")
    cfg = cargar_config(a.config)
    a.fijos = {k: float(v) for k, v in (x.split("=") for x in a.fijar.split(",") if x)}
    if a.base == "mejor":
        base = base_mejor(a.ensayos)
    else:
        base = {k: v for k, v in json.loads((PRESETS / f"{a.base}.json").read_text()).items() if k in PARAMS}
    print("[barrido] parámetros base: " + " ".join(f"{k}={v:g}" for k, v in base.items()))
    fijos = {**cfg["fijos"], **a.fijos}
    print("[barrido] fijos: " + " ".join(f"{k}={v:g}" for k, v in fijos.items() if k != a.param))
    valores = [float(x) for x in a.valores.split(",")]
    ruta = Path(a.salida) if a.salida else SALIDA / f"barrido_{a.param}_{time.strftime('%Y%m%d-%H%M%S')}.csv"
    print(f"[barrido] {a.param} = {', '.join(f'{v:g}' for v in valores)}; {a.repes} vueltas por valor "
          f"→ {ruta.relative_to(SALIDA.parents[1]) if ruta.is_relative_to(SALIDA.parents[1]) else ruta}")
    try:
        asyncio.run(correr(a, cfg, base, valores, ruta))
    except KeyboardInterrupt:
        print(f"\n[barrido] parado. Se reanuda con --salida {ruta}")
    resumen(ruta)


if __name__ == "__main__":
    main()
