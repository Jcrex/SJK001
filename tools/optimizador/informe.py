"""Informe de un estudio del optimizador: mejores ensayos, importancia de cada
parámetro y gráficas. Se puede ejecutar mientras el optimizador sigue corriendo.

    tools/optimizar.sh --informe [estudio]

Escribe en logs/optuna/:
  <estudio>_historia.png      J de cada ensayo y el mejor hasta el momento
  <estudio>_importancia.png   cuánto explica cada parámetro de la variación de J (fANOVA)
  <estudio>_cortes.png        J frente a cada parámetro (dónde está la zona buena)
  <estudio>_tiempo_vs_osc.png tiempo de vuelta frente a oscilación (cambios de w por s)
"""
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import optuna  # noqa: E402

RAIZ = Path(__file__).resolve().parents[2]
SALIDA = RAIZ / "logs" / "optuna"


def main():
    cfg = json.loads((Path(__file__).parent / "config.json").read_text())
    nombre = sys.argv[1] if len(sys.argv) > 1 else cfg["estudio"]
    ventana = SALIDA / f"{nombre}_ventana.json"
    if ventana.exists():  # ventana móvil: la generación actual tiene todos los ensayos
        v = json.loads(ventana.read_text())
        nombre = v["actual"]
        for h in v.get("historial", []):
            print(f"ventana g{h['gen']} ({h['fecha']}, {h['motivo']}): "
                  + "; ".join(f"{k} → {b[0]:.4g}–{b[1]:.4g}" for k, (a, b) in h["cambios"].items()))
    estudio = optuna.load_study(study_name=nombre, storage=f"sqlite:///{SALIDA / (nombre + '.db')}")
    hechos = [t for t in estudio.trials if t.state == optuna.trial.TrialState.COMPLETE]
    if not hechos:
        print("Todavía no hay ensayos completados.")
        return
    completas = [t for t in hechos if t.user_attrs.get("completada")]
    print(f"Estudio «{nombre}»: {len(hechos)} ensayos, {len(completas)} vueltas completas, "
          f"{len(hechos) - len(completas)} fallidas")

    # Mismos parámetros repetidos = un grupo. Se ordena por la J media de todas sus
    # repeticiones (los choques incluidos), como hace el optimizador.
    grupos = {}
    for t in hechos:
        grupos.setdefault(tuple(sorted((k, round(v, 5)) for k, v in t.params.items())), []).append(t)
    filas = []
    for g in grupos.values():
        ok = [t for t in g if t.user_attrs.get("completada")]
        if ok:
            filas.append((sum(t.value for t in g) / len(g), g, ok))
    print("\nMejores 10 por J media de sus repeticiones (J = tiempo + penalización de oscilación):")
    for Jm, g, ok in sorted(filas, key=lambda f: (f[0], -len(f[1])))[:10]:
        t = min(ok, key=lambda t: t.value)
        ua = t.user_attrs
        par = " ".join(f"{k}={v:.4g}" for k, v in t.params.items())
        tiempos = sorted(x.user_attrs["tiempo_s"] for x in ok)
        print(f"  #{t.number:<4} J media={Jm:6.2f}  {len(ok)}/{len(g)} completas  "
              f"tiempo {tiempos[0]}" + (f"–{tiempos[-1]}" if len(tiempos) > 1 else "")
              + f" s  e_rms={ua.get('e_rms')}  w±/s={ua.get('cambios_w_s')}\n        {par}")

    # Repeticiones de los mismos parámetros: ¿es reproducible?
    rep = [g for g in grupos.values() if len(g) > 1]
    if rep:
        print("\nParámetros repetidos (comprobación de que el resultado no fue suerte):")
        for g in rep:
            ok = sum(1 for t in g if t.user_attrs.get("completada"))
            print(f"  {ok}/{len(g)} completas: " + ", ".join(
                f"#{t.number}: " + (f"{t.user_attrs.get('tiempo_s')} s" if t.user_attrs.get("completada")
                                    else f"FALLO ({t.user_attrs.get('motivo') or '?'})") for t in g))

    if len(hechos) >= 8:
        try:
            imp = optuna.importance.get_param_importances(estudio)
            print("\nImportancia de cada parámetro sobre J (fANOVA, suma 1):")
            for k, v in imp.items():
                print(f"  {k:<12} {v:5.2f} " + "█" * int(v * 40))
            fig, ax = plt.subplots(figsize=(7, 4))
            ax.barh(list(imp)[::-1], list(imp.values())[::-1], color="#2f6fde")
            ax.set_xlabel("importancia sobre J")
            ax.set_title(f"{nombre}: qué parámetros importan")
            fig.tight_layout()
            fig.savefig(SALIDA / f"{nombre}_importancia.png", dpi=110)
        except Exception as ex:  # pocas muestras o todas iguales
            print("\n(importancia no disponible todavía:", ex, ")")

    n = [t.number for t in hechos]
    J = [t.value for t in hechos]
    mejor = [min(J[: i + 1]) for i in range(len(J))]
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.scatter(n, J, s=14, c=["#2f6fde" if t.user_attrs.get("completada") else "#d64545" for t in hechos])
    ax.plot(n, mejor, color="#111827", lw=1.5, label="mejor hasta el momento")
    ax.set_xlabel("ensayo")
    ax.set_ylabel("J")
    ax.set_title("Historia (rojo = choque o salida)")
    ax.legend()
    ax.grid(alpha=0.3)
    if completas:
        ax.set_ylim(min(J) - 1, max(t.value for t in completas) + 3)
    fig.tight_layout()
    fig.savefig(SALIDA / f"{nombre}_historia.png", dpi=110)

    params = list(hechos[0].params)
    cols = 4
    filas = (len(params) + cols - 1) // cols
    fig, axs = plt.subplots(filas, cols, figsize=(4 * cols, 3 * filas), squeeze=False)
    for ax, k in zip(axs.flat, params):
        ax.scatter([t.params[k] for t in completas], [t.value for t in completas], s=10, color="#2f6fde")
        fall = [t for t in hechos if not t.user_attrs.get("completada")]
        if completas:
            tope = max(t.value for t in completas) + 1
            ax.scatter([t.params[k] for t in fall], [tope] * len(fall), marker="x", s=14, color="#d64545")
        ax.set_title(k)
        ax.grid(alpha=0.3)
    for ax in list(axs.flat)[len(params):]:
        ax.axis("off")
    fig.suptitle("J frente a cada parámetro (aspas arriba = fallos)")
    fig.tight_layout()
    fig.savefig(SALIDA / f"{nombre}_cortes.png", dpi=100)

    if completas:
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.scatter([t.user_attrs["cambios_w_s"] for t in completas], [t.user_attrs["tiempo_s"] for t in completas],
                   s=14, color="#2f6fde")
        ax.set_xlabel("cambios de signo de w por s (oscilación)")
        ax.set_ylabel("tiempo de vuelta (s)")
        ax.set_title("¿Se puede ir rápido sin oscilar? (abajo a la izquierda = mejor)")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(SALIDA / f"{nombre}_tiempo_vs_osc.png", dpi=110)
    print(f"\nGráficas en logs/optuna/{nombre}_*.png")


if __name__ == "__main__":
    main()
