"""¿La curvatura que ve la cámara anticipa la que recorre el coche?

Referencia: la curvatura real recorrida, wz/vx de la odometría (_odom.csv),
interpolada a los instantes del log y suavizada. NO se usa la w/v mandada:
con el coche alineado con un eje del mundo, la fricción de contacto del
simulador anula las w pequeñas (ver PLAN.md, 5.3) y la w mandada no se cumple.

Se compara con κ (todas las filas) y κ_lejos (Z ≥ 1.5 m) desplazados en el
tiempo; el desfase de máxima correlación es el adelanto con que la cámara
"ve" la curva, y la pendiente del ajuste dice si κ está bien escalada (≈ 1).

Uso: python3 analysis/kappa_vs_trayectoria.py LOG.csv [salida.png]
     (busca LOG_odom.csv al lado)
"""
import csv
import sys

import numpy as np


def columnas(ruta):
    filas = list(csv.DictReader(open(ruta)))
    return {k: np.array([float(f[k]) if f[k] else np.nan for f in filas]) for k in filas[0]}


log = columnas(sys.argv[1])
odom = columnas(sys.argv[1].replace(".csv", "_odom.csv"))

t = log["t_sim"]
k, kl = log["kappa"], log["kappa_lejos"]
mov = odom["vx"] > 1.0
curv_odom = np.interp(t, odom["t_sim"][mov], odom["wz"][mov] / odom["vx"][mov])
curv_cmd = log["w"] / np.maximum(log["v"], 1e-3)


def suaviza(x, n=5):
    return np.convolve(np.nan_to_num(x), np.ones(n) / n, mode="same")


def mejor_desfase(x, ref):
    mejor = None
    for lag in range(0, 41):  # 0–2 s a 20 Hz
        a, b = x[: len(x) - lag], ref[lag:]
        ok = np.isfinite(a)
        r = np.corrcoef(a[ok], b[ok])[0, 1]
        if mejor is None or r > mejor[1]:
            mejor = (lag, r, np.polyfit(a[ok], b[ok], 1)[0])
    return mejor


dt = np.median(np.diff(t))
print(f"giro neto: odometría {np.sum(curv_odom * log['v']) * dt:+.2f} rad, "
      f"mandado {np.sum(log['w']) * dt:+.2f} rad, cámara κ {np.nansum(k * log['v']) * dt:+.2f} rad")
print(f"\n{'señal':<9}{'referencia':<12}{'adelanto':>10}{'correlación':>13}{'pendiente':>11}")
for nombre_ref, ref in (("odometría", suaviza(curv_odom)), ("w/v mand.", suaviza(curv_cmd))):
    for nombre, x in (("κ", k), ("κ_lejos", kl)):
        lag, r, m = mejor_desfase(x, ref)
        print(f"{nombre:<9}{nombre_ref:<12}{lag * dt:>8.2f} s{r:>13.2f}{m:>11.2f}")

ref = suaviza(curv_odom)
print(f"\nκ en recta (|curv. real| < 0.01): |κ| medio = {np.mean(np.abs(k[np.abs(ref) < 0.01])):.4f} 1/m")
print(f"curvatura máx. recorrida = {np.max(np.abs(ref)):.3f} 1/m (radio mín ≈ {1 / np.max(np.abs(ref)):.1f} m)")

if len(sys.argv) > 2:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tt = t - t[0]
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(tt, ref, label="curvatura recorrida (odometría wz/vx)", color="black", lw=1.5)
    ax.plot(tt, suaviza(curv_cmd), label="w/v mandada", color="grey", lw=1, ls="--")
    ax.plot(tt, k, label="κ cámara (todas las filas)", lw=1)
    ax.plot(tt, kl, label="κ_lejos (Z ≥ 1.5 m)", lw=1, alpha=0.8)
    ax.set_xlabel("t (s)"); ax.set_ylabel("curvatura (1/m)"); ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(sys.argv[2], dpi=110)
    print("gráfica:", sys.argv[2])
