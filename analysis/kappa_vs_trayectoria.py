"""¿La curvatura que ve la cámara anticipa la que recorre el coche?

El coche es cinemático: su curvatura real es w/v. Se compara con κ (todas las
filas) y κ_lejos (Z ≥ 1.5 m) desplazados en el tiempo; el desfase de máxima
correlación es el adelanto con que la cámara "ve" la curva.
Uso: python3 analysis/kappa_vs_trayectoria.py LOG.csv [salida.png]
"""
import csv
import sys

import numpy as np

filas = list(csv.DictReader(open(sys.argv[1])))
v = np.array([float(f["v"]) for f in filas])
w = np.array([float(f["w"]) for f in filas])
k = np.array([float(f["kappa"]) for f in filas])
kl = np.array([float(f["kappa_lejos"]) if f["kappa_lejos"] else np.nan for f in filas])
real = w / np.maximum(v, 1e-3)

def suaviza(x, n=5):
    return np.convolve(np.nan_to_num(x), np.ones(n) / n, mode="same")

real_s = suaviza(real)
print(f"{'señal':<9}{'adelanto':>10}{'correlación':>13}{'pendiente':>11}")
for nombre, x in (("κ", k), ("κ_lejos", kl)):
    mejor = None
    for lag in range(0, 41):  # 0–2 s a 20 Hz
        a, b = x[: len(x) - lag], real_s[lag:]
        ok = np.isfinite(a)
        r = np.corrcoef(a[ok], b[ok])[0, 1]
        if mejor is None or r > mejor[1]:
            mejor = (lag, r, np.polyfit(a[ok], b[ok], 1)[0])
    print(f"{nombre:<9}{mejor[0] * 0.05:>8.2f} s{mejor[1]:>13.2f}{mejor[2]:>11.2f}")

print(f"\nκ en recta (|w/v| < 0.01): |κ| medio = {np.mean(np.abs(k[np.abs(real_s) < 0.01])):.4f} 1/m")
print(f"curvatura máx. recorrida |w/v| = {np.max(np.abs(real_s)):.3f} 1/m (radio mín ≈ {1/np.max(np.abs(real_s)):.1f} m)")

if len(sys.argv) > 2:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = np.arange(len(k)) * 0.05
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, real_s, label="w/v (curvatura recorrida)", color="black", lw=1.5)
    ax.plot(t, k, label="κ cámara (todas las filas)", lw=1)
    ax.plot(t, kl, label="κ_lejos (Z ≥ 1.5 m)", lw=1, alpha=0.8)
    ax.set_xlabel("t (s)"); ax.set_ylabel("curvatura (1/m)"); ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(sys.argv[2], dpi=110)
    print("gráfica:", sys.argv[2])
