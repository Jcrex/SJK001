"""Resumen de un log CSV de vuelta: error, giro y oscilación. Uso: python3 tools/resumen_log.py LOG.csv"""
import csv
import statistics as st
import sys

filas = list(csv.DictReader(open(sys.argv[1])))
e = [float(f["e"]) for f in filas if f["e"]]
w = [float(f["w"]) for f in filas]
t = [float(f["t_sim"]) for f in filas]
perdidas = sum(1 for f in filas if not f["e"])
cambios = sum(1 for a, b in zip(w, w[1:]) if a * b < 0)
dts = [b - a for a, b in zip(t, t[1:])]
print(f"fotogramas={len(filas)}  perdidos={perdidas}  dt medio={st.mean(dts)*1000:.1f} ms (máx {max(dts)*1000:.0f})")
print(f"|e| medio={st.mean(map(abs, e)):.1f} px  máx={max(map(abs, e)):.0f} px  rms={st.mean(x*x for x in e)**0.5:.1f}")
print(f"|w| máx={max(map(abs, w)):.2f} rad/s  cambios de signo de w={cambios} ({cambios/(t[-1]-t[0]):.2f}/s)")
