"""Macroanálisis de todos los ensayos del optimizador y de la telemetría de las vueltas.

Responde a tres preguntas:
  1. ¿Cómo ha evolucionado la búsqueda? (tiempo, fiabilidad, generaciones)
  2. ¿Qué parámetros tienen más impacto? Por separado sobre el TIEMPO de las vueltas
     completas y sobre la PROBABILIDAD DE COMPLETAR, que no dependen de lo mismo.
  3. ¿Hacia dónde tiende el PID? Cómo se mueven los mejores parámetros al subir v_max
     y qué término del control (P, D, FF, HOR) hace el trabajo en la pista.
  4. Con tandas de 5 vueltas seguidas sin reiniciar (simple_v9): ¿qué fiabilidad hay
     en vueltas lanzadas, dónde falla y qué combinación aguanta? (sección 11, aparte:
     un ensayo de simple_v9 es una tanda, no una vuelta, y no se mezcla con el resto).

Se puede ejecutar con el optimizador en marcha (solo lee los CSV):

    uv run --with pandas --with scikit-learn --with matplotlib --with scipy \
        python tools/optimizador/macro_analisis.py

Escribe las figuras en docs/img/estudio/ y el resumen numérico por la salida estándar.
"""
import glob
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from scipy.stats import spearmanr  # noqa: E402
from sklearn.ensemble import RandomForestClassifier, RandomForestRegressor  # noqa: E402
from sklearn.inspection import partial_dependence, permutation_importance  # noqa: E402
from sklearn.model_selection import KFold, StratifiedKFold, cross_val_predict  # noqa: E402
from sklearn.metrics import r2_score, roc_auc_score  # noqa: E402

RAIZ = Path(__file__).resolve().parents[2]
OPT = RAIZ / "logs" / "optuna"
FIG = RAIZ / "docs" / "img" / "estudio"
FIG.mkdir(parents=True, exist_ok=True)

PARAMS = ["v_max", "k_v_curva", "v_min", "kp", "td", "n_filtro", "k_ff", "kv_kp",
          "t_mirar", "z_mirar_min", "k_hor"]
DERIVADOS = ["L_max", "kp_ef", "ganancia_lazo", "tau_d", "v_curva"]
AZUL, ROJO, GRIS, NEGRO, VERDE = "#2f6fde", "#d64545", "#9ca3af", "#111827", "#16a34a"
plt.rcParams.update({"axes.grid": True, "grid.alpha": 0.3, "axes.spines.top": False,
                     "axes.spines.right": False, "figure.dpi": 110})


# Regímenes con el mismo espacio de parámetros: w_max = 6 (simple_v3), w_max = 12 (simple_v4–v5) y
# w_max = 14 (simple_v6–v8; simple_v7 con v_max fija en 93 y otra J, más peso a la estabilidad; simple_v8
# con la J de simple_v7 y v_max en 93–97). simple_v9 (tandas de 5 vueltas) va aparte, en la sección 11.
# Cada uno tiene su corte de «rápido», sus tramos de v_max y su sufijo en los nombres de las figuras.
REGIMENES = [
    {"nombre": "simple_v3 (w_max = 6)", "prefijos": ("simple_v3",), "sufijo": "", "rapido": 48,
     "tramos": [0, 30, 35, 40, 44, 48, 51, 54, 56, 58, 60, 70], "palancas_desde": 54,
     "cortes_kvc": [0, 3, 4, 5, 6, 7, 8, 10, 15], "cortes_khor": [0, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.12, 1]},
    {"nombre": "simple_v4–v5 (w_max = 12)", "prefijos": ("simple_v4", "simple_v5"), "sufijo": "_w12", "rapido": 90,
     "tramos": [55, 65, 70, 74, 78, 84, 88, 90, 92, 94, 96, 98, 105], "palancas_desde": 90,
     "cortes_kvc": [1.5, 2.2, 2.4, 2.6, 2.8, 3.0, 3.5, 6, 11], "cortes_khor": [0.02, 0.035, 0.04, 0.045, 0.05, 0.06, 0.08]},
    {"nombre": "simple_v6–v8 (w_max = 14)", "prefijos": ("simple_v6", "simple_v7", "simple_v8"), "sufijo": "_w14", "rapido": 97,
     "tramos": [90, 93.5, 95, 97, 98, 99, 100, 103], "palancas_desde": 92.5,
     "cortes_kvc": [1.0, 1.4, 1.6, 1.8, 2.0, 2.2, 2.6], "cortes_khor": [0.025, 0.033, 0.036, 0.039, 0.042, 0.055]},
]


def del_regimen(df, reg):
    return df[df.estudio.str.startswith(reg["prefijos"])].copy()


def titulo(t):
    print("\n" + "=" * 78 + f"\n{t}\n" + "=" * 78)


# ---------------------------------------------------------------- carga
def cargar():
    dfs = []
    for f in ["simple_v1", "simple_v2", "simple_v3", "simple_v4", "simple_v5", "simple_v6", "simple_v7",
              "simple_v8"]:
        ruta = OPT / f"{f}_ensayos.csv"
        if ruta.exists():
            d = pd.read_csv(ruta)
            d["fichero"] = f
            dfs.append(d)
    df = pd.concat(dfs, ignore_index=True)
    df["estudio"] = df["estudio"].fillna(df["fichero"]) if "estudio" in df else df["fichero"]
    df["k_hor"] = df["k_hor"].fillna(0.0)
    df["fecha"] = pd.to_datetime(df["fecha"])
    df = df.sort_values("fecha", kind="stable").reset_index(drop=True)
    df["orden"] = np.arange(len(df))
    df["ok"] = df["completada"].astype(int)
    # Magnitudes con sentido físico (a la velocidad máxima, que es donde se decide la vuelta)
    df["L_max"] = np.maximum(df.v_max * df.t_mirar, df.z_mirar_min)       # m que mira por delante
    df["kp_ef"] = df.kp / (1 + df.kv_kp * df.v_max)                        # Kp tras el gain scheduling
    df["ganancia_lazo"] = df.kp_ef * df.v_max                              # Y/W = v/s² → K·v
    df["tau_d"] = df.td / df.n_filtro.clip(lower=1e-6)                      # constante del filtro de la D
    df["v_curva"] = (df.v_max / (1 + df.k_v_curva * 0.05)).clip(lower=df.v_min)  # v en κ_lejos = 0.05
    df["v_media"] = np.where(df.ok == 1, df.distancia_m / df.tiempo_s, np.nan)
    df["clave"] = df[PARAMS].round(5).astype(str).agg("|".join, axis=1)
    return df


# ---------------------------------------------------------------- 1. evolución
MARCAS = ("simple_v1", "simple_v2", "simple_v3", "simple_v3_g4", "simple_v3_g5", "simple_v3_g6", "simple_v4",
          "simple_v5", "simple_v6", "simple_v7", "simple_v8")


def evolucion(df):
    titulo("1. EVOLUCIÓN DE LA BÚSQUEDA")
    g = df.groupby("estudio", sort=False)
    tab = pd.DataFrame({
        "ensayos": g.size(),
        "completas_%": (g.ok.mean() * 100).round(1),
        "v_max_med": g.v_max.median().round(1),
        "t_min": g.apply(lambda x: x.loc[x.ok == 1, "tiempo_s"].min()),
        "t_mediana_ok": g.apply(lambda x: x.loc[x.ok == 1, "tiempo_s"].median()),
        "J_min": g.J.min().round(2),
        "desde": g.fecha.min().dt.strftime("%d %H:%M"),
        "hasta": g.fecha.max().dt.strftime("%d %H:%M"),
    })
    print(tab.to_string())
    print(f"\nTotal: {len(df)} ensayos, {df.ok.sum()} vueltas completas ({df.ok.mean():.0%}), "
          f"fallos: {df.motivo.value_counts().to_dict()}")

    fig, axs = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    ok, ko = df[df.ok == 1], df[df.ok == 0]
    ax = axs[0]
    ax.scatter(ok.orden, ok.tiempo_s, s=6, c=AZUL, alpha=0.5, label="vuelta completa")
    best = ok.set_index("orden").tiempo_s.cummin()
    ax.step(best.index, best.values, where="post", c=NEGRO, lw=1.5, label="mejor tiempo hasta el momento")
    ax.set_ylabel("tiempo de vuelta (s)")
    ax.set_ylim(4.8, 30)
    ax.legend(loc="center right")
    ax = axs[1]
    roll = df.ok.rolling(60, min_periods=20).mean() * 100
    ax.plot(df.orden, roll, c=VERDE)
    ax.set_ylabel("% completas\n(ventana 60)")
    ax.set_ylim(0, 105)
    ax = axs[2]
    ax.scatter(df.orden, df.v_max, s=5, c=np.where(df.ok == 1, AZUL, ROJO), alpha=0.5)
    ax.set_ylabel("v_max propuesta")
    ax = axs[3]
    vm = ok.set_index("orden").v_media.rolling(40, min_periods=10).median()
    vx = df.set_index("orden").v_max.rolling(40, min_periods=10).median()
    ax.plot(vx.index, vx.values, c=GRIS, label="v_max (mediana móvil)")
    ax.plot(vm.index, vm.values, c=AZUL, label="v media de las vueltas completas")
    ax.set_ylabel("m/s")
    ax.set_xlabel("ensayo (orden cronológico, todos los estudios)")
    ax.legend(loc="upper left")
    for a in axs:
        for est, x0 in df.groupby("estudio", sort=False).orden.min().items():
            if est in MARCAS[1:]:
                a.axvline(x0, c=GRIS, lw=0.8, ls="--")
    for est, x0 in df.groupby("estudio", sort=False).orden.min().items():
        if est in MARCAS:
            axs[0].text(x0 + 5, 29, est.replace("simple_", ""), fontsize=8, color="#4b5563", va="top")
    fig.suptitle("Evolución: más rápido, pero cada vez menos vueltas completas")
    fig.tight_layout()
    fig.savefig(FIG / "01_evolucion.png")
    plt.close(fig)
    return tab


# ---------------------------------------------------------------- 2. ruido
def ruido(df):
    titulo("2. REPETIBILIDAD (mismos parámetros, varias vueltas)")
    g = df.groupby("clave")
    rep = g.filter(lambda x: len(x) >= 2)
    grupos = rep.groupby("clave").agg(n=("ok", "size"), ok=("ok", "sum"), v_max=("v_max", "first"),
                                      t_med=("tiempo_s", lambda s: s[rep.loc[s.index, "ok"] == 1].mean()),
                                      t_std=("tiempo_s", lambda s: s[rep.loc[s.index, "ok"] == 1].std()))
    print(f"Grupos con ≥2 vueltas: {len(grupos)} (en total {grupos.n.sum()} vueltas)")
    std = grupos.t_std.dropna()
    print(f"Desviación del tiempo entre repeticiones que completan: mediana {std.median():.3f} s, "
          f"p90 {std.quantile(.9):.3f} s  (n={len(std)})")
    # fiabilidad: de las que completaron la primera vez, ¿cuántas repiten?
    primeras = rep.sort_values("orden").groupby("clave").head(1)
    resto = rep.drop(primeras.index)
    bien = primeras[primeras.ok == 1].clave
    r = resto[resto.clave.isin(bien)]
    print(f"Si la 1.ª vuelta completó, las repeticiones completan el {r.ok.mean():.0%} de las veces "
          f"({r.ok.sum()}/{len(r)}).")
    for lo, hi in [(0, 45), (45, 52), (52, 56), (56, 70), (70, 84), (84, 92), (92, 105)]:
        s = r[(r.v_max >= lo) & (r.v_max < hi)]
        if len(s):
            print(f"   v_max {lo}-{hi}: {s.ok.mean():.0%} ({s.ok.sum()}/{len(s)})")
    grupos["fiab"] = grupos.ok / grupos.n
    top = grupos[grupos.n >= 3].sort_values(["fiab", "t_med"], ascending=[False, True]).head(12)
    print("\nGrupos con ≥3 vueltas (mejores):")
    print(top[["n", "ok", "fiab", "v_max", "t_med", "t_std"]].round(3).to_string(index=False))
    return {"std_mediana": float(std.median()), "p_repite": float(r.ok.mean())}


# ---------------------------------------------------------------- 3. importancia
def _rf_importancia(X, y, clasif, semilla=0):
    if clasif:
        m = RandomForestClassifier(n_estimators=400, min_samples_leaf=3, random_state=semilla, n_jobs=-1)
        cv = StratifiedKFold(5, shuffle=True, random_state=semilla)
        pred = cross_val_predict(m, X, y, cv=cv, method="predict_proba")[:, 1]
        calidad = roc_auc_score(y, pred)
        esc = "roc_auc"
    else:
        m = RandomForestRegressor(n_estimators=400, min_samples_leaf=3, random_state=semilla, n_jobs=-1)
        cv = KFold(5, shuffle=True, random_state=semilla)
        pred = cross_val_predict(m, X, y, cv=cv)
        calidad = r2_score(y, pred)
        esc = "r2"
    m.fit(X, y)
    pi = permutation_importance(m, X, y, n_repeats=15, random_state=semilla, scoring=esc, n_jobs=-1)
    imp = pd.Series(pi.importances_mean, index=X.columns).clip(lower=0)
    return m, imp, calidad


CALIDAD = {}  # (régimen, métrica) → AUC o R² fuera de muestra del modelo de importancia


def importancia(df, reg):
    titulo(f"3. QUÉ PARÁMETROS TIENEN MÁS IMPACTO · {reg['nombre']}")
    # Un régimen cada vez (mismo espacio de parámetros) para comparar manzanas con manzanas
    d = del_regimen(df, reg)
    suf, vr = reg["sufijo"], reg["rapido"]
    rap = d[d.v_max >= vr]
    objetivos = {
        "P(completar)": (d, "ok", True),
        "tiempo (completas)": (d[d.ok == 1], "tiempo_s", False),
        "oscilación w±/s": (d[d.ok == 1], "cambios_w_s", False),
        f"P(completar) | v_max≥{vr}": (rap, "ok", True),
        f"tiempo | v_max≥{vr}": (rap[rap.ok == 1], "tiempo_s", False),
    }
    res, modelos, rho = {}, {}, {}
    for nombre, (sub, col, clas) in objetivos.items():
        X, y = sub[PARAMS], sub[col]
        m, imp, cal = _rf_importancia(X, y, clas)
        res[nombre] = imp / imp.sum()
        CALIDAD[(reg["nombre"], nombre)] = cal
        modelos[nombre] = (m, sub)
        rho[nombre] = pd.Series({p: spearmanr(sub[p], y).statistic for p in PARAMS})
        met = "AUC" if clas else "R²"
        print(f"\n{nombre}: n={len(sub)}, {met} fuera de muestra = {cal:.3f}")
        tab = pd.DataFrame({"importancia": res[nombre], "spearman": rho[nombre]}).sort_values("importancia",
                                                                                                ascending=False)
        print(tab.round(3).to_string())
    imp = pd.DataFrame(res)
    r = pd.DataFrame(rho)

    # Figura: mapa de calor de importancias + signo
    orden = imp.mean(axis=1).sort_values(ascending=False).index
    fig, axs = plt.subplots(1, 2, figsize=(13, 5.2), gridspec_kw={"width_ratios": [1, 1]})
    for ax, M, cmap, t, vmin, vmax in [(axs[0], imp.loc[orden], "Blues", "Importancia (permutación, suma 1)", 0, None),
                                       (axs[1], r.loc[orden], "RdBu_r", "Correlación de Spearman (signo)", -0.6, 0.6)]:
        im = ax.imshow(M.values, cmap=cmap, aspect="auto", vmin=vmin, vmax=vmax)
        ax.set_yticks(range(len(M)), M.index)
        ax.set_xticks(range(M.shape[1]), [c.replace(" | ", "\n") for c in M.columns], rotation=25, ha="right",
                      fontsize=8)
        ax.grid(False)
        for i in range(M.shape[0]):
            for j in range(M.shape[1]):
                ax.text(j, i, f"{M.values[i, j]:.2f}", ha="center", va="center", fontsize=7.5,
                        color="white" if abs(M.values[i, j]) > (0.25 if vmin == 0 else 0.4) else "black")
        ax.set_title(t, fontsize=10)
        fig.colorbar(im, ax=ax, shrink=0.8)
    fig.suptitle(f"Impacto de cada parámetro según la métrica ({reg['nombre']}, todas las generaciones)")
    fig.tight_layout()
    fig.savefig(FIG / f"02_importancia{suf}.png")
    plt.close(fig)

    # Dependencia parcial 1D: P(completar) y tiempo frente a cada parámetro
    fig, axs = plt.subplots(3, 4, figsize=(15, 9))
    mok, sok = modelos["P(completar)"]
    mt, st = modelos["tiempo (completas)"]
    # Misma escala de tiempo en todos (la que cubre todas las curvas): un efecto pequeño se ve pequeño
    curvas_t = {p: partial_dependence(mt, st[PARAMS], [p], grid_resolution=30, percentiles=(0.02, 0.98))
                for p in PARAMS}
    t_lo = min(c["average"][0].min() for c in curvas_t.values())
    t_hi = max(c["average"][0].max() for c in curvas_t.values())
    t_lo, t_hi = t_lo - 0.05 * (t_hi - t_lo), t_hi + 0.05 * (t_hi - t_lo)
    mejor = d.loc[d[d.ok == 1].tiempo_s.idxmin()]
    for ax, p in zip(axs.flat, PARAMS):
        pdp = partial_dependence(mok, sok[PARAMS], [p], grid_resolution=30, percentiles=(0.02, 0.98))
        x, yv = pdp["grid_values"][0], pdp["average"][0]
        ax.plot(x, yv * 100, c=VERDE, lw=2)
        ax.set_ylabel("% completas", color=VERDE, fontsize=8)
        ax.set_ylim(0, 100)
        ax2 = ax.twinx()
        pdt = curvas_t[p]
        ax2.plot(pdt["grid_values"][0], pdt["average"][0], c=AZUL, lw=2)
        ax2.set_ylabel("tiempo (s)", color=AZUL, fontsize=8)
        ax2.set_ylim(t_lo, t_hi)
        ax2.grid(False)
        ax.set_title(p)
        ax.axvline(mejor[p], c=NEGRO, ls=":", lw=1)
    axs.flat[-1].axis("off")
    axs.flat[-1].text(0.05, 0.5, "verde: probabilidad de completar\nazul: tiempo de las vueltas completas\n"
                      "punteado: vuelta más rápida\n\n(dependencia parcial de un bosque\naleatorio: efecto medio del\n"
                      "parámetro dejando el resto\ncomo en los datos)", fontsize=9, va="center")
    fig.suptitle(f"Efecto medio de cada parámetro sobre la fiabilidad y sobre el tiempo ({reg['nombre']})")
    fig.tight_layout()
    fig.savefig(FIG / f"03_dependencia_parcial{suf}.png")
    plt.close(fig)

    # Interacciones 2D sobre P(completar)
    pares = [("v_max", "k_v_curva"), ("v_max", "v_min"), ("kp", "td"), ("t_mirar", "kp"),
             ("k_ff", "k_hor"), ("v_max", "t_mirar")]
    fig, axs = plt.subplots(2, 3, figsize=(14, 8.5))
    for ax, (a, b) in zip(axs.flat, pares):
        pdp = partial_dependence(mok, sok[PARAMS], [a, b], grid_resolution=25, percentiles=(0.03, 0.97))
        xa, xb = pdp["grid_values"]
        Z = pdp["average"][0].T * 100
        cs = ax.contourf(xa, xb, Z, levels=12, cmap="RdYlGn", vmin=0, vmax=100)
        fig.colorbar(cs, ax=ax, shrink=0.85, label="% completas")
        bueno = sok[(sok.ok == 1)]
        ax.scatter(bueno[a], bueno[b], s=3, c="k", alpha=0.25)
        ax.set_xlabel(a)
        ax.set_ylabel(b)
        ax.set_xlim(xa.min(), xa.max())
        ax.set_ylim(xb.min(), xb.max())
        ax.grid(False)
    fig.suptitle(f"Interacciones: probabilidad de completar según pares de parámetros ({reg['nombre']}; "
                 "puntos = vueltas completas)")
    fig.tight_layout()
    fig.savefig(FIG / f"04_interacciones{suf}.png")
    plt.close(fig)

    # Importancia con magnitudes físicas en lugar de los parámetros crudos
    print(f"\nCon magnitudes físicas derivadas ({reg['nombre']}):")
    fis = ["v_max", "v_curva", "v_min", "L_max", "kp_ef", "ganancia_lazo", "td", "tau_d", "k_ff", "k_hor"]
    for nombre, sub, col, clas in [("P(completar)", d, "ok", True), ("tiempo", d[d.ok == 1], "tiempo_s", False)]:
        _, imp2, cal = _rf_importancia(sub[fis], sub[col], clas)
        imp2 = (imp2 / imp2.sum()).sort_values(ascending=False)
        print(f"  {nombre} ({'AUC' if clas else 'R²'} {cal:.3f}): "
              + ", ".join(f"{k} {v:.2f}" for k, v in imp2.items()))
    return imp, r


# ---------------------------------------------------------------- 4. frontera
def frontera(df, reg):
    titulo(f"4. COMPROMISO VELOCIDAD ↔ FIABILIDAD · {reg['nombre']}")
    d = del_regimen(df, reg)
    bins = reg["tramos"]
    d["tramo"] = pd.cut(d.v_max, bins)
    t = d.groupby("tramo", observed=True).agg(
        n=("ok", "size"), completas=("ok", "mean"),
        t_med=("tiempo_s", lambda s: s[d.loc[s.index, "ok"] == 1].median()),
        t_min=("tiempo_s", lambda s: s[d.loc[s.index, "ok"] == 1].min()),
        v_media=("v_media", "median"),
        dist_fallo=("distancia_m", lambda s: s[d.loc[s.index, "ok"] == 0].median()))
    t["completas"] = (t.completas * 100).round(0)
    # tiempo esperado para conseguir UNA vuelta completa si se reintenta tras cada fallo
    # (el fallo cuesta lo recorrido hasta chocar): t_ok + (1-p)/p · t_fallo
    t["t_fallo"] = d[d.ok == 0].groupby("tramo", observed=True).tiempo_s.median()
    p = t.completas / 100
    t["t_esperado_1_vuelta"] = t.t_med + (1 - p) / p.clip(lower=0.01) * t.t_fallo
    t["eficiencia_v"] = (t.v_media / d.groupby("tramo", observed=True).v_max.median()).round(2)
    print(t.round(2).to_string())
    print("\neficiencia_v = v media real / v_max: cuánto frena el perfil de velocidad.")

    ok = d[d.ok == 1]
    ko = d[d.ok == 0]
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
    ax = axs[0]
    ax.scatter(ok.v_max, ok.tiempo_s, s=6, c=AZUL, alpha=0.5)
    vv = np.linspace(max(ok.v_max.min(), 20), ok.v_max.max(), 50)
    ax.plot(vv, 430 / vv, c=GRIS, ls="--", label="430 m / v_max (sin frenar nunca)")
    ax.set_xlabel("v_max")
    ax.set_ylabel("tiempo (s)")
    ax.set_ylim(430 / ok.v_max.max() - 0.3, ok.tiempo_s.quantile(0.9))
    ax.legend()
    ax.set_title("Tiempo de las vueltas completas")
    ax = axs[1]
    ax.bar(range(len(t)), t.completas, color=VERDE)
    ax.set_xticks(range(len(t)), [f"{i.left:.0f}-{i.right:.0f}" for i in t.index], rotation=45, fontsize=8)
    for i, (n, c) in enumerate(zip(t.n, t.completas)):
        ax.text(i, c + 1, f"n={n}", ha="center", fontsize=7)
    ax.set_ylabel("% vueltas completas")
    ax.set_xlabel("v_max")
    ax.set_title("Fiabilidad por velocidad")
    ax = axs[2]
    ax.hist(ko.distancia_m, bins=60, color=ROJO, alpha=0.8)
    ax.set_xlabel("distancia recorrida al fallar (m)")
    ax.set_ylabel("fallos")
    ax.set_title("Dónde fallan (distancia desde la salida)")
    fig.suptitle(reg["nombre"])
    fig.tight_layout()
    fig.savefig(FIG / f"05_frontera{reg['sufijo']}.png")
    plt.close(fig)
    h, e = np.histogram(ko.distancia_m, bins=np.arange(0, 440, 10))
    print("\nFallos por tramo de 10 m (los 6 tramos con más):")
    for i in np.argsort(h)[::-1][:6]:
        print(f"   {e[i]:.0f}-{e[i+1]:.0f} m: {h[i]} fallos ({h[i]/len(ko):.0%})")
    return t


# ---------------------------------------------------------------- 5. tendencia
def tendencia(df):
    titulo("5. TENDENCIA DEL PID: CÓMO CAMBIAN LOS BUENOS PARÁMETROS AL IR MÁS RÁPIDO")
    ok = df[(df.ok == 1)].copy()
    # «buenos» = el 15 % más rápido dentro de cada tramo de v_max y de cada régimen de w_max
    # (w_max = 6 hasta simple_v3, 12 en simple_v4): así no se mezclan comportamientos distintos
    ok["tramo"] = pd.cut(ok.v_max, [0, 20, 25, 30, 35, 40, 44, 48, 51, 54, 57, 62, 66, 70, 73, 75, 77, 80, 84, 88, 91,
                                       94, 97, 105])
    corte = ok.groupby(["w_max", "tramo"], observed=True).tiempo_s.transform(lambda s: s.quantile(0.15))
    buenos = ok[ok.tiempo_s <= corte]
    cols = PARAMS[1:] + ["L_max", "kp_ef", "ganancia_lazo", "tau_d"]
    tablas, pend = {}, {}
    for wm, b in buenos.groupby("w_max"):
        t = b.groupby("tramo", observed=True)[["v_max"] + cols].median()
        t["n"] = b.groupby("tramo", observed=True).size()
        tablas[wm] = t
        print(f"\nw_max = {wm:g}:")
        with pd.option_context("display.width", 220):
            print(t.round(4).to_string())
        pend[wm] = {c: spearmanr(b.v_max, b[c]).statistic for c in cols}
        print(f"Spearman con v_max (w_max = {wm:g}): "
              + ", ".join(f"{k} {v:+.2f}" for k, v in sorted(pend[wm].items(), key=lambda kv: -abs(kv[1]))))

    colores = {6.0: AZUL, 12.0: "#f59e0b", 14.0: VERDE}
    lineas = {6.0: NEGRO, 12.0: "#92400e", 14.0: "#065f46"}
    fig, axs = plt.subplots(3, 5, figsize=(17, 9.5))
    for ax, c in zip(axs.flat, cols):
        ax.scatter(ok.v_max, ok[c], s=3, c=GRIS, alpha=0.2)
        for wm, t in tablas.items():
            b = buenos[buenos.w_max == wm]
            col = colores.get(wm, VERDE)
            ax.scatter(b.v_max, b[c], s=8, c=col, alpha=0.5)
            ax.plot(t.v_max, t[c], c=lineas.get(wm, NEGRO), lw=2, marker="o", ms=3)
        ax.set_title(c, fontsize=10)
        if c == "k_v_curva":
            ax.set_ylim(-0.5, 14)
        ax.set_xlabel("v_max", fontsize=8)
    axs.flat[-1].axis("off")
    axs.flat[-1].text(0.0, 0.5, "gris: vueltas completas\nazul: 15 % más rápido, w_max = 6\n(simple_v1–v3)\n"
                      "naranja: 15 % más rápido, w_max = 12\n(simple_v4–v5)\nverde: 15 % más rápido, w_max = 14\n(simple_v6–v8)\n"
                      "líneas: medianas por tramo",
                      fontsize=10, va="center")
    fig.suptitle("Tendencia: hacia dónde se mueven los parámetros buenos al subir la velocidad")
    fig.tight_layout()
    fig.savefig(FIG / "06_tendencia.png")
    plt.close(fig)
    return tablas, pend


def barridos():
    titulo("8. BARRIDOS (un parámetro, resto fijo)")
    rutas = sorted(OPT.glob("barrido_*.csv"), key=lambda r: r.stem.rsplit("_", 1)[1])  # por fecha
    rutas = [r for r in rutas if len(pd.read_csv(r))]  # un barrido recién lanzado aún no tiene vueltas
    if not rutas:
        print("No hay barridos.")
        return
    ncol = min(len(rutas), 4)
    nfil = -(-len(rutas) // ncol)
    fig, axs = plt.subplots(nfil, ncol, figsize=(6.5 * ncol, 4.5 * nfil), squeeze=False)
    for ax in axs.flat[len(rutas):]:
        ax.axis("off")
    for ax, ruta in zip(axs.flat, rutas):
        b = pd.read_csv(ruta)
        param = ruta.stem.split("_", 1)[1].rsplit("_", 1)[0]
        g = b.groupby("valor").agg(n=("completada", "size"), ok=("completada", "mean"),
                                   t_med=("tiempo_s", lambda s: s[b.loc[s.index, "completada"] == 1].median()),
                                   t_min=("tiempo_s", lambda s: s[b.loc[s.index, "completada"] == 1].min()))
        print(f"\n{ruta.name} ({param}):")
        print(g.round(3).to_string())
        x = np.arange(len(g))
        ax.bar(x, g.ok * 100, color=VERDE, alpha=0.8)
        for i, n in enumerate(g.n):
            ax.text(i, 2, f"n={n}", ha="center", fontsize=7)
        ax.set_xticks(x, [f"{v:g}" for v in g.index])
        ax.set_xlabel(param)
        ax.set_ylabel("% vueltas completas", color=VERDE)
        ax.set_ylim(0, 105)
        ax2 = ax.twinx()
        ax2.grid(False)
        ax2.plot(x, g.t_med, c=AZUL, marker="o", label="mediana")
        ax2.plot(x, g.t_min, c=AZUL, ls=":", marker=".", label="mínimo")
        ax2.set_ylabel("tiempo de las completas (s)", color=AZUL)
        ax2.legend(loc="upper right", fontsize=8)
        fijo = (f"w_max = {b.w_max.iloc[0]:g}, k_hor = {b.k_hor.iloc[0]:.3g}" if param == "v_max"
                else f"v_max = {b.v_max.iloc[0]:.1f}")
        hora = ruta.stem.rsplit("_", 1)[1]
        ax.set_title(f"{param} con {fijo}  ({hora[6:8]}/{hora[4:6]} {hora[9:11]}:{hora[11:13]})", fontsize=10)
    fig.suptitle("Barridos: fiabilidad y tiempo según el valor del parámetro")
    fig.tight_layout()
    fig.savefig(FIG / "09_barridos.png")
    plt.close(fig)


# ---------------------------------------------------------------- 6. telemetría
def telemetria(df):
    titulo("6. TELEMETRÍA: QUÉ TÉRMINO DEL CONTROL HACE EL TRABAJO")
    filas, choques, tray, t_tray = [], [], None, 1e9
    for f in sorted(glob.glob(str(RAIZ / "logs" / "2026110*_panel.csv"))):
        fv = f.replace(".csv", "_vueltas.csv")
        if not Path(fv).exists():
            continue
        try:
            t = pd.read_csv(f, low_memory=False)
            v = pd.read_csv(fv)
        except Exception:
            continue
        if "hor" not in t or "x" not in t or t.v_max.max() < 25:
            continue
        for _, lap in v.iterrows():
            s = t[t.vuelta == lap.vuelta]
            if len(s) < 10:
                continue
            if lap.completada and lap.distancia_m < 0.8 * 430:
                continue  # vuelta falsa del cronómetro (teletransporte o meta mal fijada, ESTUDIO_PID §17)
            fila = {"fichero": Path(f).name[:15], "vuelta": lap.vuelta, "ok": int(lap.completada),
                    "v_max": lap.v_max, "w_max": lap.w_max, "tiempo_s": lap.tiempo_s,
                    "distancia": lap.distancia_m}
            for k in ["p", "d", "ff", "hor"]:
                fila[f"|{k}|"] = s[k].abs().mean()
                fila[f"cuota_{k}"] = s[k].abs().sum() / (s[["p", "d", "ff", "hor"]].abs().sum().sum() + 1e-9)
            fila["sat_w"] = (s.w.abs() >= s.w_max.iloc[0] - 1e-3).mean()
            fila["e_cerca_perdido"] = (s.e.abs() >= 300).mean()
            fila["e_ctl_rms"] = np.sqrt((s.e_ctl.dropna() ** 2).mean())
            fila["v_min_vuelta"] = s.v.min()
            fila["kappa_lejos_p95"] = s.kappa_lejos.abs().quantile(0.95)
            fila["dt_med"] = s.dt.median()
            fila["dt_p99"] = s.dt.quantile(0.99)
            filas.append(fila)
            if not lap.completada:
                u = s.iloc[-1]
                choques.append({"x": u.x, "y": u.y, "v": u.v, "v_max": lap.v_max, "w": u.w,
                                "w_max": lap.w_max, "motivo": lap.motivo, "d": lap.distancia_m})
            elif lap.tiempo_s < t_tray:  # la vuelta completa más rápida de todas
                t_tray = lap.tiempo_s
                tray = s[["x", "y", "v", "kappa_lejos", "w", "p", "d", "ff", "hor", "e_ctl"]].copy()
            for zona, m in [("sur", (s.x > 33) & (s.y < -25) & (s.y > -56)),
                            ("norte", (s.x > 15) & (s.x < 46) & (s.y > 25))]:
                if m.any():
                    fila[f"sat_{zona}"] = (s.w[m].abs() >= s.w_max.iloc[0] - 1e-3).mean()
                    fila[f"muere_{zona}"] = int((not lap.completada) and bool(m.iloc[-1]))
    T = pd.DataFrame(filas)
    C = pd.DataFrame(choques)
    for wm, Tw in T[T.v_max >= 45].groupby("w_max"):
        if len(Tw) < 20:
            continue
        for zona in ["sur", "norte"]:
            z = Tw.dropna(subset=[f"sat_{zona}"])
            g = z.groupby(f"muere_{zona}")[f"sat_{zona}"].median()
            print(f"w_max {wm:g} · chicane {zona} (v_max ≥ 45): {int(z[f'muere_{zona}'].sum())} fallos de {len(z)} "
                  f"pasos; tiempo con w saturada: {g.get(0, np.nan):.0%} si pasa, {g.get(1, np.nan):.0%} si choca")
    print(f"Vueltas con telemetría (v_max ≥ 25): {len(T)} ({T.ok.sum()} completas, {len(C)} fallidas)")
    T["tramo"] = pd.cut(T.v_max, [25, 35, 45, 50, 54, 57, 62, 70, 76, 84, 90, 96, 105])
    cols = ["cuota_p", "cuota_d", "cuota_ff", "cuota_hor", "sat_w", "e_cerca_perdido", "e_ctl_rms",
            "v_min_vuelta", "dt_med", "dt_p99"]
    print("\nVueltas completas, por tramo de v_max (cuota = parte de Σ|término| que aporta cada uno):")
    print(T[T.ok == 1].groupby("tramo", observed=True)[cols].median().round(3).to_string())
    print("\nCompletas frente a fallidas con v_max ≥ 54 (mediana; las fallidas hasta el fallo):")
    print(T[T.v_max >= 54].groupby(["w_max", "ok"])[cols].median().round(3).to_string())

    fig, axs = plt.subplots(1, 3, figsize=(17, 5.5))
    ax = axs[0]
    g = T[T.ok == 1].groupby("tramo", observed=True)[["cuota_p", "cuota_d", "cuota_ff", "cuota_hor"]].median()
    g = g.div(g.sum(axis=1), axis=0)
    bot = np.zeros(len(g))
    for c, col in zip(g.columns, [AZUL, "#7c3aed", VERDE, "#f59e0b"]):
        ax.bar(range(len(g)), g[c], bottom=bot, color=col, label=c.replace("cuota_", "").upper())
        bot += g[c].values
    ax.set_xticks(range(len(g)), [f"{i.left:.0f}-{i.right:.0f}" for i in g.index])
    ax.set_xlabel("v_max")
    ax.set_ylabel("parte del esfuerzo de giro Σ|término|")
    ax.legend()
    ax.set_title("Quién gira el coche")
    if tray is not None:
        ax = axs[1]
        sc = ax.scatter(tray.x, tray.y, c=tray.v, s=4, cmap="viridis")
        fig.colorbar(sc, ax=ax, label=f"v (m/s) en la vuelta más rápida ({t_tray:.2f} s)")
        if len(C):
            for mot, mk in [("choque", "x"), ("sin_linea", "^")]:
                cc = C[C.motivo == mot]
                ax.scatter(cc.x, cc.y, marker=mk, s=18, c=ROJO, alpha=0.6, label=f"{mot} ({len(cc)})")
            ax.legend(loc="best", fontsize=8)
        ax.set_aspect("equal")
        ax.set_title("Dónde fallan las vueltas (v_max ≥ 25)")
        ax = axs[2]
        dist = np.r_[0, np.cumsum(np.hypot(np.diff(tray.x), np.diff(tray.y)))]
        for c, col in zip(["p", "d", "ff", "hor"], [AZUL, "#7c3aed", VERDE, "#f59e0b"]):
            ax.plot(dist, tray[c], c=col, lw=1, label=c.upper())
        ax.set_xlabel("distancia en la vuelta (m)")
        ax.set_ylabel("aporte a w (rad/s)")
        ax.set_ylim(-13, 13)
        ax.legend(ncol=4, fontsize=8)
        ax.set_title(f"Términos del control en la vuelta más rápida ({t_tray:.2f} s)")
    fig.tight_layout()
    fig.savefig(FIG / "07_telemetria.png")
    plt.close(fig)
    return T, C


def palancas(df, reg):
    vr = reg["palancas_desde"]
    titulo(f"7. LAS DOS PALANCAS DE LA FIABILIDAD EN RÉGIMEN RÁPIDO (v_max ≥ {vr}) · {reg['nombre']}")
    f = del_regimen(df, reg)
    f = f[f.v_max >= vr]
    tabs = {}
    for col, cortes in [("k_v_curva", reg["cortes_kvc"]), ("k_hor", reg["cortes_khor"])]:
        f["b"] = pd.cut(f[col], cortes)
        t = f.groupby("b", observed=True).agg(
            n=("ok", "size"), completas=("ok", "mean"),
            t_med=("tiempo_s", lambda s: s[f.loc[s.index, "ok"] == 1].median()),
            osc=("cambios_w_s", lambda s: s[f.loc[s.index, "ok"] == 1].median()))
        print(f"\n{col}:")
        print(t.round(2).to_string())
        tabs[col] = t
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.5))
    for ax, (col, t) in zip(axs, tabs.items()):
        x = np.arange(len(t))
        ax.bar(x, t.completas * 100, color=VERDE, alpha=0.85)
        ax.set_xticks(x, [f"{i.left:g}-{i.right:g}" for i in t.index], rotation=30, fontsize=8)
        ax.set_ylabel("% vueltas completas", color=VERDE)
        ax.set_ylim(0, 100)
        ax.set_xlabel(col)
        ax2 = ax.twinx()
        ax2.grid(False)
        if col == "k_v_curva":
            ax2.plot(x, t.t_med, c=AZUL, marker="o")
            ax2.set_ylabel("tiempo mediano de las completas (s)", color=AZUL)
        else:
            ax2.plot(x, t.osc, c="#f59e0b", marker="o")
            ax2.set_ylabel("cambios de signo de w por s", color="#f59e0b")
        for i, n in enumerate(t.n):
            ax.text(i, 2, f"n={n}", ha="center", fontsize=7)
    axs[0].set_title("Cuánto frenar en curva: fiabilidad a cambio de tiempo")
    axs[1].set_title("Banda del horizonte: hay un punto óptimo")
    fig.suptitle(f"Régimen rápido (v_max ≥ {vr}) · {reg['nombre']}")
    fig.tight_layout()
    fig.savefig(FIG / f"08_palancas{reg['sufijo']}.png")
    plt.close(fig)


# ---------------------------------------------------------------- 9. decisión final
def wilson(k, n, z=1.96):
    """Cota inferior (95 %) de la tasa de vueltas completas: con pocas vueltas, 10/10 no es 100 %."""
    p = k / n
    return (p + z * z / (2 * n) - z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / (1 + z * z / n)


def decision():
    """Junta todas las vueltas (estudios y barridos) de cada combinación de parámetros y elige la de
    mayor fiabilidad demostrada (cota inferior de Wilson); a igualdad, la más rápida. Solo cuentan
    combinaciones con ≥ 20 vueltas: con menos, una racha de suerte las pondría arriba."""
    titulo("9. DECISIÓN: MEJOR COMBINACIÓN EN TIEMPO Y FIABILIDAD")
    cols = PARAMS + ["w_max"]
    trozos = []
    for ruta in sorted(OPT.glob("simple_v*_ensayos.csv")) + sorted(OPT.glob("barrido_*.csv")):
        if ruta.stem in ESTUDIOS_TANDAS:
            continue  # una fila es una tanda de vueltas seguidas, no una vuelta (sección 11)
        d = pd.read_csv(ruta)
        if "w_max" not in d or not len(d):
            continue
        d["k_hor"] = d["k_hor"].fillna(0.0) if "k_hor" in d else 0.0
        d["fuente"] = ruta.stem.replace("_ensayos", "")
        trozos.append(d[cols + ["completada", "tiempo_s", "e_rms", "cambios_w_s", "fuente"]])
    a = pd.concat(trozos, ignore_index=True)
    a["clave"] = a[cols].round(4).astype(str).agg("|".join, axis=1)
    g = a.groupby("clave").agg(n=("completada", "size"), ok=("completada", "sum"),
                               fuentes=("fuente", lambda s: ", ".join(sorted(set(s)))))
    g = g.join(a[a.completada == 1].groupby("clave").agg(t_med=("tiempo_s", "median"), e_rms=("e_rms", "median"),
                                                          cambios_w=("cambios_w_s", "median")))
    g = g.join(a.groupby("clave")[cols].first())
    g["tasa"], g["cota"] = g.ok / g.n, wilson(g.ok, g.n)
    G = g[g.n >= 20].sort_values(["cota", "t_med"], ascending=[False, True])
    print(f"{len(g)} combinaciones distintas; {len(G)} con ≥ 20 vueltas.")
    with pd.option_context("display.width", 250):
        print(G[["n", "ok", "tasa", "cota", "t_med", "e_rms", "cambios_w", "v_max", "w_max", "k_v_curva", "kp",
                 "td", "k_ff", "k_hor", "t_mirar"]].head(15).round(3).to_string(index=False))
    # Frontera: ninguna otra es a la vez más rápida y más fiable
    frontera, mejor = [], -1.0
    for i, r in G.sort_values("t_med").iterrows():
        if r.cota > mejor:
            frontera.append(i)
            mejor = r.cota
    elegida = G.index[0]
    e = G.loc[elegida]
    print(f"\nElegida: {e.ok:.0f}/{e.n:.0f} completas ({e.tasa:.0%}, cota {e.cota:.0%}), {e.t_med:.2f} s, "
          f"e_rms {e.e_rms:.0f} px\n" + "  ".join(f"{c}={e[c]:.5g}" for c in cols))
    poco = g[(g.n >= 5) & (g.n < 20) & (g.ok == g.n)].sort_values("e_rms").head(5)
    if len(poco):
        print("\nSin vueltas suficientes para decidir (todas completas, < 20 vueltas):")
        print(poco[["n", "ok", "cota", "t_med", "e_rms", "v_max", "w_max", "k_v_curva", "kp", "td", "k_ff",
                    "t_mirar", "fuentes"]].round(4).to_string(index=False))

    fig, ax = plt.subplots(figsize=(10, 6))
    color = G.w_max.map({6.0: AZUL, 12.0: "#f59e0b", 14.0: VERDE}).fillna(GRIS)
    ax.errorbar(G.t_med, G.tasa * 100, yerr=[(G.tasa - G.cota) * 100, np.zeros(len(G))], fmt="none",
                ecolor=GRIS, alpha=0.5, lw=1)
    ax.scatter(G.t_med, G.tasa * 100, s=G.n * 3, c=color, alpha=0.8, edgecolor="white", lw=0.5)
    F = G.loc[frontera].sort_values("t_med")
    ax.plot(F.t_med, F.cota * 100, c=NEGRO, lw=1, ls="--", label="frontera (cota inferior)")
    ax.scatter([e.t_med], [e.tasa * 100], s=e.n * 3 + 120, facecolor="none", edgecolor=ROJO, lw=2,
               label=f"elegida: v_max {e.v_max:g}, w_max {e.w_max:g} ({e.ok:.0f}/{e.n:.0f}, {e.t_med:.2f} s)")
    for wm, c in [(6.0, AZUL), (12.0, "#f59e0b"), (14.0, VERDE)]:
        ax.scatter([], [], c=c, s=40, label=f"w_max = {wm:g}")
    ax.set_xlabel("tiempo mediano de las vueltas completas (s)")
    ax.set_ylabel("% vueltas completas (barra: hasta la cota inferior del 95 %)")
    ax.set_title("Combinaciones con ≥ 20 vueltas: fiabilidad frente a tiempo (tamaño = vueltas)")
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(FIG / "10_decision.png")
    plt.close(fig)
    return e


# ---------------------------------------------------------------- 10. pesos por régimen
def pesos(imps):
    """Peso (importancia por permutación, suma 1) de cada parámetro en el tiempo, la fiabilidad y la
    oscilación, para los tres regímenes de w_max. Resume las figuras 02 en una sola."""
    titulo("10. PESO DE CADA PARÁMETRO POR RÉGIMEN")
    metricas = ["P(completar)", "tiempo (completas)", "oscilación w±/s"]
    regs = list(imps)
    tabla = pd.concat({m: pd.DataFrame({r: imps[r][m] for r in regs}) for m in metricas}, axis=1)
    with pd.option_context("display.width", 220):
        print(tabla.round(2).to_string())
    colores = [AZUL, "#f59e0b", VERDE]
    fig, axs = plt.subplots(1, 3, figsize=(16, 6.5), sharey=True)
    orden = tabla.max(axis=1).sort_values().index
    y = np.arange(len(orden))
    alto = 0.8 / len(regs)
    for ax, m in zip(axs, metricas):
        clas = m.startswith("P(")
        for i, (r, c) in enumerate(zip(regs, colores)):
            val = imps[r][m].reindex(orden).fillna(0)
            cal = CALIDAD.get((r, m), np.nan)
            # Un modelo que no predice (AUC < 0.65 o R² < 0.3) no da pesos de fiar: barra rayada y clara
            malo = cal < (0.65 if clas else 0.3)
            etiqueta = f"{r} · {'AUC' if clas else 'R²'} {cal:.2f}" + (" (no predice)" if malo else "")
            ax.barh(y + (i - (len(regs) - 1) / 2) * alto, val, height=alto, color=c, label=etiqueta,
                    alpha=0.35 if malo else 1.0, hatch="//" if malo else None, edgecolor=c if malo else None)
            for yy, vv in zip(y + (i - (len(regs) - 1) / 2) * alto, val):
                if vv >= 0.08:
                    ax.text(vv + 0.01, yy, f"{vv:.2f}", va="center", fontsize=7)
        ax.set_title(m)
        ax.legend(loc="lower right", fontsize=7)
        ax.set_xlim(0, 1.05)
        ax.set_xlabel("peso (importancia por permutación, suma 1)")
    axs[0].set_yticks(y, orden)
    fig.suptitle("Qué pesa en cada métrica: w_max = 6 (simple_v3), 12 (v4–v5) y 14 (v6–v8)")
    fig.tight_layout()
    fig.savefig(FIG / "11_pesos.png")
    plt.close(fig)
    return tabla


# ---------------------------------------------------------------- 11. tandas de vueltas seguidas
ESTUDIOS_TANDAS = ("simple_v9_ensayos",)  # una fila = una tanda de «vueltas_seguidas» vueltas sin reiniciar
LONG_VUELTA = 424.0  # m que mide el panel en una vuelta completa (421–426)


def _telemetria_tandas(desde):
    """Cada vuelta del panel desde `desde` (fecha del primer ensayo de tandas) con su posición en la
    tanda: 1 = desde parado (tras «reinicio: pedido»), 2+ = lanzadas. Los fallos llevan la posición
    (x, y) y la velocidad del último fotograma antes del choque."""
    filas = []
    for f in sorted(glob.glob(str(RAIZ / "logs" / "2026*_panel.csv"))):
        if Path(f).name[:15] < desde:
            continue
        fv = f.replace(".csv", "_vueltas.csv")
        if not Path(fv).exists():
            continue
        t = pd.read_csv(f, low_memory=False, usecols=["t", "vuelta", "marca", "x", "y", "v", "w", "w_max"])
        v = pd.read_csv(fv).set_index("vuelta")
        inicio = None  # número de vuelta del panel con que empezó la tanda en curso
        pedidos = t[t.marca == "reinicio: pedido"].vuelta.tolist()
        for n, lap in v.iterrows():
            previos = [p for p in pedidos if p <= n]
            if not previos:
                continue
            inicio = previos[-1]
            s = t[t.vuelta == n]
            if lap.completada and lap.distancia_m < 0.8 * 430:
                continue  # vuelta falsa del cronómetro
            fila = {"fichero": Path(f).name[:15], "vuelta": n, "pos": int(n - inicio + 1),
                    "ok": int(lap.completada), "motivo": lap.motivo, "tiempo_s": lap.tiempo_s,
                    "distancia_m": lap.distancia_m, "e_rms": lap.e_rms, "cambios_w_s": lap.cambios_w_s,
                    "v_max": lap.v_max, "sat_w": float((s.w.abs() >= s.w_max - 1e-3).mean()) if len(s) else np.nan,
                    "v_salida": float(s.v.iloc[:5].mean()) if len(s) else np.nan}
            if not lap.completada and len(s):
                u = s.dropna(subset=["x"]).iloc[-1] if s.x.notna().any() else None
                if u is not None:
                    fila.update(x=u.x, y=u.y, v_choque=u.v)
            filas.append(fila)
    return pd.DataFrame(filas)


def tandas():
    """simple_v9: cada ensayo son 5 vueltas seguidas con un solo reinicio al principio."""
    titulo("11. TANDAS DE 5 VUELTAS SEGUIDAS (simple_v9)")
    trozos = []
    for nombre in ESTUDIOS_TANDAS:
        ruta = OPT / f"{nombre}.csv"
        if ruta.exists():
            trozos.append(pd.read_csv(ruta))
    if not trozos:
        print("No hay estudios de tandas.")
        return None
    d = pd.concat(trozos, ignore_index=True)
    d["fecha"] = pd.to_datetime(d.fecha)
    d = d.sort_values("fecha").reset_index(drop=True)
    N = 5
    d["vueltas_ok"] = d.vueltas_ok.fillna(0).astype(int)
    d["ok"] = d.completada.astype(int)
    d["intentadas"] = d.vueltas_ok + (1 - d.ok)
    d["clave"] = d[PARAMS].round(5).astype(str).agg("|".join, axis=1)
    tiempos = d.tiempos.fillna("").astype(str).str.split().apply(lambda l: [float(x) for x in l])
    d["t_lanzada"] = tiempos.apply(lambda l: np.mean(l[1:]) if len(l) > 1 else np.nan)
    print(f"{len(d)} tandas ({d.fecha.min():%d %H:%M}–{d.fecha.max():%H:%M}), {d.vueltas_ok.sum()} vueltas completas "
          f"de {d.intentadas.sum()} intentadas ({d.vueltas_ok.sum() / d.intentadas.sum():.1%} por vuelta)")
    print(f"Tandas 5/5: {d.ok.sum()} ({d.ok.mean():.1%}). Vueltas completas por tanda: "
          f"{d.vueltas_ok.value_counts().sort_index().to_dict()}. Fallos: {d[d.ok == 0].motivo.value_counts().to_dict()}")
    # Riesgo por vuelta: P(fallar la k-ésima | llegó a empezarla)
    riesgo = []
    for k in range(1, N + 1):
        llegan = int((d.vueltas_ok >= k - 1).sum())
        fallan = int(((d.vueltas_ok == k - 1) & (d.ok == 0)).sum())
        riesgo.append({"vuelta": k, "llegan": llegan, "fallan": fallan, "p_fallo": fallan / llegan,
                       "lo": 1 - wilson(llegan - fallan, llegan, ) if llegan else np.nan})
    R = pd.DataFrame(riesgo)
    # Intervalo de Wilson para p_fallo (complemento de la cota de completar)
    R["hi"] = 1 - wilson(R.llegan - R.fallan, R.llegan)
    R["lo"] = wilson(R.fallan, R.llegan)
    print("\nRiesgo de fallar cada vuelta de la tanda (sobre las que llegan a empezarla):")
    print(R.round(3).to_string(index=False))
    L = pd.DataFrame(tiempos.tolist()).reindex(columns=range(N))
    print("\nTiempo medio por posición en la tanda (s): "
          + "  ".join(f"{k + 1}: {L[k].mean():.3f} (n={L[k].count()})" for k in range(N)))

    # Telemetría: dónde falla, desde parado frente a lanzada
    tel = _telemetria_tandas(d.fecha.min().strftime("%Y%m%d-%H%M%S"))
    if len(tel):
        tel = tel[tel.pos <= N]
        ko = tel[tel.ok == 0]
        print(f"\nTelemetría: {len(tel)} vueltas del panel ({len(ko)} fallidas)")
        for nombre, sub in [("1.ª (desde parado)", tel[tel.pos == 1]), ("lanzadas (2.ª–5.ª)", tel[tel.pos > 1])]:
            f = sub[sub.ok == 0]
            print(f"  {nombre}: {len(sub)} vueltas, fallan {len(f) / max(len(sub), 1):.1%}; "
                  f"e_rms {sub[sub.ok == 1].e_rms.median():.0f} px, giro saturado {sub.sat_w.median():.1%}, "
                  f"v al empezar {sub.v_salida.median():.1f} m/s")
            if len(f):
                h = (f.distancia_m // 10 * 10).value_counts(normalize=True).head(4)
                print("     dónde: " + ", ".join(f"{int(m)}–{int(m) + 10} m {p:.0%}" for m, p in h.items()))

    # La zona a la que convergió el optimizador: tandas cerca de la mejor combinación
    g = d.groupby("clave").agg(n=("ok", "size"), ok=("ok", "sum"), vueltas=("vueltas_ok", "sum"),
                               intentadas=("intentadas", "sum"), ensayo=("ensayo", "min"))
    g = g.join(d[d.ok == 1].groupby("clave").agg(t_med=("tiempo_s", "median"), t_lanz=("t_lanzada", "median"),
                                                 e_rms=("e_rms", "median"), cambios_w=("cambios_w_s", "median")))
    g = g.join(d.groupby("clave")[PARAMS].first())
    g["p_vuelta"] = g.vueltas / g.intentadas
    g["cota_vuelta"] = wilson(g.vueltas, g.intentadas)
    g["p_tanda"], g["cota_tanda"] = g.ok / g.n, wilson(g.ok, g.n)
    G = g[g.intentadas >= 15].sort_values(["cota_vuelta", "t_med"], ascending=[False, True])
    print(f"\n{len(g)} combinaciones; {len(G)} con ≥ 15 vueltas intentadas. Mejores por fiabilidad por vuelta:")
    with pd.option_context("display.width", 250):
        print(G[["ensayo", "n", "ok", "vueltas", "intentadas", "cota_vuelta", "t_med", "t_lanz", "e_rms", "cambios_w"]
                + PARAMS].head(12).round(4).to_string(index=False))
    e = G.iloc[0]
    # Vecindad: tandas a < 15 % del ancho de la ventana en todos los parámetros (salvo kv_kp). Con 5–10 %
    # solo entra la propia combinación; con 15 % la estimación ya junta ~25 tandas de la zona.
    anchos = d[PARAMS].quantile(0.98) - d[PARAMS].quantile(0.02)
    dist = ((d[PARAMS] - e[PARAMS]).abs() / anchos.replace(0, 1)).drop(columns=["kv_kp"]).max(axis=1)
    zona = d[dist < 0.15]
    zv, zi = zona.vueltas_ok.sum(), zona.intentadas.sum()
    print(f"\nElegida: ensayo #{e.ensayo:.0f}: {e.ok:.0f}/{e.n:.0f} tandas, {e.vueltas:.0f}/{e.intentadas:.0f} vueltas "
          f"(cota {e.cota_vuelta:.0%}), {e.t_med:.3f} s por vuelta en la tanda, {e.t_lanz:.3f} s lanzada, "
          f"e_rms {e.e_rms:.0f}\n" + "  ".join(f"{c}={e[c]:.5g}" for c in PARAMS))
    print(f"Su vecindad (±15 % de la ventana, {len(zona)} tandas de {zona.clave.nunique()} combinaciones): "
          f"{zona.ok.sum()}/{len(zona)} tandas 5/5 ({zona.ok.mean():.0%}), {zv}/{zi} vueltas "
          f"({zv / zi:.1%}, cota {wilson(zv, zi):.0%}); tiempo lanzada {zona.t_lanzada.median():.3f} s")
    print(f"Con la fiabilidad por vuelta de la vecindad, P(5/5) esperada = {(zv / zi) ** N:.0%}; "
          f"con la de w14_v93_v6 en una vuelta (63/65) sería {(63 / 65) ** N:.0%}.")
    base = d[(d.v_max.round(3) == 93.0) & (d.kp.round(5) == 0.01139)]
    if len(base):
        print(f"w14_v93_v6 en tandas: {base.ok.sum()}/{len(base)} tandas, {base.vueltas_ok.sum()}/{base.intentadas.sum()} vueltas")

    # Importancia sobre las tandas
    print("\nImportancia en las tandas (bosque aleatorio, permutación):")
    obj = {"P(5/5)": (d, "ok", True), "vueltas completas": (d, "vueltas_ok", False),
           "tiempo lanzada": (d[d.t_lanzada.notna()], "t_lanzada", False)}
    imps, cal, mods = {}, {}, {}
    for nombre, (sub, col, clas) in obj.items():
        m, imp, c = _rf_importancia(sub[PARAMS], sub[col], clas)
        imps[nombre], cal[nombre], mods[nombre] = imp / imp.sum(), c, (m, sub)
        rho = pd.Series({p: spearmanr(sub[p], sub[col]).statistic for p in PARAMS})
        print(f"  {nombre} (n={len(sub)}, {'AUC' if clas else 'R²'} {c:.3f}): "
              + ", ".join(f"{k} {v:.2f} ({rho[k]:+.2f})" for k, v in imps[nombre].sort_values(ascending=False)
                          .head(6).items()))
    I = pd.DataFrame(imps)

    # Figura 12: fiabilidad y tiempo en las tandas
    fig, axs = plt.subplots(2, 3, figsize=(16, 9.5))
    ax = axs[0, 0]
    x = np.arange(len(d))
    ax.plot(x, d.ok.rolling(40, min_periods=10).mean() * 100, c=VERDE, label="% tandas 5/5 (ventana 40)")
    ax.plot(x, (d.vueltas_ok.rolling(40, min_periods=10).sum() / d.intentadas.rolling(40, min_periods=10).sum())
            * 100, c=AZUL, label="% vueltas completas (ventana 40)")
    for i, (est, x0) in enumerate(d.groupby("estudio", sort=False).apply(lambda s: s.index.min()).items()):
        ax.axvline(x0, c=GRIS, lw=0.8, ls="--")
        ax.text(x0 + 3, 3 + 6 * (i % 2), est.replace("simple_", ""), fontsize=7, color="#4b5563")
    ax.set_ylim(0, 100)
    ax.set_xlabel("tanda (orden)")
    ax.set_title("Evolución del estudio")
    ax.legend(fontsize=8, loc="center right")
    ax = axs[0, 1]
    cuenta = d.vueltas_ok.value_counts().reindex(range(N + 1), fill_value=0)
    ax.bar(cuenta.index, cuenta.values, color=[ROJO] * N + [VERDE])
    for k, c in cuenta.items():
        ax.text(k, c, f"{c}", ha="center", va="bottom", fontsize=8)
    ax.set_xlabel("vueltas completas en la tanda")
    ax.set_ylabel("tandas")
    ax.set_title(f"{d.ok.mean():.0%} acaban las 5")
    ax = axs[0, 2]
    ax.errorbar(R.vuelta, R.p_fallo * 100, yerr=[(R.p_fallo - R.lo) * 100, (R.hi - R.p_fallo) * 100], fmt="o-",
                c=ROJO, capsize=4)
    for _, r in R.iterrows():
        ax.text(r.vuelta + 0.08, r.p_fallo * 100, f"{int(r.fallan)}/{int(r.llegan)}", fontsize=8)
    ax.set_xticks(range(1, N + 1), ["1.ª\n(parado)"] + [f"{k}.ª" for k in range(2, N + 1)])
    ax.set_ylim(0, None)
    ax.set_ylabel("% de fallo (sobre las que la empiezan)")
    ax.set_title("Riesgo de cada vuelta de la tanda (barra: IC 95 %)")
    ax = axs[1, 0]
    datos = [L[k].dropna() for k in range(N)]
    ax.boxplot(datos, showfliers=False)
    ax.set_xticks(range(1, N + 1), ["1.ª\n(parado)"] + [f"{k}.ª" for k in range(2, N + 1)])
    for k, s_ in enumerate(datos):
        ax.text(k + 1, s_.median(), f" {s_.median():.2f}", fontsize=8, va="bottom")
    ax.set_ylabel("tiempo de vuelta (s)")
    ax.set_title("Tiempo según la posición en la tanda (completas)")
    ax = axs[1, 1]
    if len(tel):
        bins = np.arange(0, 440, 10)
        for sub, c, lab in [(ko[ko.pos == 1], AZUL, "1.ª vuelta (desde parado)"),
                            (ko[ko.pos > 1], ROJO, "vueltas lanzadas")]:
            ax.hist(sub.distancia_m, bins=bins, color=c, alpha=0.6, label=f"{lab} ({len(sub)})")
        ax.legend(fontsize=8)
    ax.set_xlabel("distancia recorrida en la vuelta al fallar (m)")
    ax.set_ylabel("fallos")
    ax.set_title("Dónde falla")
    ax = axs[1, 2]
    if len(tel) and "x" in ko:
        tray = None
        for f in sorted(glob.glob(str(RAIZ / "logs" / "2026111*_panel.csv")))[-1:]:
            tt = pd.read_csv(f, low_memory=False, usecols=["x", "y", "vuelta"])
            buena = tel[(tel.fichero == Path(f).name[:15]) & (tel.ok == 1)]
            if len(buena):
                tray = tt[tt.vuelta == buena.vuelta.iloc[-1]]
        if tray is not None:
            ax.plot(tray.x, tray.y, c=GRIS, lw=1.5, label="vuelta completa")
        for sub, c, lab in [(ko[ko.pos == 1], AZUL, "fallo en la 1.ª"), (ko[ko.pos > 1], ROJO, "fallo lanzada")]:
            ax.scatter(sub.x, sub.y, s=14, c=c, alpha=0.6, label=lab, edgecolor="none")
        ax.set_aspect("equal")
        ax.legend(fontsize=8, loc="best")
    ax.set_title("Dónde choca (último fotograma)")
    fig.suptitle(f"simple_v9: {len(d)} tandas de 5 vueltas seguidas sin reiniciar")
    fig.tight_layout()
    fig.savefig(FIG / "12_tandas.png")
    plt.close(fig)

    # Figura 13: qué pesa en las tandas y efecto de los más importantes
    top = I["P(5/5)"].sort_values(ascending=False).index[:4].tolist()
    fig = plt.figure(figsize=(16, 8.5))
    gs = fig.add_gridspec(2, 4)
    ax = fig.add_subplot(gs[:, :2])
    orden = I.mean(axis=1).sort_values().index
    y = np.arange(len(orden))
    for i, (col, c) in enumerate(zip(I.columns, [VERDE, AZUL, "#f59e0b"])):
        clas = col.startswith("P(")
        malo = cal[col] < (0.65 if clas else 0.3)
        ax.barh(y + (i - 1) * 0.27, I[col].reindex(orden), height=0.27, color=c, alpha=0.35 if malo else 1,
                hatch="//" if malo else None, edgecolor=c if malo else None,
                label=f"{col} · {'AUC' if clas else 'R²'} {cal[col]:.2f}" + (" (no predice)" if malo else ""))
    ax.set_yticks(y, orden)
    ax.set_xlabel("peso (importancia por permutación, suma 1)")
    ax.legend(fontsize=8, loc="lower right")
    ax.set_title("Qué pesa en las tandas")
    m, sub = mods["P(5/5)"]
    for k, p in enumerate(top):
        ax = fig.add_subplot(gs[k // 2, 2 + k % 2])
        pdp = partial_dependence(m, sub[PARAMS], [p], grid_resolution=30, percentiles=(0.02, 0.98))
        ax.plot(pdp["grid_values"][0], pdp["average"][0] * 100, c=VERDE, lw=2)
        ax.scatter(sub[p], sub.ok * 100, s=4, c=np.where(sub.ok == 1, AZUL, ROJO), alpha=0.15)
        ax.axvline(e[p], c=NEGRO, ls=":", lw=1)
        ax.set_ylim(-3, 103)
        ax.set_title(p)
        ax.set_ylabel("% tandas 5/5", fontsize=8)
    fig.suptitle("Importancia en las tandas y efecto medio de los 4 parámetros que más pesan en acabar las 5 "
                 "(punteado: elegida)")
    fig.tight_layout()
    fig.savefig(FIG / "13_tandas_importancia.png")
    plt.close(fig)

    # Figura 14: decisión en tandas
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.errorbar(G.t_lanz, G.p_vuelta * 100, yerr=[(G.p_vuelta - G.cota_vuelta) * 100, np.zeros(len(G))], fmt="none",
                ecolor=GRIS, alpha=0.5, lw=1)
    sc = ax.scatter(G.t_lanz, G.p_vuelta * 100, s=G.intentadas * 6, c=G.v_max, cmap="viridis", alpha=0.85,
                    edgecolor="white", lw=0.5)
    fig.colorbar(sc, ax=ax, label="v_max")
    ax.scatter([e.t_lanz], [e.p_vuelta * 100], s=e.intentadas * 6 + 150, facecolor="none", edgecolor=ROJO, lw=2,
               label=f"elegida #{e.ensayo:.0f}: v_max {e.v_max:.2f} ({e.vueltas:.0f}/{e.intentadas:.0f} vueltas, "
                     f"{e.ok:.0f}/{e.n:.0f} tandas)")
    ax.axhline(zv / zi * 100, c=VERDE, ls="--", lw=1, label=f"su vecindad: {zv / zi:.0%} por vuelta ({len(zona)} tandas)")
    ax.axhline(63 / 65 * 100, c=NEGRO, ls=":", lw=1, label="w14_v93_v6 en vueltas sueltas (63/65)")
    ax.set_xlabel("tiempo mediano de vuelta lanzada (s)")
    ax.set_ylabel("% vueltas completas en tandas (barra: hasta la cota del 95 %)")
    ax.set_title("Combinaciones con ≥ 15 vueltas en tandas (tamaño = vueltas intentadas)")
    ax.legend(fontsize=8, loc="lower left")
    fig.tight_layout()
    fig.savefig(FIG / "14_tandas_decision.png")
    plt.close(fig)
    return e


def main():
    df = cargar()
    resumen = {}
    resumen["evolucion"] = evolucion(df)
    resumen["ruido"] = ruido(df)
    imps = {}
    for reg in REGIMENES:
        imps[reg["nombre"]] = importancia(df, reg)[0]
        frontera(df, reg)
    pesos(imps)
    tendencia(df)
    telemetria(df)
    for reg in REGIMENES:
        palancas(df, reg)
    barridos()
    decision()
    tandas()
    titulo("MEJORES VUELTAS INDIVIDUALES")
    print(df[df.ok == 1].nsmallest(10, "tiempo_s")[["estudio", "tiempo_s", "J", "e_rms", "cambios_w_s"] + PARAMS]
          .round(4).to_string(index=False))
    print(f"\nFiguras en {FIG.relative_to(RAIZ)}/")


if __name__ == "__main__":
    main()
