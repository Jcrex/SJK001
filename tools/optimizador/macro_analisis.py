"""Macroanálisis de todos los ensayos del optimizador y de la telemetría de las vueltas.

Responde a tres preguntas:
  1. ¿Cómo ha evolucionado la búsqueda? (tiempo, fiabilidad, generaciones)
  2. ¿Qué parámetros tienen más impacto? Por separado sobre el TIEMPO de las vueltas
     completas y sobre la PROBABILIDAD DE COMPLETAR, que no dependen de lo mismo.
  3. ¿Hacia dónde tiende el PID? Cómo se mueven los mejores parámetros al subir v_max
     y qué término del control (P, D, FF, HOR) hace el trabajo en la pista.

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


# Regímenes con el mismo espacio de parámetros: w_max = 6 (simple_v3) y w_max = 12 (simple_v4–v5).
# Cada uno tiene su corte de «rápido», sus tramos de v_max y su sufijo en los nombres de las figuras.
REGIMENES = [
    {"nombre": "simple_v3 (w_max = 6)", "prefijos": ("simple_v3",), "sufijo": "", "rapido": 48,
     "tramos": [0, 30, 35, 40, 44, 48, 51, 54, 56, 58, 60, 70], "palancas_desde": 54,
     "cortes_kvc": [0, 3, 4, 5, 6, 7, 8, 10, 15], "cortes_khor": [0, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.12, 1]},
    {"nombre": "simple_v4–v5 (w_max = 12)", "prefijos": ("simple_v4", "simple_v5"), "sufijo": "_w12", "rapido": 90,
     "tramos": [55, 65, 70, 74, 78, 84, 88, 90, 92, 94, 96, 98, 105], "palancas_desde": 90,
     "cortes_kvc": [1.5, 2.2, 2.4, 2.6, 2.8, 3.0, 3.5, 6, 11], "cortes_khor": [0.02, 0.035, 0.04, 0.045, 0.05, 0.06, 0.08]},
]


def del_regimen(df, reg):
    return df[df.estudio.str.startswith(reg["prefijos"])].copy()


def titulo(t):
    print("\n" + "=" * 78 + f"\n{t}\n" + "=" * 78)


# ---------------------------------------------------------------- carga
def cargar():
    dfs = []
    for f in ["simple_v1", "simple_v2", "simple_v3", "simple_v4", "simple_v5"]:
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
          "simple_v5")


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
    ax.set_ylim(5.0, 30)
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

    colores = {6.0: AZUL, 12.0: "#f59e0b"}
    fig, axs = plt.subplots(3, 5, figsize=(17, 9.5))
    for ax, c in zip(axs.flat, cols):
        ax.scatter(ok.v_max, ok[c], s=3, c=GRIS, alpha=0.2)
        for wm, t in tablas.items():
            b = buenos[buenos.w_max == wm]
            col = colores.get(wm, VERDE)
            ax.scatter(b.v_max, b[c], s=8, c=col, alpha=0.5)
            ax.plot(t.v_max, t[c], c=NEGRO if wm == 6 else "#92400e", lw=2, marker="o", ms=3)
        ax.set_title(c, fontsize=10)
        if c == "k_v_curva":
            ax.set_ylim(-0.5, 14)
        ax.set_xlabel("v_max", fontsize=8)
    axs.flat[-1].axis("off")
    axs.flat[-1].text(0.0, 0.5, "gris: vueltas completas\nazul: 15 % más rápido, w_max = 6\n(simple_v1–v3)\n"
                      "naranja: 15 % más rápido, w_max = 12\n(simple_v4–v5)\nlíneas: medianas por tramo",
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


def main():
    df = cargar()
    resumen = {}
    resumen["evolucion"] = evolucion(df)
    resumen["ruido"] = ruido(df)
    for reg in REGIMENES:
        importancia(df, reg)
        frontera(df, reg)
    tendencia(df)
    telemetria(df)
    for reg in REGIMENES:
        palancas(df, reg)
    barridos()
    titulo("MEJORES VUELTAS INDIVIDUALES")
    print(df[df.ok == 1].nsmallest(10, "tiempo_s")[["estudio", "tiempo_s", "J", "e_rms", "cambios_w_s"] + PARAMS]
          .round(4).to_string(index=False))
    print(f"\nFiguras en {FIG.relative_to(RAIZ)}/")


if __name__ == "__main__":
    main()
