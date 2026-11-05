# SJK001 · Follow Line F1 con PID

Controlador para el ejercicio **Follow Line** de JdeRobot Robotics Academy (ROS 2 Humble + Gazebo Harmonic). Un F1 sigue la línea roja del circuito usando solo su cámara. El controlador es un **PID discreto** con varios añadidos que se explican abajo: mirada adelantada, prealimentación, banda del horizonte y perfil de velocidad. Sus parámetros se ajustan con un panel en vivo y con un optimizador bayesiano.

| Etapa | Vuelta al circuito *simple* |
|---|---|
| Control P (fase 1, v = 10 m/s) | 43.4 s |
| PID ajustado a mano (v = 22) | 20.2 s |
| + banda del horizonte + optimizador | 15.1 s |
| Optimizador con ventana móvil y mejores por repeticiones (v ≈ 56, 3 de 3 vueltas) | **9.25 s** |

El plan, las medidas y las decisiones paso a paso están en [`docs/PLAN.md`](docs/PLAN.md).

---

## 1. Lazo de control

![Lazo cerrado del controlador](docs/img/lazo_control.svg)

En cada fotograma nuevo de la cámara (20 Hz, T ≈ 50 ms) se ejecuta un paso:

1. **Percepción.** Se busca el rojo en 17 filas fijas de la imagen (*scanlines*) y en una banda junto al horizonte. Cada píxel se pasa a metros sobre el suelo con la geometría calibrada de la cámara:

```math
Z = \frac{F\,H}{y - c_y}, \qquad Y = -\frac{H\,(x - c_x)}{y - c_y}
\qquad (F = 320\ \text{px},\ H = 0.1015\ \text{m},\ c_y = 237.45)
```

   Con esos puntos se ajusta una parábola $Y(Z)$, que da la posición y el rumbo de la línea, y un círculo, que da la curvatura $\kappa$. También se calcula $\kappa_{lejos}$, la curvatura de los puntos a más de 1.5 m.

2. **Velocidad.** Se frena según la curva que viene: $v_{ref}$ depende de $\kappa_{lejos}$ y una rampa limita la aceleración.
3. **Giro.** El PID y los términos de prealimentación dan $w$, que se satura a $\pm w_{max}$.
4. Se mandan $v$ y $w$ al coche. Un **vigilante** devuelve el coche a la salida si pierde la línea o choca.

---

## 2. La ecuación de control

### 2.1 De dónde sale el PID

Se parte del PID continuo del tema 3 (T03), con la derivada filtrada y la referencia ponderada (PID de dos grados de libertad):

```math
u(t) = K_p\big(b\,r - y\big) + \frac{K_p}{T_i}\int_0^t e\,d\tau + K_p T_d\,\frac{d e_f}{dt},
\qquad \tau_d\,\frac{d e_f}{dt} + e_f = c\,r - y,\quad \tau_d = \frac{T_d}{N}
```

Se discretiza con Euler hacia atrás, con el periodo $T$ **medido** en cada paso, porque en Python no es exacto:

```math
\begin{aligned}
I_k &= I_{k-1} + q_i\,e_k, & q_i &= K_p\,T/T_i\\
e_{f,k} &= a\,e_{f,k-1} + (1-a)\,(c\,r_k - y_k), & a &= \tau_d/(T+\tau_d)\\
D_k &= q_d\,(e_{f,k} - e_{f,k-1}), & q_d &= K_p\,T_d/T\\
u_k &= \mathrm{sat}\!\big(K_p(b\,r_k - y_k) + I_k + D_k + u_{FF,k}\big)
\end{aligned}
```

El anti-windup es por **integración condicional**: si $u$ satura y $q_i e_k$ empuja hacia la saturación, se deshace la integración de ese paso.

Aquí la referencia es la línea centrada ($r = 0$), la medida $y$ es el error en píxeles $e$ y la salida es la velocidad de giro $w$. Con $b = 1$ y $c = 0$, la D actúa solo sobre la medida y no sobre la referencia, así que no hay pico de derivada.

### 2.2 Ecuación completa del controlador

```math
\boxed{\;
w_k = \mathrm{sat}_{\pm w_{max}}\Big(
\underbrace{-K_p'\,e_k}_{P}
\;\underbrace{-\,K_p'\,T_d\,\frac{\Delta\bar e_k}{T}}_{D\ \text{filtrada}}
\;+\;\underbrace{I_k}_{=\,0}
\;+\;\underbrace{k_{ff}\,v_k\,\kappa_k}_{\text{prealimentación}}
\;\underbrace{-\,k_{hor}\,e^{hor}_k}_{\text{horizonte}}
\Big)\;}
```

con

```math
\begin{aligned}
K_p' &= \frac{K_p}{1 + k_{v}\,v_k} && \text{(ganancia según la velocidad)}\\
e_k &= -F\,\frac{Y(L)}{L}, \quad L = \max\!\big(v_k\,t_{mirar},\ z_{min}\big) && \text{(error con mirada adelantada)}\\
\bar e_k &= a\,\bar e_{k-1} + (1-a)\,e_k, \quad a = \frac{T_d/N}{T + T_d/N} && \text{(error filtrado para la D)}\\
\Delta\bar e_k &= \bar e_k - \bar e_{k-1}, \qquad \frac{\Delta\bar e_k}{T} \approx \frac{d\bar e}{dt} && \text{(derivada discreta: cambio entre dos fotogramas)}\\
\mathrm{sat}_{\pm w_{max}}(u) &= \min\!\big(\max(u,\,-w_{max}),\ w_{max}\big) && \text{(saturación: recorta } w \text{ a } [-w_{max},\,w_{max}]\text{)}\\
e^{hor}_k &= \overline{x}_{banda} - c_x && \text{(columna media de la línea en el horizonte)}\\
v_k &= \mathrm{rampa}\!\Big(\mathrm{clip}\Big(\frac{v_{max}}{1 + k_{vc}\,|\kappa_{lejos}|},\ v_{min},\ v_{max}\Big)\Big) && \text{(perfil de velocidad)}
\end{aligned}
```

### 2.3 Por qué cada término

**El problema: un doble integrador.** El coche es cinemático: el giro cambia el rumbo y el rumbo cambia la posición lateral.

```math
\dot\psi = w,\qquad \dot y \approx v\,\psi \quad\Longrightarrow\quad \frac{Y(s)}{W(s)} = \frac{v}{s^2}
```

Un doble integrador tiene −180° de fase. Con solo P, el coche hace zigzag sin parar, y además hay ≈ 100 ms de retardo entre la imagen y la acción. Para estabilizarlo hace falta **adelanto de fase**. De ahí salen casi todos los términos:

| Término | Parámetros | Por qué está |
|---|---|---|
| **P** | `kp` | Corrige la distancia a la línea. Es la base del lazo. |
| **D filtrada** | `td`, `n_filtro` | Da adelanto de fase y amortigua el zigzag. El filtro $\tau_d = T_d/N$ suaviza los saltos de píxel entre fotogramas. Con $T = 50$ ms el filtro solo actúa si $\tau_d \gtrsim T$, es decir, si $N \lesssim T_d/T$. |
| **I** | `ti` = 0 | **Desactivada a propósito.** El coche no tiene ninguna perturbación constante que corregir (no derrapa) y la I solo resta 90° de fase. El código la mantiene con anti-windup por si se activa. |
| **Saturación y anti-windup** | `w_max` | Limita el giro a lo físicamente razonable. El anti-windup evita que la I se cargue mientras $w$ está saturada y que el coche se pase a la salida de la curva. |
| **Mirada adelantada** | `t_mirar`, `z_mirar_min` | El error se mide a $L = v\,t_{mirar}$ metros por delante (como *pure pursuit*). Como $e \approx F\,(y + L\psi)/L$ incluye el rumbo, añade un **cero en** $s = -1/t_{mirar}$: adelanto de fase **sin derivar ruido**. Es el término que más importa después de la velocidad. |
| **Prealimentación (FF)** | `k_ff` | Para recorrer una curva de curvatura $\kappa$ a velocidad $v$ hace falta exactamente $w = v\,\kappa$. Sin FF, el P necesitaría un error permanente para producir ese giro. En teoría $k_{ff} = 1$, pero κ se mide por delante del coche y el óptimo está en ≈ 0.7. |
| **Banda del horizonte** | `k_hor`, `fila_hor`, `n_filas_hor` | Las filas que tocan el horizonte ven la curva hasta 15 m antes (≈ 0.3 s a 50 m/s), cuando el error cercano aún es casi 0. Se suma como prealimentación, fuera de la D, para empezar a girar antes. En la imagen de la web se ve como una banda azul. |
| **Ganancia según la velocidad** | `kv_kp` | La ganancia del lazo crece con $v$ (por $Y/W = v/s^2$). Bajar $K_p$ a alta velocidad mantiene el mismo margen de estabilidad. |
| **Perfil de velocidad y rampa** | `v_max`, `k_v_curva`, `v_min`, `a_sub`, `a_fren` | El tiempo de vuelta es ≈ 430 m / v. Ir rápido en recta y frenar solo cuando $\kappa_{lejos}$ anuncia una curva cerrada. |
| **Vigilante** | `t_perdida_max`, `t_atasco` | Si se pierde la línea o se atasca contra la pared, el coche vuelve solo a la salida. Así el optimizador puede encadenar cientos de vueltas sin intervención. |

---

## 3. Herramientas para estudiar el PID

### 3.1 Panel en vivo (`tools/panel.sh` → http://localhost:8050)

Recibe la telemetría del controlador en cada fotograma, cambia parámetros sin reiniciar y graba todo en `logs/`.

| Gráfica | Para qué sirve |
|---|---|
| **Error lateral**: `e`, `e_ctl`, `e_hor`, `L` | Ver si el coche oscila alrededor de la línea y comparar el error cercano con el que usa el PID (mirada) y con el del horizonte. Si `e_hor` cambia antes que `e`, el horizonte se anticipa a la curva. |
| **Giro**: `w` y sus partes P, I, D, FF, HOR; `wz` real | Ver qué término domina en cada tramo. Si \|D\| > \|P\| en recta, la D está amplificando el zigzag. Si `wz` no sigue a `w`, el coche no obedece (choque o saturación). |
| **Velocidad**: `v`, `v_ref`, `vx` real | Comprobar el perfil de frenado en las curvas. |
| **Curvatura**: κ, κ_lejos y κ real (`wz/vx`) | Validar la percepción: κ debe adelantarse a la curvatura recorrida y tener su misma escala. |
| **Percepción y tiempo**: puntos válidos, `dt`, `t_perc` | Detectar fotogramas perdidos o lentos. Con `dt` = 50 ms siempre y `t_perc` ≈ 1.2 ms, el cuello de botella es la cámara, no el cálculo. |
| **Mapa** de la vuelta coloreado por \|e\| | Localizar en qué curva concreta se pierde precisión. |
| **Tabla de vueltas** | Tiempo, e_rms, cambios de signo de w por segundo (oscilación) y los parámetros de cada vuelta. Un clic recarga esos parámetros. |
| **Dispersión parámetro → métrica** | Ver qué parámetro afecta a qué métrica entre vueltas. |

### 3.2 Optimizador bayesiano (`tools/optimizar.sh`)

Optuna (TPE) lanza vueltas una tras otra desde el panel y minimiza

```math
J = t_{vuelta} + 0.5\,\frac{e_{rms}}{100} + 0.1\cdot\text{cambios de signo de } w/\text{s},
\qquad J_{fallo} = 60\,(2 - \text{fracción recorrida})
```

1 s de vuelta pesa como 200 px de e_rms o 10 cambios de signo por segundo, así que manda el tiempo. Además:

- Desde el panel se le pueden **proponer ensayos en caliente** (botón «Probar en el optimizador»).
- La **ventana de búsqueda se desplaza sola** cuando los mejores ensayos se acercan a un borde, así que no hay techo para ningún parámetro.
- Cada 10 ensayos **repite uno de los mejores** para comprobar que no fue suerte. Los mejores se ordenan por la **J media de todas sus repeticiones**: un choque cuenta ~75, así que unos parámetros que completan 2 de 24 vueltas ya no pasan por buenos por una vuelta con suerte.

`tools/optimizar.sh --informe` genera estas gráficas (las de abajo son del estudio `simple_v3_g6`, 1176 ensayos):

| | |
|---|---|
| ![Historia](docs/img/optuna_historia.png) | **Historia.** J de cada ensayo y el mejor hasta el momento. Cada escalón de la línea negra es una mejora. Que sigan apareciendo escalones tras cientos de ensayos indica que mover la ventana de `v_max` hacia arriba sigue dando mejoras. |
| ![Importancia](docs/img/optuna_importancia.png) | **Importancia (fANOVA).** Cuánto explica cada parámetro de la variación de J. Al final mandan `v_max` (0.21), `k_ff`, `k_v_curva` y `v_min` (≈ 0.15 cada uno): con la anticipación ya ajustada, lo que decide el tiempo es cuánto frenar en las curvas. `kv_kp` y `n_filtro` apenas cuentan y la I ni aparece. |
| ![Cortes](docs/img/optuna_cortes.png) | **J frente a cada parámetro.** Dónde está la zona buena de cada uno (las aspas rojas de arriba son vueltas fallidas). Se ve que J baja con más `v_max` hasta ≈ 56 y que `k_ff` funciona mejor hacia 0.75. |
| ![Tiempo frente a oscilación](docs/img/optuna_tiempo_vs_osc.png) | **Tiempo frente a oscilación.** El compromiso entre ir rápido y no hacer zigzag; abajo a la izquierda es mejor. |

### 3.3 Análisis fuera de línea

- `analysis/kappa_vs_trayectoria.py`: correlaciona la κ que ve la cámara con la curvatura que recorre el coche (odometría). Da el adelanto real de la percepción y comprueba que la escala es ≈ 1.
- `tools/resumen_log.py`: resumen de un log (error, giro, oscilación y fotogramas perdidos).
- `tools/bench_percepcion.py`: coste en ms de la percepción.

---

## 4. Mejores parámetros

Ensayo #1054 de `simple_v3_g6`: **9.25–9.3 s en 3 de 3 vueltas** (J media 10.61), guardado como preset `optuna_simple_v3`. Hay otros 6 grupos de parámetros muy parecidos con 3/3 vueltas entre 9.25 y 9.4 s.

| v_max | k_v_curva | v_min | kp | td | n_filtro | k_ff | kv_kp | t_mirar | z_mirar_min | k_hor | w_max | ti |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 56.4 | 6.51 | 10.8 | 0.00771 | 0.0836 | 15.2 | 0.765 | 0.023 | 0.175 | 0.316 | 0.0455 | 6 | 0 |

El límite de rendimiento es la cámara (20 Hz, un fotograma cada 50 ms), no el cálculo: la percepción tarda ≈ 1.2 ms, así que pasar el controlador a C++ no mejoraría la vuelta (ver `docs/PLAN.md` §5.8).

---

## 5. Uso

```bash
# Simulador (contenedor con GPU)
docker run -d --rm --name robotics --device /dev/dri --gpus all \
  -p 6080-6090:6080-6090 -p 7163:7163 jderobot/robotics-backend:latest

tools/panel.sh          # panel en http://localhost:8050
# Pegar src/follow_line.py en la web del ejercicio (http://localhost:7163) y pulsar play
tools/optimizar.sh      # optimizador (opcional); --informe para el informe
tools/test.sh           # pruebas
```

| Ruta | Contenido |
|---|---|
| `src/follow_line.py` | Controlador completo: percepción, PID, telemetría y recuperación. Se pega tal cual en la web. |
| `tools/panel/` | Panel web en vivo (aiohttp + uPlot) y presets de parámetros. |
| `tools/optimizador/` | Optimizador (Optuna), su configuración e informe. |
| `tools/sim/` | Utilidades del simulador: calibración de la cámara y pruebas de giro. |
| `tests/` | Pruebas de percepción, control y ventana del optimizador. |
| `docs/PLAN.md` | Plan, teoría del T03 y registro de cada fase. |
