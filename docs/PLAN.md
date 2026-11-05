# SJK001 · Laboratorio 1 — Follow Line con PID y trazada tipo F1

Plan de acción e investigación previa. Fecha: 2026-11-04.

## 0. Contexto y objetivo

- **Ejercicio:** robot de carreras (F1) simulado en ROS 2 + Gazebo que sigue una línea roja con una cámara frontal y OpenCV. Lenguaje: Python (o C++).
- **Entorno:** JdeRobot Robotics Academy en docker:
  ```bash
  docker run --rm -it --device /dev/dri --gpus all \
    -p 6080-6090:6080-6090 -p 7163:7163 jderobot/robotics-backend:latest
  ```
- **Objetivo:** mejor tiempo por vuelta (contrarreloj).
- **Idea propia:**
  1. Aprovechar que la imagen no tiene ruido y que la cámara mira al frente: la parte alta de la imagen (horizonte) muestra las curvas antes de llegar a ellas. Con eso se puede **anticipar** la velocidad y el giro.
  2. Trazar las curvas como un F1, **exterior → vértice interior → exterior**, para ir más rápido.
  3. Leer la imagen a bajo nivel, por filas, para detectar curvas pronto y reducir el tiempo de procesado.
  4. Cerrar el bucle con un PID del temario de IR2124 combinado con estos datos de visión.
- **Supuesto:** en la imagen no aparece nada más que la línea.

---

## 1. Resultados de la investigación

### 1.1 La lectura fila a fila no da ventaja de tiempo

- En Gazebo, la cámara genera el fotograma entero y lo publica como un único mensaje de ROS 2 (`sensor_msgs/Image`). El código lo recibe completo como un array de numpy (`HAL.getImage()`). **Ninguna fila llega antes que otra**, porque no hay *rolling shutter* simulado ni lectura progresiva.
- Por eso, leer primero las filas de arriba **no adelanta la información en el tiempo**.

### 1.2 Lo que sí funciona de la idea

1. **Procesar menos píxeles.** Numpy guarda la imagen por filas y cada fila es contigua en memoria (*row-major*). Recortar filas (`img[y0:y1]`) o tomar filas sueltas (`img[[y1, y2, ...]]`) es muy barato. Analizar entre 6 y 10 filas, en lugar de umbralizar los 640×480 píxeles, reduce el cálculo uno o dos órdenes de magnitud. Así el bucle de control puede ir más rápido y con menos retardo.
2. **Mirar lejos (*preview control*).** Las filas cercanas al horizonte muestran la pista futura y las de abajo, la posición actual. Si se comparan los centros de la línea en varias filas, se obtiene la curvatura que viene **antes** de entrar en la curva. Esto permite:
   - frenar antes de la curva, no dentro;
   - empezar a girar con una prealimentación (*feedforward*);
   - preparar la trazada abriéndose al exterior.

### 1.3 Material del temario que se va a usar (IR2124, `~/uji/uji/IR2124/teoria/`)

| Tema | Fichero | Qué se aprovecha |
|---|---|---|
| T03 | `T03 - Implementación digital del control PID/Slides_T3_Implem_PID.pdf` | PID discreto: **filtro en la derivada**, **ponderación de referencia (b, c)**, **anti-windup**, **prealimentación (FF)**, bumpless/tracking, elección del periodo T, filtro paso bajo de primer orden |
| T03 | `ControlMotorDC_PID.ipynb`, ZIP `EjemplosPIDArduino (C)` | Ejemplos de código del PID discreto |
| T02 | `T02 - Control entrada-salida/Slides_T2_Control_PID.pdf` p.24-55 | Significado de Kp, Ti y Td, PID-2DOF, sintonía Ziegler–Nichols y métodos experimentales |
| T02 | `Resumen_Control_Asignacion_Polos.pdf` | Diseño por asignación de polos (si se identifica un modelo) |
| T10 | `Slides_T10_Control_Digital.pdf` p.44-62, notebook `Identificación_de_sistemas_discretos.ipynb` | Identificar el modelo `w → error lateral` por mínimos cuadrados |
| T10 | `Slides_T10` p.68-80, notebooks `Control_predictivo.ipynb`, `Optimización_con_restricciones.ipynb` | MPC (opcional) usando el horizonte de visión |

#### PID de referencia (T03, diapositiva "Control PID con filtro, ponderación y AW")

```
u(t) = Kp·(b·r − y) + (1/Ti)∫e dt + Td·d(ef)/dt
d(ef)/dt = ((c·r − y) − ef)/τd
a = τd/(T+τd),  qi = Kp·T/Ti,  qd = Kp·Td/T,  τd = Td/N

e  = r - y
I  = I + qi*e
ef = a*ef1 + (1-a)*(c*r - y)
D  = qd*(ef - ef1)
u  = Kp*(b*r - y) + I + D
if ((u<umin) AND (qi*e<0)) OR ((u>umax) AND (qi*e>0)):  I = I - qi*e   # anti-windup
u  = Limita(u, umin, umax)
ef1 = ef
```

#### PID con FF y AW (T03, diapositiva "Control PID con FF y AW")

```
I = I + qi*e
D = qd*(e - e1)
u = Kp*e + I + D + uFF
if (u<umin) OR (u>umax):  I = I - qi*e
u = Limita(u, umin, umax)
```

**Por qué encaja con este problema:**
- **Ponderación b, c:** la trazada F1 cambia la referencia `r` de golpe en cada fase. Con `c = 0` (y `b < 1`) se evita el pico de la derivada (*derivative kick*).
- **Filtro en D:** aunque la imagen no tenga ruido, el centroide va a saltos (píxeles enteros, cambios de fotograma). El filtro suaviza la derivada.
- **Anti-windup:** `w` está saturada (giro máximo). Sin anti-windup, el integrador se acumula en las curvas largas y el coche se pasa a la salida.
- **FF:** la curvatura vista por delante da `w_FF ≈ v·κ`, de modo que el giro empieza antes de que aparezca el error.
- **Periodo T:** en Python el periodo no es fijo. Hay que medir `dt` en cada iteración y recalcular `qi`, `qd` y `a`.

### 1.4 Técnicas de seguimiento de línea y de trazada consideradas

| Técnica | Idea | Uso en el plan |
|---|---|---|
| Centroide con momentos (`cv2.moments`) | `cx = m10/m00` sobre una franja | Error de posición actual (filas inferiores) |
| Scanlines | Centro de la línea en N filas fijas | Percepción rápida y curvatura |
| Ajuste polinómico `x = a·y² + b·y + c` | `a` ~ curvatura y su signo indica el sentido | Clasificar recta/curva y calcular el FF |
| IPM / homografía (vista de pájaro) | Pasar puntos de la imagen al suelo | Opcional: curvatura en metros |
| Pure Pursuit | Seguir un punto a distancia L por delante | Alternativa o complemento del PID con FF |
| Stanley | Error lateral más error de orientación | Alternativa (necesita orientación: se obtiene de la pendiente del ajuste) |
| Trazada exterior–interior–exterior | Radio efectivo mayor, así que más velocidad en curva | Offset de la referencia con una máquina de estados |
| Gain scheduling | Kp(v) decreciente con v | Estabilidad a alta velocidad |
| MPC | Optimizar v y w en un horizonte con restricciones | Opcional avanzado (T10) |

---

## 2. Arquitectura propuesta

```
Imagen ──► [Percepción por scanlines] ──► cx_cerca, cx_medio, cx_lejos, curvatura κ, ¿recta?
                                               │
                     ┌─────────────────────────┴────────────────────────┐
                     ▼                                                  ▼
       [Planificador de trazada "F1"]                    [Perfil de velocidad]
       referencia r = centro + offset(κ, fase)           v_ref = f(κ_lejos), frenar antes
                     │                                                  │
                     ▼                                                  ▼
  PID de giro (T03: filtro D + ponderación b,c + AW + FF)       Limitador de aceleración
       w = PID(r, cx) + w_FF(v·κ)                                       │
                     └──────────────► HAL.setW(w)   HAL.setV(v) ◄───────┘
```

### 2.1 Percepción por scanlines
- Umbral HSV del rojo con **dos rangos de H** (aprox. 0–10 y 170–180), porque el rojo está en los dos extremos del círculo de tono. Hay que ajustarlos con las imágenes reales del simulador.
- Aplicar el umbral **solo a la franja útil** de la imagen (por debajo del horizonte), no a la imagen entera.
- Filas candidatas (ajustar a la resolución real): `[470, 440, 400, 350, 300, 260]`.
- En cada fila, el centro de la línea es la media de `np.flatnonzero(mask_row)`. Si la fila no tiene rojo, el punto se marca como inválido.
- Ajuste `np.polyfit(y, x, 2)` con los puntos válidos, del que salen:
  - `κ` (proporcional a `a`) y el sentido de la curva;
  - la orientación de la línea (pendiente en la fila inferior);
  - el criterio de recta: residuo bajo y `|a|` menor que un umbral.
- Error de posición: `cv2.moments` en la franja inferior, o la media de las 2–3 filas más bajas.
- Medir el tiempo de la percepción completa frente a la de scanlines y documentarlo.

### 2.2 Planificador de trazada F1 (máquina de estados)

| Estado | Condición (aprox.) | Offset de la referencia | Velocidad |
|---|---|---|---|
| RECTA | κ_lejos ≈ 0 | 0 | v_max |
| APROXIMACIÓN | κ_lejos alta, κ_cerca baja | hacia el **exterior** de la curva | frenar hasta v_curva(κ) |
| VÉRTICE | κ_medio/κ_cerca altas | hacia el **interior** | mantener v_curva |
| SALIDA | κ bajando | volver al exterior y luego a 0 | acelerar con rampa |

- `r = cx_img + offset`. El offset está **limitado** para que la línea no salga de la imagen.
- Hay que comprobar si el ejercicio penaliza separarse de la línea.
- Histéresis en las transiciones para que el estado no oscile.

### 2.3 Control de giro
- PID del T03 con filtro D, ponderación (b, c), anti-windup y `dt` medido.
- `w = PID(r, cx) + w_FF`, con `w_FF = k_ff · v · κ`.
- Gain scheduling: `Kp(v) = Kp0 / (1 + kv·v)` o una tabla por tramos.

### 2.4 Control de velocidad
- `v_ref = v_max / (1 + k·|κ_lejos|)` o una tabla por estado.
- Limitador de aceleración y frenada (rampa) para evitar derrapes.
- Línea perdida: frenar y girar hacia el lado donde se vio por última vez.

---

## 3. Fases de trabajo

| Fase | Qué se hace | Cómo se comprueba |
|---|---|---|
| **0. Entorno** | Arrancar el docker, abrir el ejercicio Follow Line y confirmar la API de la versión instalada (`HAL.getImage()`, `HAL.setV()`, `HAL.setW()`, `GUI.showImage()`), la resolución, la frecuencia de la cámara, los circuitos y si hay penalizaciones | Script mínimo que muestra la máscara del rojo |
| **1. Línea base** | Control P sobre el centroide, a velocidad constante | Primer tiempo por vuelta (referencia) |
| **2. Percepción por scanlines** | Puntos por fila, ajuste de parábola, κ, dibujo de los puntos y el estado en la GUI, comparación de tiempos | Tabla de ms por fotograma y Hz del bucle |
| **3. PID completo T03** | Filtro D, anti-windup y `dt` medido. Sintonía inicial con Ziegler–Nichols o un método experimental en bucle cerrado (T02), después ajuste fino | Gráficas de `e` y `w`, sin oscilaciones en recta |
| **4. Velocidad con anticipación** | `v` según κ_lejos más la rampa | Tiempo por vuelta y error máximo en curva |
| **5. FF y gain scheduling** | `w_FF = v·κ`, `Kp(v)` | Error en curva con y sin FF |
| **6. Trazada F1** | Máquina de estados de offset y ponderación b, c | Tiempo por vuelta frente a la fase 5 |
| **7. Optimización** | Barridos o búsqueda de parámetros (offsets, umbrales de κ, v_max, ganancias). Validar en varios circuitos | Tabla comparativa final de todas las fases |
| **8. (Opcional) Avanzado** | Identificar el modelo `w → e` (T10, mínimos cuadrados), diseñar por asignación de polos (T02) y/o un MPC pequeño (T10) | Comparación con el PID |

**En todas las fases:** guardar en CSV `t, dt, cx, r, e, κ, v, w, estado` y hacer las gráficas en `analysis/`. Ese material se usa en la memoria y sirve para justificar cada mejora con números.

---

## 4. Estructura prevista del repo

```
SJK001/
├── docs/PLAN.md            # este documento
├── src/follow_line.py      # versión de un solo fichero para pegar en el ejercicio
├── src/modules/            # perception.py, pid.py, planner.py (desarrollo y pruebas fuera del simulador)
├── tests/                  # pruebas de pid.py y perception.py con imágenes sintéticas
├── logs/                   # CSV de cada ejecución
├── analysis/               # notebooks y scripts de gráficas
└── tools/panel/            # panel de ajuste en vivo (tools/panel.sh → localhost:8050)
```

---

## 5. Riesgos y comprobaciones pendientes

- [x] **API del ejercicio** (ver 5.1): cambia entre versiones de `robotics-backend`. Confirmarla en la fase 0.
- [x] **Resolución y frecuencia de la cámara** (640×480 a 20 Hz, ver 5.1): de ellas dependen las filas de scanline y la T efectiva.
- [ ] **Fotogramas repetidos:** si el bucle va más rápido que la cámara, el término D sale 0 o salta. Detectar si el fotograma es nuevo y solo actualizar el PID cuando lo sea.
- [x] **Penalización por alejarse de la línea:** no hay. Antes: limita lo agresiva que puede ser la trazada F1.
- [ ] **Offset excesivo:** la línea puede salir de la imagen. Limitarlo según la anchura visible.
- [ ] **Sobreajuste a un circuito:** validar en al menos dos circuitos.
- [ ] **Rangos HSV:** ajustarlos con capturas reales (el rojo cubre H≈0 y H≈180).

---

## 5.1 Datos confirmados en la fase 0 (inspección del contenedor, 2026-11-04)

Fuente: `/home/ws/install/custom_robots/share/custom_robots/models/f1/` dentro de `jderobot/robotics-backend:latest` (Gazebo Harmonic, ROS 2 Humble).

| Dato | Valor | Consecuencia para el plan |
|---|---|---|
| Resolución | 640×480, `R8G8B8` | Las filas de scanline previstas valen tal cual |
| Frecuencia de cámara | **20 Hz** (`update_rate`) | T efectivo del PID ≈ 50 ms. Si el bucle va más rápido, habrá fotogramas repetidos: actualizar el PID solo con fotograma nuevo |
| FOV horizontal | 1.57 rad (90°) → focal ≈ 320 px | Permite pasar píxeles a metros (IPM) |
| Pose de la cámara | `xyz = 0.45 0.04 0.1`, `rpy = 0 0 0` | Cabeceo 0: el **horizonte está en la fila ≈240**. Solo las filas >240 ven el suelo. La cámara está **4 cm a la izquierda** del eje: con el coche centrado, la línea aparece algo desplazada a la derecha, más en las filas cercanas |
| Alcance | `clip far = 15 m` | El *preview* no ve más allá de 15 m |
| Ruido | **gaussiano, σ = 0.007** | La imagen sí tiene algo de ruido (corrige el supuesto del apartado 0). Sigue siendo pequeño; el umbral HSV lo absorbe, pero refuerza el filtro de la derivada y sirve para detectar fotogramas nuevos (dos fotogramas distintos nunca son idénticos) |
| Actuación | Plugin `VelocityControl` en `/f1/cmd_vel` (variante holonómica); hay también variante Ackermann | `setV`/`setW` mandan un *twist* directo: no hay dinámica de dirección que modelar en la versión normal |
| Circuitos | simple, montmelo, montreal, nurburgring, monaco, spa | Validar en al menos dos |

**API real** (el frontend la copia a `/workspace/code/` del contenedor al conectar):

| Elemento | Detalle | Consecuencia |
|---|---|---|
| `HAL.getImage()` | Array **BGR** 480×640×3. Convierte el mensaje ROS con `cv_bridge` **en cada llamada**, aunque el fotograma no haya cambiado | Llamarla solo cuando haya fotograma nuevo |
| Fotograma nuevo | `HAL.camera_node.last_img_.header.stamp` (sello de simulación) | Detección barata de fotograma nuevo; da también `dt` en tiempo simulado para el PID |
| `HAL.setV(v)`, `HAL.setW(w)` | `Twist` en `/f1/cmd_vel`. `MotorsNode(topic, 4, 0.3)` **ignora** los límites | Sin saturación en el HAL: `umin`/`umax` del anti-windup los ponemos nosotros |
| Interfaz | Módulo **`WebGUI`** (`WebGUI.showImage(img)`), no `GUI`. Codifica JPEG en otro hilo | Mostrar imágenes pequeñas o no en cada iteración para no cargar la CPU |
| `Frequency.tick(hz)` | Duerme hasta completar el periodo | Opcional; mejor sincronizar con el fotograma nuevo |
| Cronómetro de vuelta | `lap.py` usa `datetime.now()`, **hora de reloj**, y arranca con play | Si la simulación va por debajo de tiempo real (RTF < 1), la vuelta "dura más". Medir el RTF |
| Física | Paso por defecto (1 ms), objetivo RTF = 1 | — |
| Modo del coche | `mode:=holo` en el circuito simple, salida en (53.46, −10.73), yaw −π/2 | — |
| Penalizaciones | **No hay** penalización por salirse de la línea (confirmado por el usuario) | La trazada F1 puede ser agresiva; el único límite es no perder la línea de vista |

**Medidas con la simulación en marcha** (proceso aparte suscrito a `/f1/camera/image_raw`, coche parado en la salida del circuito simple):

| Medida | Valor | Consecuencia |
|---|---|---|
| Fotogramas | 20.2 Hz en tiempo simulado, **18.1 Hz de reloj** | **RTF ≈ 0.90**: la simulación va al 90 % del tiempo real y el cronómetro (hora de reloj) lo nota. Cerrar programas pesados al medir tiempos y comparar siempre en las mismas condiciones |
| Codificación del topic | `rgb8`; el HAL la pasa a BGR | — |
| Conversión `cv_bridge` | ≈ 0.3–0.5 ms | Poco coste, pero no repetirla sin fotograma nuevo |
| Máscara HSV de la imagen completa | ≈ 1.3 ms | — |
| Máscara HSV de 8 filas | ≈ 0.03 ms | **40× más rápida**, pero los dos tiempos son mínimos frente a los 50 ms entre fotogramas. Las scanlines se justifican por la **información de *preview***, no por la velocidad. El cuello de botella es la cámara a 20 Hz |
| Horizonte | Primera fila con rojo: **239** | Confirma el cálculo (cabeceo 0 → fila 240) |
| Anchura de la línea | 243 px (fila 470), 118 (350), 26 (260), 10 (245) | La línea es muy ancha abajo y fina cerca del horizonte, pero se detecta incluso en la fila 245 |
| Centro de la línea en la salida | 317 (fila 470) → 320 (fila 245) | El coche sale casi centrado y alineado |
| Color | Rojo puro, BGR = [0, 0, 216] | El umbral HSV `(0–10, 100–255, 60–255) ∪ (170–180, …)` funciona |

## 5.2 Fase 1 — línea base con control P (2026-11-04)

**Banco de pruebas** (`tools/vuelta.sh`): ejecuta `src/follow_line.py` dentro del contenedor con el HAL real y un `WebGUI` vacío. Antes de empezar recoloca el coche en la salida (`gz service /world/default/set_pose`) y quita la pausa. Cronometra la vuelta con `/f1/odom`: cuenta como vuelta cuando el coche se aleja más de 30 m y vuelve a menos de 2.5 m de la salida. Falla si la línea se pierde durante 3 s. Guarda el CSV en `logs/`, y `tools/resumen_log.py` lo resume. Mientras corre, **no hay que tocar la web**: recargarla reinicia Gazebo.

Controlador: centroide del rojo en las filas 250–480, `w = −Kp·(cx − 320)` y `v` constante. Solo actualiza con fotograma nuevo (dt = 50.0 ms exactos).

| v (m/s) | Kp (rad/s/px) | Vuelta (s, sim) | \|e\| medio (px) | Cambios de signo de w (/s) | Resultado |
|---|---|---|---|---|---|
| 3 | 0.005 | 144.5 | 33 | 0.05 | limpia |
| 5 | 0.005 | 87.5 | 60 | 0.24 | limpia |
| 7 | 0.005 | 62.3 | 89 | 0.50 | limpia |
| 10 | 0.005 | — | — | — | pierde la línea en la 1.ª curva |
| **10** | **0.008** | **43.5** | 80 | 0.83 | **limpia → referencia** |
| 12 | 0.010 | 36.2 | — | — | completa |
| 14 | 0.012 | 31.2 | 112 | 3.2 | oscila |
| 17 | 0.015 | 26.2 | 187 | 3.7 | oscila mucho y llega al borde de la imagen |
| 20 | 0.018 | 22.7 | 198 | 3.8 | completa de milagro (4 fotogramas sin línea, 445 m frente a 428) |
| 20 | 0.025 | — | — | — | pierde la línea |
| 24 | 0.016–0.020 | — | — | — | pierde la línea |

**Conclusiones:**
- El coche es cinemático (`VelocityControl`): no derrapa, así que **tiempo ≈ 428 m / v**. Todo se juega en mantener la línea a la máxima v posible.
- Con P solo, el límite está entre 20 y 24 m/s. A partir de 14 m/s el coche oscila a unos 3–4 Hz: con un retardo de 50 ms y Kp alto, el lazo se queda sin margen de fase. **Aquí entra el plan:** la derivada filtrada (fase 3) da margen de fase, el *preview* y el FF (fases 4–5) adelantan el giro, y la v variable permite ir a más de 20 m/s en recta.
- Referencia oficial para comparar: **43.5 s (v = 10, Kp = 0.008), vuelta limpia**. Techo del P: 22.7 s, al límite.

## 5.3 Fase 2 — percepción por scanlines (2026-11-04, EN CURSO)

**Hecho** (en `src/follow_line.py`, con 21 pruebas en `tests/` que pasan con `tools/test.sh`):
- 15 filas de scanline, de la 470 (0.14 m por delante) a la 242 (13.3 m). Si una fila tiene varios tramos rojos, se elige el más cercano al punto de la fila anterior.
- Paso de píxel a suelo: `Z = F·H/(y−CY)`, `Y = −H·(x−CX)/(y−CY)`, con F = 320, CX = 320, CY = 239.5 y H = 0.104 m.
- Parábola ponderada → `y0` (posición lateral) y `rumbo`. **Curvatura por ajuste algebraico de círculo** (la parábola la sobrestimaba un 26 %). El signo lo da la parábola. `kappa` usa todas las filas y `kappa_lejos` solo Z ≥ 1.5 m. Es recta si |κ| < 0.02.
- En arcos sintéticos de R = 4–30 m, el error de κ es de 4 % como máximo, y en recta κ ≈ 1e-4.
- Coste (`tools/bench_percepcion.py`): **fase 2 completa 0.68 ms** frente a 1.0 ms de la fase 1. Las dos son mínimas frente a los 50 ms entre fotogramas.
- La GUI dibuja los puntos, el estado (RECTA o CURVA IZQ/DER), κ, v, w y el número de puntos. Se refresca a 10 Hz.
- El control sigue siendo P sobre `e_px` (media de las filas ≥ 390): a v = 10 da **43.4 s**, igual que la fase 1.
- El banco (`tools/vuelta.sh`) ahora guarda también la **odometría real** (`logs/*_odom.csv`: t, x, y, yaw, vx, wz). El controlador muere si muere el banco (PDEATHSIG), así que ya no quedan procesos huérfanos.
- Análisis: `analysis/kappa_vs_trayectoria.py` (en el contenedor la gráfica falla por un problema de matplotlib; hay que hacerla en el host con `uv run --with matplotlib`). También `tools/sim/serie_cinematica.py v w s` y `prueba_cinematica.py` para mandar (v, w) fijos y ver la odometría.

**Resultados de validación** (log `logs/20261104-195234_f2_v10*.csv`):
- κ de la cámara frente a `w/v` mandada: **correlación de 0.94**, y la cámara ve la curva **0.25 s antes** (κ) o **0.35 s antes** (κ_lejos). La preview funciona.
- Pero la pendiente es de unas 3×, y ∫κ·v·dt = −2.39 frente a −2π. **Con lo de abajo, `w/v` mandada NO sirve como referencia de curvatura.**
- La odometría da un giro neto de −6.283 rad (la pista es una vuelta normal). ∫wz real = −6.36 rad, pero **∫w mandada = −8.99 rad**.
**Resuelto (2026-11-05, PC nuevo) — el coche "ignora" las `w` pequeñas:**
- **La prueba con (v, w) fijos en la salida era un choque.** Hay una pared lisa en **x = 51.95**, a 1.5 m a la izquierda de la salida (distancia punto–triángulo a `pared.stl`). Con w = −0.3 el coche gira hacia ella, la toca en x ≈ 52.14 (medio ancho = 0.18 m) y desliza pegado. Las paredes rodean la pista a ~1.5 m de la línea.
- **Lo de la vuelta es otra cosa: la fricción del simulador.** En modo holo el coche es **un solo cuerpo** (`Car.obj` × 0.2, sin ruedas) que desliza sobre el plano mientras `VelocityControl` le impone la velocidad. DART calcula la fricción por separado en los ejes X/Y del mundo. Cuando el coche avanza **alineado con un eje**, la componente perpendicular queda en rozamiento estático y anula los giros pequeños:
  - en marcha a 8 m/s, alineado: w = −0.15 → **1 %** del giro; girado 0.15 rad: **91 %**;
  - una vez bloqueado, w = −0.33 no lo suelta; −0.5 sí (tras ~0.2 s); con −0.8 gira al 99 %;
  - reproduciendo en bucle abierto los (v, w) del log (`tools/sim/reproducir_log.py`) el bloqueo sale **en los mismos tramos** (yaw clavado en 1.5708 y 3.1410): es determinista.
  - Consecuencias: (1) hay una **zona muerta de w ≈ ±0.35–0.5 rad/s cuando el coche va alineado con un eje** (las rectas del circuito simple lo están). El error residual en recta es de 2–3 cm, inofensivo. Pero la I del PID puede acumularse en esa zona y soltarse de golpe, así que conviene vigilarlo en las gráficas de `wz real`. (2) La `w` mandada no sirve como medida de curvatura: hay que usar la odometría.
- **κ salía ~3 veces pequeña por la calibración de la cámara, no por la referencia.** Frente a la odometría: correlación 0.96 y adelanto de 0.30 s (κ) o 0.45 s (κ_lejos), pero con pendiente 3.0, y el giro neto por la cámara salía −2.40 rad frente a −2π. La causa es que el coche apoya **0.52° morro abajo** (cuaternión de Gazebo), lo que sube el horizonte. Calibrado con desplazamientos laterales conocidos (`tools/sim/calibrar_h.py`, 44 puntos): **CY = 237.45 px** (antes 239.5) y **H = 0.1015 m** (antes 0.104). La fila 242 estaba en realidad a 7.1 m, no a 13.3. La "primera fila con rojo = 239" era el corte de la cámara a 15 m, no el horizonte. Se añaden las filas 241 (9.1 m) y 240 (12.7 m).
- Comprobación en fotos fijas (`tools/sim/foto_pose.py`): curvas de R = 7.0 y 6.5 m (odometría) → κ da **7.0 y 6.4 m**; en recta κ = 0. En la curva larga de R = 35 m da 50 m (antes 94).
- Además, con el coche centrado la línea sale centrada (x ≈ 319): el desfase de 4 cm de la cámara no se aprecia.
- Gráfica: `analysis/kappa_vs_odometria_v10.png` (log anterior a la calibración). El script ya compara con la odometría.

## 5.4 Flujo de trabajo con el panel en vivo (desde 2026-11-05)

El usuario ejecuta el controlador desde la web (así ve el coche) y ajusta los parámetros en vivo; el banco `tools/vuelta.sh` queda para pruebas automáticas.

1. `tools/panel.sh` en el PC → abrir **http://localhost:8050**.
2. Pegar `src/follow_line.py` en el editor del ejercicio y pulsar play. El controlador abre la telemetría en el puerto **6089** del contenedor (mapeado); el panel se conecta solo. No se usa el sentido contenedor → PC porque `ufw` lo bloquea.
3. En el panel:
   - **sliders** de todos los parámetros de `PARAMS` (velocidad, PID del T03, FF, gain scheduling), que se aplican al momento;
   - **gráficas** de e, w (P/I/D/FF y `wz` real), v, κ frente a curvatura real, y percepción. Los cambios de parámetros, las marcas (tecla M) y las vueltas salen como líneas verticales;
   - **mapa** de la vuelta coloreado por |e|;
   - **tabla de vueltas**: tiempo, |e| medio/rms/máx, perdidos, cambios de signo de w/s y los parámetros de cada vuelta (con * si cambiaron durante la vuelta). Clic en una fila → recarga esos parámetros;
   - **dispersión parámetro → métrica** entre vueltas, para ver qué parámetro afecta a qué;
   - **presets** (`tools/panel/presets/*.json`).
4. Grabación automática en `logs/<fecha>_panel.csv` (todos los fotogramas, con los parámetros vigentes) y `logs/<fecha>_panel_vueltas.csv`. Cada play (o el botón «Nueva prueba») abre ficheros nuevos.

Con los valores por defecto, el controlador es el P de la fase 1 (v = 10, Kp = 0.008). Ti = 0 y Td = 0 desactivan I y D.

**Siguiente paso:** fase 3, en la que el usuario sintoniza el PID con el panel. Hay que validar la nueva calibración con una vuelta completa: la curvatura de la gráfica «κ frente a real» debería tener pendiente ≈ 1.

## 5.5 Fase 3 — ajuste manual, oscilaciones y optimizador automático (2026-11-05)

**Resultados del ajuste manual en el panel** (logs `20261105-01*`/`02*_panel_vueltas.csv`):

| Preset | v_max | Kp | Td | N | k_ff | Vuelta | e_rms (px) | Cambios de w/s |
|---|---|---|---|---|---|---|---|---|
| P de la fase 1 | 10 | 0.008 | 0 | – | 0 | 43.4 s | 120 | 1.2 |
| — | 16 | 0.008 | 0.05 | 10 | 0.25 | 27.3 s | 148 | 1.4 |
| — | 19 | 0.0085 | 0.06 | 10 | 0.35 | 23.0 s | 160 | 3.6 |
| **v8** | **22** | 0.009 | 0.08 | 13 | 0.4 | **20.2 s** | 221 | 5.0 |
| v9 | 17 | 0.0085 | 0.18 | 15 | 0.2 | 25.8 s | 196 | 7.4 |

- El tiempo es ≈ 430 m / v. Todo consiste en poder subir v sin perder la línea.
- **Diagnóstico de la oscilación** (vuelta v8): en recta |D| = 1.68 > |P| = 1.45 y |w| = 2.8 rad/s, así que el coche va en zigzag constante. El 47 % del tiempo |e| > 250 px (la línea casi en el borde) y e_max = 320 en cada vuelta. Con Td = 0.18 (v9) los cambios de signo suben a 7.4/s: la D amplifica.
- **Causa estructural:** el error se mide en las filas cercanas (0.14–0.2 m del morro) con ~100 ms de retardo. Desde w, ese error es casi un doble integrador (w → rumbo → posición, −180°). Solo la D da adelanto de fase, y lo hace derivando una señal que va a saltos.
- **La I empeora:** no hay perturbación constante (coche cinemático, sin deslizamiento) y la I solo añade −90° de fase. Ti = 0.
- **N:** τd = Td/N. Con dt = 50 ms, el filtro solo actúa si τd ≳ dt, es decir, N ≲ Td/dt. En la v8 (Td = 0.08, N = 13) τd = 6 ms y a = 0.11: **el filtro no hace casi nada**. Para filtrar, N ≈ 1–3.
- **kv_kp:** Kp_ef = Kp/(1 + kv·v). La ganancia del lazo crece con v (y'' = v·w), así que baja Kp a alta velocidad. **Con velocidad constante equivale a usar otro Kp**; solo tiene efecto propio con perfil de velocidad (k_v_curva > 0).
- **k_ff:** para recorrer una curva κ a v hace falta exactamente w = v·κ, así que k_ff = 1 es el valor teórico si κ es exacta. Pero κ se mide por delante (preview de 0.3 s), y con 1 el coche gira antes de tiempo. De ahí que 0.4 funcione mejor. El optimizador explora 0–1.2.

**Cambios en el controlador:**
- **Mirada adelantada (`t_mirar`)**, tipo pure pursuit. El error se mide a L = v·t_mirar m por delante sobre la parábola de la línea: e = −F·Y(L)/L px (mismas unidades que antes). Así e ≈ (Y + L·ψ)·F/L incluye el rumbo, lo que añade un cero en s = −1/t_mirar: adelanto de fase geométrico, sin derivar ruido. Con L = v·t, el P equivale a pure pursuit con Kp ≈ 2/(F·t) (t = 0.5 s → Kp ≈ 0.0125). t_mirar = 0 es el comportamiento anterior.
- **Recuperación automática:** si pasa > t_perdida_max sin línea o la velocidad real cae por debajo del 40 % de la mandada durante t_atasco (choque con la pared), el coche vuelve solo a la salida (`gz service set_pose`) y el panel apunta el intento como fallido (✗). Botón «Volver a la salida» en el panel.
- Se envía `e_ctl` (el error que usa el PID) y `L` a las gráficas.

**Optimizador automático** (`tools/optimizar.sh`, preparado y **sin ejecutar** todavía sobre el simulador; probado en seco con un controlador falso):
- Optimización bayesiana con Optuna (TPE multivariante). Cada ensayo es una vuelta desde parado que se lanza a través del panel, así que se ve en directo.
- J = tiempo + 0.5·e_rms/100 + 0.1·(cambios de w/s). Una vuelta fallida vale 60·(2 − fracción recorrida).
- Arranca con los presets v8 y v9 y cada 10 ensayos repite el mejor para confirmar.
- Se guarda en `logs/optuna/` y es reanudable. El mejor se guarda como preset `optuna_<estudio>`.
- `tools/optimizar.sh --informe` da los mejores ensayos, la importancia de cada parámetro (fANOVA) y las gráficas.
- Configuración en `tools/optimizador/config.json`: espacio de búsqueda, parámetros fijos, pesos y semillas.

Arranque del entorno:
```bash
docker run -d --rm --name robotics --device /dev/dri --gpus all \
  -p 6080-6090:6080-6090 -p 7163:7163 jderobot/robotics-backend:latest
tools/test.sh                                  # pruebas
SJK_V=10 SJK_KP=0.008 tools/vuelta.sh etiqueta 100   # vuelta cronometrada
```

## 6. Registro de progreso

| Fecha | Fase | Resultado / tiempo por vuelta | Notas |
|---|---|---|---|
| 2026-11-04 | — | — | Plan e investigación iniciales |
| 2026-11-04 | 0 | Contenedor arrancado con GPU NVIDIA | Datos de cámara confirmados (5.1). API confirmada (BGR, `WebGUI`, sin límites de V/W, sin penalización). Medido: cámara a 20 Hz, RTF ≈ 0.90, máscara de 1.3 ms → **fase 0 cerrada** |
| 2026-11-04 | 1 | **43.5 s** limpia (v = 10); techo del P: 22.7 s (v = 20, oscilando) | Banco de pruebas automático `tools/vuelta.sh`, ver 5.2 |
| 2026-11-04 | 2 (en curso) | 43.4 s (v = 10, mismo P) | Scanlines + κ por círculo (error ≤ 4 %), preview de 0.25–0.35 s. Problema abierto: el coche ignora `w` pequeña a 10 m/s en algunos tramos. Ver 5.3 |
| 2026-11-05 | 2 → 3 | — | PC nuevo. Bloqueo de giro explicado (fricción de DART alineada con los ejes + pared junto a la salida). Cámara recalibrada (CY = 237.45, H = 0.1015): κ correcta en curvas cerradas. PID del T03 en el controlador y panel de ajuste en vivo (5.4) |
| 2026-11-05 | 3 | **20.2 s** (v8: v = 22, Kp = 0.009, Td = 0.08, k_ff = 0.4) | Ajuste manual en el panel; oscila mucho a v ≥ 19. Mirada adelantada, recuperación automática y optimizador bayesiano preparados (5.5) |
