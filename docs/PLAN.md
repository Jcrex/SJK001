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

### 5.6 Banda del horizonte (2026-11-05, por probar en el simulador)

Idea: en las curvas, los puntos más altos de la línea ya muestran hacia dónde gira la
pista antes de que el error cercano cambie. Se usa una banda de `n_filas_hor` filas
(10 por defecto) que empieza en `fila_hor` (240, la primera con suelo visible; la
cámara corta a 15 m en la fila ≈ 239.6). En cada fila de la banda se busca la línea,
siguiéndola desde la última scanline de abajo y descartando saltos > 50 px. La
columna media respecto al centro es `e_hor`. Término de control (prealimentación,
fuera de la D y saturado con el resto en `w_max`):

    w_hor = −k_hor · e_hor        (k_hor = 0 → desactivado, comportamiento de antes)

En la imagen de la web, la banda se pinta en azul transparente, los puntos de la banda
en cian y la columna media con una raya cian. En el panel, `e_hor` aparece en la
gráfica de error y `HOR` en la de giro. Medido con 35 fotogramas reales: en la
entrada de una curva, `e_px` ≈ +8 px mientras `e_hor` ≈ −174 px (curva a la
izquierda); en recta, |e_hor| < 10 px; coste de +0.2 ms por fotograma.
Límite conocido: si el coche va muy torcido y la línea se sale por el lateral de la
imagen, las scanlines lejanas (y con ellas la banda) pueden saltar a otro tramo del
circuito que asome por el horizonte.

### 5.7 Estudio en caliente: proponer ensayos al optimizador desde el panel

- **Parámetros nuevos sin perder lo aprendido:** se crea un estudio nuevo que importa los
  ensayos del anterior (`importar` en `config.json`). Al parámetro nuevo se le da el valor
  que tenía de verdad (k_hor = 0). Así se pasó de `simple_v1` a `simple_v2`.
- **Modo borrador:** con el optimizador en marcha, los sliders y los presets no tocan el
  coche. Los cambios quedan en un borrador (en naranja en el panel), y «Probar en el
  optimizador» lo mete en la cola del estudio. Se ensaya tras la vuelta en curso y el
  modelo aprende de él como de los suyos (`user_attrs.manual = 1`). Solo cuentan los
  parámetros del espacio de búsqueda; los demás se ignoran, con aviso.
- **simple_v3 (rangos ampliados):** en `simple_v2` el mejor resultado fue 15.1 s (ensayos
  manuales #325 y #327, repetido por el optimizador en el #332): v_max = 30, k_hor = 0.008,
  kp = 0.0096, td = 0.111, N = 14.8, k_ff = 0.46. Los 20 mejores estaban pegados a los
  límites del espacio: v_max 30 (14 de 20), k_hor 0.008 (20), n_filtro 15 (19),
  z_mirar_min 0.5 (15). `simple_v3` importa `simple_v2` y amplía los rangos: v_max ≤ 40
  (en el controlador ≤ 45), k_hor ≤ 0.02, td ≤ 0.25, n_filtro ≤ 30, z_mirar_min ≥ 0.2.
  w_max sigue fijo en 6, porque en las vueltas a v = 30 solo satura el 0.4 % del tiempo.
- **Sin techo y ventana móvil:** el controlador ya no recorta ningún parámetro por arriba
  (salvo `reinicio_auto`); el máximo de PARAMS es solo el recorrido inicial del slider.
  En el optimizador, los rangos de `espacio` son una ventana que se desplaza. Cada 10
  ensayos se miran los 10 mejores: si la mitad están a menos del 10 % de un borde, la
  ventana se desplaza un 25 % del ancho hacia ese lado. También se desplaza si desde el
  panel se pide un valor de fuera. Al moverse, la ventana deja a los mejores lejos del
  borde contrario para que no rebote. Cada desplazamiento crea una generación del estudio
  (`<estudio>_g<n>`) que importa los ensayos que caen dentro. El estado está en
  `logs/optuna/<estudio>_ventana.json`, y `--informe` lee la generación actual.
- **Mejores por repeticiones:** el mejor del estudio era la mejor vuelta suelta, así que el
  #595 (9.35 s) se confirmó 24 veces aunque solo completó 2. Ahora los parámetros repetidos
  se agrupan y se ordenan por la J media de todas sus repeticiones (los choques, ~75,
  incluidos). Con ese orden se eligen las confirmaciones (el primero de los 5 mejores con
  menos de `confirmar_hasta` = 3 repeticiones), el preset `optuna_<estudio>`, los «mejores»
  que mueven la ventana y el «mejor» del panel. El informe ordena igual.
- **Protección:** si los parámetros cambian durante un ensayo, o se pulsa «Volver a la
  salida», el ensayo se descarta (`interferencia`) en vez de guardarse con datos falsos.

### 5.8 Resultados de simple_v3 y estado al cerrar la sesión (2026-11-05)

**Entrenamiento:** `simple_v3`, de 13:30 a 18:30 aprox., terminó en la generación `g6` con 1176 ensayos
(627 vueltas completas). La ventana se desplazó 6 veces:

| Gen. | Motivo | Cambio |
|---|---|---|
| g1, g2 | pedido desde el panel | k_hor → 0–0.18 → 0–0.96 |
| g3–g6 | los mejores estaban en el borde | v_max 16–40 → 22–46 → 28–52 → 34–58 → 40–64 |

- **Mejor fiable: 9.25–9.3 s en 3 de 3 vueltas** (#1054, J media 10.61; guardado como preset
  `optuna_simple_v3`): v_max = 56.4, k_v_curva = 6.51, v_min = 10.8, kp = 0.00771, td = 0.0836,
  n_filtro = 15.2, k_ff = 0.765, kv_kp = 0.023, t_mirar = 0.175, z_mirar_min = 0.316, k_hor = 0.0455
  (w_max = 6, ti = 0). Hay 7 grupos de parámetros con 3/3 vueltas entre 9.25 y 9.4 s, todos muy
  parecidos: la zona buena está bien localizada.
- **v_max se ha estabilizado en ≈ 55–57** con la ventana en 40–64: los mejores ya no están en el borde,
  así que el límite ya no es la ventana sino el circuito (frenar en las curvas).
- **Importancia (fANOVA) al final:** v_max 0.21, k_ff 0.15, k_v_curva 0.15, v_min 0.14, z_mirar_min 0.09,
  td 0.09; t_mirar, kp y k_hor ≈ 0.04–0.05. Al principio de g5 mandaban v_max, t_mirar y k_hor. Ahora, con
  la anticipación ya ajustada, lo que más mueve J es **el perfil de velocidad** (cuánto frenar en curva)
  y la prealimentación.
- k_hor terminó en ≈ 0.045–0.055, unas 6 veces el 0.008 de la vuelta de 15.1 s. Los valores grandes
  (hasta 0.96, pedidos desde el panel para probar) fallaban.

**Corrección del optimizador: mejores por repeticiones.** Con el mejor como mejor vuelta suelta, el #595
(9.35 s) se confirmó 24 veces y solo completó 2. Desde el commit 43139c8 los grupos de parámetros se ordenan
por la J media de todas sus repeticiones (ver 5.7). Tras el cambio, las confirmaciones dejaron 8 grupos
3/3 en ~270 ensayos, frente a uno solo antes.

**Estudio de rendimiento: ¿pasar el controlador a C++?** No compensa. Medido en el log de la sesión
(23 841 fotogramas): dt = 50 ms en el 99 % (la cámara está fijada a 20 Hz en `f1_gz.urdf.xacro`) y
no se pierde ningún fotograma. La percepción tarda 1.25 ms de media (0.78 ms en banco), el control
≈ 0 y dibujar para la web 1.2 ms, pero después de mandar v y w. C++ ahorraría ≈ 1 ms de retardo
(≈ 4 cm a 45 m/s, frente a 2.25 m entre fotogramas). El simulador sí acepta C++ (colcon, paquete `academy`),
pero sin HAL ni WebGUI habría que reescribir la cámara, los comandos, la imagen web, el reinicio y la
telemetría. **El cuello de botella es la cámara a 20 Hz**, que es parte del ejercicio. Revisar solo si
t_perc se acerca a 50 ms o aparecen dt de 0.1 s.

**Otros cambios de la sesión:** `.gitignore` (logs, estudios y presets generados fuera del repo);
README con la ecuación de control completa en LaTeX, el diagrama del lazo (`docs/img/lazo_control.svg`)
y las gráficas del optimizador (`docs/img/optuna_*.png`, copiadas del informe de `simple_v3_g6`).

**Siguiente paso propuesto:**
1. Comprobar a mano el preset `optuna_simple_v3` desde la web y ver la vuelta.
2. Como ahora lo que más pesa es el perfil de velocidad, se puede liberar `a_sub` y `a_fren` (fijos a 0,
   sin límite) o `w_max` en un estudio nuevo que importe `simple_v3_g6`.
3. En las gráficas de cortes, J sigue bajando hacia `v_min` ≈ 9.5 y `z_mirar_min` ≈ 0.2 (borde inferior
   de sus ventanas). La ventana móvil los bajará sola si se sigue entrenando.
4. Probar en otros circuitos para ver si generaliza: la ventana y el preset están afinados para `simple`.

### 5.9 Macroanálisis, giro máximo y `simple_v4` (2026-11-06)

Detalle completo, con cifras y figuras, en [`ESTUDIO_PID.md`](ESTUDIO_PID.md) (§0–9 para `simple_v3`, §10 para el
barrido de `w_max` y §11 para `simple_v4`). Lo regenera `tools/optimizador/macro_analisis.py`.

- **Macroanálisis de `simple_v1`–`v3`** (2076 ensayos y 1827 vueltas con telemetría):
  - El tiempo lo decide `v_max`; si la vuelta se completa lo decide `k_v_curva`; la oscilación la decide `k_hor`.
  - `kp`, `td`, `n_filtro`, `kv_kp` y `z_mirar_min` son planos dentro de sus ventanas.
  - P + D aportan solo el ~14 % del giro; la prealimentación y el horizonte, el resto.
  - El 91 % de los fallos ocurre en las dos chicanes, con el giro saturado en `w_max` = 6.
  - Con los mismos parámetros, el tiempo se repite con ±0.025 s, pero completar o no se repite solo la mitad de las
    veces en el borde.
- **Barrido de `w_max`** (`tools/barrido.sh`, 8 valores × 20 vueltas sobre el preset de 8.95 s):
  - El simulador obedece giros por encima de 6.
  - Con 7 o más se completan 140 de 140 vueltas, y la saturación desaparece a partir de 9.
  - El tiempo no cambia, porque el perfil de velocidad era el mismo en todos los valores.
- **`simple_v4`** (`config_v4.json`): w_max = 12, sin importar `simple_v3` y con ventanas centradas en su mejor.
  - En 273 ensayos llega a **6.4 s en 3 de 3** (#194) y a 6.25 s de mejor vuelta, con un 94 % de vueltas completas.
  - Casi no frena en curva: `k_v_curva` ≈ 2 y la v mínima ≈ 57 m/s.
  - `k_ff` baja a ≈ 0.6.
- **Cronómetro del panel:**
  - El paso por meta se busca ahora en el tramo entre dos fotogramas y el instante se interpola. A más de ~70 m/s el
    coche avanza más de 3.5 m por fotograma y el círculo de 2.5 m podía saltarse. Con las vueltas de v4 da las mismas
    257 completas y +0.011 s de media.
  - Todos los tiempos incluyen 0.35 s de reinicio del simulador. Es constante.
- **Barrido de `v_max`** sobre el preset de `simple_v4` (6 valores × 20 vueltas):
  - 82 m/s → **5.92 s** de mediana (18/20); 86 m/s → 5.67 s, pero solo 55 % de vueltas completas.
  - El tiempo ≈ 426 m / (0.94·v_max) + 0.35 s.
  - Límites: la cámara (12.7 m de alcance, 3 fotogramas a 86 m/s) y la saturación del giro corrector en las chicanes.
- **Barrido de `w_max` a 75 m/s** (12, 14, 16 y 20; 20 vueltas cada uno):
  - El simulador obedece giros de hasta ~16.5 rad/s, y la saturación baja del 8 % al 0 %.
  - El tiempo mejora solo 0.02 s y las vueltas completas no cambian (80–90 %).
  - La telemetría de los 49 fallos con w_max ≥ 12 señala la **banda del horizonte**: en el último medio segundo pone
    el 65 % del giro (48 % en vueltas completas), y su salto entre fotogramas crece con v (p95 de 59 px a 60–70 m/s y
    de 131 px a 80–84 m/s).
  - Siguiente: ajustar `k_hor` a 82–84 m/s, escalarlo con v o filtrar `e_hor` (ESTUDIO_PID.md §13).

### 5.10 Estado al cerrar la sesión (2026-11-06, 02:50) y siguiente sesión

**Estado:**
- **Mejor fiable:** 5.92 s de mediana (18/20) con el preset `optuna_simple_v4` y v_max = 82. Tiempos con 0.35 s de
  reinicio incluidos.
- **Mejor del optimizador:** `simple_v4` #194, 6.40–6.45 s en 3/3 (v_max = 75.1). Parado en `simple_v4_g7` tras
  273 ensayos.
- **Parámetros:** en la tabla del README (§4). Ojo: `tools/panel/presets/optuna_*.json` está en `.gitignore`, así que
  los presets `optuna_simple_v4` y `optuna_simple_v3` solo existen en este PC.
- **Límite actual:** la banda del horizonte a ≥ 80 m/s (ESTUDIO_PID.md §13). `w_max` ya no limita, y la cámara
  (12.7 m de alcance) es el muro de fondo.
- **Datos de los barridos:** `logs/optuna/barrido_w_max_20261106-004141.csv` (6–13 a 60 m/s),
  `barrido_v_max_20261106-020516.csv` (76–86) y `barrido_w_max_20261106-021947.csv` (12–20 a 75 m/s). Están en `logs/`,
  que no se versiona.
- **Procesos:** el panel, el optimizador y los barridos están parados. El contenedor `robotics` sigue en marcha.

**Para arrancar:**
1. Si el contenedor no está: `docker run …` (README §5). Después, `tools/panel.sh`.
2. Pegar `src/follow_line.py` en http://localhost:7163 y pulsar play.
3. Cargar el preset `optuna_simple_v4` en el panel y comprobar una vuelta (~6.4 s a v_max = 75.1).

**Siguientes pasos, por orden:**
1. **Barrer `k_hor` a v_max = 84**, porque el valor actual (0.064) se ajustó a 75:
   `tools/barrido.sh --base optuna_simple_v4 --param k_hor --valores 0.03,0.04,0.05,0.064 --fijar w_max=12,v_max=84`
2. **Según lo que salga**, escalar `k_hor` con la velocidad o filtrar `e_hor` en `src/follow_line.py`. Hay que tocar el
   controlador, añadir pruebas en `tests/test_control.py` y medir con un barrido igual que el anterior.
3. **Estudio `simple_v5`:** copiar `config_v4.json`, con `v_max` en [80, 92], `w_max` = 12–16 y ventanas estrechas
   alrededor del mejor. Repetir cada candidato al menos 5 veces, porque a ≥ 80 m/s 3 de 3 no garantiza fiabilidad.
4. **Regenerar el macroanálisis:** `macro_analisis.py`, y después §11–13 del estudio y el README. Pendiente: que las
   figuras 02–05 y 08 (importancia, interacciones, palancas) se calculen también para `simple_v4` y no solo para
   `simple_v3`.
5. **Probar otros circuitos**, para ver si los parámetros generalizan fuera de `simple`.

### 5.11 Barridos de `k_hor` y `v_max`, filtro del horizonte descartado y `simple_v5` (2026-11-06, tarde)

- **`k_hor` a v_max = 84:** 0.04 da 20/20 (5.92 s) y 0.05 da 19/20 (5.87 s); 0.03 no anticipa (6/20) y 0.064 oscila
  (15/20). El óptimo baja con la velocidad.
- **`v_max` con k_hor = 0.045:** 86 → **5.76 s en 20/20**; 88 → 5.66 s (16/20); 90 → 5.56 s (15/20). Los choques se
  concentran en la curva de los ~350 m.
- **Filtro de `e_hor` (`tau_hor`) descartado:** 0/20 vueltas con τ = 0.05–0.15 a 88 m/s. El horizonte forma parte del
  lazo de giro y retrasarlo lo vuelve inestable. Se quitó del controlador. Detalle en ESTUDIO_PID.md §14.
- **`simple_v5` lanzado** con `tools/optimizador/config_v5.json`: v_max en [84, 92], k_hor en [0.035, 0.06], resto ±20 %
  alrededor de #194, confirmación con 5 repeticiones. Semillas: presets `w12_v86_kh045`, `w12_v90_kh045` y
  `optuna_simple_v4`.
- **Cargar el código:** desde unibotics.org (ejercicio Follow Line, simulador local). localhost:7163 no es una página.

**Siguientes pasos:**
1. Analizar `simple_v5` (`tools/optimizar.sh --informe simple_v5`) y confirmar el mejor con un barrido de 20 vueltas.
2. Si el k_hor óptimo sigue bajando con v_max, escalarlo con la velocidad dentro de la vuelta (k_hor / (1 + k·v)).
3. Mirar la curva de los ~350 m en la telemetría: es donde fallan casi todas las vueltas a ≥ 88 m/s.
4. Regenerar el macroanálisis y probar otros circuitos (pasos 4–5 de §5.10).

### 5.12 `simple_v5` y siguiente sesión (2026-11-06, cierre a las 17:00)

**Estado:**
- **Mejor:** 5.25 s en 4/4 (`simple_v5` #775, v_max = 95.5, k_hor = 0.044, k_v_curva = 2.63), preset
  `w12_v95_n775`. Mejor vuelta: 5.18 s. **Mejor fiable con 20 vueltas:** 5.76 s en 20/20 (preset `w12_v86_kh045`).
- **Optimizador:** `simple_v5` llegó a `simple_v5_g6` (1338 ensayos, ventana de v_max en 93–102). A ≥ 96 m/s completa
  menos de la mitad de las vueltas. Dejarlo parado al empezar.
- **Límite:** la chicane norte (350–360 m), con el 67 % de los fallos. El giro va saturado en w_max = 12 la mitad del
  tiempo en esa curva (ESTUDIO_PID.md §15).
- **Macroanálisis:** regenerado con los 3875 ensayos. `macro_analisis.py` ya incluye `simple_v5` y calcula las figuras
  02–05 y 08 también para w_max = 12 (sufijo `_w12`). La tarea pendiente de §5.10, paso 4, está hecha, salvo
  `03_dependencia_parcial_w12.png`: se generó con una escala de tiempo que oculta las curvas. El script ya está
  corregido, pero hay que volver a ejecutar `macro_analisis.py` (se cortó al apagar el PC).
- **Procesos:** al cerrar seguían en marcha el panel, el optimizador y el contenedor `robotics`.

**Para arrancar:**
1. Si el contenedor no está: `docker run …` (README §5). Después, `tools/panel.sh`.
2. En unibotics.org (Follow Line, simulador local), pegar `src/follow_line.py` y pulsar play.
3. Comprobar una vuelta con el preset `w12_v95_n775` (~5.25 s).

**Siguientes pasos, por orden:**
1. **Barrido de `w_max` sobre #775** (20 vueltas por valor):
   `tools/barrido.sh --base w12_v95_n775 --param w_max --valores 12,14,16,18`
2. **Barrido de `k_v_curva` sobre #775**, por si basta con frenar algo más en la chicane norte:
   `tools/barrido.sh --base w12_v95_n775 --param k_v_curva --valores 2.6,2.8,3.0,3.2 --fijar w_max=<el mejor del paso 1>`
3. **Estudio `simple_v6`:** copiar `config_v5.json` con el w_max y la ventana de k_v_curva que salgan de los barridos,
   v_max desde ~93, e importar `simple_v5` solo si w_max no cambia.
4. **Confirmar el mejor con 20 vueltas** antes de anotarlo como fiable: a ≥ 92 m/s solo el 64 % de los que completan
   la primera vuelta completan las repeticiones.
5. Pendiente de antes: escalar `k_hor` con la velocidad solo si su óptimo vuelve a bajar (de 88 a 96 m/s se mantuvo en
   0.040–0.044), y probar otros circuitos.

### 5.13 Barridos sobre #775 y `simple_v6` (2026-11-07)

**Hecho:**
- Barrido de `w_max` sobre #775: **14 es el mejor** (13/20). 12 falla en la chicane norte (saturación), 16–18 en la
  primera curva (~43 m).
- Barridos de `k_v_curva` con w_max = 14 (1.2–3.2): frenar menos acelera y no pierde fiabilidad hasta ~1.6.
  **5.19 s en 34/40** con 1.8 (preset `w14_v95_kvc18`) y **5.12 s en 15/20** con 1.2 (preset `w14_v95_kvc12`).
  Detalle en ESTUDIO_PID.md §16.
- `macro_analisis.py` regenerado (la figura `03_dependencia_parcial_w12.png` ya sale bien) y corregido para ignorar
  barridos sin vueltas. `barrido.py` repite las vueltas con «timeout» (play cortado), igual que las interrumpidas.
- Lanzado **`simple_v6`** (`config_v6.json`): w_max = 14 fijo, k_v_curva en [1.0, 2.2], v_max en [93, 100], ±20 %
  alrededor de #775 en el resto, semillas `w14_v95_kvc18` y `w14_v95_kvc12`.

**Siguientes pasos:**
1. Dejar correr `simple_v6` y revisarlo con `tools/optimizar.sh --informe simple_v6`.
2. Confirmar su mejor con un barrido de 20 vueltas (por ejemplo, de `v_max` alrededor de su valor) antes de darlo por
   fiable. Con 20 vueltas, ±10–15 puntos de fiabilidad son ruido: juntar tandas para comparar.
3. Primera curva: probar a ver la curva antes (`t_mirar`, `z_mirar_min`) si sigue siendo el fallo principal.

### 5.14 Fiabilidad, `simple_v6`–`v7` y cierre: parámetros elegidos (2026-11-07/08)

**Hecho:**
- `simple_v6` (719 ensayos): ~5.0 s de mediana a v_max ≈ 99, pero 60 % de vueltas completas. Preset `w14_v99_v6`.
- Objetivo cambiado por el usuario a **fiabilidad y estabilidad**. Barridos de v_max (86–99.7) y t_mirar sobre la
  combinación de `simple_v6`: 92–93 m/s es el punto dulce (57/60); por encima falla la chicane norte y por debajo la
  primera curva.
- `simple_v7` (1009 ensayos): v_max fija en 93, J con más peso a la estabilidad y repetición inmediata de cada
  candidato (opción `repetir` del optimizador). Mejor: 10/10 a 5.30 s con e_rms 213.
- Corregido el cronómetro del panel (vueltas falsas por el teletransporte del reinicio y meta fijada en la salida).
- **Macroanálisis final** (`macro_analisis.py`, régimen w_max = 14 y sección de decisión, ESTUDIO_PID.md §18).
  Figura 11 con el peso de cada parámetro en el tiempo, la fiabilidad y la oscilación para los tres regímenes de
  w_max (resumen final de ESTUDIO_PID.md), y telemetría sin las vueltas falsas del cronómetro. Gráficas del
  optimizador del README regeneradas con `simple_v7_g3`.
- **Elegida `w14_v93_v6`: 63/65 vueltas (97 %, cota inferior del 95 %: 89 %) a 5.29 s de mediana.** Es la de mayor
  fiabilidad demostrada de las 4599 combinaciones y queda como valor por defecto de `src/follow_line.py` (el control P
  de las fases 1–2 sigue disponible como `FASE1` / `params_fase1()`).

**Si se retoma:**
1. Confirmar con ≥ 40 vueltas la combinación elegida con td ≈ 0.07 y k_hor ≈ 0.04 (ESTUDIO_PID.md §18, pendiente).
2. Probar otros circuitos con los valores por defecto.

### 5.15 Tandas de 5 vueltas seguidas: `simple_v8`–`v9` (2026-11-10)

**Hecho:**
- `simple_v8` (`config_v8.json`, 34 ensayos de una vuelta): v_max libre en 93–97 con la J de `simple_v7`. Se paró
  enseguida para cambiar de prueba.
- **Cambio de prueba pedido por el usuario:** fiabilidad en **5 vueltas seguidas sin reiniciar**. `optimizar.py` tiene
  la opción `vueltas_seguidas`: un reinicio y N vueltas encadenadas. La J usa la media por vuelta y, si falla, la
  distancia recorrida en toda la tanda.
- `simple_v9` (`config_v9.json`, 576 tandas):
  - el 41 % acaban las 5;
  - las vueltas lanzadas tardan **4.97 s**;
  - las lanzadas fallan en la chicane sur (70–80 m y 110–120 m), y la 1.ª, desde parado, en la primera curva (40–50 m).
- Macroanálisis actualizado (sección 11 de `macro_analisis.py`, figuras 12–14, ESTUDIO_PID.md §19). Mejor
  combinación en tandas: **#271, 4/4 tandas y 20/20 vueltas** a 4.97 s por vuelta lanzada (preset `w14_v92_v9`).
  Su vecindad da el 92 % por vuelta y el 68 % de tandas 5/5.
- Los valores por defecto de `src/follow_line.py` siguen siendo `w14_v93_v6`.

**Si se retoma:**
1. Añadir tandas a `barrido.sh` y comparar `w14_v92_v9` con `w14_v93_v6` en ≥ 10 tandas cada uno. Cambiar el valor
   por defecto solo si `w14_v92_v9` gana.
2. Estudiar en la telemetría cómo cruza la meta el coche en las vueltas lanzadas (chicane sur).

## 6. Registro de progreso

| Fecha | Fase | Resultado / tiempo por vuelta | Notas |
|---|---|---|---|
| 2026-11-04 | — | — | Plan e investigación iniciales |
| 2026-11-04 | 0 | Contenedor arrancado con GPU NVIDIA | Datos de cámara confirmados (5.1). API confirmada (BGR, `WebGUI`, sin límites de V/W, sin penalización). Medido: cámara a 20 Hz, RTF ≈ 0.90, máscara de 1.3 ms → **fase 0 cerrada** |
| 2026-11-04 | 1 | **43.5 s** limpia (v = 10); techo del P: 22.7 s (v = 20, oscilando) | Banco de pruebas automático `tools/vuelta.sh`, ver 5.2 |
| 2026-11-04 | 2 (en curso) | 43.4 s (v = 10, mismo P) | Scanlines + κ por círculo (error ≤ 4 %), preview de 0.25–0.35 s. Problema abierto: el coche ignora `w` pequeña a 10 m/s en algunos tramos. Ver 5.3 |
| 2026-11-05 | 2 → 3 | — | PC nuevo. Bloqueo de giro explicado (fricción de DART alineada con los ejes + pared junto a la salida). Cámara recalibrada (CY = 237.45, H = 0.1015): κ correcta en curvas cerradas. PID del T03 en el controlador y panel de ajuste en vivo (5.4) |
| 2026-11-05 | 3 | **20.2 s** (v8: v = 22, Kp = 0.009, Td = 0.08, k_ff = 0.4) | Ajuste manual en el panel; oscila mucho a v ≥ 19. Mirada adelantada, recuperación automática y optimizador bayesiano preparados (5.5) |
| 2026-11-05 | 3 | **15.1 s** (v = 30, k_hor = 0.008, Kp = 0.0096, Td = 0.111, N = 14.8, k_ff = 0.46) | Banda del horizonte + optimizador con ensayos manuales en caliente (simple_v2). Rangos ampliados en simple_v3 (5.7) |
| 2026-11-05 | 3 | 9.4 s (#828, v ≈ 54.5, 6/7 vueltas) | Sin techo en el controlador y ventana móvil: v_max sube por generaciones (g3–g5). Detectado que el «mejor» era una vuelta con suerte (#595, 2/24) (5.7) |
| 2026-11-05 | 3 | **9.25 s, 3 de 3 vueltas** (#1054: v_max = 56.4, kp = 0.0077, td = 0.084, k_ff = 0.77, k_hor = 0.046, t_mirar = 0.18) | Mejores por J media de las repeticiones. Optimizador parado en simple_v3_g6 (1176 ensayos). C++ descartado: el límite es la cámara a 20 Hz. README con la ecuación y el diagrama (5.8) |
| 2026-11-06 | 3 | 8.95 s, 3 de 3 (#1676: v_max = 60, k_v_curva = 7.6, k_ff = 0.91) | Macroanálisis de todos los ensayos (ESTUDIO_PID.md). Barrido de w_max 6–13: 140/140 vueltas con w_max ≥ 7, mismo tiempo (5.9) |
| 2026-11-06 | 3 | **6.4 s, 3 de 3** (#194 de simple_v4: v_max = 75.1, k_v_curva = 2.2, k_ff = 0.60, w_max = 12) | Estudio simple_v4 con w_max = 12: 94 % de vueltas completas. Cronómetro con paso por meta interpolado (5.9) |
| 2026-11-06 | 3 | **5.92 s** (mediana, 18/20; mismos parámetros con v_max = 82) | Barrido de v_max 76–86 (20 vueltas por valor). Tiempo ≈ 426/(0.94·v_max) + 0.35 s (5.9) |
| 2026-11-06 | 3 | 6.35 s (w_max = 20, v_max = 75.1; −0.02 s frente a w_max = 12) | Barrido de w_max 12–20 a 75 m/s: sin cambio de fiabilidad. Nuevo límite: la banda del horizonte sin filtrar a ≥ 80 m/s (5.9) |
| 2026-11-06 | 3 | **5.76 s** (mediana, 20/20; v_max = 86, k_hor = 0.045) | Barrido de k_hor a 84 (óptimo 0.04–0.05) y de v_max 84–90 (5.56 s a 90, 15/20). Filtro de e_hor descartado (0/20). Estudio simple_v5 lanzado (5.11) |
| 2026-11-06 | 3 | **5.25 s, 4 de 4** (#775 de simple_v5: v_max = 95.5, k_hor = 0.044, k_v_curva = 2.63); mejor vuelta 5.18 s | 1338 ensayos (g0–g6). Techo en la chicane norte (67 % de los fallos, giro saturado la mitad del tiempo). Macroanálisis regenerado con figuras para w_max = 12 (5.12) |
| 2026-11-07 | 3 | **5.19 s** (mediana, 34/40; #775 con w_max = 14 y k_v_curva = 1.8) · 5.12 s con k_v_curva = 1.2 (15/20) | Barridos de w_max (14 mejor) y k_v_curva (1.2–3.2) sobre #775. Lanzado simple_v6 (5.13) |
| 2026-11-08 | 3 → cierre | **5.29 s, 63 de 65 vueltas (97 %)** (v_max = 93, w_max = 14; preset `w14_v93_v6`, valores por defecto del controlador) · más rápido: ~5.0 s con el 60–76 % (simple_v6, v ≈ 99) | Objetivo cambiado a fiabilidad. Barridos de v_max y t_mirar, simple_v7 (repetición inmediata, J con más peso a la estabilidad), cronómetro corregido y macroanálisis final con decisión por cota de Wilson (5.14) |
| 2026-11-10 | 3 | **4.97 s por vuelta lanzada, 20/20 vueltas en 4 tandas de 5 seguidas** (#271 de simple_v9, v_max = 92.2; preset `w14_v92_v9`) · zona: 92 % por vuelta, 68 % de tandas 5/5 | Nueva prueba: 5 vueltas seguidas sin reiniciar (`vueltas_seguidas` en el optimizador). simple_v9: 576 tandas, 41 % 5/5. Macroanálisis con sección de tandas (5.15) |
