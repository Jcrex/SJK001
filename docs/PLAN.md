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
└── analysis/               # notebooks y scripts de gráficas
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

## 6. Registro de progreso

| Fecha | Fase | Resultado / tiempo por vuelta | Notas |
|---|---|---|---|
| 2026-11-04 | — | — | Plan e investigación iniciales |
| 2026-11-04 | 0 | Contenedor arrancado con GPU NVIDIA | Datos de cámara confirmados (5.1). API confirmada (BGR, `WebGUI`, sin límites de V/W, sin penalización). Medido: cámara a 20 Hz, RTF ≈ 0.90, máscara de 1.3 ms → **fase 0 cerrada** |
