# RUNME — lanzar la simulación y generar vídeos

*This document is also available in [English](RUNME-en.md).*

Referencia rápida y práctica: cómo poner en marcha la simulación en Gazebo,
y cómo generar un vídeo **aparte**, sin tener que volar a mano — ya sea
para una demo/presentación o como footage para probar un pipeline de
detección (YOLO). No repite lo que ya cuentan otros documentos; solo reúne
los comandos que realmente se usan día a día, con todos sus parámetros.

Si es la primera vez que arrancas el proyecto en esta máquina (instalar
ROS 2, Gazebo, ArduPilot, compilar), sigue primero
[INSTRUCTIONS.md](INSTRUCTIONS.md) (secciones 1–4) o
[docs/GETTING_STARTED.md](docs/GETTING_STARTED.md). Este documento asume que
eso ya está hecho — es lo que necesitas a partir de ahí.

---

## Contenido

0. [Antes de nada: variables de entorno](#0-antes-de-nada-variables-de-entorno)
1. [Lanzar la simulación](#1-lanzar-la-simulación)
   - [1.1 Cuánta proporción de daño usar](#11-cuánta-proporción-de-daño-usar)
2. [Generar un vídeo aparte](#2-generar-un-vídeo-aparte)
   - [2.1 `flight_video.py` — vuelo real, cinematográfico, para demos](#21-flight_videopy--vuelo-real-cinematográfico-para-demos)
   - [2.2 `capture.py --fly` — flythrough headless, sin overlay, para YOLO](#22-capturepy---fly--flythrough-headless-sin-overlay-para-yolo)
   - [2.3 Chuleta: térmico vs RGB, títulos](#23-chuleta-térmico-vs-rgb-títulos)
3. [El dataset de entrenamiento YOLO no es un vídeo](#3-el-dataset-de-entrenamiento-yolo-no-es-un-vídeo)
4. [Si algo falla](#4-si-algo-falla)

---

## 0. Antes de nada: variables de entorno

Todos los comandos de este documento son `ros2 run ...` / `ros2 launch ...`
— y **no van a funcionar en una terminal nueva** si antes no se ha cargado
el entorno de ROS 2 y el propio paquete ya compilado. Si ves `ros2: command
not found` o `Package 'solar_farm_gz' not found`, es justo esto.

Cada terminal nueva necesita, en este orden:

```bash
source /opt/ros/jazzy/setup.bash              # entorno de ROS 2 Jazzy
source ~/solar_farm_sim/install/setup.bash    # este paquete, una vez compilado
```

Para no repetirlo a mano en cada terminal, añádelo una vez a tu `~/.bashrc`:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/solar_farm_sim/install/setup.bash' >> ~/.bashrc
```

(Si todavía no has compilado el paquete ni una vez, hazlo primero — ver
[INSTRUCTIONS.md](INSTRUCTIONS.md) o
[docs/GETTING_STARTED.md, sección 3](docs/GETTING_STARTED.md#3-compilarlo-una-vez).)

Opcional — solo si Gazebo va lento o la ventana sale en negro, típico de un
portátil con gráfica híbrida Intel/NVIDIA:

```bash
echo 'export __NV_PRIME_RENDER_OFFLOAD=1' >> ~/.bashrc
echo 'export __GLX_VENDOR_LIBRARY_NAME=nvidia' >> ~/.bashrc
```

Los bloques de comandos de este documento siguen incluyendo
`source install/setup.bash` de todas formas, por claridad — con las líneas
de arriba ya en tu `~/.bashrc` esa llamada es redundante pero inofensiva
(no da error si se ejecuta de más).

---

## 1. Lanzar la simulación

Dos terminales, ambas con ROS cargado.

**Terminal 1 — simulador con el dron ya generado (spawn) y ambas vistas
abiertas:**

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 launch solar_farm_gz inspection.launch.py
```

Espera a que el mundo termine de cargar (~18 s con el mundo de 1000
paneles).

**Terminal 2 — controlador de vuelo (ArduPilot SITL):**

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --console --map
```

Dale 30–60 s a que el GPS y el EKF se estabilicen. Luego, en la consola de
MAVProxy:

```
mode GUIDED
arm throttle
takeoff 8
```

Para bajar: `mode LAND`. Para volar con mando en vez de por consola, ver
[INSTRUCTIONS.md, sección 7](INSTRUCTIONS.md#7-volar-con-un-mando).

**Opciones de lanzamiento más usadas:**

| Argumento | Por defecto | Significado |
|---|---|---|
| `world` | `solar_farm` | nombre base del fichero de mundo dentro de `worlds/` |
| `headless` | `false` | sin interfaz gráfica — más rápido, útil junto con la sección 2 |
| `drone_x` `drone_y` `drone_z` | `-6 -6 0.13` | posición de aparición (spawn) |
| `drone_yaw` | `0.0` | rumbo de aparición, en radianes |

```bash
ros2 launch solar_farm_gz inspection.launch.py headless:=true
ros2 launch solar_farm_gz inspection.launch.py drone_x:=13.0 drone_y:=-14.0
```

Si solo quieres ver el mundo sin dron: `ros2 launch solar_farm_gz
solar_farm.launch.py`.

### 1.1 Cuánta proporción de daño usar

El dron no decide cuántos paneles están dañados — eso lo fija el mundo que
generaste con `generate_farm.py` (paso 6 de
[docs/GETTING_STARTED.md](docs/GETTING_STARTED.md#6-crea-tus-propias-variaciones-de-dataset)),
a través de `--clean-ratio`. Todo lo que grabes después, con
`flight_video.py` o con `capture.py --fly` (sección 2), o todo lo que
recojas para el dataset de YOLO (sección 3), solo muestra lo que ya hay en
ese mundo.

Para que un vídeo o un dataset *parezcan* la inspección de una instalación
real, **no conviene abusar de los daños**: una instalación bien mantenida
tiene pocos paneles con problemas visibles en un momento dado. Como
referencia razonable, algo en torno a un 85% de paneles en buen estado
(`--clean-ratio 0.85`) da un resultado creíble sin dejar de tener defectos
suficientes para que se noten en el vídeo. Guarda una proporción de daño
mucho más alta para pruebas del detector o demos que quieran enseñar
variedad de defectos en poco tiempo — eso ya no pretende ser realista, y
está bien que no lo sea si el objetivo es ese.

Tres puntos de referencia, de más a menos realista, con el mismo mundo
llevado tanto a `generate_farm.py` como a un vídeo de ejemplo:

**Óptimas — instalación casi sin incidencias, buen mantenimiento:**

```bash
ros2 run solar_farm_gz generate_farm -- --panels 1000 --seed 101 \
    --clean-ratio 0.95 -o src/solar_farm_gz/worlds
colcon build --symlink-install && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 45 -o videos/optimas_rgb.mp4
```

**Buenas — la referencia para una demo o simulación realista (~85% limpio):**

```bash
ros2 run solar_farm_gz generate_farm -- --panels 1000 --seed 42 \
    --clean-ratio 0.85 -o src/solar_farm_gz/worlds
colcon build --symlink-install && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 45 -o videos/buenas_rgb.mp4
```

**Desastrosas — solo para pruebas del detector o una demo de "mira todo lo
que es capaz de detectar"; no pretende ser realista:**

```bash
ros2 run solar_farm_gz generate_farm -- --panels 1000 --seed 7 \
    --clean-ratio 0.45 -o src/solar_farm_gz/worlds
colcon build --symlink-install && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 45 -o videos/desastrosas_rgb.mp4
```

---

## 2. Generar un vídeo aparte

Hay dos herramientas para esto, y cada una sirve a un propósito distinto —
no son intercambiables:

| | `flight_video.py` | `capture.py --fly` |
|---|---|---|
| Para qué | vídeo de demo/presentación | footage crudo para un pipeline (YOLO, revisión de detección) |
| Vista | seguimiento (chase) + nadir incrustado en una esquina | solo la cámara, sin composición |
| Overlay de texto (título, estado) | sí, personalizable | no |
| Arranca su propio Gazebo/ArduPilot | sí (cierra cualquier simulador abierto antes) | sí |
| Vuelo | real, bajo ArduPilot SITL | cámara interpolada entre waypoints (no hay "vuelo") |
| Soporta `--thermal` | sí | sí |

### 2.1 `flight_video.py` — vuelo real, cinematográfico, para demos

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 46 --spawn "13.0,-14,0.13" \
    -o videos/mi_video.mp4
```

Pilota un transecto real con ArduPilot (no es una cámara animada) y graba
la vista de seguimiento con la señal de nadir incrustada y una
superposición de telemetría.

**Recomendado: usa `--route`.** Por defecto (sin `--route`), el dron
vuela en línea recta desde `--spawn` con un rumbo de crucero fijo — para
que el vídeo recorra las filas de mesas, ese rumbo tiene que coincidir
con la orientación real de las filas en el mundo concreto que estés
grabando, y eso no es fiable de un mundo a otro (ver
[docs/ROADMAP.md](docs/ROADMAP.md)). `--route` evita el problema de raíz:
lee las mesas reales del `.sdf` del mundo, construye un recorrido en
zigzag mesa a mesa, y vuela cada tramo por posición GPS absoluta (misma
estrategia que `autonomous_flight.py`), así que el rumbo de spawn deja de
importar.

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --route --route-tolerance 1.0 --route-waypoint-timeout 25 \
    --duration 120 -o videos/mi_video.mp4 --nadir-out videos/mi_video_nadir.mp4
```

**Parámetros:**

| Opción | Por defecto | Significado |
|---|---|---|
| `--world` | *(obligatorio)* | ruta al `.sdf` del mundo |
| `--model` | junto al paquete | `x500_rgb/model.sdf`; normalmente no hace falta tocarlo |
| `--ardupilot` | `~/ardupilot` | checkout de ArduPilot |
| `--plugin-path` | `~/ardupilot_gazebo/build` | build del puente Gazebo↔ArduPilot |
| `--alt` | 8.0 | altitud de crucero, metros |
| `--speed` | 1.5 | velocidad de crucero, m/s |
| `--duration` | 40.0 | segundos de crucero grabados; con `--route`, si la ruta termina antes, el vuelo (y la grabación) acaban ahí en vez de agotar el tiempo con un hover |
| `--route` | desactivado | vuela mesa a mesa por posición GPS absoluta en vez de crucero en línea recta — ver recomendación arriba. Con `--route`, `--spawn` sigue fijando el punto de despegue pero el rumbo deja de importar |
| `--route-tolerance` | 1.0 | metros de tolerancia en X para agrupar mesas en la misma fila (solo con `--route`) |
| `--route-arrival-radius` | 1.5 | metros desde un waypoint que cuentan como "llegado" (solo con `--route`) |
| `--route-waypoint-timeout` | 25.0 | segundos máximos en un waypoint antes de pasar al siguiente igualmente (solo con `--route`) |
| `--spawn` | `3.25,-10,0.13` | posición inicial `x,y,z`; sin `--route`, el rumbo de crucero se fija asumiendo que coincide con la orientación de las filas — no siempre es así (ver recomendación arriba) |
| `--bob-amplitude` | 0.0 | metros de balanceo sinusoidal de sube-baja; 0 lo desactiva. Solo aplica sin `--route` |
| `--bob-pitch` | 11.88 | metros de avance por ciclo de balanceo. Solo aplica sin `--route` |
| `--thermal` | desactivado | **la señal de nadir incrustada usa la cámara térmica simulada (falso color) en vez de RGB** — la vista de seguimiento no cambia |
| `--width` `--height` | 1280 × 720 | resolución de salida |
| `--fps` | 30 | fotogramas por segundo |
| `--inset` | 416 | tamaño en píxeles del recuadro de nadir incrustado |
| `--title-seconds` | 5.0 | cuánto tiempo se ve la banda de título; 0 la desactiva |
| `--env-file` | `.env` | fichero `CLAVE=VALOR` de donde se leen los textos si no se pasan por flag |
| `--title-line1` | *(de `.env` o por defecto)* | línea 1 del título; sobrescribe `.env` |
| `--title-line2` | *(de `.env` o por defecto)* | línea 2; el conteo real de módulos/defectos del mundo se añade automáticamente |
| `--status-label` | *(de `.env` o por defecto)* | etiqueta de estado (esquina) |
| `--fourcc` | `mp4v` | códec de vídeo |
| `-o`, `--out` | `inspection_flight.mp4` | ruta del vídeo de salida |
| `--nadir-out` | desactivado | además del vídeo compuesto de `--out`, escribe la señal de nadir en crudo (resolución nativa, sin recuadro ni HUD) a esta ruta, grabada en el mismo vuelo — la resolución sobre la que entrena el detector, útil para correr inferencia sin la pérdida de nitidez de recortar el recuadro incrustado |
| `--keep` | desactivado | conserva el mundo de captura temporal (útil para depurar) |

**Personalizar el título sin tocar nada en cada comando** — copia
`.env-sample` a `.env` (no versionado, ver `.gitignore`) en la raíz del
proyecto y ajusta los valores:

```bash
# .env
FLIGHT_TITLE_LINE1=EuropeSIP Communications - Inspección Solar con IA
FLIGHT_TITLE_LINE2=Dron Autoconstruido | Raspberry Pi Camera Module 3, nadir
FLIGHT_STATUS_LABEL=Simulacion Ardupilot (SITL-GUIDED)
```

Si una clave falta o el fichero no existe, se usa el valor por defecto
incorporado en el script. Un flag (`--title-line1`, etc.) siempre gana sobre
el `.env`.

**Ejemplos:**

```bash
# demo RGB estándar, título desde .env
ros2 run solar_farm_gz flight_video -- --world worlds/solar_farm.sdf \
    --duration 45 -o videos/demo_rgb.mp4

# misma toma, pero con la cámara térmica simulada
ros2 run solar_farm_gz flight_video -- --world worlds/solar_farm.sdf \
    --duration 45 --thermal -o videos/demo_thermal.mp4

# título distinto solo para esta toma, sin tocar el .env
ros2 run solar_farm_gz flight_video -- --world worlds/solar_farm.sdf \
    --duration 30 \
    --title-line1 "Demo interna - equipo detección" \
    --title-line2 "Cámara térmica simulada" \
    --status-label "PRUEBA" \
    --thermal -o videos/demo_interna_thermal.mp4

# recorrido más largo, punto de partida distinto, sin banda de título
ros2 run solar_farm_gz flight_video -- --world worlds/solar_farm.sdf \
    --duration 60 --spawn "3.25,-10,0.13" --title-seconds 0 \
    -o videos/recorrido_largo.mp4
```

### 2.2 `capture.py --fly` — flythrough headless, sin overlay, para YOLO

Cuando lo que hace falta es footage limpio (sin chase-cam ni texto
superpuesto) para pasarlo por un detector, o una ruta de cámara concreta
que no depende de cómo vuela el controlador:

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz capture -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --fly \
    --path "30,-10,8,0,1.5708,1.5708; 30,110,8,0,1.5708,1.5708" \
    --frames 300 --fps 30 --save-frames \
    -o videos/flythrough_para_yolo.mp4
```

**Parámetros:**

| Opción | Por defecto | Significado |
|---|---|---|
| `--world` | *(obligatorio)* | ruta al `.sdf` del mundo |
| `--fly` | desactivado | modo flythrough (si no, captura una sola imagen fija) |
| `--thermal` | desactivado | la sonda lee el canal térmico simulado (falso color) en vez de RGB — funciona tanto con `--fly` como con una imagen fija |
| `--path` | — | waypoints `x y z roll pitch yaw` separados por `;`, interpolados a ritmo constante |
| `--pose` | `15 15 10 0 0.45 2.2` | pose única, solo para imagen fija (sin `--fly`) |
| `--frames` | 150 | fotogramas totales del flythrough |
| `--fps` | 30 | fotogramas por segundo del vídeo de salida |
| `--rate` | 10.0 | Hz de tiempo de simulación al que actualiza el sensor de cámara |
| `--settle` | 2 | fotogramas descartados tras cada reposición (evita capturar movimiento a medias) |
| `--width` `--height` | 1280 × 720 | resolución |
| `--fov` | 1.05 | campo de visión vertical, radianes |
| `--save-frames` | desactivado | además del vídeo, guarda cada fotograma como PNG en `--outdir` |
| `--outdir` | `frames` | carpeta para los PNG si `--save-frames` |
| `-o`, `--out` | `flythrough.mp4` (o `preview.png` sin `--fly`) | ruta de salida |
| `-v`, `--verbose` | desactivado | más detalle en consola |

**Ejemplo con `--thermal`** (mismo flythrough, canal térmico en vez de RGB):

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz capture -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --fly --thermal \
    --path "30,-10,8,0,1.5708,1.5708; 30,110,8,0,1.5708,1.5708" \
    --frames 300 --fps 30 \
    -o videos/flythrough_termico_para_yolo.mp4
```

`capture.py --thermal` intercambia el material del mundo por su variante
térmica (el mismo mecanismo que `flight_video.py --thermal`, sección 2.1) y
aplica el mismo falso color calibrado — así que el footage de ambas
herramientas es directamente comparable. Ojo, esto no tiene relación con el
topic ROS 2 `/x500_rgb/nadir` del dron real durante un vuelo con ArduPilot
— ese topic siempre es RGB; el canal térmico solo existe en el mundo
renderizado offline por `capture.py` y `flight_video.py`. Si lo que
necesitas son **imágenes fijas térmicas ya etiquetadas** para entrenar, esa
es otra herramienta — ver sección 3.

### 2.3 Chuleta: térmico vs RGB, títulos

- **RGB (por defecto):** no pases `--thermal`.
- **Térmico:** añade `--thermal` a `flight_video.py` (solo afecta al
  recuadro de nadir incrustado; la vista de seguimiento sigue en RGB) o a
  `capture.py --fly` (footage térmico sin overlay, sección 2.2).
- **Título por defecto:** no pases nada — sale de `.env`, o del texto
  incorporado si `.env` no existe.
- **Título fijo para todos los vídeos de este proyecto:** edita `.env`.
- **Título distinto solo para una toma puntual:** usa `--title-line1`,
  `--title-line2`, `--status-label` en ese comando — no toca `.env`.
- **Sin título:** `--title-seconds 0`.

---

## 3. El dataset de entrenamiento YOLO no es un vídeo

Importante para no confundirlo con lo de arriba: el dataset de
entrenamiento no sale de un vídeo grabado. Un vídeo (sección 2) es footage
para demos o para probar un detector ya entrenado contra una toma
continua; el dataset son imágenes fijas + etiquetas YOLO, y **hay dos
formas de generarlo en este proyecto**, con enfoques distintos:

| | `tools/build_quicklook_dataset.py` | `tools/capture_dataset/capture_dataset.py` |
|---|---|---|
| De dónde salen las imágenes | recorta cada módulo directamente del atlas de textura | renderiza tomas reales de cámara, en poses cercanas al nadir, sobre el mundo cargado en Gazebo |
| Cómo se calculan las cajas | ya vienen en `defects.json`, normalizadas al módulo — sin proyección | `tools/capture_dataset/projection.py` reconstruye la posición 3D del defecto y la proyecta con el modelo de cámara real (pinhole, mismo FOV que `x500_rgb`) |
| Encuadre/perspectiva | ninguno — es el módulo "de frente", sin cámara real | el mismo objetivo (66° horizontal, 1920×1080) que la Raspberry Pi Camera Module 3 del dron real, así que un modelo entrenado aquí aplica directo al topic `/x500_rgb/nadir` |
| Modo térmico | no | sí — `--thermal` renderiza el mundo con material térmico (igual que `flight_video.py --thermal`) y etiqueta todo como una única clase `thermal_problem`, porque una cámara térmica no distingue la causa del punto caliente |
| Dataset resultante en este proyecto | `quicklook_dataset/` | `yolo_dataset/` |

Los scripts que **generan** los datasets viven todos en
[`tools/`](tools/README.md), fuera de `yolo_dataset/` y
`quicklook_dataset/` — esas dos carpetas contienen solo datos (imágenes,
etiquetas, `data.yaml`), listos para subir a Colab, Roboflow o donde haga
falta, sin arrastrar código.

`capture_dataset.py` es la herramienta más nueva y la que genera el
dataset "de verdad" pensado para el detector — `build_quicklook_dataset.py`
sigue siendo útil como comprobación rápida, pero al no pasar por una
cámara real no representa el punto de vista con el que el dron realmente
inspecciona.

**Los tres scripts de `tools/capture_dataset/`:**

- **`capture_dataset.py`** — el que realmente genera el dataset: carga un
  mundo generado, dispara `--n` tomas en poses aleatorias (sesgadas hacia
  mesas dañadas), y escribe cada imagen junto a su `.txt` de etiquetas YOLO.
- **`projection.py`** — la geometría pura: reconstruye en 3D dónde está
  cada defecto (a partir de la pose de la mesa y el índice de módulo) y lo
  proyecta a la caja 2D que vería la cámara en esa pose concreta. No se
  ejecuta suelto; lo importa `capture_dataset.py`.
- **`pick_spawn.py`** — utilidad aparte, para `flight_video.py`: calcula un
  punto de aparición (spawn) cerca del centro del parque pero que caiga en
  un hueco real entre dos mesas, para que el dron no aparezca encima de una
  mesa y se estrelle nada más generarse.

**Ejemplo — generar más imágenes para un site ya existente:**

```bash
cd ~/solar_farm_sim/src/solar_farm_gz
export PYTHONPATH="$PWD:$PYTHONPATH"

# RGB (4 clases: dirt, bird_dropping, crack, delamination)
python3 ../../tools/capture_dataset/capture_dataset.py \
    --world-dir /ruta/a/worlds/site_g --site site_g --n 40 --seed 42 \
    --images-out /ruta/a/salida/images --labels-out /ruta/a/salida/labels

# térmico (1 clase: thermal_problem), mismo site
python3 ../../tools/capture_dataset/capture_dataset.py \
    --world-dir /ruta/a/worlds/site_g --site site_g --n 40 --seed 77 --thermal \
    --images-out /ruta/a/salida/images --labels-out /ruta/a/salida/labels
```

El detalle completo — composición del dataset actual (460 imágenes, 5
sites, RGB+térmico), por qué el térmico usa una sola clase, cómo se validó
la proyección, y cómo entrenar con Ultralytics — está en
[docs/YOLO_DATASET.md](docs/YOLO_DATASET.md), que es más específico
que este documento para todo lo relacionado con el dataset en sí.
Referencia completa de los scripts en [tools/README.md](tools/README.md).

El método más simple de `tools/build_quicklook_dataset.py` (recorte
directo del atlas, sin cámara) sigue documentado en
[docs/MANUAL.md, sección 10](docs/MANUAL.md#10-construir-un-dataset-de-entrenamiento).

---

## 4. Si algo falla

- El dron no arma, el mando no responde, el puente no compila, o Gazebo va
  lento: tabla completa de síntomas/soluciones en
  [INSTRUCTIONS.md, sección 10](INSTRUCTIONS.md#10-si-algo-no-funciona) y
  en [docs/MANUAL.md, sección 13](docs/MANUAL.md#13-solución-de-problemas).
- `flight_video` o `capture` fallan al arrancar: asegúrate de haber cerrado
  cualquier otro Gazebo/ArduPilot en marcha antes — ambas herramientas
  arrancan los suyos propios.
- Paneles grises sin textura: el mundo no se recompiló tras generarse —
  `colcon build --symlink-install` y vuelve a cargar (`source`).
