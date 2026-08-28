# El dataset YOLO (`yolo_dataset/`)

Dataset de detección de objetos en formato YOLO para la inspección de
defectos en paneles fotovoltaicos, construido a partir de este proyecto
`solar_farm_sim` (suelo de hierba/tierra, valla perimetral, estaciones de
inversor, dron X500 real pilotado con ArduPilot).

460 imágenes (230 RGB + 230 térmicas), 3152 cajas, procedentes de 5 sitios ×
2 modos de captura. Además, 8 vídeos de vuelo (vuelos reales de ArduPilot
SITL, con distintas semillas, algunos con una señal térmica simulada) para
pruebas cualitativas — ver [Vídeos](#vídeos).

Este documento (la explicación completa) vive aquí, en `docs/`, para que
`yolo_dataset/` se pueda subir a Colab/Roboflow/etc. tal cual, sin
documentación extensa ni scripts mezclados. Lo único que hay dentro de
`yolo_dataset/` además de los propios datos es un `README.md` breve que
enlaza de vuelta a este documento — así la carpeta no queda huérfana si se
separa del resto del repositorio.

## Contenido

```
yolo_dataset/                      los datos, listos para subir tal cual
  README.md                        puntero breve a este documento
  data.yaml                        configuración del dataset Ultralytics/YOLO
  images/{train,val,test}/*.jpg    renders 1920x1080 (igual que la cámara real);
                                    el prefijo thermal_ = señal térmica en falso color
  labels/{train,val,test}/*.txt    etiquetas YOLO (class cx cy w h, normalizadas)
../tools/                          los scripts que generaron este dataset
  build_yolo_dataset.py            reconstruye yolo_dataset/ entero de un tirón
                                    (genera los sitios, captura, hace el split
                                    train/val, talla un test ~70/20/10 sobre ese
                                    pool y escribe data.yaml + el README)
  make_test_split.py               la misma talla train/val/test como paso
                                    suelto, sin recapturar nada -- útil para
                                    re-repartir un dataset ya construido; ya no
                                    hace falta correrlo aparte tras un build
                                    completo, build_yolo_dataset.py lo hace solo
../tools/capture_dataset/          las piezas que build_yolo_dataset.py orquesta
  projection.py                    proyector 3D defecto-de-módulo -> bbox 2D
  capture_dataset.py               driver de captura en Gazebo + auto-etiquetado,
                                    --thermal cambia al render térmico
  pick_spawn.py                    elige un punto de spawn del dron sin colisiones
  source_worlds_metadata/*.json    defects.json (verdad de terreno) de cada sitio
                                    (no hace falta para entrenar — se conserva como
                                    referencia; ver
                                    tools/capture_dataset/source_worlds_metadata/README.md
                                    para saber exactamente qué es y cómo se convierte
                                    en una etiqueta YOLO)
../videos/*.mp4                    8 vídeos de vuelo (ver Vídeos) — se guardan en la
                                    propia carpeta videos/ del proyecto, no se copian
                                    aquí dentro, para no duplicar archivos de
                                    ~50-100 MB que ya viven en la ruta que documenta
                                    INSTRUCTIONS.md
```

Los scripts vivían antes en `yolo_dataset/scripts/`; se movieron a
`tools/capture_dataset/` para que esta carpeta contenga solo datos y se
pueda subir a Colab/Roboflow/etc. tal cual. Ver
[`../tools/README.md`](../tools/README.md).

## Clases

| id | nombre | apariencia típica | modo de captura |
|---|---|---|---|
| 0 | `dirt` | suciedad, con tendencia al borde inferior (cuesta abajo) del panel | RGB |
| 1 | `bird_dropping` | mancha pequeña opaca, blanca o blancuzca | RGB |
| 2 | `crack` | fractura brillante y ramificada | RGB |
| 3 | `delamination` | mancha amarillenta lechosa, con tendencia al perímetro del módulo | RGB |
| 4 | `thermal_problem` | cualquier punto caliente en la señal térmica en falso color | térmico |

**Por qué lo térmico tiene una sola clase en lugar de cuatro:** el RGB
distingue visiblemente qué causó un defecto (una grieta no se parece en nada
a una mancha de suciedad). Una cámara térmica lee temperatura, no forma ni
color — y los propios datos de este proyecto confirman que los cuatro tipos
de defecto producen firmas de calor *solapadas* (ver
[Imágenes térmicas](#imágenes-térmicas) para las cifras reales), así que
etiquetar detecciones térmicas con una causa que solo el RGB puede
justificar estaría afirmando más de lo que el sensor puede sostener.
`thermal_problem` marca "aquí hay una anomalía térmica", que es lo que un
pase de inspección real solo-térmico puede afirmar honestamente; averiguar
*por qué* es tarea del pase RGB (o de una inspección de seguimiento).

## Composición

**RGB** (clases 0-3):

| Sitio | Semilla | Mesas | Suelo | Infra | Tomas | Train | Val | Cajas |
|---|---|---|---|---|---|---|---|---|
| site_default | 11 | 100 (1000 paneles) | hierba | valla+carretera+4 inversores | 70 | 60 | 10 | 277 |
| site_g | 901 | 35 (350 paneles) | tierra | valla+carretera+3 inversores | 40 | 34 | 6 | 279 |
| site_h | 902 | 35 (350 paneles) | hierba | valla+carretera+5 inversores | 40 | 34 | 6 | 388 |
| site_i | 903 | 35 (350 paneles) | tierra | **ninguna** (`--no-infrastructure`) | 40 | 34 | 6 | 361 |
| site_j | 1101 | 35 (350 paneles) | hierba | valla+carretera+4 inversores | 40 | 34 | 6 | 251 |

**Térmico** (solo clase 4, prefijo de archivo `thermal_`), los mismos 5
sitios, con poses de cámara muestreadas de forma independiente (semilla de
RNG de pose distinta a la del RGB, así que estas tomas no están emparejadas
fotograma a fotograma con las de RGB de arriba):

| Sitio | Tomas | Train | Val | Cajas |
|---|---|---|---|---|
| thermal_site_default | 70 | 60 | 10 | 254 |
| thermal_site_g | 40 | 34 | 6 | 303 |
| thermal_site_h | 40 | 34 | 6 | 373 |
| thermal_site_i | 40 | 34 | 6 | 441 |
| thermal_site_j | 40 | 34 | 6 | 225 |

`site_default` es el mundo de 1000 módulos que este proyecto ya traía
pregenerado. `site_g/h/i/j` se generaron para esta ampliación con semillas,
mezclas de defectos, cobertura de suelo, infraestructura y (`site_j`) color
de cielo distintos, de modo que el dataset no es solo una misma granja
fotografiada muchas veces.

Recuento de instancias por clase:

| Split | dirt | bird_dropping | crack | delamination | thermal_problem |
|---|---|---|---|---|---|
| train | 560 | 361 | 197 | 227 | 1315 |
| val | 67 | 64 | 36 | 44 | 281 |

## La cámara coincide con el dron real

El dron de este proyecto (`models/x500_rgb/model.sdf`) lleva una **cámara
nadir real y fija** — una Raspberry Pi Camera Module 3, 1920x1080, 66° de
FOV horizontal (`horizontal_fov=1.151917`), montada rígidamente apuntando
hacia abajo. La cámara sintética del dataset usa **exactamente estos
mismos parámetros intrínsecos**, así que un modelo entrenado aquí se aplica
directamente al topic real `/x500_rgb/nadir` sin desajuste de resolución ni
de FOV.

El montaje de la cámara es **nadir fijo** — un vuelo real nunca puede mirar
de forma oblicua — así que las poses se mantienen cerca del nadir
(`pitch = 90° ± 3°`, `roll = 0° ± 1.7°`, que reproduce las pequeñas
desviaciones de actitud que induce un crucero real en modo GUIDED mientras
se desplaza), y varían en cambio en:

- **altitud**: 5 / 6.5 / 8 / 10 / 13 m (la altitud de crucero real es 8 m;
  esto la rodea para dar diversidad de escala)
- **rumbo** (yaw): 0°, ±90°, 180°, 45°
- **posición**: una mesa elegida al azar (80% de probabilidad ponderada
  hacia las dañadas), con jitter de ±1.5 m / ±2.5 m

## Cómo se calcularon las etiquetas

El método (ver
[`../tools/capture_dataset/projection.py`](../tools/capture_dataset/projection.py)
para la derivación completa) reconstruye la posición 3D de cada defecto a
partir de `defects.json` (pose de la mesa + índice de módulo, usando la
geometría exacta de `pv_mesh.py`), lo proyecta con un modelo de cámara
pinhole que sigue la convención `<camera horizontal_fov>` de Gazebo,
conserva la caja solo si ≥50% está dentro del encuadre y ≥3 px, y la
descarta si el panel da la espalda a la cámara.

**Validado antes de generar el conjunto completo**: el cálculo de
proyección se comprobó superponiendo cajas calculadas sobre un render real
(`site_i_037`, 37 cajas, densamente dañado) y confirmando que las cajas
caen sobre los píxeles reales de grieta/suciedad/delaminación, no solo
sobre lo que se esperaba en teoría.

## Imágenes térmicas

`--thermal` en `capture_dataset.py` renderiza una variante con **cambio de
material** del mundo, en lugar de usar un modelo de sensor distinto: el
`albedo_map` de cada mesa (la textura en luz visible que renderiza Gazebo)
se redirige a `pv_atlas_NN_thermal.png`, que este proyecto ya genera junto
con la textura visible — mismo mapeado UV, mismas posiciones de defecto,
solo cambia la textura de origen — y se desactivan las sombras (una cámara
térmica lee temperatura, no sombra). Es exactamente el "cambio de material
sobre los assets existentes" que ya describe `docs/METHODOLOGY.md` de este
proyecto para una hipotética cámara térmica; simplemente nunca se había
conectado a nada hasta este dataset y `flight_video.py --thermal`.

**El falso color se calibra sobre valores de píxel medidos, no sobre el
rango teórico.** El generador térmico de `pv_textures.py` asigna a un panel
limpio un valor base de ~0.42 (0-1) y a los píxeles de defecto más
calientes ~0.73, pero esa señal ocupa solo una fracción estrecha del rango
0-255 tras el resto del pipeline de render. Por eso, antes de aplicar
`COLORMAP_INFERNO`, se estira el tramo `[110, 170]` — medido directamente
sobre renders reales (panel limpio ~132/255, picos de defecto ~160-171/255,
fondo ~67/255) — a `[0, 255]` (`solar_farm_gz/flight_video.py`,
`THERMAL_LOW`/`THERMAL_HIGH`, `_thermal_swap()`). `thermal_colour()` de
`capture_dataset.py` y `_thermal_colour()` de `capture.py` reutilizan las
mismas constantes, para que las imágenes fijas y los vídeos de todas las
herramientas coincidan.

**Precaución al reutilizar esta calibración.** Al estar ajustada sobre
valores medidos de esta combinación concreta de iluminación y texturas —no
derivada directamente del rango teórico 0.40-0.74 de `pv_textures.py`—, el
tramo `[110, 170]` es específico del render actual. Si se cambia de forma
sustancial la iluminación de la escena, las texturas térmicas o el motor
de render, conviene volver a medir un panel limpio y un pico de defecto
sobre un render real antes de confiar en el resultado: el código no avisa
si la calibración ha dejado de encajar, y el síntoma sería la imagen
térmica volviendo a verse como una mancha de color uniforme.

**Por qué una sola clase.** Los cuatro tipos de defecto RGB se contrastaron
con los deltas térmicos reales por tipo de `pv_textures.py` — la señal
adicional que aporta cada tipo de defecto sobre la base de ~0.42, antes del
factor de escala 0.45 que se aplica en `render_module()`:

| Tipo | factor de delta térmico | valor pico (base + factor×0.45) |
|---|---|---|
| dirt | 0.30 | ~0.55 |
| bird_dropping | 0.55 | ~0.67 |
| crack | 0.65 | ~0.71 |
| delamination | 0.70 | ~0.73 |

Estos rangos **se solapan** — una mancha de suciedad severa y una grieta
leve pueden caer en el mismo valor térmico — así que una etiqueta de clase
que afirmara "este punto caliente es una grieta, no suciedad" estaría
asumiendo más precisión de la que el sensor (simulado o real) realmente
ofrece. `thermal_problem` es la etiqueta honesta: hay una anomalía presente
y localizada; el pase RGB es el que identifica la causa.

## Vídeos

Ocho vídeos de vuelo — **vuelos reales de ArduPilot SITL**, no trayectorias
de cámara guionizadas: la aeronave obtiene una posición GPS, arma motores,
despega hasta 8 m y avanza en crucero bajo control de velocidad en modo
GUIDED mientras la cámara de seguimiento (con la señal nadir en vivo
incrustada y un HUD de telemetría) graba. Se guardan en la propia carpeta
`videos/` del proyecto (`$HOME/solar_farm_sim/videos/`), según
[`videos/README.md`](../videos/README.md) — no se duplican dentro de esta
carpeta del dataset.

| Archivo | Sitio | Semilla | Duración | Notas |
|---|---|---|---|---|
| `preview_flight_76s.mp4` | site_default | 11 | 76 s | primera vista de este proyecto, antes del resto del trabajo de abajo |
| `site_g_seed901_flight_46s.mp4` | site_g | 901 | 46 s | |
| `site_h_seed902_flight_49s.mp4` | site_h | 902 | 49 s | |
| `site_i_seed903_flight_60s.mp4` | site_i | 903 | 60 s | |
| `site_j_seed1101_flight_55s_wow.mp4` | site_j | 1101 | 55 s | hierba, cielo azul saturado, oscilación senoidal de 3 m en altitud por cada fila cruzada |
| `site_j_seed1101_flight_60s.mp4` | site_j | 1101 | 60 s | + texto de título/estado personalizado desde `.env`, oscilación de 4 m, recuento real de módulos/defectos (corrección de error) |
| `site_j_seed1101_flight_45s_thermal.mp4` | site_j | 1101 | 45 s | primera prueba de `--thermal`, **antes** de la corrección de contraste (mancha roja uniforme — se conserva como referencia de antes/después) |
| `site_j_seed1101_flight_60s_thermal.mp4` | site_j | 1101 | 60 s | todo lo anterior + señal nadir térmica calibrada — el vídeo combinado final |

**Estado actual (27/08):** de los ocho, solo `preview_flight_76s.mp4`
sigue en `videos/`; el resto se ha borrado en una limpia posterior, ya
superados por el recorrido de 120 s con `--route` (site_j, seed 1101,
RGB + térmico) grabado para la demo de inferencia de `yolo_sim_training`
— ver [`videos/README.md`](../videos/README.md) para el catálogo vigente
y [inference_demo/README.md](../../yolo_sim_training/inference_demo/README.md)
para ese vídeo. La tabla de arriba queda como registro histórico de los
modos de fallo de rumbo descritos a continuación.

Las duraciones de los tres primeros sitios nuevos (`g`/`h`/`i`) se
aleatorizaron en el rango [40, 90] s (`random.seed(8842)` → `[46, 49, 60]`);
los vídeos de `site_j` se construyeron de forma incremental según
peticiones posteriores en la misma sesión, verificando cada uno muestreando
fotogramas antes de pasar al siguiente.

**Son vuelos reales, así que el rumbo no está guionizado.**
`flight_video.py` hace aparecer la aeronave con un yaw fijo y avanza en
crucero en su propio marco de referencia corporal, pero a qué dirección del
*marco de referencia del mundo* corresponde eso depende de la propia
estimación de rumbo del EKF de ArduPilot en el momento de armar — en la
práctica esto no coincidía de forma fiable con el yaw de aparición. Se
encontraron y corrigieron dos modos de fallo mientras se producían estos
vídeos:

- Aparecer en el borde de una fila (para que la dirección "prevista" vuele
  hacia el interior del array): en dos de los tres sitios nuevos la
  aeronave voló en cambio directamente hacia fuera, a través de la valla
  perimetral, en cuestión de segundos, porque el rumbo resuelto real
  apuntaba hacia el otro lado, o en diagonal cruzando filas en vez de a lo
  largo de una.
- Aparecer en el centro geométrico del array, que es robusto frente al
  *rumbo* pero no frente a la *posición*: para `site_h` (un número impar de
  mesas por fila) el centro caía exactamente sobre la huella de una mesa en
  lugar de en un hueco, y la aeronave chocaba contra ella al aparecer y se
  quedaba ahí durante toda la grabación (visible en el log como `at
  altitude (45s)` — el timeout completo de ascenso, sin llegar nunca a los
  8 m).

La versión final de `tools/capture_dataset/pick_spawn.py` hace aparecer la
aeronave en el centro del array para tener despeje independiente del rumbo,
pero ajusta la posición al hueco real más cercano *entre* dos mesas en
lugar de a la media geométrica en bruto, de modo que no puede caer sobre
una mesa. Los cuatro vídeos se revalidaron después muestreando fotogramas
al 5/25/50/75/95% de su duración y revisándolos a ojo — no se dieron por
buenos solo porque el render terminara sin error. En los sitios más
pequeños (350 paneles), sea cual sea la dirección que tome realmente la
aeronave, acaba volando fuera del array modelado antes de que termine el
clip (visible como terreno abierto en el último 10-20% de
`site_g`/`site_h`/`site_i`) — vuelo real sobre un sitio de tamaño real
(aunque modesto), no un bucle sin límites.

## Limitaciones conocidas

- **Sin modelado de oclusión** en la proyección de etiquetas — las poses de
  cámara se mantienen cerca del nadir precisamente para minimizar esto.
- **460 imágenes es un conjunto inicial sólido, no enorme.** Es fácil
  ampliarlo — ver Ampliación.
- **Las imágenes RGB y térmicas no están emparejadas fotograma a
  fotograma.** Se muestrean con semillas de RNG de pose distintas por
  sitio, así que son dos conjuntos de imágenes independientes dentro de un
  mismo dataset, no pares multimodales sincronizados. Un modelo entrenado
  aquí aprende "qué aspecto tiene un defecto/anomalía" en cada modalidad
  por separado, no una fusión cross-modal.
- **El límite de `thermal_problem` es una decisión de modelado, no
  física.** El factor de escala 0.45 y los deltas por tipo en
  `pv_textures.py` son el propio modelo de calor-por-defecto del proyecto,
  no la respuesta calibrada de una cámara térmica real — la *forma* del
  argumento (rangos solapados → no sobre-afirmar la identidad de clase) se
  mantiene igualmente, pero las cifras exactas son de este simulador, no de
  una hoja de datos.
- **Los vídeos no tienen verdad de terreno**, y siguen un régimen de
  muestreo distinto al de las imágenes fijas (vuelo real continuo frente a
  tomas discretas preparadas) — úsense para inspección cualitativa, no para
  métricas.
- **Imágenes simuladas, no reales** — misma salvedad que en todo el
  proyecto.

## Reproducción

**Un solo comando** reconstruye `yolo_dataset/` entero (los 5 sitios, RGB +
térmico, split train/val, un test ~70/20/10 tallado sobre ese pool
(`tools/make_test_split.py`, integrado como último paso -- misma semilla
42, así que un rebuild completo desde cero reproduce el mismo reparto que
ya trae el dataset), `data.yaml` y el `README.md` de la carpeta):

```bash
python3 tools/build_yolo_dataset.py
```

`site_default` nunca se regenera (se reutiliza el mundo que el proyecto ya
trae) y la receta de `site_g` está confirmada byte a byte (es la que se
detalla más abajo). Para `site_h`/`site_i`/`site_j` no quedó registrada la
línea de comandos original de `generate_farm.py`, solo el resultado
(`defects.json`); `tools/build_yolo_dataset.py` fija en `SITE_RECIPES`
unos pesos por tipo reconstruidos proporcionalmente a esos recuentos
(`peso_tipo = recuento_tipo / total`) como constantes explícitas — ya no
son "pesos reconstruidos sobre la marcha", quedan registrados aquí, así
que regenerar estos tres sitios a partir de ahora es determinista y
reproducible byte a byte dado el mismo seed.

Esa reconstrucción es aproximada, eso sí, no una inversión calibrada del
generador — y de forma medible. Contrastada contra `site_g` (el único
sitio cuyos pesos SÍ son los originales confirmados: 0,65/0,20/0,10/0,05
para dirt/bird_dropping/delamination/crack), el mismo método proporcional
aplicado a los propios recuentos de `site_g` recuperaría
0,71/0,17/0,10/0,03 — un error de hasta ~43% relativo en la clase de
menor recuento (`crack`). En la práctica, comprobado sobre el terreno,
se queda más cerca que ese peor caso: regenerar `site_h` con sus pesos
registrados (seed 902) reprodujo `defects_by_type` dentro de un ~5%
relativo por clase respecto a los recuentos originales (dirt=90
bird_dropping=69 crack=114 delamination=101, 374 en total, frente a los
84/65/107/101, 357 en total, ya registrados) — `site_i` y `site_j` no se
han reverificado así de forma independiente. En cualquier caso, esto no
garantiza imágenes idénticas píxel a píxel a las que ya hay en
`yolo_dataset/` para estos tres sitios — eso ya se sabía y sigue siendo
irrecuperable, al no haberse registrado nunca la línea de comandos
original. El detalle exacto de qué está confirmado y qué reconstruido
está en el docstring de
[`../tools/build_yolo_dataset.py`](../tools/build_yolo_dataset.py).

Lo que sigue es la misma receta desglosada paso a paso, para quien quiera
entender o repetir un único sitio a mano:

```bash
cd $HOME/solar_farm_sim/src/solar_farm_gz
export PYTHONPATH="$PWD:$PYTHONPATH"

# regenerar un sitio (bit a bit idéntico dado el mismo seed)
python3 -m solar_farm_gz.generate_farm --panels 350 --tables-per-row 10 --variants 18 \
    --seed 901 --clean-ratio 0.60 --w-dirt 0.65 --w-crack 0.05 --w-bird-dropping 0.20 \
    --w-delamination 0.10 --ground-style earth --sun-elevation 40 --sun-azimuth 110 \
    --inverters 3 -o /path/to/worlds/site_g

# capturar y auto-etiquetar N tomas RGB de ese sitio (1920x1080, FOV de la cámara real, poses con sesgo nadir)
python3 ../../tools/capture_dataset/capture_dataset.py \
    --world-dir /path/to/worlds/site_g --site site_g --n 40 --seed 42 \
    --images-out /path/to/out/images --labels-out /path/to/out/labels

# mismo sitio, modo térmico: render con cambio de material, una única clase thermal_problem
python3 ../../tools/capture_dataset/capture_dataset.py \
    --world-dir /path/to/worlds/site_g --site site_g --n 40 --seed 77 --thermal \
    --images-out /path/to/out/images --labels-out /path/to/out/labels

# o volar un vídeo de vuelo real de ArduPilot sobre él (añadir --thermal para la señal nadir térmica)
spawn=$(python3 ../../tools/capture_dataset/pick_spawn.py /path/to/worlds/site_g/defects.json)
ros2 run solar_farm_gz flight_video -- --world /path/to/worlds/site_g/solar_farm.sdf \
    --duration 46 --spawn "$spawn" -o site_g_flight.mp4
```

## Ampliación

Más semillas, más tomas por sitio (RGB o `--thermal`), y/o muestrear con más
intensidad el `site_default` de 1000 paneles ya existente son todo
opciones gratuitas — nada en el pipeline está ajustado a mano para estos
cinco sitios, es completamente paramétrico (seed, número de paneles, estilo
de suelo, infraestructura, mezcla de defectos y ángulo solar pasan
directamente a `generate_farm.py`).

## Entrenamiento

```bash
pip install ultralytics
yolo detect train data=yolo_dataset/data.yaml model=yolov8n.pt imgsz=1920 epochs=100
```

Después, ejecuta inferencia sobre los vídeos de vuelo para una comprobación
cualitativa, o directamente sobre el topic real `/x500_rgb/nadir` según el
§6 de `INSTRUCTIONS.md` (misma resolución y FOV, así que `best.pt` encaja
directamente):

```bash
yolo detect predict model=runs/detect/train/weights/best.pt \
    source=$HOME/solar_farm_sim/videos/site_i_seed903_flight_60s.mp4 save=True
```
