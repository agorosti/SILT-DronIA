# tools

Scripts que **generan** datasets de entrenamiento. Nada de lo que hay aquí
es el dataset en sí — los datos viven en `yolo_dataset/` y
`quicklook_dataset/`, en la raíz del proyecto, sin ningún script dentro:
así se pueden subir tal cual a Google Colab, Roboflow o donde haga falta,
sin arrastrar código.

| Herramienta | Genera | Cómo |
|---|---|---|
| [`build_yolo_dataset.py`](build_yolo_dataset.py) | `yolo_dataset/` completo, de un tirón | orquesta `capture_dataset/` para los 5 sitios × RGB/térmico, hace el split train/val, talla un test ~70/20/10 sobre ese pool y escribe `data.yaml` + el `README.md` de la carpeta |
| [`capture_dataset/`](capture_dataset/) | una captura RGB o térmica de un sitio | renderiza tomas desde la cámara del dron en poses realistas y proyecta las cajas 3D→2D — es la pieza que `build_yolo_dataset.py` invoca por sitio |
| [`make_test_split.py`](make_test_split.py) | reparte un `yolo_dataset/` ya construido en train/val/test | mismo algoritmo que el paso final de `build_yolo_dataset.py`, como script suelto para re-repartir sin recapturar nada; ya no hace falta correrlo aparte tras un build completo |
| [`build_quicklook_dataset.py`](build_quicklook_dataset.py) | `quicklook_dataset/` — recortes rápidos, sin cámara | recorta cada módulo directamente del atlas de textura |

## `build_yolo_dataset.py` — reconstruir `yolo_dataset/` entero

```bash
# ver el plan (comandos, semillas, recuentos) sin tocar Gazebo ni el disco
python3 tools/build_yolo_dataset.py --dry-run

# reconstrucción completa
python3 tools/build_yolo_dataset.py

# solo un par de sitios (útil para probar el script)
python3 tools/build_yolo_dataset.py --sites site_g site_h
```

`site_default` se reutiliza tal cual (nunca se regenera); `site_g` usa una
receta confirmada byte a byte; `site_h`/`site_i`/`site_j` reconstruyen los
pesos de cada tipo de defecto a partir de los recuentos guardados en su
`defects.json`, así que reproducen el mismo sitio y la misma mezcla de
defectos pero no garantizan imágenes idénticas píxel a píxel a las ya
existentes. El detalle completo de qué está confirmado y qué reconstruido
está en el docstring del propio script y en
[`../docs/YOLO_DATASET.md`](../docs/YOLO_DATASET.md#reproducción).

## `capture_dataset/` — dataset real (con cámara)

Tres ficheros:

- **`capture_dataset.py`** — el orquestador. Carga un mundo generado en
  Gazebo, dispara `--n` tomas en poses aleatorias (sesgadas hacia mesas
  dañadas, con el mismo campo de visión que la Raspberry Pi Camera Module
  3 del dron real), y escribe cada imagen junto a su etiqueta YOLO.
  `--thermal` renderiza el mundo con el material térmico y etiqueta todo
  como la clase única `thermal_problem`.
- **`projection.py`** — la geometría: reconstruye en 3D la posición de
  cada defecto y la proyecta a la caja 2D que vería la cámara en esa pose.
  Lo importa `capture_dataset.py`; no se ejecuta suelto.
- **`pick_spawn.py`** — utilidad para `flight_video.py`: calcula un punto
  de aparición del dron cerca del centro del parque pero en un hueco real
  entre mesas, para que no se genere encima de una.

`capture_dataset/source_worlds_metadata/` guarda una copia del
`defects.json` de cada site usado para generar el dataset actual (ground
truth de referencia, no algo que haga falta para entrenar — por eso vive
aquí y no dentro de `yolo_dataset/`, que solo tiene los datos que se suben
a Colab/Roboflow). Explicación detallada de qué es, y de cómo se
convierte en las etiquetas YOLO reales, en
[`capture_dataset/source_worlds_metadata/README.md`](capture_dataset/source_worlds_metadata/README.md).

**Cómo correrlo** (necesita el paquete `solar_farm_gz` en el `PYTHONPATH`,
por eso el `cd` a `src/solar_farm_gz`; no hace falta `ros2 run` ni tener
Gazebo abierto a mano — el propio script arranca su servidor headless):

```bash
cd ~/solar_farm_sim/src/solar_farm_gz
export PYTHONPATH="$PWD:$PYTHONPATH"

# RGB (4 clases: dirt, bird_dropping, crack, delamination)
python3 ../../tools/capture_dataset/capture_dataset.py \
    --world-dir /ruta/a/worlds/site_g --site site_g --n 40 --seed 42 \
    --images-out ../../yolo_dataset/images/train \
    --labels-out ../../yolo_dataset/labels/train

# térmico (1 clase: thermal_problem), mismo site
python3 ../../tools/capture_dataset/capture_dataset.py \
    --world-dir /ruta/a/worlds/site_g --site site_g --n 40 --seed 77 --thermal \
    --images-out ../../yolo_dataset/images/train \
    --labels-out ../../yolo_dataset/labels/train
```

```bash
# spawn seguro para flight_video.py, a partir del defects.json de un mundo
python3 tools/capture_dataset/pick_spawn.py /ruta/a/worlds/site_g/defects.json
```

Detalle completo — composición del dataset, validación de la proyección,
entrenamiento con Ultralytics — en
[`../docs/YOLO_DATASET.md`](../docs/YOLO_DATASET.md).

## `build_quicklook_dataset.py` — recortes rápidos (sin cámara)

Recorta cada módulo del atlas de textura tal cual, sin pasar por ningún
render ni modelo de cámara. Rápido y útil para una primera comprobación de
que las cajas de `defects.json` caen donde deben, pero **no** representa
la perspectiva real del dron — para eso usa `capture_dataset/` arriba.

```bash
# 1. una sola vez, para confirmar el formato de defects.json
python3 tools/build_quicklook_dataset.py --inspect

# 2. genera el dataset completo (images/ + labels/ + classes.txt en quicklook_dataset/)
python3 tools/build_quicklook_dataset.py

# 3. comprueba visualmente que las cajas caen sobre el defecto real
python3 tools/build_quicklook_dataset.py --verify
```

Se puede correr desde cualquier directorio: las rutas de entrada y salida
están fijadas en las constantes `WORLD_DIR`/`OUTPUT_DIR` al principio del
script, no dependen del directorio de trabajo.
