# source_worlds_metadata/

Una copia de `defects.json` por cada site usado para construir el dataset
actual (`site_default_defects.json`, `site_g_defects.json`,
`site_h_defects.json`, `site_i_defects.json`, `site_j_defects.json`). Este
documento explica qué es ese fichero, por qué existe una copia aquí y no
dentro de `yolo_dataset/`, y cómo se relaciona exactamente con las
imágenes y etiquetas que sí hay en `yolo_dataset/`.

Está escrito asumiendo que ya sabes lo básico de YOLO (una imagen, un
`.txt` con una línea por caja: `clase cx cy ancho alto`, todo normalizado
0–1 respecto a la imagen) pero no necesariamente cómo se generó ese `.txt`
en este proyecto.

## Qué es `defects.json`

Cuando `generate_farm.py` construye un parque solar sintético, **decide
él mismo, de forma procedural**, qué módulos están dañados, de qué tipo es
cada defecto y dónde cae exactamente sobre la superficie del módulo —
porque es él quien dibuja el defecto sobre la textura. No hay que
detectarlo ni anotarlo a mano: el generador ya sabe la respuesta correcta
mientras crea el mundo, y la vuelca en `defects.json` junto al propio
mundo (`worlds/<site>/defects.json`).

`site_default_defects.json` es la copia de ese fichero para `site_default`
— el parque de 1000 paneles (seed 11) que el proyecto trae ya generado.
Cabecera real de ese fichero:

```json
{
 "seed": 11,
 "modules": 1000,
 "tables": 100,
 "modules_per_table": 10,
 "clean_ratio_requested": 0.8,
 "clean_ratio_actual": 0.8,
 "defect_instances": 420,
 "defects_by_type": {
  "dirt": 170,
  "bird_dropping": 105,
  "crack": 55,
  "delamination": 90
 },
 ...
}
```

Y luego, por cada módulo del parque (agrupados por `atlas`, la textura
compartida a la que pertenecen), una entrada así:

```json
{
 "module_index": 3,
 "atlas_cell": [3, 0],
 "clean": false,
 "defects": [
   {
     "type": "bird_dropping",
     "severity": 0.962,
     "bbox_uv_cxcywh": [0.54297, 0.47754, 0.32031, 0.1543]
   },
   {
     "type": "dirt",
     "severity": 0.439,
     "bbox_uv_cxcywh": [0.49902, 0.78809, 0.99805, 0.42188]
   }
 ]
}
```

Es decir: "el módulo 3 de esta mesa tiene un excremento de ave centrado al
54%/48% de su propia superficie, ocupando el 32%×15% de esa superficie; y
además una mancha de suciedad centrada más abajo, casi tan ancha como todo
el módulo". `severity` es la intensidad usada para pintar el defecto
(afecta a cuánto "se nota", no a la caja). Todo esto es **1000% exacto**
porque no es una medición ni una estimación — es literalmente la receta
que se usó para dibujar la textura.

## La pieza clave: esto NO está en el sistema de coordenadas de una foto

Aquí está el punto que no es obvio viniendo de YOLO normal: la caja
`bbox_uv_cxcywh` de arriba **no es una caja YOLO**, aunque tenga la misma
forma `(cx, cy, w, h)` normalizada 0–1. Es una caja en el espacio UV del
**módulo individual** — como si dijeras "en este panel de PowerPoint en
concreto, el logo está al 54% del ancho y 48% del alto". No sabe nada de
cámaras, ni de ángulos, ni de a qué distancia vuela el dron, ni de en qué
píxel de qué foto va a acabar cayendo. Es una coordenada sobre el objeto
físico, no sobre una imagen.

Una etiqueta YOLO de verdad (`class cx cy w h` normalizado a la imagen
completa, una línea por caja, en un `.txt` dentro de `yolo_dataset/labels/`)
depende en cambio de una foto concreta: desde qué pose exacta se disparó
la cámara, con qué campo de visión, y si ese módulo en concreto queda de
frente, de refilón o fuera de encuadre en esa toma. La misma mesa
fotografiada desde dos alturas o ángulos distintos produce dos cajas YOLO
completamente distintas — pero el `defects.json` que las origina es el
mismo, porque el defecto físico no se ha movido.

## Cómo se convierte lo uno en lo otro

Ese es exactamente el trabajo de
[`tools/capture_dataset/`](../README.md) — dos pasos, dentro de
`capture_dataset.py`:

1. **Reconstrucción 3D** (`projection.py: defect_world_corners`): a partir
   de la caja UV del módulo, el índice del módulo dentro de la mesa, y la
   pose de la mesa en el mundo (`tables_placed` en `defects.json`),
   reconstruye las 4 esquinas del defecto como puntos 3D reales en el
   mundo — usando la misma geometría exacta (tamaño de módulo, inclinación
   28°, huecos) que `pv_mesh.py` usa para construir la malla que ve
   Gazebo.
2. **Proyección de cámara** (`projection.py: project_defect_bbox`): esos
   puntos 3D se proyectan con un modelo de cámara pinhole que replica el
   `horizontal_fov` real de la Raspberry Pi Camera Module 3 del dron
   (`x500_rgb`), para la pose de cámara concreta de esa toma. El
   resultado — recortado al encuadre, descartado si el panel queda de
   espaldas o casi fuera de plano — sí es ya una caja en coordenadas de
   imagen, y es la que `capture_dataset.py` escribe tal cual en el
   `.txt` de `yolo_dataset/labels/`, con la clase (`dirt`→0,
   `bird_dropping`→1, `crack`→2, `delamination`→3, o `thermal_problem`→4
   en modo `--thermal`).

En resumen, la cadena completa para una imagen del dataset es:

```
defects.json (verdad del mundo, en espacio de módulo)
        │
        │  projection.py: reconstruye 3D + proyecta con la cámara real
        ▼
yolo_dataset/labels/<imagen>.txt (verdad de ESA foto, en espacio de imagen)
```

## Por qué "un `defects.json` → muchas imágenes/etiquetas"

`defects.json` no depende de la cámara, así que un único fichero sirve
para generar tantas fotos (y etiquetas YOLO) como se quiera del mismo
parque, sin tener que regenerar el mundo: `capture_dataset.py --n 40`
sencillamente prueba 40 poses de cámara distintas sobre el mismo
`defects.json` y proyecta una etiqueta nueva cada vez. Las 70 imágenes de
`site_default` en `yolo_dataset/` (ver
[`docs/YOLO_DATASET.md`](../../../docs/YOLO_DATASET.md#composition)) salen
todas de este mismo `site_default_defects.json` — 70 fotos distintas del
mismo parque, no 70 parques distintos.

## Por qué vive aquí y no dentro de `yolo_dataset/`

`yolo_dataset/` está pensado para subirse tal cual a Colab, Roboflow o
donde haga falta — solo `data.yaml`, `images/` y `labels/`, el formato que
un pipeline de entrenamiento YOLO espera. `defects.json` no encaja ahí: no
es una imagen, no es una etiqueta YOLO, y ningún paso del entrenamiento lo
lee. Se guarda igualmente porque:

- **Reproducibilidad**: con el mismo `--seed` en `generate_farm.py` se
  reconstruye el mundo exacto, y con este `defects.json` se puede generar
  más imágenes/etiquetas de `site_default` sin volver a generar el mundo.
- **Verificación**: si una caja de `yolo_dataset/labels/` parece mal
  puesta, se puede comprobar contra la caja UV original y la geometría de
  `projection.py`, en vez de fiarse a ojo.
- **Contexto del dataset**: los recuentos `defects_by_type` de la cabecera
  son la fuente de las tablas de composición del dataset en
  [`docs/YOLO_DATASET.md`](../../../docs/YOLO_DATASET.md#composition).

Si algún día no hace falta ni la reproducibilidad ni la verificación, esta
carpeta se puede borrar sin que `yolo_dataset/` deje de funcionar para
entrenar — es metadato de referencia, no parte del dataset en sí.
