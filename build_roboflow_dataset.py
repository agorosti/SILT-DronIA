#!/usr/bin/env python3
"""
Construye un dataset en formato YOLO (imágenes + etiquetas .txt), listo
para subir a Roboflow, a partir de:
    - los atlas de textura del mundo (pv_atlas_NN_albedo.png)
    - defects.json (posición y tipo de cada defecto, generado por el
      mundo)

Cada módulo (celda del atlas) se recorta como una imagen independiente.
Si el módulo está dañado, se genera además un fichero de etiqueta YOLO
con la clase y el bounding box de cada defecto, normalizado al recorte
del módulo. Si está limpio, se genera una etiqueta vacía (imagen de
fondo, sin objetos) — útil para que el modelo aprenda también cómo es
un panel sano.

USO:
    # 1. Primero, inspecciona la estructura real de defects.json
    #    (necesario una sola vez, para confirmar los nombres de campo):
    python3 build_roboflow_dataset.py --inspect

    # 2. Si el bounding box se extrae bien, genera el dataset completo:
    python3 build_roboflow_dataset.py

Salida (en OUTPUT_DIR):
    images/<atlas>_<modulo>.png
    labels/<atlas>_<modulo>.txt      (formato YOLO: clase xc yc w h, normalizado 0-1)
    classes.txt                      (una clase por línea, en el orden de sus IDs)
"""

import argparse
import json
import os

from PIL import Image, ImageDraw

# ---------------------------------------------------------------------
# CONFIGURACIÓN — ajusta las rutas a tu entorno
# ---------------------------------------------------------------------

WORLD_DIR = "/home/andres/solar_farm_sim/src/solar_farm_gz/worlds"
TEXTURES_DIR = os.path.join(WORLD_DIR, "solar_farm_assets/materials/textures")
DEFECTS_JSON_PATH = os.path.join(WORLD_DIR, "defects.json")

OUTPUT_DIR = "/home/andres/solar_farm_sim/roboflow_dataset"

# Clases, en el orden en que se asignará su ID (0, 1, 2, 3...) —
# debe coincidir con los valores de "type" que aparecen en defects.json.
CLASSES = ["dirt", "bird_dropping", "crack", "delamination"]

# Geometría del atlas (ver metodología, Fase 1): rejilla de 5x2 celdas
# de 512x1024 px cada una, atlas total 2560x2048.
ATLAS_COLS = 5
ATLAS_ROWS = 2
CELL_W = 512
CELL_H = 1024

# Si las coordenadas UV vienen con el origen abajo (convención habitual
# en UV de mallas 3D) en vez de arriba (convención de imágenes), hay que
# invertir el eje Y al convertir a formato YOLO. Empieza en False;
# cámbialo a True si la verificación visual (--verify) muestra las
# cajas puestas "boca abajo" respecto al defecto real.
FLIP_V = False


# ---------------------------------------------------------------------
# Paso 1 — inspección de la estructura real de defects.json
# ---------------------------------------------------------------------

def inspect(defects):
    """Imprime el primer módulo dañado que encuentre, tal cual está en
    el JSON, para poder confirmar los nombres de campo del bounding box
    antes de intentar extraerlo automáticamente."""
    for atlas_name, modules in defects["atlases"].items():
        for m in modules:
            if not m["clean"]:
                print(f"Ejemplo de módulo dañado (atlas={atlas_name}):\n")
                print(json.dumps(m, indent=2, ensure_ascii=False))
                return
    print("No se ha encontrado ningún módulo dañado en defects.json.")


# ---------------------------------------------------------------------
# Paso 2 — extracción del bounding box
# ---------------------------------------------------------------------

def extract_bbox_yolo(defect):
    """Devuelve (xc, yc, w, h) ya normalizados 0-1 respecto al módulo,
    listos para formato YOLO. El mundo ya exporta esto directamente en
    'bbox_uv_cxcywh' (coordenadas UV del propio módulo), así que no
    hace falta ninguna conversión de píxeles."""
    if "bbox_uv_cxcywh" in defect:
        xc, yc, w, h = defect["bbox_uv_cxcywh"]
        if FLIP_V:
            yc = 1.0 - yc
        return xc, yc, w, h

    raise ValueError(
        "No se ha encontrado 'bbox_uv_cxcywh' en este defecto. Ejecuta "
        "--inspect y comprueba el nombre real del campo.\n"
        f"Contenido del defecto: {json.dumps(defect, ensure_ascii=False)}"
    )


def cell_origin(module):
    """Usa 'atlas_cell' [col, fila] si está presente (más fiable);
    si no, lo calcula a partir de module_index como respaldo."""
    if "atlas_cell" in module:
        col, row = module["atlas_cell"]
    else:
        idx = module["module_index"]
        col, row = idx % ATLAS_COLS, idx // ATLAS_COLS
    return col * CELL_W, row * CELL_H


# ---------------------------------------------------------------------
# Paso 3 — verificación visual (dibuja las cajas sobre unos ejemplos)
# ---------------------------------------------------------------------

def verify(defects, n_samples=6):
    verify_dir = os.path.join(OUTPUT_DIR, "verify")
    os.makedirs(verify_dir, exist_ok=True)
    shown = 0

    for atlas_name, modules in defects["atlases"].items():
        albedo_path = os.path.join(TEXTURES_DIR, f"{atlas_name}_albedo.png")
        if not os.path.exists(albedo_path):
            continue
        atlas_img = Image.open(albedo_path).convert("RGB")

        for m in modules:
            if m["clean"] or shown >= n_samples:
                continue
            ox, oy = cell_origin(m)
            crop = atlas_img.crop((ox, oy, ox + CELL_W, oy + CELL_H)).copy()
            draw = ImageDraw.Draw(crop)

            for defect in m["defects"]:
                xc, yc, w, h = extract_bbox_yolo(defect)
                x_min = (xc - w / 2) * CELL_W
                y_min = (yc - h / 2) * CELL_H
                x_max = (xc + w / 2) * CELL_W
                y_max = (yc + h / 2) * CELL_H
                draw.rectangle([x_min, y_min, x_max, y_max], outline="red", width=4)
                draw.text((x_min + 4, y_min + 4), defect["type"], fill="red")

            base_name = f"{atlas_name}_{m['module_index']:02d}"
            crop.save(os.path.join(verify_dir, base_name + "_verify.png"))
            shown += 1
        if shown >= n_samples:
            break

    print(f"Guardadas {shown} imágenes de verificación en: {verify_dir}")
    print("Ábrelas y comprueba que el recuadro rojo cae sobre el defecto real.")
    print("Si las cajas aparecen invertidas verticalmente (por ejemplo, la")
    print("mancha está arriba pero la caja aparece abajo), pon FLIP_V = True")
    print("al principio del script y vuelve a ejecutar --verify.")


# ---------------------------------------------------------------------
# Paso 4 — construcción del dataset completo
# ---------------------------------------------------------------------

def build_dataset(defects):
    images_dir = os.path.join(OUTPUT_DIR, "images")
    labels_dir = os.path.join(OUTPUT_DIR, "labels")
    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(labels_dir, exist_ok=True)

    # classes.txt, en el mismo orden que CLASSES (los IDs son su índice)
    with open(os.path.join(OUTPUT_DIR, "classes.txt"), "w") as f:
        f.write("\n".join(CLASSES) + "\n")

    n_clean, n_damaged, n_defects, n_errors = 0, 0, 0, 0

    for atlas_name, modules in defects["atlases"].items():
        albedo_path = os.path.join(TEXTURES_DIR, f"{atlas_name}_albedo.png")
        if not os.path.exists(albedo_path):
            print(f"  aviso: no encuentro {albedo_path}, salto este atlas")
            continue
        atlas_img = Image.open(albedo_path).convert("RGB")

        for m in modules:
            ox, oy = cell_origin(m)
            idx = m["module_index"]

            # Recorta el módulo del atlas
            crop = atlas_img.crop((ox, oy, ox + CELL_W, oy + CELL_H))
            base_name = f"{atlas_name}_{idx:02d}"
            crop.save(os.path.join(images_dir, base_name + ".png"))

            label_path = os.path.join(labels_dir, base_name + ".txt")

            if m["clean"]:
                # Imagen de fondo: etiqueta vacía (sin objetos)
                open(label_path, "w").close()
                n_clean += 1
                continue

            n_damaged += 1
            lines = []
            for defect in m["defects"]:
                cls_name = defect["type"]
                if cls_name not in CLASSES:
                    print(f"  aviso: clase desconocida '{cls_name}' en {base_name}, saltada")
                    continue
                cls_id = CLASSES.index(cls_name)
                try:
                    xc, yc, w, h = extract_bbox_yolo(defect)
                except ValueError as e:
                    print(f"  error en {base_name}: {e}")
                    n_errors += 1
                    continue

                lines.append(f"{cls_id} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")
                n_defects += 1

            with open(label_path, "w") as f:
                f.write("\n".join(lines) + ("\n" if lines else ""))

    print("\nResumen:")
    print(f"  módulos limpios (fondo):     {n_clean}")
    print(f"  módulos dañados:             {n_damaged}")
    print(f"  defectos etiquetados:        {n_defects}")
    print(f"  errores de bounding box:     {n_errors}")
    print(f"\nDataset generado en: {OUTPUT_DIR}")
    print("  -> images/   (recortes de cada módulo)")
    print("  -> labels/   (etiquetas YOLO, una .txt por imagen)")
    print("  -> classes.txt")
    if n_errors:
        print("\n⚠ Hubo errores extrayendo bounding boxes — revisa los mensajes "
              "de arriba antes de subir el dataset a Roboflow.")


# ---------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inspect", action="store_true",
                         help="Solo muestra la estructura de un defecto de ejemplo, sin generar nada")
    parser.add_argument("--verify", action="store_true",
                         help="Genera unas pocas imágenes con las cajas dibujadas, para comprobarlas a ojo")
    args = parser.parse_args()

    with open(DEFECTS_JSON_PATH) as f:
        defects = json.load(f)

    if args.inspect:
        inspect(defects)
    elif args.verify:
        verify(defects)
    else:
        build_dataset(defects)


if __name__ == "__main__":
    main()
