#!/usr/bin/env python3
"""Añade una partición de test a yolo_dataset/, separada de train/val.

Nota: desde que este mismo reparto se integró como último paso de
tools/build_yolo_dataset.py (carve_test_split(), misma semilla y
fracciones), un build completo ya no necesita este script aparte. Sigue
siendo útil como herramienta suelta para re-repartir un yolo_dataset/ ya
construido sin recapturar nada (por ejemplo, para probar otra semilla o
proporción).

yolo_dataset/ solo definía train/val (ver README.md) -- no hay ningún
conjunto que quede completamente al margen del entrenamiento y de la
selección del mejor checkpoint (Ultralytics elige "best.pt" según el
propio val en cada época). Para evaluar un modelo entrenado sobre este
dataset de forma comparable a los experimentos reales del TFM, que sí
usan train/val/test con el test nunca visto durante el entrenamiento,
hace falta ese tercer split.

Reparte las imágenes actuales (train+val combinados) en una nueva
proporción train/val/test ≈ 70/20/10 -- la misma que usó el TFM real en
E1_baseline (507/145/72 de 724 imágenes) -- con una asignación aleatoria
determinista (seed=42, la misma semilla que el resto del proyecto) para
que el resultado sea reproducible. El reparto es una reasignación
completa de las tres particiones, no solo un recorte de val: algunas
imágenes que hoy están en train pueden acabar en val y viceversa, además
de las que pasan a test.

Mueve los ficheros físicamente (no los copia): tras ejecutarlo, train/ y
val/ tienen menos imágenes y aparece un test/ nuevo, y data.yaml se
actualiza para declarar el nuevo split. Es reversible sin pérdida de
datos porque todo el dataset se puede regenerar desde cero con
`python3 tools/build_yolo_dataset.py`.

Uso:
    python3 tools/make_test_split.py
    python3 tools/make_test_split.py --dry-run   # solo muestra el reparto, no mueve nada
"""
import argparse
import random
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATASET_DIR = PROJECT_ROOT / "yolo_dataset"
SEED = 42
# Misma proporción que E1_baseline en el TFM real (507/145/72 de 724 imágenes)
TRAIN_FRAC = 0.70
VAL_FRAC = 0.20
# El resto (~0.10) va a test


def collect_basenames(images_dir: Path):
    if not images_dir.exists():
        return []
    return sorted(p.stem for p in images_dir.glob("*.jpg"))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true", help="solo muestra el reparto, no mueve nada")
    args = parser.parse_args()

    train_dir = DATASET_DIR / "images" / "train"
    val_dir = DATASET_DIR / "images" / "val"
    test_img_dir = DATASET_DIR / "images" / "test"
    test_lbl_dir = DATASET_DIR / "labels" / "test"

    pool = [(n, "train") for n in collect_basenames(train_dir)] + \
           [(n, "val") for n in collect_basenames(val_dir)]

    rng = random.Random(SEED)
    rng.shuffle(pool)

    n = len(pool)
    n_train = round(n * TRAIN_FRAC)
    n_val = round(n * VAL_FRAC)
    n_test = n - n_train - n_val

    assignment = {}
    for name, cur in pool[:n_train]:
        assignment[name] = ("train", cur)
    for name, cur in pool[n_train:n_train + n_val]:
        assignment[name] = ("val", cur)
    for name, cur in pool[n_train + n_val:]:
        assignment[name] = ("test", cur)

    print(f"Total: {n} imagenes -> train={n_train} val={n_val} test={n_test}")

    if args.dry_run:
        return

    for split in ("train", "val", "test"):
        (DATASET_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (DATASET_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)

    moved = 0
    for name, (target, current) in assignment.items():
        if target == current:
            continue
        src_img = DATASET_DIR / "images" / current / f"{name}.jpg"
        dst_img = DATASET_DIR / "images" / target / f"{name}.jpg"
        src_lbl = DATASET_DIR / "labels" / current / f"{name}.txt"
        dst_lbl = DATASET_DIR / "labels" / target / f"{name}.txt"
        src_img.rename(dst_img)
        if src_lbl.exists():
            src_lbl.rename(dst_lbl)
        moved += 1

    data_yaml_path = DATASET_DIR / "data.yaml"
    data = yaml.safe_load(data_yaml_path.read_text())
    data["test"] = "images/test"
    data_yaml_path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))

    print(f"Movidos {moved} ficheros. data.yaml actualizado con 'test: images/test'.")
    print(f"Final: train={len(collect_basenames(train_dir))} "
          f"val={len(collect_basenames(val_dir))} "
          f"test={len(collect_basenames(test_img_dir))}")


if __name__ == "__main__":
    main()
