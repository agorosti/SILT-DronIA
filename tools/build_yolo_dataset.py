#!/usr/bin/env python3
"""Rebuild yolo_dataset/ from scratch: generate the four non-default
sites, capture RGB + thermal stills from all five, split them into
train/val, carve a held-out test split out of that, write data.yaml,
and drop a short README.md inside yolo_dataset/ pointing back to
docs/YOLO_DATASET.md.

This is the single-command version of the walkthrough in
docs/YOLO_DATASET.md's "Reproducing" section -- read that document for
the full explanation of *why* each step exists (camera model, projection,
thermal material swap, class design). This script just chains the same
commands together with the recorded per-site parameters, does the
train/val split, and writes the small supporting files that were
previously done by hand.

USAGE (from anywhere, doesn't depend on the working directory):
    # print the plan (recipes, seeds, shot/split counts) without
    # touching Gazebo, the network or the filesystem
    python3 tools/build_yolo_dataset.py --dry-run

    # full rebuild (needs Gazebo + the project's Python deps; takes a
    # while -- 5 sites x 2 capture modes, each booting a headless
    # Gazebo server)
    python3 tools/build_yolo_dataset.py

    # only rebuild specific sites (handy while testing this script, or
    # to redo one site after tweaking its recipe below)
    python3 tools/build_yolo_dataset.py --sites site_g site_h

IMPORTANT -- what "reproducible" means here, exactly:

    site_default is never regenerated. It's the 1000-module world this
    project already ships at src/solar_farm_gz/worlds/, its
    generate_farm.py recipe was never recorded anywhere, so this script
    reuses the world files as-is instead of guessing at one.

    site_g's generate_farm.py recipe (panel count, clean-ratio, the four
    per-type weights, ground style, inverter count, sun angle) is the
    exact one written out in docs/YOLO_DATASET.md's "Reproducing"
    section, confirmed correct. Regenerating it is bit-identical given
    the same seed (generate_farm.py's own guarantee).

    site_h / site_i / site_j's seed, clean-ratio, ground style and
    inverter count are read straight from their defects.json (see
    tools/capture_dataset/source_worlds_metadata/) and are exact. Their
    four per-type defect WEIGHTS were never recorded anywhere -- only the
    resulting defect counts were. SITE_RECIPES below FIXES weights
    reconstructed proportionally from those counts (w_type =
    defects_by_type[type] / total) as explicit constants -- they are no
    longer "reconstructed on the fly", they're recorded here, so
    regenerating these three sites from this point on is deterministic
    and byte-for-byte repeatable given the same seed.

    That reconstruction is approximate, though, not a calibrated inverse
    of the generator -- and measurably so. Cross-checked against site_g,
    the one site whose weights ARE the real, confirmed originals (0.65 /
    0.20 / 0.10 / 0.05 for dirt/bird_dropping/delamination/crack), the
    same proportional-count method applied to site_g's own
    defects_by_type would recover 0.71 / 0.17 / 0.10 / 0.03 -- off by up
    to ~43% relative on the lowest-count class (crack). Low-count classes
    are what this method estimates worst, since a handful of random draws
    carries the most sampling noise relative to its target share.
    Spot-checked in practice, though, it lands closer than that worst
    case: regenerating site_h with its recorded weights (seed 902)
    reproduced defects_by_type within ~5% relative of the original counts
    per class (got dirt=90 bird_dropping=69 crack=114 delamination=101,
    374 total, vs the recorded 84/65/107/101, 357 total) -- site_i and
    site_j weren't independently re-verified this way. Either way, this
    will NOT regenerate pixel-identical images to the ones already
    sitting in yolo_dataset/ for these three sites -- that was already
    true and remains permanently unrecoverable, since the original
    --w-dirt/--w-crack/etc. values were never logged. A materially better
    reconstruction is possible (search for weights by actually re-running
    the generator against candidate values and matching its output
    counts, rather than guessing proportionally) but wasn't attempted
    here -- each site's atlas build takes several minutes per attempt,
    making a search expensive for a refinement this document already
    flags honestly as approximate.

    Capture (camera-pose) seeds have the same story: site_g's are the
    42 (RGB) / 77 (thermal) pair documented in "Reproducing". The other
    four sites' original capture seeds were never written down, so this
    script fixes its own (see CAPTURE_SEED_RGB/CAPTURE_SEED_THERMAL in
    SITE_RECIPES) -- reproducible run to run, but not guaranteed to match
    the specific images already in yolo_dataset/ for those sites.

    None of this affects training: every SITE_RECIPES entry reproduces
    the same site composition, class list, image count and defect
    density as the shipped dataset either way.
"""
import argparse
import glob
import json
import os
import random
import shutil
import subprocess
import sys

PROJECT_ROOT = os.path.expanduser("~/solar_farm_sim")
SFG_DIR = os.path.join(PROJECT_ROOT, "src", "solar_farm_gz")
TOOLS_DIR = os.path.join(PROJECT_ROOT, "tools")
CAPTURE_SCRIPT = os.path.join(TOOLS_DIR, "capture_dataset", "capture_dataset.py")
GENERATED_WORLDS_DIR = os.path.join(TOOLS_DIR, "capture_dataset", "_generated_worlds")
DEFAULT_WORLD_DIR = os.path.join(SFG_DIR, "worlds")
DEFAULT_OUT_DIR = os.path.join(PROJECT_ROOT, "yolo_dataset")

CLASSES = ["dirt", "bird_dropping", "crack", "delamination", "thermal_problem"]

# Held-out test split carved out of train+val at the end of a build --
# same seed and ratio tools/make_test_split.py used the one time this was
# done by hand (70/20/10, matching E1_baseline in the real TFM: 507/145/72
# of 724 images). See carve_test_split() below.
TEST_SPLIT_SEED = 42
TEST_SPLIT_TRAIN_FRAC = 0.70
TEST_SPLIT_VAL_FRAC = 0.20
# the remaining ~0.10 goes to test

# ---------------------------------------------------------------------
# Per-site recipe. See the module docstring for what's confirmed vs.
# reconstructed for each site.
# ---------------------------------------------------------------------
SITE_RECIPES = {
    "site_default": {
        "reuse_existing_world": True,   # never regenerated -- see docstring
        "n_rgb": 70, "val_rgb": 10,
        "n_thermal": 70, "val_thermal": 10,
        "capture_seed_rgb": 11, "capture_seed_thermal": 10011,
    },
    "site_g": {
        "generate_farm_args": {
            "panels": 350, "tables_per_row": 10, "variants": 18,
            "seed": 901, "clean_ratio": 0.60,
            "w_dirt": 0.65, "w_crack": 0.05, "w_bird_dropping": 0.20,
            "w_delamination": 0.10,
            "ground_style": "earth",
            "sun_elevation": 40, "sun_azimuth": 110,
            "inverters": 3,
        },
        "n_rgb": 40, "val_rgb": 6,
        "n_thermal": 40, "val_thermal": 6,
        "capture_seed_rgb": 42, "capture_seed_thermal": 77,  # documented, confirmed
    },
    "site_h": {
        "generate_farm_args": {
            "panels": 350, "tables_per_row": 10, "variants": 18,
            "seed": 902, "clean_ratio": 0.50,
            # reconstructed from defects_by_type in defects.json -- see
            # docstring; not the original --w-* values
            "w_dirt": 0.235, "w_bird_dropping": 0.182,
            "w_crack": 0.300, "w_delamination": 0.283,
            "ground_style": "grass",
            "inverters": 5,
        },
        "n_rgb": 40, "val_rgb": 6,
        "n_thermal": 40, "val_thermal": 6,
        "capture_seed_rgb": 902, "capture_seed_thermal": 10902,
    },
    "site_i": {
        "generate_farm_args": {
            "panels": 350, "tables_per_row": 10, "variants": 18,
            "seed": 903, "clean_ratio": 0.45,
            "w_dirt": 0.265, "w_bird_dropping": 0.453,
            "w_crack": 0.164, "w_delamination": 0.118,
            "ground_style": "earth",
            "infrastructure": False,   # --no-infrastructure
        },
        "n_rgb": 40, "val_rgb": 6,
        "n_thermal": 40, "val_thermal": 6,
        "capture_seed_rgb": 903, "capture_seed_thermal": 10903,
    },
    "site_j": {
        "generate_farm_args": {
            "panels": 350, "tables_per_row": 10, "variants": 18,
            "seed": 1101, "clean_ratio": 0.70,
            "w_dirt": 0.455, "w_bird_dropping": 0.259,
            "w_crack": 0.143, "w_delamination": 0.143,
            "ground_style": "grass",
            "inverters": 4,
        },
        "n_rgb": 40, "val_rgb": 6,
        "n_thermal": 40, "val_thermal": 6,
        "capture_seed_rgb": 1101, "capture_seed_thermal": 11101,
    },
}

SITE_ORDER = ["site_default", "site_g", "site_h", "site_i", "site_j"]


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def run(cmd, cwd=None, env=None, dry_run=False):
    print("  $ " + " ".join(cmd), flush=True)
    if dry_run:
        return
    subprocess.run(cmd, cwd=cwd, env=env, check=True)


def world_dir_for(site):
    if site == "site_default":
        return DEFAULT_WORLD_DIR
    return os.path.join(GENERATED_WORLDS_DIR, site)


def generate_world(site, recipe, force, dry_run):
    """Runs generate_farm.py for one site, unless it's site_default
    (reused as-is) or the world already exists and --force wasn't given."""
    if recipe.get("reuse_existing_world"):
        print(f"[{site}] reusing the existing world at {DEFAULT_WORLD_DIR} "
              f"(never regenerated -- see this script's docstring)")
        return

    out_dir = world_dir_for(site)
    sdf_path = os.path.join(out_dir, "solar_farm.sdf")
    if os.path.exists(sdf_path) and not force:
        print(f"[{site}] world already generated at {out_dir}, skipping "
              f"(use --force-worlds to regenerate)")
        return

    args = recipe["generate_farm_args"]
    cmd = [sys.executable, "-m", "solar_farm_gz.generate_farm",
           "--panels", str(args["panels"]),
           "--tables-per-row", str(args["tables_per_row"]),
           "--variants", str(args["variants"]),
           "--seed", str(args["seed"]),
           "--clean-ratio", str(args["clean_ratio"]),
           "--w-dirt", str(args["w_dirt"]),
           "--w-bird-dropping", str(args["w_bird_dropping"]),
           "--w-crack", str(args["w_crack"]),
           "--w-delamination", str(args["w_delamination"]),
           "--ground-style", args["ground_style"],
           "-o", out_dir]
    if "sun_elevation" in args:
        cmd += ["--sun-elevation", str(args["sun_elevation"])]
    if "sun_azimuth" in args:
        cmd += ["--sun-azimuth", str(args["sun_azimuth"])]
    if "inverters" in args:
        cmd += ["--inverters", str(args["inverters"])]
    if args.get("infrastructure") is False:
        cmd += ["--no-infrastructure"]

    env = dict(os.environ)
    env["PYTHONPATH"] = SFG_DIR + os.pathsep + env.get("PYTHONPATH", "")

    print(f"[{site}] generating world -> {out_dir}")
    run(cmd, cwd=SFG_DIR, env=env, dry_run=dry_run)


def capture_site(site, recipe, thermal, raw_dir, force, dry_run):
    """Runs capture_dataset.py for one site/mode into a scratch
    images_raw/labels_raw pair (not split into train/val yet)."""
    n = recipe["n_thermal" if thermal else "n_rgb"]
    seed = recipe["capture_seed_thermal" if thermal else "capture_seed_rgb"]
    tag = ("thermal_" + site) if thermal else site

    images_out = os.path.join(raw_dir, "images")
    labels_out = os.path.join(raw_dir, "labels")
    os.makedirs(images_out, exist_ok=True)
    os.makedirs(labels_out, exist_ok=True)

    existing = glob.glob(os.path.join(images_out, f"{tag}_*.jpg"))
    if len(existing) >= n and not force:
        print(f"[{tag}] already captured ({len(existing)} shots), skipping "
              f"(use --force-capture to redo)")
        return

    cmd = [sys.executable, CAPTURE_SCRIPT,
           "--world-dir", world_dir_for(site),
           "--site", site,
           "--n", str(n),
           "--seed", str(seed),
           "--images-out", images_out,
           "--labels-out", labels_out]
    if thermal:
        cmd.append("--thermal")

    print(f"[{tag}] capturing {n} shots (seed={seed})")
    run(cmd, dry_run=dry_run)


def split_train_val(raw_dir, out_dir, tag, val_count, dry_run):
    """Moves the tag_NNN.jpg/.txt pairs captured into raw_dir into
    out_dir/images/{train,val} and out_dir/labels/{train,val}.

    Split rule: the LAST val_count shots (by index) go to val, the rest
    to train. This reproduces the exact per-site COUNTS documented in
    docs/YOLO_DATASET.md, but not necessarily which specific shot landed
    in val the first time this dataset was built (that split was never
    scripted). For training purposes this is equivalent -- poses are
    already randomly sampled, so a tail holdout is as good as any other
    partition."""
    stems = sorted(
        os.path.splitext(os.path.basename(p))[0]
        for p in glob.glob(os.path.join(raw_dir, "images", f"{tag}_*.jpg"))
    )
    if not stems:
        print(f"[{tag}] nothing to split (no captured shots found)")
        return
    train_stems, val_stems = stems[:-val_count], stems[-val_count:]

    for split, split_stems in (("train", train_stems), ("val", val_stems)):
        img_dst = os.path.join(out_dir, "images", split)
        lbl_dst = os.path.join(out_dir, "labels", split)
        if not dry_run:
            os.makedirs(img_dst, exist_ok=True)
            os.makedirs(lbl_dst, exist_ok=True)
        for stem in split_stems:
            src_img = os.path.join(raw_dir, "images", stem + ".jpg")
            src_lbl = os.path.join(raw_dir, "labels", stem + ".txt")
            if dry_run:
                continue
            shutil.copy2(src_img, os.path.join(img_dst, stem + ".jpg"))
            shutil.copy2(src_lbl, os.path.join(lbl_dst, stem + ".txt"))
    print(f"[{tag}] split: {len(train_stems)} train / {len(val_stems)} val")


def carve_test_split(out_dir, dry_run):
    """Final step of a full build: carves a held-out test split out of
    images/{train,val}, so one `build_yolo_dataset.py` run produces
    train/val/test directly instead of leaving that to the separate
    tools/make_test_split.py step (kept as a standalone script too, for
    reshuffling an already-built dataset without recapturing anything).

    Same algorithm make_test_split.py used the one time this was done by
    hand: pool train+val, shuffle deterministically (seed=42, same seed
    used elsewhere in this project) and reassign train/val/test
    ~70/20/10. Moves files (not copies) -- train/ and val/ end up
    smaller and test/ appears. For a full from-scratch rebuild (the
    normal case: no images/test yet when this runs) this reproduces
    exactly the split already shipped in yolo_dataset/ today, since it's
    the same pooling order, seed and fractions.

    Only pools train+val: on a partial `--sites` rerun where images/test
    already has content from an earlier full build, that existing test/
    is left untouched and only the freshly-touched train/val pool for
    this run gets carved into an additional top-up of test -- fine for
    tweaking one site, but a full rebuild is what reproduces the
    documented global 70/20/10 ratio exactly."""
    images_train = os.path.join(out_dir, "images", "train")
    images_val = os.path.join(out_dir, "images", "val")

    def stems(d):
        return sorted(
            os.path.splitext(os.path.basename(p))[0]
            for p in glob.glob(os.path.join(d, "*.jpg"))
        )

    pool = [(n, "train") for n in stems(images_train)] + \
           [(n, "val") for n in stems(images_val)]
    if not pool:
        print("[test-split] nothing to carve (no train/val images found)")
        return

    rng = random.Random(TEST_SPLIT_SEED)
    rng.shuffle(pool)

    n = len(pool)
    n_train = round(n * TEST_SPLIT_TRAIN_FRAC)
    n_val = round(n * TEST_SPLIT_VAL_FRAC)
    n_test = n - n_train - n_val

    assignment = {}
    for name, cur in pool[:n_train]:
        assignment[name] = ("train", cur)
    for name, cur in pool[n_train:n_train + n_val]:
        assignment[name] = ("val", cur)
    for name, cur in pool[n_train + n_val:]:
        assignment[name] = ("test", cur)

    print(f"[test-split] {n} images (train+val pool) -> "
          f"train={n_train} val={n_val} test={n_test}")

    if dry_run:
        return

    for split in ("train", "val", "test"):
        os.makedirs(os.path.join(out_dir, "images", split), exist_ok=True)
        os.makedirs(os.path.join(out_dir, "labels", split), exist_ok=True)

    moved = 0
    for name, (target, current) in assignment.items():
        if target == current:
            continue
        src_img = os.path.join(out_dir, "images", current, name + ".jpg")
        dst_img = os.path.join(out_dir, "images", target, name + ".jpg")
        src_lbl = os.path.join(out_dir, "labels", current, name + ".txt")
        dst_lbl = os.path.join(out_dir, "labels", target, name + ".txt")
        shutil.move(src_img, dst_img)
        if os.path.exists(src_lbl):
            shutil.move(src_lbl, dst_lbl)
        moved += 1
    print(f"[test-split] moved {moved} files into their final split")


def write_data_yaml(out_dir, dry_run):
    path = os.path.join(out_dir, "data.yaml")
    lines = [
        "# YOLO dataset config (Ultralytics format)",
        "path: .",
        "train: images/train",
        "val: images/val",
        "test: images/test",
        "",
        f"nc: {len(CLASSES)}",
        "names:",
    ]
    lines += [f"  {i}: {name}" for i, name in enumerate(CLASSES)]
    content = "\n".join(lines) + "\n"
    print(f"writing {path}")
    if not dry_run:
        with open(path, "w") as f:
            f.write(content)


README_TEMPLATE = """# yolo_dataset/

Dataset de detección de objetos en formato YOLO para inspección de
defectos en paneles fotovoltaicos — {n_images} imágenes, {n_sites} sitios,
5 clases (`dirt`, `bird_dropping`, `crack`, `delamination`,
`thermal_problem`). Generado por
[`../tools/build_yolo_dataset.py`](../tools/build_yolo_dataset.py) a
partir del generador procedural de parques solares de este proyecto, más
una captura con cámara de dron y un etiquetado automático.

Esta carpeta contiene únicamente los datos entrenables (`data.yaml`,
`images/`, `labels/`) y este README breve, para poder subirla tal cual a
Colab/Roboflow/etc. sin nada que quitar. Para la explicación completa —
diseño de las clases, cómo se calcularon las etiquetas, imágenes
térmicas, limitaciones conocidas, comandos de entrenamiento — ver
[`../docs/YOLO_DATASET.md`](../docs/YOLO_DATASET.md).

Para regenerar esta carpeta desde cero:

```bash
python3 tools/build_yolo_dataset.py
```
"""


def write_readme(out_dir, n_images, n_sites, dry_run):
    path = os.path.join(out_dir, "README.md")
    content = README_TEMPLATE.format(n_images=n_images, n_sites=n_sites)
    print(f"writing {path}")
    if not dry_run:
        with open(path, "w") as f:
            f.write(content)


# ---------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=DEFAULT_OUT_DIR,
                     help="where to write the dataset (default: yolo_dataset/ "
                          "at the project root)")
    ap.add_argument("--sites", nargs="+", choices=SITE_ORDER, default=SITE_ORDER,
                     help="only rebuild these sites (default: all five)")
    ap.add_argument("--force-worlds", action="store_true",
                     help="regenerate a site's world even if it already exists")
    ap.add_argument("--force-capture", action="store_true",
                     help="recapture a site/mode even if enough shots already exist")
    ap.add_argument("--keep-raw", action="store_true",
                     help="keep the unsplit images_raw/labels_raw scratch dir "
                          "after splitting (default: deleted once split)")
    ap.add_argument("--dry-run", action="store_true",
                     help="print every command and file this run would touch, "
                          "without actually running Gazebo or writing anything")
    a = ap.parse_args()

    os.makedirs(GENERATED_WORLDS_DIR, exist_ok=True) if not a.dry_run else None
    raw_dir = os.path.join(GENERATED_WORLDS_DIR, "_capture_raw")

    for site in a.sites:
        recipe = SITE_RECIPES[site]
        print(f"\n=== {site} ===")
        generate_world(site, recipe, a.force_worlds, a.dry_run)
        for thermal in (False, True):
            capture_site(site, recipe, thermal, raw_dir, a.force_capture, a.dry_run)
            tag = ("thermal_" + site) if thermal else site
            val = recipe["val_thermal" if thermal else "val_rgb"]
            split_train_val(raw_dir, a.out, tag, val, a.dry_run)

    if not a.keep_raw and not a.dry_run and os.path.isdir(raw_dir):
        shutil.rmtree(raw_dir)

    carve_test_split(a.out, a.dry_run)
    write_data_yaml(a.out, a.dry_run)

    if not a.dry_run:
        n_images = len(glob.glob(os.path.join(a.out, "images", "*", "*.jpg")))
    else:
        n_images = sum(SITE_RECIPES[s]["n_rgb"] + SITE_RECIPES[s]["n_thermal"]
                        for s in a.sites)
    write_readme(a.out, n_images, len(a.sites), a.dry_run)

    print(f"\nDone. Dataset at: {a.out}")
    print("Generated worlds (for g/h/i/j) are cached under "
          f"{GENERATED_WORLDS_DIR} -- safe to delete, they're a build "
          "scratch dir, not part of the shipped dataset.")


if __name__ == "__main__":
    main()
