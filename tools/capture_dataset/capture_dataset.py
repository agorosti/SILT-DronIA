#!/usr/bin/env python3
"""Capture a set of inspection-style stills from one generated
solar_farm_sim world and write YOLO-format labels alongside them.

Camera intrinsics/resolution match the real x500_rgb nadir sensor
(models/x500_rgb/model.sdf: 1920x1080, horizontal_fov=1.151917) so a
detector trained on this dataset applies directly to the live
/x500_rgb/nadir feed. The mount is a fixed nadir gimbal (no oblique
views in real operation), so poses stay near-nadir with only the small
pitch/roll jitter a real cruise induces.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import projection as proj  # noqa: E402

SFG_SRC = os.path.expanduser("~/solar_farm_sim/src/solar_farm_gz")
sys.path.insert(0, SFG_SRC)
from solar_farm_gz import capture as cap  # noqa: E402
from solar_farm_gz import flight_video as fv  # noqa: E402 -- _thermal_swap, THERMAL_LOW/HIGH

CLASS_IDS = {"dirt": 0, "bird_dropping": 1, "crack": 2, "delamination": 3}
# Thermal imagery can't tell defect *cause* apart -- a hot spot is a hot
# spot, whether it's soiling, a crack, a dropping or delamination under it --
# so every defect collapses to one class when --thermal is set, instead of
# the 4 RGB can tell apart.
THERMAL_CLASS_ID = 4
THERMAL_CLASS_NAME = "thermal_problem"

# Real x500_rgb nadir_camera sensor spec (models/x500_rgb/model.sdf)
WIDTH, HEIGHT, FOV, RATE = 1920, 1080, 1.151917, 5.0
SETTLE = 3


def gen_poses(defects, rng, n):
    tables = defects["tables_placed"]
    counts = []
    for t in tables:
        cells = defects["atlases"][t["atlas"]]
        counts.append(sum(len(c["defects"]) for c in cells))
    damaged_idx = [i for i, c in enumerate(counts) if c > 0]
    if not damaged_idx:
        damaged_idx = list(range(len(tables)))

    # Real cruise is 8 m; vary around it for scale diversity while staying
    # in the realistic inspection envelope (docs: 8 m -> 10.4 m swath).
    alts = [5.0, 6.5, 8.0, 10.0, 13.0]
    yaws = [0.0, math.pi / 2, math.pi, -math.pi / 2, math.pi / 4]

    poses = []
    for i in range(n):
        prefer_damaged = rng.random() < 0.8
        pool = damaged_idx if prefer_damaged else list(range(len(tables)))
        t = tables[int(rng.choice(pool))]
        ox, oy = t["pose_xyzyaw"][0], t["pose_xyzyaw"][1]
        alt = float(rng.choice(alts))
        # Fixed nadir mount: only the small pitch/roll a real GUIDED cruise
        # induces while translating, not a free oblique gimbal.
        pitch = math.pi / 2 + float(rng.normal(0, 0.05))
        roll = float(rng.normal(0, 0.03))
        yaw = float(rng.choice(yaws))
        jx = float(rng.normal(0, 1.5))
        jy = float(rng.normal(0, 2.5))
        x, y = ox + jx, oy + jy
        z = proj.PIVOT_Z + alt
        poses.append((x, y, z, roll, pitch, yaw))
    return poses


def label_for_pose(defects, pose, thermal=False):
    n_mod = defects["modules_per_table"]
    cam = proj.Camera(pose, WIDTH, HEIGHT, FOV)
    lines = []
    for t in defects["tables_placed"]:
        ox, oy, _, yaw = t["pose_xyzyaw"]
        cells = defects["atlases"][t["atlas"]]
        for c in cells:
            for d in c["defects"]:
                cx, cy, w, h = d["bbox_uv_cxcywh"]
                corners = proj.defect_world_corners(
                    cx, cy, w, h, c["module_index"], n_mod, ox, oy, yaw)
                bbox = proj.project_defect_bbox(cam, corners, yaw)
                if bbox is None:
                    continue
                bcx, bcy, bw, bh = bbox
                cid = THERMAL_CLASS_ID if thermal else CLASS_IDS[d["type"]]
                lines.append(f"{cid} {bcx:.6f} {bcy:.6f} {bw:.6f} {bh:.6f}")
    return lines


def thermal_colour(img):
    """Raw render of the thermal-swapped world -> calibrated false-colour,
    identical treatment to flight_video.composite()'s thermal path (same
    THERMAL_LOW/HIGH, same INFERNO map) so RGB and thermal images in this
    dataset look like they came from the same camera family the videos
    show."""
    import cv2
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).astype(np.float32)
    span = fv.THERMAL_HIGH - fv.THERMAL_LOW
    gray = np.clip((gray - fv.THERMAL_LOW) * (255.0 / span), 0, 255)
    gray = gray.astype(np.uint8)
    return cv2.cvtColor(cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO),
                        cv2.COLOR_BGR2RGB)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world-dir", required=True)
    ap.add_argument("--site", required=True, help="short tag, e.g. site_g")
    ap.add_argument("--n", type=int, default=18)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--images-out", required=True)
    ap.add_argument("--labels-out", required=True)
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--thermal", action="store_true",
                    help="render the thermal-swapped world (material swap, "
                         "shadows off, calibrated false-colour) and label "
                         "every defect as the single 'thermal_problem' "
                         "class instead of its RGB-distinguishable type")
    a = ap.parse_args()

    os.makedirs(a.images_out, exist_ok=True)
    os.makedirs(a.labels_out, exist_ok=True)

    world_sdf = os.path.join(a.world_dir, "solar_farm.sdf")
    defects = json.load(open(os.path.join(a.world_dir, "defects.json")))
    rng = np.random.default_rng(a.seed)
    poses = gen_poses(defects, rng, a.n)

    sdf = open(world_sdf).read()
    if a.thermal:
        # build_env() below resolves model:// against world_sdf's directory,
        # not the temp file's -- swapping the *text* (not the file) keeps
        # that resolution intact while pointing every table's albedo_map at
        # its _thermal.png sibling and turning shadows off.
        sdf = fv._thermal_swap(sdf)
    env = cap.build_env(world_sdf)
    wname = cap.world_name(sdf)
    start = " ".join(f"{v:.4f}" for v in poses[0])
    tag = "thermal_" + a.site if a.thermal else a.site
    tmp = f"/tmp/_capture_{tag}.sdf"
    open(tmp, "w").write(cap.inject_camera(sdf, start, WIDTH, HEIGHT, FOV, RATE))

    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.transport13 import Node

    print(f"[{tag}] loading world...", flush=True)
    proc = cap.start_server(tmp, env, verbose=False)
    sink = cap.FrameSink()
    node = Node()
    node.subscribe(GzImage, cap.CAM_TOPIC, sink)
    if not cap.wait_for_frames(sink, proc, 1, a.timeout):
        cap.stop_server(proc)
        raise SystemExit(f"[{tag}] no first frame within {a.timeout}s")
    print(f"[{tag}] loaded, capturing {len(poses)} shots", flush=True)

    n_saved, n_boxes_total = 0, 0
    for i, pose in enumerate(poses):
        if not cap.set_pose(env, wname, pose):
            print(f"  [{tag}] shot {i}: set_pose failed, skipping",
                  file=sys.stderr)
            continue
        target = sink.count + SETTLE + 1
        if not cap.wait_for_frames(sink, proc, target, 30.0):
            print(f"  [{tag}] shot {i}: frame timeout, skipping",
                  file=sys.stderr)
            continue
        img = sink.img.copy()
        if a.thermal:
            img = thermal_colour(img)

        lines = label_for_pose(defects, pose, thermal=a.thermal)
        stem = f"{tag}_{i:03d}"
        Image.fromarray(img).save(os.path.join(a.images_out, stem + ".jpg"),
                                  quality=92)
        with open(os.path.join(a.labels_out, stem + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        n_saved += 1
        n_boxes_total += len(lines)
        print(f"  [{tag}] {stem}  boxes={len(lines)}", flush=True)

    cap.stop_server(proc)
    del node
    print(f"[{tag}] done: {n_saved}/{len(poses)} shots, "
          f"{n_boxes_total} boxes total")


if __name__ == "__main__":
    sys.exit(main())
