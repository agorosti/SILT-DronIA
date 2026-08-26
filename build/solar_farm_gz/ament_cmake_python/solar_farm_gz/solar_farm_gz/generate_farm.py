#!/usr/bin/env python3
"""Parameterised solar-farm world generator for Gazebo Harmonic.

Re-run with a different --seed to get a completely different farm: different
defect types, positions, orientations, sizes and severities, at whatever
clean/damaged ratio you ask for. Nothing is placed by hand.

    # 1000 panels, 80% clean, default defect mix
    python3 -m solar_farm_gz.generate_farm --panels 1000 --out install/...

    # a second dataset variation, more damage, different randomisation
    python3 -m solar_farm_gz.generate_farm --panels 1000 --clean-ratio 0.6 \
        --seed 7

Every generated defect is also written to defects.json with its type, the
module it sits on, and its bounding box in module UV space, so detector
ground truth comes out of the generator rather than out of hand labelling.
"""

import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image

from . import pv_mesh, pv_textures, site
from .pv_textures import CELL_PX_H, CELL_PX_W, DEFECT_TYPES

MODULES_PER_TABLE = pv_mesh.ATLAS_COLS * pv_mesh.ATLAS_ROWS   # 10

# Assets live in a real Gazebo model package addressed by model:// URIs.
# Plain relative URIs are NOT usable here: gz-sim resolves a relative <uri>
# for a mesh against GZ_SIM_RESOURCE_PATH, but relative <albedo_map> /
# <roughness_map> / <normal_map> paths inside <pbr> are not resolved the same
# way and are silently dropped, leaving every surface untextured with no error
# logged. model:// resolves consistently for both, and stays portable.
ASSET_PKG = "solar_farm_assets"


def asset_uri(rel):
    return f"model://{ASSET_PKG}/{rel}"


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>solar_farm_assets</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>
    Generated PV table meshes and defect texture atlases.
  </description>
</model>
"""


# --- textures ---------------------------------------------------------------

def build_atlases(n_variants, clean_ratio, mix, rng, outdir, scale):
    """Render the atlas pool.

    Each atlas holds MODULES_PER_TABLE module cells. A table references one
    atlas, so the clean/damaged ratio is realised across the pool and holds
    globally however many tables reuse it.
    """
    tex_dir = os.path.join(outdir, ASSET_PKG, "materials", "textures")
    os.makedirs(tex_dir, exist_ok=True)

    n_cells = n_variants * MODULES_PER_TABLE
    n_bad = int(round(n_cells * (1.0 - clean_ratio)))
    damaged = np.zeros(n_cells, bool)
    damaged[rng.permutation(n_cells)[:n_bad]] = True

    kinds = list(DEFECT_TYPES)
    weights = np.array([mix[k] for k in kinds], float)
    weights /= weights.sum()

    aw = CELL_PX_W * pv_mesh.ATLAS_COLS
    ah = CELL_PX_H * pv_mesh.ATLAS_ROWS
    manifest, cell_i = {}, 0

    for v in range(n_variants):
        alb = np.zeros((ah, aw, 3), np.float32)
        rgh = np.zeros((ah, aw), np.float32)
        thm = np.zeros((ah, aw), np.float32)
        cells = []

        for m in range(MODULES_PER_TABLE):
            col, row = m % pv_mesh.ATLAS_COLS, m // pv_mesh.ATLAS_COLS
            x0, y0 = col * CELL_PX_W, row * CELL_PX_H

            plan = []
            if damaged[cell_i]:
                # 1-3 defects per damaged module, each localised: a module is
                # never uniformly covered, which is the point for detection
                for _ in range(int(rng.integers(1, 4))):
                    k = kinds[int(rng.choice(len(kinds), p=weights))]
                    plan.append((k, float(rng.uniform(0.35, 1.0))))

            a, r, t, found = pv_textures.render_module(rng, plan)
            alb[y0:y0 + CELL_PX_H, x0:x0 + CELL_PX_W] = a
            rgh[y0:y0 + CELL_PX_H, x0:x0 + CELL_PX_W] = r
            thm[y0:y0 + CELL_PX_H, x0:x0 + CELL_PX_W] = t

            cells.append({
                "module_index": m,
                "atlas_cell": [col, row],
                "clean": not found,
                "defects": [{"type": d.kind,
                             "severity": round(d.severity, 3),
                             "bbox_uv_cxcywh": [round(q, 5) for q in d.bbox_uv()],
                             **d.meta} for d in found],
            })
            cell_i += 1

        def _save(arr, name, mode):
            img = Image.fromarray(pv_textures.to_u8(arr), mode)
            if scale != 1.0:
                img = img.resize((int(aw * scale), int(ah * scale)),
                                 Image.LANCZOS)
            img.save(os.path.join(tex_dir, name), optimize=True)

        _save(alb, f"pv_atlas_{v:02d}_albedo.png", "RGB")
        _save(rgh, f"pv_atlas_{v:02d}_roughness.png", "L")
        # Phase 2 reads this; Phase 1 only has to not throw it away
        _save(thm, f"pv_atlas_{v:02d}_thermal.png", "L")
        manifest[f"pv_atlas_{v:02d}"] = cells

    nrm = pv_textures.module_normal_map(rng)
    Image.fromarray(np.tile(nrm, (pv_mesh.ATLAS_ROWS, pv_mesh.ATLAS_COLS, 1)),
                    "RGB").resize((int(aw * scale), int(ah * scale)),
                                  Image.LANCZOS).save(
        os.path.join(tex_dir, "pv_normal.png"), optimize=True)

    return manifest


GROUND_STYLES = ("grass", "earth")

# Scene ambient tint per ground style. The ground bounces most of the indirect
# light in an open site, so leaving the earth tint under grass reads muddy.
GROUND_AMBIENT = {"earth": "0.30 0.27 0.21", "grass": "0.20 0.26 0.15"}

# Lush and drought-stressed grass, sRGB. Real sites are patchy between the two
# rather than uniformly either, which is what stops a large tiled plane from
# reading as flat colour.
_GRASS_LUSH = np.array([0.16, 0.30, 0.10], np.float32)
_GRASS_DRY = np.array([0.45, 0.42, 0.22], np.float32)


def build_ground_texture(rng, outdir, style="grass", px=2048):
    """Tileable ground albedo.

    'earth' is dry graded soil. 'grass' is mown vegetation, which is what most
    utility sites actually carry between rows — vegetation management is a
    standard O&M activity, so this is not purely a cosmetic choice.
    """
    tex_dir = os.path.join(outdir, ASSET_PKG, "materials", "textures")
    img = np.zeros((px, px, 3), np.float32)

    if style == "earth":
        base = pv_textures.fbm_tileable((px, px), rng, octaves=6, k0=3)
        grit = pv_textures.fbm_tileable((px, px), rng, octaves=3, k0=24)
        img[..., 0] = 0.44 + 0.20 * base + 0.07 * grit
        img[..., 1] = 0.38 + 0.18 * base + 0.07 * grit
        img[..., 2] = 0.29 + 0.13 * base + 0.05 * grit
    else:
        # Three scales, because grass fails to convince if any one is missing:
        # metre-scale moisture patches, decimetre-scale clumping, and a fine
        # blade-scale breakup that survives being viewed from 8 m up.
        patch = pv_textures.fbm_tileable((px, px), rng, octaves=5, k0=3)
        clump = pv_textures.fbm_tileable((px, px), rng, octaves=4, k0=12)
        blade = pv_textures.fbm_tileable((px, px), rng, octaves=2, k0=96,
                                         gain=0.6)
        dry = np.clip(0.45 * patch + 0.40 * clump, 0.0, 1.0)
        img[:] = _GRASS_LUSH + (_GRASS_DRY - _GRASS_LUSH) * dry[..., None]
        img *= (0.80 + 0.34 * blade)[..., None]

    Image.fromarray(pv_textures.to_u8(img), "RGB").save(
        os.path.join(tex_dir, "ground_albedo.png"), optimize=True)


def write_ground_mesh(path, size, tile=25.0):
    h = size / 2.0
    n = size / tile
    with open(path, "w") as f:
        f.write("# generated by solar_farm_gz\n")
        for x, y in ((-h, -h), (h, -h), (h, h), (-h, h)):
            f.write(f"v {x:.2f} {y:.2f} 0.00\n")
        for u, v in ((0, 0), (n, 0), (n, n), (0, n)):
            f.write(f"vt {u:.2f} {v:.2f}\n")
        f.write("vn 0 0 1\n")
        f.write("f 1/1/1 2/2/1 3/3/1 4/4/1\n")


# --- world ------------------------------------------------------------------

def pbr_block(albedo, rough, normal=None, metal=0.0):
    n = f"\n            <normal_map>{normal}</normal_map>" if normal else ""
    return f"""
        <material>
          <ambient>0.25 0.25 0.25 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.4 0.4 0.4 1</specular>
          <pbr><metal>
            <albedo_map>{albedo}</albedo_map>
            <roughness_map>{rough}</roughness_map>{n}
            <metalness>{metal}</metalness>
          </metal></pbr>
        </material>"""


def table_sdf(idx, ox, oy, yaw, variant, n_mod, shadows):
    cx, cy, cz = pv_mesh.collision_box(n_mod)
    a = asset_uri(f"materials/textures/pv_atlas_{variant:02d}_albedo.png")
    r = asset_uri(f"materials/textures/pv_atlas_{variant:02d}_roughness.png")
    nrm = asset_uri("materials/textures/pv_normal.png")
    glass_mesh = asset_uri("meshes/pv_glass.obj")
    rack_mesh = asset_uri("meshes/pv_rack.obj")
    return f"""
  <model name="table_{idx:04d}">
    <static>true</static>
    <pose>{ox:.3f} {oy:.3f} 0 0 0 {yaw:.5f}</pose>
    <link name="table">
      <collision name="glass_col">
        <pose>{pv_mesh.glass_pose()}</pose>
        <geometry><box><size>{cx:.3f} {cy:.3f} {cz:.3f}</size></box></geometry>
      </collision>
      <visual name="glass">
        <pose>{pv_mesh.glass_pose()}</pose>
        <cast_shadows>{'true' if shadows else 'false'}</cast_shadows>
        <geometry><mesh><uri>{glass_mesh}</uri></mesh></geometry>{
            pbr_block(a, r, nrm, 0.1)}
      </visual>
      <visual name="rack">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>{rack_mesh}</uri></mesh></geometry>
        <material>
          <ambient>0.18 0.18 0.20 1</ambient>
          <diffuse>0.40 0.41 0.43 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <pbr><metal><roughness>0.45</roughness>
            <metalness>0.85</metalness></metal></pbr>
        </material>
      </visual>
    </link>
  </model>"""


def build_world(a, assignments, ground_size, infra=""):
    ground_mesh = asset_uri("meshes/ground.obj")
    ground_tex = asset_uri("materials/textures/ground_albedo.png")
    tables = "".join(
        table_sdf(i, ox, oy, yaw, v, a.modules_per_table, a.shadows)
        for i, (ox, oy, yaw, v) in enumerate(assignments))

    sun_el = math.radians(a.sun_elevation)
    sun_az = math.radians(a.sun_azimuth)
    d = (-math.cos(sun_el) * math.cos(sun_az),
         -math.cos(sun_el) * math.sin(sun_az),
         -math.sin(sun_el))

    return f"""<?xml version="1.0" ?>
<!-- Generated by solar_farm_gz.generate_farm.
     seed={a.seed} panels={a.panels} clean_ratio={a.clean_ratio}
     Do not edit by hand: re-run the generator instead. -->
<sdf version="1.10">
  <world name="{a.world_name}">
    <!--
      1 ms step (1000 Hz), not the 4 ms a static world would be happy with.
      ArduCopter runs a 400 Hz main loop and refuses to arm unless the gyro
      delivers at least 1.8x that (720 Hz). At a 4 ms step the simulator feeds
      it 250 Hz and every arm attempt fails with "Gyro 0 rate 250Hz < loop
      rate*1.8" and "Main loop slow". The panels are static, so the extra
      steps cost little: physics is not what this world spends its time on.
    -->
    <physics name="default" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <!--
      ArduPilot needs a georeferenced origin to derive its home position and
      GPS solution. Without this block SITL arms but never gets a position
      estimate, so GUIDED-mode takeoff is accepted and then does nothing.
      The coordinates are ArduPilot's own SITL default location, which keeps
      the simulated farm consistent with stock ArduPilot tooling and logs.
    -->
    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <scene>
      <ambient>0.50 0.51 0.55 1</ambient>
      <background>0.62 0.72 0.85 1</background>
      <shadows>{'true' if a.shadows else 'false'}</shadows>
      <grid>false</grid>
      <sky></sky>
    </scene>

    <light type="directional" name="sun">
      <pose>0 0 60 0 0 0</pose>
      <cast_shadows>{'true' if a.shadows else 'false'}</cast_shadows>
      <diffuse>1.00 0.97 0.90 1</diffuse>
      <specular>0.35 0.35 0.33 1</specular>
      <direction>{d[0]:.4f} {d[1]:.4f} {d[2]:.4f}</direction>
      <attenuation><range>1000</range><constant>0.9</constant>
        <linear>0.0</linear><quadratic>0.0</quadratic></attenuation>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal>
            <size>{ground_size} {ground_size}</size></plane></geometry>
          <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="v">
          <geometry><mesh><uri>{ground_mesh}</uri></mesh></geometry>
          <material>
            <ambient>{GROUND_AMBIENT[a.ground_style]} 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
            <pbr><metal>
              <albedo_map>{ground_tex}</albedo_map>
              <roughness>0.95</roughness><metalness>0.0</metalness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>{tables}{infra}
  </world>
</sdf>
"""


# --- layout -----------------------------------------------------------------

def layout(a, rng, n_tables):
    """Place tables on a row grid with small survey-tolerance jitter."""
    span = pv_mesh.table_span(a.modules_per_table)
    per_row = a.tables_per_row
    out = []
    for t in range(n_tables):
        col, row = t % per_row, t // per_row
        ox = row * a.row_pitch + rng.normal(0, a.jitter_m)
        oy = col * (span + a.table_gap) + rng.normal(0, a.jitter_m)
        yaw = rng.normal(0, math.radians(a.jitter_deg))
        out.append((ox, oy, yaw, 0))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Generate a randomised solar-farm world for Gazebo Harmonic.")
    g = p.add_argument_group("farm")
    g.add_argument("--panels", type=int, default=1000,
                   help="total PV modules (rounded to whole tables)")
    g.add_argument("--modules-per-table", type=int, default=MODULES_PER_TABLE,
                   help="must match the atlas grid (default 10)")
    g.add_argument("--tables-per-row", type=int, default=10)
    g.add_argument("--row-pitch", type=float, default=6.5,
                   help="metres between row centrelines")
    g.add_argument("--table-gap", type=float, default=1.2)
    g.add_argument("--jitter-m", type=float, default=0.04)
    g.add_argument("--jitter-deg", type=float, default=0.6)

    g = p.add_argument_group("defects")
    g.add_argument("--clean-ratio", type=float, default=0.80,
                   help="fraction of modules with no defects")
    g.add_argument("--variants", type=int, default=20,
                   help="distinct atlases in the pool; more = less repetition, "
                        "more texture memory")
    for k, dv in (("dirt", 0.45), ("bird-dropping", 0.25),
                  ("delamination", 0.18), ("crack", 0.12)):
        g.add_argument(f"--w-{k}", type=float, default=dv,
                       help=f"relative weight of {k.replace('-', '_')} defects")

    g = p.add_argument_group("environment")
    g.add_argument("--sun-elevation", type=float, default=55.0)
    g.add_argument("--sun-azimuth", type=float, default=140.0)
    g.add_argument("--ground-style", choices=GROUND_STYLES, default="grass",
                   help="ground cover: mown vegetation (default) or bare "
                        "graded soil")
    g.add_argument("--shadows", action="store_true", default=True)
    g.add_argument("--no-shadows", dest="shadows", action="store_false")

    g = p.add_argument_group("site infrastructure")
    g.add_argument("--infrastructure", action="store_true", default=True,
                   help="perimeter fence, ring road and inverter stations")
    g.add_argument("--no-infrastructure", dest="infrastructure",
                   action="store_false")
    g.add_argument("--fence-margin", type=float, default=8.0,
                   help="metres of O&M setback between array and fence")
    g.add_argument("--inverters", type=int, default=4,
                   help="inverter/transformer skids along the west road")

    g = p.add_argument_group("output")
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("--texture-scale", type=float, default=1.0,
                   help="downscale atlases, e.g. 0.5 for low-VRAM machines")
    g.add_argument("--world-name", default="solar_farm")
    g.add_argument("-o", "--out", default="worlds",
                   help="package directory to write into")
    a = p.parse_args(argv)

    if a.modules_per_table != MODULES_PER_TABLE:
        p.error(f"--modules-per-table must be {MODULES_PER_TABLE} "
                "(the atlas is a 5x2 grid)")
    if not 0.0 <= a.clean_ratio <= 1.0:
        p.error("--clean-ratio must be in [0, 1]")

    rng = np.random.default_rng(a.seed)
    n_tables = max(1, round(a.panels / a.modules_per_table))
    n_modules = n_tables * a.modules_per_table
    n_variants = max(1, min(a.variants, n_tables))

    pkg = os.path.join(a.out, ASSET_PKG)
    os.makedirs(os.path.join(pkg, "meshes"), exist_ok=True)
    with open(os.path.join(pkg, "model.config"), "w") as f:
        f.write(MODEL_CONFIG)

    print(f"[1/5] atlases: {n_variants} variants "
          f"({n_variants * MODULES_PER_TABLE} distinct modules)", flush=True)
    mix = {"dirt": a.w_dirt, "bird_dropping": a.w_bird_dropping,
           "delamination": a.w_delamination, "crack": a.w_crack}
    manifest = build_atlases(n_variants, a.clean_ratio, mix, rng, a.out,
                             a.texture_scale)

    print(f"[2/5] ground texture ({a.ground_style})", flush=True)
    # Independent stream, not the main rng: the two styles draw a different
    # number of samples, and sharing the stream would make --ground-style
    # perturb the table layout downstream. Keeping it separate means one seed
    # gives the same farm under either ground cover, so the two can be
    # rendered as a matched A/B pair.
    build_ground_texture(np.random.default_rng([a.seed, 0x62726F]),
                         a.out, a.ground_style)

    print("[3/5] meshes", flush=True)
    md = os.path.join(pkg, "meshes")
    pv_mesh.write_glass_obj(os.path.join(md, "pv_glass.obj"),
                            a.modules_per_table)
    pv_mesh.write_rack_obj(os.path.join(md, "pv_rack.obj"),
                           a.modules_per_table)

    span = pv_mesh.table_span(a.modules_per_table)
    extent = max(n_tables // a.tables_per_row + 1, 2) * a.row_pitch
    width = a.tables_per_row * (span + a.table_gap)
    ground = float(max(extent, width) * 1.6 + 60)
    write_ground_mesh(os.path.join(md, "ground.obj"), ground)

    print(f"[4/5] layout: {n_tables} tables, {n_modules} modules", flush=True)
    placed = layout(a, rng, n_tables)
    # Balanced assignment, not sampling with replacement: every atlas is used
    # a near-equal number of times, so the realised damaged-module fraction
    # matches --clean-ratio instead of drifting with the draw, and no variant
    # is generated and then left unplaced.
    order = np.concatenate([rng.permutation(n_variants)
                            for _ in range(n_tables // n_variants + 1)])
    placed = [(ox, oy, yaw, int(order[i]))
              for i, (ox, oy, yaw, _) in enumerate(placed)]

    infra_sdf = ""
    fence = None
    if a.infrastructure:
        print("[4b/5] site infrastructure", flush=True)
        # Its own stream again, for the same reason as the ground texture: the
        # site assets draw a variable number of samples, and sharing rng would
        # make --no-infrastructure silently change the defect distribution.
        site_rng = np.random.default_rng([a.seed, 0x517E])
        fence = site.extent(placed, span, pv_mesh.MODULE_L, a.fence_margin)
        site.build_textures(site_rng, a.out, ASSET_PKG)
        site.write_fence_obj(os.path.join(md, "fence_fabric.obj"), *fence)
        site.write_posts_obj(os.path.join(md, "fence_posts.obj"), *fence)
        site.write_road_obj(os.path.join(md, "road.obj"), *fence)
        infra_sdf = site.sdf(asset_uri, *fence, a.inverters, a.shadows)

    print("[5/5] world + manifest", flush=True)
    world_path = os.path.join(a.out, f"{a.world_name}.sdf")
    with open(world_path, "w") as f:
        f.write(build_world(a, placed, ground, infra_sdf))

    # Report over the modules actually placed in the world, not over the
    # atlas pool: a variant used twice contributes its defects twice.
    n_def, n_bad, per_kind = 0, 0, {k: 0 for k in DEFECT_TYPES}
    for _, _, _, v in placed:
        for c in manifest[f"pv_atlas_{v:02d}"]:
            n_def += len(c["defects"])
            n_bad += 0 if c["clean"] else 1
            for d in c["defects"]:
                per_kind[d["type"]] += 1

    with open(os.path.join(a.out, "defects.json"), "w") as f:
        json.dump({
            "seed": a.seed,
            "modules": n_modules,
            "tables": n_tables,
            "modules_per_table": a.modules_per_table,
            "clean_ratio_requested": a.clean_ratio,
            "clean_ratio_actual": round(1.0 - n_bad / n_modules, 4),
            "defect_instances": n_def,
            "defects_by_type": per_kind,
            "ground_style": a.ground_style,
            "infrastructure": ({
                "fence_rect_xyxy": [round(v, 3) for v in fence],
                "fence_height_m": site.FENCE_HEIGHT,
                "road_width_m": site.ROAD_WIDTH,
                "inverters": a.inverters,
            } if a.infrastructure else None),
            "module_size_m": [pv_mesh.MODULE_W, pv_mesh.MODULE_L],
            "tilt_deg": round(math.degrees(pv_mesh.TILT_RAD), 2),
            "atlases": manifest,
            "tables_placed": [
                {"index": i, "pose_xyzyaw": [round(ox, 3), round(oy, 3),
                                             0.0, round(yaw, 5)],
                 "atlas": f"pv_atlas_{v:02d}"}
                for i, (ox, oy, yaw, v) in enumerate(placed)],
        }, f, indent=1)

    frac = n_bad / n_modules
    print(f"\n  world     {world_path}")
    print(f"  tables    {n_tables}  modules {n_modules}")
    print(f"  damaged   {frac * 100:.1f}% of modules "
          f"(requested {(1 - a.clean_ratio) * 100:.0f}%)")
    print(f"  defects   {n_def} instances  " +
          "  ".join(f"{k}={v}" for k, v in per_kind.items()))
    print(f"  manifest  {os.path.join(a.out, 'defects.json')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
