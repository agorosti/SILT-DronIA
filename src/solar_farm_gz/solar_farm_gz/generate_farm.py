#!/usr/bin/env python3
"""Generador parametrizado de mundos de parque solar para Gazebo Harmonic.

Vuelve a ejecutarlo con un --seed distinto para obtener un parque
completamente diferente: distintos tipos de defecto, posiciones,
orientaciones, tamaños y severidades, con el ratio limpio/dañado que pidas.
Nada se coloca a mano.

    # 1000 paneles, 80% limpio, mezcla de defectos por defecto
    python3 -m solar_farm_gz.generate_farm --panels 1000 --out install/...

    # una segunda variación de dataset, más daño, aleatorización distinta
    python3 -m solar_farm_gz.generate_farm --panels 1000 --clean-ratio 0.6 \
        --seed 7

Cada defecto generado también se escribe en defects.json con su tipo, el
módulo en el que se encuentra, y su caja delimitadora en espacio UV del
módulo, así que la referencia (ground truth) del detector sale del
generador en lugar de un etiquetado manual.
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

# Los recursos viven en un paquete de modelo de Gazebo real, referenciado
# mediante URIs model://. Los URIs relativos simples NO se pueden usar
# aquí: gz-sim resuelve un <uri> relativo para una malla contra
# GZ_SIM_RESOURCE_PATH, pero las rutas relativas <albedo_map> /
# <roughness_map> / <normal_map> dentro de <pbr> no se resuelven de la
# misma forma y se descartan silenciosamente, dejando cada superficie sin
# textura y sin que se registre ningún error. model:// se resuelve de
# forma consistente para ambos casos, y se mantiene portable.
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


# --- texturas -----------------------------------------------------------------

def build_atlases(n_variants, clean_ratio, mix, rng, outdir, scale):
    """Renderiza el conjunto de atlases.

    Cada atlas contiene MODULES_PER_TABLE celdas de módulo. Una mesa hace
    referencia a un atlas, así que el ratio limpio/dañado se realiza a
    través del conjunto y se mantiene globalmente sin importar cuántas
    mesas lo reutilicen.
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
                # 1-3 defectos por módulo dañado, cada uno localizado: un
                # módulo nunca queda cubierto uniformemente, que es
                # precisamente el objetivo para la detección
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
        # La Fase 2 lee esto; la Fase 1 solo tiene que no descartarlo
        _save(thm, f"pv_atlas_{v:02d}_thermal.png", "L")
        manifest[f"pv_atlas_{v:02d}"] = cells

    nrm = pv_textures.module_normal_map(rng)
    Image.fromarray(np.tile(nrm, (pv_mesh.ATLAS_ROWS, pv_mesh.ATLAS_COLS, 1)),
                    "RGB").resize((int(aw * scale), int(ah * scale)),
                                  Image.LANCZOS).save(
        os.path.join(tex_dir, "pv_normal.png"), optimize=True)

    return manifest


GROUND_STYLES = ("grass", "earth")

# Tinte ambiental de la escena por estilo de suelo. El suelo rebota la
# mayor parte de la luz indirecta en un emplazamiento abierto, así que
# dejar el tinte de tierra bajo el césped se ve embarrado.
GROUND_AMBIENT = {"earth": "0.30 0.27 0.21", "grass": "0.20 0.26 0.15"}

# Césped exuberante y estresado por sequía, sRGB. Los emplazamientos reales
# son un mosaico entre ambos en lugar de uniformemente uno u otro, que es
# lo que evita que un plano grande en mosaico se lea como un color plano.
_GRASS_LUSH = np.array([0.16, 0.30, 0.10], np.float32)
_GRASS_DRY = np.array([0.45, 0.42, 0.22], np.float32)


def build_ground_texture(rng, outdir, style="grass", px=2048):
    """Albedo de suelo en mosaico.

    'earth' es tierra seca nivelada. 'grass' es vegetación segada, que es
    lo que realmente llevan la mayoría de los emplazamientos industriales
    entre filas — la gestión de la vegetación es una actividad estándar de
    O&M, así que esto no es puramente una elección cosmética.
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
        # Tres escalas, porque el césped no resulta convincente si falta
        # alguna: manchas de humedad a escala de metro, apelmazamiento a
        # escala de decímetro, y una ruptura fina a escala de brizna que
        # sobrevive vista desde 8 m de altura.
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
        f.write("# generado por solar_farm_gz\n")
        for x, y in ((-h, -h), (h, -h), (h, h), (-h, h)):
            f.write(f"v {x:.2f} {y:.2f} 0.00\n")
        for u, v in ((0, 0), (n, 0), (n, n), (0, n)):
            f.write(f"vt {u:.2f} {v:.2f}\n")
        f.write("vn 0 0 1\n")
        f.write("f 1/1/1 2/2/1 3/3/1 4/4/1\n")


# --- mundo ------------------------------------------------------------------

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
<!-- Generado por solar_farm_gz.generate_farm.
     seed={a.seed} panels={a.panels} clean_ratio={a.clean_ratio}
     No editar a mano: vuelve a ejecutar el generador en su lugar. -->
<sdf version="1.10">
  <world name="{a.world_name}">
    <!--
      Paso de 1 ms (1000 Hz), no los 4 ms con los que se conformaría un
      mundo estático. ArduCopter ejecuta un bucle principal a 400 Hz y se
      niega a armar a menos que el giroscopio entregue al menos 1.8x eso
      (720 Hz). Con un paso de 4 ms el simulador le da 250 Hz y cada
      intento de armado falla con "Gyro 0 rate 250Hz < loop rate*1.8" y
      "Main loop slow". Los paneles son estáticos, así que los pasos
      extra cuestan poco: la física no es en lo que este mundo gasta su
      tiempo.
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
      ArduPilot necesita un origen georreferenciado para derivar su
      posición home y su solución GPS. Sin este bloque, SITL arma pero
      nunca obtiene una estimación de posición, así que el despegue en
      modo GUIDED se acepta y luego no hace nada. Las coordenadas son la
      ubicación por defecto propia de SITL de ArduPilot, lo que mantiene
      el parque simulado consistente con las herramientas y los logs
      estándar de ArduPilot.
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


# --- disposición ---------------------------------------------------------

def layout(a, rng, n_tables):
    """Coloca las mesas en una cuadrícula de filas con una pequeña variación de tolerancia de topografía."""
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
    # Flujo independiente, no el rng principal: los dos estilos toman un
    # número distinto de muestras, y compartir el flujo haría que
    # --ground-style perturbara la disposición de mesas más adelante.
    # Mantenerlo separado significa que una misma semilla da el mismo
    # parque bajo cualquiera de las dos coberturas de suelo, así que ambas
    # se pueden renderizar como un par A/B emparejado.
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
    # Asignación equilibrada, no muestreo con reemplazo: cada atlas se usa
    # un número de veces casi igual, así que la fracción de módulos
    # dañados realizada coincide con --clean-ratio en lugar de fluctuar
    # con el sorteo, y ninguna variante se genera y luego queda sin
    # colocar.
    order = np.concatenate([rng.permutation(n_variants)
                            for _ in range(n_tables // n_variants + 1)])
    placed = [(ox, oy, yaw, int(order[i]))
              for i, (ox, oy, yaw, _) in enumerate(placed)]

    infra_sdf = ""
    fence = None
    if a.infrastructure:
        print("[4b/5] site infrastructure", flush=True)
        # Otra vez su propio flujo, por la misma razón que la textura del
        # suelo: los recursos del emplazamiento toman un número variable
        # de muestras, y compartir el rng haría que --no-infrastructure
        # cambiara silenciosamente la distribución de defectos.
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

    # Se informa sobre los módulos realmente colocados en el mundo, no
    # sobre el conjunto de atlases: una variante usada dos veces aporta
    # sus defectos dos veces.
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
