"""Infraestructura del emplazamiento: valla perimetral, caminos de acceso y estaciones de inversores.

La Fase 1 entregó deliberadamente solo paneles y terreno. Este módulo añade
el resto de lo que lleva un emplazamiento fotovoltaico real, para los planos
de establecimiento y el pase de "efecto wow", sin tocar ningún recurso de la
Fase 1.

Dos restricciones dieron forma a todo esto:

* **Estabilidad de la semilla.** La infraestructura toma valores de su
  propio flujo de RNG, así que activarla o desactivarla no puede desplazar
  ni una sola mesa. Una semilla dada produce el mismo parque en ambos casos,
  lo que mantiene reproducibles los mundos de la Fase 1 y permite
  renderizar ambos como un par emparejado.

* **Presupuesto de triángulos.** El mundo de 1000 módulos ya mantiene el
  tiempo real fusionando geometría en lugar de instanciarla, y la
  infraestructura tiene que respetar la misma regla. Un perímetro de 420 m
  con postes cada 3 m son ~140 postes; como modelos individuales eso son
  140 entidades más que la fase amplia (broadphase) de física y la cola de
  renderizado tienen que recorrer cada fotograma. Fusionado en una sola
  malla es un único draw call, así que eso es lo que ocurre a
  continuación. Por la misma razón, el tejido de malla metálica es un
  quad con textura alpha por lado en lugar de alambre modelado.
"""

import math
import os

import numpy as np
from PIL import Image

from . import pv_textures

FENCE_HEIGHT = 2.10      # m, valla de seguridad típica incluyendo brazo de púas
POST_SPACING = 3.00      # m entre postes de línea
POST_SIDE = 0.06         # m, sección cuadrada
ROAD_WIDTH = 4.00        # m, ancho suficiente para un vehículo de servicio
ROAD_Z = 0.02            # m, elevado del plano del suelo para evitar z-fighting

# Bancada de inversor/transformador. Los inversores centrales reales para un
# parque de este tamaño son aproximadamente del tamaño de un contenedor;
# estos tienen el tamaño de una unidad pequeña montada sobre bancada.
INV_SIZE = (2.60, 1.60, 2.10)


# --- texturas -----------------------------------------------------------------

def _chainlink_rgba(rng, px=512, pitch=42, wire=4):
    """Tejido de malla metálica en mosaico como RGBA, con el alfa recortado al alambre.

    La retícula de rombos son dos familias de líneas diagonales, así que se
    obtiene a partir de las dos coordenadas diagonales (x+y) y (x-y) tomadas
    módulo el paso de la malla. Trabajar en la base diagonal en lugar de
    dibujar alambres uno a uno es lo que mantiene esto exactamente en
    mosaico.
    """
    yy, xx = np.mgrid[0:px, 0:px]
    d1 = (xx + yy) % pitch
    d2 = (xx - yy) % pitch
    on = ((np.minimum(d1, pitch - d1) < wire) |
          (np.minimum(d2, pitch - d2) < wire))

    # Acero galvanizado, con suficiente ruido tonal para que un tramo largo
    # de valla no se lea como una pantalla gris plana a distancia.
    grime = pv_textures.fbm_tileable((px, px), rng, octaves=4, k0=6)
    base = 0.62 + 0.16 * grime
    img = np.zeros((px, px, 4), np.float32)
    img[..., 0] = base * 0.98
    img[..., 1] = base * 1.00
    img[..., 2] = base * 1.03
    img[..., 3] = on.astype(np.float32)
    return img


def _gravel(rng, px=1024):
    """Camino de acceso de grava compactada."""
    coarse = pv_textures.fbm_tileable((px, px), rng, octaves=5, k0=8)
    grit = pv_textures.fbm_tileable((px, px), rng, octaves=3, k0=48)
    img = np.zeros((px, px, 3), np.float32)
    v = 0.34 + 0.22 * coarse + 0.14 * grit
    img[..., 0] = v * 1.04
    img[..., 1] = v * 1.00
    img[..., 2] = v * 0.92
    return np.clip(img, 0.0, 1.0)


def _housing(rng, px=512):
    """Carcasa de chapa de acero pintada con tenues costuras de panel verticales."""
    n = pv_textures.fbm_tileable((px, px), rng, octaves=4, k0=10)
    img = np.zeros((px, px, 3), np.float32)
    v = 0.68 + 0.08 * n
    seam = (np.arange(px) % 96 < 2).astype(np.float32)
    v = v - 0.16 * seam[None, :]
    img[..., 0] = v * 0.98
    img[..., 1] = v * 0.99
    img[..., 2] = v * 0.94
    return np.clip(img, 0.0, 1.0)


def build_textures(rng, outdir, asset_pkg):
    tex = os.path.join(outdir, asset_pkg, "materials", "textures")
    os.makedirs(tex, exist_ok=True)

    Image.fromarray(pv_textures.to_u8(_chainlink_rgba(rng)), "RGBA").save(
        os.path.join(tex, "fence_fabric.png"), optimize=True)
    Image.fromarray(pv_textures.to_u8(_gravel(rng)), "RGB").save(
        os.path.join(tex, "road_albedo.png"), optimize=True)
    Image.fromarray(pv_textures.to_u8(_housing(rng)), "RGB").save(
        os.path.join(tex, "inverter_albedo.png"), optimize=True)

    rough = np.clip(0.86 + 0.10 * pv_textures.fbm_tileable(
        (512, 512), rng, octaves=3, k0=12), 0, 1)
    Image.fromarray(pv_textures.to_u8(rough), "L").save(
        os.path.join(tex, "site_roughness.png"), optimize=True)


# --- geometría ------------------------------------------------------------

def _quad(verts, uvs, faces, p0, p1, p2, p3, uv):
    base = len(verts) + 1
    verts += [p0, p1, p2, p3]
    uvs += list(uv)
    faces.append((base, base + 1, base + 2, base + 3))


def _write_obj(path, verts, uvs, faces, normal=(0.0, 0.0, 1.0)):
    with open(path, "w") as f:
        f.write("# generado por solar_farm_gz.site\n")
        for x, y, z in verts:
            f.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
        for u, v in uvs:
            f.write(f"vt {u:.5f} {v:.5f}\n")
        f.write(f"vn {normal[0]:.4f} {normal[1]:.4f} {normal[2]:.4f}\n")
        for quad in faces:
            f.write("f " + " ".join(f"{i}/{i}/1" for i in quad) + "\n")
    return len(faces)


def write_fence_obj(path, x0, y0, x1, y1):
    """Tejido de malla metálica como cuatro quads en mosaico, uno por lado.

    Las UV se repiten en mosaico a lo largo del tramo con una repetición de
    malla cada 1.4 m, así que el tamaño del rombo se mantiene físicamente
    consistente sin importar la longitud del lado.

    Cada lado lleva su propia normal hacia afuera. No es un detalle que se
    pueda omitir: el tejido es vertical, así que una normal compartida
    orientada hacia arriba deja cada panel iluminado como si fuera suelo, y
    la valla se renderiza negra o desaparece por completo. `double_sided`
    en el material controla el culling, no el sombreado, y no lo compensa.
    """
    verts, uvs, faces, normals = [], [], [], []
    corners = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
               ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    for (ax, ay), (bx, by) in corners:
        run = math.hypot(bx - ax, by - ay)
        u = run / 1.4
        # normal hacia afuera: la dirección del tramo rotada -90 grados en planta
        nx, ny = (by - ay) / run, -(bx - ax) / run
        normals.append((nx, ny, 0.0))
        base = len(verts) + 1
        verts += [(ax, ay, 0.0), (bx, by, 0.0),
                  (bx, by, FENCE_HEIGHT), (ax, ay, FENCE_HEIGHT)]
        uvs += [(0.0, 0.0), (u, 0.0), (u, 1.0), (0.0, 1.0)]
        faces.append(((base, base + 1, base + 2, base + 3), len(normals)))

    with open(path, "w") as f:
        f.write("# generado por solar_farm_gz.site\n")
        for x, y, z in verts:
            f.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
        for u, v in uvs:
            f.write(f"vt {u:.5f} {v:.5f}\n")
        for nx, ny, nz in normals:
            f.write(f"vn {nx:.4f} {ny:.4f} {nz:.4f}\n")
        for quad, ni in faces:
            f.write("f " + " ".join(f"{i}/{i}/{ni}" for i in quad) + "\n")
    return len(faces)


def write_posts_obj(path, x0, y0, x1, y1):
    """Postes de línea fusionados en una sola malla."""
    from . import pv_mesh
    verts, faces = [], []
    h = FENCE_HEIGHT + 0.08
    s = POST_SIDE

    def run(ax, ay, bx, by):
        length = math.hypot(bx - ax, by - ay)
        n = max(2, int(round(length / POST_SPACING)) + 1)
        for i in range(n):
            t = i / (n - 1)
            pv_mesh._box(verts, faces,
                         ax + (bx - ax) * t, ay + (by - ay) * t, h / 2.0,
                         s, s, h)

    run(x0, y0, x1, y0)
    run(x1, y0, x1, y1)
    run(x1, y1, x0, y1)
    run(x0, y1, x0, y0)

    # Riel superior. El tejido de malla metálica es sobre todo huecos, así
    # que a la altitud de vuelo de inspección su textura alfa se desvanece
    # a nada con el mipmapping y el perímetro se lee como una línea de
    # postes desconectados. Una valla de seguridad real lleva un riel a lo
    # largo de la parte superior, y esa línea horizontal continua es lo que
    # realmente hace legible una valla desde el aire — así que se gana sus
    # cuatro cajas.
    r = 0.05
    rz = FENCE_HEIGHT
    pv_mesh._box(verts, faces, (x0 + x1) / 2, y0, rz, x1 - x0, r, r)
    pv_mesh._box(verts, faces, (x0 + x1) / 2, y1, rz, x1 - x0, r, r)
    pv_mesh._box(verts, faces, x0, (y0 + y1) / 2, rz, r, y1 - y0, r)
    pv_mesh._box(verts, faces, x1, (y0 + y1) / 2, rz, r, y1 - y0, r)

    with open(path, "w") as f:
        f.write("# generado por solar_farm_gz.site\n")
        for x, y, z in verts:
            f.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
        f.write("vt 0.0 0.0\n")
        for nx, ny, nz in pv_mesh._BOX_NORMALS:
            f.write(f"vn {nx} {ny} {nz}\n")
        for quad, ni in faces:
            f.write("f " + " ".join(f"{i}/1/{ni}" for i in quad) + "\n")
    return len(faces)


def road_rect(x0, y0, x1, y1):
    """Línea central del camino en anillo, retranqueada respecto a la valla.

    El camino se dibuja alrededor de esta línea central, así que el
    retranqueo tiene que despejar media anchura de camino más un margen —
    si no, la calzada se monta sobre la línea de la valla y la grava se
    renderiza a ambos lados de esta.
    """
    d = ROAD_WIDTH / 2.0 + 1.0
    return x0 + d, y0 + d, x1 - d, y1 - d


def write_road_obj(path, x0, y0, x1, y1):
    """Camino perimetral en anillo, colocado justo dentro de la línea de la valla.

    Cuatro rectángulos en lugar de un anillo con inglete real: las esquinas
    se solapan, lo cual es invisible sobre una textura de grava plana y
    evita las matemáticas del inglete.
    """
    x0, y0, x1, y1 = road_rect(x0, y0, x1, y1)
    verts, uvs, faces = [], [], []
    w = ROAD_WIDTH

    def strip(ax, ay, bx, by, horizontal):
        """Una calzada.

        Ambas ramas deben devanarse en sentido antihorario vistas desde
        arriba, o el quad mira hacia el suelo y se descarta por
        backface-culling — desaparece mientras sus vecinas en los otros dos
        lados renderizan con normalidad, lo que parece un fallo de
        colocación en lugar de uno de devanado.
        """
        if horizontal:
            p = [(ax, ay - w / 2), (bx, ay - w / 2),
                 (bx, ay + w / 2), (ax, ay + w / 2)]
            tile = abs(bx - ax) / 6.0
            uv = [(0.0, 0.0), (tile, 0.0), (tile, 1.0), (0.0, 1.0)]
        else:
            p = [(ax - w / 2, ay), (ax + w / 2, ay),
                 (ax + w / 2, by), (ax - w / 2, by)]
            tile = abs(by - ay) / 6.0
            uv = [(0.0, 0.0), (1.0, 0.0), (1.0, tile), (0.0, tile)]
        _quad(verts, uvs, faces,
              (p[0][0], p[0][1], ROAD_Z), (p[1][0], p[1][1], ROAD_Z),
              (p[2][0], p[2][1], ROAD_Z), (p[3][0], p[3][1], ROAD_Z),
              uv)

    strip(x0, y0, x1, y0, True)
    strip(x0, y1, x1, y1, True)
    strip(x0, y0, x0, y1, False)
    strip(x1, y0, x1, y1, False)
    return _write_obj(path, verts, uvs, faces)


# --- fragmento de mundo -----------------------------------------------------

def extent(assignments, span, module_l, margin):
    """Rectángulo de valla alrededor del arreglo, con un retranqueo de O&M."""
    xs = [a[0] for a in assignments]
    ys = [a[1] for a in assignments]
    return (min(xs) - module_l / 2.0 - margin,
            min(ys) - span / 2.0 - margin,
            max(xs) + module_l / 2.0 + margin,
            max(ys) + span / 2.0 + margin)


def inverter_positions(x0, y0, x1, y1, n):
    """Espaciados uniformemente a lo largo del borde interior del camino oeste.

    Toma el rectángulo de la valla y deriva el camino a partir de él, de
    modo que quien llame solo tenga que tratar con un único sistema de
    coordenadas.
    """
    rx0, ry0, _, ry1 = road_rect(x0, y0, x1, y1)
    out = []
    for i in range(n):
        t = (i + 0.5) / n
        out.append((rx0 + ROAD_WIDTH / 2.0 + INV_SIZE[0] * 0.7,
                    ry0 + (ry1 - ry0) * t,
                    0.0))
    return out


def sdf(asset_uri, x0, y0, x1, y1, n_inverters, shadows):
    """Fragmento SDF para toda la infraestructura del emplazamiento. Estático, así que sin coste de física."""
    cast = 'true' if shadows else 'false'
    rough = asset_uri("materials/textures/site_roughness.png")

    def pbr(albedo, metal=0.0, extra=""):
        return f"""
          <material>
            <ambient>0.28 0.28 0.28 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>{extra}
            <pbr><metal>
              <albedo_map>{asset_uri(albedo)}</albedo_map>
              <roughness_map>{rough}</roughness_map>
              <metalness>{metal}</metalness>
            </metal></pbr>
          </material>"""

    inv = ""
    for i, (ix, iy, _) in enumerate(inverter_positions(x0, y0, x1, y1,
                                                       n_inverters)):
        sx, sy, sz = INV_SIZE
        inv += f"""
    <model name="inverter_{i:02d}">
      <static>true</static>
      <pose>{ix:.3f} {iy:.3f} {sz/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="v">
          <cast_shadows>{cast}</cast_shadows>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>{pbr(
              "materials/textures/inverter_albedo.png", metal=0.6)}
        </visual>
      </link>
    </model>"""

    return f"""
    <model name="site_road">
      <static>true</static>
      <link name="link">
        <visual name="v">
          <cast_shadows>false</cast_shadows>
          <geometry><mesh><uri>{asset_uri("meshes/road.obj")}</uri></mesh></geometry>{pbr(
              "materials/textures/road_albedo.png")}
        </visual>
      </link>
    </model>

    <model name="site_fence_posts">
      <static>true</static>
      <link name="link">
        <visual name="v">
          <cast_shadows>{cast}</cast_shadows>
          <geometry><mesh><uri>{asset_uri("meshes/fence_posts.obj")}</uri></mesh></geometry>
          <material>
            <ambient>0.22 0.23 0.24 1</ambient>
            <diffuse>0.55 0.57 0.60 1</diffuse>
            <specular>0.4 0.4 0.4 1</specular>
            <pbr><metal><metalness>0.8</metalness>
              <roughness>0.45</roughness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <!--
      El tejido está recortado por alfa en lugar de modelado como alambre.
      double_sided es obligatorio: un quad de una sola cara es invisible
      desde fuera del perímetro, lo que se ve como una valla que
      desaparece cuando el dron pasa volando junto a ella.
    -->
    <model name="site_fence_fabric">
      <static>true</static>
      <link name="link">
        <visual name="v">
          <cast_shadows>false</cast_shadows>
          <transparency>0.0</transparency>
          <geometry><mesh><uri>{asset_uri("meshes/fence_fabric.obj")}</uri></mesh></geometry>
          <material>
            <double_sided>true</double_sided>
            <ambient>0.35 0.36 0.38 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.25 0.25 0.25 1</specular>
            <pbr><metal>
              <albedo_map>{asset_uri("materials/textures/fence_fabric.png")}</albedo_map>
              <metalness>0.7</metalness>
              <roughness>0.5</roughness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>{inv}"""
