"""Generación de malla para una mesa fotovoltaica de inclinación fija.

El número de draw calls es la restricción principal en este mundo, no el
número de polígonos. Un parque de 1000 paneles construido con un visual por
módulo renderiza a 0.12x tiempo real en gráficos integrados; el mismo
parque con cada mesa fusionada en una sola malla renderiza a 0.75x. Así que
una mesa es exactamente dos mallas:

    glass_<variant>.obj   los N módulos como una única superficie fusionada,
                          cada módulo mapeado por UV a su propia celda de un
                          atlas de textura compartido
    rack.obj              vigas de torsión y postes, idénticos para cada
                          mesa y por tanto cargados una vez e instanciados
                          por Gazebo

Eso son 2 draw calls por mesa en lugar de ~15, y es la razón por la que los
defectos viven en el atlas en lugar de en la geometría.
"""

import math

# geometría de la mesa, en metros
MODULE_W = 1.05          # a lo ancho de la fila
MODULE_L = 2.10          # cuesta arriba
MODULE_GAP = 0.02
TILT_RAD = math.radians(28.0)
PIVOT_Z = 1.60           # altura de la línea central de la mesa
ATLAS_COLS, ATLAS_ROWS = 5, 2


def _tilt(x, z):
    """Rota un punto en el plano XZ de la mesa según el ángulo de inclinación."""
    c, s = math.cos(TILT_RAD), math.sin(TILT_RAD)
    return x * c + z * s, -x * s + z * c + PIVOT_Z


def table_span(n_modules):
    return n_modules * MODULE_W + (n_modules - 1) * MODULE_GAP


def write_glass_obj(path, n_modules):
    """Superficie de vidrio fusionada. El módulo m mapea a la celda de atlas (m%5, m//5)."""
    verts, uvs, faces = [], [], []
    half_l = MODULE_L / 2.0
    pitch = MODULE_W + MODULE_GAP

    for m in range(n_modules):
        yc = (m - (n_modules - 1) / 2.0) * pitch
        y0, y1 = yc - MODULE_W / 2.0, yc + MODULE_W / 2.0

        col, row = m % ATLAS_COLS, (m // ATLAS_COLS) % ATLAS_ROWS
        u0, u1 = col / ATLAS_COLS, (col + 1) / ATLAS_COLS
        # la v de OBJ va de abajo hacia arriba, las filas de la imagen de arriba hacia abajo
        v1, v0 = 1.0 - row / ATLAS_ROWS, 1.0 - (row + 1) / ATLAS_ROWS

        base = len(verts) + 1
        # el borde cuesta arriba (-x) es el alto; +x es el bajo, orientado al sur
        verts += [(-half_l, y0, 0.0), (half_l, y0, 0.0),
                  (half_l, y1, 0.0), (-half_l, y1, 0.0)]
        uvs += [(u0, v1), (u0, v0), (u1, v0), (u1, v1)]
        faces.append((base, base + 1, base + 2, base + 3))

    # Deliberadamente sin mtllib/usemtl. Una malla que trae su propio
    # material sobrescribe el bloque SDF <material> en gz-sim, lo que
    # fijaría cada mesa a un único atlas y anularía el conjunto de
    # variantes. La malla se entrega sin material y el SDF del mundo
    # proporciona los mapas PBR por mesa.
    with open(path, "w") as f:
        f.write("# generado por solar_farm_gz.pv_mesh (materiales desde el SDF)\n")
        for x, y, z in verts:
            f.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
        for u, v in uvs:
            f.write(f"vt {u:.6f} {v:.6f}\n")
        f.write("vn 0.0000 0.0000 1.0000\n")
        for a, b, c, d in faces:
            f.write(f"f {a}/{a}/1 {b}/{b}/1 {c}/{c}/1 {d}/{d}/1\n")
    return len(faces)


# Normales hacia afuera para las seis caras de la caja, en el orden de devanado usado abajo.
_BOX_NORMALS = ((0, 0, -1), (0, 0, 1), (0, -1, 0),
                (1, 0, 0), (0, 1, 0), (-1, 0, 0))


def _box(verts, faces, cx, cy, cz, sx, sy, sz):
    """Añade una caja alineada a los ejes. Las caras llevan un índice de
    normal explícito: una malla sin normales recibe un material por defecto
    del cargador, que sobrescribe el <material> del SDF y renderiza en
    blanco plano."""
    b = len(verts) + 1
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    verts += [(cx - hx, cy - hy, cz - hz), (cx + hx, cy - hy, cz - hz),
              (cx + hx, cy + hy, cz - hz), (cx - hx, cy + hy, cz - hz),
              (cx - hx, cy - hy, cz + hz), (cx + hx, cy - hy, cz + hz),
              (cx + hx, cy + hy, cz + hz), (cx - hx, cy + hy, cz + hz)]
    for ni, q in enumerate(((0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
                            (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0))):
        faces.append((tuple(b + i for i in q), ni + 1))


def write_rack_obj(path, n_modules, n_posts=5):
    """Vigas de torsión y postes de cimentación, en el sistema de referencia sin inclinar de la mesa."""
    verts, faces = [], []
    span = table_span(n_modules)

    # dos vigas que recorren la longitud de la mesa, justo bajo el vidrio
    for xl in (-0.70, 0.70):
        bx, bz = _tilt(xl, -0.07)
        _box(verts, faces, bx, 0.0, bz, 0.10, span + 0.15, 0.10)

    # postes: clavados verticalmente en el suelo, así que no están inclinados
    for i in range(n_posts):
        py = (i - (n_posts - 1) / 2.0) * (span / max(1, n_posts - 1)) * 0.94
        for xl in (-0.70, 0.70):
            px, top = _tilt(xl, -0.12)
            _box(verts, faces, px, py, top / 2.0, 0.12, 0.12, top)

    with open(path, "w") as f:
        f.write("# generado por solar_farm_gz.pv_mesh (materiales desde el SDF)\n")
        for x, y, z in verts:
            f.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
        # una única UV ficticia mantiene uniforme la sintaxis de las caras;
        # el material del rack no lleva textura, así que la coordenada en
        # sí es irrelevante
        f.write("vt 0.0 0.0\n")
        for nx, ny, nz in _BOX_NORMALS:
            f.write(f"vn {nx} {ny} {nz}\n")
        for q, ni in faces:
            f.write("f " + " ".join(f"{i}/1/{ni}" for i in q) + "\n")
    return len(faces)


def collision_box(n_modules):
    """Proxy de colisión alineado a los ejes para el vidrio inclinado: barato,
    y lo bastante exacto como para que un dron no pueda atravesar una mesa."""
    return MODULE_L, table_span(n_modules), 0.04


def glass_pose():
    """Pose SDF para el visual del vidrio dentro del modelo de la mesa."""
    return f"0 0 {PIVOT_Z} 0 {TILT_RAD:.6f} 0"
