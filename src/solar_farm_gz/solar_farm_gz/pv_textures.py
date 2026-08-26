"""Síntesis procedural de texturas de módulo fotovoltaico.

Cada celda de módulo se renderiza como cuatro canales co-registrados:

    albedo     RGB uint8   apariencia en luz visible
    roughness  L   uint8   rugosidad PBR (vidrio liso, suciedad rugosa)
    thermal    L   uint8   proxy de temperatura de superficie, sin usar en la Fase 1
    normal     RGB uint8   normal en espacio tangente, compartida por cada celda

El canal térmico es lo que hace que la Fase 2 sea un simple cambio de
material en lugar de una reconstrucción de recursos: un defecto que dispersa
luz en `albedo` también escribe su firma de calor en `thermal` en los mismos
píxeles, así que una cámara térmica añadida más adelante ve los mismos
defectos en los mismos lugares sin cambios de geometría ni de UV.

Los defectos se emiten con cajas delimitadoras en coordenadas de píxel para
que el generador del mundo pueda escribir un manifiesto de referencia
(ground truth) para el entrenamiento del detector.

La geometría del módulo sigue una disposición moderna de celdas cortadas por
la mitad (half-cut): 6 columnas x 24 semiceldas, marco de 25 mm, vertical
1.05 m x 2.10 m. A la resolución de celda por defecto de 512 x 1024 eso es
una resolución uniforme de 488 px/m en ambos ejes.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

# --- constantes del módulo ---------------------------------------------------

MODULE_W_M, MODULE_H_M = 1.05, 2.10
CELL_PX_W, CELL_PX_H = 512, 1024
FRAME_PX = 12
N_COLS, N_ROWS = 6, 24          # semiceldas (half-cut)
GAP_PX = 3

DEFECT_TYPES = ("dirt", "bird_dropping", "crack", "delamination")


@dataclass
class Defect:
    """Una instancia de defecto, en coordenadas de píxel relativas a su celda de módulo."""
    kind: str
    x0: int
    y0: int
    x1: int
    y1: int
    severity: float
    meta: dict = field(default_factory=dict)

    def bbox_uv(self):
        """Caja delimitadora normalizada dentro de la celda de módulo, formato YOLO cx,cy,w,h."""
        cx = (self.x0 + self.x1) / 2.0 / CELL_PX_W
        cy = (self.y0 + self.y1) / 2.0 / CELL_PX_H
        return (cx, cy,
                (self.x1 - self.x0) / CELL_PX_W,
                (self.y1 - self.y0) / CELL_PX_H)


# --- ruido --------------------------------------------------------------------

def _octave(shape, freq, rng):
    """Una octava de ruido de valor suave a la frecuencia base dada."""
    h, w = shape
    lo = rng.random((max(2, int(h * freq)), max(2, int(w * freq))))
    return ndimage.zoom(lo, (h / lo.shape[0], w / lo.shape[1]), order=3)


def _octave_tileable(shape, k, rng):
    """Una octava que enlaza sin costuras: sobremuestrea una retícula
    periódica repitiéndola en un mosaico 3x3, remuestreando, y recortando
    el mosaico central."""
    h, w = shape
    lo = rng.random((k, k))
    big = np.tile(lo, (3, 3))
    z = ndimage.zoom(big, (3 * h / big.shape[0], 3 * w / big.shape[1]), order=3)
    return z[h:2 * h, w:2 * w]


def fbm_tileable(shape, rng, octaves=5, k0=4, gain=0.5):
    """fbm sin costuras, para cualquier cosa aplicada con mosaico UV. El
    fbm() normal de abajo funciona bien en una celda de módulo, que nunca
    se repite en mosaico, pero deja costuras visibles en el plano del
    suelo."""
    out = np.zeros(shape)
    amp, k, norm = 1.0, k0, 0.0
    for _ in range(octaves):
        out += amp * _octave_tileable(shape, k, rng)
        norm += amp
        amp *= gain
        k *= 2
    out /= norm
    out -= out.min()
    return out / max(out.max(), 1e-6)


def fbm(shape, rng, octaves=4, base=0.02, gain=0.5):
    """Movimiento browniano fractal, normalizado a 0..1."""
    out = np.zeros(shape)
    amp, freq, norm = 1.0, base, 0.0
    for _ in range(octaves):
        out += amp * _octave(shape, freq, rng)
        norm += amp
        amp *= gain
        freq *= 2.0
    out /= norm
    out -= out.min()
    return out / max(out.max(), 1e-6)


# --- módulo limpio -----------------------------------------------------------

def clean_module(rng):
    """Renderiza un módulo prístino: marco, backsheet, celdas half-cut, busbars."""
    h, w = CELL_PX_H, CELL_PX_W
    alb = np.zeros((h, w, 3), np.float32)
    rough = np.full((h, w), 0.12, np.float32)     # el vidrio es liso

    alb[:] = (0.62, 0.63, 0.65)                   # marco de aluminio anodizado
    rough[:] = 0.35

    iy0, iy1 = FRAME_PX, h - FRAME_PX
    ix0, ix1 = FRAME_PX, w - FRAME_PX
    alb[iy0:iy1, ix0:ix1] = (0.90, 0.90, 0.89)    # backsheet blanco
    rough[iy0:iy1, ix0:ix1] = 0.55

    cw = (ix1 - ix0 - (N_COLS - 1) * GAP_PX) / N_COLS
    ch = (iy1 - iy0 - (N_ROWS - 1) * GAP_PX) / N_ROWS

    # sutil variación de color entre obleas, como en un string mono real
    for r in range(N_ROWS):
        for c in range(N_COLS):
            y0 = int(iy0 + r * (ch + GAP_PX))
            x0 = int(ix0 + c * (cw + GAP_PX))
            y1, x1 = int(y0 + ch), int(x0 + cw)
            tint = rng.normal(0.0, 0.006)
            alb[y0:y1, x0:x1] = (0.030 + tint, 0.042 + tint, 0.085 + tint)
            rough[y0:y1, x0:x1] = 0.10

            # tres busbars plateados por celda
            for bb in (0.25, 0.5, 0.75):
                bx = int(x0 + bb * cw)
                alb[y0:y1, bx:bx + 2] = (0.55, 0.56, 0.58)
                rough[y0:y1, bx:bx + 2] = 0.25

    # textura muy fina para que el vidrio no sea un color plano bajo luz especular
    alb *= (0.97 + 0.06 * fbm((h, w), rng, octaves=3, base=0.25))[..., None]

    # temperatura ambiente del módulo, cálida pero uniforme
    thermal = np.full((h, w), 0.42, np.float32)
    thermal += 0.02 * fbm((h, w), rng, octaves=2, base=0.05)
    return alb, rough, thermal


# --- generadores de defectos --------------------------------------------------
# Cada uno devuelve (mask, colour, d_rough, d_thermal, Defect). `mask` es la
# cobertura en 0..1; quien llama compone el color con alpha sobre el albedo.

def _blob(shape, cy, cx, ry, rx, rng, rough_edge=0.45):
    """Mancha radial irregular centrada en (cy, cx)."""
    h, w = shape
    yy, xx = np.ogrid[:h, :w]
    d = np.sqrt(((yy - cy) / max(ry, 1)) ** 2 + ((xx - cx) / max(rx, 1)) ** 2)
    d = d * (1.0 - rough_edge * (fbm(shape, rng, octaves=4, base=0.06) - 0.5))
    return np.clip(1.0 - d, 0.0, 1.0) ** 0.6


def dirt(shape, rng, severity):
    """Suciedad. Se acumula hacia el borde inferior, siguiendo la escorrentía de la lluvia."""
    h, w = shape
    band = rng.uniform(0.25, 0.6)                 # fracción de la altura afectada
    y0 = int(h * (1.0 - band))

    grad = np.zeros((h, w), np.float32)
    grad[y0:] = np.linspace(0.0, 1.0, h - y0)[:, None] ** 1.4
    tex = fbm((h, w), rng, octaves=5, base=0.03)
    mask = np.clip(grad * (0.45 + 1.1 * tex) * severity, 0, 1)
    mask[mask < 0.06] = 0.0

    colour = np.array(rng.choice([(0.42, 0.36, 0.26),      # polvo seco
                                  (0.34, 0.30, 0.24),      # tierra
                                  (0.48, 0.44, 0.35)]))    # arenoso
    ys, xs = np.nonzero(mask > 0.08)
    if len(ys) == 0:
        return None
    d = Defect("dirt", int(xs.min()), int(ys.min()), int(xs.max()),
               int(ys.max()), severity, {"band": round(band, 3)})
    # la suciedad bloquea la luz -> la celda de debajo se calienta
    return mask * 0.85, colour, mask * 0.5, mask * 0.30, d


def bird_dropping(shape, rng, severity):
    """Una mancha con regueros de goteo hacia abajo. Pequeña, opaca, de alto contraste."""
    h, w = shape
    r = rng.uniform(0.012, 0.045) * h * (0.6 + severity)
    cy = rng.uniform(0.1, 0.85) * h
    cx = rng.uniform(0.1, 0.9) * w

    mask = _blob((h, w), cy, cx, r, r * rng.uniform(0.7, 1.3), rng,
                 rough_edge=0.75)
    # goteos
    for _ in range(rng.integers(1, 4)):
        dx = cx + rng.normal(0, r * 0.5)
        dlen = r * rng.uniform(1.2, 3.5)
        dw = max(1.0, r * rng.uniform(0.10, 0.22))
        mask = np.maximum(mask, _blob((h, w), cy + dlen * 0.5, dx,
                                      dlen * 0.5, dw, rng, rough_edge=0.5))
    mask = np.clip(mask * 1.6, 0, 1)

    colour = np.array((0.88, 0.87, 0.80)) * rng.uniform(0.85, 1.0)
    ys, xs = np.nonzero(mask > 0.1)
    if len(ys) == 0:
        return None
    d = Defect("bird_dropping", int(xs.min()), int(ys.min()), int(xs.max()),
               int(ys.max()), severity)
    return mask, colour, mask * 0.7, mask * 0.55, d


def crack(shape, rng, severity):
    """Fractura de vidrio ramificada que irradia desde un punto de impacto."""
    h, w = shape
    mask = np.zeros((h, w), np.float32)
    cy = rng.uniform(0.15, 0.85) * h
    cx = rng.uniform(0.15, 0.85) * w
    reach = rng.uniform(0.10, 0.30) * h * (0.5 + severity)

    def walk(y, x, ang, length, depth):
        steps = max(3, int(length / 3))
        for _ in range(steps):
            ang += rng.normal(0, 0.22)
            y += 3 * np.sin(ang)
            x += 3 * np.cos(ang)
            if not (0 <= y < h and 0 <= x < w):
                return
            yi, xi = int(y), int(x)
            mask[max(0, yi - 1):yi + 2, max(0, xi - 1):xi + 2] = 1.0
            if depth < 3 and rng.random() < 0.07:
                walk(y, x, ang + rng.choice([-1, 1]) * rng.uniform(0.4, 1.1),
                     length * 0.55, depth + 1)

    for _ in range(rng.integers(3, 7)):
        walk(cy, cx, rng.uniform(0, 2 * np.pi), reach, 0)

    mask = ndimage.gaussian_filter(mask, 0.8)
    mask = np.clip(mask * 2.2, 0, 1)
    ys, xs = np.nonzero(mask > 0.12)
    if len(ys) == 0:
        return None
    # el vidrio fracturado dispersa la luz -> se lee brillante, no oscuro
    colour = np.array((0.78, 0.79, 0.80))
    d = Defect("crack", int(xs.min()), int(ys.min()), int(xs.max()),
               int(ys.max()), severity)
    return mask * 0.85, colour, mask * 0.4, mask * 0.65, d


def delamination(shape, rng, severity):
    """Decoloración lechosa del EVA, con sesgo hacia el perímetro del módulo."""
    h, w = shape
    edge = rng.random() < 0.7
    if edge:
        side = rng.integers(0, 4)
        cy = rng.uniform(0.05, 0.2) * h if side == 0 else \
             rng.uniform(0.8, 0.95) * h if side == 1 else rng.uniform(0.1, 0.9) * h
        cx = rng.uniform(0.05, 0.2) * w if side == 2 else \
             rng.uniform(0.8, 0.95) * w if side == 3 else rng.uniform(0.1, 0.9) * w
    else:
        cy, cx = rng.uniform(0.2, 0.8) * h, rng.uniform(0.2, 0.8) * w

    ry = rng.uniform(0.04, 0.14) * h * (0.6 + severity)
    rx = rng.uniform(0.08, 0.28) * w * (0.6 + severity)
    mask = _blob((h, w), cy, cx, ry, rx, rng, rough_edge=0.6)
    mask = np.clip(mask * 1.3, 0, 1) * 0.75

    ys, xs = np.nonzero(mask > 0.1)
    if len(ys) == 0:
        return None
    colour = np.array((0.72, 0.66, 0.45)) * rng.uniform(0.9, 1.15)  # amarillento
    d = Defect("delamination", int(xs.min()), int(ys.min()), int(xs.max()),
               int(ys.max()), severity, {"edge": bool(edge)})
    return mask, colour, mask * 0.35, mask * 0.70, d


_GENERATORS = {
    "dirt": dirt,
    "bird_dropping": bird_dropping,
    "crack": crack,
    "delamination": delamination,
}


# --- composición ---------------------------------------------------------------

def render_module(rng, defect_plan):
    """Renderiza un módulo.

    `defect_plan` es una lista de (kind, severity). Devuelve los tres
    canales más las anotaciones Defect que realmente se aplicaron.
    """
    alb, rough, therm = clean_module(rng)
    found = []
    for kind, sev in defect_plan:
        res = _GENERATORS[kind]((CELL_PX_H, CELL_PX_W), rng, sev)
        if res is None:
            continue
        mask, colour, d_rough, d_therm, ann = res
        m = mask[..., None]
        alb = alb * (1.0 - m) + colour[None, None, :] * m
        rough = np.clip(rough + d_rough, 0.02, 1.0)
        therm = np.clip(therm + d_therm * 0.45, 0.0, 1.0)
        found.append(ann)

    # los defectos nunca cubren el marco
    alb[:FRAME_PX], alb[-FRAME_PX:] = (0.62, 0.63, 0.65), (0.62, 0.63, 0.65)
    alb[:, :FRAME_PX], alb[:, -FRAME_PX:] = (0.62, 0.63, 0.65), (0.62, 0.63, 0.65)
    return alb, rough, therm, found


def module_normal_map(rng):
    """Mapa de normales en espacio tangente. Idéntico para cada módulo, así
    que se construye una sola vez y se comparte: el relieve de la
    cuadrícula de celdas no varía entre módulos."""
    h, w = CELL_PX_H, CELL_PX_W
    height = np.zeros((h, w), np.float32)
    iy0, iy1, ix0, ix1 = FRAME_PX, h - FRAME_PX, FRAME_PX, w - FRAME_PX
    height[iy0:iy1, ix0:ix1] = 1.0                # las celdas quedan por debajo del marco
    cw = (ix1 - ix0 - (N_COLS - 1) * GAP_PX) / N_COLS
    ch = (iy1 - iy0 - (N_ROWS - 1) * GAP_PX) / N_ROWS
    for r in range(N_ROWS):
        y0 = int(iy0 + r * (ch + GAP_PX))
        height[y0:y0 + 1, ix0:ix1] = 0.6
    for c in range(N_COLS):
        x0 = int(ix0 + c * (cw + GAP_PX))
        height[iy0:iy1, x0:x0 + 1] = 0.6
    height = ndimage.gaussian_filter(height, 1.2)

    gy, gx = np.gradient(height * 4.0)
    n = np.dstack([-gx, -gy, np.ones_like(height)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return ((n * 0.5 + 0.5) * 255).astype(np.uint8)


def to_u8(a):
    return np.clip(a * 255.0, 0, 255).astype(np.uint8)
