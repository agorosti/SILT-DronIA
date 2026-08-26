"""Procedural PV module texture synthesis.

Every module cell is rendered as four co-registered channels:

    albedo     RGB uint8   visible-light appearance
    roughness  L   uint8   PBR roughness (glass smooth, soiling rough)
    thermal    L   uint8   surface-temperature proxy, unused in Phase 1
    normal     RGB uint8   tangent-space normal, shared by every cell

The thermal channel is what makes Phase 2 a material swap rather than an asset
rebuild: a defect that scatters light in `albedo` also writes its heat
signature into `thermal` at the same pixels, so a thermal camera added later
sees the same defects in the same places with no geometry or UV changes.

Defects are emitted with pixel-space bounding boxes so the world generator can
write a ground-truth manifest for detector training.

Module geometry follows a modern half-cut layout: 6 columns x 24 half-cells,
25 mm frame, portrait 1.05 m x 2.10 m. At the default 512 x 1024 cell
resolution that is a uniform 488 px/m in both axes.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

# --- module constants -------------------------------------------------------

MODULE_W_M, MODULE_H_M = 1.05, 2.10
CELL_PX_W, CELL_PX_H = 512, 1024
FRAME_PX = 12
N_COLS, N_ROWS = 6, 24          # half-cut cells
GAP_PX = 3

DEFECT_TYPES = ("dirt", "bird_dropping", "crack", "delamination")


@dataclass
class Defect:
    """One defect instance, in pixel coords relative to its module cell."""
    kind: str
    x0: int
    y0: int
    x1: int
    y1: int
    severity: float
    meta: dict = field(default_factory=dict)

    def bbox_uv(self):
        """Normalised bbox within the module cell, YOLO-style cx,cy,w,h."""
        cx = (self.x0 + self.x1) / 2.0 / CELL_PX_W
        cy = (self.y0 + self.y1) / 2.0 / CELL_PX_H
        return (cx, cy,
                (self.x1 - self.x0) / CELL_PX_W,
                (self.y1 - self.y0) / CELL_PX_H)


# --- noise ------------------------------------------------------------------

def _octave(shape, freq, rng):
    """One octave of smooth value noise at the given base frequency."""
    h, w = shape
    lo = rng.random((max(2, int(h * freq)), max(2, int(w * freq))))
    return ndimage.zoom(lo, (h / lo.shape[0], w / lo.shape[1]), order=3)


def _octave_tileable(shape, k, rng):
    """One octave that wraps seamlessly: upsample a periodic lattice by
    tiling it 3x3, resampling, and cropping the centre tile."""
    h, w = shape
    lo = rng.random((k, k))
    big = np.tile(lo, (3, 3))
    z = ndimage.zoom(big, (3 * h / big.shape[0], 3 * w / big.shape[1]), order=3)
    return z[h:2 * h, w:2 * w]


def fbm_tileable(shape, rng, octaves=5, k0=4, gain=0.5):
    """Seamless fbm, for anything applied with UV tiling. The plain fbm()
    below is fine on a module cell, which is never tiled, but leaves visible
    seams on the ground plane."""
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
    """Fractal brownian motion, normalised to 0..1."""
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


# --- clean module -----------------------------------------------------------

def clean_module(rng):
    """Render a pristine module: frame, backsheet, half-cut cells, busbars."""
    h, w = CELL_PX_H, CELL_PX_W
    alb = np.zeros((h, w, 3), np.float32)
    rough = np.full((h, w), 0.12, np.float32)     # glass is smooth

    alb[:] = (0.62, 0.63, 0.65)                   # anodised aluminium frame
    rough[:] = 0.35

    iy0, iy1 = FRAME_PX, h - FRAME_PX
    ix0, ix1 = FRAME_PX, w - FRAME_PX
    alb[iy0:iy1, ix0:ix1] = (0.90, 0.90, 0.89)    # white backsheet
    rough[iy0:iy1, ix0:ix1] = 0.55

    cw = (ix1 - ix0 - (N_COLS - 1) * GAP_PX) / N_COLS
    ch = (iy1 - iy0 - (N_ROWS - 1) * GAP_PX) / N_ROWS

    # subtle wafer-to-wafer colour variation, as on a real mono string
    for r in range(N_ROWS):
        for c in range(N_COLS):
            y0 = int(iy0 + r * (ch + GAP_PX))
            x0 = int(ix0 + c * (cw + GAP_PX))
            y1, x1 = int(y0 + ch), int(x0 + cw)
            tint = rng.normal(0.0, 0.006)
            alb[y0:y1, x0:x1] = (0.030 + tint, 0.042 + tint, 0.085 + tint)
            rough[y0:y1, x0:x1] = 0.10

            # three silver busbars per cell
            for bb in (0.25, 0.5, 0.75):
                bx = int(x0 + bb * cw)
                alb[y0:y1, bx:bx + 2] = (0.55, 0.56, 0.58)
                rough[y0:y1, bx:bx + 2] = 0.25

    # very fine texture so the glass is not a flat colour under specular light
    alb *= (0.97 + 0.06 * fbm((h, w), rng, octaves=3, base=0.25))[..., None]

    # ambient module temperature, warm but uniform
    thermal = np.full((h, w), 0.42, np.float32)
    thermal += 0.02 * fbm((h, w), rng, octaves=2, base=0.05)
    return alb, rough, thermal


# --- defect generators ------------------------------------------------------
# Each returns (mask, colour, d_rough, d_thermal, Defect). `mask` is 0..1
# coverage; the caller alpha-composites colour over the albedo.

def _blob(shape, cy, cx, ry, rx, rng, rough_edge=0.45):
    """Irregular radial blob centred at (cy, cx)."""
    h, w = shape
    yy, xx = np.ogrid[:h, :w]
    d = np.sqrt(((yy - cy) / max(ry, 1)) ** 2 + ((xx - cx) / max(rx, 1)) ** 2)
    d = d * (1.0 - rough_edge * (fbm(shape, rng, octaves=4, base=0.06) - 0.5))
    return np.clip(1.0 - d, 0.0, 1.0) ** 0.6


def dirt(shape, rng, severity):
    """Soiling. Accumulates toward the lower edge, as rain runoff does."""
    h, w = shape
    band = rng.uniform(0.25, 0.6)                 # fraction of height affected
    y0 = int(h * (1.0 - band))

    grad = np.zeros((h, w), np.float32)
    grad[y0:] = np.linspace(0.0, 1.0, h - y0)[:, None] ** 1.4
    tex = fbm((h, w), rng, octaves=5, base=0.03)
    mask = np.clip(grad * (0.45 + 1.1 * tex) * severity, 0, 1)
    mask[mask < 0.06] = 0.0

    colour = np.array(rng.choice([(0.42, 0.36, 0.26),      # dry dust
                                  (0.34, 0.30, 0.24),      # soil
                                  (0.48, 0.44, 0.35)]))    # sandy
    ys, xs = np.nonzero(mask > 0.08)
    if len(ys) == 0:
        return None
    d = Defect("dirt", int(xs.min()), int(ys.min()), int(xs.max()),
               int(ys.max()), severity, {"band": round(band, 3)})
    # soiling blocks light -> the cell beneath runs hot
    return mask * 0.85, colour, mask * 0.5, mask * 0.30, d


def bird_dropping(shape, rng, severity):
    """A splat with downward drip streaks. Small, opaque, high contrast."""
    h, w = shape
    r = rng.uniform(0.012, 0.045) * h * (0.6 + severity)
    cy = rng.uniform(0.1, 0.85) * h
    cx = rng.uniform(0.1, 0.9) * w

    mask = _blob((h, w), cy, cx, r, r * rng.uniform(0.7, 1.3), rng,
                 rough_edge=0.75)
    # drips
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
    """Branching glass fracture radiating from an impact point."""
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
    # fractured glass scatters light -> reads bright, not dark
    colour = np.array((0.78, 0.79, 0.80))
    d = Defect("crack", int(xs.min()), int(ys.min()), int(xs.max()),
               int(ys.max()), severity)
    return mask * 0.85, colour, mask * 0.4, mask * 0.65, d


def delamination(shape, rng, severity):
    """Milky EVA discolouration, biased toward the module perimeter."""
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
    colour = np.array((0.72, 0.66, 0.45)) * rng.uniform(0.9, 1.15)  # yellowed
    d = Defect("delamination", int(xs.min()), int(ys.min()), int(xs.max()),
               int(ys.max()), severity, {"edge": bool(edge)})
    return mask, colour, mask * 0.35, mask * 0.70, d


_GENERATORS = {
    "dirt": dirt,
    "bird_dropping": bird_dropping,
    "crack": crack,
    "delamination": delamination,
}


# --- composition ------------------------------------------------------------

def render_module(rng, defect_plan):
    """Render one module.

    `defect_plan` is a list of (kind, severity). Returns the three channels
    plus the Defect annotations that actually landed.
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

    # defects never cover the frame
    alb[:FRAME_PX], alb[-FRAME_PX:] = (0.62, 0.63, 0.65), (0.62, 0.63, 0.65)
    alb[:, :FRAME_PX], alb[:, -FRAME_PX:] = (0.62, 0.63, 0.65), (0.62, 0.63, 0.65)
    return alb, rough, therm, found


def module_normal_map(rng):
    """Tangent-space normal map. Identical for every module, so it is built
    once and shared: the cell grid relief does not vary between modules."""
    h, w = CELL_PX_H, CELL_PX_W
    height = np.zeros((h, w), np.float32)
    iy0, iy1, ix0, ix1 = FRAME_PX, h - FRAME_PX, FRAME_PX, w - FRAME_PX
    height[iy0:iy1, ix0:ix1] = 1.0                # cells sit below the frame
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
