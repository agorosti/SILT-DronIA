"""3D -> 2D projection of PV module defects for YOLO ground truth.

Reconstructs, independently of the renderer, the exact same geometry
solar_farm_gz.pv_mesh uses to build the glass mesh and table placement, then
projects a defect's module-local UV bbox through a pinhole camera model that
matches Gazebo's <camera horizontal_fov=...> sensor convention.

Coordinate conventions (matched empirically against pv_mesh.py / capture.py):

  Table-local (pre-tilt): x in [-MODULE_L/2, MODULE_L/2] is up-slope(-)/
  down-slope(+); y is across the row.  Module m's cell in the merged glass
  mesh runs y in [y0, y1] for that module.

  Module-local UV bbox (cx, cy, w, h), all in [0, 1], as stored in
  defects.json: cx=0 is the module's y0 edge, cx=1 is y1; cy=0 is the
  up-slope (high) edge, cy=1 is the down-slope (low) edge -- this matches the
  documented soiling bias "toward the lower edge" landing at cy near 1.

  World: table pose (ox, oy, yaw) placed on the ground, tables static, +Z up.

  Camera: SDF pose (x, y, z, roll, pitch, yaw), aerospace convention
  R = Rz(yaw) Ry(pitch) Rx(roll) applied to the local forward axis +X.
  pitch = pi/2 is straight down (matches the README's nadir examples).
  Converted to an optical frame (X right, Y down, Z forward) for the pinhole
  projection; horizontal_fov sets fx, and fy = fx (square pixels), matching
  how gz-sim/ogre2 derives vertical FOV from a horizontal_fov-only camera.
"""

import math

import numpy as np

MODULE_W = 1.05
MODULE_L = 2.10
MODULE_GAP = 0.02
TILT_RAD = math.radians(28.0)
PIVOT_Z = 1.60


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def camera_world_from_local(roll, pitch, yaw):
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


# optical frame (X right, Y down, Z forward) from the robot/local frame
# (X forward, Y left, Z up) -- the standard Gazebo/ROS camera-optical swap.
_OPTICAL_FROM_LOCAL = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)


class Camera:
    def __init__(self, pose_xyzrpy, width, height, hfov):
        x, y, z, roll, pitch, yaw = pose_xyzrpy
        self.pos = np.array([x, y, z], float)
        self.R_wl = camera_world_from_local(roll, pitch, yaw)   # world<-local
        self.R_lw = self.R_wl.T                                  # local<-world
        self.width, self.height = width, height
        self.fx = (width / 2.0) / math.tan(hfov / 2.0)
        self.fy = self.fx
        self.cx_px, self.cy_px = width / 2.0, height / 2.0

    def project(self, p_world):
        """Returns (u, v, in_front). u,v in pixel space (not clipped)."""
        p_local = self.R_lw @ (np.asarray(p_world, float) - self.pos)
        p_opt = _OPTICAL_FROM_LOCAL @ p_local
        z = p_opt[2]
        if z <= 1e-3:
            return None, None, False
        u = self.cx_px + self.fx * p_opt[0] / z
        v = self.cy_px + self.fy * p_opt[1] / z
        return u, v, True

    def project_many(self, pts_world):
        out = []
        for p in pts_world:
            u, v, ok = self.project(p)
            out.append((u, v, ok))
        return out


def module_y_range(module_index, n_modules_per_table):
    pitch = MODULE_W + MODULE_GAP
    yc = (module_index - (n_modules_per_table - 1) / 2.0) * pitch
    return yc - MODULE_W / 2.0, yc + MODULE_W / 2.0


def module_uv_to_table_local(cx, cy, y0, y1):
    """Module-local UV (0..1) -> table-local (pre-tilt) 3D point, z=0."""
    half_l = MODULE_L / 2.0
    x_local = half_l * (2.0 * cy - 1.0)     # cy=0 -> -half_l (up-slope)
    y_local = y0 + cx * (y1 - y0)           # cx=0 -> y0
    return x_local, y_local


def table_local_to_world(x_local, y_local, ox, oy, yaw):
    """Apply glass tilt (about table-local Y) then the table's world pose."""
    ct, st = math.cos(TILT_RAD), math.sin(TILT_RAD)
    x_t = x_local * ct
    z_t = -x_local * st + PIVOT_Z
    y_t = y_local

    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    x_w = ox + x_t * cy_ - y_t * sy_
    y_w = oy + x_t * sy_ + y_t * cy_
    z_w = z_t
    return x_w, y_w, z_w


def table_normal_world(yaw):
    """Outward glass normal in world coords (panels face +X, tilted up)."""
    nx_l, nz_l = math.sin(TILT_RAD), math.cos(TILT_RAD)
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    return np.array([nx_l * cy_, nx_l * sy_, nz_l])


def defect_world_corners(cx, cy, w, h, module_index, n_mod_per_table, ox, oy, yaw):
    y0, y1 = module_y_range(module_index, n_mod_per_table)
    corners = []
    for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        u = min(max(cx + du * w / 2.0, 0.0), 1.0)
        v = min(max(cy + dv * h / 2.0, 0.0), 1.0)
        xl, yl = module_uv_to_table_local(u, v, y0, y1)
        corners.append(table_local_to_world(xl, yl, ox, oy, yaw))
    return corners


def project_defect_bbox(cam, corners_world, table_yaw, eps_facing=0.05):
    """Project a defect's 3D corners to a clipped, normalised image-space
    YOLO bbox. Returns None if not usable (behind camera, off-frame, facing
    away, or too small)."""
    normal = table_normal_world(table_yaw)
    to_cam = cam.pos - np.mean(corners_world, axis=0)
    if np.dot(normal, to_cam) < eps_facing * np.linalg.norm(to_cam):
        return None   # panel faces (mostly) away from the camera

    pts = cam.project_many(corners_world)
    if not all(ok for _, _, ok in pts):
        return None
    us = [u for u, v, ok in pts]
    vs = [v for u, v, ok in pts]
    u0, u1 = min(us), max(us)
    v0, v1 = min(vs), max(vs)

    full_area = max(u1 - u0, 1e-6) * max(v1 - v0, 1e-6)
    cu0, cu1 = max(u0, 0.0), min(u1, cam.width)
    cv0, cv1 = max(v0, 0.0), min(v1, cam.height)
    if cu1 <= cu0 or cv1 <= cv0:
        return None
    clipped_area = (cu1 - cu0) * (cv1 - cv0)
    if clipped_area / full_area < 0.5:
        return None   # mostly cropped out at the frame edge

    bw, bh = cu1 - cu0, cv1 - cv0
    if bw < 3 or bh < 3:
        return None   # sub-pixel-scale, not a useful training example

    bcx, bcy = (cu0 + cu1) / 2.0 / cam.width, (cv0 + cv1) / 2.0 / cam.height
    return bcx, bcy, bw / cam.width, bh / cam.height
