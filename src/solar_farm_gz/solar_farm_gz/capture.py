#!/usr/bin/env python3
"""Headless capture of still images and flight videos from a generated world.

Gazebo's GUI costs about as much to render as the scene itself, so images
are produced by injecting a camera into the world, running the server
without a GUI, and pulling frames straight off the gz-transport image
topic -- worth doing regardless of how much GPU headroom the machine has,
and what makes headless, scripted capture possible in the first place.

Two modes:

    still     one launch, one frame, fixed pose
    fly       one launch, camera repositioned per frame via the set_pose
              service, frames encoded to mp4

The flight video deliberately does *not* relaunch the world per frame. A
1000-module world takes ~18 s to load, so a 120-frame sequence built that
way would cost over half an hour; moving the camera inside an already-
running world brings it down to under a minute.

    # single still image
    python3 -m solar_farm_gz.capture --world worlds/solar_farm.sdf \\
        --pose "42 8 15 0 0.36 3.0" -o array_front.png

    # flight video along an inspection transect
    python3 -m solar_farm_gz.capture --world worlds/solar_farm.sdf --fly \\
        --path "30,-10,12,0,0.35,1.5708; 30,110,12,0,0.35,1.5708" \\
        --frames 150 --fps 30 -o flythrough.mp4

    # same, but with the thermal channel (false colour) instead of RGB
    python3 -m solar_farm_gz.capture --world worlds/solar_farm.sdf --fly \\
        --thermal \\
        --path "30,-10,12,0,0.35,1.5708; 30,110,12,0,0.35,1.5708" \\
        --frames 150 --fps 30 -o flythrough_thermal.mp4

`--fly` eases in/out of motion, rounds the corner at each interior
waypoint, and banks into turns by default (see `lerp_path` / --no-ease /
--corner-radius / --bank-deg to tune or disable), so a multi-waypoint path
reads as piloted rather than a piecewise-linear survey pass.
"""

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np
from PIL import Image as PILImage

CAM_TOPIC = "/solar_farm/probe/image"
PROBE_NAME = "capture_probe"

_CAM_SDF = """
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <sensor name="probe" type="camera">
          <topic>{topic}</topic>
          <update_rate>{rate}</update_rate>
          <always_on>1</always_on>
          <camera>
            <horizontal_fov>{fov}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height>
              <format>R8G8B8</format></image>
            <clip><near>0.1</near><far>2000</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""


# --- helpers -----------------------------------------------------------------

def parse_pose(s):
    v = [float(x) for x in re.split(r"[,\s]+", s.strip()) if x]
    if len(v) != 6:
        raise SystemExit(f"pose needs 6 numbers (x y z roll pitch yaw), got {len(v)}")
    return v


def _smoothstep(u):
    """Cubic ease-in/ease-out remap of a [0, 1] parameter (zero derivative
    at both ends). Applied to the arc-length parameter so a flythrough
    accelerates into motion and decelerates into the final waypoint,
    instead of cutting in and out at a constant cruise speed."""
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def _smooth_columns(arr, sigma):
    """Gaussian-weighted moving average along axis 0, faded back to the
    raw values over the first/last ~3*sigma samples so the sequence still
    starts and ends exactly where it did before smoothing.

    This is what turns the sharp corners a piecewise-linear path makes at
    each interior waypoint into a continuous curve, without moving the
    two endpoints the caller actually asked for.
    """
    n = len(arr)
    if sigma <= 0 or n < 5:
        return arr.copy()
    span = int(min(round(sigma * 3), n // 2))
    if span < 1:
        return arr.copy()
    k = np.arange(-span, span + 1)
    kernel = np.exp(-0.5 * (k / sigma) ** 2)
    kernel /= kernel.sum()
    smoothed = np.empty_like(arr)
    for c in range(arr.shape[1]):
        padded = np.pad(arr[:, c], span, mode="edge")
        smoothed[:, c] = np.convolve(padded, kernel, mode="valid")
    fade_n = min(span, n // 2)
    if fade_n < 1:
        return smoothed
    ramp = np.linspace(0.0, 1.0, fade_n)
    weight = np.ones(n)
    weight[:fade_n] = ramp
    weight[n - fade_n:] = ramp[::-1]
    return arr * (1.0 - weight)[:, None] + smoothed * weight[:, None]


def _wrap_pi(a):
    """Wraps an angle (radians) to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def lerp_path(waypoints, n, ease=True, corner_radius=6.0, bank_deg=15.0,
              fps=30.0):
    """Piecewise-linear interpolation between waypoints, paced by arc
    length -- with three optional cinematic refinements layered on top so
    the recorded flythrough reads as piloted rather than surveyed:

    * `ease` re-times the arc-length parameter with a smoothstep, so the
      camera ramps up to cruise speed and ramps back down instead of
      starting/stopping instantaneously. Default on.
    * `corner_radius` (Gaussian sigma, in frames; 0 disables) rounds the
      hard corner a piecewise-linear path makes at each interior waypoint
      into a smooth curve. Only position (x, y, z) is smoothed -- pitch,
      roll and yaw stay exactly what the waypoints ask for, before any
      banking below is added. A 2-waypoint path has no interior corner to
      round, so this is a no-op for the common straight-line case.
    * `bank_deg` (max degrees of added roll; 0 disables) banks the camera
      into a turn, proportional to how fast the path's yaw is changing
      per second (using `fps` to convert per-frame yaw change into a
      rate), clipped to +/-bank_deg. This is a cosmetic approximation
      tuned by eye, not a modelled coordinated turn -- there is no
      aircraft here to derive the physical relationship from, so a path
      with sharp direction changes may want a lower --bank-deg than the
      default.

    All three default to on but degrade to the original plain
    piecewise-linear behaviour when disabled (`ease=False,
    corner_radius=0, bank_deg=0`).
    """
    wp = np.array(waypoints, float)
    if len(wp) == 1:
        return np.repeat(wp, n, axis=0)
    seg = np.maximum(np.linalg.norm(np.diff(wp[:, :3], axis=0), axis=1), 1e-6)
    t = np.concatenate([[0.0], np.cumsum(seg)])
    t /= t[-1]
    q = np.linspace(0.0, 1.0, n)
    if ease:
        q = _smoothstep(q)
    out = np.stack([np.interp(q, t, wp[:, i]) for i in range(6)], axis=1)

    if corner_radius > 0 and len(wp) > 2:
        out[:, :3] = _smooth_columns(out[:, :3], corner_radius)

    if bank_deg > 0 and n > 4:
        yaw = out[:, 5]
        dyaw = np.zeros(n)
        dyaw[1:-1] = [_wrap_pi(yaw[i + 1] - yaw[i - 1]) / 2.0
                      for i in range(1, n - 1)]
        dyaw[0], dyaw[-1] = dyaw[1], dyaw[-2]
        # Smooth the yaw-rate signal itself even if corner_radius is 0:
        # otherwise the hard corner in the (still piecewise-linear-in-t)
        # yaw column produces a single-frame bank spike right at each
        # waypoint instead of a gradual lean into the turn.
        smooth_sigma = corner_radius if corner_radius > 0 else max(2.0, n * 0.02)
        dyaw = _smooth_columns(dyaw[:, None], smooth_sigma)[:, 0]
        turn_rate = dyaw * fps   # rad/s
        max_bank = math.radians(bank_deg)
        # Saturates at max_bank around a 0.5 rad/s (~29 deg/s) turn rate --
        # a fairly brisk turn. Not physically derived; adjust --bank-deg
        # (or this constant) after previewing footage.
        bank = np.clip(turn_rate * (max_bank / 0.5), -max_bank, max_bank)
        out[:, 3] = out[:, 3] + bank

    return out


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def world_name(sdf):
    m = re.search(r'<world name="([^"]+)"', sdf)
    if not m:
        raise SystemExit("could not find <world name=...> in the SDF")
    return m.group(1)


def build_env(world_path):
    env = dict(os.environ)
    env.setdefault("GZ_CONFIG_PATH", "/usr/share/gz")
    root = os.path.dirname(os.path.abspath(world_path))
    prev = env.get("GZ_SIM_RESOURCE_PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = f"{root}:{prev}" if prev else root
    return env


def inject_camera(sdf, pose, w, h, fov, rate):
    cam = _CAM_SDF.format(name=PROBE_NAME, pose=pose, topic=CAM_TOPIC,
                          w=w, h=h, fov=fov, rate=rate)
    if "</world>" not in sdf:
        raise SystemExit("no </world> tag in the SDF")
    return sdf.replace("</world>", cam + "\n  </world>", 1)


def _thermal_colour(img):
    """Raw probe frame over an already thermal-swapped world -> calibrated
    false colour. Same treatment as flight_video.py's thermal path (same
    THERMAL_LOW/HIGH, same INFERNO map), so the stills and videos generated
    here look like they came from the same camera family as the flight
    videos."""
    import cv2
    from . import flight_video as fv  # deferred import: flight_video imports capture
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).astype(np.float32)
    span = fv.THERMAL_HIGH - fv.THERMAL_LOW
    gray = np.clip((gray - fv.THERMAL_LOW) * (255.0 / span), 0, 255)
    gray = gray.astype(np.uint8)
    return cv2.cvtColor(cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO),
                        cv2.COLOR_BGR2RGB)


class FrameSink:
    """The latest camera frame plus a monotonically increasing frame counter.

    The counter is what makes repositioning safe: after moving the camera
    we wait N more frames before sampling, so the saved image can't be one
    rendered before the move finished.
    """

    def __init__(self):
        self.img = None
        self.count = 0

    def __call__(self, msg):
        self.img = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.width, 3).copy()
        self.count += 1


def start_server(world_file, env, verbose=False):
    return subprocess.Popen(
        ["/usr/bin/gz", "sim", "-s", "-r", "-v", "1" if verbose else "0",
         world_file],
        env=env,
        stdout=None if verbose else subprocess.DEVNULL,
        stderr=None if verbose else subprocess.DEVNULL)


def stop_server(proc):
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()


def wait_for_frames(sink, proc, target, timeout):
    t0 = time.time()
    while sink.count < target and time.time() - t0 < timeout:
        if proc.poll() is not None:
            return False
        time.sleep(0.05)
    return sink.count >= target


def set_pose(env, wname, pose, timeout_ms=5000, attempts=4):
    """Repositions the probe. With retries: the service sometimes times out
    on a loaded machine, and losing one call would truncate the whole
    sequence."""
    x, y, z, r, p, yw = pose
    qx, qy, qz, qw = rpy_to_quat(r, p, yw)
    req = (f'name: "{PROBE_NAME}", '
           f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
           f'orientation: {{x: {qx:.6f}, y: {qy:.6f}, '
           f'z: {qz:.6f}, w: {qw:.6f}}}')
    for k in range(attempts):
        res = subprocess.run(
            ["/usr/bin/gz", "service", "-s", f"/world/{wname}/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", str(timeout_ms), "--req", req],
            env=env, capture_output=True, text=True)
        if "true" in (res.stdout or "").lower():
            return True
        time.sleep(0.25 * (k + 1))
    return False


# --- modes -----------------------------------------------------------------

def capture_still(a):
    sdf = open(a.world).read()
    if a.thermal:
        from . import flight_video as fv
        sdf = fv._thermal_swap(sdf)
    env = build_env(a.world)
    out = inject_camera(sdf, " ".join(f"{v:.4f}" for v in parse_pose(a.pose)),
                        a.width, a.height, a.fov, 5.0)
    tmp = "/tmp/solar_farm_capture_still.sdf"
    open(tmp, "w").write(out)

    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.transport13 import Node

    proc = start_server(tmp, env, a.verbose)
    sink = FrameSink()
    node = Node()
    node.subscribe(GzImage, CAM_TOPIC, sink)
    ok = wait_for_frames(sink, proc, 1, a.timeout)
    stop_server(proc)
    del node

    if not ok:
        print(f"no frame within {a.timeout}s", file=sys.stderr)
        return 1
    img = _thermal_colour(sink.img) if a.thermal else sink.img
    PILImage.fromarray(img).save(a.out)
    print(f"  {a.out}")
    return 0


def capture_fly(a):
    if not a.path:
        raise SystemExit("--fly needs --path with at least two waypoints")
    sdf = open(a.world).read()
    if a.thermal:
        from . import flight_video as fv
        sdf = fv._thermal_swap(sdf)
    env = build_env(a.world)
    wname = world_name(sdf)
    track = lerp_path([parse_pose(p) for p in a.path.split(";")], a.frames,
                      ease=a.ease, corner_radius=a.corner_radius,
                      bank_deg=a.bank_deg, fps=a.fps)

    start = " ".join(f"{v:.4f}" for v in track[0])
    tmp = "/tmp/solar_farm_capture_fly.sdf"
    open(tmp, "w").write(inject_camera(sdf, start, a.width, a.height,
                                       a.fov, a.rate))

    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.transport13 import Node

    print(f"loading world ({wname})...", flush=True)
    proc = start_server(tmp, env, a.verbose)
    sink = FrameSink()
    node = Node()
    node.subscribe(GzImage, CAM_TOPIC, sink)

    t0 = time.time()
    if not wait_for_frames(sink, proc, 1, a.timeout):
        stop_server(proc)
        print(f"no first frame within {a.timeout}s", file=sys.stderr)
        return 1
    print(f"  loaded in {time.time()-t0:.0f}s, capturing {a.frames} frames",
          flush=True)

    os.makedirs(a.outdir, exist_ok=True)
    frames, t1 = [], time.time()
    for i, pose in enumerate(track):
        if not set_pose(env, wname, pose):
            print(f"  frame {i}: set_pose failed", file=sys.stderr)
            break
        # Discard `settle` frames so the sample can't be older than the move.
        target = sink.count + a.settle + 1
        if not wait_for_frames(sink, proc, target, a.frame_timeout):
            print(f"  frame {i}: timed out waiting for a fresh frame",
                  file=sys.stderr)
            break
        frame = _thermal_colour(sink.img) if a.thermal else sink.img.copy()
        frames.append(frame)
        if a.save_frames:
            PILImage.fromarray(frame).save(
                os.path.join(a.outdir, f"frame_{i:04d}.png"))
        if (i + 1) % 10 == 0 or i + 1 == len(track):
            el = time.time() - t1
            print(f"  {i+1}/{len(track)} frames  ({el:.0f}s, "
                  f"{(i+1)/max(el,1e-6):.1f} fps)", flush=True)

    stop_server(proc)
    del node

    if len(frames) < 2:
        print("not enough frames to encode", file=sys.stderr)
        return 1
    return encode(frames, a)


def encode(frames, a):
    try:
        import cv2
    except ImportError:
        print("OpenCV not available; frames are in " + a.outdir,
              file=sys.stderr)
        return 1
    h, w, _ = frames[0].shape
    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*a.fourcc),
                         a.fps, (w, h))
    if not vw.isOpened():
        print(f"could not open writer for {a.out} with fourcc {a.fourcc}",
              file=sys.stderr)
        return 1
    for f in frames:
        vw.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))   # OpenCV expects BGR
    vw.release()
    size = os.path.getsize(a.out) / 1e6
    print(f"\n  {a.out}  {len(frames)} frames @ {a.fps} fps  ({size:.1f} MB)")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", required=True)
    p.add_argument("--fly", action="store_true",
                   help="flythrough instead of a single still")
    p.add_argument("--thermal", action="store_true",
                   help="probe reads the simulated thermal channel "
                        "(false-colour) instead of visible light; works "
                        "with --fly as well as a single still")
    p.add_argument("--pose", default="15 15 10 0 0.45 2.2",
                   help="x y z roll pitch yaw, for a still")
    p.add_argument("--path", default=None,
                   help="semicolon-separated poses to interpolate through")
    p.add_argument("--frames", type=int, default=150)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--no-ease", dest="ease", action="store_false",
                   default=True,
                   help="--fly only: disable ease-in/ease-out timing, "
                        "reverting to constant cruise speed start-to-finish")
    p.add_argument("--corner-radius", type=float, default=6.0,
                   help="--fly only: Gaussian sigma (frames) rounding off "
                        "each interior waypoint corner into a curve; 0 "
                        "reverts to a hard corner (default 6.0)")
    p.add_argument("--bank-deg", type=float, default=15.0,
                   help="--fly only: maximum roll (degrees) added when "
                        "banking into a turn; 0 disables banking "
                        "(default 15.0)")
    p.add_argument("--rate", type=float, default=10.0,
                   help="camera sensor update rate, Hz of sim time")
    p.add_argument("--settle", type=int, default=2,
                   help="frames to discard after each reposition")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fov", type=float, default=1.05)
    p.add_argument("--timeout", type=float, default=300.0,
                   help="seconds to wait for the world to load")
    p.add_argument("--frame-timeout", type=float, default=30.0)
    p.add_argument("--fourcc", default="mp4v")
    p.add_argument("--save-frames", action="store_true",
                   help="also write individual PNGs")
    p.add_argument("--outdir", default="frames")
    p.add_argument("-o", "--out", default=None)
    p.add_argument("-v", "--verbose", action="store_true")
    a = p.parse_args(argv)

    if a.out is None:
        a.out = "flythrough.mp4" if a.fly else "preview.png"
    return capture_fly(a) if a.fly else capture_still(a)


if __name__ == "__main__":
    sys.exit(main())
