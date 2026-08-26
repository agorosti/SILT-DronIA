#!/usr/bin/env python3
"""Headless still and flythrough capture from a generated world.

There is no discrete GPU on the reference machine and the Gazebo GUI is the
expensive half of the renderer, so imagery is produced by injecting a camera
into the world, running the server with no GUI, and pulling frames straight off
the gz-transport image topic.

Two modes:

    still     one launch, one frame, fixed pose
    fly       one launch, camera repositioned per frame via the set_pose
              service, frames encoded to mp4

The flythrough deliberately does *not* relaunch the world per frame. A
1000-module world takes ~18 s to load, so a 120-frame sequence built that way
would cost over half an hour; moving the camera inside one running world brings
it under a minute.

    # single still
    python3 -m solar_farm_gz.capture --world worlds/solar_farm.sdf \\
        --pose "42 8 15 0 0.36 3.0" -o array_front.png

    # flythrough along an inspection transect
    python3 -m solar_farm_gz.capture --world worlds/solar_farm.sdf --fly \\
        --path "30,-10,12,0,0.35,1.5708; 30,110,12,0,0.35,1.5708" \\
        --frames 150 --fps 30 -o flythrough.mp4
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


# --- helpers ----------------------------------------------------------------

def parse_pose(s):
    v = [float(x) for x in re.split(r"[,\s]+", s.strip()) if x]
    if len(v) != 6:
        raise SystemExit(f"pose needs 6 numbers (x y z roll pitch yaw), got {len(v)}")
    return v


def lerp_path(waypoints, n):
    """Piecewise-linear interpolation through waypoints, arc-length paced."""
    wp = np.array(waypoints, float)
    if len(wp) == 1:
        return np.repeat(wp, n, axis=0)
    seg = np.maximum(np.linalg.norm(np.diff(wp[:, :3], axis=0), axis=1), 1e-6)
    t = np.concatenate([[0.0], np.cumsum(seg)])
    t /= t[-1]
    q = np.linspace(0.0, 1.0, n)
    return np.stack([np.interp(q, t, wp[:, i]) for i in range(6)], axis=1)


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


class FrameSink:
    """Latest camera frame plus a monotonically increasing frame counter.

    The counter is what makes repositioning safe: after moving the camera we
    wait for N further frames before sampling, so the saved image cannot be one
    that was rendered before the move landed.
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
    """Reposition the probe. Retries: the service occasionally times out on a
    loaded machine, and losing one call would truncate the whole sequence."""
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


# --- modes ------------------------------------------------------------------

def capture_still(a):
    sdf = open(a.world).read()
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
    PILImage.fromarray(sink.img).save(a.out)
    print(f"  {a.out}")
    return 0


def capture_fly(a):
    if not a.path:
        raise SystemExit("--fly needs --path with at least two waypoints")
    sdf = open(a.world).read()
    env = build_env(a.world)
    wname = world_name(sdf)
    track = lerp_path([parse_pose(p) for p in a.path.split(";")], a.frames)

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
        # Discard `settle` frames so the sample cannot predate the move.
        target = sink.count + a.settle + 1
        if not wait_for_frames(sink, proc, target, a.frame_timeout):
            print(f"  frame {i}: timed out waiting for a fresh frame",
                  file=sys.stderr)
            break
        frames.append(sink.img.copy())
        if a.save_frames:
            PILImage.fromarray(sink.img).save(
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
    p.add_argument("--pose", default="15 15 10 0 0.45 2.2",
                   help="x y z roll pitch yaw, for a still")
    p.add_argument("--path", default=None,
                   help="semicolon-separated poses to interpolate through")
    p.add_argument("--frames", type=int, default=150)
    p.add_argument("--fps", type=int, default=30)
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
