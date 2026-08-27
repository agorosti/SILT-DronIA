#!/usr/bin/env python3
"""Record an inspection flight: chase view with the live nadir feed inset.

This is a real flight, not a camera animation. ArduPilot SITL flies the
aircraft in GUIDED mode with the inspection parameters the client actually
flew (8 m altitude, 1.5 m/s cruise), and both cameras record whatever
happens. If the controller wobbles, the video shows it.

    python3 -m solar_farm_gz.flight_video --world worlds/solar_farm.sdf \\
        --ardupilot ~/ardupilot --duration 40 -o videos/inspection_flight.mp4

Three details drive the design:

* **The drone is embedded inline, not included.** The chase camera has to
  move with the aircraft, and repositioning a separate static model via the
  set_pose service costs a subprocess round trip per frame -- too slow at
  20+ fps. Embedding the model lets the chase camera hang off base_link via
  a fixed joint, so it follows the drone for free. The shipped model.sdf is
  left untouched; the extra camera only exists in the temporary capture
  world.

* **The aircraft spawns nose-aligned with the rows** and is flown with
  velocity in the body frame. ArduPilot's NED axes and Gazebo's world axes
  differ by a rotation that's easy to get wrong; using MAV_FRAME_BODY_NED
  sidesteps the conversion entirely, and a nose-forward transect is also
  what a chase camera wants to look at.

* **Frames are streamed to the encoder.** A 40 s flight is ~900 frames;
  buffering 1920x1080 RGB would be about 5.6 GB, so each composited frame
  is written and discarded.
"""

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np

from . import capture

# ---------------------------------------------------------------------
# Route mode (--route): fly real table-to-table waypoints by absolute
# GPS position instead of a fixed-heading body-frame cruise. Ported from
# autonomous_flight.py (repo root), which flies this exact route
# reliably because it never depends on which way the aircraft's nose is
# pointing at spawn -- only --yaw-deg's straight-cruise mode does. See
# fly_route() below and the "Encontrado hoy" entries in docs/ROADMAP.md
# for the reliability problem this works around.
#
# HOME_LAT/HOME_LON are the same constants autonomous_flight.py uses;
# they were cross-checked this session against a live sitl.log ("Home:
# -35.363262 149.165237"), so they're known correct for this project's
# ArduPilot parameter set, not a guess.
HOME_LAT = -35.363262
HOME_LON = 149.165237

# Length of one table (10 modules in a row) -- see docs/METHODOLOGY.md.
TABLE_LENGTH_M = 10.68


def read_table_positions(sdf_path):
    """Extracts (name, x, y, yaw) for every table in the world, in metres
    and radians, in Gazebo's local coordinate system. Identical to the
    function of the same name in autonomous_flight.py."""
    sdf = open(sdf_path).read()
    tables = re.findall(
        r'<model name="(table_\d+)">.*?<pose[^>]*>([^<]+)</pose>',
        sdf, re.DOTALL,
    )
    positions = []
    for name, pose in tables:
        parts = pose.split()
        x, y = float(parts[0]), float(parts[1])
        yaw = float(parts[5]) if len(parts) >= 6 else 0.0
        positions.append((name, x, y, yaw))
    return positions


def table_endpoints(x, y, yaw):
    """Returns the table's two endpoints (along its 10.68 m axis),
    accounting for its orientation (yaw)."""
    half = TABLE_LENGTH_M / 2.0
    dx = -half * math.sin(yaw)
    dy = half * math.cos(yaw)
    return (x - dx, y - dy), (x + dx, y + dy)


def group_into_rows(positions, tolerance_m):
    """Groups the tables into parallel rows by their X coordinate (each
    table is already a full 10.68 m row along Y; distinct rows are
    separated by ~6.5 m in X)."""
    sorted_pos = sorted(positions, key=lambda p: p[1])  # sort by X
    rows = []
    for name, x, y, yaw in sorted_pos:
        if rows and abs(x - rows[-1][0][1]) <= tolerance_m:
            rows[-1].append((name, x, y, yaw))
        else:
            rows.append([(name, x, y, yaw)])
    return rows


def build_boustrophedon_route(positions, tolerance_m):
    """For each row (group of tables with similar X), sorts them by Y to
    fly them in the correct physical order along the row, and visits
    both endpoints of each table. Alternates direction on each row for a
    continuous zigzag route. Returns a list of (name, x, y) in Gazebo
    local coordinates -- the same route logic as autonomous_flight.py,
    reused here instead of kept as a second, separate implementation."""
    rows = group_into_rows(positions, tolerance_m)
    route = []
    going_up = True
    for row in rows:
        row_sorted = sorted(row, key=lambda p: p[2], reverse=not going_up)
        for name, x, y, yaw in row_sorted:
            p1, p2 = table_endpoints(x, y, yaw)
            if going_up:
                entry, exit_ = (p1, p2) if p1[1] <= p2[1] else (p2, p1)
            else:
                entry, exit_ = (p1, p2) if p1[1] >= p2[1] else (p2, p1)
            route.append((f"{name}_in", *entry))
            route.append((f"{name}_out", *exit_))
        going_up = not going_up
    return route


def local_xy_to_latlon(x, y, home_lat=HOME_LAT, home_lon=HOME_LON):
    """x = metres East of the world origin, y = metres North of it (ENU
    convention, matching the Gazebo/ArduPilot bridge) -> absolute
    (lat, lon). Assumes the world origin coincides with SITL's HOME
    point, same assumption autonomous_flight.py documents and this
    session verified against sitl.log."""
    d_lat = y / 111320.0
    d_lon = x / (111320.0 * math.cos(math.radians(home_lat)))
    return home_lat + d_lat, home_lon + d_lon


def distance_m(lat1, lon1, lat2, lon2):
    d_lat = (lat2 - lat1) * 111320.0
    d_lon = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.sqrt(d_lat**2 + d_lon**2)

# Chase camera: behind and above, looking forward and slightly down. The
# offsets are in the aircraft's body frame, so this holds through turns.
CHASE_BACK = 7.0
CHASE_UP = 2.6
CHASE_TOPIC = "/cine/chase"
NADIR_TOPIC = "/x500_rgb/nadir"

# Text overlaid on the video: customisable via .env (see _load_env), and
# these are the fallback value if the key isn't in the file. Never needed
# more than these three strings, so there's no reason to pull in
# python-dotenv for this -- a 12-line parser is enough.
DEFAULT_TITLE_LINE1 = "Solar PV Farm - Autonomous Inspection Flight"
DEFAULT_TITLE_LINE2 = "Holybro X500 V2  |  Raspberry Pi Camera Module 3, nadir"
DEFAULT_STATUS_LABEL = "ArduPilot SITL - GUIDED"

# Contrast stretch for the thermal false colour, calibrated against actual
# values measured on a render (not against pv_textures.py's theoretical
# 0.40-0.74, which the scene's PBR lighting shifts): clean panel ~132/255,
# defect peaks ~160-171/255, background grass ~67/255. Without this
# stretch, that real range (a ~40-value band) gets painted over the
# palette's 256 and everything comes out the same dull red -- which is
# exactly what this fix was correcting for.
THERMAL_LOW, THERMAL_HIGH = 110.0, 170.0


def _load_env(path):
    """Minimal .env parser: KEY=VALUE per line, '#' as a comment, optional
    single/double quotes around the value. Doesn't pull in a new dependency
    just for three customisable strings."""
    env = {}
    if not path or not os.path.exists(path):
        return env
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, _, v = line.partition("=")
        k, v = k.strip(), v.strip()
        if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
            v = v[1:-1]
        env[k] = v
    return env


def _world_stats_suffix(world_path):
    """'  |  N modules, M defects' read from the world's actual defects.json,
    instead of a fixed string: a video recorded over a 350-panel world
    shouldn't keep announcing the 1000 panels of the world this script was
    originally written against."""
    dj = os.path.join(os.path.dirname(os.path.abspath(world_path)),
                      "defects.json")
    if not os.path.exists(dj):
        return ""
    try:
        d = json.load(open(dj))
        return f"  |  {d['modules']} modules, {d['defect_instances']} defects"
    except Exception:
        return ""

_CHASE_LINK = """
    <link name="chase_cam_link">
      <pose>{back:.3f} 0 {up:.3f} 0 {pitch:.4f} 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia><ixx>1e-08</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-08</iyy><iyz>0</iyz><izz>1e-08</izz></inertia>
      </inertial>
      <sensor name="chase" type="camera">
        <topic>{topic}</topic>
        <update_rate>{rate}</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>1.20</horizontal_fov>
          <image><width>{w}</width><height>{h}</height>
            <format>R8G8B8</format></image>
          <clip><near>0.1</near><far>2000</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="chase_cam_joint" type="fixed">
      <parent>base_link</parent>
      <child>chase_cam_link</child>
    </joint>
"""


def _thermal_swap(world_text):
    """Converts the world to its 'thermal' variant: every table's
    albedo_map is repointed at the pv_atlas_NN_thermal.png generated
    alongside the visible one, instead of pv_atlas_NN_albedo.png -- same
    UV, same position for every defect, exactly the "material swap on
    existing assets" the project's methodology describes for a future
    thermal camera (docs/METHODOLOGY.md), just actually applied now.
    Doesn't touch ground_albedo.png (no thermal equivalent) because the
    pattern requires the numbered pv_atlas_NN_ prefix.

    Also turns off shadows: a thermal camera reads temperature, not
    reflected light, so a neighbouring table's shadow shouldn't cool
    (darken) the reading on the table next to it -- with normal shadows on,
    the colour map would paint false cold spots where there's really just
    less visible light, not less heat."""
    world_text = re.sub(r"pv_atlas_(\d+)_albedo\.png",
                        r"pv_atlas_\1_thermal.png", world_text)
    world_text = re.sub(r"<shadows>true</shadows>",
                        "<shadows>false</shadows>", world_text)
    world_text = re.sub(r"<cast_shadows>true</cast_shadows>",
                        "<cast_shadows>false</cast_shadows>", world_text)
    return world_text


def build_capture_world(world_path, model_path, spawn, chase_wh, rate, outdir,
                        thermal=False):
    """Farm world + the drone embedded with a chase camera attached."""
    world = open(world_path).read()
    if thermal:
        world = _thermal_swap(world)
    model = open(model_path).read()

    # Strip the model's own sdf wrapper so the <model> can be embedded.
    start = model.index("<model name=")
    end = model.rindex("</model>") + len("</model>")
    model_body = model[start:end]

    pitch = math.atan2(CHASE_UP, CHASE_BACK)
    chase = _CHASE_LINK.format(back=-CHASE_BACK, up=CHASE_UP, pitch=pitch,
                               topic=CHASE_TOPIC, rate=rate,
                               w=chase_wh[0], h=chase_wh[1])
    model_body = model_body.replace("</model>", chase + "\n  </model>", 1)

    x, y, z, yaw = spawn
    posed = model_body.replace(
        "<model name=\"x500_rgb\">",
        f"<model name=\"x500_rgb\">\n    "
        f"<pose>{x} {y} {z} 0 0 {yaw}</pose>", 1)

    world = world.replace("</world>", posed + "\n  </world>", 1)
    out = os.path.join(outdir, "capture_world.sdf")
    with open(out, "w") as f:
        f.write(world)
    return out


def _default_model():
    """Locates x500_rgb/model.sdf in an installed package, or failing that
    in the source tree. The two layouts differ: the installed Python lands
    in lib/pythonX/site-packages, which isn't near share/."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("solar_farm_gz")
        cand = os.path.join(share, "models", "x500_rgb", "model.sdf")
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "models", "x500_rgb", "model.sdf")


def start_sitl(ardupilot, workdir, log):
    binary = os.path.join(ardupilot, "build", "sitl", "bin", "arducopter")
    if not os.path.exists(binary):
        raise SystemExit(f"arducopter not built at {binary}")
    defaults = ",".join(
        os.path.join(ardupilot, "Tools", "autotest", "default_params", p)
        for p in ("copter.parm", "gazebo-iris.parm"))
    return subprocess.Popen(
        [binary, "-w", "--model", "JSON", "--speedup", "1",
         "--sim-address=127.0.0.1", "--defaults", defaults, "-I0"],
        cwd=workdir, stdout=log, stderr=subprocess.STDOUT)


def _arm_and_takeoff(mav, alt):
    """GPS wait, GUIDED, arm, takeoff, climb -- the prelude shared by
    every flight mode this module offers (straight-cruise fly() and
    waypoint-following fly_route()). Pulled out of fly() unchanged so the
    two modes can't drift apart on the part that's already proven to
    work; only what happens *after* reaching altitude differs between
    them."""
    from pymavlink import mavutil

    # Wait for a position fix before touching the mode or arming. A cold
    # SITL needs almost a full minute for GPS and the EKF to settle, and
    # arming before that fails the prearm checks instead of waiting
    # politely.
    print("  waiting for GPS/EKF ...", flush=True)
    t0 = time.time()
    fixed = False
    while time.time() - t0 < 180:
        g = mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if g and g.fix_type >= 3:
            fixed = True
            print(f"  3D fix, {g.satellites_visible} sats "
                  f"({time.time() - t0:.0f}s)", flush=True)
            break
    if not fixed:
        raise SystemExit("no GPS fix; the aircraft cannot be armed")

    mav.set_mode_apm('GUIDED')
    time.sleep(2)

    # Prearm can still reject for a few seconds after the fix appears, so
    # the arm request is retried instead of sent just once.
    t0, seen = time.time(), set()
    while time.time() - t0 < 90 and not mav.motors_armed():
        mav.arducopter_arm()
        for _ in range(10):
            m = mav.recv_match(type=['HEARTBEAT', 'STATUSTEXT'],
                               blocking=True, timeout=1)
            # Surfaces the rejection reason. "PreArm: ..." is the only
            # thing that says why, and swallowing it turns a one-line
            # parameter tweak into a whole afternoon.
            if m and m.get_type() == 'STATUSTEXT':
                txt = m.text.strip()
                if txt not in seen and ('rm:' in txt or 'rror' in txt):
                    seen.add(txt)
                    print(f"    {txt}", flush=True)
            if mav.motors_armed():
                break
    if not mav.motors_armed():
        raise SystemExit("aircraft would not arm")
    print(f"  armed ({time.time() - t0:.0f}s)", flush=True)

    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                              0, 0, 0, 0, 0, 0, 0, alt)

    # Climb before recording: the first few seconds are a vertical ascent
    # with nothing but grass below, which makes for a bad opening shot.
    t0 = time.time()
    while time.time() - t0 < 45:
        m = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                           timeout=3)
        if m and m.relative_alt / 1000.0 >= alt * 0.95:
            break
    print(f"  at altitude ({time.time() - t0:.0f}s)", flush=True)


def fly(mav, alt, speed, duration, on_tick, bob_amplitude=0.0, bob_pitch=11.88):
    """Takes off, then holds a forward cruise in the body frame, calling
    on_tick throughout.

    If bob_amplitude > 0, a sinusoidal dip-and-return is superimposed on
    the horizontal cruise: one full cycle every time the aircraft crosses
    bob_pitch metres (by default, the table pitch, so the bob feels
    synced to "moving on to the next row of cells" instead of being an
    arbitrary metronome). It's a vertical-velocity feed-forward, not a
    position target: no altitude feedback, but GUIDED will track it with
    enough fidelity for the visual effect, and the HUD shows the actual
    measured altitude, not the desired one.

    Reliability note: this mode steers by a fixed spawn heading
    (--yaw-deg) and cruises nose-forward from there, so it only flies
    along the rows if that heading happens to be right for the world
    being recorded -- it isn't derived from anything, just guessed and
    tested by eye (see docs/ROADMAP.md). fly_route() below doesn't have
    this problem; prefer it (--route) unless this mode's straight,
    constant-speed cruise is specifically what's needed."""
    from pymavlink import mavutil

    _arm_and_takeoff(mav, alt)

    # Velocity-only mask, body frame. Resent continuously: ArduPilot
    # expires a guided setpoint after a few seconds and would stop in a
    # hover.
    mask = 0b0000111111000111
    t0 = time.time()
    last_cmd = 0.0
    # One full dip-and-return cycle per bob_pitch metres of forward travel.
    period = (bob_pitch / speed) if bob_amplitude > 0 and speed > 0 else 0.0
    omega = (2.0 * math.pi / period) if period > 0 else 0.0
    hud = {"alt": alt, "spd": 0.0, "gsd": 0.0, "swath": 0.0}
    # Surfaces failsafes/mode changes during the cruise itself -- the
    # arming loop above already listens for STATUSTEXT, but once flying
    # starts nothing did, so a fence/EKF/battery failsafe forcing a mode
    # change (e.g. an unexplained descent) used to happen completely
    # silently. Both reads are non-blocking, same reasoning as the
    # position poll below.
    seen_flight_msgs = set()
    last_mode = None
    while time.time() - t0 < duration:
        now = time.time()
        # See the identical drain loop in fly_route() for why this is one
        # untyped recv_match() drained in a loop rather than three separate
        # type-filtered calls: each of those only inspects a single queued
        # message and discards it if it's not a match, so with three types
        # competing every iteration, a GLOBAL_POSITION_INT can get thrown
        # away by the HEARTBEAT or STATUSTEXT call before ever reaching the
        # one that wants it. Harmless here since this mode only feeds the
        # HUD from it, but kept consistent with fly_route() rather than
        # leaving the same landmine in place.
        hb, st, m = None, None, None
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT':
                hb = msg
            elif mtype == 'STATUSTEXT':
                st = msg
                txt = st.text.strip()
                if txt not in seen_flight_msgs:
                    seen_flight_msgs.add(txt)
                    print(f"\n  [{now - t0:5.1f}s] {txt}", flush=True)
            elif mtype == 'GLOBAL_POSITION_INT':
                m = msg
        if hb:
            mode = mavutil.mode_string_v10(hb)
            if mode != last_mode:
                print(f"\n  [{now - t0:5.1f}s] mode -> {mode}", flush=True)
                last_mode = mode
        if now - last_cmd > 0.25:
            # Vertical descent rate (NED, positive down) that traces a
            # relative-altitude profile e(t) = -(A/2)(1 - cos(wt)): starts
            # at 0, dips to -A at mid-cycle, returns to 0 -- never climbs
            # above cruise altitude, only "dips" the flight.
            vz = (bob_amplitude * omega / 2.0 * math.sin(omega * (now - t0))
                  if omega > 0 else 0.0)
            mav.mav.set_position_target_local_ned_send(
                0, mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                mask, 0, 0, 0, speed, 0, vz, 0, 0, 0, 0, 0)
            last_cmd = now

        if m:
            hud["alt"] = m.relative_alt / 1000.0
            hud["spd"] = math.hypot(m.vx, m.vy) / 100.0
            # Derived from the camera's real geometry rather than
            # hardcoded, so the overlay stays accurate if the aircraft
            # drifts off altitude.
            hud["swath"] = 2.0 * hud["alt"] * math.tan(1.151917 / 2.0)
            hud["gsd"] = hud["swath"] * 1000.0 / 1920.0
        on_tick(now - t0, hud)
        time.sleep(0.005)


def fly_route(mav, route, alt, on_tick, duration,
              arrival_radius_m=1.5, waypoint_timeout_s=25):
    """Takes off, then flies a sequence of absolute GPS waypoints
    (table-to-table, e.g. from build_boustrophedon_route() +
    local_xy_to_latlon()), calling on_tick throughout, same as fly().

    Unlike fly()'s body-frame cruise, this doesn't care which way the
    aircraft's nose was pointing at spawn: MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    position targets are absolute, so the spawn-yaw guessing game that
    motivated --yaw-deg (docs/ROADMAP.md) doesn't apply here -- the
    aircraft goes to each waypoint regardless of heading. This is the
    same navigation autonomous_flight.py already uses successfully; the
    only real change is folding its blocking goto()/wait_until_arrived()
    pattern into this module's non-blocking on_tick-driven loop, so
    recording keeps working the same way it does for fly().

    duration still caps the recording length as usual: if the route
    finishes first, the flight (and recording) ends there rather than
    padding out the remaining time with a hover. If duration runs out
    first, whatever waypoint was in progress is simply left unfinished.
    """
    from pymavlink import mavutil

    if not route:
        raise SystemExit("empty route; nothing to fly")

    _arm_and_takeoff(mav, alt)

    route_ll = [(name, *local_xy_to_latlon(x, y)) for name, x, y in route]
    print(f"  route: {len(route_ll)} waypoints "
          f"({len(route_ll) // 2} tables x 2 endpoints), "
          f"arrival {arrival_radius_m} m, timeout {waypoint_timeout_s} s/wp",
          flush=True)

    mask = 0b0000111111111000  # position only (matches autonomous_flight.py)
    t0 = time.time()
    last_cmd = 0.0
    wp_idx = 0
    wp_t0 = t0
    hud = {"alt": alt, "spd": 0.0, "gsd": 0.0, "swath": 0.0}
    seen_flight_msgs = set()
    last_mode = None
    last_status = 0.0
    # Unconditional, low-rate liveness line -- deliberately NOT gated behind
    # "if m:" or any other branch, so it can't go silent for the same reason
    # that might be silencing the arrival/"to go"/timeout prints below. If
    # this stops appearing, the loop body itself has stalled; if it keeps
    # appearing with since_wp_t0 climbing past waypoint_timeout_s while
    # wp_idx never changes and nothing else prints, the bug is specifically
    # in the arrival/timeout branches below, not in the loop reaching them.
    last_heartbeat_dbg = 0.0
    last_m_time = None
    while time.time() - t0 < duration and wp_idx < len(route_ll):
        now = time.time()
        if now - last_heartbeat_dbg > 2.0:
            since_m = f"{now - last_m_time:.1f}s ago" if last_m_time else "never"
            print(f"\n  [dbg {now - t0:5.1f}s] wp_idx={wp_idx}/{len(route_ll)} "
                  f"since_wp_t0={now - wp_t0:.1f}s last_m={since_m}", flush=True)
            last_heartbeat_dbg = now
        # Drain everything currently queued in ONE pass, dispatching by the
        # message's real type, instead of three separate
        # recv_match(type=X, blocking=False) calls (one per type). Each of
        # those separate calls only ever looks at a single message off the
        # connection; if that message isn't a match it's discarded right
        # there -- so with three type-filtered calls competing every loop, a
        # GLOBAL_POSITION_INT sitting behind a HEARTBEAT or STATUSTEXT in the
        # queue gets thrown away by whichever call reaches it first instead
        # of ever reaching the one actually waiting for it. This is what was
        # silently starving GLOBAL_POSITION_INT in testing (confirmed via
        # the last_m debug line above going "never" for most/all of a
        # flight) -- fly() has the same pattern but tolerates losing most
        # position updates since it only feeds the HUD; fly_route() cannot,
        # since arrival detection depends on it. Draining untyped keeps
        # every message.
        hb, st, m = None, None, None
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT':
                hb = msg
            elif mtype == 'STATUSTEXT':
                st = msg
                txt = st.text.strip()
                if txt not in seen_flight_msgs:
                    seen_flight_msgs.add(txt)
                    print(f"\n  [{now - t0:5.1f}s] {txt}", flush=True)
            elif mtype == 'GLOBAL_POSITION_INT':
                m = msg  # keep the most recent if several arrived at once
        if hb:
            mode = mavutil.mode_string_v10(hb)
            if mode != last_mode:
                print(f"\n  [{now - t0:5.1f}s] mode -> {mode}", flush=True)
                last_mode = mode

        name, wlat, wlon = route_ll[wp_idx]
        if now - last_cmd > 0.5:
            mav.mav.set_position_target_global_int_send(
                0, mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
                int(wlat * 1e7), int(wlon * 1e7), alt,
                0, 0, 0, 0, 0, 0, 0, 0)
            last_cmd = now

        if m:
            last_m_time = now
            cur_lat, cur_lon = m.lat / 1e7, m.lon / 1e7
            hud["alt"] = m.relative_alt / 1000.0
            hud["spd"] = math.hypot(m.vx, m.vy) / 100.0
            hud["swath"] = 2.0 * hud["alt"] * math.tan(1.151917 / 2.0)
            hud["gsd"] = hud["swath"] * 1000.0 / 1920.0
            d = distance_m(cur_lat, cur_lon, wlat, wlon)
            if d <= arrival_radius_m:
                print(f"\n  [{now - t0:5.1f}s] {name} reached "
                      f"({d:.1f} m)", flush=True)
                wp_idx += 1
                wp_t0 = now
            elif now - last_status > 3.0:
                # Distance-remaining, printed periodically rather than
                # only on arrival/timeout, so a stalled or wrong-direction
                # waypoint is visible while it's happening instead of only
                # after a 25 s timeout. north_m/east_m decompose the
                # remaining distance along each axis -- if one shrinks
                # while the other doesn't (or the total stops shrinking
                # despite GND SPD > 0 in the HUD), that points at the
                # local_xy_to_latlon() ENU mapping being rotated for this
                # project, the same kind of axis mismatch --yaw-deg works
                # around in the other flight mode.
                north_m = (wlat - cur_lat) * 111320.0
                east_m = (wlon - cur_lon) * 111320.0 * math.cos(
                    math.radians(cur_lat))
                print(f"\n  [{now - t0:5.1f}s] {name}: {d:5.1f} m to go "
                      f"(N {north_m:+.1f} m, E {east_m:+.1f} m), "
                      f"spd {hud['spd']:.1f} m/s", flush=True)
                last_status = now
        if now - wp_t0 > waypoint_timeout_s:
            print(f"\n  [{now - t0:5.1f}s] {name} timed out, "
                  f"moving on", flush=True)
            wp_idx += 1
            wp_t0 = now

        on_tick(now - t0, hud)
        time.sleep(0.005)

    if wp_idx >= len(route_ll):
        print(f"\n  route complete ({time.time() - t0:.0f}s)", flush=True)


_FONT = 0  # cv2.FONT_HERSHEY_SIMPLEX, avoids importing cv2 at module scope


def _shadowed(img, text, org, scale, colour, thick=1):
    """Text with a thick black outline behind it, instead of a 1 px
    diagonal shadow.

    The footage is a mix of bright grass, blue sky and near-black panels,
    so plain text -- or a 1 px shadow, which is nearly invisible at 1080p
    -- is unreadable over a good chunk of every frame. A black outline
    (same stroke, much thicker, drawn first) gives consistent contrast
    regardless of the background."""
    import cv2
    # Hershey fonts are ASCII-only and silently draw "?" for anything else.
    text = text.encode("ascii", "replace").decode("ascii")
    cv2.putText(img, text, org, _FONT, scale, (0, 0, 0), thick + 4,
                cv2.LINE_AA)
    cv2.putText(img, text, org, _FONT, scale, colour, thick, cv2.LINE_AA)


def _thermalize(img):
    """Applies the same false-colour transform to a nadir frame that
    composite() used to apply only to its own inset copy. Pulled out so
    --nadir-out can call it too: before this, --nadir-out --thermal wrote
    nadir_sink.img straight through with no colour transform at all -- the
    onboard camera's raw grayscale-ish thermal texture (contrast-band
    ~110-170/255, see THERMAL_LOW/THERMAL_HIGH), not the purple/orange
    INFERNO palette. The composite's embedded inset looked correctly
    thermal because composite() colourises its own copy; a stand-alone
    --nadir-out recording did not, silently, because nothing called this
    on it. Confirmed by comparing a frame of each side by side: the
    composite inset showed the palette, the --nadir-out frame at the same
    instant did not."""
    import cv2
    # True grayscale (not just the R channel) in case the scene's shading
    # tints the three channels slightly differently.
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).astype(np.float32)
    # Stretches THERMAL_LOW..THERMAL_HIGH to 0..255 before colouring:
    # without this, the real range (see the comment next to the constant)
    # occupies such a narrow band of the palette that everything comes out
    # the same red.
    span = THERMAL_HIGH - THERMAL_LOW
    gray = np.clip((gray - THERMAL_LOW) * (255.0 / span), 0, 255)
    gray = gray.astype(np.uint8)
    return cv2.cvtColor(cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO),
                        cv2.COLOR_BGR2RGB)


def composite(chase, nadir, inset_w, hud=None, title=None,
             status_label=DEFAULT_STATUS_LABEL, thermal=False):
    """Full chase-view frame, nadir feed inset bottom-right, telemetry HUD.

    thermal=True only changes the inset (what the aircraft's camera
    "sees"), not the outside chase view -- an outside observer still sees
    the normal daylight scene; only the onboard feed is thermal, just like
    on a real drone with two sensors."""
    import cv2
    out = chase.copy()
    h, w = out.shape[:2]
    ih = int(inset_w * nadir.shape[0] / nadir.shape[1])
    small = cv2.resize(nadir, (inset_w, ih), interpolation=cv2.INTER_AREA)
    if thermal:
        small = _thermalize(small)

    m = 24
    x0, y0 = w - inset_w - m, h - ih - m
    cv2.rectangle(out, (x0 - 3, y0 - 3), (x0 + inset_w + 3, y0 + ih + 3),
                  (235, 235, 235), 3)
    out[y0:y0 + ih, x0:x0 + inset_w] = small
    label = ("NADIR THERMAL CAMERA (SIMULATED)  1920x1080  66 deg" if thermal
             else "NADIR INSPECTION CAMERA  1920x1080  66 deg")
    _shadowed(out, label, (x0, y0 - 12), 0.52, (245, 245, 245))

    if hud:
        # Fixed formatting so the numbers don't jitter horizontally from
        # frame to frame, which looks sloppy even when the values are
        # correct.
        line = (f"ALT {hud['alt']:5.1f} m    "
                f"GND SPD {hud['spd']:4.1f} m/s    "
                f"GSD {hud['gsd']:4.1f} mm/px    "
                f"SWATH {hud['swath']:4.1f} m")
        _shadowed(out, line, (m, h - 22), 0.58, (250, 250, 250))
        # Skipped while the title band is up, since it would otherwise sit
        # on top of it.
        if not title:
            _shadowed(out, status_label, (m, 34), 0.55, (200, 230, 255))

    if title:
        band = out[: 132].astype(np.float32) * 0.35
        out[: 132] = band.astype(np.uint8)
        _shadowed(out, title[0], (m, 52), 0.95, (255, 255, 255), 2)
        _shadowed(out, title[1], (m, 92), 0.6, (215, 235, 255))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", required=True)
    p.add_argument("--model", default=None,
                   help="x500_rgb model.sdf (default: alongside the package)")
    p.add_argument("--ardupilot", default=os.path.expanduser("~/ardupilot"))
    p.add_argument("--plugin-path",
                   default=os.path.expanduser("~/ardupilot_gazebo/build"))
    p.add_argument("--alt", type=float, default=8.0)
    p.add_argument("--speed", type=float, default=1.5)
    p.add_argument("--duration", type=float, default=40.0,
                   help="seconds of cruise to record")
    p.add_argument("--spawn", default="3.25,-10,0.13",
                   help="x,y,z; nose heading is --yaw-deg (default assumes "
                        "that faces along the rows, but the Gazebo/ArduPilot "
                        "NED axes have a fixed rotation between them that "
                        "isn't always 90 deg -- see --yaw-deg)")
    p.add_argument("--yaw-deg", type=float, default=90.0,
                   help="spawn heading in degrees (0 = Gazebo model +X, 90 "
                        "= +Y); the aircraft cruises nose-forward from "
                        "there, so this is what actually determines which "
                        "way it flies. The historical default (90) assumes "
                        "Gazebo +Y lines up with ArduPilot's body-forward "
                        "after the NED/world rotation, which isn't "
                        "guaranteed for a freshly generated world -- if the "
                        "aircraft flies away from the rows instead of "
                        "along them, try 0, 180 or -90 here before anything "
                        "else")
    p.add_argument("--bob-amplitude", type=float, default=0.0,
                   help="metres of sinusoidal descend-and-return per "
                        "--bob-pitch of travel; 0 disables (default). "
                        "Only applies without --route")
    p.add_argument("--bob-pitch", type=float, default=11.88,
                   help="metres of forward travel per dip cycle, e.g. the "
                        "table pitch so one dip happens per row crossed. "
                        "Only applies without --route")
    p.add_argument("--route", action="store_true",
                   help="fly real table-to-table waypoints (read from the "
                        "world's own .sdf, same logic as the project's "
                        "autonomous_flight.py) by absolute GPS position, "
                        "instead of a fixed-heading straight cruise. "
                        "Recommended: unlike the default mode, this "
                        "doesn't depend on --yaw-deg/--spawn guessing the "
                        "right heading for a given world -- see "
                        "docs/ROADMAP.md. --duration still caps the "
                        "recording; if the route finishes first the "
                        "flight ends there instead of padding out the "
                        "rest with a hover. Not yet flight-tested against "
                        "live Gazebo/SITL -- the navigation logic is "
                        "copied from autonomous_flight.py's proven "
                        "goto()/wait_until_arrived(), but this is its "
                        "first run folded into flight_video.py's "
                        "recording loop, so treat the first attempt as a "
                        "trial and watch the console for stalled "
                        "waypoints")
    p.add_argument("--route-tolerance", type=float, default=1.0,
                   help="metres of X tolerance for grouping tables into "
                        "the same row (only with --route)")
    p.add_argument("--route-arrival-radius", type=float, default=1.5,
                   help="metres from a waypoint that counts as \"arrived\" "
                        "(only with --route)")
    p.add_argument("--route-waypoint-timeout", type=float, default=25.0,
                   help="max seconds to spend on one waypoint before "
                        "moving on regardless (only with --route)")
    p.add_argument("--thermal", action="store_true",
                   help="onboard nadir feed reads the simulated thermal "
                        "channel (false-colour) instead of visible light; "
                        "the outside chase view is unaffected")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--inset", type=int, default=416)
    p.add_argument("--title-seconds", type=float, default=5.0,
                   help="how long the title band stays up (0 to disable)")
    p.add_argument("--env-file", default=".env",
                   help="KEY=VALUE file for FLIGHT_TITLE_LINE1/2 and "
                        "FLIGHT_STATUS_LABEL; missing keys/file fall back "
                        "to the built-in defaults")
    p.add_argument("--title-line1", default=None,
                   help="overrides --env-file / the built-in default")
    p.add_argument("--title-line2", default=None,
                   help="overrides --env-file / the built-in default "
                        "(the real module/defect count is appended "
                        "automatically from the world's defects.json)")
    p.add_argument("--status-label", default=None,
                   help="overrides --env-file / the built-in default")
    p.add_argument("--fourcc", default="mp4v")
    p.add_argument("-o", "--out", default="inspection_flight.mp4")
    p.add_argument("--nadir-out", default=None,
                   help="also write the raw, undecorated nadir feed (native "
                        "resolution, no HUD/inset border) to this path -- "
                        "the same signal the detector trains on, recorded "
                        "alongside the composite --out video from the same "
                        "flight instead of as a separate run")
    p.add_argument("--keep", action="store_true",
                   help="keep the temporary capture world")
    a = p.parse_args(argv)

    try:
        import cv2
    except ImportError:
        raise SystemExit("OpenCV is required to encode the video")
    from gz.transport13 import Node
    from pymavlink import mavutil

    model = a.model or _default_model()
    model = os.path.abspath(model)
    if not os.path.exists(model):
        raise SystemExit(f"drone model not found: {model}\n"
                         "pass --model to point at x500_rgb/model.sdf")

    tmp = tempfile.mkdtemp(prefix="flightvid_")
    sx, sy, sz = (float(v) for v in a.spawn.split(","))
    world_file = build_capture_world(
        a.world, model, (sx, sy, sz, math.radians(a.yaw_deg)),
        (a.width, a.height), a.fps, tmp, thermal=a.thermal)
    wname = capture.world_name(open(world_file).read())

    env = capture.build_env(a.world)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
        a.plugin_path + ":" + env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""))
    # Model URIs inside the farm world resolve against the world's
    # directory; the drone is embedded, so it doesn't need its own
    # resource entry.
    try:
        from . import gpu
        env.update(gpu.offload_env())
    except Exception:
        pass

    chase_sink, nadir_sink = capture.FrameSink(), capture.FrameSink()
    node = Node()

    print(f"[1/4] starting simulator ({wname})", flush=True)
    server = capture.start_server(world_file, env, verbose=False)
    from gz.msgs10.image_pb2 import Image as GzImage
    node.subscribe(GzImage, CHASE_TOPIC, chase_sink)
    node.subscribe(GzImage, NADIR_TOPIC, nadir_sink)

    if not capture.wait_for_frames(chase_sink, server, 3, 180):
        capture.stop_server(server)
        raise SystemExit("no chase frames; is the world loading?")
    if not capture.wait_for_frames(nadir_sink, server, 3, 120):
        capture.stop_server(server)
        raise SystemExit("no nadir frames; did the drone spawn?")
    print("  both cameras live", flush=True)

    sitl_log = open(os.path.join(tmp, "sitl.log"), "w")
    print("[2/4] starting ArduPilot SITL", flush=True)
    sitl = start_sitl(a.ardupilot, tmp, sitl_log)

    mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
    if mav.wait_heartbeat(timeout=120) is None:
        capture.stop_server(server)
        sitl.terminate()
        raise SystemExit("no MAVLink heartbeat from SITL")
    for mid, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5),
                    (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2)):
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mid, int(1e6 / hz), 0, 0, 0, 0, 0)
    print("  link up", flush=True)

    writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*a.fourcc),
                             a.fps, (a.width, a.height))
    if not writer.isOpened():
        capture.stop_server(server)
        sitl.terminate()
        raise SystemExit(f"could not open video writer for {a.out}")

    nadir_writer = None
    if a.nadir_out:
        nh, nw = nadir_sink.img.shape[:2]
        nadir_writer = cv2.VideoWriter(a.nadir_out, cv2.VideoWriter_fourcc(*a.fourcc),
                                       a.fps, (nw, nh))
        if not nadir_writer.isOpened():
            capture.stop_server(server)
            sitl.terminate()
            raise SystemExit(f"could not open video writer for {a.nadir_out}")

    state = {"n": 0, "last": -1}

    # Text resolution order: explicit --flag > .env > built-in default. The
    # module/defect count on the second line isn't text-customisable -- it's
    # computed from the actual defects.json of the world being recorded, so
    # it never again announces "1000 modules, 420 defects" over a different
    # world.
    env_text = _load_env(a.env_file)
    title_line1 = (a.title_line1 or env_text.get("FLIGHT_TITLE_LINE1")
                   or DEFAULT_TITLE_LINE1)
    title_line2 = (a.title_line2 or env_text.get("FLIGHT_TITLE_LINE2")
                   or DEFAULT_TITLE_LINE2) + _world_stats_suffix(a.world)
    status_label = (a.status_label or env_text.get("FLIGHT_STATUS_LABEL")
                    or DEFAULT_STATUS_LABEL)

    # ASCII only: OpenCV draws with vector Hershey fonts, which have no
    # glyphs beyond ASCII and render an em dash or a middle dot as "???".
    title = (title_line1, title_line2)

    def on_tick(elapsed, hud):
        # Gated on the chase camera's own frame counter so every recorded
        # frame is a genuinely new render, not the same one sampled twice.
        if chase_sink.count == state["last"] or chase_sink.img is None:
            return
        if nadir_sink.img is None:
            return
        state["last"] = chase_sink.count
        writer.write(cv2.cvtColor(
            composite(chase_sink.img, nadir_sink.img, a.inset, hud,
                      title if elapsed < a.title_seconds else None,
                      status_label, a.thermal),
            cv2.COLOR_RGB2BGR))
        if nadir_writer is not None:
            # Same frame the inset is cropped from, but at native
            # resolution and with no border/label/HUD drawn on top --
            # exactly what the detector trains on. With --thermal, also
            # apply the same false-colour transform composite() applies to
            # its own inset copy -- otherwise this would write the raw,
            # uncoloured onboard thermal texture instead of the palette a
            # real simulated thermal camera would output (see
            # _thermalize()'s docstring for how this was caught).
            nadir_frame = (_thermalize(nadir_sink.img) if a.thermal
                           else nadir_sink.img)
            nadir_writer.write(cv2.cvtColor(nadir_frame, cv2.COLOR_RGB2BGR))
        state["n"] += 1
        if state["n"] % 30 == 0:
            print(f"\r  {elapsed:5.1f}s  {state['n']} frames", end="",
                  flush=True)

    try:
        if a.route:
            positions = read_table_positions(a.world)
            route_xy = build_boustrophedon_route(positions, a.route_tolerance)
            print(f"[3/4] flying route: {a.alt} m, {len(route_xy)} waypoints "
                  f"({len(positions)} tables), capped at {a.duration}s",
                  flush=True)
            fly_route(mav, route_xy, a.alt, on_tick, a.duration,
                      arrival_radius_m=a.route_arrival_radius,
                      waypoint_timeout_s=a.route_waypoint_timeout)
        else:
            print(f"[3/4] flying: {a.alt} m, {a.speed} m/s, {a.duration}s",
                  flush=True)
            fly(mav, a.alt, a.speed, a.duration, on_tick,
                bob_amplitude=a.bob_amplitude, bob_pitch=a.bob_pitch)
    finally:
        print("\n[4/4] finishing", flush=True)
        writer.release()
        if nadir_writer is not None:
            nadir_writer.release()
        try:
            sitl.terminate()
            sitl.wait(timeout=10)
        except Exception:
            sitl.kill()
        sitl_log.close()
        capture.stop_server(server)
        if not a.keep:
            shutil.rmtree(tmp, ignore_errors=True)
        else:
            print(f"  capture world kept in {tmp}")

    if state["n"] == 0:
        raise SystemExit("no frames recorded")
    size = os.path.getsize(a.out) / 1e6
    print(f"\n  {a.out}  {state['n']} frames @ {a.fps} fps  ({size:.1f} MB)")
    if a.nadir_out and os.path.exists(a.nadir_out):
        nsize = os.path.getsize(a.nadir_out) / 1e6
        print(f"  {a.nadir_out}  (raw nadir, native resolution, {nsize:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
