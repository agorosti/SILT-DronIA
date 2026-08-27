#!/usr/bin/env python3
"""
Automatic survey flight over the PV farm (Gazebo + ArduPilot SITL).

What it does, in order:
    1. Reads the real position of EVERY table in the world directly from
       the .sdf file (doesn't assume a uniform grid).
    2. Groups them into columns and builds a boustrophedon (zigzag) route
       that visits all of them, covering the full field whatever its
       shape.
    3. Connects to the drone over MAVLink (the same vehicle you see in
       MAVProxy).
    4. Switches to GUIDED mode and arms the motors.
    5. Takes off to the given altitude (8 m by default).
    6. Flies the full route, waiting at each point until close enough
       before moving to the next.
    7. Returns to home (RTL) automatically when done.

Requirements:
    - Gazebo + the solar farm world must already be running
      (Terminal 1: ros2 launch solar_farm_gz inspection.launch.py)
    - ArduPilot SITL must already be running
      (Terminal 2: Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map)
    - pymavlink installed (you already have it: pip install pymavlink)

How to run it:
    Leave Terminals 1 and 2 open and running. In a THIRD terminal:

        python3 autonomous_flight.py

    If the script can't connect, explicitly add an extra MAVLink output
    when launching sim_vehicle.py (Terminal 2):

        Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \\
            --console --map --out=udp:127.0.0.1:14550

IMPORTANT -- calibration assumption:
    This script assumes the Gazebo world origin (x=0, y=0) coincides with
    the HOME point ArduPilot SITL uses (HOME_LAT/HOME_LON below), and that
    the world's X axis points East and its Y axis points North (the
    standard ENU convention of the ArduPilot-Gazebo plugin). That's the
    most reasonable assumption without more information, but while the
    drone is flying, confirm visually (the "Follow" view in Gazebo) that
    the first tables visited really are the ones closest to home -- if the
    route starts off skewed, the most likely fix is swapping
    north_m/east_m or flipping a sign in to_latlon().
"""

import math
import os
import re
import time

from pymavlink import mavutil

# ---------------------------------------------------------------------
# CONFIGURATION -- adjust here if needed
# ---------------------------------------------------------------------

CONNECTION_STRING = "udp:127.0.0.1:14550"

# Path to the generated world (to read the tables' real positions).
WORLD_SDF_PATH = os.path.expanduser(
    "~/solar_farm_sim/src/solar_farm_gz/worlds/solar_farm.sdf")

# Flight altitude, in metres (relative to takeoff).
ALTITUDE = 8

# HOME point ArduPilot SITL uses (the same one you already located in MAVProxy).
HOME_LAT = -35.363262
HOME_LON = 149.165237

# Tolerance (in metres) for grouping tables into the same physical row
# (tables that share Y, within this margin, are considered the same row).
ROW_TOLERANCE_M = 1.0

# Arrival radius: a point is considered "reached" below this distance (m).
ARRIVAL_RADIUS_M = 1.0

# Maximum time to wait per waypoint before moving on (seconds).
WAYPOINT_TIMEOUT_S = 30


# ---------------------------------------------------------------------
# Reading the tables' real positions from the .sdf
# ---------------------------------------------------------------------

def read_table_positions(sdf_path):
    """Extracts (name, x, y, yaw) for every table in the world, in metres
    and radians, in Gazebo's local coordinate system."""
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


# Length of one table (10 modules in a row) -- see docs/METHODOLOGY.md.
TABLE_LENGTH_M = 10.68


def table_endpoints(x, y, yaw):
    """Returns the table's two endpoints (along its 10.68 m axis),
    accounting for its orientation (yaw)."""
    half = TABLE_LENGTH_M / 2.0
    dx = -half * math.sin(yaw)
    dy = half * math.cos(yaw)
    return (x - dx, y - dy), (x + dx, y + dy)


def group_into_rows(positions, tolerance_m):
    """Groups the tables into PARALLEL ROWS by their X coordinate (each
    table is already a full 10.68 m row along Y; distinct rows are
    separated by ~6.5 m in X). Returns a list of rows (each one a list
    with a single table for now, but grouped and sorted from lowest to
    highest X)."""
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
    fly them in the correct physical order along the row, and visits both
    endpoints of each table (entering from whichever one matches the
    direction of travel). Alternates direction on each row for a
    continuous zigzag route."""
    rows = group_into_rows(positions, tolerance_m)
    print(f"Rows (tables) detected, grouped by X position: {len(rows)} groups")
    for i, row in enumerate(rows):
        print(f"  group {i}: {len(row)} table(s)")

    route = []
    going_up = True  # direction of travel in Y within the current row
    for row in rows:
        # Sorts the row's tables by their Y position, in the current
        # direction of travel -- so they're flown in the real physical
        # order, not the order they appeared in the file.
        row_sorted = sorted(row, key=lambda p: p[2], reverse=not going_up)
        for name, x, y, yaw in row_sorted:
            p1, p2 = table_endpoints(x, y, yaw)
            # Enters from whichever endpoint matches the direction of travel.
            if going_up:
                entry, exit_ = (p1, p2) if p1[1] <= p2[1] else (p2, p1)
            else:
                entry, exit_ = (p1, p2) if p1[1] >= p2[1] else (p2, p1)
            route.append((f"{name}_in", *entry))
            route.append((f"{name}_out", *exit_))
        going_up = not going_up
    return route  # list of (name, x, y)


# ---------------------------------------------------------------------
# Converting Gazebo local coordinates (x, y in metres) to GPS
# ---------------------------------------------------------------------

def local_xy_to_latlon(x, y, home_lat, home_lon):
    """x = metres East of the origin, y = metres North of the origin
    (ENU convention) -> absolute (lat, lon)."""
    d_lat = y / 111320.0
    d_lon = x / (111320.0 * math.cos(math.radians(home_lat)))
    return home_lat + d_lat, home_lon + d_lon


def distance_m(lat1, lon1, lat2, lon2):
    d_lat = (lat2 - lat1) * 111320.0
    d_lon = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.sqrt(d_lat**2 + d_lon**2)


# ---------------------------------------------------------------------
# Flight functions
# ---------------------------------------------------------------------

def connect():
    print(f"Connecting to {CONNECTION_STRING} ...")
    master = mavutil.mavlink_connection(CONNECTION_STRING)
    master.wait_heartbeat()
    print(f"Connected (system {master.target_system}, component {master.target_component})")
    return master


def set_mode(master, mode_name):
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    while True:
        ack = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if ack is not None and mavutil.mode_string_v10(ack) == mode_name:
            print(f"Mode changed to {mode_name}")
            return
        if ack is None:
            print(f"Warning: change to {mode_name} not confirmed, continuing anyway")
            return


def arm(master):
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    master.motors_armed_wait()
    print("Motors armed")


def takeoff(master, altitude):
    print(f"Taking off to {altitude} m...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude,
    )
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=10)
        if msg is None:
            print("Warning: no altitude data, retrying...")
            continue
        current_alt = msg.relative_alt / 1000.0
        print(f"  current altitude: {current_alt:.1f} m")
        if current_alt >= altitude * 0.95:
            print("Cruise altitude reached")
            return


def goto(master, lat, lon, altitude):
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), altitude,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )


def wait_until_arrived(master, lat, lon, radius_m, timeout_s):
    start = time.time()
    while time.time() - start < timeout_s:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg is None:
            continue
        cur_lat = msg.lat / 1e7
        cur_lon = msg.lon / 1e7
        d = distance_m(cur_lat, cur_lon, lat, lon)
        if d <= radius_m:
            print(f"  waypoint reached (at {d:.1f} m)")
            return True
    print("  warning: timed out waiting, moving on to the next waypoint")
    return False


def return_to_launch(master):
    print("Returning to home (RTL)...")
    set_mode(master, "RTL")


# ---------------------------------------------------------------------
# Main program
# ---------------------------------------------------------------------

def main():
    positions = read_table_positions(WORLD_SDF_PATH)
    print(f"Tables read from the world: {len(positions)}")

    route_xy = build_boustrophedon_route(positions, ROW_TOLERANCE_M)
    route = [
        (name, *local_xy_to_latlon(x, y, HOME_LAT, HOME_LON))
        for name, x, y in route_xy
    ]
    print(f"Route generated: {len(route)} points ({len(route)//2} tables x 2 endpoints)\n")

    master = connect()

    set_mode(master, "GUIDED")
    arm(master)
    takeoff(master, ALTITUDE)

    for i, (name, lat, lon) in enumerate(route, start=1):
        print(f"\nWaypoint {i}/{len(route)}: {name} ({lat:.6f}, {lon:.6f})")
        goto(master, lat, lon, ALTITUDE)
        wait_until_arrived(master, lat, lon, ARRIVAL_RADIUS_M, WAYPOINT_TIMEOUT_S)

    print("\nSurvey complete -- every table visited.")
    return_to_launch(master)


if __name__ == "__main__":
    main()
