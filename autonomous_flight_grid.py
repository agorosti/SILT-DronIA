#!/usr/bin/env python3
"""
Automatic survey flight over the PV farm (Gazebo + ArduPilot SITL).

What it does, in order:
    1. Connects to the drone over MAVLink (the same vehicle you see in
       MAVProxy).
    2. Switches to GUIDED mode and arms the motors.
    3. Takes off to the given altitude (8 m by default).
    4. Flies the panel rows in a zigzag, waiting at each point until
       close enough before moving to the next.
    5. Returns to home (RTL) automatically when done.

Requirements:
    - Gazebo + the solar farm world must already be running
      (Terminal 1: ros2 launch solar_farm_gz inspection.launch.py)
    - ArduPilot SITL must already be running
      (Terminal 2: Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map)
    - pymavlink installed (you already have it: pip install pymavlink)

How to run it:
    Leave Terminals 1 and 2 open and running. In a THIRD terminal:

        python3 autonomous_flight_grid.py

    If the script can't connect, explicitly add an extra MAVLink output
    when launching sim_vehicle.py (Terminal 2):

        Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \\
            --console --map --out=udp:127.0.0.1:14550
"""

import math
import time

from pymavlink import mavutil

# ---------------------------------------------------------------------
# CONFIGURATION -- adjust here if needed
# ---------------------------------------------------------------------

# MAVLink connection string. sim_vehicle.py exposes a GCS output on this
# port by default; if it doesn't connect, check the note above.
CONNECTION_STRING = "udp:127.0.0.1:14550"

# Flight altitude, in metres (relative to takeoff) -- the same one you
# already used by hand with "takeoff 8".
ALTITUDE = 6

# Starting point (home), the same one you already located in MAVProxy.
HOME_LAT = -35.363262
HOME_LON = 149.165237

# Real-world dimensions: length of a row of tables and spacing between rows.
ROW_LENGTH_M = 10.68
ROW_SPACING_M = 6.5

# How many rows to fly on this run.
NUM_ROWS = 10

# Arrival radius: a point is considered "reached" once the drone is
# within this distance (in metres).
ARRIVAL_RADIUS_M = 1.5

# Maximum time to wait per point before moving on regardless (keeps the
# script from hanging if something goes wrong).
WAYPOINT_TIMEOUT_S = 30


# ---------------------------------------------------------------------
# Metres <-> degrees conversion utilities
# ---------------------------------------------------------------------

def meters_to_latlon_offset(lat_ref, north_m, east_m):
    """Converts a north/east offset in metres into a latitude/longitude
    increment, referenced to lat_ref."""
    delta_lat = north_m / 111320.0
    delta_lon = east_m / (111320.0 * math.cos(math.radians(lat_ref)))
    return delta_lat, delta_lon


def build_zigzag_route(home_lat, home_lon, num_rows, row_length_m, row_spacing_m):
    """Generates the list of (lat, lon) waypoints in a zigzag, matching
    the pattern you already flew by hand with 'guided' commands."""
    route = []
    lat, lon = home_lat, home_lon
    going_north = True

    # Starting point
    route.append((lat, lon))

    for row in range(num_rows):
        # Flies the current row (north or south, whichever is next)
        north_m = row_length_m if going_north else -row_length_m
        d_lat, _ = meters_to_latlon_offset(lat, north_m, 0)
        lat = lat + d_lat
        route.append((lat, lon))

        # Steps over to the next row (eastward), unless this is the last one
        if row < num_rows - 1:
            _, d_lon = meters_to_latlon_offset(lat, 0, row_spacing_m)
            lon = lon + d_lon
            route.append((lat, lon))
            going_north = not going_north

    return route


def distance_m(lat1, lon1, lat2, lon2):
    """Approximate distance in metres between two GPS points (flat-earth
    formula, good enough for this route's short distances)."""
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
    # Waits for mode-change confirmation
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
    # Waits until ~95% of the target altitude is reached
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=10)
        if msg is None:
            print("Warning: no altitude data, retrying...")
            continue
        current_alt = msg.relative_alt / 1000.0  # mm -> m
        print(f"  current altitude: {current_alt:.1f} m")
        if current_alt >= altitude * 0.95:
            print("Cruise altitude reached")
            return


def goto(master, lat, lon, altitude):
    """Sends the drone to an absolute GPS position, in GUIDED mode."""
    master.mav.set_position_target_global_int_send(
        0,  # timestamp (ignored by the receiver)
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # position only (ignores velocity/acceleration)
        int(lat * 1e7), int(lon * 1e7), altitude,
        0, 0, 0,  # velocity (ignored)
        0, 0, 0,  # acceleration (ignored)
        0, 0,     # yaw, yaw_rate (ignored)
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
    route = build_zigzag_route(HOME_LAT, HOME_LON, NUM_ROWS, ROW_LENGTH_M, ROW_SPACING_M)
    print(f"Route generated: {len(route)} points (zigzag pattern, {NUM_ROWS} rows)\n")

    master = connect()

    set_mode(master, "GUIDED")
    arm(master)
    takeoff(master, ALTITUDE)

    for i, (lat, lon) in enumerate(route, start=1):
        print(f"\nWaypoint {i}/{len(route)}: ({lat:.6f}, {lon:.6f})")
        goto(master, lat, lon, ALTITUDE)
        wait_until_arrived(master, lat, lon, ARRIVAL_RADIUS_M, WAYPOINT_TIMEOUT_S)

    print("\nSurvey complete.")
    return_to_launch(master)


if __name__ == "__main__":
    main()
