#!/usr/bin/env python3
"""Pick a spawn point for flight_video.py: near the centre of the array (for
maximum clearance regardless of which way ArduPilot's EKF actually decides
to head -- see the module docstring history for why), but landing in an
actual GAP between two tables, not on a table's footprint.

Tables sit at oy = col * pitch and each occupies roughly +-table_span/2
around that centre -- table_span/pitch is ~0.9, so nearly every y in a row's
extent is *inside* some table's footprint except a ~1.2 m gap right at each
pitch boundary. The naive bounding-box centre (min+max)/2 lands exactly on a
table's own centre whenever the table count is odd (observed: this crashed
the aircraft into a table on spawn, and it sat there the whole recording,
never reaching altitude). Using the midpoint between the two tables
straddling the middle of the sorted list is safe for both odd and even
counts."""
import json
import sys


def pick_spawn(defects_path):
    d = json.load(open(defects_path))
    tables = d["tables_placed"]
    xs = [t["pose_xyzyaw"][0] for t in tables]
    rows = {}
    for t in tables:
        ox, oy = t["pose_xyzyaw"][0], t["pose_xyzyaw"][1]
        rows.setdefault(round(ox, 0), []).append(oy)

    keys = sorted(rows)
    row_x = keys[len(keys) // 2]
    ys = sorted(rows[row_x])
    n = len(ys)
    y_gap = (ys[n // 2 - 1] + ys[n // 2]) / 2.0 if n > 1 else ys[0]
    return row_x, y_gap, 0.13


if __name__ == "__main__":
    x, y, z = pick_spawn(sys.argv[1])
    print(f"{x:.3f},{y:.3f},{z:.3f}")
