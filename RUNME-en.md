# RUNME — launching the simulation and generating videos

*This document is a translation. The original is in [Spanish](RUNME.md).*

Quick, practical reference: how to get the simulation running in Gazebo,
and how to generate a video **separately**, without having to fly by hand —
whether for a demo/presentation or as footage to test a detection
pipeline (YOLO). It doesn't repeat what other documents already cover; it
just gathers the commands actually used day to day, with all their
parameters.

If this is the first time you're setting up the project on this machine
(installing ROS 2, Gazebo, ArduPilot, building it), start with
[INSTRUCTIONS-en.md](INSTRUCTIONS-en.md) (sections 1–4) or
[docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) (in Spanish). This
document assumes that's already done — it covers what you need from
there on.

---

## Contents

0. [Before anything else: environment variables](#0-before-anything-else-environment-variables)
1. [Launching the simulation](#1-launching-the-simulation)
   - [1.1 How much damage ratio to use](#11-how-much-damage-ratio-to-use)
2. [Generating a video separately](#2-generating-a-video-separately)
   - [2.1 `flight_video.py` — real, cinematic flight, for demos](#21-flight_videopy--real-cinematic-flight-for-demos)
   - [2.2 `capture.py --fly` — headless flythrough, no overlay, for YOLO](#22-capturepy---fly--headless-flythrough-no-overlay-for-yolo)
   - [2.3 Cheat sheet: thermal vs RGB, titles](#23-cheat-sheet-thermal-vs-rgb-titles)
3. [The YOLO training dataset is not a video](#3-the-yolo-training-dataset-is-not-a-video)
4. [If something fails](#4-if-something-fails)

---

## 0. Before anything else: environment variables

Every command in this document is `ros2 run ...` / `ros2 launch ...` —
and it **won't work in a new terminal** unless the ROS 2 environment and
the already-built package itself have been sourced first. If you see
`ros2: command not found` or `Package 'solar_farm_gz' not found`, this is
exactly why.

Every new terminal needs, in this order:

```bash
source /opt/ros/jazzy/setup.bash              # ROS 2 Jazzy environment
source ~/solar_farm_sim/install/setup.bash    # this package, once built
```

To avoid repeating this by hand in every terminal, add it once to your
`~/.bashrc`:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/solar_farm_sim/install/setup.bash' >> ~/.bashrc
```

(If you haven't built the package even once yet, do that first — see
[INSTRUCTIONS-en.md](INSTRUCTIONS-en.md) or
[docs/GETTING_STARTED.md, section 3](docs/GETTING_STARTED.md#3-compilarlo-una-vez).)

Optional — only if Gazebo runs slowly or the window shows up black,
typical of a laptop with hybrid Intel/NVIDIA graphics:

```bash
echo 'export __NV_PRIME_RENDER_OFFLOAD=1' >> ~/.bashrc
echo 'export __GLX_VENDOR_LIBRARY_NAME=nvidia' >> ~/.bashrc
```

The command blocks in this document still include
`source install/setup.bash` regardless, for clarity — with the lines
above already in your `~/.bashrc` that call becomes redundant but
harmless (running it again causes no error).

---

## 1. Launching the simulation

Two terminals, both with ROS sourced.

**Terminal 1 — simulator with the drone already spawned and both views
open:**

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 launch solar_farm_gz inspection.launch.py
```

Wait for the world to finish loading (~18 s with the 1000-panel world).

**Terminal 2 — flight controller (ArduPilot SITL):**

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --console --map
```

Give the GPS and EKF 30–60 s to settle. Then, in the MAVProxy console:

```
mode GUIDED
arm throttle
takeoff 8
```

To land: `mode LAND`. To fly with a remote controller instead of via
console, see
[INSTRUCTIONS-en.md, section 7](INSTRUCTIONS-en.md#7-flying-with-a-gamepad).

**Most commonly used launch options:**

| Argument | Default | Meaning |
|---|---|---|
| `world` | `solar_farm` | base name of the world file inside `worlds/` |
| `headless` | `false` | no graphical interface — faster, useful alongside section 2 |
| `drone_x` `drone_y` `drone_z` | `-6 -6 0.13` | spawn position |
| `drone_yaw` | `0.0` | spawn heading, in radians |

```bash
ros2 launch solar_farm_gz inspection.launch.py headless:=true
ros2 launch solar_farm_gz inspection.launch.py drone_x:=13.0 drone_y:=-14.0
```

If you just want to see the world without a drone: `ros2 launch
solar_farm_gz solar_farm.launch.py`.

### 1.1 How much damage ratio to use

The drone doesn't decide how many panels are damaged — that's fixed by
the world you generated with `generate_farm.py` (step 6 of
[docs/GETTING_STARTED.md](docs/GETTING_STARTED.md#6-crea-tus-propias-variaciones-de-dataset)),
via `--clean-ratio`. Everything you record afterward, with
`flight_video.py` or `capture.py --fly` (section 2), or everything you
collect for the YOLO dataset (section 3), only shows what's already
present in that world.

For a video or a dataset to *look like* the inspection of a real
installation, **don't overdo the damage**: a well-maintained installation
has few panels with visible issues at any given time. As a reasonable
reference, something around 85% of panels in good condition
(`--clean-ratio 0.85`) gives a credible result while still having enough
defects to be noticeable in the video. Save a much higher damage ratio
for detector testing or demos meant to show a variety of defects in a
short time — that's no longer meant to be realistic, and that's fine if
that's the goal.

Three reference points, from more to less realistic, with the same world
carried through both `generate_farm.py` and a sample video:

**Optimal — installation with almost no issues, well maintained:**

```bash
ros2 run solar_farm_gz generate_farm -- --panels 1000 --seed 101 \
    --clean-ratio 0.95 -o src/solar_farm_gz/worlds
colcon build --symlink-install && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 45 -o videos/optimas_rgb.mp4
```

**Good — the reference for a realistic demo or simulation (~85% clean):**

```bash
ros2 run solar_farm_gz generate_farm -- --panels 1000 --seed 42 \
    --clean-ratio 0.85 -o src/solar_farm_gz/worlds
colcon build --symlink-install && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 45 -o videos/buenas_rgb.mp4
```

**Disastrous — only for detector testing or a "look at everything it can
detect" demo; not meant to be realistic:**

```bash
ros2 run solar_farm_gz generate_farm -- --panels 1000 --seed 7 \
    --clean-ratio 0.45 -o src/solar_farm_gz/worlds
colcon build --symlink-install && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 45 -o videos/desastrosas_rgb.mp4
```

---

## 2. Generating a video separately

There are two tools for this, and each serves a different purpose — they
are not interchangeable:

| | `flight_video.py` | `capture.py --fly` |
|---|---|---|
| What it's for | demo/presentation video | raw footage for a pipeline (YOLO, detection review) |
| View | chase view + nadir inset in a corner | camera only, no compositing |
| Text overlay (title, status) | yes, customizable | no |
| Launches its own Gazebo/ArduPilot | yes (closes any running simulator first) | yes |
| Flight | real, under ArduPilot SITL | camera interpolated between waypoints (no actual "flight") |
| Supports `--thermal` | yes | yes |

### 2.1 `flight_video.py` — real, cinematic flight, for demos

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 46 --spawn "13.0,-14,0.13" \
    -o videos/mi_video.mp4
```

Flies a real transect with ArduPilot (it's not an animated camera) and
records the chase view with the nadir feed inset and a telemetry
overlay.

**Recommended: use `--route`.** By default (without `--route`), the
drone flies in a straight line from `--spawn` with a fixed cruise
heading — for the video to travel along the rows of tables, that heading
has to match the actual orientation of the rows in the specific world
you're recording, and that's not reliable from one world to another (see
[docs/ROADMAP.md](docs/ROADMAP.md) (in Spanish)). `--route` fixes the
problem at its root: it reads the actual tables from the world's `.sdf`,
builds a zigzag table-to-table route, and flies each leg by absolute GPS
position (the same strategy as `autonomous_flight.py`), so the spawn
heading stops mattering.

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --route --route-tolerance 1.0 --route-waypoint-timeout 25 \
    --duration 120 -o videos/mi_video.mp4 --nadir-out videos/mi_video_nadir.mp4
```

**Parameters:**

| Option | Default | Meaning |
|---|---|---|
| `--world` | *(required)* | path to the world's `.sdf` |
| `--model` | next to the package | `x500_rgb/model.sdf`; you normally don't need to touch this |
| `--ardupilot` | `~/ardupilot` | ArduPilot checkout |
| `--plugin-path` | `~/ardupilot_gazebo/build` | Gazebo↔ArduPilot bridge build |
| `--alt` | 8.0 | cruise altitude, meters |
| `--speed` | 1.5 | cruise speed, m/s |
| `--duration` | 40.0 | seconds of cruise recorded; with `--route`, if the route finishes early, the flight (and recording) end there instead of using up the time hovering |
| `--route` | disabled | flies table to table by absolute GPS position instead of a straight-line cruise — see the recommendation above. With `--route`, `--spawn` still sets the takeoff point but the heading stops mattering |
| `--route-tolerance` | 1.0 | meters of X tolerance for grouping tables into the same row (only with `--route`) |
| `--route-arrival-radius` | 1.5 | meters from a waypoint that count as "arrived" (only with `--route`) |
| `--route-waypoint-timeout` | 25.0 | maximum seconds at a waypoint before moving on to the next one regardless (only with `--route`) |
| `--spawn` | `3.25,-10,0.13` | starting position `x,y,z`; without `--route`, the cruise heading is set assuming it matches the orientation of the rows — that's not always the case (see recommendation above) |
| `--bob-amplitude` | 0.0 | meters of sinusoidal up-and-down bobbing; 0 disables it. Only applies without `--route` |
| `--bob-pitch` | 11.88 | meters of forward travel per bobbing cycle. Only applies without `--route` |
| `--thermal` | disabled | **the inset nadir feed uses the simulated thermal camera (false color) instead of RGB** — the chase view doesn't change |
| `--width` `--height` | 1280 × 720 | output resolution |
| `--fps` | 30 | frames per second |
| `--inset` | 416 | pixel size of the inset nadir box |
| `--title-seconds` | 5.0 | how long the title band is shown; 0 disables it |
| `--env-file` | `.env` | `KEY=VALUE` file to read text from when it's not passed via flag |
| `--title-line1` | *(from `.env` or default)* | title line 1; overrides `.env` |
| `--title-line2` | *(from `.env` or default)* | line 2; the world's actual module/defect count is appended automatically |
| `--status-label` | *(from `.env` or default)* | status label (corner) |
| `--fourcc` | `mp4v` | video codec |
| `-o`, `--out` | `inspection_flight.mp4` | output video path |
| `--nadir-out` | disabled | besides the composite video from `--out`, writes the raw nadir feed (native resolution, no inset box or HUD) to this path, recorded during the same flight — the resolution the detector trains on, useful for running inference without the sharpness loss from cropping the inset box |
| `--keep` | disabled | keeps the temporary capture world (useful for debugging) |

**Customize the title without touching anything in each command** — copy
`.env-sample` to `.env` (not version-controlled, see `.gitignore`) in
the project root and adjust the values:

```bash
# .env
FLIGHT_TITLE_LINE1=EuropeSIP Communications - AI-Powered Solar Inspection
FLIGHT_TITLE_LINE2=Self-Built Drone | Raspberry Pi Camera Module 3, nadir
FLIGHT_STATUS_LABEL=ArduPilot Simulation (SITL-GUIDED)
```

If a key is missing or the file doesn't exist, the default built into
the script is used. A flag (`--title-line1`, etc.) always takes priority
over `.env`.

**Examples:**

```bash
# standard RGB demo, title from .env
ros2 run solar_farm_gz flight_video -- --world worlds/solar_farm.sdf \
    --duration 45 -o videos/demo_rgb.mp4

# same shot, but with the simulated thermal camera
ros2 run solar_farm_gz flight_video -- --world worlds/solar_farm.sdf \
    --duration 45 --thermal -o videos/demo_thermal.mp4

# different title just for this shot, without touching .env
ros2 run solar_farm_gz flight_video -- --world worlds/solar_farm.sdf \
    --duration 30 \
    --title-line1 "Internal demo - detection team" \
    --title-line2 "Simulated thermal camera" \
    --status-label "TEST" \
    --thermal -o videos/demo_interna_thermal.mp4

# longer route, different starting point, no title band
ros2 run solar_farm_gz flight_video -- --world worlds/solar_farm.sdf \
    --duration 60 --spawn "3.25,-10,0.13" --title-seconds 0 \
    -o videos/recorrido_largo.mp4
```

### 2.2 `capture.py --fly` — headless flythrough, no overlay, for YOLO

When what's needed is clean footage (no chase-cam, no text overlay) to
run through a detector, or a specific camera path that doesn't depend on
how the controller flies:

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz capture -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --fly \
    --path "30,-10,8,0,1.5708,1.5708; 30,110,8,0,1.5708,1.5708" \
    --frames 300 --fps 30 --save-frames \
    -o videos/flythrough_para_yolo.mp4
```

**Parameters:**

| Option | Default | Meaning |
|---|---|---|
| `--world` | *(required)* | path to the world's `.sdf` |
| `--fly` | disabled | flythrough mode (otherwise, captures a single still image) |
| `--thermal` | disabled | the sensor reads the simulated thermal channel (false color) instead of RGB — works both with `--fly` and with a still image |
| `--path` | — | waypoints `x y z roll pitch yaw` separated by `;`, interpolated at a constant rate |
| `--pose` | `15 15 10 0 0.45 2.2` | single pose, for a still image only (without `--fly`) |
| `--frames` | 150 | total flythrough frames |
| `--fps` | 30 | frames per second of the output video |
| `--rate` | 10.0 | simulation-time Hz at which the camera sensor updates |
| `--settle` | 2 | frames discarded after each repositioning (avoids capturing mid-motion) |
| `--width` `--height` | 1280 × 720 | resolution |
| `--fov` | 1.05 | vertical field of view, radians |
| `--save-frames` | disabled | besides the video, saves each frame as a PNG in `--outdir` |
| `--outdir` | `frames` | folder for the PNGs if `--save-frames` |
| `-o`, `--out` | `flythrough.mp4` (or `preview.png` without `--fly`) | output path |
| `-v`, `--verbose` | disabled | more detail in the console |

**Example with `--thermal`** (same flythrough, thermal channel instead
of RGB):

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz capture -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --fly --thermal \
    --path "30,-10,8,0,1.5708,1.5708; 30,110,8,0,1.5708,1.5708" \
    --frames 300 --fps 30 \
    -o videos/flythrough_termico_para_yolo.mp4
```

`capture.py --thermal` swaps the world's material for its thermal
variant (the same mechanism as `flight_video.py --thermal`, section 2.1)
and applies the same calibrated false color — so footage from both tools
is directly comparable. Note that this has no relation to the real
drone's ROS 2 topic `/x500_rgb/nadir` during an ArduPilot flight — that
topic is always RGB; the thermal channel only exists in the world
rendered offline by `capture.py` and `flight_video.py`. If what you need
are **already-labeled thermal still images** for training, that's a
different tool — see section 3.

### 2.3 Cheat sheet: thermal vs RGB, titles

- **RGB (default):** don't pass `--thermal`.
- **Thermal:** add `--thermal` to `flight_video.py` (only affects the
  inset nadir box; the chase view stays RGB) or to `capture.py --fly`
  (thermal footage with no overlay, section 2.2).
- **Default title:** don't pass anything — it comes from `.env`, or the
  built-in text if `.env` doesn't exist.
- **Fixed title for all videos in this project:** edit `.env`.
- **Different title just for a one-off shot:** use `--title-line1`,
  `--title-line2`, `--status-label` in that command — doesn't touch
  `.env`.
- **No title:** `--title-seconds 0`.

---

## 3. The YOLO training dataset is not a video

Important, so as not to confuse this with the above: the training
dataset doesn't come from a recorded video. A video (section 2) is
footage for demos or for testing an already-trained detector against a
continuous shot; the dataset is still images + YOLO labels, and **there
are two ways to generate it in this project**, with different
approaches:

| | `tools/build_quicklook_dataset.py` | `tools/capture_dataset/capture_dataset.py` |
|---|---|---|
| Where the images come from | crops each module directly from the texture atlas | renders real camera shots, in near-nadir poses, over the world loaded in Gazebo |
| How the boxes are computed | already given in `defects.json`, normalized to the module — no projection | `tools/capture_dataset/projection.py` reconstructs the defect's 3D position and projects it with the real camera model (pinhole, same FOV as `x500_rgb`) |
| Framing/perspective | none — it's the module "head-on," with no real camera | the same lens (66° horizontal, 1920×1080) as the real drone's Raspberry Pi Camera Module 3, so a model trained here applies directly to the `/x500_rgb/nadir` topic |
| Thermal mode | no | yes — `--thermal` renders the world with thermal material (same as `flight_video.py --thermal`) and labels everything as a single `thermal_problem` class, because a thermal camera can't distinguish the cause of a hot spot |
| Resulting dataset in this project | `quicklook_dataset/` | `yolo_dataset/` |

The scripts that **generate** the datasets all live in
[`tools/`](tools/README.md) (in Spanish), outside `yolo_dataset/` and
`quicklook_dataset/` — those two folders contain only data (images,
labels, `data.yaml`), ready to upload to Colab, Roboflow, or wherever
needed, without dragging code along.

`capture_dataset.py` is the newer tool and the one that generates the
"real" dataset meant for the detector — `build_quicklook_dataset.py` is
still useful as a quick check, but since it doesn't go through a real
camera, it doesn't represent the viewpoint the drone actually inspects
from.

**The three scripts in `tools/capture_dataset/`:**

- **`capture_dataset.py`** — the one that actually generates the
  dataset: loads a generated world, fires `--n` shots at random poses
  (biased toward damaged tables), and writes each image alongside its
  YOLO `.txt` label file.
- **`projection.py`** — the pure geometry: reconstructs in 3D where each
  defect is (from the table's pose and the module index) and projects it
  to the 2D box the camera would see at that specific pose. Not run
  standalone; it's imported by `capture_dataset.py`.
- **`pick_spawn.py`** — a separate utility, for `flight_video.py`:
  computes a spawn point near the center of the farm that falls in an
  actual gap between two tables, so the drone doesn't spawn on top of a
  table and crash immediately.

**Example — generating more images for an existing site:**

```bash
cd ~/solar_farm_sim/src/solar_farm_gz
export PYTHONPATH="$PWD:$PYTHONPATH"

# RGB (4 classes: dirt, bird_dropping, crack, delamination)
python3 ../../tools/capture_dataset/capture_dataset.py \
    --world-dir /ruta/a/worlds/site_g --site site_g --n 40 --seed 42 \
    --images-out /ruta/a/salida/images --labels-out /ruta/a/salida/labels

# thermal (1 class: thermal_problem), same site
python3 ../../tools/capture_dataset/capture_dataset.py \
    --world-dir /ruta/a/worlds/site_g --site site_g --n 40 --seed 77 --thermal \
    --images-out /ruta/a/salida/images --labels-out /ruta/a/salida/labels
```

The full detail — current dataset composition (460 images, 5 sites,
RGB+thermal), why thermal uses a single class, how the projection was
validated, and how to train with Ultralytics — is in
[docs/YOLO_DATASET.md](docs/YOLO_DATASET.md) (in Spanish), which is more
specific than this document for anything related to the dataset itself.
Full script reference in [tools/README.md](tools/README.md).

The simpler method in `tools/build_quicklook_dataset.py` (direct atlas
cropping, no camera) is still documented in
[docs/MANUAL-en.md, section 10](docs/MANUAL-en.md#10-building-a-training-dataset).

---

## 4. If something fails

- The drone won't arm, the remote controller doesn't respond, the bridge
  won't build, or Gazebo runs slowly: full table of symptoms/solutions
  in
  [INSTRUCTIONS-en.md, section 10](INSTRUCTIONS-en.md#10-if-something-doesnt-work)
  and in
  [docs/MANUAL-en.md, section 13](docs/MANUAL-en.md#13-troubleshooting).
- `flight_video` or `capture` fail to start: make sure you've closed any
  other running Gazebo/ArduPilot first — both tools launch their own.
- Gray, untextured panels: the world wasn't recompiled after being
  generated — `colcon build --symlink-install` and reload it (`source`).
