# solar_farm_gz

*This document is a translation. The original is in [Spanish](README.md).*

Procedurally generated photovoltaic solar farm worlds for
**Gazebo Harmonic** and **ROS 2 Jazzy**, built for aerial inspection
research.

Open source project by **EuropeSIP Communications S.L.** — a company
specializing in Digital Transformation, Portals, and Artificial
Intelligence — to explore the possibilities of AI in a field as relevant
as image recognition and vision-based decision-making applied to
engineering. More information about EuropeSIP's AI solutions at
[europesip.com/es/europesip/soluciones/inteligencia-artificial](https://www.europesip.com/es/europesip/soluciones/inteligencia-artificial).

## Background and motivation

The concrete goal of this project: simulate the flight of an inspection
drone over a photovoltaic solar farm capable of automatically detecting
defects in the panels — dirt, cracks, delamination, bird droppings —
without manual intervention, using **YOLO** and **OpenCV** for image
recognition.

This project grows out of the difficulties involved in carrying out this
inspection with a real drone: building it from scratch, mounting the
necessary sensors — including a thermal camera, key to detecting hot spots
in damaged cells —, flying over real photovoltaic installations to capture
defect images, and from there creating, debugging, and maintaining a
dataset of real defects to train and test the AI model.

Each of these difficulties is, on its own, enough to derail the project:

- **Equipment cost.** A thermal camera with enough resolution to detect
  hot spots on a damaged panel isn't cheap, and it adds to the rest of the
  required hardware: frame, flight controller, RGB camera, video link,
  batteries, spares in case something breaks during a test flight... for a
  prototype, that budget stops being trivial very quickly.
- **Legal access to installations.** Flying a drone over a real
  photovoltaic plant, in compliance with current civil aviation
  regulations (authorizations, airspace restrictions, liability
  insurance), is neither a quick nor a simple process — and the vast
  majority of installations are private property, with their own access
  control.
- **Finding actually damaged panels.** Even after solving the two points
  above, you need an installation with real defects, varied and in
  sufficient quantity to train and evaluate a detector — and, naturally,
  no solar plant operator has broken or dirty panels sitting around
  waiting to be photographed for a demo.
- **Creating, debugging, and managing the real-defect dataset.** Even
  after solving the three points above, every captured image still has to
  be reviewed, labeled, and validated by hand, labeling errors have to be
  found and fixed, and the set has to be kept growing and organized as new
  cases accumulate — a manual, slow, error-prone process that has to be
  sustained for as long as the detector is being developed.

Given that scenario, the reasonable alternative became clear: if the drone
can't be brought to a damaged solar farm, the solar farm — with its
defects, and on demand — can be brought to the drone, inside a simulated
world. That's why the environment was simulated with **Gazebo** and
**ROS 2** — tools that allow high-fidelity simulation of the behavior of
industrial robots and drones — in order to prototype the end-to-end
solution before depending on real hardware.

**Why simulation with Gazebo and ROS 2 fills those gaps, and isn't a
second-rate substitute:**

- A procedurally generated solar farm has no access or ownership cost: it
  is generated with a single command, with as many defects, types, and
  severity levels as needed, and as many times as needed.
- There's no need to buy a real thermal camera to have a thermal channel:
  the generator already renders, alongside each defect, a temperature
  channel co-registered pixel-for-pixel with the visible damage, and
  `flight_video.py --thermal` uses it to simulate a real thermal camera on
  the recorded nadir feed (see [Thermal channel](#thermal-channel)),
  without reworking any assets.
- No flight authorization, insurance, or weather window is needed: Gazebo
  simulates the environment's physics, and the one piloting the drone
  within that physics is **ArduPilot SITL**, the same flight software that
  a real drone would run — so the observed flight behavior is
  representative of what the physical drone would show, not a camera
  animation.
- Reproducibility is total: every generated world carries its own exact
  ground truth for each defect, with no need for manual labeling, and it
  can be repeated with different seeds as many times as needed to build a
  robust, varied training dataset.

In short: simulation doesn't replace the real drone for lack of ambition —
it's the path that makes it possible to complete the project's goal —
training and validating a defect-detection system for an inspection
drone — without depending on equipment cost, flight bureaucracy, or the
availability of an already-damaged real installation. Besides the virtual
world in which to fly the drone, this prototype comes ready to generate
the annotated image dataset that then serves as a proof of concept for
training the AI model — the full pipeline, start to finish. The rest of
this document is the complete technical reference for that system.

---



The complete world — panel layout, terrain, lighting, and every surface
defect — is produced by a Python generator from a single random seed.
Re-running it with a different seed produces a different farm: different
defect types, in different locations, with different sizes and
orientations. Nothing is placed by hand, so a detector can be trained on
many world variations without repeated manual work.

Besides all the necessary material, this repository has different guides
to help you operate it. The [beginner's guide](docs/GETTING_STARTED.md)
(in Spanish) walks you through everything from installation instructions
to the first flight, step by step. The [RUNME](RUNME-en.md) is the quick
reference for launching the simulation and generating videos (demo or
dataset/YOLO). This README is the complete reference; the
[methodology](docs/METHODOLOGY.md) (in Spanish) covers the design
decisions, and the [roadmap](docs/ROADMAP.md) (in Spanish) covers the
optional improvements that remain open.

![Solar array, front view](docs/images/array_front.png)

---

## Contents

- [solar\_farm\_gz](#solar_farm_gz)
  - [Background and motivation](#background-and-motivation)
  - [Contents](#contents)
  - [What this gives you](#what-this-gives-you)
  - [Requirements](#requirements)
  - [Building](#building)
  - [Generating a world](#generating-a-world)
    - [Parameters](#parameters)
    - [Generating dataset variations](#generating-dataset-variations)
  - [Running it](#running-it)
  - [Flying the inspection drone](#flying-the-inspection-drone)
    - [Initial setup (one time)](#initial-setup-one-time)
    - [Launch](#launch)
    - [The aircraft](#the-aircraft)
    - [What the camera sees](#what-the-camera-sees)
    - [Joystick teleoperation](#joystick-teleoperation)
    - [Recording a flight](#recording-a-flight)
  - [Capturing flight images and video](#capturing-flight-images-and-video)
  - [Ground-truth annotations](#ground-truth-annotations)
  - [How it works](#how-it-works)
    - [Draw calls are the binding constraint](#draw-calls-are-the-binding-constraint)
    - [The atlas set](#the-atlas-set)
    - [Assets are a `model://` package](#assets-are-a-model-package)
  - [Defect model](#defect-model)
  - [Thermal channel](#thermal-channel)
  - [Performance](#performance)
  - [Layout reference](#layout-reference)
  - [Known limitations](#known-limitations)
  - [License](#license)

---

## What this gives you

| | |
|---|---|
| **Simulator** | Gazebo Harmonic (`gz-sim` 8), SDF 1.10 |
| **Middleware** | ROS 2 Jazzy, `ros_gz` bridge |
| **Panel count** | parametrizable; 200 fully validated, 1000 validated with fallback options |
| **Defect types** | dirt, bird droppings, glass cracks, EVA delamination |
| **Clean/damaged split** | fixed with `--clean-ratio`, achieved with single-module precision |
| **Ground truth** | `defects.json` with each defect's type and bounding box, ready to use directly in YOLO and AI models |
| **Thermal** | temperature channel rendered alongside each albedo atlas; simulated thermal camera available with `flight_video.py --thermal` |
| **Reproducibility** | a single seed fully determines the world |
| **Site layout** | perimeter fence, ring-shaped service road, inverter stations |
| **Aircraft** | Holybro X500 V2-class quadcopter, ArduPilot SITL, nadir-mounted RGB camera |
| **Teleoperation** | USB joystick to RC MAVLink, or recorded autonomous routes |

---

## Requirements

Ubuntu 24.04, with:

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz gz-harmonic \
                 python3-numpy python3-scipy python3-pil python3-opencv
```

The generator itself only needs NumPy, SciPy, and Pillow — it can run
without ROS if you only want the SDF and the assets.

Flying the drone additionally needs ArduPilot SITL, the `ardupilot_gazebo`
plugin, and `pymavlink`; see
[Flying the inspection drone](#flying-the-inspection-drone).

**Laptops with switchable graphics.** Gazebo doesn't request the discrete
GPU, so on a machine with both Intel and NVIDIA adapters it will render on
the integrated one and simply run slower — the only symptom is a
`libEGL ... dri2` line in the log. Both launch files detect an NVIDIA GPU
and automatically set the PRIME offload variables. To confirm it took
effect, `nvidia-smi` should show Gazebo using on the order of a gigabyte
instead of a few megabytes.

---

## Building

```bash
git clone <este-repo> solar_farm_sim
cd solar_farm_sim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Loading the workspace (`source`) sets `GZ_SIM_RESOURCE_PATH` to the
installed `worlds/` directory via an environment hook, which is how
`model://` URIs inside the world get resolved. Nothing needs to be
exported by hand.

A freshly made clone **contains no worlds at all** — `worlds/` is
generated, not versioned. Generate one before launching the simulation.

---

## Generating a world

```bash
# the 200-panel demo world that appears in the screenshots
ros2 run solar_farm_gz generate_farm -- \
    --panels 200 --tables-per-row 5 --variants 20 --seed 3 \
    -o src/solar_farm_gz/worlds

colcon build --symlink-install     # picks up the generated assets
```

Equivalently, without ROS:

```bash
cd src/solar_farm_gz
python3 -m solar_farm_gz.generate_farm --panels 200 --seed 3 -o worlds
```

### Parameters

**Farm**

| Flag | Default | Meaning |
|---|---|---|
| `--panels` | 1000 | total modules, rounded up to full tables |
| `--modules-per-table` | 10 | must match the atlas's 5x2 grid |
| `--tables-per-row` | 10 | tables per east-west row |
| `--row-pitch` | 6.5 | meters between row centerlines |
| `--table-gap` | 1.2 | meters between tables in the same row |
| `--jitter-m` | 0.04 | per-table position variation (topography tolerance) |
| `--jitter-deg` | 0.6 | per-table yaw variation |

**Defects**

| Flag | Default | Meaning |
|---|---|---|
| `--clean-ratio` | 0.80 | fraction of modules with no defects |
| `--variants` | 20 | distinct atlases in the set |
| `--w-dirt` | 0.45 | relative weight of dirt |
| `--w-bird-dropping` | 0.25 | relative weight of bird droppings |
| `--w-delamination` | 0.18 | relative weight of delamination |
| `--w-crack` | 0.12 | relative weight of cracks |

**Environment / output**

| Flag | Default | Meaning |
|---|---|---|
| `--sun-elevation` | 55.0 | degrees above the horizon |
| `--sun-azimuth` | 140.0 | degrees |
| `--ground-style` | `grass` | ground cover: `grass` or `earth` |
| `--no-shadows` | disabled | disables shadow casting |
| `--seed` | 0 | controls the layout **and** every defect |
| `--texture-scale` | 1.0 | reduces atlas resolution, e.g. `0.5` for machines with limited VRAM |
| `-o`, `--out` | `worlds` | output directory |

### Generating dataset variations

Each seed is an independent farm. The proportion of defects, their mix,
and their placement are independent parameters:

```bash
# more damage, different distribution
python3 -m solar_farm_gz.generate_farm --seed 7  --clean-ratio 0.60 -o worlds_a

# site dominated by dirt
python3 -m solar_farm_gz.generate_farm --seed 12 --w-dirt 0.8 --w-crack 0.05 -o worlds_b

# low-light evening pass
python3 -m solar_farm_gz.generate_farm --seed 21 --sun-elevation 18 -o worlds_c
```

---

## Running it

```bash
ros2 launch solar_farm_gz solar_farm.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `world` | `solar_farm` | base name of the world file inside `worlds/` |
| `headless` | `false` | server only, no GUI |
| `bridge` | `true` | starts the `ros_gz` `/clock` bridge |

The GUI is the expensive half of the renderer. On integrated graphics,
prefer `headless:=true` together with the capture tool below.

---

## Flying the inspection drone

Adds a quadcopter flown by a real ArduPilot flight stack, with a
downward-facing camera streamed to ROS 2.

### Initial setup (one time)

ArduPilot SITL and the Gazebo bridge plugin are external to this
repository:

```bash
# flight controller
git clone --recursive https://github.com/ArduPilot/ardupilot ~/ardupilot
cd ~/ardupilot && ./waf configure --board sitl && ./waf copter

# Gazebo <-> ArduPilot bridge
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir build && cd build && cmake .. && make -j$(nproc)
```

`ardupilot_gazebo` needs `libgstreamer1.0-dev` and
`libgstreamer-plugins-base1.0-dev` (used only by its GStreamer camera
plugin, which this project doesn't use, but its CMake requires them).
Teleoperation needs `pymavlink`.

### Launch

```bash
ros2 launch solar_farm_gz inspection.launch.py
```

This opens **both operator views at once**: the free-orbit 3D view, which
gives a horizon reference for manual flight, and the nadir camera feed
docked alongside it as the inspection view. Nothing needs to be opened by
hand.

| Argument | Default | Meaning |
|---|---|---|
| `world` | `solar_farm` | base name of the world file inside `worlds/` |
| `headless` | `false` | server only, no GUI |
| `bridge` | `true` | connects camera, `camera_info`, and `/clock` to ROS 2 |
| `drone_x` `drone_y` `drone_z` | `-6 -6 0.13` | spawn position |
| `drone_yaw` | `0.0` | spawn heading, in radians |
| `ardupilot_gazebo` | `~/ardupilot_gazebo` | checkout containing `build/` |

The flight controller runs as its own process, so it can be restarted
without having to close the world:

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --console --map
```

The camera reaches ROS 2 on `/x500_rgb/nadir`, with the intrinsic
parameters on `/x500_rgb/camera_info` — it can feed directly into an
OpenCV or YOLO pipeline.

### The aircraft

Modeled after a specific physical airframe rather than a generic
quadcopter, so that the simulated images are comparable to those from the
real drone.

| Property | Value |
|---|---|
| Frame | Holybro X500 V2, 500 mm motor-to-motor distance |
| Total weight | 1.30 kg |
| Motor layout | quad-X, ArduPilot channel order |
| Camera | Raspberry Pi Camera Module 3 (standard) |
| Field of view | 66° horizontal, 40.1° vertical at 16:9 |
| Resolution | 1920 × 1080 |
| Mount | fixed nadir (−90°) |

The camera is a genuine pinhole model, with the actual focal length,
rather than an approximation: the measured `fx` is 1478.27 px versus the
1478.27 px predicted from a 66° horizontal field of view at 1920 px.

### What the camera sees

The inspection flight geometry follows directly from the optics, and it's
worth knowing before building a training dataset:

| At 8 m altitude | Value |
|---|---|
| Ground footprint width | 10.4 m |
| Ground sample distance (GSD) | 5.4 mm/px |
| One module (1.05 × 2.10 m) | ≈ 195 × 390 px |
| Overlap between consecutive frames at 1.5 m/s, 30 fps | ≈ 99% |
| Passes to cover the 10-row array | ≈ 7 |

The footprint width exceeds the 6.5 m row spacing, so a single pass covers
a row with margin to spare. **Don't train on every frame.** At 30 fps,
consecutive frames overlap by around 99%, so a dataset built from the raw
video is thousands of near-duplicates, which inflates validation metrics
without improving the detector. Sampling at roughly 1 Hz gives around 80%
overlap — full coverage, with genuinely distinct frames.

### Joystick teleoperation

```bash
ros2 run joy joy_node
ros2 run solar_farm_gz teleop_joy
```

The sticks drive ArduPilot's RC channels over MAVLink, so a USB joystick
flies the simulated aircraft the same way a transmitter flies the real
one. The defaults follow the Mode 2 layout that an Xbox or PlayStation
controller exposes through `joy`; every axis and button is a ROS
parameter, so no source-code changes are needed to adapt to a different
controller.

| Control | Default | Channel |
|---|---|---|
| Left stick Y / X | axes 1, 0 | throttle, yaw |
| Right stick Y / X | axes 4, 3 | pitch, roll |
| Buttons 0–3 | — | LOITER, ALT_HOLD, STABILIZE, RTL |
| Buttons 7 / 6 | — | arm, disarm |

```bash
ros2 run solar_farm_gz teleop_joy --ros-args \
    -p axis_throttle:=1 -p deadzone:=0.08 -p master:=tcp:127.0.0.1:5760
```

MAVLink is spoken directly rather than through MAVROS: this amounts to
four numbers and a heartbeat, and skipping MAVROS avoids a large
dependency that would need to be kept version-synced with the flight
stack.

### Recording a flight

```bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 46 --spawn "13.0,-14,0.13" -o videos/inspection_flight.mp4
```

Flies an autonomous transect using the inspection parameters and records a
chase view with the live nadir feed embedded and a telemetry overlay. It's
a real flight under ArduPilot control, not an animated camera path — if
the controller wobbles, the recording shows it.

Add `--thermal` so the embedded nadir feed shows the simulated thermal
camera (false color, from the atlas's `thermal` channel) instead of
visible light; the outer chase view is unaffected. The overlay text (title
and status label) can be customized via `.env` or flags (`--title-line1`,
`--title-line2`, `--status-label`).

**Recommended: use `--route`** instead of standalone `--spawn`/`--duration`
— it flies an absolute-GPS, table-to-table zigzag route read from the
world's own `.sdf`, rather than a straight-line cruise from a fixed spawn
point; it also avoids the non-deterministic spawn-heading issue (see
[docs/ROADMAP.md](docs/ROADMAP.md) (in Spanish)). Full examples, with
`--route`, RGB, and thermal, in [RUNME.md](RUNME-en.md).

---

## Capturing flight images and video

Renders without opening a GUI, by injecting a camera into the world,
running the server in headless mode, and pulling frames from the
`gz-transport` image topic. It's the practical way to produce images on a
machine with no discrete GPU.

```bash
# single image: --pose is "x y z roll pitch yaw"
ros2 run solar_farm_gz capture -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --pose "42 8 15 0 0.36 3.0" -o array_front.png

# nadir inspection view at 11 m
ros2 run solar_farm_gz capture -- \
    --world .../solar_farm.sdf --pose "1.2 23.7 11 0 1.32 1.5708" -o nadir.png

# flight video: settling approach, descent, inspection transect
ros2 run solar_farm_gz capture -- \
    --world .../solar_farm.sdf --fly \
    --path "92,53,36,0,0.50,3.1416; 70,53,23,0,0.45,3.1416; \
            29,4,11,0,0.45,1.5708; 29,100,11,0,0.45,1.5708" \
    --frames 240 --fps 30 -o flythrough.mp4
```

`--path` is a semicolon-separated list of `x y z roll pitch yaw`
waypoints, interpolated at a constant pace by arc length so that speed
stays constant through the turns. Encoding is done with OpenCV, so ffmpeg
doesn't need to be installed. Add `--save-frames` to also keep the
individual PNGs.

The flight video loads the world **only once** and repositions the camera
between frames via the `set_pose` service, instead of relaunching for
every frame. On the 1000-panel world, that's the difference between about
a minute and about half an hour. After each move it discards `--settle`
frames (2 by default) before sampling, so a saved frame can never be one
that was rendered before the move finished.

![Nadir inspection view](docs/images/inspection_nadir.png)

*Nadir view at 11 m. A branching crack is visible in the lower-center area
and EVA delamination in the upper-left, both covering only part of their
module.*

![Inspection transect](docs/images/flythrough_transect.png)

*Frame from a flight video over the 1000-module farm, descending between
rows at 11 m.*

---

## Ground-truth annotations

Every generated defect is recorded in `worlds/defects.json`, so the
detector's labels come from the generator rather than manual labeling.

```jsonc
{
  "seed": 3,
  "modules": 200,
  "tables": 20,
  "clean_ratio_requested": 0.8,
  "clean_ratio_actual": 0.8,
  "defect_instances": 72,
  "defects_by_type": {"dirt": 32, "bird_dropping": 19,
                      "crack": 6, "delamination": 15},
  "module_size_m": [1.05, 2.10],
  "tilt_deg": 28.0,

  "atlases": {
    "pv_atlas_02": [
      {
        "module_index": 4,
        "atlas_cell": [4, 0],
        "clean": false,
        "defects": [
          { "type": "crack",
            "severity": 0.72,
            "bbox_uv_cxcywh": [0.51, 0.83, 0.34, 0.19] }
        ]
      }
    ]
  },

  "tables_placed": [
    { "index": 2, "pose_xyzyaw": [0.015, 23.72, 0.0, -0.004],
      "atlas": "pv_atlas_02" }
  ]
}
```

`bbox_uv_cxcywh` is the normalized center-x, center-y, width, and height
**within the module face** — YOLO's native format. To locate a defect in
the world, look up which atlas a given table uses in `tables_placed`, then
combine the table's pose with the module index and the module dimensions.

---

## How it works

### Draw calls are the binding constraint

Rendering cost here is dominated by the number of draw calls, not the
polygon count. Built the obvious way — one visual per module, with defect
overlays as separate geometry — a 1000-panel farm produces ~1500 visuals
and renders at **0.12x real time** on integrated graphics. A 2-minute
flight would take 16 minutes to simulate.

That's why a table is exactly **two meshes**:

- `pv_glass.obj` — the 10 modules merged into a single surface, each
  module UV-mapped to its own cell of a shared texture atlas
- `pv_rack.obj` — beams and posts, identical for every table, so Gazebo
  loads it once and instances it

That's 2 draw calls per table instead of ~15, and it's why defects live in
the texture rather than in the geometry.

### The atlas set

Each atlas is a 5x2 grid of 512x1024 module cells (2560x2048 total, at a
uniform resolution of 488 px/m). A table references a single atlas, so
`--variants` atlases cover `variants x 10` distinct module appearances.
Atlases are assigned to tables via a balanced shuffle rather than sampling
with replacement, so the realized fraction of damaged modules matches
`--clean-ratio` and no atlas ends up generated unused.

![Texture atlas](docs/images/atlas_example.png)

*An atlas: ten module cells. Dirt on the lower edge of one module,
droppings and delamination on others, the rest clean.*

### Assets are a `model://` package

The generated meshes and textures live in a Gazebo model package,
`worlds/solar_farm_assets/`, and are referenced as
`model://solar_farm_assets/...`.

This isn't cosmetic. `gz-sim` resolves a **relative** `<uri>` for a mesh
against `GZ_SIM_RESOURCE_PATH`, but relative `<albedo_map>`,
`<roughness_map>`, and `<normal_map>` paths inside `<pbr>` are *not*
resolved the same way — they're silently dropped, leaving every surface
untextured **with no error logged**. `model://` resolves consistently for
both cases and stays portable across machines.

Two related pitfalls, which also silently produce untextured geometry:

- An OBJ that includes its own `mtllib`/`usemtl` overrides the SDF's
  `<material>`, which would pin every table to a single atlas. The meshes
  are deliberately written without those overrides.
- A mesh with no vertex normals gets a default material from the loader,
  which also overrides the SDF material and renders as flat white. That's
  why the rack mesh writes explicit per-face normals.

---

## Defect model

Four types, each with randomized position, size, orientation, and
severity, each covering only part of a module face.

| Type | Appearance | Typical coverage | Physical placement bias |
|---|---|---|---|
| **Dirt** | brown/tan dust, multi-octave noise | ~30% | accumulates toward the bottom edge, following rain runoff |
| **Bird dropping** | opaque whitish stain with drip streaks | ~2% | uniform across the face |
| **Crack** | branching fracture, renders bright | ~30% of the box, sparse fill | radiates from a random impact point |
| **Delamination** | milky yellowish patch | ~11% | biased toward the module perimeter |

Severity (0.35–1.0) scales size and opacity. A damaged module carries 1 to
3 defect instances. Defects never cover the frame.

Since they're drawn onto the atlas rather than placed as geometry, a
defect costs nothing at render time — a farm damaged at 20% and one
damaged at 60% have exactly the same per-frame cost.

---

## Thermal channel

Each atlas is rendered into three co-registered channels:

```
pv_atlas_NN_albedo.png      visible appearance
pv_atlas_NN_roughness.png   PBR roughness (smooth glass, rough dirt)
pv_atlas_NN_thermal.png     surface temperature proxy
```

A defect that scatters light in the albedo also writes its heat signature
into the thermal channel **at the very same pixels**: cracks and
delamination read hot because a broken cell dissipates instead of
converting, dirt reads warm because it blocks light.

![Thermal channel](docs/images/atlas_thermal.png)

This channel is already in use: `flight_video.py --thermal` swaps the
albedo material of the recorded nadir feed for the corresponding thermal
channel and false-colors it, simulating a real thermal camera over the
same mesh, the same UVs, and the same defect positions — without
rebuilding any assets. See [MANUAL.md, section
3.4](docs/MANUAL-en.md#34-the-thermal-channel-how-the-thermal-camera-reuses-the-same-assets)
for the technical detail, and [RUNME.md](RUNME-en.md) for the commands.

---

## Performance

Measured on the development machine: Intel i7-10510U, **integrated Intel
UHD 620 graphics, no discrete GPU**, 8 GB RAM, Ubuntu 24.04, Gazebo
Harmonic 8.14, headless server with a 1280x720 camera at 30 Hz.

| World | Real-time factor | Peak RSS |
|---|---|---|
| 200 panels, merged geometry, full PBR | **1.00** (keeps up with real time) | ~0.6 GB |
| 1000 panels, merged geometry, `--texture-scale 0.5` | **1.00** (keeps up with real time) | ~0.6 GB |
| 1000 panels, merged geometry, full PBR, `--no-shadows` | **1.00** (keeps up with real time) | ~0.6 GB |
| 1000 panels, merged geometry, full PBR | 0.85 | ~0.6 GB |
| 1000 panels, one visual per module (naive, superseded) | 0.12 | ~1.0 GB |

The real-time factor is capped at 1.0 by the physics configuration, so
**1.00 means "keeping pace with real time," not "at its ceiling."**

The 1000-panel world loads in ~18 s and renders correctly. Three of its
four configurations keep up with real time with no trouble on hardware
with no discrete GPU whatsoever; the full-resolution case is the only one
that doesn't, and it's the most GPU-sensitive.

**Confirmed on the target hardware.** The 1000-panel world at full
resolution was independently measured at a **real-time factor of 1.00** on
an HP OMEN 16 (Core Ultra 7 255H, 32 GB, RTX 5070, native Ubuntu 24.04) —
a machine representative of the intended deployment. The 0.85 figure above
is therefore a floor from integrated graphics, not a ceiling. On any
discrete GPU the full-resolution world keeps up with real time and the
fallback options below aren't needed.

A 240-frame flight video of the 1000-panel world captures at
~1.5 frames/second from start to finish, including camera repositioning.

![1000-module farm](docs/images/farm_1000_overview.png)

*The complete 1000-module world: 100 tables, 20% damaged, 420 individual
defect instances, generated with `--seed 11`.*

Levers if you're GPU-constrained:

- `--texture-scale 0.5` cuts texture memory to a quarter. At 256x512 px
  per module this is still well above what a drone camera resolves at 8 m
  altitude (~67 px per module), so it costs nothing visually.
- `--variants` trades texture memory for visual repetition.
- `--no-shadows` removes the shadow-map cost.
- Prefer `headless:=true` with the capture tool over Gazebo's GUI.

---

## Layout reference

The module and table geometry follows a fixed-tilt industrial
installation.

| Quantity | Value |
|---|---|
| Module | 1.05 m x 2.10 m portrait, 6 x 24 half-cut cells |
| Table | 10 modules in a single row, 10.68 m long |
| Tilt | 28 degrees, low edge facing +X |
| Pivot height | 1.60 m |
| Row pitch | 6.5 m (default) |

World axes: +X is upslope (panels face +X), +Y runs along a table's
length, +Z is up.

---

## Known limitations

- **Full-resolution performance with 1000 panels is provisional.** The
  world generates, loads, and renders, and both fallback configurations
  keep up with real time, but the steady-state figure at full resolution
  hasn't been independently reproduced (see [Performance](#performance)).
- **Flat terrain.** The ground is a textured plane; there's no height
  field.
- **Cracks use a recursive random walk** and need `sys.setrecursionlimit`
  raised when generating very large worlds in a single process.

---

## License

MIT — see [LICENSE](LICENSE).
