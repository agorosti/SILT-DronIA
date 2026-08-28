# Running the simulation

*This document is a translation. The original is in [Spanish](INSTRUCTIONS.md).*

How to set up and operate the solar farm inspection drone.

Sections 1–4 are a one-time initial setup. After that, section 5 is all you
need.

---

## Contents

1. [Requirements](#1-requirements)
2. [Install prerequisites](#2-install-prerequisites)
3. [Install ArduPilot and the Gazebo bridge](#3-install-ardupilot-and-the-gazebo-bridge)
4. [Build the workspace](#4-build-the-workspace)
5. [Run it](#5-run-it)
6. [The camera feed](#6-the-camera-feed)
7. [Flying with a gamepad](#7-flying-with-a-gamepad)
8. [Recording a video](#8-recording-a-video)
9. [Generating new worlds](#9-generating-new-worlds)
10. [If something doesn't work](#10-if-something-doesnt-work)
11. [File map](#11-file-map)

---

## 1. Requirements

- **Ubuntu 24.04 LTS**, native (not WSL)
- **ROS 2 Jazzy**
- **Gazebo Harmonic** (`gz-sim` 8)
- A discrete GPU — your RTX 5070 is more than enough

On a laptop with both integrated and NVIDIA graphics, Gazebo will use the
integrated GPU unless told otherwise. The launch files handle this
automatically. To confirm, run `nvidia-smi` while the simulator is running —
you should see roughly one gigabyte in use.

---

## 2. Install prerequisites

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-joy \
    python3-numpy python3-scipy python3-pil python3-opencv \
    python3-pymavlink \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    cmake g++ git rapidjson-dev libopencv-dev
```

If `python3-pymavlink` isn't available on your mirror:

```bash
pip install --user --break-system-packages pymavlink MAVProxy
```

---

## 3. Install ArduPilot and the Gazebo bridge

These are independent open-source projects that live outside this
workspace.

### ArduPilot SITL

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot ~/ardupilot
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

The clone pulls down a lot of submodules — this is the slowest step. Check
that it worked:

```bash
ls ~/ardupilot/build/sitl/bin/arducopter
```

### The Gazebo bridge

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

The `cmake` output should say **Compiling against Gazebo Harmonic**. Check
that it worked:

```bash
ls ~/ardupilot_gazebo/build/libArduPilotPlugin.so
```

---

## 4. Build the workspace

```bash
cd ~/solar_farm_sim          # wherever you unpacked this
source /opt/ros/jazzy/setup.bash
colcon build --packages-select solar_farm_gz
source install/setup.bash
```

Add the last two lines to your `~/.bashrc` so you don't have to repeat them
every time.

A ready-to-fly 1000-panel world is already included — there's nothing to
generate.

---

## 5. Run it

Two terminals, both with ROS sourced.

### Terminal 1 — simulator

```bash
cd ~/solar_farm_sim
source install/setup.bash
ros2 launch solar_farm_gz inspection.launch.py
```

Gazebo opens with both views active: the free-orbit 3D view for flying with
a horizon reference, and the nadir camera feed docked alongside it.

Wait for the world to finish loading before continuing.

### Terminal 2 — flight controller

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --console --map
```

Give it 30 to 60 seconds to get a GPS fix and stabilize. Then, in the
MAVProxy console:

```
mode GUIDED
arm throttle
takeoff 8
```

The drone climbs to 8 m and holds. To land:

```
mode LAND
```

### Launch options

```bash
ros2 launch solar_farm_gz inspection.launch.py headless:=true
ros2 launch solar_farm_gz inspection.launch.py drone_x:=13.0 drone_y:=-14.0
```

| Argument | Default | Meaning |
|---|---|---|
| `world` | `solar_farm` | base name of the world file inside `worlds/` |
| `headless` | `false` | no GUI — faster |
| `bridge` | `true` | connects camera and clock to ROS 2 |
| `drone_x` `drone_y` `drone_z` | `-6 -6 0.13` | spawn position |
| `drone_yaw` | `0.0` | spawn heading, in radians |
| `ardupilot_gazebo` | `~/ardupilot_gazebo` | where you cloned the plugin |

---

## 6. The camera feed

With the simulator running:

```bash
ros2 topic list | grep x500
ros2 run rqt_image_view rqt_image_view /x500_rgb/nadir
```

| Topic | Type | Content |
|---|---|---|
| `/x500_rgb/nadir` | `sensor_msgs/Image` | 1920×1080 RGB, nadir |
| `/x500_rgb/camera_info` | `sensor_msgs/CameraInfo` | intrinsic parameters (fx = 1478.27) |
| `/clock` | `rosgraph_msgs/Clock` | simulation time |

Minimal subscriber:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Sub(Node):
    def __init__(self):
        super().__init__('detector')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/x500_rgb/nadir', self.cb, 10)

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # your YOLO inference goes here

rclpy.init(); rclpy.spin(Sub())
```

### Sampling rate for training data

At 8 m the camera covers a **10.4 m swath at 5.4 mm per pixel**, so a panel
takes up roughly 195 × 390 px.

At 1.5 m/s and 30 fps, consecutive frames overlap by roughly **99%** — a
dataset built straight from the raw video ends up as thousands of nearly
identical images, which inflates validation metrics without improving the
detector.

**Sample roughly one frame per second** to get around 80% overlap:

```python
if msg.header.stamp.sec != self._last_sec:
    self._last_sec = msg.header.stamp.sec
    process(frame)
```

The ground truth for each defect, with bounding boxes in YOLO format, lives
in `src/solar_farm_gz/worlds/defects.json`.

---

## 7. Flying with a gamepad

Connect a USB gamepad. Third terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_node
```

Fourth terminal:

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz teleop_joy
```

Fly in **LOITER** or **ALT_HOLD** rather than STABILIZE — position-hold
modes are much easier to fly for inspection work.

| Control | Default | Function |
|---|---|---|
| Left stick | axes 1, 0 | throttle, yaw |
| Right stick | axes 4, 3 | pitch, roll |
| Buttons 0–3 | | LOITER, ALT_HOLD, STABILIZE, RTL |
| Button 7 / 6 | | arm / disarm |

Axis numbering varies between gamepads. To find yours:

```bash
ros2 topic echo /joy
```

Move one stick at a time and note which `axes` entry changes, then pass the
correct indices — no need to edit any code:

```bash
ros2 run solar_farm_gz teleop_joy --ros-args \
    -p axis_throttle:=1 -p axis_yaw:=0 -p axis_pitch:=4 -p axis_roll:=3
```

Also available: `deadzone` (0.06 by default — raise it if the drone drifts
with the sticks centered) and `expo` (0.35 by default — raise it for a
smoother response near center).

---

## 8. Recording a video

Records a flight as a chase view with the nadir feed embedded. It starts
the simulator and flight controller on its own, so **close any simulator
that's already running first**.

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 46 --spawn "13.0,-14,0.13" \
    -o videos/my_flight.mp4
```

| Option | Default | Meaning |
|---|---|---|
| `--duration` | 40 | seconds recorded |
| `--alt` | 8.0 | altitude, in meters |
| `--speed` | 1.5 | cruise speed, m/s |
| `--spawn` | `3.25,-10,0.13` | starting position |
| `--width` `--height` | 1280 × 720 | output resolution |

It also supports `--thermal` (simulated thermal camera in the nadir inset)
and customizable overlay text (`--title-line1`, `--title-line2`,
`--status-label`, or the `.env` file).

**Recommended: use `--route`** instead of standalone `--spawn`/`--duration`
— it flies an absolute-GPS, table-by-table zigzag route read straight from
the world's own `.sdf`, rather than a straight-line cruise from a fixed
spawn point; it also avoids the non-deterministic spawn-heading issue (see
[docs/ROADMAP.md](docs/ROADMAP.md), in Spanish). The complete parameter
list, with `--route`, thermal/RGB, and title examples, is in
[RUNME-en.md](RUNME-en.md).

For hand-flown recordings, use any screen recorder while flying with the
gamepad.

---

## 9. Generating new worlds

The farm is procedural — a seed reproduces it exactly, so you can build as
many dataset variations as you want.

```bash
cd ~/solar_farm_sim/src/solar_farm_gz

# the world as shipped
python3 -m solar_farm_gz.generate_farm --panels 1000 --seed 11 -o worlds

# a different defect distribution
python3 -m solar_farm_gz.generate_farm --panels 1000 --seed 42 -o worlds

# more damage
python3 -m solar_farm_gz.generate_farm --panels 1000 --seed 7 \
    --clean-ratio 0.6 -o worlds

# lighter-weight world
python3 -m solar_farm_gz.generate_farm --panels 200 --seed 3 \
    --texture-scale 0.5 -o worlds
```

Rebuild afterward so the new world gets installed:

```bash
cd ~/solar_farm_sim && colcon build --packages-select solar_farm_gz
```

Other options: `--ground-style grass|earth`, `--no-infrastructure`,
`--fence-margin`, `--inverters`, `--sun-elevation`, `--sun-azimuth`. Full
list in the README.

If you edit a world file by hand, leave `<max_step_size>` at `0.001` — the
flight controller needs it.

---

## 10. If something doesn't work

### It won't arm

Check the MAVProxy console for the reason.

| Message | Fix |
|---|---|
| `Need Position Estimate` | Wait 30–60 s for GPS and the EKF to settle. |
| `Check frame class and type` | Use `sim_vehicle.py -f gazebo-iris` — it loads the frame parameters. |
| `Gyro 0 rate ... < loop rate*1.8` | The world's `<max_step_size>` must be `0.001`. |
| Nothing happens, it just refuses | It's still waiting on the EKF. Give it a full minute. |

### The gamepad does nothing

Check that `ros2 topic echo /joy` produces output. If not, the problem is
in the gamepad or the `joy` driver.

If `/joy` works but the drone ignores it, and you've changed `SYSID_MYGCS`
on the vehicle, pass the matching value:

```bash
ros2 run solar_farm_gz teleop_joy --ros-args -p sysid_mygcs:=<value>
```

### Connection refused, or two tools interfering with each other

The flight controller serves only one client per port, and MAVProxy
usually occupies 5760. Point other tools at 5762:

```bash
ros2 run solar_farm_gz teleop_joy --ros-args -p master:=tcp:127.0.0.1:5762
```

### It's running slow

- Check that the discrete GPU is being used: `nvidia-smi` while Gazebo is
  running.
- Try `headless:=true` — the GUI is half the cost.
- Regenerate with `--texture-scale 0.5`, or with fewer panels.

### The bridge won't build

- `gstreamer-1.0 not found` — install the two GStreamer packages from
  section 2.
- `Could not find gz-sim8` — source ROS before running `cmake`.

### The camera runs at ~23 fps, not 30

That's expected. The bottleneck is pulling each frame out of the renderer,
not drawing it. It makes no difference for flying, or for your dataset —
you should be sampling at ~1 Hz anyway (section 6). If you ever need the
full 30 frames, recording slower than real time gets you there.

---

## 11. File map

```
solar_farm_sim/
├── INSTRUCTIONS.md              this file
├── README.md                    full reference
├── RUNME.md                     quick-start guide: launching the simulation and generating videos
├── docs/
│   ├── MANUAL.md                full technical manual
│   ├── GETTING_STARTED.md       beginner's guide
│   └── ROADMAP.md               what's done, what might come next
├── videos/                      generated videos (RGB and thermal, demos and footage)
├── tools/                       dataset generation scripts (see tools/README.md)
├── yolo_dataset/                real dataset: data only (images/, labels/, data.yaml)
├── quicklook_dataset/           quick-look dataset: data only
└── src/solar_farm_gz/
    ├── launch/
    │   ├── inspection.launch.py world + drone + both views + ROS bridge
    │   └── solar_farm.launch.py world only
    ├── models/x500_rgb/         the aircraft
    ├── gui/inspection.config    dual-view layout
    ├── worlds/                  world, assets, defects.json
    └── solar_farm_gz/           generator, teleop, capture, recorder
```

---

## Help

If something doesn't work, send me:

1. Exactly what you ran.
2. The last 30 lines of the terminal that failed.
3. The MAVProxy console output, if it's a flight problem.
