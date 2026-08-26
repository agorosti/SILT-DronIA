# Ejecutar la simulación

Cómo configurar y operar el dron de inspección del parque solar.

Las secciones 1–4 son una configuración inicial, de una sola vez. Después de
eso, la sección 5 es todo lo que necesitas.

---

## Contenido

1. [Requisitos](#1-requisitos)
2. [Instalar prerrequisitos](#2-instalar-prerrequisitos)
3. [Instalar ArduPilot y el puente de Gazebo](#3-instalar-ardupilot-y-el-puente-de-gazebo)
4. [Compilar el workspace](#4-compilar-el-workspace)
5. [Ejecutarlo](#5-ejecutarlo)
6. [La señal de la cámara](#6-la-señal-de-la-cámara)
7. [Volar con un mando](#7-volar-con-un-mando)
8. [Grabar un vídeo](#8-grabar-un-vídeo)
9. [Generar nuevos mundos](#9-generar-nuevos-mundos)
10. [Si algo no funciona](#10-si-algo-no-funciona)
11. [Mapa de ficheros](#11-mapa-de-ficheros)

---

## 1. Requisitos

- **Ubuntu 24.04 LTS**, nativo (no WSL)
- **ROS 2 Jazzy**
- **Gazebo Harmonic** (`gz-sim` 8)
- Una GPU discreta — tu RTX 5070 sobra de sobra

En un portátil con gráficos integrados y NVIDIA a la vez, Gazebo usará la
integrada a menos que se le indique lo contrario. Los ficheros de lanzamiento
se encargan de esto automáticamente. Para confirmarlo, ejecuta `nvidia-smi`
mientras el simulador está en marcha — deberías ver aproximadamente un
gigabyte en uso.

---

## 2. Instalar prerrequisitos

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-joy \
    python3-numpy python3-scipy python3-pil python3-opencv \
    python3-pymavlink \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    cmake g++ git rapidjson-dev libopencv-dev
```

Si `python3-pymavlink` no está disponible en tu mirror:

```bash
pip install --user --break-system-packages pymavlink MAVProxy
```

---

## 3. Instalar ArduPilot y el puente de Gazebo

Son proyectos de código abierto independientes que viven fuera de este
workspace.

### ArduPilot SITL

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot ~/ardupilot
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

El clonado descarga muchos submódulos — este es el paso más lento.
Comprueba que ha funcionado:

```bash
ls ~/ardupilot/build/sitl/bin/arducopter
```

### El puente de Gazebo

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

La salida de `cmake` debería decir **Compiling against Gazebo Harmonic**.
Comprueba que ha funcionado:

```bash
ls ~/ardupilot_gazebo/build/libArduPilotPlugin.so
```

---

## 4. Compilar el workspace

```bash
cd ~/solar_farm_sim          # donde hayas descomprimido esto
source /opt/ros/jazzy/setup.bash
colcon build --packages-select solar_farm_gz
source install/setup.bash
```

Añade las dos últimas líneas a tu `~/.bashrc` para no repetirlas cada vez.

Ya se incluye un mundo de 1000 módulos listo para volar — no hay nada que
generar.

---

## 5. Ejecutarlo

Dos terminales, ambas con ROS cargado (`source`).

### Terminal 1 — simulador

```bash
cd ~/solar_farm_sim
source install/setup.bash
ros2 launch solar_farm_gz inspection.launch.py
```

Gazebo se abre con ambas vistas activas: la vista 3D de órbita libre para
volar con referencia de horizonte, y la señal de la cámara en nadir acoplada
al lado.

Espera a que el mundo termine de cargar antes de continuar.

### Terminal 2 — controlador de vuelo

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --console --map
```

Dale entre 30 y 60 segundos para obtener una fijación GPS y estabilizarse.
Luego, en la consola de MAVProxy:

```
mode GUIDED
arm throttle
takeoff 8
```

El dron sube a 8 m y se mantiene. Para bajar:

```
mode LAND
```

### Opciones de lanzamiento

```bash
ros2 launch solar_farm_gz inspection.launch.py headless:=true
ros2 launch solar_farm_gz inspection.launch.py drone_x:=13.0 drone_y:=-14.0
```

| Argumento | Valor por defecto | Significado |
|---|---|---|
| `world` | `solar_farm` | nombre base del fichero de mundo dentro de `worlds/` |
| `headless` | `false` | sin interfaz gráfica — más rápido |
| `bridge` | `true` | conecta cámara y reloj a ROS 2 |
| `drone_x` `drone_y` `drone_z` | `-6 -6 0.13` | posición de aparición (spawn) |
| `drone_yaw` | `0.0` | rumbo de aparición, en radianes |
| `ardupilot_gazebo` | `~/ardupilot_gazebo` | dónde clonaste el plugin |

---

## 6. La señal de la cámara

Con el simulador en marcha:

```bash
ros2 topic list | grep x500
ros2 run rqt_image_view rqt_image_view /x500_rgb/nadir
```

| Topic | Tipo | Contenido |
|---|---|---|
| `/x500_rgb/nadir` | `sensor_msgs/Image` | 1920×1080 RGB, nadir |
| `/x500_rgb/camera_info` | `sensor_msgs/CameraInfo` | parámetros intrínsecos (fx = 1478.27) |
| `/clock` | `rosgraph_msgs/Clock` | tiempo de simulación |

Suscriptor mínimo:

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
        # aquí va tu inferencia YOLO

rclpy.init(); rclpy.spin(Sub())
```

### Frecuencia de muestreo para datos de entrenamiento

A 8 m la cámara cubre una **franja de 10.4 m a 5.4 mm por píxel**, así que un
módulo ocupa aproximadamente 195 × 390 px.

A 1.5 m/s y 30 fps, los fotogramas consecutivos se solapan en torno al
**99%** — un dataset construido a partir del vídeo en bruto son miles de
imágenes casi idénticas, lo que infla las métricas de validación sin mejorar
el detector.

**Muestrea aproximadamente un fotograma por segundo** para obtener alrededor
de un 80% de solape:

```python
if msg.header.stamp.sec != self._last_sec:
    self._last_sec = msg.header.stamp.sec
    process(frame)
```

La referencia (ground truth) de cada defecto, con cajas delimitadoras en
formato YOLO, está en `src/solar_farm_gz/worlds/defects.json`.

---

## 7. Volar con un mando

Conecta un mando USB. Tercera terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_node
```

Cuarta terminal:

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz teleop_joy
```

Vuela en **LOITER** o **ALT_HOLD** en lugar de STABILIZE — los modos que
mantienen la posición son mucho más fáciles de volar para trabajo de
inspección.

| Control | Valor por defecto | Función |
|---|---|---|
| Stick izquierdo | ejes 1, 0 | acelerador (throttle), guiñada (yaw) |
| Stick derecho | ejes 4, 3 | cabeceo (pitch), alabeo (roll) |
| Botones 0–3 | | LOITER, ALT_HOLD, STABILIZE, RTL |
| Botón 7 / 6 | | armar / desarmar |

La numeración de los ejes varía entre mandos. Para averiguar la del tuyo:

```bash
ros2 topic echo /joy
```

Mueve un stick a la vez y anota qué entrada de `axes` cambia, luego pasa los
índices correctos — no hace falta editar código:

```bash
ros2 run solar_farm_gz teleop_joy --ros-args \
    -p axis_throttle:=1 -p axis_yaw:=0 -p axis_pitch:=4 -p axis_roll:=3
```

También disponibles: `deadzone` (0.06 por defecto — súbelo si el dron se
mueve solo con los sticks centrados) y `expo` (0.35 por defecto — súbelo
para una respuesta más suave cerca del centro).

---

## 8. Grabar un vídeo

Graba un vuelo como una vista de seguimiento (chase view) con la señal de
nadir incrustada. Arranca el simulador y el controlador de vuelo por sí
mismo, así que **cierra primero cualquier simulador en marcha**.

```bash
cd ~/solar_farm_sim && source install/setup.bash
ros2 run solar_farm_gz flight_video -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --duration 46 --spawn "13.0,-14,0.13" \
    -o visuals/my_flight.mp4
```

| Opción | Valor por defecto | Significado |
|---|---|---|
| `--duration` | 40 | segundos grabados |
| `--alt` | 8.0 | altitud, en metros |
| `--speed` | 1.5 | velocidad de crucero, m/s |
| `--spawn` | `3.25,-10,0.13` | posición inicial |
| `--width` `--height` | 1280 × 720 | resolución de salida |

Para grabaciones pilotadas a mano, usa cualquier grabador de pantalla
mientras vuelas con el mando.

---

## 9. Generar nuevos mundos

El parque es procedural — una semilla lo reproduce exactamente, así que
puedes construir tantas variaciones del dataset como quieras.

```bash
cd ~/solar_farm_sim/src/solar_farm_gz

# el mundo tal como se entrega
python3 -m solar_farm_gz.generate_farm --panels 1000 --seed 11 -o worlds

# una distribución de defectos distinta
python3 -m solar_farm_gz.generate_farm --panels 1000 --seed 42 -o worlds

# más daño
python3 -m solar_farm_gz.generate_farm --panels 1000 --seed 7 \
    --clean-ratio 0.6 -o worlds

# mundo más ligero
python3 -m solar_farm_gz.generate_farm --panels 200 --seed 3 \
    --texture-scale 0.5 -o worlds
```

Recompila después para que el nuevo mundo quede instalado:

```bash
cd ~/solar_farm_sim && colcon build --packages-select solar_farm_gz
```

Otras opciones: `--ground-style grass|earth`, `--no-infrastructure`,
`--fence-margin`, `--inverters`, `--sun-elevation`, `--sun-azimuth`. Lista
completa en el README.

Si editas un fichero de mundo a mano, deja `<max_step_size>` en `0.001` — el
controlador de vuelo lo necesita.

---

## 10. Si algo no funciona

### No arma

Revisa la consola de MAVProxy para ver el motivo.

| Mensaje | Solución |
|---|---|
| `Need Position Estimate` | Espera 30–60 s a que el GPS y el EKF se estabilicen. |
| `Check frame class and type` | Usa `sim_vehicle.py -f gazebo-iris` — carga los parámetros del chasis. |
| `Gyro 0 rate ... < loop rate*1.8` | El `<max_step_size>` del mundo debe ser `0.001`. |
| No pasa nada, simplemente se niega | Todavía está esperando al EKF. Dale un minuto entero. |

### El mando no hace nada

Comprueba que `ros2 topic echo /joy` produce salida. Si no, el problema está
en el mando o en el driver `joy`.

Si `/joy` funciona pero el dron lo ignora, y has cambiado `SYSID_MYGCS` en
el vehículo, pasa el valor correspondiente:

```bash
ros2 run solar_farm_gz teleop_joy --ros-args -p sysid_mygcs:=<valor>
```

### Conexión rechazada, o dos herramientas interfiriendo

El controlador de vuelo atiende a un solo cliente por puerto, y MAVProxy
normalmente ocupa el 5760. Apunta las demás herramientas al 5762:

```bash
ros2 run solar_farm_gz teleop_joy --ros-args -p master:=tcp:127.0.0.1:5762
```

### Va lento

- Comprueba que se está usando la GPU discreta: `nvidia-smi` mientras
  Gazebo está en marcha.
- Prueba `headless:=true` — la interfaz gráfica es la mitad cara.
- Regenera con `--texture-scale 0.5`, o con menos paneles.

### El puente no compila

- `gstreamer-1.0 not found` — instala los dos paquetes de GStreamer de la
  sección 2.
- `Could not find gz-sim8` — carga (`source`) ROS antes de ejecutar `cmake`.

### La cámara va a ~23 fps, no a 30

Es lo esperado. El límite está en sacar cada fotograma del renderizador, no
en dibujarlo. No supone ninguna diferencia para volar, ni para tu dataset —
de todas formas deberías estar muestreando a ~1 Hz (sección 6). Si alguna
vez necesitas los 30 fotogramas completos, grabar más lento que el tiempo
real te los da.

---

## 11. Mapa de ficheros

```
solar_farm_sim/
├── INSTRUCTIONS.md              este fichero
├── README.md                    referencia completa
├── docs/
│   ├── GETTING_STARTED.md       guía para principiantes
│   └── ROADMAP.md               qué está hecho, qué podría venir después
├── visuals/
│   └── inspection_flight.mp4    vuelo de ejemplo
└── src/solar_farm_gz/
    ├── launch/
    │   ├── inspection.launch.py mundo + dron + ambas vistas + puente ROS
    │   └── solar_farm.launch.py solo el mundo
    ├── models/x500_rgb/         la aeronave
    ├── gui/inspection.config    disposición de doble vista
    ├── worlds/                  mundo, recursos, defects.json
    └── solar_farm_gz/           generador, teleoperación, captura, grabador
```

---

## Ayuda

Si algo no funciona, envíame:

1. Qué ejecutaste, exactamente.
2. Las últimas 30 líneas de la terminal que falló.
3. La salida de la consola de MAVProxy, si es un problema de vuelo.
