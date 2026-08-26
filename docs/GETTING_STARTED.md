# Primeros pasos — guía para principiantes

Esta guía te lleva desde una máquina Ubuntu recién instalada hasta volar una
cámara por el parque solar, paso a paso. Se asume que te manejas con
soltura en una terminal pero que eres nuevo en Gazebo y ROS 2 — no se
necesita experiencia previa en simulación.

Si solo quieres la versión corta: [instalar](#1-instalar-el-simulador) →
[obtener el código](#2-obtener-el-código) → [compilar](#3-compilarlo-una-vez) →
[lanzar](#4-lanzar-el-mundo). Luego lee
[crear tus propias variaciones de dataset](#6-crea-tus-propias-variaciones-de-dataset),
que es la parte que importa para el trabajo de detección.

---

## Antes de empezar: qué sistema arrancar

Tu portátil tiene arranque dual entre Windows/WSL y Ubuntu 24.04 nativo.
**Usa Ubuntu nativo**, no WSL. El renderizador de Gazebo necesita acceso
directo a la GPU, y la capa gráfica de WSL es lenta y poco fiable para esto.
Ubuntu nativo habla directamente con tu RTX 5070 y todo lo de abajo
simplemente funciona.

**Comprueba primero tu driver de NVIDIA.** La RTX 5070 es una tarjeta
reciente y necesita un driver reciente (serie 570 o posterior):

```bash
nvidia-smi
```

Si eso imprime una tabla con tu GPU y una versión de driver ≥ 570, ya está
listo. Si el comando no existe o muestra un driver más antiguo:

```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

---

## 1. Instalar el simulador

Tres cosas: ROS 2 Jazzy, Gazebo Harmonic, y unas cuantas librerías de
Python.

**ROS 2 Jazzy** — sigue los pasos oficiales una vez:
<https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>. La
versión corta:

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

**Gazebo Harmonic, el puente de ROS, y las librerías de Python:**

```bash
sudo apt install -y gz-harmonic ros-jazzy-ros-gz \
  python3-numpy python3-scipy python3-pil python3-opencv \
  python3-colcon-common-extensions
```

Esa es toda la instalación, y solo se hace una vez.

---

## 2. Obtener el código

Si te he añadido como colaborador en el repositorio privado de GitHub:

```bash
cd ~
git clone git@github.com:drhafiz-ayaan/solar-farm-gz.git solar_farm_sim
cd solar_farm_sim
```

Si recibiste el proyecto como un zip en su lugar, descomprímelo y entra
(`cd`) en la carpeta.

---

## 3. Compilarlo una vez

Los proyectos de ROS 2 se "compilan" antes del primer uso. Desde la carpeta
del proyecto:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Esto tarda menos de un minuto e imprime `Finished <<< solar_farm_gz`. Solo
hace falta recompilar después de cambiar código o generar un mundo nuevo
(paso 6).

> **Cada terminal nueva** necesita dos comandos `source` antes de conocer el
> proyecto. Acostúmbrate a ello:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source install/setup.bash
> ```
> Para no tener que escribirlos cada vez, añade ambas líneas al final de tu
> `~/.bashrc`.

---

## 4. Lanzar el mundo

Tienes dos opciones.

**Vía rápida — usar el mundo ya preparado que te entregué.** Descomprime el
`prebuilt_world_1000.zip` que te envié dentro del paquete, y luego compila:

```bash
unzip prebuilt_world_1000.zip -d src/solar_farm_gz/
colcon build --symlink-install
```

**O genera el tuyo propio** (ver el
[paso 6](#6-crea-tus-propias-variaciones-de-dataset)).

En cualquiera de los dos casos, lánzalo:

```bash
source install/setup.bash
ros2 launch solar_farm_gz solar_farm.launch.py
```

Gazebo se abre mostrando el parque solar. Si se abre en pausa, pulsa el
botón **▶ play** de la esquina inferior izquierda para arrancar la
simulación.

> **Si Gazebo va lento o la ventana está en negro**, tu portátil
> probablemente está dibujando con los gráficos integrados de Intel en lugar
> de la RTX 5070. Fuerza la tarjeta NVIDIA:
> ```bash
> __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch solar_farm_gz solar_farm.launch.py
> ```
> Si eso lo soluciona, añade esas dos variables de forma permanente
> poniendo `export __NV_PRIME_RENDER_OFFLOAD=1` y
> `export __GLX_VENDOR_LIBRARY_NAME=nvidia` en tu `~/.bashrc`.

### Mover la cámara en Gazebo

| Acción | Control |
|---|---|
| Orbitar / mirar alrededor | clic izquierdo y arrastrar |
| Zoom | rueda del ratón |
| Desplazar (pan) | clic central y arrastrar (o Shift + arrastrar con el izquierdo) |
| Restablecer vista | clic derecho en un panel → *Move To* / *Follow* |

Para ver los defectos, orbita hacia abajo hasta unos 10 metros sobre una
mesa de paneles. Las grietas, la suciedad, los excrementos de aves y la
delaminación se vuelven claramente visibles a esa altura — es
aproximadamente la altitud a la que volaría un dron de inspección real.

---

## 5. Dónde está la referencia (ground truth) de los defectos

Cada defecto del mundo queda registrado, con su tipo y ubicación exacta, en:

```
src/solar_farm_gz/worlds/defects.json
```

Para cada módulo dañado obtienes la clase del defecto y una caja
delimitadora en las coordenadas propias del módulo, en el formato nativo de
YOLO (`centro-x, centro-y, ancho, alto`, todo normalizado entre 0 y 1). Como
los defectos se generan en lugar de fotografiarse, estas cajas son exactas
— **nunca etiquetas nada a mano**, y las etiquetas se mantienen perfectas
sin importar cuánto crezca tu dataset.

El [README principal](../README.md#anotaciones-de-referencia-ground-truth)
explica la estructura del JSON en detalle.

---

## 6. Crea tus propias variaciones de dataset

Esta es la parte pensada para tu trabajo. Un solo comando regenera todo el
parque con una distribución de defectos completamente distinta y
aleatorizada. Cambia la semilla y obtienes un parque diferente; cambia las
proporciones y los pesos y obtienes un tipo de emplazamiento diferente.

```bash
# un parque nuevo, defectos aleatorios distintos, mismo reparto 80/20
ros2 run solar_farm_gz generate_farm -- --panels 1000 --seed 42 \
    -o src/solar_farm_gz/worlds

# un emplazamiento más dañado (60% limpio en lugar de 80%)
ros2 run solar_farm_gz generate_farm -- --panels 1000 --seed 7 \
    --clean-ratio 0.60 -o src/solar_farm_gz/worlds

# un parque dominado por la suciedad, pocas grietas
ros2 run solar_farm_gz generate_farm -- --panels 1000 --seed 12 \
    --w-dirt 0.8 --w-crack 0.05 -o src/solar_farm_gz/worlds

# una pasada vespertina con sol bajo (sombras largas)
ros2 run solar_farm_gz generate_farm -- --panels 1000 --seed 21 \
    --sun-elevation 18 -o src/solar_farm_gz/worlds
```

Después de generar, recompila una vez para que Gazebo vea los recursos
nuevos, y luego lanza:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch solar_farm_gz solar_farm.launch.py
```

La generación tarda unos minutos (está dibujando cada textura de panel
desde cero). El mismo `--seed` siempre produce el parque idéntico, así que
tus datasets son reproducibles — anota la semilla que usaste y podrás
recrear cualquier mundo exactamente.

La lista completa de opciones está en el
[README](../README.md#generar-un-mundo). Las que más usarás:

| Opción | Qué hace |
|---|---|
| `--seed N` | elige un parque aleatorio distinto |
| `--panels N` | cuántos paneles (tu RTX 5070 maneja 1000+ sin problema) |
| `--clean-ratio 0.8` | fracción de paneles limpios (0.8 = 20% dañado) |
| `--w-dirt / --w-crack / ...` | frecuencia relativa de cada tipo de defecto |
| `--sun-elevation` / `--sun-azimuth` | iluminación según la hora del día |

---

## 7. Captura imágenes y vídeo tú mismo

No necesitas tener abierta la ventana de Gazebo para producir imágenes — el
proyecto puede renderizar directamente a ficheros. Esto es útil para
construir un dataset o un clip de demostración.

```bash
source install/setup.bash

# una única imagen desde una pose de cámara elegida (x y z roll pitch yaw)
ros2 run solar_farm_gz capture -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf \
    --pose "42 8 15 0 0.36 3.0" -o my_shot.png

# un vídeo de vuelo a lo largo de una ruta de waypoints
ros2 run solar_farm_gz capture -- \
    --world install/solar_farm_gz/share/solar_farm_gz/worlds/solar_farm.sdf --fly \
    --path "92,53,36,0,0.50,3.1416; 29,4,11,0,0.45,1.5708; 29,100,11,0,0.45,1.5708" \
    --frames 240 --fps 30 -o my_flythrough.mp4
```

El vídeo se codifica con OpenCV, así que no hay que instalar nada extra. Ver
el [README](../README.md#capturar-imágenes-y-vídeos-de-vuelo) para más
detalles.

---

## 8. Solución de problemas

| Síntoma | Solución |
|---|---|
| `ros2: command not found` | Olvidaste `source /opt/ros/jazzy/setup.bash` en esta terminal. |
| `Package 'solar_farm_gz' not found` | Olvidaste `source install/setup.bash`, o todavía no has compilado. |
| Los paneles están grises planos / sin textura | El mundo no se compiló después de generarlo. Ejecuta `colcon build --symlink-install` de nuevo, y vuelve a cargar (`source`). |
| Gazebo lento, ventana en negro, o FPS bajos | Fuerza la GPU NVIDIA — ver el recuadro del [paso 4](#4-lanzar-el-mundo). |
| `nvidia-smi` no existe o driver antiguo | `sudo ubuntu-drivers autoinstall && sudo reboot`. |
| Lanzado pero el mundo está vacío | Asegúrate de que existe un mundo en `src/solar_farm_gz/worlds/` (descomprime el prefabricado, o genera uno). |

Si surge algo más, envíame el comando exacto que ejecutaste y el texto
completo del error y lo solucionaré.

---

## Lo que tienes, en una línea

Un parque solar completo y fotorrealista en Gazebo por el que puedes volar
una cámara, que puedes regenerar con defectos aleatorios distintos cuando
quieras, y que te entrega etiquetas de detección perfectas para cada
defecto — sin etiquetado manual.
