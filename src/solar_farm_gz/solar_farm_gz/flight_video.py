#!/usr/bin/env python3
"""Graba un vuelo de inspección: vista de seguimiento (chase) con la señal de nadir en directo incrustada.

Esto es un vuelo real, no una animación de cámara. ArduPilot SITL pilota la
aeronave en modo GUIDED con los parámetros de inspección que el cliente
realmente voló (8 m de altitud, crucero a 1.5 m/s), y ambas cámaras graban
lo que ocurre. Si el controlador se tambalea, el vídeo lo muestra.

    python3 -m solar_farm_gz.flight_video --world worlds/solar_farm.sdf \\
        --ardupilot ~/ardupilot --duration 40 -o visuals/inspection_flight.mp4

Tres detalles guían el diseño:

* **El dron está incrustado (inline), no incluido.** La cámara de
  seguimiento tiene que moverse con la aeronave, y reposicionar un modelo
  estático aparte mediante el servicio set_pose cuesta una ida y vuelta de
  subproceso por fotograma — demasiado lento a 20+ fps. Incrustar el modelo
  permite que la cámara de seguimiento cuelgue de base_link mediante una
  junta fija, así que sigue al dron gratis. El model.sdf entregado se deja
  intacto; la cámara extra solo existe en el mundo de captura temporal.

* **La aeronave se genera (spawn) con el morro alineado con las filas** y
  se pilota con velocidad en el sistema de referencia del cuerpo (body
  frame). Los ejes NED de ArduPilot y los ejes de mundo de Gazebo difieren
  en una rotación fácil de hacer mal; usar MAV_FRAME_BODY_NED evita por
  completo la conversión, y un transecto con el morro hacia adelante
  también es lo que una cámara de seguimiento quiere mirar.

* **Los fotogramas se transmiten al codificador.** Un vuelo de 40 s son
  ~900 fotogramas; almacenar en búfer RGB de 1920x1080 serían unos 5.6 GB,
  así que cada fotograma compuesto se escribe y se descarta.
"""

import argparse
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np

from . import capture

# Cámara de seguimiento: detrás y arriba, mirando hacia adelante y
# ligeramente hacia abajo. Los offsets están en el sistema de referencia
# del cuerpo de la aeronave, así que esto se mantiene durante los giros.
CHASE_BACK = 7.0
CHASE_UP = 2.6
CHASE_TOPIC = "/cine/chase"
NADIR_TOPIC = "/x500_rgb/nadir"

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


def build_capture_world(world_path, model_path, spawn, chase_wh, rate, outdir):
    """Mundo del parque + el dron incrustado con una cámara de seguimiento acoplada."""
    world = open(world_path).read()
    model = open(model_path).read()

    # Elimina la envoltura sdf propia del modelo para poder incrustar el <model>.
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
    """Localiza x500_rgb/model.sdf en un paquete instalado, o si no en el
    árbol fuente. Las dos disposiciones difieren: el Python instalado
    aterriza en lib/pythonX/site-packages, que no está cerca de share/."""
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


def fly(mav, alt, speed, duration, on_tick):
    """Despega, y luego mantiene un crucero hacia adelante en el sistema
    de referencia del cuerpo, llamando a on_tick durante todo el proceso."""
    from pymavlink import mavutil

    # Espera a tener una solución de posición antes de tocar el modo o
    # armar. Un SITL en frío necesita casi un minuto entero para que el
    # GPS y el EKF se estabilicen, y armar antes de eso falla las
    # comprobaciones prearm en lugar de esperar educadamente.
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

    # Prearm todavía puede rechazar durante unos segundos después de que
    # aparezca el fix, así que la petición de armado se reintenta en lugar
    # de enviarse una sola vez.
    t0, seen = time.time(), set()
    while time.time() - t0 < 90 and not mav.motors_armed():
        mav.arducopter_arm()
        for _ in range(10):
            m = mav.recv_match(type=['HEARTBEAT', 'STATUSTEXT'],
                               blocking=True, timeout=1)
            # Muestra el motivo del rechazo. "PreArm: ..." es lo único que
            # dice por qué, y tragárselo convierte un ajuste de un
            # parámetro de una línea en una tarde entera.
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

    # Sube antes de grabar: los primeros segundos son un ascenso vertical
    # con nada más que césped debajo, lo que hace un mal plano de apertura.
    t0 = time.time()
    while time.time() - t0 < 45:
        m = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                           timeout=3)
        if m and m.relative_alt / 1000.0 >= alt * 0.95:
            break
    print(f"  at altitude ({time.time() - t0:.0f}s)", flush=True)

    # Máscara solo de velocidad, sistema de referencia del cuerpo. Se
    # reenvía continuamente: ArduPilot expira un setpoint guiado tras unos
    # segundos y se detendría en vuelo estacionario.
    mask = 0b0000111111000111
    t0 = time.time()
    last_cmd = 0.0
    hud = {"alt": alt, "spd": 0.0, "gsd": 0.0, "swath": 0.0}
    while time.time() - t0 < duration:
        now = time.time()
        if now - last_cmd > 0.25:
            mav.mav.set_position_target_local_ned_send(
                0, mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                mask, 0, 0, 0, speed, 0, 0, 0, 0, 0, 0, 0)
            last_cmd = now

        # No bloqueante: el bucle de grabación no debe detenerse esperando
        # telemetría, o el vídeo pierde fotogramas cada vez que un paquete
        # llega tarde.
        m = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if m:
            hud["alt"] = m.relative_alt / 1000.0
            hud["spd"] = math.hypot(m.vx, m.vy) / 100.0
            # Derivado de la geometría real de la cámara en lugar de fijo
            # en el código, así que la superposición se mantiene fiel si
            # la aeronave se desvía de la altitud.
            hud["swath"] = 2.0 * hud["alt"] * math.tan(1.151917 / 2.0)
            hud["gsd"] = hud["swath"] * 1000.0 / 1920.0
        on_tick(now - t0, hud)
        time.sleep(0.005)


_FONT = 0  # cv2.FONT_HERSHEY_SIMPLEX, evita importar cv2 en el ámbito del módulo


def _shadowed(img, text, org, scale, colour, thick=1):
    """Texto con una copia oscura desplazada debajo.

    El metraje es una mezcla de césped brillante y paneles casi negros, así
    que el texto blanco liso es ilegible sobre aproximadamente la mitad de
    cada fotograma."""
    import cv2
    # Las fuentes Hershey son solo ASCII y dibujan "?" silenciosamente para cualquier otra cosa.
    text = text.encode("ascii", "replace").decode("ascii")
    cv2.putText(img, text, (org[0] + 1, org[1] + 1), _FONT, scale,
                (0, 0, 0), thick + 1, cv2.LINE_AA)
    cv2.putText(img, text, org, _FONT, scale, colour, thick, cv2.LINE_AA)


def composite(chase, nadir, inset_w, hud=None, title=None):
    """Fotograma completo de vista de seguimiento, señal de nadir incrustada abajo a la derecha, HUD de telemetría."""
    import cv2
    out = chase.copy()
    h, w = out.shape[:2]
    ih = int(inset_w * nadir.shape[0] / nadir.shape[1])
    small = cv2.resize(nadir, (inset_w, ih), interpolation=cv2.INTER_AREA)

    m = 24
    x0, y0 = w - inset_w - m, h - ih - m
    cv2.rectangle(out, (x0 - 3, y0 - 3), (x0 + inset_w + 3, y0 + ih + 3),
                  (235, 235, 235), 3)
    out[y0:y0 + ih, x0:x0 + inset_w] = small
    _shadowed(out, "NADIR INSPECTION CAMERA  1920x1080  66 deg",
              (x0, y0 - 12), 0.52, (245, 245, 245))

    if hud:
        # Formato fijo para que los números no tiemblen horizontalmente de
        # fotograma a fotograma, lo cual se ve descuidado incluso cuando
        # los valores son correctos.
        line = (f"ALT {hud['alt']:5.1f} m    "
                f"GND SPD {hud['spd']:4.1f} m/s    "
                f"GSD {hud['gsd']:4.1f} mm/px    "
                f"SWATH {hud['swath']:4.1f} m")
        _shadowed(out, line, (m, h - 22), 0.58, (250, 250, 250))
        # Se omite mientras la banda del título está activa, ya que si no
        # quedaría encima de ella.
        if not title:
            _shadowed(out, "ArduPilot SITL - GUIDED", (m, 34), 0.55,
                      (200, 230, 255))

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
                   help="x,y,z; yaw is set to fly along the rows")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--inset", type=int, default=416)
    p.add_argument("--title-seconds", type=float, default=5.0,
                   help="how long the title band stays up (0 to disable)")
    p.add_argument("--fourcc", default="mp4v")
    p.add_argument("-o", "--out", default="inspection_flight.mp4")
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
    # Morro alineado con +y para que el transecto recorra la longitud de las filas.
    world_file = build_capture_world(
        a.world, model, (sx, sy, sz, math.pi / 2),
        (a.width, a.height), a.fps, tmp)
    wname = capture.world_name(open(world_file).read())

    env = capture.build_env(a.world)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
        a.plugin_path + ":" + env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""))
    # Los URI de modelo dentro del mundo del parque se resuelven contra el
    # directorio del mundo; el dron está incrustado, así que no necesita
    # su propia entrada de recurso.
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

    state = {"n": 0, "last": -1}

    # Solo ASCII: OpenCV dibuja con fuentes vectoriales Hershey, que no
    # tienen glifos más allá de ASCII y renderizan una raya larga o un
    # punto medio como "???".
    title = ("Solar PV Farm - Autonomous Inspection Flight",
             "Holybro X500 V2  |  Raspberry Pi Camera Module 3, nadir  |  "
             "1000 modules, 420 defects")

    def on_tick(elapsed, hud):
        # Se rige por el contador de fotogramas de la cámara de seguimiento
        # para que cada fotograma grabado sea un render genuinamente nuevo
        # y no el mismo muestreado dos veces.
        if chase_sink.count == state["last"] or chase_sink.img is None:
            return
        if nadir_sink.img is None:
            return
        state["last"] = chase_sink.count
        writer.write(cv2.cvtColor(
            composite(chase_sink.img, nadir_sink.img, a.inset, hud,
                      title if elapsed < a.title_seconds else None),
            cv2.COLOR_RGB2BGR))
        state["n"] += 1
        if state["n"] % 30 == 0:
            print(f"\r  {elapsed:5.1f}s  {state['n']} frames", end="",
                  flush=True)

    print(f"[3/4] flying: {a.alt} m, {a.speed} m/s, {a.duration}s", flush=True)
    try:
        fly(mav, a.alt, a.speed, a.duration, on_tick)
    finally:
        print("\n[4/4] finishing", flush=True)
        writer.release()
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
