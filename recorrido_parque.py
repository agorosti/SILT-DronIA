#!/usr/bin/env python3
"""
Recorrido automático del parque fotovoltaico (Gazebo + ArduPilot SITL).

Qué hace, en orden:
    1. Lee las posiciones reales de TODAS las mesas del mundo directamente
       desde el fichero .sdf (no asume una rejilla uniforme).
    2. Las agrupa en columnas y genera una ruta en zigzag que las visita
       todas, cubriendo el campo completo tenga la forma que tenga.
    3. Se conecta al dron vía MAVLink (mismo vehículo que ves en MAVProxy).
    4. Cambia a modo GUIDED y arma los motores.
    5. Despega hasta la altitud indicada (por defecto, 8 m).
    6. Recorre la ruta completa, esperando en cada punto hasta llegar lo
       bastante cerca antes de pasar al siguiente.
    7. Al terminar, vuelve a home (RTL) automáticamente.

Requisitos:
    - Gazebo + el mundo del parque solar ya deben estar lanzados
      (Terminal 1: ros2 launch solar_farm_gz inspection.launch.py)
    - ArduPilot SITL ya debe estar corriendo
      (Terminal 2: Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map)
    - pymavlink instalado (ya lo tienes: pip install pymavlink)

Cómo ejecutarlo:
    Deja las Terminales 1 y 2 abiertas y en marcha. En una TERCERA terminal:

        python3 recorrido_parque.py

    Si el script no consigue conectar, añade explícitamente una salida
    MAVLink extra al lanzar sim_vehicle.py (Terminal 2):

        Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \\
            --console --map --out=udp:127.0.0.1:14550

IMPORTANTE — supuesto de calibración:
    Este script asume que el origen del mundo de Gazebo (x=0, y=0) coincide
    con el punto HOME que usa ArduPilot SITL (HOME_LAT/HOME_LON de abajo),
    y que el eje X del mundo apunta al Este y el eje Y apunta al Norte
    (convención ENU estándar del plugin ArduPilot-Gazebo). Es la suposición
    más razonable sin más información, pero mientras el dron vuela,
    confirma visualmente (vista "Follow" en Gazebo) que las primeras
    mesas visitadas son realmente las más cercanas a home — si el
    recorrido empieza desviado, lo más probable es que haya que
    intercambiar north_m/east_m o invertir algún signo en to_latlon().
"""

import math
import re
import time

from pymavlink import mavutil

# ---------------------------------------------------------------------
# CONFIGURACIÓN — ajusta aquí si hace falta
# ---------------------------------------------------------------------

CONNECTION_STRING = "udp:127.0.0.1:14550"

# Ruta al mundo generado (para leer las posiciones reales de las mesas).
WORLD_SDF_PATH = "/home/andres/solar_farm_sim/src/solar_farm_gz/worlds/solar_farm.sdf"

# Altitud de vuelo, en metros (relativa al despegue).
ALTITUDE = 8

# Punto HOME que usa ArduPilot SITL (el mismo que ya localizaste en MAVProxy).
HOME_LAT = -35.363262
HOME_LON = 149.165237

# Tolerancia (en metros) para agrupar mesas en la misma fila física
# (mesas que comparten Y, dentro de este margen, se consideran de la
# misma fila).
ROW_TOLERANCE_M = 1.0

# Radio de llegada: se considera "alcanzado" un punto por debajo de esto (m).
ARRIVAL_RADIUS_M = 1.0

# Tiempo máximo de espera por waypoint antes de continuar (segundos).
WAYPOINT_TIMEOUT_S = 30


# ---------------------------------------------------------------------
# Lectura de las posiciones reales de las mesas desde el .sdf
# ---------------------------------------------------------------------

def read_table_positions(sdf_path):
    """Extrae (nombre, x, y, yaw) de cada mesa del mundo, en metros
    y radianes, en el sistema de coordenadas local de Gazebo."""
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


# Longitud de una mesa (10 módulos en fila) — ver metodología, Fase 1.
TABLE_LENGTH_M = 10.68


def table_endpoints(x, y, yaw):
    """Devuelve los dos extremos de la mesa (a lo largo de su eje
    de 10,68 m), teniendo en cuenta su orientación (yaw)."""
    half = TABLE_LENGTH_M / 2.0
    dx = -half * math.sin(yaw)
    dy = half * math.cos(yaw)
    return (x - dx, y - dy), (x + dx, y + dy)


def group_into_rows(positions, tolerance_m):
    """Agrupa las mesas en FILAS PARALELAS según su coordenada X
    (cada mesa es ya una fila completa de 10,68 m a lo largo de Y;
    filas distintas están separadas ~6,5 m en X). Devuelve una lista
    de filas (cada una, una lista con una sola mesa por ahora, pero
    agrupadas y ordenadas de menor a mayor X)."""
    sorted_pos = sorted(positions, key=lambda p: p[1])  # ordena por X
    rows = []
    for name, x, y, yaw in sorted_pos:
        if rows and abs(x - rows[-1][0][1]) <= tolerance_m:
            rows[-1].append((name, x, y, yaw))
        else:
            rows.append([(name, x, y, yaw)])
    return rows


def build_boustrophedon_route(positions, tolerance_m):
    """Por cada fila (grupo de mesas con X similar), las ordena por
    Y para recorrerlas en el orden físico correcto a lo largo de la
    fila, y por cada mesa visita sus dos extremos (entra por el que
    corresponda según el sentido de avance). Alterna el sentido de
    avance en cada fila para un recorrido continuo en zigzag."""
    rows = group_into_rows(positions, tolerance_m)
    print(f"Filas (mesas) detectadas, agrupadas por posición X: {len(rows)} grupos")
    for i, row in enumerate(rows):
        print(f"  grupo {i}: {len(row)} mesa(s)")

    route = []
    going_up = True  # sentido de avance en Y dentro de la fila actual
    for row in rows:
        # Ordena las mesas de la fila por su posición Y, en el
        # sentido de avance actual — así se recorren en el orden
        # físico real, no en el orden en que aparecían en el archivo.
        row_sorted = sorted(row, key=lambda p: p[2], reverse=not going_up)
        for name, x, y, yaw in row_sorted:
            p1, p2 = table_endpoints(x, y, yaw)
            # Entra por el extremo que corresponda al sentido de avance.
            if going_up:
                entry, exit_ = (p1, p2) if p1[1] <= p2[1] else (p2, p1)
            else:
                entry, exit_ = (p1, p2) if p1[1] >= p2[1] else (p2, p1)
            route.append((f"{name}_in", *entry))
            route.append((f"{name}_out", *exit_))
        going_up = not going_up
    return route  # lista de (name, x, y)


# ---------------------------------------------------------------------
# Conversión de coordenadas locales de Gazebo (x, y en metros) a GPS
# ---------------------------------------------------------------------

def local_xy_to_latlon(x, y, home_lat, home_lon):
    """x = metros al Este del origen, y = metros al Norte del origen
    (convención ENU) -> (lat, lon) absolutos."""
    d_lat = y / 111320.0
    d_lon = x / (111320.0 * math.cos(math.radians(home_lat)))
    return home_lat + d_lat, home_lon + d_lon


def distance_m(lat1, lon1, lat2, lon2):
    d_lat = (lat2 - lat1) * 111320.0
    d_lon = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.sqrt(d_lat**2 + d_lon**2)


# ---------------------------------------------------------------------
# Funciones de vuelo
# ---------------------------------------------------------------------

def connect():
    print(f"Conectando a {CONNECTION_STRING} ...")
    master = mavutil.mavlink_connection(CONNECTION_STRING)
    master.wait_heartbeat()
    print(f"Conectado (sistema {master.target_system}, componente {master.target_component})")
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
            print(f"Modo cambiado a {mode_name}")
            return
        if ack is None:
            print(f"Aviso: no se confirmó el cambio a {mode_name}, continuando igualmente")
            return


def arm(master):
    print("Armando motores...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    master.motors_armed_wait()
    print("Motores armados")


def takeoff(master, altitude):
    print(f"Despegando a {altitude} m...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude,
    )
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=10)
        if msg is None:
            print("Aviso: sin datos de altitud, reintentando...")
            continue
        current_alt = msg.relative_alt / 1000.0
        print(f"  altitud actual: {current_alt:.1f} m")
        if current_alt >= altitude * 0.95:
            print("Altitud de crucero alcanzada")
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
            print(f"  waypoint alcanzado (a {d:.1f} m)")
            return True
    print("  aviso: tiempo de espera agotado, continúo con el siguiente waypoint")
    return False


def return_to_launch(master):
    print("Volviendo a home (RTL)...")
    set_mode(master, "RTL")


# ---------------------------------------------------------------------
# Programa principal
# ---------------------------------------------------------------------

def main():
    positions = read_table_positions(WORLD_SDF_PATH)
    print(f"Mesas leídas del mundo: {len(positions)}")

    route_xy = build_boustrophedon_route(positions, ROW_TOLERANCE_M)
    route = [
        (name, *local_xy_to_latlon(x, y, HOME_LAT, HOME_LON))
        for name, x, y in route_xy
    ]
    print(f"Ruta generada: {len(route)} puntos ({len(route)//2} mesas x 2 extremos)\n")

    master = connect()

    set_mode(master, "GUIDED")
    arm(master)
    takeoff(master, ALTITUDE)

    for i, (name, lat, lon) in enumerate(route, start=1):
        print(f"\nWaypoint {i}/{len(route)}: {name} ({lat:.6f}, {lon:.6f})")
        goto(master, lat, lon, ALTITUDE)
        wait_until_arrived(master, lat, lon, ARRIVAL_RADIUS_M, WAYPOINT_TIMEOUT_S)

    print("\nRecorrido completo — todas las mesas visitadas.")
    return_to_launch(master)


if __name__ == "__main__":
    main()

