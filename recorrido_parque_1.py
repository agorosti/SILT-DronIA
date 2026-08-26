#!/usr/bin/env python3
"""
Recorrido automático del parque fotovoltaico (Gazebo + ArduPilot SITL).

Qué hace, en orden:
    1. Se conecta al dron vía MAVLink (mismo vehículo que ves en MAVProxy).
    2. Cambia a modo GUIDED y arma los motores.
    3. Despega hasta la altitud indicada (por defecto, 8 m).
    4. Recorre en zigzag las filas de paneles, esperando en cada punto
       hasta llegar lo bastante cerca antes de pasar al siguiente.
    5. Al terminar, vuelve a home (RTL) automáticamente.

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
"""

import math
import time

from pymavlink import mavutil

# ---------------------------------------------------------------------
# CONFIGURACIÓN — ajusta aquí si hace falta
# ---------------------------------------------------------------------

# Cadena de conexión MAVLink. sim_vehicle.py expone por defecto una
# salida de GCS en este puerto; si no conecta, revisa la nota de arriba.
CONNECTION_STRING = "udp:127.0.0.1:14550"

# Altitud de vuelo, en metros (relativa al despegue) — la misma que
# ya usaste a mano con "takeoff 8".
ALTITUDE = 6

# Punto de partida (home), el mismo que ya localizaste en MAVProxy.
HOME_LAT = -35.363262
HOME_LON = 149.165237

# Dimensiones reales del mundo (Fase 1): longitud de una fila de
# mesas y separación entre filas.
ROW_LENGTH_M = 10.68
ROW_SPACING_M = 6.5

# Cuántas filas quieres recorrer en este vuelo.
NUM_ROWS = 10

# Radio de llegada: se considera "alcanzado" un punto cuando el dron
# está a menos de esta distancia (en metros).
ARRIVAL_RADIUS_M = 1.5

# Tiempo máximo de espera por punto antes de continuar de todos
# modos (evita que el script se quede colgado si algo va mal).
WAYPOINT_TIMEOUT_S = 30


# ---------------------------------------------------------------------
# Utilidades de conversión metros <-> grados
# ---------------------------------------------------------------------

def meters_to_latlon_offset(lat_ref, north_m, east_m):
    """Convierte un desplazamiento en metros (norte/este) a un
    incremento de latitud/longitud, tomando como referencia lat_ref."""
    delta_lat = north_m / 111320.0
    delta_lon = east_m / (111320.0 * math.cos(math.radians(lat_ref)))
    return delta_lat, delta_lon


def build_zigzag_route(home_lat, home_lon, num_rows, row_length_m, row_spacing_m):
    """Genera la lista de waypoints (lat, lon, alt) en zigzag, igual
    al patrón que ya recorriste a mano con comandos 'guided'."""
    route = []
    lat, lon = home_lat, home_lon
    going_north = True

    # Punto de partida
    route.append((lat, lon))

    for row in range(num_rows):
        # Recorre la fila actual (norte o sur, según toque)
        north_m = row_length_m if going_north else -row_length_m
        d_lat, _ = meters_to_latlon_offset(lat, north_m, 0)
        lat = lat + d_lat
        route.append((lat, lon))

        # Salta a la siguiente fila (hacia el este), si no es la última
        if row < num_rows - 1:
            _, d_lon = meters_to_latlon_offset(lat, 0, row_spacing_m)
            lon = lon + d_lon
            route.append((lat, lon))
            going_north = not going_north

    return route


def distance_m(lat1, lon1, lat2, lon2):
    """Distancia aproximada en metros entre dos puntos GPS (fórmula
    plana, suficiente para las distancias cortas de este recorrido)."""
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
    # Espera confirmación de cambio de modo
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
    # Espera hasta alcanzar ~95% de la altitud objetivo
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=10)
        if msg is None:
            print("Aviso: sin datos de altitud, reintentando...")
            continue
        current_alt = msg.relative_alt / 1000.0  # mm -> m
        print(f"  altitud actual: {current_alt:.1f} m")
        if current_alt >= altitude * 0.95:
            print("Altitud de crucero alcanzada")
            return


def goto(master, lat, lon, altitude):
    """Envía al dron a una posición GPS absoluta, en modo GUIDED."""
    master.mav.set_position_target_global_int_send(
        0,  # timestamp (ignorado por el receptor)
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # solo posición (ignora velocidad/aceleración)
        int(lat * 1e7), int(lon * 1e7), altitude,
        0, 0, 0,  # velocidad (ignorada)
        0, 0, 0,  # aceleración (ignorada)
        0, 0,     # yaw, yaw_rate (ignorados)
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
    route = build_zigzag_route(HOME_LAT, HOME_LON, NUM_ROWS, ROW_LENGTH_M, ROW_SPACING_M)
    print(f"Ruta generada: {len(route)} puntos (patrón en zigzag, {NUM_ROWS} filas)\n")

    master = connect()

    set_mode(master, "GUIDED")
    arm(master)
    takeoff(master, ALTITUDE)

    for i, (lat, lon) in enumerate(route, start=1):
        print(f"\nWaypoint {i}/{len(route)}: ({lat:.6f}, {lon:.6f})")
        goto(master, lat, lon, ALTITUDE)
        wait_until_arrived(master, lat, lon, ARRIVAL_RADIUS_M, WAYPOINT_TIMEOUT_S)

    print("\nRecorrido completo.")
    return_to_launch(master)


if __name__ == "__main__":
    main()
