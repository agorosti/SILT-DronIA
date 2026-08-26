#!/usr/bin/env python3
"""Teleoperación con mando para el dron de inspección.

Lee sensor_msgs/Joy y maneja los canales RC de ArduPilot a través de
MAVLink, de modo que un mando USB pilota la aeronave simulada exactamente
como un transmisor pilotaría la física.

Por qué no MAVROS
------------------
MAVROS es la respuesta habitual y funciona, pero es una dependencia grande
para añadir por lo que al final son cuatro números y un heartbeat. Hablar
directamente con SITL usando pymavlink mantiene los requisitos en tiempo de
ejecución en `joy` (ya incluido en una instalación de escritorio estándar
de ROS 2) más pymavlink, y elimina todo un paquete de la lista de cosas que
pueden desincronizarse en versión en la máquina del cliente.

El mapeo de canales sigue los valores por defecto de RC de ArduPilot:

    CH1 roll     CH2 pitch     CH3 throttle     CH4 yaw

Las asignaciones de eje por defecto siguen la disposición Modo 2 que un
mando estilo Xbox/PlayStation presenta a través del driver `joy`: el stick
izquierdo es throttle/yaw, el stick derecho es pitch/roll. Cada índice es
un parámetro de ROS, porque los mandos varían y el cliente no debería tener
que editar el código fuente para volar.

    ros2 run solar_farm_gz teleop_joy
    ros2 run solar_farm_gz teleop_joy --ros-args -p master:=tcp:127.0.0.1:5760

Los modos de vuelo están en botones en lugar de en un interruptor: un mando
no tiene un interruptor de modo de tres posiciones, y
LOITER/ALT_HOLD/STABILIZE cubre todo lo que necesita un vuelo de
inspección.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover - dependency guard
    raise SystemExit(
        "teleop_joy needs pymavlink:\n"
        "    pip install --user pymavlink\n"
        "  or: sudo apt install python3-pymavlink") from exc

# ArduPilot lee el RC como microsegundos PWM. 1500 es el stick centrado; el
# throttle es la excepción, donde el centro significa "mantener la
# altitud actual" en ALT_HOLD y LOITER pero significa media potencia en
# STABILIZE.
PWM_MIN, PWM_MID, PWM_MAX = 1000, 1500, 2000


class TeleopJoy(Node):

    def __init__(self):
        super().__init__('teleop_joy')

        self.declare_parameter('master', 'tcp:127.0.0.1:5760')
        self.declare_parameter('sysid_mygcs', 255)
        self.declare_parameter('axis_roll', 3)
        self.declare_parameter('axis_pitch', 4)
        self.declare_parameter('axis_throttle', 1)
        self.declare_parameter('axis_yaw', 0)
        self.declare_parameter('button_arm', 7)
        self.declare_parameter('button_disarm', 6)
        self.declare_parameter('button_loiter', 0)
        self.declare_parameter('button_althold', 1)
        self.declare_parameter('button_stabilize', 2)
        self.declare_parameter('button_rtl', 3)
        # Los sticks tienen ruido en reposo; sin una zona muerta el aparato se desplaza solo.
        self.declare_parameter('deadzone', 0.06)
        self.declare_parameter('expo', 0.35)

        master = self.get_parameter('master').value
        self.get_logger().info(f'connecting to {master} ...')
        # source_system DEBE coincidir con el SYSID_MYGCS de ArduPilot
        # (255 por defecto). Los overrides de cualquier otro id de sistema
        # se aceptan en el enlace y luego se descartan silenciosamente:
        # RC_CHANNELS sigue reportando las entradas sin modificar y la
        # aeronave simplemente no responde, sin ningún error en ningún
        # sitio que explique por qué.
        self.mav = mavutil.mavlink_connection(
            master, source_system=self.get_parameter('sysid_mygcs').value)
        self.mav.wait_heartbeat()
        self.get_logger().info(
            f'link up: system {self.mav.target_system}')

        self._prev_buttons = []
        self.create_subscription(Joy, 'joy', self.on_joy, 10)

        # ArduPilot vuelve a failsafe si dejan de llegar overrides, así que
        # el último comando se vuelve a publicar con un temporizador en
        # lugar de solo con eventos del joystick. Un mando que se mantiene
        # quieto no produce ningún mensaje en absoluto.
        self._last = [PWM_MID, PWM_MID, PWM_MIN, PWM_MID]
        self.create_timer(0.05, self.publish_rc)

    # --- funciones auxiliares -----------------------------------------------

    def _shape(self, v):
        """Zona muerta y luego expo cúbica, para que los pequeños movimientos del stick sean suaves."""
        dz = self.get_parameter('deadzone').value
        if abs(v) < dz:
            return 0.0
        v = (abs(v) - dz) / (1.0 - dz) * (1 if v > 0 else -1)
        e = self.get_parameter('expo').value
        return (1.0 - e) * v + e * v ** 3

    def _pwm(self, v, invert=False):
        v = self._shape(v)
        if invert:
            v = -v
        return int(max(PWM_MIN, min(PWM_MAX, PWM_MID + v * 500)))

    def _axis(self, msg, name):
        i = self.get_parameter(name).value
        return msg.axes[i] if 0 <= i < len(msg.axes) else 0.0

    def _pressed(self, msg, name):
        i = self.get_parameter(name).value
        if not (0 <= i < len(msg.buttons)):
            return False
        was = (self._prev_buttons[i]
               if i < len(self._prev_buttons) else 0)
        return msg.buttons[i] == 1 and was == 0

    # --- callbacks ------------------------------------------------------------

    def on_joy(self, msg):
        for name, mode in (('button_loiter', 'LOITER'),
                           ('button_althold', 'ALT_HOLD'),
                           ('button_stabilize', 'STABILIZE'),
                           ('button_rtl', 'RTL')):
            if self._pressed(msg, name):
                self.mav.set_mode_apm(mode)
                self.get_logger().info(f'mode -> {mode}')

        if self._pressed(msg, 'button_arm'):
            self.mav.arducopter_arm()
            self.get_logger().info('arm requested')
        if self._pressed(msg, 'button_disarm'):
            self.mav.arducopter_disarm()
            self.get_logger().info('disarm requested')

        # Pitch y throttle están invertidos: empujar un stick hacia adelante
        # da un valor de eje negativo desde el driver joy, pero significa
        # morro abajo y más potencia respectivamente.
        self._last = [
            self._pwm(self._axis(msg, 'axis_roll')),
            self._pwm(self._axis(msg, 'axis_pitch'), invert=True),
            self._pwm(self._axis(msg, 'axis_throttle'), invert=True),
            self._pwm(self._axis(msg, 'axis_yaw')),
        ]
        self._prev_buttons = list(msg.buttons)

    def publish_rc(self):
        r, p, t, y = self._last
        # 0 deja un canal sin tocar, así que los canales 5-8 se quedan con
        # lo que sea que el controlador de vuelo o una GCS les haya fijado.
        self.mav.mav.rc_channels_override_send(
            self.mav.target_system, self.mav.target_component,
            r, p, t, y, 0, 0, 0, 0)


def main(argv=None):
    rclpy.init(args=argv)
    node = TeleopJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
