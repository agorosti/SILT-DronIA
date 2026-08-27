#!/usr/bin/env python3
"""Gamepad teleoperation for the inspection drone.

Reads sensor_msgs/Joy and drives ArduPilot's RC channels over MAVLink, so a
USB gamepad pilots the simulated aircraft exactly the way a transmitter
would pilot the real thing.

Why not MAVROS
---------------
MAVROS is the usual answer and it works, but it's a big dependency to add
for what ultimately comes down to four numbers and a heartbeat. Talking
directly to SITL with pymavlink keeps the runtime requirements at `joy`
(already included in a standard ROS 2 desktop install) plus pymavlink, and
removes an entire package from the list of things that can drift out of
version sync on the client machine.

The channel mapping follows ArduPilot's default RC assignment:

    CH1 roll     CH2 pitch     CH3 throttle     CH4 yaw

The default axis assignments follow the Mode 2 layout an Xbox/PlayStation-
style gamepad exposes through the `joy` driver: the left stick is
throttle/yaw, the right stick is pitch/roll. Each index is a ROS parameter,
because gamepads vary and the client shouldn't have to edit source code
just to fly.

    ros2 run solar_farm_gz teleop_joy
    ros2 run solar_farm_gz teleop_joy --ros-args -p master:=tcp:127.0.0.1:5760

Flight modes are on buttons rather than a switch: a gamepad doesn't have a
three-position mode switch, and LOITER/ALT_HOLD/STABILIZE covers everything
an inspection flight needs.
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

# ArduPilot reads RC as PWM microseconds. 1500 is stick-centred; throttle is
# the exception, where centre means "hold current altitude" in ALT_HOLD and
# LOITER but means half power in STABILIZE.
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
        # The sticks are noisy at rest; without a deadzone the aircraft
        # drifts on its own.
        self.declare_parameter('deadzone', 0.06)
        self.declare_parameter('expo', 0.35)

        master = self.get_parameter('master').value
        self.get_logger().info(f'connecting to {master} ...')
        # source_system MUST match ArduPilot's SYSID_MYGCS (255 by default).
        # Overrides from any other system id are accepted on the link and
        # then silently dropped: RC_CHANNELS keeps reporting the unmodified
        # inputs and the aircraft simply doesn't respond, with no error
        # anywhere explaining why.
        self.mav = mavutil.mavlink_connection(
            master, source_system=self.get_parameter('sysid_mygcs').value)
        self.mav.wait_heartbeat()
        self.get_logger().info(
            f'link up: system {self.mav.target_system}')

        self._prev_buttons = []
        self.create_subscription(Joy, 'joy', self.on_joy, 10)

        # ArduPilot falls back to failsafe if overrides stop arriving, so
        # the last command is republished on a timer instead of only on
        # joystick events. A gamepad held still produces no messages at
        # all.
        self._last = [PWM_MID, PWM_MID, PWM_MIN, PWM_MID]
        self.create_timer(0.05, self.publish_rc)

    # --- helpers ----------------------------------------------------------

    def _shape(self, v):
        """Deadzone then cubic expo, so small stick movements stay smooth."""
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

    # --- callbacks ----------------------------------------------------------

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

        # Pitch and throttle are inverted: pushing a stick forward gives a
        # negative axis value from the joy driver, but means nose-down and
        # more power respectively.
        self._last = [
            self._pwm(self._axis(msg, 'axis_roll')),
            self._pwm(self._axis(msg, 'axis_pitch'), invert=True),
            self._pwm(self._axis(msg, 'axis_throttle'), invert=True),
            self._pwm(self._axis(msg, 'axis_yaw')),
        ]
        self._prev_buttons = list(msg.buttons)

    def publish_rc(self):
        r, p, t, y = self._last
        # 0 leaves a channel untouched, so channels 5-8 keep whatever the
        # flight controller or a GCS has set them to.
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
