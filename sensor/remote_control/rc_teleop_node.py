#!/usr/bin/env python3
"""HotRC DS600 receiver pulses (/rc/pulses from the ESP32) -> /cmd_vel_rc.

Channels are 0-based indexes into the /rc/pulses array [CH1, CH2, CH3]:
    steering_ch (default 0 = CH1, wheel)    -> angular.z
    throttle_ch (default 1 = CH2, trigger)  -> linear.x
    enable_ch   (default -1 = off)          -> if set, the channel must be above
                                               enable_threshold_us to drive

Publishes only while the sticks are deflected, then zeros for `release_hold`
seconds after they return to neutral (or the signal is lost / enable is off),
then goes silent. twist_mux gives /cmd_vel_rc priority over /cmd_vel_pc, so the
remote overrides the PC while in use and hands control back when released.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray


class RcTeleop(Node):
    def __init__(self):
        super().__init__("rc_teleop")
        p = self.declare_parameter
        self.throttle_ch = p("throttle_ch", 1).value
        self.steering_ch = p("steering_ch", 0).value
        self.enable_ch = p("enable_ch", -1).value
        self.enable_threshold = p("enable_threshold_us", 1700).value
        self.throttle_center = p("throttle_center_us", 1500).value
        self.steering_center = p("steering_center_us", 1500).value
        self.span = float(p("span_us", 500).value)        # center -> full deflection
        self.deadband = p("deadband_us", 40).value
        self.invert_throttle = p("invert_throttle", False).value
        self.invert_steering = p("invert_steering", False).value
        self.max_linear = float(p("max_linear", 0.3).value)   # m/s
        self.max_angular = float(p("max_angular", 1.5).value)  # rad/s
        self.timeout = float(p("timeout", 0.3).value)
        self.release_hold = float(p("release_hold", 0.3).value)

        self.pub = self.create_publisher(Twist, "cmd_vel_rc", 10)
        self.create_subscription(Int32MultiArray, "rc/pulses", self._on_pulses, qos_profile_sensor_data)
        self.create_timer(0.1, self._watchdog)
        self.last_rx = None
        self.released_at = None  # time of the last active command; None = idle (silent)

    def _axis(self, pulse, center, invert):
        if pulse == 0:
            return None
        x = pulse - center
        if abs(x) <= self.deadband:
            return 0.0
        # rescale so the output starts at 0 right after the deadband
        x = (abs(x) - self.deadband) / (self.span - self.deadband) * (1 if x > 0 else -1)
        x = max(-1.0, min(1.0, x))
        return -x if invert else x

    def _on_pulses(self, msg):
        self.last_rx = self.get_clock().now()
        d = list(msg.data)
        get = lambda ch: d[ch] if 0 <= ch < len(d) else 0

        thr = self._axis(get(self.throttle_ch), self.throttle_center, self.invert_throttle)
        steer = self._axis(get(self.steering_ch), self.steering_center, self.invert_steering)
        enabled = self.enable_ch < 0 or get(self.enable_ch) > self.enable_threshold

        if thr is None or steer is None or not enabled or (thr == 0.0 and steer == 0.0):
            self._release()
            return
        cmd = Twist()
        cmd.linear.x = thr * self.max_linear
        cmd.angular.z = steer * self.max_angular
        self.pub.publish(cmd)
        self.released_at = self.get_clock().now()  # active: restart the hold window

    def _watchdog(self):
        if self.last_rx is None:
            return
        age = (self.get_clock().now() - self.last_rx).nanoseconds / 1e9
        if age > self.timeout:
            if self.released_at is not None:
                self.get_logger().warn(f"/rc/pulses lost for {age:.1f}s, stopping")
                self.pub.publish(Twist())
                self.released_at = None

    def _release(self):
        """Send zeros for release_hold seconds after the last active command, then stay silent."""
        if self.released_at is None:
            return
        held = (self.get_clock().now() - self.released_at).nanoseconds / 1e9
        if held <= self.release_hold:
            self.pub.publish(Twist())
        else:
            self.released_at = None


def main():
    rclpy.init()
    node = RcTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.released_at is not None:
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
