#!/usr/bin/env python3
"""ROS 2 driver for DDSM115 hub motors via the Waveshare DDSM Driver HAT (A).

Talks to the HAT's factory ESP32 firmware over its "ESP32-USB" Type-C port
(JSON lines, 115200 baud):
    {"T":11002,"type":115}                     select DDSM115 protocol
    {"T":11001,"time":500}                     heartbeat: HAT stops motors if no command for 500 ms
    {"T":10012,"id":1,"mode":2}                speed loop
    {"T":10010,"id":1,"cmd":<rpm>,"act":<a>}   speed command; HAT answers with
    {"T":20010,"id":1,"typ":115,"mode":2,"tor":..,"spd":<rpm>,"pos":<0..32767>,"err":..}

Subscribes /cmd_vel (geometry_msgs/Twist). With one wheel id only linear.x is
used; with two ids [left, right] it is a differential drive.
Publishes /wheel/joint_states (sensor_msgs/JointState: velocity rad/s, position rad).
"""
import json
import math
import sys
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

POS_FULL_TURN = 32768.0


class Ddsm115Node(Node):
    def __init__(self):
        super().__init__("ddsm115")
        p = self.declare_parameter
        port = p("port", "/dev/ddsm").value
        self.ids = list(p("wheel_ids", [1]).value)
        self.invert = list(p("invert", [False] * len(self.ids)).value)
        self.wheel_diameter = float(p("wheel_diameter", 0.10).value)  # m, measure your wheel
        self.track_width = float(p("track_width", 0.30).value)        # m, only for 2 wheels
        self.max_rpm = int(p("max_rpm", 60).value)                     # DDSM115 limit is 200
        self.act = int(p("accel_time", 3).value)                      # 0.1 ms per rpm
        self.cmd_timeout = float(p("cmd_timeout", 0.5).value)
        heartbeat_ms = int(p("heartbeat_ms", 500).value)
        rate = float(p("rate", 20.0).value)

        if len(self.invert) != len(self.ids):
            self.invert = [False] * len(self.ids)

        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, 115200, 0
        self.ser.dtr = False  # keep the auto-program circuit from resetting the ESP32
        self.ser.rts = False
        self.ser.open()
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        self._send({"T": 11002, "type": 115})
        self._send({"T": 11001, "time": heartbeat_ms})
        for wid in self.ids:
            self._send({"T": 10012, "id": wid, "mode": 2})
        self.get_logger().info(
            f"DDSM HAT on {port}: wheels {self.ids}, max {self.max_rpm} rpm, "
            f"heartbeat {heartbeat_ms} ms")

        self.target = {wid: 0 for wid in self.ids}
        self.last_cmd = None
        self.rx = ""
        self.state = {wid: (0.0, 0.0, 0.0) for wid in self.ids}  # pos rad, vel rad/s, torque raw
        self.last_err = {wid: 0 for wid in self.ids}

        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.pub_js = self.create_publisher(JointState, "wheel/joint_states", 10)
        self.create_timer(1.0 / rate, self._tick)

    # ------------------------------------------------------------- serial
    def _send(self, obj):
        self.ser.write((json.dumps(obj, separators=(",", ":")) + "\n").encode())
        time.sleep(0.005)  # firmware needs a few ms between bus commands

    def _read_feedback(self):
        try:
            self.rx += self.ser.read(self.ser.in_waiting or 0).decode(errors="ignore")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Serial error: {e}")
            raise SystemExit(1)
        *lines, self.rx = self.rx.split("\n")
        for line in lines:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                fb = json.loads(line)
            except json.JSONDecodeError:
                continue
            if fb.get("T") != 20010 or fb.get("id") not in self.state:
                continue
            wid = fb["id"]
            sign = -1.0 if self.invert[self.ids.index(wid)] else 1.0
            vel = sign * fb.get("spd", 0) * 2 * math.pi / 60.0
            pos = sign * fb.get("pos", 0) / POS_FULL_TURN * 2 * math.pi
            self.state[wid] = (pos, vel, float(fb.get("tor", 0)))
            err = fb.get("err", 0)
            if err and err != self.last_err[wid]:
                self.get_logger().error(f"wheel {wid} fault code {err}")
            self.last_err[wid] = err

    # ------------------------------------------------------------- control
    def _on_cmd(self, msg):
        self.last_cmd = self.get_clock().now()
        v, w = msg.linear.x, msg.angular.z
        if len(self.ids) >= 2:
            speeds = [v - w * self.track_width / 2, v + w * self.track_width / 2]
        else:
            speeds = [v]
        circumference = math.pi * self.wheel_diameter
        for i, wid in enumerate(self.ids):
            rpm = speeds[min(i, len(speeds) - 1)] / circumference * 60.0
            rpm = max(-self.max_rpm, min(self.max_rpm, rpm))
            self.target[wid] = int(round(-rpm if self.invert[i] else rpm))

    def _tick(self):
        stale = (self.last_cmd is None or
                 (self.get_clock().now() - self.last_cmd).nanoseconds / 1e9 > self.cmd_timeout)
        for wid in self.ids:
            rpm = 0 if stale else self.target[wid]
            self._send({"T": 10010, "id": wid, "cmd": rpm, "act": self.act})
        self._read_feedback()

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for wid in self.ids:
            pos, vel, tor = self.state[wid]
            js.name.append(f"wheel_{wid}")
            js.position.append(pos)
            js.velocity.append(vel)
            js.effort.append(tor)
        self.pub_js.publish(js)

    def stop(self):
        for wid in self.ids:
            self._send({"T": 10010, "id": wid, "cmd": 0, "act": self.act})


def main():
    rclpy.init()
    try:
        node = Ddsm115Node()
    except (serial.SerialException, OSError) as e:
        print(f"[ddsm115] cannot open HAT: {e}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.stop()
        node.ser.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
