#!/usr/bin/env python3
"""ROS 2 driver for the Yahboom 10-axis IMU (WitMotion protocol), reusing ImuParser from imu_viewer.

Topics:
    /imu/data         sensor_msgs/Imu              orientation, angular velocity (rad/s), accel (m/s^2)
    /imu/rpy          geometry_msgs/Vector3Stamped roll, pitch, yaw (deg) -- handy for plotting
    /imu/pose         geometry_msgs/PoseStamped    IMU orientation, shown as axes in Foxglove 3D
    /imu/mag_raw      geometry_msgs/Vector3Stamped magnetometer (raw LSB)
    /imu/temperature  sensor_msgs/Temperature
    /imu/pressure     sensor_msgs/FluidPressure    (Pa)
    /imu/altitude     std_msgs/Float64             (m)

Parameters: port, baud (0 = auto-detect), frame_id, parent_frame, publish_tf
"""
import math
import sys
import time

import rclpy
import serial
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu, Temperature
from std_msgs.msg import Float64
from tf2_ros import StaticTransformBroadcaster

from imu_viewer import BAUDS, G, ImuParser

DEG = math.pi / 180.0


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,   # w
            sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy)   # z


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        port = self.declare_parameter("port", "/dev/imu").value
        baud = self.declare_parameter("baud", 0).value
        self.frame_id = self.declare_parameter("frame_id", "imu_link").value
        parent = self.declare_parameter("parent_frame", "base_link").value

        self.ser = self._open(port, baud)
        self.parser = ImuParser()

        qos = qos_profile_sensor_data
        self.pub_imu = self.create_publisher(Imu, "imu/data", qos)
        self.pub_rpy = self.create_publisher(Vector3Stamped, "imu/rpy", qos)
        self.pub_pose = self.create_publisher(PoseStamped, "imu/pose", qos)
        self.pub_mag = self.create_publisher(Vector3Stamped, "imu/mag_raw", qos)
        self.pub_temp = self.create_publisher(Temperature, "imu/temperature", qos)
        self.pub_press = self.create_publisher(FluidPressure, "imu/pressure", qos)
        self.pub_alt = self.create_publisher(Float64, "imu/altitude", qos)

        if self.declare_parameter("publish_tf", True).value:
            self.tf = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = self.frame_id
            t.transform.rotation.w = 1.0
            self.tf.sendTransform(t)

        self.parent = parent
        self.timer = self.create_timer(0.002, self._poll)
        self.last_stats = time.time()

    def _open(self, port, baud):
        bauds = [baud] if baud else BAUDS
        for b in bauds:
            ser = serial.Serial(port, b, timeout=0)
            ser.reset_input_buffer()
            probe = ImuParser()
            t0 = time.time()
            while time.time() - t0 < 1.5:
                probe.feed(ser.read(256))
                if probe.packets >= 3:
                    self.get_logger().info(f"IMU on {port} @ {b} baud, packet types: "
                                           + " ".join(f"0x{k:02X}" for k in sorted(probe.seen)))
                    return ser
                time.sleep(0.01)
            ser.close()
            self.get_logger().warn(f"{port} @ {b}: no valid IMU packets")
        raise RuntimeError(f"No IMU data on {port}")

    def _poll(self):
        try:
            chunk = self.ser.read(self.ser.in_waiting or 0)
        except serial.SerialException as e:
            self.get_logger().fatal(f"Serial error: {e}")
            raise SystemExit(1)
        if not chunk:
            return
        kinds = self.parser.feed(chunk)
        seen = self.parser.seen
        trigger = 0x53 if 0x53 in seen else (0x52 if 0x52 in seen else 0x51)
        stamp = self.get_clock().now().to_msg()
        d = self.parser.data
        for k in kinds:
            if k == trigger:
                self._publish_imu(stamp, d)
            elif k == 0x54:
                m = Vector3Stamped()
                m.header.stamp, m.header.frame_id = stamp, self.frame_id
                m.vector.x, m.vector.y, m.vector.z = (float(v) for v in d["mag"])
                self.pub_mag.publish(m)
            elif k == 0x56:
                p = FluidPressure()
                p.header.stamp, p.header.frame_id = stamp, self.frame_id
                p.fluid_pressure = float(d["pressure"])
                self.pub_press.publish(p)
                self.pub_alt.publish(Float64(data=float(d["height"])))
            if k == 0x51:
                t = Temperature()
                t.header.stamp, t.header.frame_id = stamp, self.frame_id
                t.temperature = d["temp"]
                self.pub_temp.publish(t)

        now = time.time()
        if now - self.last_stats > 30:
            self.get_logger().info(f"packets={self.parser.packets} checksum_errors={self.parser.bad}")
            self.last_stats = now

    def _publish_imu(self, stamp, d):
        roll, pitch, yaw = d["angle"]
        if 0x59 in self.parser.seen:
            qw, qx, qy, qz = d["quat"]
        else:
            qw, qx, qy, qz = rpy_to_quat(roll * DEG, pitch * DEG, yaw * DEG)

        msg = Imu()
        msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
        if 0x53 in self.parser.seen or 0x59 in self.parser.seen:
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = qw, qx, qy, qz
            msg.orientation_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
        else:
            msg.orientation_covariance[0] = -1.0  # orientation not provided
        gx, gy, gz = d["gyro"]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx * DEG, gy * DEG, gz * DEG
        msg.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        ax, ay, az = d["acc"]
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax * G, ay * G, az * G
        msg.linear_acceleration_covariance = [1e-2, 0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 1e-2]
        self.pub_imu.publish(msg)

        rpy = Vector3Stamped()
        rpy.header = msg.header
        rpy.vector.x, rpy.vector.y, rpy.vector.z = roll, pitch, yaw
        self.pub_rpy.publish(rpy)

        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, self.parent
        pose.pose.orientation = msg.orientation if msg.orientation_covariance[0] >= 0 else pose.pose.orientation
        self.pub_pose.publish(pose)


def main():
    rclpy.init()
    try:
        node = ImuNode()
    except (RuntimeError, serial.SerialException) as e:
        print(f"[imu_node] {e}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
