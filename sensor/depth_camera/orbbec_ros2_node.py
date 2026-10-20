#!/usr/bin/env python3
"""ROS 2 driver for the Orbbec Gemini (SV1301S_U3) using pyorbbecsdk.

Topics:
    /camera/color/image_raw/compressed  sensor_msgs/CompressedImage  (JPEG, straight from the MJPG stream)
    /camera/color/camera_info           sensor_msgs/CameraInfo
    /camera/depth/image_raw             sensor_msgs/Image            (16UC1, millimetres)
    /camera/depth/camera_info           sensor_msgs/CameraInfo
    /camera/depth/points                sensor_msgs/PointCloud2      (xyz, downsampled)

Parameters: enable_color, enable_pointcloud, pointcloud_stride, pointcloud_rate,
            min_depth, max_depth (m), parent_frame, camera_x/y/z (m, mount offset)
"""
import array
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

from pyorbbecsdk import Config, OBFormat, OBSensorType, Pipeline
from ob_utils import frame_to_bgr_image

# camera_link (x forward, z up) -> optical frame (z forward, x right, y down)
OPTICAL_QUAT = (-0.5, 0.5, -0.5, 0.5)  # x, y, z, w


def to_u8_array(buf):
    a = array.array("B")
    a.frombytes(buf)
    return a


class OrbbecNode(Node):
    def __init__(self):
        super().__init__("orbbec_camera")
        p = self.declare_parameter
        self.enable_color = p("enable_color", True).value
        self.enable_cloud = p("enable_pointcloud", True).value
        self.stride = max(1, int(p("pointcloud_stride", 2).value))
        self.cloud_period = 1.0 / max(0.1, float(p("pointcloud_rate", 10.0).value))
        self.min_depth = float(p("min_depth", 0.15).value)
        self.max_depth = float(p("max_depth", 8.0).value)
        parent = p("parent_frame", "base_link").value
        offset = (p("camera_x", 0.0).value, p("camera_y", 0.0).value, p("camera_z", 0.1).value)

        self.depth_frame = "camera_depth_optical_frame"
        self.color_frame = "camera_color_optical_frame"

        qos = qos_profile_sensor_data
        self.pub_color = self.create_publisher(CompressedImage, "camera/color/image_raw/compressed", qos)
        self.pub_color_info = self.create_publisher(CameraInfo, "camera/color/camera_info", qos)
        self.pub_depth = self.create_publisher(Image, "camera/depth/image_raw", qos)
        self.pub_depth_info = self.create_publisher(CameraInfo, "camera/depth/camera_info", qos)
        self.pub_cloud = self.create_publisher(PointCloud2, "camera/depth/points", qos)

        self._start_pipeline()
        self._publish_static_tf(parent, offset)

        self.rays = None
        self.last_cloud = 0.0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ---------------------------------------------------------------- setup
    def _start_pipeline(self):
        self.pipeline = Pipeline()
        info = self.pipeline.get_device().get_device_info()
        self.get_logger().info(f"Device: {info.get_name()} SN={info.get_serial_number()}")

        config = Config()
        depth_profile = self.pipeline.get_stream_profile_list(
            OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
        config.enable_stream(depth_profile)
        self.get_logger().info(f"depth: {depth_profile}")
        self.has_color = False
        if self.enable_color:
            try:
                color_profile = self.pipeline.get_stream_profile_list(
                    OBSensorType.COLOR_SENSOR).get_default_video_stream_profile()
                config.enable_stream(color_profile)
                self.has_color = True
                self.get_logger().info(f"color: {color_profile}")
            except Exception as e:
                self.get_logger().warn(f"Color stream unavailable: {e}")
        self.pipeline.start(config)

        param = self.pipeline.get_camera_param()
        self.depth_intr = param.depth_intrinsic
        self.color_intr = param.rgb_intrinsic
        # Depth -> color extrinsic translation is in mm
        self.color_trans = [float(t) / 1000.0 for t in param.transform.transform]
        self.get_logger().info(f"camera param: {param}")

    def _publish_static_tf(self, parent, offset):
        self.tf = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()

        def tf(parent_frame, child, xyz, q=(0.0, 0.0, 0.0, 1.0)):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent_frame, child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = q
            return t

        self.tf.sendTransform([
            tf(parent, "camera_link", offset),
            tf("camera_link", self.depth_frame, (0, 0, 0), OPTICAL_QUAT),
            # Colour optical frame, offset from depth by the factory extrinsic (expressed in optical axes)
            tf(self.depth_frame, self.color_frame, self.color_trans),
        ])

    # ---------------------------------------------------------------- helpers
    @staticmethod
    def _scaled_intrinsics(intr, w, h):
        sx = w / intr.width if intr.width > 0 else 1.0
        sy = h / intr.height if intr.height > 0 else 1.0
        return intr.fx * sx, intr.fy * sy, intr.cx * sx, intr.cy * sy

    def _camera_info(self, stamp, frame_id, intr, w, h):
        fx, fy, cx, cy = self._scaled_intrinsics(intr, w, h)
        ci = CameraInfo()
        ci.header.stamp, ci.header.frame_id = stamp, frame_id
        ci.width, ci.height = w, h
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0] * 5
        ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def _cloud(self, stamp, depth_m):
        h, w = depth_m.shape
        s = self.stride
        if self.rays is None or self.rays[0].shape != depth_m[::s, ::s].shape:
            fx, fy, cx, cy = self._scaled_intrinsics(self.depth_intr, w, h)
            u, v = np.meshgrid(np.arange(0, w, s), np.arange(0, h, s))
            self.rays = ((u - cx) / fx, (v - cy) / fy)
        z = depth_m[::s, ::s]
        valid = (z > self.min_depth) & (z < self.max_depth)
        z = z[valid]
        pts = np.empty((z.size, 3), dtype=np.float32)
        pts[:, 0] = self.rays[0][valid] * z
        pts[:, 1] = self.rays[1][valid] * z
        pts[:, 2] = z

        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = stamp, self.depth_frame
        msg.height, msg.width = 1, pts.shape[0]
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * pts.shape[0]
        msg.is_dense = True
        msg.data = to_u8_array(pts.tobytes())
        return msg

    # ---------------------------------------------------------------- main loop
    def _loop(self):
        while self.running and rclpy.ok():
            try:
                frames = self.pipeline.wait_for_frames(100)
            except Exception as e:
                self.get_logger().error(f"wait_for_frames failed: {e}")
                time.sleep(0.1)
                continue
            if frames is None:
                continue
            stamp = self.get_clock().now().to_msg()

            depth = frames.get_depth_frame()
            if depth is not None:
                self._handle_depth(stamp, depth)
            if self.has_color:
                color = frames.get_color_frame()
                if color is not None:
                    self._handle_color(stamp, color)

    def _handle_depth(self, stamp, frame):
        w, h = frame.get_width(), frame.get_height()
        raw = np.frombuffer(frame.get_data(), dtype=np.uint16).reshape(h, w)
        depth_mm = (raw.astype(np.float32) * frame.get_depth_scale())

        img = Image()
        img.header.stamp, img.header.frame_id = stamp, self.depth_frame
        img.height, img.width = h, w
        img.encoding = "16UC1"
        img.is_bigendian = 0
        img.step = w * 2
        img.data = to_u8_array(np.clip(depth_mm, 0, 65535).astype(np.uint16).tobytes())
        self.pub_depth.publish(img)
        self.pub_depth_info.publish(self._camera_info(stamp, self.depth_frame, self.depth_intr, w, h))

        now = time.time()
        if self.enable_cloud and now - self.last_cloud >= self.cloud_period \
                and self.pub_cloud.get_subscription_count() > 0:
            self.last_cloud = now
            self.pub_cloud.publish(self._cloud(stamp, depth_mm / 1000.0))

    def _handle_color(self, stamp, frame):
        w, h = frame.get_width(), frame.get_height()
        if frame.get_format() == OBFormat.MJPG:
            jpeg = frame.get_data().tobytes()
        else:
            bgr = frame_to_bgr_image(frame)
            if bgr is None:
                return
            ok, enc = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                return
            jpeg = enc.tobytes()
        msg = CompressedImage()
        msg.header.stamp, msg.header.frame_id = stamp, self.color_frame
        msg.format = "jpeg"
        msg.data = to_u8_array(jpeg)
        self.pub_color.publish(msg)
        self.pub_color_info.publish(self._camera_info(stamp, self.color_frame, self.color_intr, w, h))

    def shutdown(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.pipeline.stop()


def main():
    rclpy.init()
    try:
        node = OrbbecNode()
    except Exception as e:
        print(f"[orbbec_camera] failed to open camera: {e}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
