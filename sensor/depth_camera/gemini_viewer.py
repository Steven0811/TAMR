#!/usr/bin/env python3
"""Orbbec Gemini (Yahboom Gemini-Max) live viewer: color + depth side by side.

Keys:
  q / ESC  quit
  s        save color PNG + raw depth (16-bit PNG, millimetres) to ./captures
  l        toggle laser (IR projector)
  m        toggle mirror
Mouse: left-click on the depth image to print the distance at that point.
"""
import os
import time

import cv2
import numpy as np

from pyorbbecsdk import Config, OBPermissionType, OBPropertyID, OBSensorType, Pipeline
from ob_utils import frame_to_bgr_image

MIN_DEPTH_MM = 150
MAX_DEPTH_MM = 5000
CAPTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")

clicked = None  # (x, y) on the depth image


def on_mouse(event, x, y, flags, param):
    global clicked
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked = (x, y)


def try_toggle(device, prop, name):
    if not device.is_property_supported(prop, OBPermissionType.PERMISSION_READ_WRITE):
        print(f"{name}: not supported on this device")
        return
    value = not device.get_bool_property(prop)
    device.set_bool_property(prop, value)
    print(f"{name}: {'on' if value else 'off'}")


def main():
    pipeline = Pipeline()
    device = pipeline.get_device()
    print(device.get_device_info())

    config = Config()
    depth_profile = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
    config.enable_stream(depth_profile)
    print("depth:", depth_profile)

    # Color comes through the SDK if it exposes the color sensor; otherwise fall
    # back to the plain UVC node (/dev/video0) with OpenCV.
    uvc = None
    try:
        color_profile = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR).get_default_video_stream_profile()
        config.enable_stream(color_profile)
        print("color:", color_profile)
    except Exception as e:
        print(f"SDK color stream unavailable ({e}); using /dev/video0 via V4L2")
        uvc = cv2.VideoCapture(0, cv2.CAP_V4L2)

    pipeline.start(config)
    cv2.namedWindow("Gemini", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Gemini", on_mouse)
    os.makedirs(CAPTURE_DIR, exist_ok=True)

    global clicked
    color_img, depth_mm = None, None
    try:
        while True:
            frames = pipeline.wait_for_frames(100)
            if frames is not None:
                depth_frame = frames.get_depth_frame()
                if depth_frame is not None:
                    h, w = depth_frame.get_height(), depth_frame.get_width()
                    raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(h, w)
                    depth_mm = (raw.astype(np.float32) * depth_frame.get_depth_scale()).astype(np.uint16)
                if uvc is None:
                    color_frame = frames.get_color_frame()
                    if color_frame is not None:
                        color_img = frame_to_bgr_image(color_frame)
            if uvc is not None:
                ok, img = uvc.read()
                if ok:
                    color_img = img
            if depth_mm is None:
                continue

            valid = np.where((depth_mm > MIN_DEPTH_MM) & (depth_mm < MAX_DEPTH_MM), depth_mm, 0)
            depth_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(valid, alpha=255.0 / MAX_DEPTH_MM), cv2.COLORMAP_JET)
            depth_vis[valid == 0] = 0

            if clicked is not None:
                x, y = clicked
                if x < depth_vis.shape[1] and y < depth_vis.shape[0]:
                    print(f"distance at ({x},{y}): {depth_mm[y, x]} mm")
                    cv2.circle(depth_vis, (x, y), 4, (255, 255, 255), -1)
                clicked = None

            view = depth_vis
            if color_img is not None:
                c = cv2.resize(color_img, (depth_vis.shape[1], depth_vis.shape[0]))
                view = np.hstack([depth_vis, c])
            cv2.imshow("Gemini", view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('s'):
                stamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(os.path.join(CAPTURE_DIR, f"depth_{stamp}.png"), depth_mm)
                if color_img is not None:
                    cv2.imwrite(os.path.join(CAPTURE_DIR, f"color_{stamp}.png"), color_img)
                print("saved to", CAPTURE_DIR)
            elif key == ord('l'):
                try_toggle(device, OBPropertyID.OB_PROP_LASER_BOOL, "laser")
            elif key == ord('m'):
                try_toggle(device, OBPropertyID.OB_PROP_DEPTH_MIRROR_BOOL, "depth mirror")
                try_toggle(device, OBPropertyID.OB_PROP_COLOR_MIRROR_BOOL, "color mirror")
    finally:
        pipeline.stop()
        if uvc is not None:
            uvc.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
