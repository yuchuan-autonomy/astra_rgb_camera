#!/usr/bin/env python3
"""Publish the Astra Pro Plus UVC RGB stream without an Orbbec SDK."""

from __future__ import annotations

import re
import time
from pathlib import Path
from typing import Iterable

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


ASTRA_RGB_VENDOR_ID = "2bc5"
ASTRA_RGB_PRODUCT_ID = "050f"


def _video_device_sort_key(path: Path) -> tuple[int, str]:
    match = re.search(r"(\d+)$", path.name)
    return (int(match.group(1)) if match else 2**31, path.name)


def _usb_ids_for_video_class(video_class: Path) -> tuple[str, str] | None:
    try:
        device_path = video_class.resolve(strict=True)
    except (FileNotFoundError, RuntimeError):
        return None

    for parent in (device_path, *device_path.parents):
        vendor_file = parent / "idVendor"
        product_file = parent / "idProduct"
        if not vendor_file.is_file() or not product_file.is_file():
            continue
        try:
            return (
                vendor_file.read_text(encoding="ascii").strip().lower(),
                product_file.read_text(encoding="ascii").strip().lower(),
            )
        except OSError:
            return None
    return None


def find_video_devices(
    vendor_id: str = ASTRA_RGB_VENDOR_ID,
    product_id: str = ASTRA_RGB_PRODUCT_ID,
    sys_class_root: Path = Path("/sys/class/video4linux"),
    dev_root: Path = Path("/dev"),
) -> list[str]:
    """Return V4L2 nodes belonging to the requested USB VID/PID."""
    if not sys_class_root.is_dir():
        return []

    matches: list[Path] = []
    for video_class in sorted(sys_class_root.glob("video*"), key=_video_device_sort_key):
        usb_ids = _usb_ids_for_video_class(video_class)
        if usb_ids != (vendor_id.lower(), product_id.lower()):
            continue
        device_node = dev_root / video_class.name
        if device_node.exists():
            matches.append(device_node)
    return [str(path) for path in matches]


class AstraRgbCameraNode(Node):
    """Read an Astra UVC node and publish BGR frames as sensor_msgs/Image."""

    def __init__(self) -> None:
        super().__init__("astra_rgb_camera")

        self.declare_parameter("device", "")
        self.declare_parameter("vendor_id", ASTRA_RGB_VENDOR_ID)
        self.declare_parameter("product_id", ASTRA_RGB_PRODUCT_ID)
        self.declare_parameter("image_topic", "/front_camera/image_raw")
        self.declare_parameter("frame_id", "front_camera_color_optical_frame")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("pixel_format", "MJPG")
        self.declare_parameter("flip_horizontal", False)
        self.declare_parameter("flip_vertical", False)
        self.declare_parameter("reconnect_interval", 1.0)

        self.device = str(self.get_parameter("device").value)
        self.vendor_id = str(self.get_parameter("vendor_id").value).lower()
        self.product_id = str(self.get_parameter("product_id").value).lower()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.pixel_format = str(self.get_parameter("pixel_format").value).upper()
        self.flip_horizontal = bool(self.get_parameter("flip_horizontal").value)
        self.flip_vertical = bool(self.get_parameter("flip_vertical").value)
        self.reconnect_interval = float(self.get_parameter("reconnect_interval").value)

        if self.width <= 0 or self.height <= 0 or self.fps <= 0.0:
            raise ValueError("width, height, and fps must be positive")
        if len(self.pixel_format) != 4:
            raise ValueError("pixel_format must be a four-character V4L2 code")

        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.capture: cv2.VideoCapture | None = None
        self.capture_device = ""
        self.pending_frame = None
        self.failed_reads = 0
        self.next_reconnect_time = 0.0
        self.next_warning_time = 0.0

        self.timer = self.create_timer(1.0 / self.fps, self._capture_and_publish)
        self.get_logger().info(
            f"Publishing Astra RGB images on {self.image_topic}; "
            f"USB ID {self.vendor_id}:{self.product_id}"
        )

    def _candidate_devices(self) -> Iterable[str]:
        if self.device:
            return [self.device]
        return find_video_devices(self.vendor_id, self.product_id)

    def _configure_capture(self, capture: cv2.VideoCapture) -> None:
        fourcc = cv2.VideoWriter_fourcc(*self.pixel_format)
        capture.set(cv2.CAP_PROP_FOURCC, fourcc)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now >= self.next_warning_time:
            self.get_logger().warning(message)
            self.next_warning_time = now + 5.0

    def _open_camera(self) -> bool:
        candidates = list(self._candidate_devices())
        if not candidates:
            self._warn_throttled(
                f"No V4L2 node found for USB ID {self.vendor_id}:{self.product_id}"
            )
            return False

        for device in candidates:
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not capture.isOpened():
                capture.release()
                continue

            self._configure_capture(capture)
            frame = None
            for _ in range(10):
                ok, candidate_frame = capture.read()
                if ok and candidate_frame is not None and candidate_frame.size > 0:
                    frame = candidate_frame
                    break
            if frame is None:
                capture.release()
                continue

            self.capture = capture
            self.capture_device = device
            self.pending_frame = frame
            self.failed_reads = 0
            actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = capture.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(
                f"Opened {device}: {actual_width}x{actual_height}@{actual_fps:.1f}"
            )
            return True

        self._warn_throttled(
            "Astra RGB V4L2 nodes were found but none produced a frame; "
            "the camera may be in use"
        )
        return False

    def _close_camera(self) -> None:
        if self.capture is not None:
            self.capture.release()
        self.capture = None
        self.capture_device = ""
        self.pending_frame = None

    def _capture_and_publish(self) -> None:
        if self.capture is None:
            now = time.monotonic()
            if now < self.next_reconnect_time:
                return
            if not self._open_camera():
                self.next_reconnect_time = now + self.reconnect_interval
                return

        if self.pending_frame is not None:
            frame = self.pending_frame
            self.pending_frame = None
            ok = True
        else:
            ok, frame = self.capture.read()

        if not ok or frame is None or frame.size == 0:
            self.failed_reads += 1
            if self.failed_reads >= 5:
                self.get_logger().warning(
                    f"Lost RGB frames from {self.capture_device}; reconnecting"
                )
                self._close_camera()
                self.next_reconnect_time = time.monotonic() + self.reconnect_interval
            return

        self.failed_reads = 0
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        if self.flip_horizontal and self.flip_vertical:
            frame = cv2.flip(frame, -1)
        elif self.flip_horizontal:
            frame = cv2.flip(frame, 1)
        elif self.flip_vertical:
            frame = cv2.flip(frame, 0)

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.height = int(frame.shape[0])
        message.width = int(frame.shape[1])
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = frame.tobytes()
        if not rclpy.ok():
            return
        try:
            self.publisher.publish(message)
        except Exception:
            if rclpy.ok():
                raise

    def destroy_node(self) -> bool:
        self._close_camera()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AstraRgbCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
