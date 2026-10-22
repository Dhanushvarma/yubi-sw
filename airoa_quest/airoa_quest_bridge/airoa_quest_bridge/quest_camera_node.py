"""ROS 2 node publishing the Quest passthrough camera streamed by yubi-quest-app.

Connects to the app's camera server (see ``transport/camera_stream.py`` for
the wire format) and publishes, per frame:

  * ``/center_camera/image_raw/compressed`` (sensor_msgs/CompressedImage, jpeg)
  * ``/center_camera/camera_info``          (sensor_msgs/CameraInfo, pinhole)

plus static TF ``quest_hmd -> center_camera_link -> center_camera_optical_frame``
from the camera pose the app reports (re-sent whenever it changes).

Stamps are the capture time on the ROS clock: receive time minus the
exposure -> send delay the Quest measured, so they are late only by the
one-way network latency.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros import StaticTransformBroadcaster

from airoa_quest_bridge.quest_bridge_node import (
    _normalize_quat,
    _unity_pos_to_ros,
    _unity_quat_to_ros,
)
from airoa_quest_bridge.transport.camera_stream import CameraFrame, CameraStreamTransport

# camera_link (x forward, y left, z up) -> optical frame (z forward, x right, y down).
_LINK_TO_OPTICAL_QUAT = (-0.5, 0.5, -0.5, 0.5)

# Extrinsics are re-broadcast only when they move by more than this.
_EXTRINSICS_EPS = 1e-4


def extrinsics_to_ros(
    position: Tuple[float, float, float],
    rotation: Tuple[float, float, float, float],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """Convert the app's camera-in-HMD pose (Unity frame) to camera_link in quest_hmd."""
    translation = _unity_pos_to_ros(*position)
    quat = _normalize_quat(*_unity_quat_to_ros(*rotation))
    return translation, quat


def make_camera_info(frame: CameraFrame, stamp, frame_id: str) -> CameraInfo:
    info = CameraInfo()
    info.header.stamp = stamp
    info.header.frame_id = frame_id
    info.width = int(frame.width)
    info.height = int(frame.height)
    # The app sends pinhole intrinsics for the delivered image; no distortion.
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [frame.fx, 0.0, frame.cx, 0.0, frame.fy, frame.cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [frame.fx, 0.0, frame.cx, 0.0, 0.0, frame.fy, frame.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


class QuestCameraNode(Node):
    def __init__(self):
        super().__init__("quest_camera_node")

        self.declare_parameter("quest_ip", "")
        self.declare_parameter("tcp_port", 65433)
        self.declare_parameter("connect_retry_sec", 1.0)
        self.declare_parameter("parent_frame", "quest_hmd")
        self.declare_parameter("camera_frame", "center_camera_link")
        self.declare_parameter("optical_frame", "center_camera_optical_frame")
        self.declare_parameter("image_topic", "/center_camera/image_raw/compressed")
        self.declare_parameter("camera_info_topic", "/center_camera/camera_info")

        self._parent_frame = self.get_parameter("parent_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._optical_frame = self.get_parameter("optical_frame").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._image_pub = self.create_publisher(
            CompressedImage, self.get_parameter("image_topic").value, qos
        )
        self._info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("camera_info_topic").value, qos
        )
        self._static_tf = StaticTransformBroadcaster(self)
        self._last_extrinsics: Optional[Tuple[float, ...]] = None

        quest_ip = self.get_parameter("quest_ip").value
        if not quest_ip:
            self.get_logger().warn("quest_ip is empty; set it in config/local/yubi_devices.yaml")
        self._transport = CameraStreamTransport(
            ip=quest_ip,
            tcp_port=int(self.get_parameter("tcp_port").value),
            on_frame=self._on_frame,
            logger=self.get_logger(),
        )
        retry = float(self.get_parameter("connect_retry_sec").value)
        self.create_timer(retry, self._transport.try_connect)
        self.create_timer(5.0, self._log_rate)

    def _on_frame(self, frame: CameraFrame) -> None:
        stamp = (
            self.get_clock().now() - Duration(nanoseconds=frame.device_latency_ns)
        ).to_msg()

        self._maybe_send_extrinsics(frame, stamp)

        info = make_camera_info(frame, stamp, self._optical_frame)
        image = CompressedImage()
        image.header = info.header
        image.format = "jpeg"
        image.data = frame.jpeg
        self._info_pub.publish(info)
        self._image_pub.publish(image)

    def _maybe_send_extrinsics(self, frame: CameraFrame, stamp) -> None:
        current = tuple(frame.position) + tuple(frame.rotation)
        if self._last_extrinsics is not None and all(
            math.isclose(a, b, abs_tol=_EXTRINSICS_EPS)
            for a, b in zip(current, self._last_extrinsics)
        ):
            return
        self._last_extrinsics = current

        translation, quat = extrinsics_to_ros(frame.position, frame.rotation)
        link = TransformStamped()
        link.header.stamp = stamp
        link.header.frame_id = self._parent_frame
        link.child_frame_id = self._camera_frame
        (
            link.transform.translation.x,
            link.transform.translation.y,
            link.transform.translation.z,
        ) = translation
        (
            link.transform.rotation.x,
            link.transform.rotation.y,
            link.transform.rotation.z,
            link.transform.rotation.w,
        ) = quat

        optical = TransformStamped()
        optical.header.stamp = stamp
        optical.header.frame_id = self._camera_frame
        optical.child_frame_id = self._optical_frame
        (
            optical.transform.rotation.x,
            optical.transform.rotation.y,
            optical.transform.rotation.z,
            optical.transform.rotation.w,
        ) = _LINK_TO_OPTICAL_QUAT

        self._static_tf.sendTransform([link, optical])
        self.get_logger().info(
            f"Camera extrinsics in {self._parent_frame}: t={translation}, q={quat}"
        )

    def _log_rate(self) -> None:
        if self._transport.connected:
            self.get_logger().debug(f"camera fps {self._transport.fps():.1f}")

    def destroy_node(self):
        self._transport.stop()
        return super().destroy_node()


def main():
    rclpy.init()
    node = QuestCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
