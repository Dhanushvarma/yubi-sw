#!/usr/bin/env python3
"""Publish gripper joint states from the Quest controllers' analog triggers.

Stand-in for encoder_node when there are no YUBI gloves (quest_only variant):
the operator holds the Touch Plus controllers and the index trigger plays the
role of the glove's finger angle. A released trigger maps to the fully open
finger angle and a fully pulled trigger to the closed angle (0 rad, the
encoder's calibrated closed position).

Subscribes:
  * ``/quest/controller/{left,right}/joy`` (sensor_msgs/Joy, axes[2] = trigger 0..1)

Publishes (same names, topics and rate as encoder_node):
  * ``/joint_states``                      (both joints, only when both sides are fresh)
  * ``/yubi/gripper/{left,right}/joint_state``
"""
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Joy, JointState


def trigger_to_position(trigger: float, open_rad: float, closed_rad: float) -> Optional[float]:
    """Map a trigger value in [0, 1] linearly onto [open_rad, closed_rad].

    Returns None for NaN (the app did not send an analog value). Values outside
    [0, 1] are clamped.
    """
    if math.isnan(trigger):
        return None
    t = min(max(trigger, 0.0), 1.0)
    return open_rad + (closed_rad - open_rad) * t


class TriggerGripperNode(Node):
    def __init__(self):
        super().__init__("trigger_gripper_node")

        self.declare_parameter("left_joy_topic", "/quest/controller/left/joy")
        self.declare_parameter("right_joy_topic", "/quest/controller/right/joy")
        self.declare_parameter("trigger_axis", 2)
        self.declare_parameter("left_joint_name", "left_right_finger_joint")
        self.declare_parameter("right_joint_name", "right_right_finger_joint")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("topic", "/joint_states")
        self.declare_parameter("left_joint_state_topic", "/yubi/gripper/left/joint_state")
        self.declare_parameter("right_joint_state_topic", "/yubi/gripper/right/joint_state")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("data_timeout_sec", 0.5)
        # URDF finger limits (yubi_description/urdf/yubi_hand.urdf.xacro).
        self.declare_parameter("open_position_rad", 0.94)
        self.declare_parameter("closed_position_rad", 0.0)

        self.trigger_axis = int(self.get_parameter("trigger_axis").value)
        self.left_joint_name = self.get_parameter("left_joint_name").value
        self.right_joint_name = self.get_parameter("right_joint_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.data_timeout_sec = float(self.get_parameter("data_timeout_sec").value)
        self.open_rad = float(self.get_parameter("open_position_rad").value)
        self.closed_rad = float(self.get_parameter("closed_position_rad").value)

        # side -> (position, monotonic receive time)
        self._latest = {"left": None, "right": None}
        self._stale = {"left": False, "right": False}

        # Quest Joy topics are BEST_EFFORT (quest_bridge_node).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Joy,
            self.get_parameter("left_joy_topic").value,
            lambda msg: self._on_joy("left", msg),
            qos,
        )
        self.create_subscription(
            Joy,
            self.get_parameter("right_joy_topic").value,
            lambda msg: self._on_joy("right", msg),
            qos,
        )

        self.pub = self.create_publisher(JointState, self.get_parameter("topic").value, 10)
        self.left_pub = self.create_publisher(
            JointState, self.get_parameter("left_joint_state_topic").value, 10
        )
        self.right_pub = self.create_publisher(
            JointState, self.get_parameter("right_joint_state_topic").value, 10
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    def _on_joy(self, side: str, msg: Joy) -> None:
        if len(msg.axes) <= self.trigger_axis:
            return
        position = trigger_to_position(
            float(msg.axes[self.trigger_axis]), self.open_rad, self.closed_rad
        )
        if position is not None:
            self._latest[side] = (position, time.monotonic())

    def _get_latest(self, side: str) -> Optional[float]:
        latest = self._latest[side]
        if latest is None:
            return None
        position, received = latest
        age = time.monotonic() - received
        if age > self.data_timeout_sec:
            if not self._stale[side]:
                self._stale[side] = True
                self.get_logger().warning(
                    f"{side} trigger data is stale ({age:.2f}s > "
                    f"{self.data_timeout_sec:.2f}s). Pausing publishing."
                )
            return None
        if self._stale[side]:
            self._stale[side] = False
            self.get_logger().info(f"{side} trigger data resumed.")
        return position

    def _joint_state(self, stamp, names, positions) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        if self.frame_id:
            msg.header.frame_id = self.frame_id
        msg.name = names
        msg.position = positions
        return msg

    def on_timer(self):
        left_v = self._get_latest("left")
        right_v = self._get_latest("right")

        stamp = self.get_clock().now().to_msg()

        if left_v is not None:
            self.left_pub.publish(self._joint_state(stamp, [self.left_joint_name], [left_v]))
        if right_v is not None:
            self.right_pub.publish(self._joint_state(stamp, [self.right_joint_name], [right_v]))
        if left_v is not None and right_v is not None:
            self.pub.publish(
                self._joint_state(
                    stamp,
                    [self.left_joint_name, self.right_joint_name],
                    [left_v, right_v],
                )
            )


def main():
    rclpy.init()
    node = TriggerGripperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
