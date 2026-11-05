#!/usr/bin/env python3
"""ROS 2 bridge for Meta Quest HMD and controllers.

Receives a per-sample stream of HMD/controller poses and inputs from the Quest
device through a swappable transport (currently the legacy TCP/JSON
protocol; future: ``yubi_quest_app`` UDP/binary).

Publishes:
  * ``/quest/hmd/state``                       (airoa_quest_msgs/QuestHmd)
  * ``/quest/hmd/battery``                     (sensor_msgs/BatteryState)
  * ``/quest/controller/{left,right}/state``   (airoa_quest_msgs/QuestController;
                                                pose + tracking flags only)
  * ``/quest/controller/{left,right}/tracked`` (std_msgs/Bool; mirror of
                                                ``state.tracked`` for consumers
                                                without the airoa_quest_msgs dep)
  * ``/quest/controller/{left,right}/valid``   (std_msgs/Bool; mirror of
                                                ``state.valid``)
  * ``/quest/controller/{left,right}/joy``     (sensor_msgs/Joy; per-controller
                                                analog axes and digital buttons)
  * ``/quest/controller/{left,right}/battery`` (sensor_msgs/BatteryState)
  * ``/quest/body/state``                      (airoa_quest_msgs/QuestBody; only
                                                when the app sends a ``body``
                                                section, e.g. yubi-quest-app with
                                                Inside-Out Body Tracking)
  * ``/quest/body/fidelity``                   (std_msgs/UInt8; mirror of
                                                ``state.fidelity`` for consumers
                                                without the airoa_quest_msgs dep)
  * ``/tf``                                    (live RViz/Foxglove and IK
                                                consumers such as umi_ik_checker;
                                                with body data also
                                                ``body_{chest,left,right}_*``
                                                shoulder/elbow/wrist frames)
  * ``/diagnostics``                           (diagnostic_msgs/DiagnosticArray;
                                                canonical health channel —
                                                ``connected``/``streaming``/
                                                ``fps``/``*_tracked``/``*_valid``
                                                are reported here as KeyValues)

The new ``QuestHmd`` / ``QuestController`` topics carry the timestamp metadata
(``device_time_ns`` and ``pc_monotonic_ns``) needed by the offline alignment
post-processor to reconstruct evenly-spaced timestamps from the Quest's own
clock; the bridge intentionally does not perform that correction online.
"""

from typing import Any, Dict, Optional, Set, Tuple

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import BatteryState, Joy
from std_msgs.msg import Bool, UInt8
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformBroadcaster,
    TransformListener,
)

from airoa_quest_msgs.msg import QuestBody, QuestController, QuestHmd

from airoa_quest_bridge.transport.tcp_json import QuestFrame, TcpJsonTransport


# ---------------------------------------------------------------------------
# Math helpers (Unity <-> ROS coordinate conversion).
# ---------------------------------------------------------------------------


def _normalize_quat(x, y, z, w):
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / n, y / n, z / n, w / n


def _unity_pos_to_ros(x, y, z):
    """Unity (Y-up, left-handed) position -> ROS (Z-up, right-handed).

    Unity:  X=right, Y=up,   Z=forward
    ROS:    X=forward, Y=left, Z=up
    Mapping: Unity(x,y,z) -> ROS(z, -x, y).
    """
    return z, -x, y


def _unity_quat_to_ros(x, y, z, w):
    """Unity (left-handed) quaternion -> ROS (right-handed)."""
    return z, -x, y, -w


def _quat_conjugate(quat: Tuple[float, float, float, float]):
    x, y, z, w = quat
    return -x, -y, -z, w


def _quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return _normalize_quat(x, y, z, w)


def _cross(a, b):
    ax, ay, az = a
    bx, by, bz = b
    return ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx


def _rotate_vector(quat, vec):
    qx, qy, qz, qw = _normalize_quat(*quat)
    u = (qx, qy, qz)
    uv = _cross(u, vec)
    uuv = _cross(u, uv)
    uv = tuple(val * (2.0 * qw) for val in uv)
    uuv = tuple(val * 2.0 for val in uuv)
    return (
        vec[0] + uv[0] + uuv[0],
        vec[1] + uv[1] + uuv[1],
        vec[2] + uv[2] + uuv[2],
    )


def _invert_transform(trans, quat):
    inv_q = _quat_conjugate(quat)
    inv_q = _normalize_quat(*inv_q)
    neg_trans = (-trans[0], -trans[1], -trans[2])
    inv_t = _rotate_vector(inv_q, neg_trans)
    return inv_t, inv_q


def _compose_transforms(parent_trans, parent_quat, child_trans, child_quat):
    rotated_child = _rotate_vector(parent_quat, child_trans)
    composed_trans = (
        parent_trans[0] + rotated_child[0],
        parent_trans[1] + rotated_child[1],
        parent_trans[2] + rotated_child[2],
    )
    composed_quat = _quat_multiply(parent_quat, child_quat)
    return composed_trans, composed_quat


def _unity_pose_to_ros(pos, quat):
    ros_pos = _unity_pos_to_ros(*pos)
    ros_quat = _normalize_quat(*_unity_quat_to_ros(*quat))
    return ros_pos, ros_quat


def _fill_xyz(vec3, xyz: Tuple[float, float, float]) -> None:
    """Populate a Vector3-shaped field from a ROS (x, y, z) tuple."""
    vec3.x = float(xyz[0])
    vec3.y = float(xyz[1])
    vec3.z = float(xyz[2])


def _fill_xyzw(quat, xyzw: Tuple[float, float, float, float]) -> None:
    """Populate a Quaternion-shaped field from a ROS (x, y, z, w) tuple."""
    quat.x = float(xyzw[0])
    quat.y = float(xyzw[1])
    quat.z = float(xyzw[2])
    quat.w = float(xyzw[3])


def _make_pose(xyz, xyzw) -> Pose:
    pose = Pose()
    _fill_xyz(pose.position, xyz)
    _fill_xyzw(pose.orientation, xyzw)
    return pose


def _get_xyz(obj: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
    if not isinstance(obj, dict):
        return None
    try:
        return float(obj["x"]), float(obj["y"]), float(obj["z"])
    except Exception:
        return None


def _get_xyzw(obj: Dict[str, Any]) -> Optional[Tuple[float, float, float, float]]:
    if not isinstance(obj, dict):
        return None
    try:
        return float(obj["x"]), float(obj["y"]), float(obj["z"]), float(obj["w"])
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Body tracking (optional ``body`` section of the pose stream).
# ---------------------------------------------------------------------------

# OVRPlugin.BoneId names for the upper-body joint set, in index order.
_BODY_TORSO_AND_ARMS = (
    "Body_Root",
    "Body_Hips",
    "Body_SpineLower",
    "Body_SpineMiddle",
    "Body_SpineUpper",
    "Body_Chest",
    "Body_Neck",
    "Body_Head",
    "Body_LeftShoulder",
    "Body_LeftScapula",
    "Body_LeftArmUpper",
    "Body_LeftArmLower",
    "Body_LeftHandWristTwist",
    "Body_RightShoulder",
    "Body_RightScapula",
    "Body_RightArmUpper",
    "Body_RightArmLower",
    "Body_RightHandWristTwist",
)
_FINGER_BONES = (
    ("Thumb", ("Metacarpal", "Proximal", "Distal", "Tip")),
    ("Index", ("Metacarpal", "Proximal", "Intermediate", "Distal", "Tip")),
    ("Middle", ("Metacarpal", "Proximal", "Intermediate", "Distal", "Tip")),
    ("Ring", ("Metacarpal", "Proximal", "Intermediate", "Distal", "Tip")),
    ("Little", ("Metacarpal", "Proximal", "Intermediate", "Distal", "Tip")),
)


def _hand_joint_names(side: str) -> Tuple[str, ...]:
    names = [f"Body_{side}HandPalm", f"Body_{side}HandWrist"]
    for finger, bones in _FINGER_BONES:
        names.extend(f"Body_{side}Hand{finger}{bone}" for bone in bones)
    return tuple(names)


BODY_JOINT_NAMES: Tuple[str, ...] = (
    _BODY_TORSO_AND_ARMS + _hand_joint_names("Left") + _hand_joint_names("Right")
)

# Joints also published as TF children of parent_frame, as
# (BoneId name, frame suffix). Meta names each bone after the joint it starts
# at: ArmUpper starts at the shoulder, ArmLower at the elbow.
BODY_TF_JOINTS: Tuple[Tuple[str, str], ...] = (
    ("Body_Chest", "chest"),
    ("Body_LeftArmUpper", "left_shoulder"),
    ("Body_LeftArmLower", "left_elbow"),
    ("Body_LeftHandWrist", "left_wrist"),
    ("Body_RightArmUpper", "right_shoulder"),
    ("Body_RightArmLower", "right_elbow"),
    ("Body_RightHandWrist", "right_wrist"),
)

# XrSpaceLocationFlags: orientation valid | position valid.
_BODY_POSE_VALID = 0x1 | 0x2

_BODY_FIDELITY_NAMES = {1: "low", 2: "high"}


def _body_joint_name(index: int) -> str:
    if index < len(BODY_JOINT_NAMES):
        return BODY_JOINT_NAMES[index]
    return f"joint_{index}"


def _parse_body(data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Parse the optional ``body`` section of one pose-stream sample.

    Wire format (yubi-quest-app): ``time`` (s), ``confidence``, ``fidelity``,
    ``calibration``, ``skeletonChangedCount``, ``joints`` (7 floats per joint:
    Unity position x, y, z then rotation x, y, z, w, in the same tracking
    space as the HMD and controllers) and ``flags`` (one XrSpaceLocationFlags
    int per joint). Returns None when the section is absent or malformed, and
    otherwise the poses converted to ROS as ``[(pos, quat), ...]``.
    """
    body = data.get("body")
    if not isinstance(body, dict):
        return None
    joints = body.get("joints")
    flags = body.get("flags")
    if not isinstance(joints, list) or not joints or len(joints) % 7:
        return None
    count = len(joints) // 7
    if not isinstance(flags, list) or len(flags) != count:
        return None
    try:
        values = [float(v) for v in joints]
        poses = [
            _unity_pose_to_ros(values[i : i + 3], values[i + 3 : i + 7])
            for i in range(0, len(values), 7)
        ]
        return {
            "time": float(body.get("time", 0.0)),
            "confidence": float(body.get("confidence", 0.0)),
            "fidelity": int(body.get("fidelity", 0)) & 0xFF,
            "calibration": int(body.get("calibration", 0)) & 0xFF,
            "skeleton_changed_count": int(body.get("skeletonChangedCount", 0))
            & 0xFFFFFFFF,
            "poses": poses,
            "flags": [int(f) & 0xFF for f in flags],
        }
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------------------
# Diagnostic value formatters.
# ---------------------------------------------------------------------------


def _fmt_nan_float(value: float, fmt: str = ".3f") -> str:
    """Format a float, mapping NaN (which compares unequal to itself) to "nan"."""
    return f"{value:{fmt}}" if value == value else "nan"


def _fmt_nan_int(value: int) -> str:
    """Format an int that uses negatives as the missing-value sentinel."""
    return str(value) if value >= 0 else "nan"


# (status_key, value formatter) — order is preserved in the published
# DiagnosticStatus.values list so downstream tooling can rely on it.
_DIAG_FIELDS: Tuple[Tuple[str, Any], ...] = (
    ("connected", str),
    ("streaming", str),
    ("fps", lambda v: f"{v:.1f}"),
    ("last_frame_age_s", _fmt_nan_float),
    ("offset_s", lambda v: f"{v:.6f}"),
    ("rtt_ms", _fmt_nan_float),
    ("left_tracked", str),
    ("right_tracked", str),
    ("left_valid", str),
    ("right_valid", str),
    ("delta_time_s", lambda v: f"{v}" if v == v else "nan"),
    ("hmd_batt_pct", _fmt_nan_int),
    ("left_batt_pct", _fmt_nan_int),
    ("right_batt_pct", _fmt_nan_int),
    ("hmd_charging", str),
    ("body_fidelity", str),
    ("body_confidence", lambda v: _fmt_nan_float(v, ".2f")),
)


# (parameter name, default topic) for every publisher topic the node exposes.
_TOPIC_PARAMS: Tuple[Tuple[str, str], ...] = (
    ("diag_topic", "/diagnostics"),
    ("hmd_topic", "/quest/hmd/state"),
    ("hmd_battery_topic", "/quest/hmd/battery"),
    ("left_controller_topic", "/quest/controller/left/state"),
    ("right_controller_topic", "/quest/controller/right/state"),
    ("left_controller_tracked_topic", "/quest/controller/left/tracked"),
    ("right_controller_tracked_topic", "/quest/controller/right/tracked"),
    ("left_controller_valid_topic", "/quest/controller/left/valid"),
    ("right_controller_valid_topic", "/quest/controller/right/valid"),
    ("left_controller_joy_topic", "/quest/controller/left/joy"),
    ("right_controller_joy_topic", "/quest/controller/right/joy"),
    ("left_controller_battery_topic", "/quest/controller/left/battery"),
    ("right_controller_battery_topic", "/quest/controller/right/battery"),
    ("body_topic", "/quest/body/state"),
    ("body_fidelity_topic", "/quest/body/fidelity"),
)


# Field-name maps for the per-controller QuestController message. Keeping the
# raw protocol keys here (rather than scattered across publishers) so the
# mapping for a future protocol switch lives next to the legacy one.
_CONTROLLER_KEYS = {
    "left": {
        "pos": "leftControllerPosition",
        "rot": "leftControllerRotation",
        "tracked": "leftTracked",
        "valid": "leftValid",
        "joystick": "leftJoystick",
        "thumb_click": "leftThumbstickClick",
        "thumb_touch": "leftThumbstickTouched",
        "trigger_press": "leftTriggerPressed",
        "trigger_touch": "leftIndexTriggerTouched",
        "grip_press": "leftGripPressed",
        "primary_press": "buttonXPressed",
        "primary_touch": "buttonXTouched",
        "secondary_press": "buttonYPressed",
        "secondary_touch": "buttonYTouched",
        "menu_press": "startPressed",
    },
    "right": {
        "pos": "rightControllerPosition",
        "rot": "rightControllerRotation",
        "tracked": "rightTracked",
        "valid": "rightValid",
        "joystick": "rightJoystick",
        "thumb_click": "rightThumbstickClick",
        "thumb_touch": "rightThumbstickTouched",
        "trigger_press": "rightTriggerPressed",
        "trigger_touch": "rightIndexTriggerTouched",
        "grip_press": "rightGripPressed",
        "primary_press": "buttonAPressed",
        "primary_touch": "buttonATouched",
        "secondary_press": "buttonBPressed",
        "secondary_touch": "buttonBTouched",
        "menu_press": "backPressed",
    },
}


class QuestBridgeNode(Node):
    """Publish Quest sensor data with metadata sufficient for offline alignment."""

    def __init__(self):
        super().__init__("quest_bridge")

        # ---- Parameters -------------------------------------------------
        self.declare_parameter("quest_ip", "WRITE_HERE")
        self.declare_parameter("tcp_port", 65432)
        self.declare_parameter("sync_port", 42000)

        self.declare_parameter("parent_frame", "quest_origin")
        self.declare_parameter("hmd_frame", "quest_hmd")
        self.declare_parameter("left_frame", "left_hand_root")
        self.declare_parameter("right_frame", "right_hand_root")
        self.declare_parameter("left_controller_frame", "quest_left_controller")
        self.declare_parameter("right_controller_frame", "quest_right_controller")

        for name, default in _TOPIC_PARAMS:
            self.declare_parameter(name, default)

        self.declare_parameter("connect_retry_sec", 1.0)
        self.declare_parameter("diag_period_sec", 0.5)
        self.declare_parameter("battery_status_hz", 1.0)

        # Body tracking: TF frames <prefix><suffix> for BODY_TF_JOINTS.
        self.declare_parameter("publish_body_tf", True)
        self.declare_parameter("body_frame_prefix", "body_")

        # ---- Cached parameter values ------------------------------------
        # Read on every published frame; cache to avoid per-frame
        # get_parameter() lookups and Param object allocation.
        self._parent_frame = self.get_parameter("parent_frame").value
        self._hmd_frame = self.get_parameter("hmd_frame").value
        self._left_frame = self.get_parameter("left_frame").value
        self._right_frame = self.get_parameter("right_frame").value
        self._left_controller_frame = self.get_parameter("left_controller_frame").value
        self._right_controller_frame = self.get_parameter(
            "right_controller_frame"
        ).value
        self._hardware_id = self.get_parameter("quest_ip").value or "unset"
        self._publish_body_tf = bool(self.get_parameter("publish_body_tf").value)
        prefix = self.get_parameter("body_frame_prefix").value
        self._body_tf_frames: Tuple[Tuple[int, str], ...] = tuple(
            (BODY_JOINT_NAMES.index(name), f"{prefix}{suffix}")
            for name, suffix in BODY_TF_JOINTS
        )

        # ---- Latest-frame state (consumed by status / battery / diag timers) ----
        self._lock = threading.Lock()
        self._latest_raw: Optional[Dict[str, Any]] = None

        # ---- Publishers --------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter("diag_topic").value, 10
        )
        self._hmd_pub = self.create_publisher(
            QuestHmd, self.get_parameter("hmd_topic").value, qos
        )
        self._left_ctrl_pub = self.create_publisher(
            QuestController, self.get_parameter("left_controller_topic").value, qos
        )
        self._right_ctrl_pub = self.create_publisher(
            QuestController, self.get_parameter("right_controller_topic").value, qos
        )
        # Default-reliable QoS so consumers like recording_gate (RELIABLE
        # subscriber) match without an extra QoS override.
        self._controller_flag_pubs: Dict[str, Dict[str, Any]] = {
            side: {
                "tracked": self.create_publisher(
                    Bool,
                    self.get_parameter(f"{side}_controller_tracked_topic").value,
                    10,
                ),
                "valid": self.create_publisher(
                    Bool,
                    self.get_parameter(f"{side}_controller_valid_topic").value,
                    10,
                ),
            }
            for side in ("left", "right")
        }
        self._left_joy_pub = self.create_publisher(
            Joy, self.get_parameter("left_controller_joy_topic").value, qos
        )
        self._right_joy_pub = self.create_publisher(
            Joy, self.get_parameter("right_controller_joy_topic").value, qos
        )
        self._body_pub = self.create_publisher(
            QuestBody, self.get_parameter("body_topic").value, qos
        )
        # Reliable, like the controller flag mirrors, for recording_gate.
        self._body_fidelity_pub = self.create_publisher(
            UInt8, self.get_parameter("body_fidelity_topic").value, 10
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._hand_controller_transforms: Dict[
            Tuple[str, str],
            Tuple[Tuple[float, float, float], Tuple[float, float, float, float]],
        ] = {}
        self._missing_offset_logged: Set[Tuple[str, str]] = set()

        # (logical name, raw pct status-key, raw charging status-key or None,
        # topic parameter name)
        self._battery_specs: Tuple[Tuple[str, str, Optional[str], str], ...] = (
            ("hmd", "hmd_batt_pct", "hmd_charging", "hmd_battery_topic"),
            ("left", "left_batt_pct", None, "left_controller_battery_topic"),
            ("right", "right_batt_pct", None, "right_controller_battery_topic"),
        )
        self._battery_pubs: Dict[str, Any] = {
            name: self.create_publisher(
                BatteryState, self.get_parameter(topic_param).value, qos
            )
            for name, _, _, topic_param in self._battery_specs
        }

        # Shared snapshot cache to deduplicate _collect_status across
        # near-simultaneous timer callbacks. TTL = half the shortest enabled
        # publish period.
        diag_period = float(self.get_parameter("diag_period_sec").value)
        diag_hz = 1.0 / diag_period if diag_period > 0 else 0.0
        enabled_hz = [
            h
            for h in (
                float(self.get_parameter("battery_status_hz").value),
                diag_hz,
            )
            if h > 0
        ]
        self._status_cache_ttl = 0.5 / max(enabled_hz) if enabled_hz else 0.0
        self._cached_status: Optional[Dict[str, Any]] = None
        self._cached_at: float = 0.0

        # ---- Transport ---------------------------------------------------
        self._transport = TcpJsonTransport(
            ip=self.get_parameter("quest_ip").value,
            tcp_port=int(self.get_parameter("tcp_port").value),
            sync_port=int(self.get_parameter("sync_port").value),
            on_frame=self._on_frame,
            logger=self.get_logger(),
        )

        # ---- Timers ------------------------------------------------------
        self._connect_timer = self.create_timer(
            float(self.get_parameter("connect_retry_sec").value),
            self._transport.try_connect,
        )
        self._diag_timer = self.create_timer(
            float(self.get_parameter("diag_period_sec").value),
            self._publish_diagnostics,
        )
        self._battery_status_timer = self._create_hz_timer(
            "battery_status_hz",
            self._publish_battery_status,
        )

        self.get_logger().info("quest_bridge started")

    def _create_hz_timer(self, hz_param: str, callback):
        hz = float(self.get_parameter(hz_param).value)
        if hz <= 0.0:
            self.get_logger().info(f"{hz_param}={hz}; timer disabled")
            return None
        return self.create_timer(1.0 / hz, callback)

    # ----------------------------------------------------------------------
    # Per-frame entry point invoked from the transport's recv thread.
    # ----------------------------------------------------------------------

    def _on_frame(self, frame: QuestFrame) -> None:
        # Replace (do not mutate) the snapshot reference so concurrent readers
        # already holding a reference keep seeing a consistent dict.
        with self._lock:
            self._latest_raw = frame.raw

        recv_stamp = self.get_clock().now().to_msg()
        self._publish_quest_messages(frame, recv_stamp)
        self._publish_tf_from_data(frame.raw, recv_stamp)
        self._publish_joy_from_data(frame.raw, recv_stamp)
        self._publish_body(frame, recv_stamp)

    # ----------------------------------------------------------------------
    # New per-sensor publishers (QuestHmd / QuestController).
    # ----------------------------------------------------------------------

    def _publish_quest_messages(self, frame: QuestFrame, recv_stamp) -> None:
        data = frame.raw

        hmd_pos = _get_xyz(data.get("hmdPosition", {}))
        hmd_rot = _get_xyzw(data.get("hmdRotation", {}))
        if hmd_pos is not None and hmd_rot is not None:
            ros_pos, ros_quat = _unity_pose_to_ros(hmd_pos, hmd_rot)
            msg = QuestHmd()
            msg.header.stamp = recv_stamp
            msg.header.frame_id = self._parent_frame
            msg.device_time_ns = int(frame.device_time_ns)
            msg.pc_monotonic_ns = int(frame.pc_monotonic_ns)
            msg.seq = int(frame.seq)
            msg.quest_id = frame.quest_id
            _fill_xyz(msg.position, ros_pos)
            _fill_xyzw(msg.rotation, ros_quat)
            self._hmd_pub.publish(msg)

        self._publish_controller(frame, recv_stamp, "left", self._left_ctrl_pub)
        self._publish_controller(frame, recv_stamp, "right", self._right_ctrl_pub)

    def _publish_controller(
        self,
        frame: QuestFrame,
        recv_stamp,
        side: str,
        publisher,
    ) -> None:
        data = frame.raw
        keys = _CONTROLLER_KEYS[side]

        pos = _get_xyz(data.get(keys["pos"], {}))
        rot = _get_xyzw(data.get(keys["rot"], {}))
        if pos is None or rot is None:
            # Skip when the protocol payload lacks this controller's pose; the
            # next sample with valid data will publish.
            return

        ros_pos, ros_quat = _unity_pose_to_ros(pos, rot)

        msg = QuestController()
        msg.header.stamp = recv_stamp
        msg.header.frame_id = self._parent_frame
        msg.device_time_ns = int(frame.device_time_ns)
        msg.pc_monotonic_ns = int(frame.pc_monotonic_ns)
        msg.seq = int(frame.seq)
        msg.quest_id = frame.quest_id

        _fill_xyz(msg.position, ros_pos)
        _fill_xyzw(msg.rotation, ros_quat)

        msg.tracked = bool(data.get(keys["tracked"], False))
        msg.valid = bool(data.get(keys["valid"], False))

        publisher.publish(msg)

        flag_pubs = self._controller_flag_pubs[side]
        tracked_msg = Bool()
        tracked_msg.data = msg.tracked
        flag_pubs["tracked"].publish(tracked_msg)
        valid_msg = Bool()
        valid_msg.data = msg.valid
        flag_pubs["valid"].publish(valid_msg)

    def _publish_body(self, frame: QuestFrame, recv_stamp) -> None:
        """Publish QuestBody, its fidelity mirror and SEW TF frames, if sent."""
        body = _parse_body(frame.raw)
        if body is None:
            return

        msg = QuestBody()
        msg.header.stamp = recv_stamp
        msg.header.frame_id = self._parent_frame
        msg.device_time_ns = int(frame.device_time_ns)
        msg.pc_monotonic_ns = int(frame.pc_monotonic_ns)
        msg.seq = int(frame.seq)
        msg.quest_id = frame.quest_id
        msg.body_time_s = body["time"]
        msg.confidence = body["confidence"]
        msg.fidelity = body["fidelity"]
        msg.calibration_status = body["calibration"]
        msg.skeleton_changed_count = body["skeleton_changed_count"]
        poses = body["poses"]
        msg.joint_names = [_body_joint_name(i) for i in range(len(poses))]
        msg.poses = [_make_pose(pos, quat) for pos, quat in poses]
        msg.location_flags = body["flags"]
        self._body_pub.publish(msg)

        fidelity_msg = UInt8()
        fidelity_msg.data = body["fidelity"]
        self._body_fidelity_pub.publish(fidelity_msg)

        if not self._publish_body_tf or not self._transport.connected:
            return
        transforms = []
        for index, child in self._body_tf_frames:
            if index >= len(poses):
                continue
            if body["flags"][index] & _BODY_POSE_VALID != _BODY_POSE_VALID:
                continue
            pos, quat = poses[index]
            t = TransformStamped()
            t.header.stamp = recv_stamp
            t.header.frame_id = self._parent_frame
            t.child_frame_id = child
            _fill_xyz(t.transform.translation, pos)
            _fill_xyzw(t.transform.rotation, quat)
            transforms.append(t)
        if transforms:
            self._tf_broadcaster.sendTransform(transforms)

    # ----------------------------------------------------------------------
    # Diagnostics / status publishing (unchanged behavior, sourced from
    # transport.metrics() instead of internal fields).
    # ----------------------------------------------------------------------

    def _collect_status(self) -> Dict[str, Any]:
        """Snapshot every status field with a single lock acquisition.

        ``_latest_raw`` is treated read-only after snapshot; the recv path
        replaces the dict reference rather than mutating in place, so holding
        a reference is safe.
        """
        with self._lock:
            latest = self._latest_raw

        m = self._transport.metrics()
        connected = bool(m["connected"])
        last_frame_age_s = float(m["last_frame_age_s"])
        fps = float(m["fps"])
        off_ns = int(m["offset_ns"])
        rtt_ns = m["rtt_ns"]

        # Streaming = connected and the last sample arrived within the last
        # second. NaN compares false, so missing samples mark streaming False.
        streaming = (
            connected
            and last_frame_age_s == last_frame_age_s
            and last_frame_age_s <= 1.0
        )

        offset_s = off_ns / 1e9
        rtt_ms = float("nan") if rtt_ns is None else rtt_ns / 1e6

        def _batt(key: str) -> int:
            if not latest:
                return -1
            val = latest.get(key, None)
            return -1 if val is None else int(val)

        def _float(key: str) -> float:
            if not latest:
                return float("nan")
            val = latest.get(key, None)
            return float("nan") if val is None else float(val)

        def _bool(key: str) -> bool:
            return bool(latest.get(key, False)) if latest else False

        # Only the scalar fields; the joints are not parsed at diag rate.
        body = latest.get("body") if latest else None
        body_fidelity = "none"
        body_confidence = float("nan")
        if isinstance(body, dict):
            try:
                body_fidelity = _BODY_FIDELITY_NAMES.get(
                    int(body.get("fidelity", 0)), "unknown"
                )
                body_confidence = float(body.get("confidence", float("nan")))
            except (TypeError, ValueError):
                body_fidelity = "unknown"

        return {
            "connected": connected,
            "streaming": bool(streaming),
            "fps": fps,
            "last_frame_age_s": last_frame_age_s,
            "offset_s": offset_s,
            "rtt_ms": rtt_ms,
            "left_tracked": _bool("leftTracked"),
            "right_tracked": _bool("rightTracked"),
            "left_valid": _bool("leftValid"),
            "right_valid": _bool("rightValid"),
            "delta_time_s": _float("deltaTime"),
            "hmd_batt_pct": _batt("hmdBattPct"),
            "left_batt_pct": _batt("leftBattPct"),
            "right_batt_pct": _batt("rightBattPct"),
            "hmd_charging": _bool("hmdCharging"),
            "body_fidelity": body_fidelity,
            "body_confidence": body_confidence,
        }

    def _get_status(self) -> Dict[str, Any]:
        """Return a recent snapshot, reusing the cached one within TTL.

        rclpy's default single-threaded executor serializes timer callbacks,
        so callbacks that expire in the same spin iteration share one
        snapshot without extra locking.
        """
        now = time.monotonic()
        if (
            self._cached_status is not None
            and (now - self._cached_at) < self._status_cache_ttl
        ):
            return self._cached_status
        self._cached_status = self._collect_status()
        self._cached_at = now
        return self._cached_status

    def _publish_battery_status(self):
        status = self._get_status()
        stamp = self.get_clock().now().to_msg()
        for name, pct_key, charging_key, _ in self._battery_specs:
            msg = BatteryState()
            msg.header.stamp = stamp
            msg.location = name
            msg.voltage = float("nan")
            msg.temperature = float("nan")
            msg.current = float("nan")
            msg.charge = float("nan")
            msg.capacity = float("nan")
            msg.design_capacity = float("nan")
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN

            pct = status[pct_key]
            if pct < 0:
                msg.percentage = float("nan")
                msg.present = False
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            else:
                msg.percentage = float(pct) / 100.0
                msg.present = True
                if charging_key is not None:
                    msg.power_supply_status = (
                        BatteryState.POWER_SUPPLY_STATUS_CHARGING
                        if status[charging_key]
                        else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                    )
                else:
                    msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            self._battery_pubs[name].publish(msg)

    def _publish_diagnostics(self):
        status = self._get_status()

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        st = DiagnosticStatus()
        st.name = "quest_bridge"
        st.hardware_id = self._hardware_id

        if not status["connected"]:
            st.level = DiagnosticStatus.ERROR
            st.message = "disconnected"
        elif not status["streaming"]:
            st.level = DiagnosticStatus.WARN
            st.message = "connected but not streaming"
        else:
            st.level = DiagnosticStatus.OK
            st.message = "streaming"

        st.values = [
            KeyValue(key=key, value=fmt(status[key])) for key, fmt in _DIAG_FIELDS
        ]

        arr.status.append(st)
        self._diag_pub.publish(arr)

    # ----------------------------------------------------------------------
    # TF + per-controller Joy publishing.
    # ----------------------------------------------------------------------

    def _publish_tf_from_data(self, data: Dict[str, Any], stamp) -> None:
        if not self._transport.connected:
            return

        hmd_pos = _get_xyz(data.get("hmdPosition", {}))
        hmd_rot = _get_xyzw(data.get("hmdRotation", {}))
        l_pos = _get_xyz(data.get("leftControllerPosition", {}))
        l_rot = _get_xyzw(data.get("leftControllerRotation", {}))
        r_pos = _get_xyz(data.get("rightControllerPosition", {}))
        r_rot = _get_xyzw(data.get("rightControllerRotation", {}))

        if hmd_pos and hmd_rot:
            ros_pos, ros_quat = _unity_pose_to_ros(hmd_pos, hmd_rot)
            self._send_tf(self._parent_frame, self._hmd_frame, stamp, ros_pos, ros_quat)
        if l_pos and l_rot:
            self._publish_hand_root_tf(
                self._parent_frame,
                self._left_frame,
                self._left_controller_frame,
                stamp,
                l_pos,
                l_rot,
            )
        if r_pos and r_rot:
            self._publish_hand_root_tf(
                self._parent_frame,
                self._right_frame,
                self._right_controller_frame,
                stamp,
                r_pos,
                r_rot,
            )

    def _send_tf(self, parent: str, child: str, stamp, ros_pos, ros_quat):
        """Broadcast a ROS-coordinate TF. Callers convert from Unity first."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        _fill_xyz(t.transform.translation, ros_pos)
        _fill_xyzw(t.transform.rotation, ros_quat)
        self._tf_broadcaster.sendTransform(t)

    def _publish_hand_root_tf(
        self,
        parent: str,
        hand_frame: str,
        controller_frame: str,
        stamp,
        controller_pos: Tuple[float, float, float],
        controller_quat: Tuple[float, float, float, float],
    ):
        if not hand_frame or not controller_frame:
            return
        offset = self._get_hand_controller_transform(hand_frame, controller_frame)
        if offset is None:
            return
        hand_to_controller_trans, hand_to_controller_quat = offset
        controller_to_hand_trans, controller_to_hand_quat = _invert_transform(
            hand_to_controller_trans,
            hand_to_controller_quat,
        )
        controller_ros_pos, controller_ros_quat = _unity_pose_to_ros(
            controller_pos, controller_quat
        )
        hand_trans, hand_quat = _compose_transforms(
            controller_ros_pos,
            controller_ros_quat,
            controller_to_hand_trans,
            controller_to_hand_quat,
        )
        self._send_tf(parent, hand_frame, stamp, hand_trans, hand_quat)

    def _get_hand_controller_transform(
        self,
        hand_frame: str,
        controller_frame: str,
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
        key = (hand_frame, controller_frame)
        if key in self._hand_controller_transforms:
            return self._hand_controller_transforms[key]

        try:
            tf_msg = self._tf_buffer.lookup_transform(
                hand_frame,
                controller_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            if key not in self._missing_offset_logged:
                self.get_logger().warn(
                    f"TF lookup failed for {hand_frame} <- {controller_frame}: {exc}"
                )
                self._missing_offset_logged.add(key)
            return None

        trans = (
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
        )
        quat = _normalize_quat(
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        )
        self._hand_controller_transforms[key] = (trans, quat)
        if key in self._missing_offset_logged:
            self._missing_offset_logged.remove(key)
            self.get_logger().info(
                f"TF offset resolved for {hand_frame} <- {controller_frame}"
            )
        return self._hand_controller_transforms[key]

    # Per-controller sensor_msgs/Joy axis / button layout. axes[2] (trigger) and
    # axes[3] (grip) are NaN under the current TCP/JSON protocol; they will
    # carry analog 0..1 values once the yubi_quest_app UDP transport lands.
    #
    # axes:
    #   [0] joystick_x
    #   [1] joystick_y
    #   [2] trigger_value
    #   [3] grip_value
    #
    # buttons:
    #   [0] primary_pressed   (X for left / A for right)
    #   [1] secondary_pressed (Y for left / B for right)
    #   [2] menu_pressed      (Start for left / Back for right)
    #   [3] thumbstick_clicked
    #   [4] trigger_pressed
    #   [5] grip_pressed
    #   [6] primary_touched
    #   [7] secondary_touched
    #   [8] thumbstick_touched
    #   [9] trigger_touched
    #   [10] thumb_rest_touched (always 0 under the current protocol)
    def _publish_joy_from_data(self, data: Dict[str, Any], stamp) -> None:
        if not self._transport.connected:
            return
        self._publish_controller_joy(data, stamp, "left", self._left_joy_pub)
        self._publish_controller_joy(data, stamp, "right", self._right_joy_pub)

    def _publish_controller_joy(
        self,
        data: Dict[str, Any],
        stamp,
        side: str,
        publisher,
    ) -> None:
        keys = _CONTROLLER_KEYS[side]
        joystick = data.get(keys["joystick"], {}) or {}

        joy = Joy()
        joy.header.stamp = stamp
        joy.axes = [
            float(joystick.get("x", 0.0) or 0.0),
            float(joystick.get("y", 0.0) or 0.0),
            float("nan"),
            float("nan"),
        ]
        joy.buttons = [
            int(bool(data.get(keys["primary_press"], False))),
            int(bool(data.get(keys["secondary_press"], False))),
            int(bool(data.get(keys["menu_press"], False))),
            int(bool(data.get(keys["thumb_click"], False))),
            int(bool(data.get(keys["trigger_press"], False))),
            int(bool(data.get(keys["grip_press"], False))),
            int(bool(data.get(keys["primary_touch"], False))),
            int(bool(data.get(keys["secondary_touch"], False))),
            int(bool(data.get(keys["thumb_touch"], False))),
            int(bool(data.get(keys["trigger_touch"], False))),
            0,
        ]
        publisher.publish(joy)

    # ----------------------------------------------------------------------
    # Shutdown.
    # ----------------------------------------------------------------------

    def destroy_node(self):
        try:
            self._transport.stop()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = QuestBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
