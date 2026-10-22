"""Unit tests for ``airoa_quest_bridge.quest_bridge_node``.

Covers:
    * Pure-math helpers (Unity <-> ROS conversion, quaternion ops).
    * Field extraction helpers (``_get_xyz`` / ``_get_xyzw``).
    * The new per-sensor publishing path (``QuestHmd`` / ``QuestController``)
      driven through a synthetic ``QuestFrame``.
"""

from __future__ import annotations

import math

import pytest


# ---------------------------------------------------------------------------
# Module loader
# ---------------------------------------------------------------------------


@pytest.fixture()
def node_module(mock_ros):
    """Import the bridge node after the ROS mocks are installed."""
    import airoa_quest_bridge.quest_bridge_node as mod

    return mod


# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------


def test_unity_pos_to_ros_axis_mapping(node_module):
    """Unity (X right, Y up, Z forward) -> ROS (X forward, Y left, Z up)."""
    assert node_module._unity_pos_to_ros(0.0, 0.0, 1.0) == (1.0, 0.0, 0.0)
    assert node_module._unity_pos_to_ros(1.0, 0.0, 0.0) == (0.0, -1.0, 0.0)
    assert node_module._unity_pos_to_ros(0.0, 1.0, 0.0) == (0.0, 0.0, 1.0)


def test_normalize_quat_makes_unit_length(node_module):
    qx, qy, qz, qw = node_module._normalize_quat(2.0, 0.0, 0.0, 2.0)
    assert qx**2 + qy**2 + qz**2 + qw**2 == pytest.approx(1.0)


def test_normalize_quat_zero_returns_identity(node_module):
    """A zero-norm input must not produce NaN; identity is the safe fallback."""
    assert node_module._normalize_quat(0.0, 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0, 1.0)


def test_unity_quat_to_ros_sign_pattern(node_module):
    """Unity (left-handed) -> ROS (right-handed): (x,y,z,w) -> (z,-x,y,-w)."""
    assert node_module._unity_quat_to_ros(1.0, 2.0, 3.0, 4.0) == (3.0, -1.0, 2.0, -4.0)


def test_unity_pose_to_ros_returns_normalized_quat(node_module):
    pos, quat = node_module._unity_pose_to_ros((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 2.0))
    assert pos == (3.0, -1.0, 2.0)
    assert sum(c * c for c in quat) == pytest.approx(1.0)


def test_quat_multiply_identity_left_and_right(node_module):
    q = (0.1, 0.2, 0.3, math.sqrt(1.0 - 0.14))
    identity = (0.0, 0.0, 0.0, 1.0)
    left = node_module._quat_multiply(identity, q)
    right = node_module._quat_multiply(q, identity)
    for got in (left, right):
        for a, b in zip(got, q):
            assert a == pytest.approx(b, abs=1e-6)


def test_invert_then_compose_yields_identity_pose(node_module):
    """Composing a transform with its inverse should recover the identity."""
    parent_t = (0.5, -0.3, 1.2)
    parent_q = (0.0, 0.0, 0.0, 1.0)
    inv_t, inv_q = node_module._invert_transform(parent_t, parent_q)

    composed_t, composed_q = node_module._compose_transforms(
        parent_t, parent_q, inv_t, inv_q
    )
    for v in composed_t:
        assert v == pytest.approx(0.0, abs=1e-6)
    assert composed_q == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-6)


# ---------------------------------------------------------------------------
# Field extraction helpers
# ---------------------------------------------------------------------------


def test_get_xyz_parses_well_formed_dict(node_module):
    assert node_module._get_xyz({"x": 1, "y": 2, "z": 3}) == (1.0, 2.0, 3.0)


def test_get_xyz_returns_none_on_missing_or_bad_input(node_module):
    assert node_module._get_xyz(None) is None
    assert node_module._get_xyz({}) is None
    assert node_module._get_xyz({"x": "not numeric", "y": 0, "z": 0}) is None


def test_get_xyzw_parses_quaternion_dict(node_module):
    assert node_module._get_xyzw({"x": 0, "y": 0, "z": 0, "w": 1}) == (
        0.0,
        0.0,
        0.0,
        1.0,
    )


def test_get_xyzw_returns_none_on_missing_w(node_module):
    assert node_module._get_xyzw({"x": 0, "y": 0, "z": 0}) is None


# ---------------------------------------------------------------------------
# Controller-key map symmetry
# ---------------------------------------------------------------------------


def test_controller_key_map_left_and_right_share_field_set(node_module):
    """Both sides must expose the same set of logical fields."""
    assert set(node_module._CONTROLLER_KEYS["left"].keys()) == set(
        node_module._CONTROLLER_KEYS["right"].keys()
    )


def test_controller_key_map_does_not_mix_sides(node_module):
    """Sanity: a left-side key should never reference a right-side raw field."""
    for raw_key in node_module._CONTROLLER_KEYS["left"].values():
        assert "right" not in raw_key.lower()
    for raw_key in node_module._CONTROLLER_KEYS["right"].values():
        assert "left" not in raw_key.lower()


# ---------------------------------------------------------------------------
# Per-sensor publish path
# ---------------------------------------------------------------------------


def _make_node(node_module):
    """Construct a ``QuestBridgeNode`` against the mocked ROS layer."""
    return node_module.QuestBridgeNode()


def _make_frame(node_module, **overrides):
    """Build a synthetic ``QuestFrame`` with a typical full payload."""
    from airoa_quest_bridge.transport.tcp_json import QuestFrame

    raw = {
        "ovrTimeNs": 1_500_000_000,
        "deltaTime": 0.02,
        "hmdPosition": {"x": 0.1, "y": 0.2, "z": 0.3},
        "hmdRotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "leftControllerPosition": {"x": -0.2, "y": 1.0, "z": 0.5},
        "leftControllerRotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "rightControllerPosition": {"x": 0.2, "y": 1.0, "z": 0.5},
        "rightControllerRotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "leftJoystick": {"x": 0.4, "y": -0.5},
        "rightJoystick": {"x": -0.1, "y": 0.2},
        "leftTracked": True,
        "leftValid": True,
        "rightTracked": True,
        "rightValid": False,
        "leftThumbstickClick": True,
        "leftThumbstickTouched": True,
        "rightThumbstickClick": False,
        "rightThumbstickTouched": True,
        "leftTriggerPressed": True,
        "leftIndexTriggerTouched": True,
        "rightTriggerPressed": False,
        "rightIndexTriggerTouched": True,
        "leftGripPressed": True,
        "rightGripPressed": False,
        "buttonXPressed": True,
        "buttonXTouched": True,
        "buttonYPressed": False,
        "buttonYTouched": True,
        "buttonAPressed": True,
        "buttonATouched": False,
        "buttonBPressed": True,
        "buttonBTouched": True,
        "startPressed": True,
        "backPressed": False,
        "hmdBattPct": 80,
        "leftBattPct": 50,
        "rightBattPct": 60,
        "hmdCharging": False,
    }
    raw.update(overrides.get("raw_overrides", {}))

    return QuestFrame(
        device_time_ns=overrides.get("device_time_ns", 1_500_000_000),
        pc_monotonic_ns=overrides.get("pc_monotonic_ns", 9_000_000_000),
        seq=overrides.get("seq", 0),
        quest_id=overrides.get("quest_id", ""),
        delta_time_s=overrides.get("delta_time_s", 0.02),
        raw=raw,
    )


def test_publish_quest_messages_emits_one_per_sensor(node_module):
    node = _make_node(node_module)
    frame = _make_frame(node_module)

    node._publish_quest_messages(frame, recv_stamp=None)

    assert len(node.published["/quest/hmd/state"]) == 1
    assert len(node.published["/quest/controller/left/state"]) == 1
    assert len(node.published["/quest/controller/right/state"]) == 1


def test_quest_hmd_message_fields_populated_correctly(node_module):
    node = _make_node(node_module)
    frame = _make_frame(node_module)

    node._publish_quest_messages(frame, recv_stamp="STAMP")
    hmd = node.published["/quest/hmd/state"][0]

    assert hmd.header.stamp == "STAMP"
    assert hmd.header.frame_id == "quest_origin"
    assert hmd.device_time_ns == 1_500_000_000
    assert hmd.pc_monotonic_ns == 9_000_000_000
    assert hmd.seq == 0
    assert hmd.quest_id == ""
    # Unity (0.1, 0.2, 0.3) -> ROS (0.3, -0.1, 0.2).
    assert hmd.position.x == pytest.approx(0.3)
    assert hmd.position.y == pytest.approx(-0.1)
    assert hmd.position.z == pytest.approx(0.2)
    # Identity quaternion in Unity (0,0,0,1) -> ROS (0,0,0,-1) is normalized
    # but still represents the same rotation; w should be unit length.
    assert (
        hmd.rotation.x**2 + hmd.rotation.y**2 + hmd.rotation.z**2 + hmd.rotation.w**2
    ) == pytest.approx(1.0)


def test_quest_left_controller_message_carries_pose_and_tracking(node_module):
    node = _make_node(node_module)
    frame = _make_frame(node_module)

    node._publish_quest_messages(frame, recv_stamp=None)
    left = node.published["/quest/controller/left/state"][0]

    assert left.tracked is True
    assert left.valid is True
    # Pose fields are populated (Unity -> ROS conversion exercised elsewhere).
    assert (
        left.rotation.x**2
        + left.rotation.y**2
        + left.rotation.z**2
        + left.rotation.w**2
    ) == pytest.approx(1.0)


def test_quest_right_controller_message_carries_pose_and_tracking(node_module):
    node = _make_node(node_module)
    frame = _make_frame(node_module)

    node._publish_quest_messages(frame, recv_stamp=None)
    right = node.published["/quest/controller/right/state"][0]

    assert right.tracked is True
    assert right.valid is False


def test_publish_quest_messages_skips_when_pose_missing(node_module):
    """If a controller's pose is absent we should silently skip its publish."""
    node = _make_node(node_module)
    frame = _make_frame(
        node_module,
        raw_overrides={
            "leftControllerPosition": None,
            "leftControllerRotation": None,
        },
    )

    node._publish_quest_messages(frame, recv_stamp=None)

    assert len(node.published["/quest/controller/left/state"]) == 0
    assert len(node.published["/quest/controller/right/state"]) == 1
    # Skipped controllers must not emit stale tracked/valid Bool either.
    assert len(node.published["/quest/controller/left/tracked"]) == 0
    assert len(node.published["/quest/controller/left/valid"]) == 0
    assert len(node.published["/quest/controller/right/tracked"]) == 1
    assert len(node.published["/quest/controller/right/valid"]) == 1


def test_publish_quest_messages_emits_tracked_and_valid_bool(node_module):
    node = _make_node(node_module)
    frame = _make_frame(node_module)

    node._publish_quest_messages(frame, recv_stamp=None)

    # leftTracked=True, leftValid=True, rightTracked=True, rightValid=False
    # in the canonical synthetic frame.
    assert node.published["/quest/controller/left/tracked"][0].data is True
    assert node.published["/quest/controller/left/valid"][0].data is True
    assert node.published["/quest/controller/right/tracked"][0].data is True
    assert node.published["/quest/controller/right/valid"][0].data is False
    assert len(node.published["/quest/hmd/state"]) == 1


def test_publish_quest_messages_propagates_seq_and_quest_id(node_module):
    """When the protocol provides them, ``seq`` and ``quest_id`` flow through."""
    node = _make_node(node_module)
    frame = _make_frame(
        node_module, seq=42, quest_id="abcd-1234", device_time_ns=999, pc_monotonic_ns=1
    )

    node._publish_quest_messages(frame, recv_stamp=None)

    hmd = node.published["/quest/hmd/state"][0]
    assert hmd.seq == 42
    assert hmd.quest_id == "abcd-1234"
    assert hmd.device_time_ns == 999
    assert hmd.pc_monotonic_ns == 1

    for side in (
        "/quest/controller/left/state",
        "/quest/controller/right/state",
    ):
        msg = node.published[side][0]
        assert msg.seq == 42
        assert msg.quest_id == "abcd-1234"
        assert msg.device_time_ns == 999
        assert msg.pc_monotonic_ns == 1


# ---------------------------------------------------------------------------
# Per-controller sensor_msgs/Joy publishing
# ---------------------------------------------------------------------------


def test_publish_joy_emits_one_per_controller(node_module):
    node = _make_node(node_module)
    node._transport._connected = True
    frame = _make_frame(node_module)

    node._publish_joy_from_data(frame.raw, stamp="STAMP")

    assert len(node.published["/quest/controller/left/joy"]) == 1
    assert len(node.published["/quest/controller/right/joy"]) == 1


def test_publish_joy_left_axes_and_buttons(node_module):
    """Left Joy carries left thumbstick, left triggers/buttons in fixed order."""
    node = _make_node(node_module)
    node._transport._connected = True
    frame = _make_frame(node_module)

    node._publish_joy_from_data(frame.raw, stamp="STAMP")
    left = node.published["/quest/controller/left/joy"][0]

    assert left.header.stamp == "STAMP"
    assert left.axes[0] == pytest.approx(0.4)  # leftJoystick.x
    assert left.axes[1] == pytest.approx(-0.5)  # leftJoystick.y
    # Analog trigger / grip values are NaN when the app does not send them.
    assert math.isnan(left.axes[2])
    assert math.isnan(left.axes[3])
    # Buttons in the order documented next to _publish_controller_joy:
    # primary=X, secondary=Y, menu=Start, thumb_click, trigger_press,
    # grip_press, primary_touch, secondary_touch, thumb_touch, trigger_touch,
    # thumb_rest_touched.
    assert left.buttons == [
        1,  # buttonXPressed
        0,  # buttonYPressed
        1,  # startPressed (menu)
        1,  # leftThumbstickClick
        1,  # leftTriggerPressed
        1,  # leftGripPressed
        1,  # buttonXTouched
        1,  # buttonYTouched
        1,  # leftThumbstickTouched
        1,  # leftIndexTriggerTouched
        0,  # thumb_rest_touched (always 0 under current protocol)
    ]


def test_publish_joy_right_axes_and_buttons(node_module):
    """Right Joy uses A/B/Back semantics and the right-side raw keys."""
    node = _make_node(node_module)
    node._transport._connected = True
    frame = _make_frame(node_module)

    node._publish_joy_from_data(frame.raw, stamp="STAMP")
    right = node.published["/quest/controller/right/joy"][0]

    assert right.axes[0] == pytest.approx(-0.1)  # rightJoystick.x
    assert right.axes[1] == pytest.approx(0.2)  # rightJoystick.y
    assert math.isnan(right.axes[2])
    assert math.isnan(right.axes[3])
    assert right.buttons == [
        1,  # buttonAPressed
        1,  # buttonBPressed
        0,  # backPressed (menu)
        0,  # rightThumbstickClick
        0,  # rightTriggerPressed
        0,  # rightGripPressed
        0,  # buttonATouched
        1,  # buttonBTouched
        1,  # rightThumbstickTouched
        1,  # rightIndexTriggerTouched
        0,
    ]


def test_publish_joy_analog_trigger_and_grip_values(node_module):
    """Apps that send {left,right}{Trigger,Grip}Value fill axes[2] / axes[3]."""
    node = _make_node(node_module)
    node._transport._connected = True
    frame = _make_frame(
        node_module,
        raw_overrides={
            "leftTriggerValue": 0.75,
            "leftGripValue": 0.25,
            "rightTriggerValue": 1,
            "rightGripValue": "bogus",
        },
    )

    node._publish_joy_from_data(frame.raw, stamp="STAMP")
    left = node.published["/quest/controller/left/joy"][0]
    right = node.published["/quest/controller/right/joy"][0]

    assert left.axes[2] == pytest.approx(0.75)
    assert left.axes[3] == pytest.approx(0.25)
    assert right.axes[2] == pytest.approx(1.0)
    assert math.isnan(right.axes[3])
