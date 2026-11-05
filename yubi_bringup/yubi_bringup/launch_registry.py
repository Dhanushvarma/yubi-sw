"""Presence-driven node registry for yubi_bringup launch files.

Each entry maps a `yubi_devices.yaml` top-level key to a Node factory.
Launch files load common/<variant>/local yubi_devices.yaml, collect the set
of present keys, and spawn Nodes only for registry entries whose `yaml_key`
appears in that keyset.

There are two registries — kept aligned with the two launch files in this
package — so each launch only spawns its own concern:

- BRINGUP_NODE_REGISTRY     ← yubi_bringup.launch.py
    Hardware/IO nodes brought up regardless of whether data collection runs.
    (USB cameras, head camera, Quest bridge, encoder, static mount TFs.)

- DATA_COLLECTION_NODE_REGISTRY ← yubi_data_collection.launch.py
    Nodes specific to data-collection runs (footpedal, dispatch).

Adding a new hardware node:
  1. Append an entry to the appropriate registry with the matching `yaml_key`.
  2. Add the YAML section to common/<variant>/yubi_devices.yaml.

Structural nodes (robot_state_publisher, rosbridge_websocket) are launched
unconditionally by the launch files themselves; they are listed in
STRUCTURAL_NODE_KEYS so the test_node_registry consistency test ignores them.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import yaml

# ROS 2 launch / launch_ros are imported lazily inside the factories so this
# module is importable host-side (for pytest) without a ROS 2 environment.


# Top-level keys that the consistency test should NOT require to appear in the
# registry. These nodes are spawned by launch-file code outside the YAML-driven
# path (URDF-only, fixed-port services, dev-only tooling).
STRUCTURAL_NODE_KEYS = frozenset(
    {
        # No YAML keys for these in yubi_devices.yaml today.
    }
)


def _factory_usb_cam(namespace: str):
    def factory(params):
        from launch_ros.actions import Node  # ROS 2 runtime-only

        return Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            namespace=namespace,
            name="usb_cam",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=params,
        )

    return factory


def _factory_quest_bridge(params):
    from launch_ros.actions import Node

    return Node(
        package="airoa_quest_bridge",
        executable="quest_bridge_node",
        name="quest_bridge_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=params,
    )


def _factory_encoder(params):
    from launch_ros.actions import Node

    calibration = Path("/etc/yubi/encoder_limits.yaml")
    encoder_params = list(params)
    if calibration.exists():
        encoder_params.append(str(calibration))
    return Node(
        package="yubi_bringup",
        executable="encoder_node",
        name="encoder_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=encoder_params,
    )


def _factory_realsense(params):
    from launch_ros.actions import Node

    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=params,
    )


def _factory_footpedal(_params):
    from launch_ros.actions import Node

    # footpedal_node takes no parameters; presence-driven via a marker YAML
    # entry (`footpedal_node: { ros__parameters: {} }`) in the variant overlay.
    return Node(
        package="footpedal_ros",
        executable="footpedal_node",
        name="footpedal_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )


def _factory_task_command_dispatch(params):
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    return Node(
        package="yubi_core",
        executable="task_command_dispatch_node",
        name="task_command_dispatch_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=params,
        remappings=[("/joy", LaunchConfiguration("joy_remap_topic"))],
    )


def _factory_gripper_double_click(params):
    from launch_ros.actions import Node

    return Node(
        package="yubi_bringup",
        executable="gripper_double_click_node",
        name="gripper_double_click_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=params,
    )


def _factory_portable_joy_command(params):
    from launch_ros.actions import Node

    return Node(
        package="yubi_bringup",
        executable="portable_joy_command_node",
        name="portable_joy_command_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=params,
    )


def _merged_ros_params(param_files: Iterable[Path | str], yaml_key: str) -> dict:
    """Merge `<yaml_key>.ros__parameters` across param files; later files win."""
    merged: dict = {}
    for path in param_files:
        p = Path(path)
        if not p.exists():
            continue
        with p.open("r") as f:
            data = yaml.safe_load(f) or {}
        section = data.get(yaml_key) if isinstance(data, dict) else None
        if isinstance(section, dict):
            merged.update(section.get("ros__parameters") or {})
    return merged


_STATIC_TF_POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def static_tf_arguments(param_files: Iterable[Path | str], yaml_key: str) -> list[str]:
    """Command line for tf2_ros static_transform_publisher from a YAML section.

    The section's ros__parameters hold parent_frame, child_frame and the pose
    of the child in the parent: x, y, z (m) and roll, pitch, yaw (rad), each
    defaulting to 0. static_transform_publisher reads only its command line,
    so the values are turned into arguments here.
    """
    params = _merged_ros_params(param_files, yaml_key)
    missing = [k for k in ("parent_frame", "child_frame") if not params.get(k)]
    if missing:
        raise ValueError(f"{yaml_key}: missing {', '.join(missing)}")
    args: list[str] = []
    for key in _STATIC_TF_POSE_KEYS:
        args += [f"--{key}", repr(float(params.get(key, 0.0)))]
    args += [
        "--frame-id",
        str(params["parent_frame"]),
        "--child-frame-id",
        str(params["child_frame"]),
    ]
    return args


def _factory_static_tf(yaml_key: str):
    def factory(params):
        from launch_ros.actions import Node

        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=yaml_key,
            output="screen",
            arguments=static_tf_arguments(params, yaml_key),
        )

    return factory


BRINGUP_NODE_REGISTRY = [
    {"yaml_key": "left_camera/usb_cam", "factory": _factory_usb_cam("left_camera")},
    {"yaml_key": "right_camera/usb_cam", "factory": _factory_usb_cam("right_camera")},
    {"yaml_key": "center_camera/usb_cam", "factory": _factory_usb_cam("center_camera")},
    {"yaml_key": "/camera/camera", "factory": _factory_realsense},
    {"yaml_key": "quest_bridge_node", "factory": _factory_quest_bridge},
    {"yaml_key": "encoder_node", "factory": _factory_encoder},
    # Head camera rigidly mounted on the Quest (portable_head).
    {
        "yaml_key": "center_camera_mount_tf",
        "factory": _factory_static_tf("center_camera_mount_tf"),
    },
    {
        "yaml_key": "center_camera_optical_tf",
        "factory": _factory_static_tf("center_camera_optical_tf"),
    },
]


DATA_COLLECTION_NODE_REGISTRY = [
    {"yaml_key": "footpedal_node", "factory": _factory_footpedal},
    {
        "yaml_key": "task_command_dispatch_node",
        "factory": _factory_task_command_dispatch,
    },
    {
        "yaml_key": "gripper_double_click_node",
        "factory": _factory_gripper_double_click,
    },
    {
        "yaml_key": "portable_joy_command_node",
        "factory": _factory_portable_joy_command,
    },
]


# Combined view for consistency tests.
NODE_REGISTRY = BRINGUP_NODE_REGISTRY + DATA_COLLECTION_NODE_REGISTRY


def collect_yaml_keys(yaml_paths: Iterable[Path | str]) -> set[str]:
    """Return the union of top-level keys from given YAML files. Missing files are skipped."""
    keys: set[str] = set()
    for path in yaml_paths:
        p = Path(path)
        if not p.exists():
            continue
        with p.open("r") as f:
            data = yaml.safe_load(f) or {}
        if isinstance(data, dict):
            keys.update(data.keys())
    return keys


def select_nodes(
    registry: list[dict],
    yaml_keys_present: set[str],
    params: list,
) -> list:
    """Instantiate Node objects for `registry` entries whose yaml_key is present.

    Return type is list[launch_ros.actions.Node] but unannotated to keep this
    module importable host-side without ROS 2 launch installed.
    """
    return [
        entry["factory"](params)
        for entry in registry
        if entry["yaml_key"] in yaml_keys_present
    ]
