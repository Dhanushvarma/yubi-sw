"""Presence-driven node registry for yubi_bringup launch files.

Each entry maps a `yubi_devices.yaml` top-level key to a Node factory.
Launch files load common/<variant>/local yubi_devices.yaml, collect the set
of present keys, and spawn Nodes only for registry entries whose `yaml_key`
appears in that keyset.

There are two registries — kept aligned with the two launch files in this
package — so each launch only spawns its own concern:

- BRINGUP_NODE_REGISTRY     ← yubi_bringup.launch.py
    Hardware/IO nodes brought up regardless of whether data collection runs.
    (USB cameras, head camera, Quest bridge, encoder or trigger gripper.)

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
        # Variant metadata, not a node (see collect_yaml_keys).
        "yubi_variant",
    }
)

# A variant overlay can drop nodes that common/yubi_devices.yaml declares:
#
#   yubi_variant:
#     ros__parameters:
#       disabled_nodes: ["encoder_node", "left_camera/usb_cam"]
#
# The entry is shaped like a ROS param section so the file stays a valid
# --params-file; no node is named yubi_variant, so ROS ignores it.
VARIANT_META_KEY = "yubi_variant"


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


def _factory_quest_camera(params):
    from launch_ros.actions import Node

    return Node(
        package="airoa_quest_bridge",
        executable="quest_camera_node",
        name="quest_camera_node",
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


def _factory_trigger_gripper(params):
    from launch_ros.actions import Node

    return Node(
        package="yubi_bringup",
        executable="trigger_gripper_node",
        name="trigger_gripper_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=params,
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


BRINGUP_NODE_REGISTRY = [
    {"yaml_key": "left_camera/usb_cam", "factory": _factory_usb_cam("left_camera")},
    {"yaml_key": "right_camera/usb_cam", "factory": _factory_usb_cam("right_camera")},
    {"yaml_key": "center_camera/usb_cam", "factory": _factory_usb_cam("center_camera")},
    {"yaml_key": "/camera/camera", "factory": _factory_realsense},
    {"yaml_key": "quest_bridge_node", "factory": _factory_quest_bridge},
    {"yaml_key": "quest_camera_node", "factory": _factory_quest_camera},
    {"yaml_key": "encoder_node", "factory": _factory_encoder},
    {"yaml_key": "trigger_gripper_node", "factory": _factory_trigger_gripper},
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
    """Return the union of top-level keys from given YAML files. Missing files are skipped.

    Keys listed in any file's `yubi_variant.ros__parameters.disabled_nodes` are
    removed from the result, as is the `yubi_variant` key itself.
    """
    keys: set[str] = set()
    disabled: set[str] = set()
    for path in yaml_paths:
        p = Path(path)
        if not p.exists():
            continue
        with p.open("r") as f:
            data = yaml.safe_load(f) or {}
        if isinstance(data, dict):
            keys.update(data.keys())
            meta = (data.get(VARIANT_META_KEY) or {}).get("ros__parameters") or {}
            disabled.update(meta.get("disabled_nodes") or [])
    return keys - disabled - {VARIANT_META_KEY}


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
