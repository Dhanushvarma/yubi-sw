#!/usr/bin/env python3
"""Yubi bringup launch — presence-driven hardware node selection.

Loads `common/yubi_devices.yaml` and `<variant>/yubi_devices.yaml` (plus
optional `local/yubi_devices.yaml`) and spawns only those BRINGUP_NODE_REGISTRY
entries whose `yaml_key` appears in the merged keyset.

Variant defaults to env var `ROBOT_VARIANT` (else `stationary`), overridable
via the `robot_variant` launch argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import xacro

from yubi_bringup.launch_registry import (
    BRINGUP_NODE_REGISTRY,
    collect_yaml_keys,
    select_nodes,
)


def variant_yaml_paths(context, share=None):
    """Resolve (common, variant, local) yubi_devices.yaml paths from current context."""
    if share is None:
        share = Path(FindPackageShare("yubi_bringup").find("yubi_bringup"))
    variant = context.perform_substitution(LaunchConfiguration("robot_variant"))
    return (
        share / "config" / "common" / "yubi_devices.yaml",
        share / "config" / variant / "yubi_devices.yaml",
        share / "config" / "local" / "yubi_devices.yaml",
    )


def _spawn_bringup_nodes(context, *_args, **_kwargs):
    paths = variant_yaml_paths(context)
    keys = collect_yaml_keys(paths)
    # ROS 2 multi-params: later files override earlier per-key.
    params = [str(p) for p in paths if p.exists()]
    return select_nodes(BRINGUP_NODE_REGISTRY, keys, params)


def generate_launch_description():
    description_share = Path(
        FindPackageShare("yubi_description").find("yubi_description")
    )
    urdf_file = description_share / "urdf" / "yubi_hand.urdf.xacro"
    rviz_config = description_share / "rviz" / "yubi_display.rviz"
    robot_description = {
        "robot_description": xacro.process_file(str(urdf_file)).toxml()
    }

    # Structural nodes (always launched, no YAML config requirement)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="yubi_robot_state_publisher",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[robot_description],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="yubi_bringup_rviz",
        arguments=["-d", str(rviz_config)],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_variant",
                default_value=EnvironmentVariable(
                    "ROBOT_VARIANT", default_value="stationary"
                ),
                description=(
                    "Robot variant: stationary | portable | quest_only "
                    "(selects config/<variant>/...)"
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz2 visualization",
            ),
            robot_state_publisher,
            rviz,
            OpaqueFunction(function=_spawn_bringup_nodes),
        ]
    )
