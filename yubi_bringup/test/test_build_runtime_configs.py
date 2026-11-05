"""Unit tests for the variant config refactor.

Covers:
- TestDeepMerge        — merge engine semantics
- TestBuild            — build pipeline (tmp dirs, edge cases, idempotency)
- TestNoOpRefactor     — stationary _runtime/ matches the pre-refactor legacy
                         single-file snapshots in test/fixtures/legacy_stationary/
- TestPortableInvariants — structure assertions for portable variant outputs
- TestNodeRegistry     — registry <-> YAML key bidirectional consistency
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
YUBI_BRINGUP = REPO_ROOT / "yubi_bringup"
CONFIG_ROOT = YUBI_BRINGUP / "config"
FIXTURES = Path(__file__).parent / "fixtures" / "legacy_stationary"

# Make tools/ importable as a flat module, and inner package importable.
for path in (str(YUBI_BRINGUP / "tools"), str(YUBI_BRINGUP)):
    if path not in sys.path:
        sys.path.insert(0, path)


def _import_merger():
    # Re-import each call to avoid stale module state in long pytest sessions.
    if "build_runtime_configs" in sys.modules:
        importlib.reload(sys.modules["build_runtime_configs"])
    return importlib.import_module("build_runtime_configs")


merger = _import_merger()


# ---------------------------------------------------------------------------
# TestDeepMerge — merge engine semantics
# ---------------------------------------------------------------------------
class TestDeepMerge:
    def test_dict_recursive_merge(self):
        assert merger.deep_merge({"a": 1, "b": {"c": 2}}, {"b": {"d": 3}}) == {
            "a": 1,
            "b": {"c": 2, "d": 3},
        }

    def test_scalar_override(self):
        assert merger.deep_merge({"a": 1}, {"a": 2}) == {"a": 2}

    def test_list_replace(self):
        assert merger.deep_merge({"xs": [1, 2]}, {"xs": [3]}) == {"xs": [3]}

    def test_extend_appends_unique(self):
        assert merger.deep_merge({"xs": [1, 2]}, {"xs": {"__extend__": [3, 4]}}) == {
            "xs": [1, 2, 3, 4]
        }

    def test_extend_deduplicates(self):
        assert merger.deep_merge({"xs": [1, 2]}, {"xs": {"__extend__": [2, 3]}}) == {
            "xs": [1, 2, 3]
        }

    def test_extend_nested(self):
        assert merger.deep_merge(
            {"a": {"xs": [1]}}, {"a": {"xs": {"__extend__": [2, 3]}}}
        ) == {"a": {"xs": [1, 2, 3]}}

    def test_overlay_none_replaces(self):
        # Overlay None replaces (None is a valid YAML value, treated as scalar).
        assert merger.deep_merge({"a": {"b": 1}}, {"a": None}) == {"a": None}

    def test_extend_requires_list_value(self):
        with pytest.raises(ValueError):
            merger.deep_merge([1, 2], {"__extend__": "not-a-list"})


# ---------------------------------------------------------------------------
# TestBuild — build pipeline
# ---------------------------------------------------------------------------
def _write_yaml(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        yaml.safe_dump(data, f)


class TestBuild:
    def _setup_minimal_tree(self, root: Path) -> None:
        # Only robot_config.yaml is exercised here for brevity; the merger
        # iterates MERGED_FILES with the same logic. recording_gate / upload_targets
        # use the same merge engine so additional coverage would be redundant.
        for filename in merger.MERGED_FILES:
            _write_yaml(root / "common" / filename, {"common_key": "base"})

    def test_common_only_passthrough(self, tmp_path):
        self._setup_minimal_tree(tmp_path)
        written = merger.build(
            config_root=tmp_path, variant="stationary", with_local=False
        )
        for path in written:
            with path.open() as f:
                data = yaml.safe_load(f)
            assert data == {"common_key": "base"}

    def test_variant_overlay(self, tmp_path):
        self._setup_minimal_tree(tmp_path)
        for filename in merger.MERGED_FILES:
            _write_yaml(tmp_path / "portable" / filename, {"variant_key": "portable"})
        merger.build(config_root=tmp_path, variant="portable", with_local=False)
        with (tmp_path / "_runtime" / "portable" / "robot_config.yaml").open() as f:
            data = yaml.safe_load(f)
        assert data == {"common_key": "base", "variant_key": "portable"}

    def test_local_layer(self, tmp_path):
        self._setup_minimal_tree(tmp_path)
        for filename in merger.MERGED_FILES:
            _write_yaml(tmp_path / "stationary" / filename, {"variant_key": "stat"})
            _write_yaml(tmp_path / "local" / filename, {"local_key": "host"})
        merger.build(config_root=tmp_path, variant="stationary", with_local=True)
        with (tmp_path / "_runtime" / "stationary" / "robot_config.yaml").open() as f:
            data = yaml.safe_load(f)
        assert data == {
            "common_key": "base",
            "variant_key": "stat",
            "local_key": "host",
        }

    def test_local_skipped_without_flag(self, tmp_path):
        self._setup_minimal_tree(tmp_path)
        for filename in merger.MERGED_FILES:
            _write_yaml(tmp_path / "local" / filename, {"local_key": "host"})
        merger.build(config_root=tmp_path, variant="stationary", with_local=False)
        with (tmp_path / "_runtime" / "stationary" / "robot_config.yaml").open() as f:
            data = yaml.safe_load(f)
        assert "local_key" not in data

    def test_missing_common_raises(self, tmp_path):
        # No common/ at all
        with pytest.raises(FileNotFoundError):
            merger.build(config_root=tmp_path, variant="stationary", with_local=False)

    def test_idempotent(self, tmp_path):
        self._setup_minimal_tree(tmp_path)
        for filename in merger.MERGED_FILES:
            _write_yaml(tmp_path / "stationary" / filename, {"variant_key": "stat"})
        merger.build(config_root=tmp_path, variant="stationary", with_local=False)
        first = (
            tmp_path / "_runtime" / "stationary" / "robot_config.yaml"
        ).read_bytes()
        merger.build(config_root=tmp_path, variant="stationary", with_local=False)
        second = (
            tmp_path / "_runtime" / "stationary" / "robot_config.yaml"
        ).read_bytes()
        assert first == second

    def test_record_topics_extend(self, tmp_path):
        """Real-world test: record_topics list extended by variant overlay."""
        _write_yaml(
            tmp_path / "common" / "robot_config.yaml",
            {"/**": {"ros__parameters": {"record_topics": ["/a", "/b"]}}},
        )
        _write_yaml(
            tmp_path / "common" / "recording_gate.yaml",
            {},
        )
        _write_yaml(
            tmp_path / "common" / "upload_targets.yaml",
            {},
        )
        _write_yaml(
            tmp_path / "stationary" / "robot_config.yaml",
            {
                "/**": {
                    "ros__parameters": {
                        "robot_type": "test",
                        "record_topics": {"__extend__": ["/c", "/d"]},
                    }
                }
            },
        )
        merger.build(config_root=tmp_path, variant="stationary", with_local=False)
        with (tmp_path / "_runtime" / "stationary" / "robot_config.yaml").open() as f:
            data = yaml.safe_load(f)
        rp = data["/**"]["ros__parameters"]
        assert rp["robot_type"] == "test"
        assert rp["record_topics"] == ["/a", "/b", "/c", "/d"]


# ---------------------------------------------------------------------------
# TestNoOpRefactor — refactor preserves stationary semantics
# ---------------------------------------------------------------------------
class TestNoOpRefactor:
    """Verify that build_runtime_configs.py --variant stationary produces
    output semantically identical to the pre-refactor single YAML files.

    record_topics is compared as a multiset because the refactor reorders the
    list (trailing topics in common; head-camera topics appended via __extend__).
    Order is irrelevant for rosbag2 recording behavior.
    """

    @pytest.fixture(scope="class")
    def runtime(self):
        merger.build(
            config_root=CONFIG_ROOT,
            variant="stationary",
            with_local=False,
        )
        return CONFIG_ROOT / "_runtime" / "stationary"

    def _load(self, path: Path):
        with path.open() as f:
            return yaml.safe_load(f)

    def _assert_semantic_equal(self, legacy: dict, new: dict, *, label: str):
        """Recursive compare, with list-as-multiset only at known reordered paths."""
        # record_topics is the only known reordered list — handle separately.
        legacy_rt = (
            legacy.get("/**", {}).get("ros__parameters", {}).pop("record_topics", None)
        )
        new_rt = (
            new.get("/**", {}).get("ros__parameters", {}).pop("record_topics", None)
        )
        if legacy_rt is not None or new_rt is not None:
            assert sorted(legacy_rt) == sorted(new_rt), (
                f"{label}: record_topics differs"
            )
        assert legacy == new, f"{label}: non-record_topics fields differ"

    def test_robot_config_matches_legacy(self, runtime):
        legacy = self._load(FIXTURES / "robot_config.yaml")
        new = self._load(runtime / "robot_config.yaml")
        self._assert_semantic_equal(legacy, new, label="robot_config")

    def test_recording_gate_matches_legacy(self, runtime):
        legacy = self._load(FIXTURES / "recording_gate.yaml")
        new = self._load(runtime / "recording_gate.yaml")
        assert legacy == new, "recording_gate differs"

    def test_upload_targets_matches_legacy(self, runtime):
        legacy = self._load(FIXTURES / "upload_targets.yaml")
        new = self._load(runtime / "upload_targets.yaml")
        assert legacy == new, "upload_targets differs"

    def test_yubi_devices_merge_matches_legacy(self):
        """yubi_devices.yaml is loaded via ROS 2 native multi-params at runtime,
        not pre-merged on disk. Simulate that merge here and compare to legacy.

        footpedal_node is a new presence marker introduced by the refactor
        (legacy had no YAML entry for it; the launch file launched it
        unconditionally). It's expected to be ONLY in the new merged result.
        """
        legacy = self._load(FIXTURES / "yubi_devices.yaml")
        common = self._load(CONFIG_ROOT / "common" / "yubi_devices.yaml")
        stationary = self._load(CONFIG_ROOT / "stationary" / "yubi_devices.yaml")
        merged = merger.deep_merge(common, stationary)
        # footpedal_node was added as a presence marker by the refactor — strip
        # it before comparing to legacy.
        assert merged.pop("footpedal_node", None) == {"ros__parameters": {}}, (
            "expected footpedal_node presence marker in merged stationary"
        )
        assert merged == legacy, (
            "common + stationary yubi_devices.yaml diverged from legacy"
        )


# ---------------------------------------------------------------------------
# TestPortableInvariants — portable structure assertions
# ---------------------------------------------------------------------------
class TestPortableInvariants:
    @pytest.fixture(scope="class")
    def portable_runtime(self):
        merger.build(config_root=CONFIG_ROOT, variant="portable", with_local=False)
        return CONFIG_ROOT / "_runtime" / "portable"

    def test_robot_type(self, portable_runtime):
        with (portable_runtime / "robot_config.yaml").open() as f:
            data = yaml.safe_load(f)
        assert data["/**"]["ros__parameters"]["robot_type"] == "yubi_portable"

    def test_record_topics_contain_center_camera(self, portable_runtime):
        with (portable_runtime / "robot_config.yaml").open() as f:
            data = yaml.safe_load(f)
        rt = data["/**"]["ros__parameters"]["record_topics"]
        assert "/center_camera/camera_info" in rt
        assert "/center_camera/image_raw/compressed" in rt

    def test_record_topics_exclude_realsense(self, portable_runtime):
        with (portable_runtime / "robot_config.yaml").open() as f:
            data = yaml.safe_load(f)
        rt = data["/**"]["ros__parameters"]["record_topics"]
        for entry in rt:
            assert not entry.startswith("/camera/camera/"), (
                f"portable record_topics should not include RealSense topic {entry}"
            )

    def test_recording_gate_head_camera_topic(self, portable_runtime):
        with (portable_runtime / "recording_gate.yaml").open() as f:
            data = yaml.safe_load(f)
        conditions = data["groups"]["health"]["conditions"]
        for key in (
            "head_camera_warn",
            "head_camera_stop",
            "head_camera_timeout_warn",
            "head_camera_timeout_stop",
        ):
            assert conditions[key]["topic"] == "/center_camera/camera_info", (
                f"portable {key}.topic should be /center_camera/camera_info"
            )
        assert conditions["head_camera_warn"]["min_rate_hz"] == 30.0
        assert conditions["head_camera_stop"]["min_rate_hz"] == 15.0

    def test_yubi_devices_has_center_camera(self):
        with (CONFIG_ROOT / "portable" / "yubi_devices.yaml").open() as f:
            data = yaml.safe_load(f)
        assert "center_camera/usb_cam" in data
        cc = data["center_camera/usb_cam"]["ros__parameters"]
        assert cc["video_device"] == "/dev/yubi_center_camera"
        assert cc["camera_name"] == "center_camera"

    def test_portable_no_undeclared_gripper_params(self):
        """Portable YAML must not set gripper-state event params until the
        submodule's task_command_dispatch_node declares them (rclpy errors on
        undeclared params)."""
        with (CONFIG_ROOT / "portable" / "yubi_devices.yaml").open() as f:
            data = yaml.safe_load(f)
        dispatch = data.get("task_command_dispatch_node", {}).get("ros__parameters", {})
        for key in dispatch.keys():
            assert not key.startswith("gripper_"), (
                f"portable yubi_devices.yaml has premature gripper param: {key}"
            )

    def test_portable_no_footpedal_node(self):
        """Portable variant should not bring up footpedal_node (no footpedal hw)."""
        with (CONFIG_ROOT / "portable" / "yubi_devices.yaml").open() as f:
            data = yaml.safe_load(f)
        assert "footpedal_node" not in data


# ---------------------------------------------------------------------------
# TestPortableHeadInvariants — portable_head structure assertions
# ---------------------------------------------------------------------------
def _load(path):
    with path.open() as f:
        return yaml.safe_load(f)


class TestPortableHeadInvariants:
    @pytest.fixture(scope="class")
    def runtime(self):
        merger.build(config_root=CONFIG_ROOT, variant="portable_head", with_local=False)
        return CONFIG_ROOT / "_runtime" / "portable_head"

    def test_robot_type(self, runtime):
        data = _load(runtime / "robot_config.yaml")
        assert data["/**"]["ros__parameters"]["robot_type"] == "yubi_portable_head"

    def test_record_topics_superset_of_portable_plus_body(self, runtime):
        merger.build(config_root=CONFIG_ROOT, variant="portable", with_local=False)
        portable = _load(CONFIG_ROOT / "_runtime" / "portable" / "robot_config.yaml")
        head = _load(runtime / "robot_config.yaml")
        portable_topics = portable["/**"]["ros__parameters"]["record_topics"]
        head_topics = head["/**"]["ros__parameters"]["record_topics"]
        assert set(head_topics) - set(portable_topics) == {"/quest/body/state"}
        assert set(portable_topics) <= set(head_topics)
        assert "/tf_static" in head_topics

    def test_shared_device_sections_match_portable(self):
        portable = _load(CONFIG_ROOT / "portable" / "yubi_devices.yaml")
        head = _load(CONFIG_ROOT / "portable_head" / "yubi_devices.yaml")
        for key in (
            "center_camera/usb_cam",
            "task_command_dispatch_node",
            "gripper_double_click_node",
            "portable_joy_command_node",
        ):
            assert head[key] == portable[key], f"{key} differs from portable"
        assert "footpedal_node" not in head

    def test_head_camera_gate_matches_portable(self, runtime):
        portable = _load(CONFIG_ROOT / "portable" / "recording_gate.yaml")
        head = _load(CONFIG_ROOT / "portable_head" / "recording_gate.yaml")
        assert (
            head["groups"]["health"]["conditions"]
            == portable["groups"]["health"]["conditions"]
        )

    def test_body_tracking_gate(self, runtime):
        data = _load(runtime / "recording_gate.yaml")
        conditions = data["groups"]["body_tracking"]["conditions"]
        warn = conditions["body_tracking_timeout_warn"]
        stop = conditions["body_tracking_timeout_stop"]
        fidelity = conditions["body_tracking_fidelity_warn"]
        for cond in (warn, stop, fidelity):
            assert cond["topic"] == "/quest/body/fidelity"
        assert (warn["timeout_sec"], warn["escalation"]) == (1.0, 1)
        assert (stop["timeout_sec"], stop["escalation"]) == (3.0, 2)
        assert fidelity["condition"] == "msg.data == 2"
        assert fidelity["escalation"] == 1
        # The common hand-camera and gripper checks still apply.
        assert "left_camera_timeout_stop" in data["groups"]["health"]["conditions"]

    def test_head_camera_mount_tfs(self):
        from yubi_bringup.launch_registry import static_tf_arguments  # noqa: WPS433

        files = [
            CONFIG_ROOT / "common" / "yubi_devices.yaml",
            CONFIG_ROOT / "portable_head" / "yubi_devices.yaml",
        ]
        mount = static_tf_arguments(files, "center_camera_mount_tf")
        assert mount[-4:] == [
            "--frame-id",
            "quest_hmd",
            "--child-frame-id",
            "center_camera_link",
        ]
        optical = dict(
            zip(*[iter(static_tf_arguments(files, "center_camera_optical_tf"))] * 2)
        )
        assert optical["--frame-id"] == "center_camera_link"
        assert optical["--child-frame-id"] == "center_camera_optical_frame"
        assert float(optical["--roll"]) == pytest.approx(-1.5707963267948966)
        assert float(optical["--pitch"]) == 0.0
        assert float(optical["--yaw"]) == pytest.approx(-1.5707963267948966)
        assert float(optical["--x"]) == 0.0


# ---------------------------------------------------------------------------
# TestStaticTfArguments — static_transform_publisher command line from YAML
# ---------------------------------------------------------------------------
class TestStaticTfArguments:
    def test_later_files_override_and_defaults_fill(self, tmp_path):
        from yubi_bringup.launch_registry import static_tf_arguments  # noqa: WPS433

        base = tmp_path / "a.yaml"
        local = tmp_path / "b.yaml"
        section = {"parent_frame": "p", "child_frame": "c", "x": 0.1, "yaw": 1.0}
        _write_yaml(base, {"mount_tf": {"ros__parameters": section}})
        _write_yaml(local, {"mount_tf": {"ros__parameters": {"x": 2, "z": -0.5}}})

        args = static_tf_arguments([base, local, tmp_path / "missing.yaml"], "mount_tf")

        assert args == [
            "--x", "2.0",
            "--y", "0.0",
            "--z", "-0.5",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "1.0",
            "--frame-id", "p",
            "--child-frame-id", "c",
        ]  # fmt: skip

    def test_missing_frames_raise(self, tmp_path):
        from yubi_bringup.launch_registry import static_tf_arguments  # noqa: WPS433

        path = tmp_path / "a.yaml"
        _write_yaml(path, {"mount_tf": {"ros__parameters": {"parent_frame": "p"}}})
        with pytest.raises(ValueError, match="child_frame"):
            static_tf_arguments([path], "mount_tf")


# ---------------------------------------------------------------------------
# TestNodeRegistry — registry <-> YAML key bidirectional consistency
# ---------------------------------------------------------------------------
class TestNodeRegistry:
    @pytest.fixture(scope="class")
    def keys_per_variant(self):
        keys = {}
        for variant in ("common", "stationary", "portable", "portable_head"):
            with (CONFIG_ROOT / variant / "yubi_devices.yaml").open() as f:
                data = yaml.safe_load(f) or {}
            keys[variant] = set(data.keys())
        return keys

    def test_registry_entries_appear_in_some_variant(self, keys_per_variant):
        from yubi_bringup.launch_registry import NODE_REGISTRY  # noqa: WPS433

        all_keys = set().union(*keys_per_variant.values())
        unreachable = [
            entry["yaml_key"]
            for entry in NODE_REGISTRY
            if entry["yaml_key"] not in all_keys
        ]
        assert not unreachable, (
            f"NODE_REGISTRY entries with no YAML coverage (would never launch): {unreachable}"
        )

    def test_yaml_keys_covered_by_registry(self, keys_per_variant):
        from yubi_bringup.launch_registry import (  # noqa: WPS433
            NODE_REGISTRY,
            STRUCTURAL_NODE_KEYS,
        )

        registry_keys = {entry["yaml_key"] for entry in NODE_REGISTRY}
        all_yaml_keys = set().union(*keys_per_variant.values())
        orphan = all_yaml_keys - registry_keys - STRUCTURAL_NODE_KEYS
        assert not orphan, (
            f"YAML keys with no NODE_REGISTRY entry (parameters loaded but no node "
            f"will spawn): {orphan}"
        )

    def test_bringup_and_data_collection_disjoint(self):
        from yubi_bringup.launch_registry import (  # noqa: WPS433
            BRINGUP_NODE_REGISTRY,
            DATA_COLLECTION_NODE_REGISTRY,
        )

        bringup_keys = {e["yaml_key"] for e in BRINGUP_NODE_REGISTRY}
        dc_keys = {e["yaml_key"] for e in DATA_COLLECTION_NODE_REGISTRY}
        overlap = bringup_keys & dc_keys
        assert not overlap, (
            f"Same yaml_key registered in both bringup and data-collection "
            f"registries (would double-spawn): {overlap}"
        )

    def test_collect_yaml_keys_helper(self, tmp_path):
        from yubi_bringup.launch_registry import collect_yaml_keys  # noqa: WPS433

        f1 = tmp_path / "a.yaml"
        _write_yaml(f1, {"x": 1, "y": 2})
        f2 = tmp_path / "b.yaml"
        _write_yaml(f2, {"y": 3, "z": 4})  # y is shared
        assert collect_yaml_keys([f1, f2, tmp_path / "missing.yaml"]) == {"x", "y", "z"}
