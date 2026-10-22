#!/usr/bin/env python3
"""Build yubi_bringup/config/_runtime/<variant>/ from common + <variant> [+ local] overlays.

For each of (robot_config.yaml, recording_gate.yaml, upload_targets.yaml), this script:
  1. Loads common/<file>.yaml as the base (required)
  2. Deep-merges <variant>/<file>.yaml on top (optional)
  3. Deep-merges local/<file>.yaml on top if --with-local and present (optional)
  4. Writes the merged result to _runtime/<variant>/<file>.yaml

Merge semantics (deep_merge):
  - dict + dict       -> recursive merge per key (b wins on scalar conflict)
  - list + list       -> b replaces a entirely
  - list + {__extend__: [...]} -> append unique items from extend list to base list
  - key: "__delete__" -> removes key from the merged dict (lets a variant drop a
                         common entry, e.g. a recording-gate condition)
  - scalar override   -> b wins
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import yaml


MERGED_FILES = ("robot_config.yaml", "recording_gate.yaml", "upload_targets.yaml")

DELETE_SENTINEL = "__delete__"


def deep_merge(base: Any, overlay: Any) -> Any:
    """Recursively merge overlay onto base.

    See module docstring for semantics. Returns a new structure; inputs unchanged.
    """
    if isinstance(overlay, dict) and "__extend__" in overlay and isinstance(base, list):
        extension = overlay["__extend__"]
        if not isinstance(extension, list):
            raise ValueError(
                f"__extend__ must be a list, got {type(extension).__name__}"
            )
        merged = list(base)
        for item in extension:
            if item not in merged:
                merged.append(item)
        return merged

    if isinstance(base, dict) and isinstance(overlay, dict):
        result = dict(base)
        for key, overlay_val in overlay.items():
            if overlay_val == DELETE_SENTINEL:
                result.pop(key, None)
            elif key in result:
                result[key] = deep_merge(result[key], overlay_val)
            else:
                result[key] = overlay_val
        return result

    return overlay


def load_yaml(path: Path) -> Any:
    with path.open("r") as f:
        loaded = yaml.safe_load(f)
    return loaded if loaded is not None else {}


def build_one(
    filename: str,
    common_dir: Path,
    variant_dir: Path,
    local_dir: Path | None,
    out_dir: Path,
) -> Path:
    common_path = common_dir / filename
    if not common_path.exists():
        raise FileNotFoundError(f"Required common file not found: {common_path}")

    merged = load_yaml(common_path)

    variant_path = variant_dir / filename
    if variant_path.exists():
        merged = deep_merge(merged, load_yaml(variant_path))

    if local_dir is not None:
        local_path = local_dir / filename
        if local_path.exists():
            merged = deep_merge(merged, load_yaml(local_path))

    out_path = out_dir / filename
    out_dir.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as f:
        yaml.safe_dump(merged, f, sort_keys=False, default_flow_style=False)

    return out_path


def build(
    config_root: Path,
    variant: str,
    with_local: bool,
) -> list[Path]:
    common_dir = config_root / "common"
    variant_dir = config_root / variant
    local_dir = config_root / "local" if with_local else None
    out_dir = config_root / "_runtime" / variant

    written = []
    for filename in MERGED_FILES:
        written.append(build_one(filename, common_dir, variant_dir, local_dir, out_dir))
    return written


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--variant",
        required=True,
        help="Variant name (e.g. stationary, portable). Matches subdir under config/.",
    )
    parser.add_argument(
        "--with-local",
        action="store_true",
        help="Also apply config/local/<file>.yaml as a third overlay if present.",
    )
    parser.add_argument(
        "--config-root",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "config",
        help="Path to yubi_bringup/config (default: relative to this script).",
    )
    args = parser.parse_args(argv)

    written = build(
        config_root=args.config_root,
        variant=args.variant,
        with_local=args.with_local,
    )
    for path in written:
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
