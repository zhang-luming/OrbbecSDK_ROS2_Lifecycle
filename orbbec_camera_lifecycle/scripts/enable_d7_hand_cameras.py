#!/usr/bin/env python3
"""Enable the two hand cameras in the D7 config of the sourced ROS package."""

from pathlib import Path
import re
import shutil
import sys

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import yaml


CAMERA_NAMES = ("left_hand_rgbd", "right_hand_rgbd")


def enable_hand_cameras(text):
    lines = text.splitlines(keepends=True)
    found = set()
    uncomment = False
    for index, line in enumerate(lines):
        entry = re.match(r"^  (# )?- name: ([\w]+)\s*$", line)
        if entry:
            name = entry.group(2)
            uncomment = name in CAMERA_NAMES and entry.group(1) is not None
            if name in CAMERA_NAMES:
                if name in found:
                    raise ValueError(f"Duplicate camera entry: {name}")
                found.add(name)
        elif line.strip() and not line.startswith("  "):
            uncomment = False
        if uncomment and line.startswith("  # "):
            lines[index] = "  " + line[4:]

    missing = set(CAMERA_NAMES) - found
    if missing:
        raise ValueError(f"Camera entries not found: {', '.join(sorted(missing))}")
    result = "".join(lines)
    config = yaml.safe_load(result)
    names = [camera["name"] for camera in config["cameras"]]
    if any(names.count(name) != 1 for name in CAMERA_NAMES):
        raise ValueError("Both hand cameras must appear exactly once in cameras")
    return result


def main():
    try:
        share = Path(get_package_share_directory("orbbec_camera_lifecycle"))
        config_path = (share / "config" / "cameras-d7.yaml").resolve()
        original = config_path.read_text(encoding="utf-8")
        updated = enable_hand_cameras(original)
        if updated == original:
            print(f"Both D7 hand cameras are already enabled: {config_path}")
            return 0
        backup = config_path.with_name(config_path.name + ".bak")
        if not backup.exists():
            shutil.copy2(config_path, backup)
        config_path.write_text(updated, encoding="utf-8")
        print(f"Enabled left_hand_rgbd and right_hand_rgbd: {config_path}")
        print(f"Backup: {backup}")
        print("Restart the D7 camera launch for the changes to take effect.")
        return 0
    except (PackageNotFoundError, OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        print(f"Failed to enable D7 hand cameras: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
