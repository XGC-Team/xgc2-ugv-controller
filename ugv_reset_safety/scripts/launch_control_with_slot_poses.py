#!/usr/bin/env python3
"""Freeze Experiment slot poses into complete controller YAMLs before launch."""

import json
import math
import os
from pathlib import Path
import re
import sys
import uuid
import xml.etree.ElementTree as ET

import yaml

UGV_KINDS = {"scout_mini", "mecanum_ugv", "scout", "mecanum"}


def slot_poses(robots):
    """Validate the frozen roster before creating any launch configuration."""
    if not isinstance(robots, list):
        raise ValueError("robots must be a JSON array of Experiment slots")
    poses = {}
    for robot in robots:
        if not isinstance(robot, dict):
            raise ValueError("each robot slot must be an object")
        namespace = str(robot.get("namespace") or "").strip().strip("/")
        if not namespace.startswith("ugv"):
            continue
        kind = str(robot.get("kind") or "")
        if kind and kind not in UGV_KINDS:
            continue
        if not re.fullmatch(r"ugv[0-9]+", namespace) or namespace in poses:
            raise ValueError("invalid or duplicate UGV namespace: " + namespace)
        pose = robot.get("initialPose") or {}
        values = {}
        for axis in ("x", "y", "yaw"):
            value = pose.get(axis)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("UGV slot missing finite initialPose: " + namespace)
            values["reset_initial_" + axis] = float(value)
        poses[namespace] = values
    if not poses:
        raise ValueError("no UGV slots with Experiment initialPose")
    return poses


def materialize_controller_configs(poses, source_files, output_dir):
    """Create the exact private parameter files that roslaunch will load."""
    if set(poses) != set(source_files):
        raise ValueError("Experiment UGV slots do not match controller launch configuration slots")
    prepared = {}
    for namespace, source in source_files.items():
        with open(source) as stream:
            parameters = yaml.safe_load(stream)
        if not isinstance(parameters, dict):
            raise ValueError("controller configuration must be a YAML mapping: " + str(source))
        if "$(" in yaml.safe_dump(parameters):
            raise ValueError("controller slot configuration must contain resolved values: " + str(source))
        parameters.update(poses[namespace])
        prepared[namespace] = parameters
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=False)
    args = []
    manifest = {}
    for namespace, parameters in sorted(prepared.items()):
        target = directory / (namespace + ".yaml")
        target.write_text(yaml.safe_dump(parameters))
        args.append(namespace + "_config_file:=" + str(target))
        manifest[namespace] = {"source": str(source_files[namespace]), "loaded_file": str(target), "parameters": parameters}
    (directory / "manifest.yaml").write_text(yaml.safe_dump(manifest))
    return args


def publish_manifest(source, destination):
    """Publish a self-contained snapshot for this Session after all files exist."""
    target = Path(destination)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + '.' + str(uuid.uuid4()) + '.tmp')
    temporary.write_bytes(Path(source).read_bytes())
    os.replace(str(temporary), str(target))


def launch_config_sources(package, launch_file):
    import rospkg
    from roslaunch.substitution_args import resolve_args

    package_dir = Path(rospkg.RosPack().get_path(package))
    matches = list((package_dir / "launch").rglob(launch_file))
    if len(matches) != 1:
        raise ValueError("expected one controller launch: " + launch_file)
    root = ET.parse(matches[0]).getroot()
    result = {}
    for argument in root.findall("arg"):
        name = argument.get("name", "")
        match = re.fullmatch(r"(ugv[0-9]+)_config_file", name)
        if match:
            result[match.group(1)] = resolve_args(argument.attrib["default"])
    if not result:
        raise ValueError("launch does not declare per-slot configuration files")
    return result


def main(argv):
    if len(argv) not in (3, 4):
        print("usage: launch_control_with_slot_poses.py PACKAGE LAUNCH_FILE ROBOTS_JSON [SESSION_MANIFEST_FILE]", file=sys.stderr)
        return 2
    package, launch_file, robots_json = argv[:3]
    try:
        poses = slot_poses(json.loads(robots_json))
        sources = launch_config_sources(package, launch_file)
        if len(argv) == 4:
            root = Path(argv[3]).parent
        else:
            ros_home = Path(os.environ.get("ROS_HOME", str(Path.home() / ".ros")))
            root = ros_home / "xgc-controller-configurations"
        root.mkdir(parents=True, exist_ok=True)
        destination = root / str(uuid.uuid4())
        args = materialize_controller_configs(poses, sources, destination)
        if len(argv) == 4:
            publish_manifest(destination / "manifest.yaml", argv[3])
    except (ValueError, OSError, KeyError) as error:
        print("controller YAML: {}".format(error), file=sys.stderr)
        return 2
    print("Controller configuration: " + str(Path(destination) / "manifest.yaml"), file=sys.stderr, flush=True)
    if os.environ.get("XGC_PRINT_LAUNCH_ARGS") == "1":
        print(" ".join(args))
        return 0
    os.execvp("roslaunch", ["roslaunch", package, launch_file] + args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
