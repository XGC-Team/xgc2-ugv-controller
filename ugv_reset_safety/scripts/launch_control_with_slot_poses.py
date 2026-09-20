#!/usr/bin/env python3
"""Freeze Experiment slot poses and explicit world boundary into launch YAMLs."""

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


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate worldBoundary field: " + key)
        result[key] = value
    return result


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def decode_world_boundary(raw):
    """Read the existing Session v4 contract, never a second site configuration.

    An explicit null means no Experiment override; omission is a caller error.
    All six control endpoints are validated even though UGVs consume only XY.
    Coordinates are already in world/metres. No offset or ground substitution.
    """
    boundary = json.loads(raw, object_pairs_hook=_unique_object)
    if boundary is None:
        return None
    keys = {"schemaVersion", "frameId", "unit", "controlBounds", "groundZ"}
    if not isinstance(boundary, dict) or set(boundary) != keys:
        raise ValueError("worldBoundary must be the complete current Session object or null")
    if (type(boundary["schemaVersion"]) is not int or boundary["schemaVersion"] != 1
            or boundary["frameId"] != "world" or boundary["unit"] != "m"):
        raise ValueError("worldBoundary requires schemaVersion=1, frameId=world, unit=m")
    if boundary["groundZ"] is not None and not _finite_number(boundary["groundZ"]):
        raise ValueError("worldBoundary groundZ must be finite or null")
    bounds = boundary["controlBounds"]
    if bounds is not None:
        names = {axis + side for axis in "xyz" for side in ("Min", "Max")}
        if not isinstance(bounds, dict) or set(bounds) != names:
            raise ValueError("controlBounds requires exactly six endpoints")
        for axis in "xyz":
            low, high = bounds[axis + "Min"], bounds[axis + "Max"]
            if not _finite_number(low) or not _finite_number(high) or low >= high:
                raise ValueError("controlBounds " + axis.upper() + " requires finite min < max in metres")
    return boundary


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


def materialize_controller_configs(poses, source_files, output_dir, world_boundary):
    """Create the exact private parameter files that roslaunch will receive.

    This is launch-resolved evidence, not proof of a running controller load.
    """
    world_boundary = decode_world_boundary(json.dumps(world_boundary, allow_nan=False))
    bounds = world_boundary["controlBounds"] if world_boundary is not None else None
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
        if bounds is not None:
            fence = parameters.get("fence", {})
            if not isinstance(fence, dict):
                raise ValueError("controller fence must be a mapping: " + str(source))
            parameters["fence"] = dict(fence, **{
                axis + "_" + side.lower(): bounds[axis + side]
                for axis in "xy" for side in ("Min", "Max")
            })
        prepared[namespace] = parameters
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=False)
    args = []
    manifest = {}
    for namespace, parameters in sorted(prepared.items()):
        target = directory / (namespace + ".yaml")
        target.write_text(yaml.safe_dump(parameters))
        args.append(namespace + "_config_file:=" + str(target))
        manifest[namespace] = {
            "source": str(source_files[namespace]), "loaded_file": str(target), "parameters": parameters,
            "worldBoundary": world_boundary,
            "fenceSource": "experiment-controlBounds" if bounds is not None else "controller-config",
            "evidence": "launch-resolved",
        }
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
    if len(argv) not in (4, 5):
        print("usage: launch_control_with_slot_poses.py PACKAGE LAUNCH_FILE ROBOTS_JSON WORLD_BOUNDARY_JSON [SESSION_MANIFEST_FILE]", file=sys.stderr)
        return 2
    package, launch_file, robots_json, boundary_json = argv[:4]
    try:
        boundary = decode_world_boundary(boundary_json)
        poses = slot_poses(json.loads(robots_json))
        sources = launch_config_sources(package, launch_file)
        if len(argv) == 5:
            root = Path(argv[4]).parent
        else:
            ros_home = Path(os.environ.get("ROS_HOME", str(Path.home() / ".ros")))
            root = ros_home / "xgc-controller-configurations"
        destination = root / str(uuid.uuid4())
        args = materialize_controller_configs(poses, sources, destination, boundary)
        if len(argv) == 5:
            # Include resolved UAV parameters as well in mixed swarm launches.
            import roslaunch
            launch_path = roslaunch.rlutil.resolve_launch_arguments([package, launch_file])[0]
            expanded = roslaunch.config.ROSLaunchConfig()
            roslaunch.xmlloader.XmlLoader().load(launch_path, expanded, argv=args, verbose=False)
            manifest_file = destination / "manifest.yaml"
            manifest = yaml.safe_load(manifest_file.read_text())
            manifest["ros_parameters"] = {key: parameter.value for key, parameter in expanded.params.items()}
            manifest_file.write_text(yaml.safe_dump(manifest))
            publish_manifest(manifest_file, argv[4])
    except (ValueError, OSError, KeyError, yaml.YAMLError) as error:
        print("controller YAML: {}".format(error), file=sys.stderr)
        return 2
    print("Controller configuration: " + str(Path(destination) / "manifest.yaml"), file=sys.stderr, flush=True)
    if os.environ.get("XGC_PRINT_LAUNCH_ARGS") == "1":
        print(" ".join(args))
        return 0
    os.execvp("roslaunch", ["roslaunch", package, launch_file] + args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
