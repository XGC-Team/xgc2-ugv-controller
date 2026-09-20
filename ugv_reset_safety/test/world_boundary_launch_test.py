#!/usr/bin/env python3
"""Boundary values must reach the exact private YAML passed to roslaunch."""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import launch_control_with_slot_poses as launch

BOUNDARY = {
    'schemaVersion': 1, 'frameId': 'world', 'unit': 'm',
    'controlBounds': {'xMin': -9, 'xMax': 7, 'yMin': -4, 'yMax': 3, 'zMin': 2, 'zMax': 8},
    'groundZ': -0.75,
}
ROBOTS = [{'namespace': '/ugv1', 'kind': 'scout_mini', 'initialPose': {'x': 0, 'y': 0, 'yaw': 0}}]


class WorldBoundaryLaunchTest(unittest.TestCase):
    def test_explicit_null_and_current_object_are_distinct(self):
        self.assertIsNone(launch.decode_world_boundary('null'))
        self.assertEqual(launch.decode_world_boundary(json.dumps(BOUNDARY)), BOUNDARY)
        empty = dict(BOUNDARY, controlBounds=None, groundZ=None)
        self.assertEqual(launch.decode_world_boundary(json.dumps(empty)), empty)

    def test_rejects_missing_unknown_alias_duplicate_and_wrong_frame(self):
        raw = json.dumps(BOUNDARY)
        invalid = ['', '{}', '[]', 'false', raw.replace('"world"', '"map"'),
                   raw.replace('"m"', '"cm"'), raw.replace('"schemaVersion": 1', '"schemaVersion": true'),
                   raw.replace('"xMin": -9', '"xMin": -9, "xMin": -8'),
                   raw.replace('"xMin": -9', '"xMin": -9, "xM\\u0069n": -8'),
                   raw.replace('"groundZ": -0.75', '"groundZ": -0.75, "groundZ": 0'),
                   raw.replace('"xMin"', '"XMin"'), raw + raw]
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ValueError):
                launch.decode_world_boundary(value)

    def test_every_axis_rejects_partial_null_boolean_nonfinite_and_inverted_range(self):
        for axis in 'xyz':
            for bad in [None, True, '1', float('nan'), float('inf'), 10 ** 400, 99]:
                boundary = copy.deepcopy(BOUNDARY)
                boundary['controlBounds'][axis + 'Min'] = bad
                with self.subTest(axis=axis, bad=bad), self.assertRaises(ValueError):
                    launch.decode_world_boundary(json.dumps(boundary))
            boundary = copy.deepcopy(BOUNDARY)
            del boundary['controlBounds'][axis + 'Min']
            with self.assertRaises(ValueError):
                launch.decode_world_boundary(json.dumps(boundary))

    def test_xy_overrides_preserve_profile_ground_request_and_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / 'base.yaml'
            original = {'fence': {'x_min': -99, 'x_max': 99, 'y_min': -99, 'y_max': 99, 'warning_margin': 0.2},
                        'flatness': {'kp': 6}, 'pose_topic': '/ugv1/pose'}
            source.write_text(yaml.safe_dump(original))
            before = source.read_bytes()
            frozen = copy.deepcopy(BOUNDARY)
            args = launch.materialize_controller_configs(launch.slot_poses(ROBOTS), {'ugv1': source}, root / 'run', frozen)
            self.assertEqual(args, ['ugv1_config_file:=' + str(root / 'run/ugv1.yaml')])
            params = yaml.safe_load((root / 'run/ugv1.yaml').read_text())
            self.assertEqual(params['fence'], {'x_min': -9, 'x_max': 7, 'y_min': -4, 'y_max': 3, 'warning_margin': 0.2})
            self.assertEqual(params['flatness'], original['flatness'])
            self.assertEqual(params['pose_topic'], original['pose_topic'])
            self.assertEqual(source.read_bytes(), before)
            self.assertEqual(frozen, BOUNDARY)
            manifest = yaml.safe_load((root / 'run/manifest.yaml').read_text())['ugv1']
            self.assertEqual(manifest['parameters'], params)
            self.assertEqual(manifest['worldBoundary'], BOUNDARY)
            self.assertEqual(manifest['fenceSource'], 'experiment-controlBounds')
            self.assertEqual(manifest['evidence'], 'launch-resolved')
            self.assertNotIn('z_min', params['fence'])

    def test_null_preserves_original_fence_and_records_absence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / 'base.yaml'
            source.write_text('fence: {x_min: -4, x_max: 4, y_min: -3, y_max: 3}\n')
            launch.materialize_controller_configs(launch.slot_poses(ROBOTS), {'ugv1': source}, root / 'run', None)
            manifest = yaml.safe_load((root / 'run/manifest.yaml').read_text())['ugv1']
            self.assertIsNone(manifest['worldBoundary'])
            self.assertEqual(manifest['fenceSource'], 'controller-config')
            self.assertEqual(manifest['parameters']['fence'], yaml.safe_load(source.read_text())['fence'])

    def test_invalid_boundary_does_not_create_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'run'
            with self.assertRaises(ValueError):
                launch.materialize_controller_configs(launch.slot_poses(ROBOTS), {'ugv1': '/unused'}, output, {})
            self.assertFalse(output.exists())

    def test_bad_source_fence_is_not_silently_repaired(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / 'base.yaml'
            source.write_text('fence: malformed\n')
            with self.assertRaises(ValueError):
                launch.materialize_controller_configs(launch.slot_poses(ROBOTS), {'ugv1': source}, root / 'run', BOUNDARY)
            self.assertFalse((root / 'run').exists())

    def test_old_cli_and_bad_boundary_never_resolve_or_execute_launch(self):
        for args in [['pkg', 'file', json.dumps(ROBOTS)],
                     ['pkg', 'file', json.dumps(ROBOTS), '/old/session/manifest.yaml'],
                     ['pkg', 'file', json.dumps(ROBOTS), '{}']]:
            with patch.object(launch, 'launch_config_sources') as sources, patch.object(launch.os, 'execvp') as execute:
                self.assertEqual(launch.main(args), 2)
                sources.assert_not_called()
                execute.assert_not_called()

    def test_exec_receives_the_materialized_frozen_yaml(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / 'base.yaml'
            source.write_text('flatness: {kp: 6}\n')
            with patch.dict(os.environ, {'ROS_HOME': str(root), 'XGC_PRINT_LAUNCH_ARGS': ''}), \
                    patch.object(launch, 'launch_config_sources', return_value={'ugv1': source}), \
                    patch.object(launch.os, 'execvp') as execute:
                launch.main(['pkg', 'file', json.dumps(ROBOTS), json.dumps(BOUNDARY)])
                program, argv = execute.call_args.args
                self.assertEqual(program, 'roslaunch')
                self.assertEqual(argv[:3], ['roslaunch', 'pkg', 'file'])
                params = yaml.safe_load(Path(argv[3].split(':=', 1)[1]).read_text())
                self.assertEqual(params['fence']['x_min'], BOUNDARY['controlBounds']['xMin'])
                self.assertEqual(params['fence']['y_max'], BOUNDARY['controlBounds']['yMax'])


if __name__ == '__main__':
    unittest.main()
