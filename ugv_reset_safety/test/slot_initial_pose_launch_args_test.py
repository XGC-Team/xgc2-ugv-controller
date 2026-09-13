#!/usr/bin/env python3
"""Experiment initialPose is frozen into the YAML actually passed to roslaunch."""
import os
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from launch_control_with_slot_poses import materialize_controller_configs, slot_poses


class SlotInitialPoseConfigTest(unittest.TestCase):
    def test_complete_yaml_preserves_profile_and_freezes_each_slot(self):
        robots = [
            {'namespace': '/ugv1', 'kind': 'scout_mini', 'initialPose': {'x': -0.9, 'y': -2.5, 'yaw': 0.4}},
            {'namespace': '/ugv2', 'kind': 'scout_mini', 'initialPose': {'x': 1.2, 'y': 0.5, 'yaw': -0.2}},
            {'namespace': '/uav1', 'kind': 'px4_multirotor'},
        ]
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / 'base.yaml'
            original = {'tracking_strategy': 'flatness', 'flatness': {'kp': 6.0}, 'reset_initial_x': 999.0}
            source.write_text(yaml.safe_dump(original))
            args = materialize_controller_configs(slot_poses(robots), {'ugv1': source, 'ugv2': source}, root / 'run')
            self.assertEqual(len(args), 2)
            for robot, arg in zip(robots, args):
                name, path = arg.split(':=', 1)
                self.assertEqual(name, robot['namespace'].strip('/') + '_config_file')
                parameters = yaml.safe_load(Path(path).read_text())
                self.assertEqual(parameters['flatness'], {'kp': 6.0})
                self.assertEqual(parameters['tracking_strategy'], 'flatness')
                for axis in ('x', 'y', 'yaw'):
                    self.assertEqual(parameters['reset_initial_' + axis], robot['initialPose'][axis])
            self.assertEqual(yaml.safe_load(source.read_text()), original)
            manifest = yaml.safe_load((root / 'run/manifest.yaml').read_text())
            self.assertEqual(manifest['ugv1']['parameters'], yaml.safe_load((root / 'run/ugv1.yaml').read_text()))

    def test_missing_duplicate_and_nonfinite_poses_fail(self):
        good = {'namespace': '/ugv1', 'kind': 'scout_mini', 'initialPose': {'x': 0, 'y': 0, 'yaw': 0}}
        for rows in ([dict(good, initialPose={})], [good, good], [dict(good, initialPose={'x': float('nan'), 'y': 0, 'yaw': 0})]):
            with self.assertRaises(ValueError):
                slot_poses(rows)

    def test_roster_mismatch_does_not_create_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'run'
            with self.assertRaises(ValueError):
                materialize_controller_configs({'ugv1': {}}, {'ugv2': '/unused'}, output)
            self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
