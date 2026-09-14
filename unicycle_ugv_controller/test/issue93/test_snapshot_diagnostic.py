#!/usr/bin/env python3
"""Test actual diagnostic header plus guarded production call sites, without ROS."""
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[1]


class SnapshotDiagnosticTest(unittest.TestCase):
    def test_actual_header(self):
        with tempfile.TemporaryDirectory() as temporary:
            exe=Path(temporary)/'diagnostic'
            subprocess.run([os.environ.get('CXX','g++'),'-std=c++17','-Wall','-Wextra','-Werror',
                '-fsanitize=address,undefined','-fno-omit-frame-pointer','-I'+str(PACKAGE/'include'),
                str(HERE/'diagnostic_cases.cpp'),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True,env=dict(os.environ,ASAN_OPTIONS='detect_leaks=0'))

    def test_same_snapshot_and_no_control_change(self):
        raw=(PACKAGE/'src/common_types.cpp').read_bytes()
        blob=hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
        self.assertEqual(blob,'6101dab5e1a0d87f807f88533de39f0c0a030baa')
        state=(PACKAGE/'src/state_machine/custom1_state.cpp').read_text()
        self.assertIn('now, dt, command_speed_before, snapshot, lifted, output',state)
        self.assertLess(state.index('command_speed_before = body_speed_'),state.index('body_speed_ = output.linear_speed'))
        self.assertIn('computeFlatnessCommand(snapshot, lifted, body_speed_, dt, controller_.config())',state)
        pub=(PACKAGE/'src/output/cmd_vel_output_consumer.cpp').read_text()
        self.assertIn('const auto command = makeTwist(snapshot);',pub)
        self.assertIn('atFlatnessPublication(snapshot.flatness_diagnostic,',pub)
        self.assertNotIn('controlState()',pub)
        self.assertNotIn('liftedWorldPva()',pub)
        self.assertIn('cmd_vel_topic + "/flatness_diagnostic"',pub)


if __name__=='__main__':unittest.main()
