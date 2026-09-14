#!/usr/bin/env python3
"""Compile the production audit header and protect the unchanged control law.
No ROS/FSM execution is claimed by this standalone test.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]

class AuditTest(unittest.TestCase):
    def test_unchanged_law_and_wiring(self):
        law = (PACKAGE/'src/common_types.cpp').read_bytes()
        self.assertEqual(hashlib.sha1(b'blob '+str(len(law)).encode()+b'\0'+law).hexdigest(),
                         '6101dab5e1a0d87f807f88533de39f0c0a030baa')
        state = (PACKAGE/'src/state_machine/custom1_state.cpp').read_text()
        stop = state.split('void Custom1State::emitZero(', 1)[1].split('::state_machine::ActionResult', 1)[0]
        self.assertIn('body_speed_ = 0.0;', stop)
        self.assertIn('flatness_audit_.valid = false;', stop)
        self.assertIn('snapshot.stamp.toSec(), lifted.stamp.toSec()', state)
        self.assertIn('event.payload["flatness_audit_json"]', state)
        consumer = (PACKAGE/'src/output/cmd_vel_output_consumer.cpp').read_text()
        self.assertIn('const auto held_command = controller_.command();', consumer)
        self.assertIn('held_command.stamp.toSec(), *stamp', consumer)

    def test_actual_header(self):
        code = r'''
#include "unicycle_ugv_controller/common/flatness_audit.h"
#include <iostream>
#include <limits>
int main() {
 using namespace unicycle_ugv_controller;
 FlatnessAuditSample a;
 if (!a.toJson().empty()) return 1;
 a.valid = true;
 for (std::size_t i=0;i<a.values.size();++i) a.values[i]=double(i)/8.;
 a.values[9]=std::numeric_limits<double>::quiet_NaN();
 std::cout << a.toJson() << '\n';
 std::cout << flatnessAuditPublication(a.toJson(),1.,.5,-.3,.75,.75) << '\n';
 std::cout << flatnessAuditPublication(a.toJson(),1.,.5,-.3,.8,.75) << '\n';
 if (!flatnessAuditPublication(a.toJson(),std::numeric_limits<double>::infinity(),0,0,0,0).empty()) return 2;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            cpp=Path(tmp)/'test.cpp'; exe=Path(tmp)/'test'; cpp.write_text(code)
            subprocess.run([os.environ.get('CXX','g++'),'-std=c++17','-Wall','-Wextra','-Werror',
                            '-I'+str(PACKAGE/'include'),str(cpp),'-o',str(exe)],check=True)
            rows=[json.loads(s) for s in subprocess.check_output([str(exe)],text=True).splitlines()]
        self.assertEqual(len(rows[0]),30)
        self.assertIsNone(rows[0]['wz_measured_rad_s'])
        self.assertEqual(rows[0]['cached_state_speed_m_s'],3.5)
        self.assertTrue(rows[1]['sample_matches_command'])
        self.assertFalse(rows[2]['sample_matches_command'])
        self.assertEqual(rows[1]['sample'],rows[0])
        self.assertEqual(rows[1]['published_w_rad_s'],-.3)

if __name__ == '__main__': unittest.main()
