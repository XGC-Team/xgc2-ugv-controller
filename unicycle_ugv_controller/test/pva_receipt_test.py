#!/usr/bin/env python3
"""Compile the production serializer; verify its diagnostic-only wiring."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]


class PvaReceiptTest(unittest.TestCase):
    def test_round_trip_preserves_both_times_and_si_values(self):
        code = r'''
#include "unicycle_ugv_controller/common/pva_receipt.h"
#include <cassert>
#include <iostream>
int main() {
    using namespace unicycle_ugv_controller;
    PvaReceipt r;
    r.source_sec = 1800000000U;
    r.source_nsec = 999999999U;
    r.received_sec = 1800000000U;
    r.received_nsec = 901234567U;
    r.received_sequence = 9007199254740993ULL;
    r.source_sequence = 42U;
    r.pva = {-1.1234567890123457, 2.0, -3.0, -.2, .7, .01, -.02};
    std::cout << serializePvaReceipt(r) << '\n';
    for (double value : {std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
        r.pva[0] = value;
        bool rejected = false;
        try { serializePvaReceipt(r); }
        catch (const std::invalid_argument&) { rejected = true; }
        assert(rejected);
    }
    r.pva[0] = 0.0;
    r.received_nsec = 1000000000U;
    bool rejected = false;
    try { serializePvaReceipt(r); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'test.cpp'
            binary = Path(tmp) / 'test'
            source.write_text(code)
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-O2',
                            '-I', str(PACKAGE / 'include'), str(source), '-o', str(binary)], check=True)
            data = json.loads(subprocess.check_output([str(binary)], text=True))
        self.assertEqual(data['source_nsec'], 999999999)
        self.assertEqual(data['received_nsec'], 901234567)
        self.assertEqual(data['received_sequence'], 9007199254740993)
        self.assertEqual(data['source_sequence'], 42)
        self.assertEqual(data['pva'], [-1.1234567890123457, 2., -3., -.2, .7, .01, -.02])

    def test_trace_uses_the_accepted_receipt_without_changing_control_time(self):
        source = (PACKAGE / 'src/input/pva_reference_input_producer.cpp').read_text()
        self.assertIn('reference.stamp = ros::Time::now();', source)
        self.assertIn('receipt.received_sec = reference.stamp.sec;', source)
        self.assertIn('receipt.source_sec = msg->header.stamp.sec;', source)
        self.assertLess(source.index('controller_.setWorldPva(reference);'),
                        source.index('receipt_pub_.publish(trace);'))
        self.assertIn('nh.resolveName(topic) + "/receipt"', source)
        self.assertNotIn('reference.stamp = msg->header.stamp', source)


if __name__ == '__main__':
    unittest.main()
