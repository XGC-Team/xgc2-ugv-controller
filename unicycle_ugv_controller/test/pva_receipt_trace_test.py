#!/usr/bin/env python3
"""Compile the production receipt encoder and unchanged production PVA lift."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

class ReceiptTest(unittest.TestCase):
    def test_receipt_contract(self):
        source = (ROOT / 'src/common_types.cpp').read_text()
        start = source.index('WorldPvaReference liftWorldPva(')
        end = source.index('\nbool worldPvaReady(', start)
        lift = source[start:end]
        main = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>
#include "unicycle_ugv_controller/common/pva_receipt_trace.h"
struct Time { double t; double toSec() const { return t; } };
struct WorldPvaReference {
    Time stamp{0}; bool valid{true};
    double x{0},y{0},vx{0},vy{0},ax{0},ay{0};
};
''' + lift + r'''
int main() {
    using namespace unicycle_ugv_controller;
    auto trace = pvaReceiptTrace(10.01,10.1,77,3,{{1,2,.3,-.2,.4,.6}},.5);
    assert(trace.size()==12 && trace[0]==1 && trace[1]==10.01 && trace[2]==10.1);
    assert(trace[3]==77 && trace[4]==3 && trace[11]==.5);
    const std::string names=pvaReceiptTraceLayout();
    assert(std::count(names.begin(),names.end(),',')==11);
    WorldPvaReference sample;
    sample.stamp.t=trace[1]; sample.x=trace[5]; sample.y=trace[6];
    sample.vx=trace[7]; sample.vy=trace[8]; sample.ax=trace[9]; sample.ay=trace[10];
    auto lifted=liftWorldPva(sample,10.03);
    assert(lifted.valid);
    assert(std::abs(lifted.x-(1+.3*.02+.5*.4*.02*.02))<1e-12);
    assert(std::abs(lifted.vy-(-.2+.6*.02))<1e-12);
    // A future source stamp does not defer reference consumption.
    assert(lifted.x>sample.x);
    assert(liftWorldPva(sample,10.01).x==sample.x);
    assert(liftWorldPva(sample,9.0).x==sample.x);
    auto next=pvaReceiptTrace(10.11,10.2,78,4,{{2,3,0,0,0,0}},.6);
    assert(next[4]==4 && next[1]>trace[1]);
    auto rewind=pvaReceiptTrace(0.01,.1,1,5,{{0,0,0,0,0,0}},0);
    assert(rewind[1]<next[1] && rewind[4]>next[4]);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory); (path/'test.cpp').write_text(main)
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-O2',
                '-I'+str(ROOT/'include'),str(path/'test.cpp'),'-o',str(path/'test')],check=True)
            subprocess.run([str(path/'test')],check=True)

    def test_source_wiring(self):
        code=(ROOT/'src/input/pva_reference_input_producer.cpp').read_text()
        self.assertIn('reference.stamp = ros::Time::now();',code)
        self.assertIn('reference.stamp.toSec(), msg->header.stamp.toSec()',code)
        self.assertIn('controller_.setWorldPva(reference);',code)
        self.assertIn('bool publish_receipts = false;',code)
        self.assertNotIn('reference.stamp = msg->header.stamp',code)
        self.assertNotIn('cmd_vel',code)
        self.assertIn('const auto cfg = controller_.config();',code)

if __name__=='__main__': unittest.main()
