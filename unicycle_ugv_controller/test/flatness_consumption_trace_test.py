#!/usr/bin/env python3
"""Production numerical core + tick body; dependency stubs, NOT a ROS ABI test."""
from pathlib import Path
import json
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def extract(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


STUBS = r'''
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "unicycle_ugv_controller/common/flatness_trace.h"
using unicycle_ugv_controller::FlatnessTrace;
using unicycle_ugv_controller::flatnessTraceJson;
namespace ros {
struct Time {
 uint64_t ns=0;
 Time()=default;
 explicit Time(double t): ns(static_cast<uint64_t>(std::llround(t*1e9))) {}
 double toSec() const {return ns*1e-9;}
 uint64_t toNSec() const {return ns;}
};
}
namespace state_machine { struct StateContext {}; }
struct UgvState {
 ros::Time stamp; double x=0,y=0,yaw=0,vx=0,vy=0,yaw_rate=0,speed=0;
 bool velocity_valid=true;
};
struct WorldPvaReference {
 ros::Time stamp,source_stamp; uint32_t source_sequence=0;
 double x=0,y=0,vx=0,vy=0,ax=0,ay=0; bool valid=true;
};
struct ControllerConfig {
 double velocity_dt_min=.0001,velocity_dt_max=.2,flatness_kp=6,flatness_kv=4;
 double flatness_v_eps=.15,flatness_lateral_response_length=.8;
 double flatness_lateral_damping=1,chassis_max_linear_speed=1.05,chassis_max_yaw_rate=1.05;
};
struct FlatnessCommandOutput {double accel=0,linear_speed=0,angular_speed=0; bool valid=false;};
struct ControlCommand {
 ros::Time stamp; double linear_speed=0,angular_speed=0; bool valid=false;
 FlatnessTrace flatness_trace;
};
void check(bool v,const char* message) {if (!v) throw std::runtime_error(message);}
void close(double a,double b) {check(std::abs(a-b)<1e-11,"numerical mismatch");}
'''

FAKE_CONTROLLER = r'''
struct FakeController {
 double now=1.; ControllerConfig cfg; UgvState state; WorldPvaReference ref;
 ControlCommand committed;
 double currentTime() const {return now;}
 ControllerConfig config() const {return cfg;}
 bool worldPvaReady() const {return ref.valid;}
 UgvState controlState() const {return state;}
 WorldPvaReference liftedWorldPva() const {return liftWorldPva(ref,now);}
 void setCommand(ControlCommand c) {committed=c;}
};
struct Custom1State {
 explicit Custom1State(FakeController& c): controller_(c) {}
 FakeController& controller_; double body_speed_=.8,last_tick_time_=.99;
 bool have_tick_time_=true;
 void emitZero(state_machine::StateContext&) {controller_.committed=ControlCommand{};}
 void emitCommandIfDue(state_machine::StateContext&) {}
 void tickFlatness(state_machine::StateContext&);
};
'''

CASES = r'''
int main() {
 FakeController c; Custom1State tick(c); state_machine::StateContext ctx;
 c.state.stamp=ros::Time(.98); c.state.vx=.1;
 c.ref.stamp=ros::Time(.95); c.ref.source_stamp=ros::Time(.9);
 c.ref.source_sequence=17; c.ref.y=.2; c.ref.vx=.1; c.ref.ay=.3;
 tick.tickFlatness(ctx);
 const auto frozen=c.committed;
 check(frozen.valid && frozen.flatness_trace.valid,"missing accepted trace");
 const auto& t=frozen.flatness_trace;
 close(t.command_speed_before_mps,.8); close(t.dt_s,.01);
 check(t.reference_source_stamp_ns==900000000ULL,"lost source stamp");
 check(t.reference_receive_stamp_ns==950000000ULL,"receipt epoch changed");
 check(t.reference_sequence==17,"lost source sequence");
 close(t.reference[1],.2+.5*.3*.05*.05);
 close(t.reference[3],.3*.05);
 close(t.state[3],.1);
 const auto expected=computeFlatnessCommand(c.state,c.liftedWorldPva(),.8,.01,c.cfg);
 close(frozen.linear_speed,expected.linear_speed);
 close(frozen.angular_speed,expected.angular_speed);
 // A later producer callback cannot alter the committed value snapshot.
 c.ref.y=99; c.ref.source_sequence=18; c.state.vx=-1;
 close(frozen.flatness_trace.reference[1],t.reference[1]);
 check(frozen.flatness_trace.reference_sequence==17,"snapshot aliases new input");
 // Repeated simulated time emits the old command, not a new fictional sample.
 tick.tickFlatness(ctx);
 check(c.committed.flatness_trace.reference_sequence==17,"paused clock evaluated new input");
 // Losing the reference must not leave a valid old trace attached to a zero.
 c.ref.valid=false; c.now=1.01; tick.tickFlatness(ctx);
 check(!c.committed.valid && !c.committed.flatness_trace.valid,"stale trace survived zero");
 // Exact zero-speed lateral invariant of the retained production law.
 UgvState s; WorldPvaReference r; double internal=0;
 for (int k=0;k<200;++k) {
   const double tm=k*.01;
   r.y=.15*tm*tm*tm/6; r.vy=.15*tm*tm/2; r.ay=.15*tm;
   const auto o=computeFlatnessCommand(s,r,internal,.01,c.cfg);
   check(o.valid,"bounded lateral PVA rejected");
   close(o.linear_speed,0); close(o.angular_speed,0); internal=o.linear_speed;
 }
 // Formatter guards: no NaN JSON and no stamp precision loss.
 auto bad=t; bad.reference[0]=std::numeric_limits<double>::quiet_NaN();
 check(flatnessTraceJson(bad,1,0,0,0,0).empty(),"NaN serialized");
 auto epoch=t; epoch.reference_source_stamp_ns=1787259267892540528ULL;
 std::cout << flatnessTraceJson(epoch,1000000001ULL,frozen.linear_speed,
     frozen.angular_speed,frozen.linear_speed,frozen.angular_speed) << '\n';
}
'''


class ConsumptionTraceTest(unittest.TestCase):
    def test_actual_evaluation_and_snapshot(self):
        source = (ROOT/'src/common_types.cpp').read_text()
        functions = [extract(source, signature) for signature in (
            'bool finitePose(', 'double clamp(', 'void boxSaturateUnicycle(',
            'double bodySpeedFromWorld(', 'WorldPvaReference liftWorldPva(',
            'bool worldPvaReady(', 'FlatnessCommandOutput computeFlatnessCommand(')]
        tick = extract((ROOT/'src/state_machine/custom1_state.cpp').read_text(),
                       'void Custom1State::tickFlatness(')
        with tempfile.TemporaryDirectory() as directory:
            cpp, binary = Path(directory)/'test.cpp', Path(directory)/'test'
            cpp.write_text(STUBS+'\n'.join(functions)+FAKE_CONTROLLER+tick+CASES)
            subprocess.run([os.environ.get('CXX','g++'),'-std=c++17','-Wall','-Wextra',
                            '-Werror','-O2','-I'+str(ROOT/'include'),str(cpp),'-o',str(binary)],check=True)
            output = subprocess.check_output([str(binary)],text=True)
        trace = json.loads(output)
        self.assertEqual(trace['schema'],'xgc2.flatness.consumed.v1')
        self.assertEqual(trace['reference_source_stamp_ns'],'1787259267892540528')
        self.assertEqual(trace['reference_sequence'],17)
        self.assertEqual(trace['publication_stamp_ns'],'1000000001')
        self.assertEqual(len(trace['consumed_q_xy_vx_vy_ax_ay']),6)


if __name__ == '__main__':
    unittest.main()
