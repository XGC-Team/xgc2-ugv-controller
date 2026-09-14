#!/usr/bin/env python3
"""Compile the actual production flatness function without a ROS installation.

Only message/config structs are substituted. The production wrapper and its
shared kernel are compiled, not reimplemented in Python. This is a numerical
unit test, not a catkin ABI, state-machine, delay, or vehicle closed-loop test.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / 'src/common_types.cpp'

def extract(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

STUBS = r'''
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "unicycle_ugv_controller/common/flatness_kernel.hpp"
namespace flatness = unicycle_ugv_controller::flatness;
struct UgvState { double x=0,y=0,yaw=0,vx=0,vy=0; bool velocity_valid=true; };
struct WorldPvaReference {
 double x=0,y=0,vx=0,vy=0,ax=0,ay=0; bool valid=true;
};
struct ControllerConfig {
 double velocity_dt_min=.0001,velocity_dt_max=.2,flatness_kp=6,flatness_kv=4;
 double flatness_v_eps=.15,flatness_lateral_response_length=.8;
 double flatness_lateral_damping=1,chassis_max_linear_speed=1.05,chassis_max_yaw_rate=1.05;
};
struct FlatnessCommandOutput { double accel=0,linear_speed=0,angular_speed=0; bool valid=false; };
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void close(double a, double b) { check(std::fabs(a-b)<1e-11, "numeric mismatch"); }
'''

CASES = r'''
int main() {
 ControllerConfig c; UgvState s; WorldPvaReference r;
 s.vx=.1; r.vx=.1; r.y=.2;
 const auto a=computeFlatnessCommand(s,r,.8,.01,c);
 const auto b=computeFlatnessCommand(s,r,-.7,.01,c);
 check(a.valid && b.valid,"valid estimates rejected");
 close(a.angular_speed,b.angular_speed);
 close(a.angular_speed,.1*std::pow(.1/.8,2)*.2/(.01+.0225));
 close(a.linear_speed,.8); close(b.linear_speed,-.7);
 // Measured reverse motion, despite a still-positive commanded speed.
 s.vx=-.1; r.vx=-.1;
 const auto reverse=computeFlatnessCommand(s,r,.8,.01,c);
 check(reverse.valid,"reverse rejected"); close(reverse.angular_speed,-a.angular_speed);
 // Zero measured speed does not acquire fictitious inverse authority.
 s.vx=0; r.vx=0;
 const auto stopped=computeFlatnessCommand(s,r,.8,.01,c);
 check(stopped.valid,"zero speed invalid"); close(stopped.angular_speed,0);
 // A rigid rotation preserves signed longitudinal speed and yaw output.
 s.yaw=std::acos(-1.)/2; s.vx=0; s.vy=.1;
 r.x=-.2; r.y=0; r.vx=0; r.vy=.1;
 const auto rotated=computeFlatnessCommand(s,r,.8,.01,c);
 check(rotated.valid,"rotated rejected"); close(rotated.angular_speed,a.angular_speed);
 // Non-finite velocity/reference must be rejected before saturation.
 s.vx=std::numeric_limits<double>::quiet_NaN();
 check(!computeFlatnessCommand(s,r,.2,.01,c).valid,"NaN velocity accepted");
 s=UgvState{}; r=WorldPvaReference{}; r.ay=std::numeric_limits<double>::infinity();
 check(!computeFlatnessCommand(s,r,.2,.01,c).valid,"infinite reference accepted");
 r=WorldPvaReference{}; c.flatness_v_eps=std::numeric_limits<double>::quiet_NaN();
 check(!computeFlatnessCommand(s,r,.2,.01,c).valid,"NaN epsilon accepted");
 c=ControllerConfig{}; s.velocity_valid=false;
 check(!computeFlatnessCommand(s,r,.2,.01,c).valid,"invalid estimate accepted");
 s.velocity_valid=true;
 check(!computeFlatnessCommand(s,r,.2,0.,c).valid,"zero dt accepted");
 // When command and estimated speed coincide, nominal behavior is unchanged.
 s.vx=.3; r.vx=.3; r.y=.02;
 const auto nominal=computeFlatnessCommand(s,r,.3,.01,c);
 close(nominal.angular_speed,.3*std::pow(.3/.8,2)*.02/(.09+.0225));
 std::cout << "10 estimated-speed flatness regression cases passed\n";
}
'''

class EstimatedSpeedTest(unittest.TestCase):
    def test_production_function(self):
        source = SOURCE.read_text()
        functions = [extract(source, name) for name in (
            'bool finitePose(', 'double clamp(', 'void boxSaturateUnicycle(',
            'double bodySpeedFromWorld(', 'bool worldPvaReady(',
            'FlatnessCommandOutput computeFlatnessCommand(')]
        with tempfile.TemporaryDirectory() as directory:
            cpp=Path(directory)/'test.cpp'; binary=Path(directory)/'test'
            cpp.write_text(STUBS+'\n'.join(functions)+CASES)
            subprocess.run([os.environ.get('CXX','g++'),'-std=c++17','-Wall','-Wextra',
                            '-Werror','-O2','-I'+str(PACKAGE/'include'),str(cpp),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)

if __name__ == '__main__':
    unittest.main()
