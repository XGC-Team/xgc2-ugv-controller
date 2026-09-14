#!/usr/bin/env python3
"""Compile shared runtime kernel against the actual frozen pre-extraction law."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT/'unicycle_ugv_controller'
BASE = '479e1f01ef6a2e5f264ff8abed5502f7a9d6f7a4'
FILE = 'unicycle_ugv_controller/src/common_types.cpp'
TYPES = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include "unicycle_ugv_controller/common/flatness_kernel.hpp"
struct UgvState { double x=0,y=0,yaw=0,vx=0,vy=0; bool velocity_valid=true; };
struct WorldPvaReference { double x=0,y=0,vx=0,vy=0,ax=0,ay=0; bool valid=true; };
struct ControllerConfig {
 double velocity_dt_min=1.e-6,velocity_dt_max=.25,flatness_kp=1,flatness_kv=2;
 double flatness_v_eps=.15,chassis_max_linear_speed=1.05,chassis_max_yaw_rate=.5235;
 double flatness_lateral_response_length=.5,flatness_lateral_damping=1;
};
struct FlatnessCommandOutput { double accel=0,linear_speed=0,angular_speed=0; bool valid=false; };
bool finitePose(const UgvState& p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw); }
bool worldPvaReady(const WorldPvaReference& q) { return q.valid&&std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.vx)&&std::isfinite(q.vy)&&std::isfinite(q.ax)&&std::isfinite(q.ay); }
double bodySpeedFromWorld(double yaw,double vx,double vy) { return std::cos(yaw)*vx+std::sin(yaw)*vy; }
void boxSaturateUnicycle(double& v,double& w,double vm,double wm) { v=std::max(-vm,std::min(vm,v)); w=std::max(-wm,std::min(wm,w)); }
'''
TEST = r'''
using unicycle_ugv_controller::flatness::evaluate;
using unicycle_ugv_controller::flatness::Trace;
void equal(double a,double b) { assert(std::abs(a-b)<=1.e-13*(1+std::abs(a)+std::abs(b))); }
void compare(const UgvState& p,const WorldPvaReference& q,double vc,double dt,const ControllerConfig& c) {
 const auto a=legacyCommand(p,q,vc,dt,c);
 Trace trace;
 const auto b=evaluate<FlatnessCommandOutput>(p,q,vc,dt,c,&trace);
 assert(a.valid==b.valid); equal(a.accel,b.accel); equal(a.linear_speed,b.linear_speed); equal(a.angular_speed,b.angular_speed);
 assert(trace.valid==b.valid);
}
int main() {
 std::mt19937 rng(93); std::uniform_real_distribution<double> d(-2.,2.);
 ControllerConfig c;
 for(int i=0;i<10000;++i) {
  UgvState p; p.x=d(rng); p.y=d(rng); p.yaw=d(rng); p.vx=d(rng); p.vy=d(rng);
  WorldPvaReference q; q.x=d(rng);q.y=d(rng);q.vx=d(rng);q.vy=d(rng);q.ax=d(rng);q.ay=d(rng);
  compare(p,q,d(rng),.02,c);
 }
 UgvState p; WorldPvaReference q; q.y=1.;
 // The retained low-speed law has a genuine stationary lateral-error equilibrium.
 double vc=0;
 for(int i=0;i<1000;++i) {
  auto out=evaluate<FlatnessCommandOutput>(p,q,vc,.02,c);
  assert(out.valid); assert(out.linear_speed==0);assert(out.angular_speed==0);
  vc=out.linear_speed;
 }
 p.vx=-.4; Trace trace;
 auto a=evaluate<FlatnessCommandOutput>(p,q,0.,.02,c,&trace);
 assert(trace.signed_speed<0);
 auto b=evaluate<FlatnessCommandOutput>(p,q,.2,.02,c);
 equal(a.angular_speed,b.angular_speed); // inverse does not use command state
 equal(b.linear_speed-a.linear_speed,.2); // integrator does use command state
 compare(p,q,0,0,c); p.velocity_valid=false;compare(p,q,0,.02,c);
 p.velocity_valid=true; c.flatness_v_eps=0;compare(p,q,0,.02,c);
 c.flatness_v_eps=.15;c.flatness_lateral_response_length=0;compare(p,q,0,.02,c);
 c.flatness_lateral_response_length=.5;compare(p,q,std::numeric_limits<double>::quiet_NaN(),.02,c);
 std::cout<<"10000 frozen-source equivalence samples, signed/reverse/command-state and lateral-rest counterexample passed\n";
}
'''

class SharedKernelTest(unittest.TestCase):
    def test_runtime_delegates_and_other_algorithms_are_unchanged(self):
        old = subprocess.check_output(['git','show',f'{BASE}:{FILE}'],cwd=ROOT,text=True)
        new = (ROOT/FILE).read_text()
        marker = 'FlatnessCommandOutput computeFlatnessCommand('
        prefix = new.split(marker)[0].replace('#include "unicycle_ugv_controller/common/flatness_kernel.hpp"\n','')
        self.assertEqual(prefix,old.split(marker)[0])
        self.assertIn('return flatness::evaluate<FlatnessCommandOutput>',new)

    def test_actual_frozen_function_and_kernel(self):
        old = subprocess.check_output(['git','show',f'{BASE}:{FILE}'],cwd=ROOT,text=True)
        body = old[old.index('FlatnessCommandOutput computeFlatnessCommand('):].split('\n}  // namespace')[0]
        body = body.replace('computeFlatnessCommand(','legacyCommand(',1)
        with tempfile.TemporaryDirectory() as name:
            path = Path(name)
            (path/'test.cpp').write_text(TYPES+body+TEST)
            subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
                            '-I'+str(PACKAGE/'include'),str(path/'test.cpp'),'-o',str(path/'test')],check=True,timeout=30)
            subprocess.run([str(path/'test')],check=True,timeout=10)

if __name__=='__main__':
    unittest.main()
