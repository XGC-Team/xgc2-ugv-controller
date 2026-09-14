#!/usr/bin/env python3
"""No-ROS numerical/ownership tests of verbatim production function bodies.

Message, time and publisher classes are narrow test doubles. This is not a
catkin ABI test, ROS transport test, estimator test or chassis acceptance run.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
from flatness_estimated_speed_test import extract, STUBS, SOURCE

ROOT = Path(__file__).resolve().parents[1]


def compile_run(code):
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory)/'case.cpp'
        binary = Path(directory)/'case'
        source.write_text(code)
        subprocess.run([os.environ.get('CXX','g++'),'-std=c++17','-O2','-Wall','-Wextra',
                        '-Werror',str(source),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)


CALLBACK_STUBS = r'''
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#define ROS_WARN_THROTTLE(...) do {} while (0)
namespace ros {
struct Time {
 double value=0; static int reads; static double next;
 static Time now(){++reads;return Time{next};}
};
int Time::reads=0; double Time::next=0;
}
struct Header { ros::Time stamp; uint32_t seq=0; };
namespace unicycle_reference_trajectory_msgs {
struct PlanarPvaReference {
 using ConstPtr=std::shared_ptr<const PlanarPvaReference>;
 Header header; double x=0,y=0,yaw=0,vx=0,vy=0,ax=0,ay=0;
};
}
struct WorldPvaReference {ros::Time stamp;double x=0,y=0,yaw=0,vx=0,vy=0,ax=0,ay=0;bool valid=false;};
struct Controller {WorldPvaReference value;int sets=0;void setWorldPva(WorldPvaReference r){value=r;++sets;}};
struct Publisher {unicycle_reference_trajectory_msgs::PlanarPvaReference value;int count=0;
 void publish(const unicycle_reference_trajectory_msgs::PlanarPvaReference& v){value=v;++count;}};
namespace event_type {constexpr int INPUT_REFERENCE_UPDATED=1;}
double wrapAngle(double a){return std::remainder(a,2*std::acos(-1.));}
class PvaReferenceInputProducer {
public:
 Controller controller_;Publisher accepted_pub_;ros::Time posted;
 void post(int,const char*,const ros::Time& t){posted=t;}
 void callback(const unicycle_reference_trajectory_msgs::PlanarPvaReference::ConstPtr& msg);
};
void check(bool v,const char* message){if(!v)throw std::runtime_error(message);}
'''
CALLBACK_CASES = r'''
int main(){
 PvaReferenceInputProducer p;
 auto m=std::make_shared<unicycle_reference_trajectory_msgs::PlanarPvaReference>();
 m->x=2;m->y=3;m->vx=.4;m->vy=-.1;m->ax=.2;m->header.stamp.value=8;
 m->yaw=std::numeric_limits<double>::quiet_NaN();ros::Time::next=2.25;
 p.callback(m);
 check(ros::Time::reads==1,"acceptance must read the clock exactly once");
 check(p.controller_.value.stamp.value==2.25,"changed receipt-epoch control contract");
 check(p.accepted_pub_.value.header.stamp.value==p.controller_.value.stamp.value,"receipt stamp diverged from stored input");
 check(p.accepted_pub_.value.x==2 && p.accepted_pub_.value.vx==.4 && p.accepted_pub_.value.ax==.2,"receipt changed PVA");
 check(p.accepted_pub_.value.yaw==0 && p.controller_.value.yaw==0,"receipt omitted sanitization");
 check(p.posted.value==2.25,"event and reference use different epochs");
 check(m->header.stamp.value==8,"diagnostic mutated publisher-owned input");
 m->ax=std::numeric_limits<double>::quiet_NaN();p.callback(m);p.callback(nullptr);
 check(p.accepted_pub_.count==1 && p.controller_.sets==1,"invalid input received an acceptance receipt");
 std::cout<<"accepted PVA: receiver epoch, unchanged payload, sanitization, event ownership and invalid-input cases passed\n";
}
'''
REST_CASES = r'''
int main(){
 ControllerConfig c;UgvState s;WorldPvaReference q;q.y=1;
 double command=0;
 // A constant purely lateral target is a feasible stationary Cartesian PVA,
 // but is an exact equilibrium of the preserved low-speed tracking law.
 // Assert that limitation rather than secretly introducing a rotate-in-place
 // switch or claiming that a soft QP cost guarantees tracking feasibility.
 for(int k=0;k<1000;++k){
   const auto o=computeFlatnessCommand(s,q,command,.01,c);
   check(o.valid,"rest counterexample invalid");close(o.linear_speed,0);close(o.angular_speed,0);
   command=o.linear_speed;s.vx=o.linear_speed*std::cos(s.yaw);s.vy=o.linear_speed*std::sin(s.yaw);
   s.x+=s.vx*.01;s.y+=s.vy*.01;s.yaw+=o.angular_speed*.01;
 }
 close(q.y-s.y,1);
 // The measured inverse and command integrator must remain separate states.
 s=UgvState{};q=WorldPvaReference{};s.vx=-.2;q.vx=-.2;q.y=.1;
 auto a=computeFlatnessCommand(s,q,.8,.01,c);
 auto b=computeFlatnessCommand(s,q,-.7,.01,c);
 check(a.valid&&b.valid,"reverse state invalid");close(a.angular_speed,b.angular_speed);
 close(a.linear_speed,.8);close(b.linear_speed,-.7);check(a.angular_speed<0,"lost signed speed");
 std::cout<<"preserved law: exact lateral-rest limitation and independent signed measured/command states passed\n";
}
'''


class AcceptedEpochTest(unittest.TestCase):
    def test_production_callback(self):
        text=(ROOT/'src/input/pva_reference_input_producer.cpp').read_text()
        self.assertIn('topic + "/accepted", queue_size, false', text)
        callback=extract(text,'void PvaReferenceInputProducer::callback(')
        compile_run(CALLBACK_STUBS+callback+CALLBACK_CASES)

    def test_preserved_low_speed_law(self):
        text=SOURCE.read_text()
        functions=[extract(text,name) for name in (
            'bool finitePose(', 'double clamp(', 'void boxSaturateUnicycle(',
            'double bodySpeedFromWorld(', 'bool worldPvaReady(',
            'FlatnessCommandOutput computeFlatnessCommand(')]
        compile_run(STUBS+'\n'.join(functions)+REST_CASES)

if __name__=='__main__':
    unittest.main()
