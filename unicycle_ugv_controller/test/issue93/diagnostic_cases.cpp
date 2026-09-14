#include "unicycle_ugv_controller/common/flatness_diagnostic.h"
#include <cassert>
#include <limits>
#include <iostream>
struct Time { double t; double toSec()const{return t;} };
struct State { Time stamp{9.99}; double x=1,y=2,yaw=0,vx=-.2,vy=.3,yaw_rate=.4; bool velocity_valid=true;};
struct Reference { Time stamp{9.9}; double x=1.1,y=2.2,vx=-.1,vy=.2,ax=.03,ay=.04; bool valid=true;};
struct Output { double linear_speed=.7,angular_speed=-.1,accel=.5;bool valid=true;};
int main(){
 using namespace unicycle_ugv_controller;using D=FlatnessDiagnostic;
 State s;Reference q;Output o;
 auto a=makeFlatnessDiagnostic(10.,.002,.8,s,q,o);
 assert(a.valid);assert(a.values[D::EstimatedLongitudinalSpeed]==-.2);
 assert(a.values[D::CommandSpeedBefore]==.8);assert(a.values[D::ReferenceReceiptTime]==9.9);
 assert(a.values[D::Qx]==1.1);assert(a.values[D::Px]==1);
 s.x=99;q.x=88;
 auto b=atFlatnessPublication(a,10.004,.65,-.09);
 assert(b.valid&&b.values[D::Px]==1&&b.values[D::Qx]==1.1);
 assert(b.values[D::SolvedLinearSpeed]==.7&&b.values[D::PublishedLinearSpeed]==.65);
 assert(a.values[D::PublishTime]==0&&b.values[D::PublishTime]==10.004);
 assert(!atFlatnessPublication(a,std::numeric_limits<double>::quiet_NaN(),0,0).valid);
 s.velocity_valid=false;assert(!makeFlatnessDiagnostic(10.,.002,.8,s,q,o).valid);
 s.velocity_valid=true;o.valid=false;assert(!makeFlatnessDiagnostic(10.,.002,.8,s,q,o).valid);
 o.valid=true;s.yaw=std::acos(-1.)/2;s.vx=-.3;s.vy=-.2;
 auto c=makeFlatnessDiagnostic(10.,.002,.8,s,q,o);
 assert(std::abs(c.values[D::EstimatedLongitudinalSpeed]+.2)<1e-12);
 assert(std::abs(c.values[D::EstimatedLateralSpeed]-.3)<1e-12);
 assert(D::Count==25);
 std::cout<<"{\"diagnostic_snapshot_cases\":8,\"passed\":true,\"ROS_runtime_tested\":false}\n";
}
