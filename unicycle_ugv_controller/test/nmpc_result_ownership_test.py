#!/usr/bin/env python3
"""Compile the real ADT core and worker; only the numerical backend is a barrier double.

Requires the installed math, Eigen and state-machine headers. CXXFLAGS supplies additional
include/sanitizer flags when using a private SDK prefix. No ROS, fake core or fake kernel.
"""
from pathlib import Path
import os
import shlex
import subprocess
import tempfile

BACKEND = r"""
#pragma once
#include <condition_variable>
#include <mutex>
#include <vector>
#include "unicycle_ugv_controller/common/types.h"
#include <xgc2_math/control.hpp>
namespace unicycle_ugv_controller {
using Se2Reference = xgc2_math::control::Se2Reference;
struct ComputeBarrier {
    std::mutex mutex; std::condition_variable cv; bool entered=false,released=false;
    void waitEntered(){std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[&]{return entered;});}
    void release(){std::lock_guard<std::mutex> lock(mutex);released=true;cv.notify_all();}
};
inline ComputeBarrier barrier;
struct UnicycleNmpcSolver { static int horizonSteps(){return 10;} };
struct NmpcTrackingBackend {
    void configure(const ControllerConfig&){} bool enter(){return true;} void exit(){}
    bool compute(const UgvState&,const std::vector<Se2Reference>&,const Time& t,ControlCommand& c){
        std::unique_lock<std::mutex> lock(barrier.mutex);barrier.entered=true;barrier.cv.notify_all();
        barrier.cv.wait(lock,[]{return barrier.released;});
        c.stamp=t;c.linear_speed=.8;c.angular_speed=.2;c.valid=true;return true;
    }
    double solveTimeMs() const{return 0;} int status() const{return 0;}
};
}
"""
MAIN = r"""
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "unicycle_ugv_controller/nmpc/nmpc_execution.h"
using namespace unicycle_ugv_controller;
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
NmpcCompletion success(uint64_t seq,double stamp,double speed=.7){
    ControlCommand c; c.stamp=Time(stamp);c.linear_speed=speed;c.angular_speed=.1;c.valid=true;
    return {seq,c};
}
SolveNmpc request(UnicycleUgvController& c,double now){
    c.update(now,now);
    for(const auto& value:c.outputs())if(const auto* r=std::get_if<SolveNmpc>(&value))return *r;
    throw std::runtime_error("expected solve request");
}
int main(){
    try {
        UgvState state; state.received=true;state.velocity_valid=true;state.estimator_state=3;
        state.stamp=Time(1.0);
        UnicycleUgvController controller(state);
        ControllerConfig cfg;cfg.state_timeout=100;controller.setConfig(cfg);
        reference::AnalyticReference ref;ref.trajectory_id=1;ref.revision=1;ref.duration=100;
        ref.start_time=Time(1.0);ref.analytic_type=reference::AnalyticReference::ANALYTIC_HOLD;
        ref.origin.orientation.w=1;
        check(controller.referenceCache().updateAnalytic(ref),"reference rejected");
        controller.update(1,1);
        check(controller.command(OperatorCommand::kCustom1,1.002,1.002).ok(),"Custom1 entry");
        std::mutex mutex;std::condition_variable cv;std::vector<NmpcCompletion> results;unsigned wakes=0;
        NmpcExecution worker(controller,[&](NmpcCompletion result){
            std::lock_guard<std::mutex> lock(mutex);results.push_back(std::move(result));
        },[]{return 1.01;},[&]{std::lock_guard<std::mutex> lock(mutex);++wakes;cv.notify_all();});
        worker.start();auto first=request(controller,1.004);worker.handle(first);barrier.waitEntered();
        // A busy rejection is also a typed result followed by an owner wake.
        worker.handle({first.sequence+1000,1.005});
        {std::lock_guard<std::mutex> lock(mutex);check(wakes==1&&results.size()==1,"busy rejection lost wake");}
        check(controller.command(OperatorCommand::kStop,1.006,1.006).ok(),"stop");
        ControlCommand reset;reset.valid=true;reset.stamp=Time(1.006);reset.linear_speed=-.25;
        controller.setCommand(reset);
        barrier.release();
        NmpcCompletion old;
        {std::unique_lock<std::mutex> lock(mutex);check(cv.wait_for(lock,std::chrono::seconds(3),[&]{return wakes==2;}),"worker result timeout");old=results.back();}
        check(controller.command().linear_speed==-.25,"old worker overwrote owner command");
        check(controller.command(OperatorCommand::kCustom1,1.01,1.01).ok(),"reentry");
        auto second=request(controller,1.012);
        check(second.sequence>first.sequence,"request ID reused on reentry");
        controller.postCompletion(old);controller.update(1.014,1.014);
        check(!controller.command().valid,"old session accepted");
        controller.postCompletion(success(second.sequence+99,1.012));controller.update(1.016,1.016);
        check(!controller.command().valid,"unrequested ID accepted");
        controller.postCompletion(success(second.sequence,1.012));controller.update(1.018,1.018);
        check(controller.command().linear_speed==.7,"matching result rejected");
        controller.postCompletion(success(second.sequence,1.012,.9));controller.update(1.019,1.019);
        check(controller.command().linear_speed==.7,"duplicate result overwrote command");
        double now=2;
        for(int scenario=0;scenario<5;++scenario){
            controller.command(OperatorCommand::kStop,now,now);
            controller.command(OperatorCommand::kCustom1,now+.002,now+.002);
            auto req=request(controller,now+.004);auto completion=success(req.sequence,now+.004);
            if(scenario==0)completion.command.reset();
            if(scenario==1)completion.command->linear_speed=std::numeric_limits<double>::quiet_NaN();
            if(scenario==2)completion.command->stamp=Time(now+1);
            if(scenario==3)completion.command->stamp=Time(now-1);
            controller.postCompletion(completion);
            controller.update(now+(scenario==4?.06:.006),now+.006);
            check(!controller.command().valid,"invalid or late result accepted");now+=1;
        }
        worker.stop();
        check(wakes==2,"unexpected worker wake after join");
        std::cout<<"PASS: actual ADT core/worker reject old, duplicate, unmatched, invalid and late results; busy completion wakes owner\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
"""

def main():
    package = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="nmpc-ownership-") as tmp:
        root = Path(tmp)
        backend = root / "unicycle_ugv_controller/nmpc/nmpc_tracking_backend.h"
        backend.parent.mkdir(parents=True)
        backend.write_text(BACKEND)
        (root / "test.cpp").write_text(MAIN)
        flags = shlex.split(os.environ.get("CXXFLAGS", ""))
        subprocess.run([
            os.environ.get("CXX", "g++"), "-std=c++17", "-pthread", "-O2", "-Wall", "-Wextra",
            "-I", str(root), "-I", str(package / "include"),
            "-I", str(package.parent / "ugv_reset_client/include"), "-I/usr/include/eigen3",
            *flags, str(root / "test.cpp"),
            *[str(package / "src" / name) for name in ("unicycle_ugv_controller.cpp", "reference_cache.cpp",
                "common_types.cpp", "core_log.cpp", "nmpc/nmpc_execution.cpp")],
            "-o", str(root / "test"),
        ], check=True, timeout=180)
        subprocess.run([str(root / "test")], check=True, timeout=10)

if __name__ == "__main__":
    main()
