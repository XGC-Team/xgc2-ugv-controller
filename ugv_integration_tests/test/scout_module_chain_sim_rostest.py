#!/usr/bin/env python3
"""The Scout unicycle chain in the real xgc2-module host on simulation time.

The entity is the shipped scout_unicycle_sim.toml: its clock is the /clock topic, which the ROS edge
writes to the host. A simulator that runs at twice the speed of the wall clock publishes /clock, and a
kinematic Scout runs on it; the host's timers follow that clock, so the whole chain runs twice as fast
as real time. The entity is not ready before the first /clock.
"""

import math
import os
import threading
import time
import unittest

import rospkg
import rospy
import rostest
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from unicycle_reference_trajectory_msgs.msg import AnalyticReference, ReferenceStatus

from kinematic_scout import KinematicScout
from module_host import ModuleHost

PKG = "ugv_integration_tests"
NS = "/ugv1"
REF = NS + "/alg/unicycle_reference_trajectory"
MANIFEST = os.path.join(rospkg.RosPack().get_path("ugv_modules"), "manifests",
                        "scout_unicycle_sim.toml")
SPEED = 2.0  # simulation seconds per wall second


class Simulator:
    """/clock at SPEED times real time, 100 messages per simulated second."""

    def __init__(self):
        self.pub = rospy.Publisher("/clock", Clock, queue_size=100)
        self.time = 100.0
        self.running = threading.Event()
        self.quit = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def start(self):
        self.running.set()

    def stop(self):
        self.quit.set()
        self.thread.join(timeout=2.0)

    def _run(self):
        step = 0.01
        while not self.quit.is_set():
            if not self.running.wait(0.05):
                continue
            self.time += step
            self.pub.publish(Clock(clock=rospy.Time.from_sec(self.time)))
            time.sleep(step / SPEED)


def wait(condition, seconds, message):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not rospy.is_shutdown():
        if condition():
            return
        time.sleep(0.02)
    raise AssertionError("timed out: " + message)


class ScoutModuleChainSimTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node("scout_module_chain_sim_test", anonymous=True)
        self.simulator = Simulator()
        self.plant = None
        self.host = ModuleHost(MANIFEST)
        self.control_state = ""
        self.reference_state = 0
        self.reference_type = 0
        self.subscribers = [
            rospy.Subscriber(NS + "/custom/statustext", String, self._on_control, queue_size=10),
            rospy.Subscriber(REF + "/status", ReferenceStatus, self._on_reference, queue_size=10),
        ]
        self.analytic_pub = rospy.Publisher(REF + "/request/analytic", AnalyticReference, queue_size=5)
        self.command_pub = rospy.Publisher(NS + "/command", String, queue_size=5)

    def tearDown(self):
        try:
            if self.plant is not None:
                self.plant.stop()
        finally:
            status = self.host.stop()
            self.host.cleanup()
            self.simulator.stop()
        self.assertEqual(status, 0, "the host must end cleanly:\n" + "\n".join(self.host.output[-20:]))
        errors = [l for l in self.host.output if " ERROR " in l]
        self.assertFalse(errors, "the host logged errors:\n" + "\n".join(errors))

    def _on_control(self, message):
        self.control_state = message.data

    def _on_reference(self, message):
        self.reference_state = message.state
        self.reference_type = message.active_type

    def test_the_entity_waits_for_the_simulator_and_then_follows_a_circle_at_twice_real_time(self):
        self.host.start()
        # The simulator has not published time: the host's clock is not valid, so nothing is ready.
        time.sleep(1.0)
        description = self.host.describe()
        self.assertFalse(description["ready"])
        self.assertEqual(description["facts"]["clock"]["mode"], "external")
        self.assertFalse(description["facts"]["clock"]["valid"])
        self.assertTrue(any("clock" in reason for reason in description["facts"]["not_ready"]),
                        str(description["facts"]["not_ready"]))
        self.assertEqual(self.control_state, "")

        wall_start = time.monotonic()
        self.simulator.start()
        self.assertTrue(self.host.wait_ready(20.0), str(self.host.describe()["facts"]["not_ready"]))
        self.assertTrue(self.host.describe()["facts"]["clock"]["valid"])
        # The vehicle exists from the first clock on.
        self.plant = KinematicScout(NS, pose=(0.0, 0.0, math.pi / 2)).start()
        wait(lambda: self.control_state == "Ready" and self.reference_state == ReferenceStatus.STATE_READY,
             15.0, "the controller and the generator must become Ready on simulation time")

        request = AnalyticReference()
        request.request_id = request.trajectory_id = 3
        request.revision = 1
        request.analytic_type = AnalyticReference.ANALYTIC_CIRCLE
        request.duration = 120.0
        request.origin.orientation.w = 1.0
        request.params = [1.5, 0.5, 2.0, -1.5, 0.0]
        active = {}
        rospy.Subscriber(REF + "/active/analytic", AnalyticReference,
                         lambda m: active.setdefault("message", m), queue_size=5)
        time.sleep(0.5)
        sim_request = rospy.Time.now()
        request.header.stamp = sim_request
        self.analytic_pub.publish(request)
        wait(lambda: self.reference_state == ReferenceStatus.STATE_ACTIVE, 10.0, "active reference")
        wait(lambda: "message" in active, 5.0, "the active reference is published")
        # Its times are simulation times: a little after the request, with the generator's lead time.
        start = active["message"].start_time.to_sec()
        self.assertGreater(start, sim_request.to_sec())
        self.assertLess(start, sim_request.to_sec() + 1.0)

        self.command_pub.publish(String(data="track"))
        wait(lambda: self.control_state == "Custom1", 10.0, "tracking")
        trace_from = len(self.plant.trace)
        sim_from = rospy.Time.now().to_sec()
        wall_from = time.monotonic()
        wait(lambda: rospy.Time.now().to_sec() - sim_from >= 12.0, 30.0, "twelve simulated seconds")
        wall = time.monotonic() - wall_from
        sim = rospy.Time.now().to_sec() - sim_from
        # the host ran as fast as the simulator: twelve simulated seconds in about six
        self.assertAlmostEqual(sim / wall, SPEED, delta=0.35)

        worst = 0.0
        w = 0.5 / 1.5
        for stamp, x, y, _ in self.plant.trace[trace_from:]:
            tau = stamp - start
            if tau < 3.0:
                continue
            worst = max(worst, math.hypot(x - (-1.5 + 1.5 * math.cos(w * tau)),
                                          y - 1.5 * math.sin(w * tau)))
        self.assertLess(worst, 0.1, "the vehicle must follow the circle on simulation time")
        travelled = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in
                        zip(self.plant.trace[trace_from:], self.plant.trace[trace_from + 1:]))
        self.assertAlmostEqual(travelled / sim, 0.5, delta=0.06)  # 0.5 m per simulated second

        health = {i["name"]: i for i in self.host.client.get("/v1/health")["instances"]}
        for name in ("reference", "controller", "edge"):
            self.assertEqual(health[name]["step_errors"], 0, name)
            self.assertNotEqual(health[name]["health"], "failed", name)
        self.assertGreater(health["controller"]["timer_fires"], 1000)


if __name__ == "__main__":
    rostest.rosrun(PKG, "scout_module_chain_sim", ScoutModuleChainSimTest)
