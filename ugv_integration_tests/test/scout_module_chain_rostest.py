#!/usr/bin/env python3
"""The Scout unicycle chain of ugv_modules in the real xgc2-module host, against a kinematic Scout.

The entity is the shipped manifest scout_unicycle.toml (the module libraries of the workspace in place of
its relative paths, a private control socket), run by xgc2-module-host as a supervisor would. Around it
are what the vehicle's ROS side is: a kinematic Scout that integrates cmd_vel and publishes the state
estimate, and this test as the operator and the planner. It checks the chain end to end, over ROS and
over the host's control plane:

- the entity starts, becomes ready and reports through GET /v1/describe and /v1/health;
- a circle requested as a ROS message becomes the active reference, `track` makes the vehicle follow it,
  `stop` stops it, and the generator's reset drops the reference;
- sampled and waypoint requests are served and followed;
- the control plane reconfigures an instance live, refuses a configuration the module refuses, and
  leaves the running configuration alone then;
- SIGTERM ends the host cleanly.
"""

import math
import os
import threading
import time
import unittest

import rospkg
import rospy
import rostest
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Empty, String
from unicycle_reference_trajectory_msgs.msg import (
    ActivePolynomialReference,
    AnalyticReference,
    PlanarReferencePoint,
    ReferenceStatus,
    SampledReference,
    WaypointReferenceRequest,
)

from kinematic_scout import KinematicScout
from module_host import ControlError, ModuleHost

PKG = "ugv_integration_tests"
NS = "/ugv1"
REF = NS + "/alg/unicycle_reference_trajectory"
MANIFEST = os.path.join(rospkg.RosPack().get_path("ugv_modules"), "manifests", "scout_unicycle.toml")


class Probe:
    """The messages of a topic."""

    def __init__(self, topic, message_type):
        self.lock = threading.Lock()
        self.messages = []
        self.subscriber = rospy.Subscriber(topic, message_type, self._on_message, queue_size=50)

    def _on_message(self, message):
        with self.lock:
            self.messages.append((time.monotonic(), message))

    def count(self):
        with self.lock:
            return len(self.messages)

    def last(self):
        with self.lock:
            return self.messages[-1][1] if self.messages else None

    def since(self, count):
        with self.lock:
            return [m for _, m in self.messages[count:]]

    def unregister(self):
        self.subscriber.unregister()


def wait(condition, seconds, message):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not rospy.is_shutdown():
        if condition():
            return
        time.sleep(0.02)
    raise AssertionError("timed out: " + message)


class ScoutModuleChainTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node("scout_module_chain_test", anonymous=True)
        # On the circle that starts at the origin and heads +y.
        self.plant = KinematicScout(NS, pose=(0.0, 0.0, math.pi / 2)).start()
        self.host = ModuleHost(MANIFEST)
        self.exit_status = None
        self.expected_errors = []  # the lines of the host's log that a test provokes
        self.control = Probe(NS + "/custom/statustext", String)
        self.reference = Probe(REF + "/status", ReferenceStatus)
        self.active_analytic = Probe(REF + "/active/analytic", AnalyticReference)
        self.active_polynomial = Probe(REF + "/active/polynomial", ActivePolynomialReference)
        self.active_sampled = Probe(REF + "/active/sampled", SampledReference)
        self.twists = Probe(NS + "/cmd_vel", Twist)
        self.analytic_pub = rospy.Publisher(REF + "/request/analytic", AnalyticReference, queue_size=5)
        self.sampled_pub = rospy.Publisher(REF + "/request/sampled", SampledReference, queue_size=5)
        self.waypoint_pub = rospy.Publisher(REF + "/request/waypoint", WaypointReferenceRequest,
                                            queue_size=5)
        self.reset_pub = rospy.Publisher(REF + "/reset", Empty, queue_size=5)
        self.command_pub = rospy.Publisher(NS + "/command", String, queue_size=5)
        self.host.start()
        self.assertTrue(self.host.wait_ready(20.0), "the entity must become ready: " +
                        str(self.host.describe()["facts"].get("not_ready")))
        wait(lambda: self.control_state() == "Ready" and self.reference_state() == ReferenceStatus.STATE_READY,
             10.0, "the controller and the generator must become Ready")
        wait(lambda: all(p.get_num_connections() > 0 for p in
                         (self.analytic_pub, self.sampled_pub, self.waypoint_pub, self.reset_pub,
                          self.command_pub)), 10.0, "the edge must subscribe to the requests")

    def tearDown(self):
        try:
            self.command_pub.publish(String(data="stop"))
            time.sleep(0.1)
        finally:
            self.plant.stop()
            self.exit_status = self.host.stop()
            self.host.cleanup()
        self.assertEqual(self.exit_status, 0, "the host must end cleanly on SIGTERM:\n" +
                         "\n".join(self.host.output[-20:]))
        errors = [l for l in self.host.output if " ERROR " in l and
                  not any(fragment in l for fragment in self.expected_errors)]
        self.assertFalse(errors, "the host logged errors:\n" + "\n".join(errors))
        self.assertFalse(self.host.logged("panicked"))

    def control_state(self):
        message = self.control.last()
        return message.data if message else ""

    def reference_state(self):
        message = self.reference.last()
        return message.state if message else 0

    def circle_request(self, trajectory_id=5, radius=1.5, speed=0.5):
        request = AnalyticReference()
        request.header.stamp = rospy.Time.now()
        request.request_id = trajectory_id
        request.trajectory_id = trajectory_id
        request.revision = 1
        request.analytic_type = AnalyticReference.ANALYTIC_CIRCLE
        request.duration = 120.0
        request.origin.orientation.w = 1.0
        request.params = [radius, speed, 2.0, -radius, 0.0]  # radius, speed, entry, centre
        return request

    def tracking_error(self, trace, start, radius, speed, after):
        """Worst distance to the circle's reference point over the trace after `after` seconds."""
        worst = 0.0
        w = speed / radius
        for stamp, x, y, _ in trace:
            tau = stamp - start
            if tau < after:
                continue
            rx = -radius + radius * math.cos(w * tau)
            ry = radius * math.sin(w * tau)
            worst = max(worst, math.hypot(x - rx, y - ry))
        return worst

    # ---- the control plane -------------------------------------------------------------------------

    def test_the_control_plane_describes_and_measures_the_running_entity(self):
        facts = self.host.describe()["facts"]
        self.assertTrue(self.host.describe()["ready"])
        self.assertEqual(facts["entity"], "scout")
        self.assertEqual(facts["clock"]["mode"], "steady")
        self.assertEqual({i["name"] for i in facts["instances"]}, {"reference", "controller", "edge"})
        self.assertTrue(all(i["state"] == "running" and i["ready"] for i in facts["instances"]))

        time.sleep(2.0)
        health = self.host.client.get("/v1/health")
        instances = {i["name"]: i for i in health["instances"]}
        self.assertEqual(set(instances), {"reference", "controller", "edge"})
        for name, period_ms in (("reference", 10), ("controller", 2), ("edge", 10)):
            instance = instances[name]
            self.assertEqual(instance["state"], "running", name)
            self.assertEqual(instance["step_errors"], 0, name)
            self.assertEqual(instance["misuse"], 0, name)
            self.assertEqual(instance["period_ns"], period_ms * 1000000, name)
            self.assertGreater(instance["steps"], 0.8 * 2.0 * 1000.0 / period_ms, name)
            self.assertNotEqual(instance["health"], "failed", name)
        # The controller's solver wakes it; the generator and the edge are driven by their periods.
        channels = {c["name"]: c for c in health["channels"]}
        self.assertEqual(channels["state"]["drops"], 0)
        self.assertGreater(channels["state"]["commits"], 100)  # the plant's 100 Hz estimate
        self.assertGreater(channels["controller_status"]["commits"], 5)
        self.assertGreater(channels["reference_status"]["commits"], 10)
        modules = {m["name"] for m in self.host.client.get("/v1/modules")["modules"]}
        self.assertEqual(modules, {"ugv_ros_edge", "ugv_unicycle_reference", "ugv_unicycle_controller"})

    # ---- the chain over ROS ---------------------------------------------------------------------------

    def test_a_circle_requested_over_ros_is_followed_until_stopped(self):
        seen_active = self.active_analytic.count()
        t_request = rospy.Time.now()
        self.analytic_pub.publish(self.circle_request())
        wait(lambda: self.reference_state() == ReferenceStatus.STATE_ACTIVE, 5.0,
             "the generator must activate the circle")
        wait(lambda: self.active_analytic.count() > seen_active, 5.0,
             "the edge must publish the active reference")
        active = self.active_analytic.since(seen_active)[0]
        status = self.reference.last()
        self.assertEqual(status.active_type, ReferenceStatus.TYPE_ANALYTIC)
        self.assertEqual(status.active_trajectory_id, 5)
        self.assertEqual(status.flags, 0)
        self.assertEqual(active.trajectory_id, 5)
        self.assertEqual(active.header.frame_id, "world")
        # The start time is a ROS time a little after the request (the generator's lead time), not a
        # time of the host's clock.
        start = active.start_time.to_sec()
        self.assertGreater(start, t_request.to_sec())
        self.assertLess(start, t_request.to_sec() + 2.0)

        trace_from = len(self.plant.trace)
        self.command_pub.publish(String(data="track"))
        wait(lambda: self.control_state() == "Custom1", 5.0, "the controller must start tracking")
        time.sleep(9.0)
        error = self.tracking_error(self.plant.trace[trace_from:], start, 1.5, 0.5, after=2.5)
        self.assertLess(error, 0.08, "the vehicle must follow the circle")
        x, y, _ = self.plant.pose()
        self.assertGreater(math.hypot(x, y), 0.5, "and have left the start")
        self.assertEqual(self.control_state(), "Custom1")

        # stop: the vehicle is held and the controller is Ready again.
        self.command_pub.publish(String(data="stop"))
        wait(lambda: self.control_state() == "Ready", 3.0, "stop must end the tracking")
        wait(lambda: self.twists.last() is not None and self.twists.last().linear.x == 0.0, 3.0,
             "stop must stop the chassis")
        # the generator's reset drops the reference
        self.reset_pub.publish(Empty())
        wait(lambda: self.reference_state() == ReferenceStatus.STATE_READY and
             self.reference.last().active_type == ReferenceStatus.TYPE_NONE, 5.0,
             "reset must drop the reference")

    def test_a_sampled_reference_is_followed(self):
        # An arc of 0.5 m/s and 0.25 rad/s from the origin, heading +y, as explicit kinematics.
        request = SampledReference()
        request.header.stamp = rospy.Time.now()
        request.trajectory_id = 8
        request.revision = 1
        request.flags = SampledReference.FLAG_EXPLICIT_PLANAR_KINEMATICS
        request.sample_dt = 0.05
        v, w = 0.5, 0.25
        for k in range(0, 321):
            s = 0.05 * k
            yaw = math.pi / 2 + w * s
            point = PlanarReferencePoint()
            point.t_from_start = s
            point.x = (v / w) * (math.sin(yaw) - 1.0)
            point.y = -(v / w) * math.cos(yaw)
            point.yaw = yaw
            point.speed = v
            point.yaw_rate = w
            point.curvature = w / v
            point.vx = v * math.cos(yaw)
            point.vy = v * math.sin(yaw)
            point.ax = -v * w * math.sin(yaw)
            point.ay = v * w * math.cos(yaw)
            point.jx = -v * w * w * math.cos(yaw)
            point.jy = -v * w * w * math.sin(yaw)
            request.points.append(point)
        seen = self.active_sampled.count()
        self.sampled_pub.publish(request)
        wait(lambda: self.reference_state() == ReferenceStatus.STATE_ACTIVE, 5.0,
             "the generator must activate the sampled reference")
        self.assertEqual(self.reference.last().active_type, ReferenceStatus.TYPE_SAMPLED)
        wait(lambda: self.active_sampled.count() > seen, 5.0, "the edge must publish it")
        active = self.active_sampled.since(seen)[0]
        self.assertEqual(len(active.points), 321)
        self.assertAlmostEqual(active.points[320].t_from_start, 16.0, places=9)
        start = active.start_time.to_sec()
        self.command_pub.publish(String(data="track"))
        wait(lambda: self.control_state() == "Custom1", 5.0, "the controller must start tracking")
        trace_from = len(self.plant.trace)
        time.sleep(7.0)
        worst = 0.0
        for stamp, x, y, _ in self.plant.trace[trace_from:]:
            tau = stamp - start
            if tau < 2.0:
                continue
            yaw = math.pi / 2 + w * tau
            rx = (v / w) * (math.sin(yaw) - 1.0)
            ry = -(v / w) * math.cos(yaw)
            worst = max(worst, math.hypot(x - rx, y - ry))
        self.assertLess(worst, 0.1)

    def test_a_waypoint_request_is_planned_and_followed(self):
        request = WaypointReferenceRequest()
        request.header.stamp = rospy.Time()  # as soon as possible
        request.request_id = 21
        request.trajectory_id = 4
        for k in range(3):
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = 1.0 * k
            yaw = math.pi / 2
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            request.waypoints.append(pose)
        request.segment_times = [4.0, 4.0]
        request.desired_speed = 0.4
        request.objective = WaypointReferenceRequest.OBJECTIVE_SEPTIC_INTERPOLATION
        seen = self.active_polynomial.count()
        self.waypoint_pub.publish(request)
        wait(lambda: self.active_polynomial.count() > seen, 8.0, "the generator must plan the waypoints")
        polynomial = self.active_polynomial.since(seen)[0]
        self.assertEqual(polynomial.trajectory_id, 4)
        self.assertEqual(polynomial.order, 7)
        self.assertEqual(len(polynomial.segment_durations), 2)
        self.assertEqual(len(polynomial.coeff_x), 16)
        self.assertEqual(self.reference.last().active_type, ReferenceStatus.TYPE_POLYNOMIAL)
        self.command_pub.publish(String(data="track"))
        wait(lambda: self.control_state() == "Custom1", 5.0, "the controller must start tracking")
        # eight seconds along the line x = 0 from y = 0 to y = 2
        wait(lambda: self.plant.pose()[1] > 1.9, 14.0, "the vehicle must reach the last waypoint")
        x, y, _ = self.plant.pose()
        self.assertLess(abs(x), 0.1)
        self.assertLess(abs(y - 2.0), 0.15)

    # ---- the control plane changes the running entity ------------------------------------------------

    def test_the_control_plane_reconfigures_an_instance_live(self):
        def rate(seconds):
            first = self.control.count()
            time.sleep(seconds)
            return (self.control.count() - first) / seconds

        before = rate(2.0)
        self.assertAlmostEqual(before, 5.0, delta=1.5)  # status_publish_rate_hz = 5

        config = self.host.instance("controller")["config"]
        # a typo is a key the module does not know: refused, and the old configuration runs on
        refused = dict(config)
        refused["status_publish_rte_hz"] = 20.0
        self.expected_errors.append("unknown configuration key 'status_publish_rte_hz'")
        with self.assertRaises(ControlError) as raised:
            self.host.client.post("/v1/instances/configure", {"name": "controller", "config": refused})
        self.assertEqual(raised.exception.body["error"]["code"], "internal")
        self.assertAlmostEqual(rate(2.0), 5.0, delta=1.5)
        self.assertEqual(self.control_state(), "Ready")

        changed = dict(config)
        changed["status_publish_rate_hz"] = 20.0
        self.host.client.post("/v1/instances/configure", {"name": "controller", "config": changed})
        self.assertAlmostEqual(rate(2.0), 20.0, delta=4.0)
        self.assertEqual(self.control_state(), "Ready")

        # the period of an instance changes without the module
        self.host.client.post("/v1/instances/configure", {"name": "reference", "period_ms": 20})
        instances = {i["name"]: i for i in self.host.client.get("/v1/health")["instances"]}
        self.assertEqual(instances["reference"]["period_ns"], 20 * 1000000)
        health = {i["name"]: i for i in self.host.client.get("/v1/health")["instances"]}
        self.assertNotEqual(health["controller"]["health"], "failed")

    def test_a_new_generator_can_replace_the_running_one(self):
        # a circle is active and tracked while the generator is replaced by another instance of the module
        self.analytic_pub.publish(self.circle_request(trajectory_id=6))
        wait(lambda: self.reference_state() == ReferenceStatus.STATE_ACTIVE, 5.0, "active circle")
        self.command_pub.publish(String(data="track"))
        wait(lambda: self.control_state() == "Custom1", 5.0, "tracking")
        # the library under another file name, as the host requires for a new version
        copy = os.path.join(self.host.directory, "libugv_unicycle_reference_next.so")
        with open(os.path.join(self.host.modules, "libugv_unicycle_reference.so"), "rb") as source:
            with open(copy, "wb") as target:
                target.write(source.read())
        self.host.client.post("/v1/modules/load", {"path": copy, "name": "reference_next"})
        trace_from = len(self.plant.trace)
        self.host.client.post("/v1/instances/replace", {"name": "reference", "module": "reference_next"})
        # the new generator starts afresh (Ready, nothing active) and the controller keeps its last
        # reference; the vehicle goes on along the circle for the controller's reference horizon
        wait(lambda: self.reference_state() == ReferenceStatus.STATE_READY, 5.0,
             "the new generator must come up Ready")
        instances = {i["name"]: i for i in self.host.client.get("/v1/health")["instances"]}
        self.assertEqual(instances["reference"]["state"], "running")
        self.assertEqual(instances["reference"]["library"]["name"], "ugv_unicycle_reference")
        self.assertEqual(self.control_state(), "Custom1")
        # a new request is served by the new instance
        self.analytic_pub.publish(self.circle_request(trajectory_id=7))
        wait(lambda: self.reference_state() == ReferenceStatus.STATE_ACTIVE and
             self.reference.last().active_trajectory_id == 7, 5.0, "the new instance must serve")
        time.sleep(1.0)
        self.assertEqual(self.control_state(), "Custom1")


if __name__ == "__main__":
    rostest.rosrun(PKG, "scout_module_chain", ScoutModuleChainTest)
