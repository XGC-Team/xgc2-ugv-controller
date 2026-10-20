#!/usr/bin/env python3
"""The Reset protocol across real nodes: the coordinator node, the vehicles' controller nodes (or the
module entity) and synthetic plants, under rostest's isolated master.

The scenario is a parameter of the test (~scenario): one launch file per scenario.

  mecanum_open     one Wheeltec-like vehicle, no Scene: arrival, repeat click, heading-only Reset, Stop,
                   an idle coordinator, a killed coordinator
  scout_scene      one Scout, an authored Scene: held while the Scene is missing, a route around a box,
                   a Scene that changes under the granted route
  sce_pair         two omnidirectional vehicles head-on on one line (the SCE pair): straight until they
                   would collide, separated, both arrive
  mixed_pair       a Scout and a Mecanum cross: both arrive; one loses its pose stream and ends alone

No claim about wheel contact, braking, Gazebo or hardware is made; the plants are the stated surrogates.
"""

import math
import os
import subprocess
import time
import unittest

import rospy
import rostest
from ugv_reset_msgs.msg import ResetGrant, ResetResult
from xgc2_geometry_msgs.msg import (
    SceneGeometry, SceneObstacle, SceneObstacleState, ScenePart, SceneSnapshot, SceneState)
from xgc2_math.geometry import encode_geometry

from reset_rig import Vehicle, clearance, min_clearance, rectangle, wrap  # noqa: E402

RAD5 = math.radians(5.0)


class Scene:
    """A world-frame Scene with static boxes, published the way the platform's scene adapter does."""

    def __init__(self, namespace):
        from rospy import Publisher
        self.snapshot = None
        self.boxes = []
        self.snapshot_pub = Publisher(namespace + "/snapshot", SceneSnapshot, queue_size=1, latch=True)
        self.state_pub = Publisher(namespace + "/state", SceneState, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._state)

    def _state(self, _event):
        if self.snapshot is None:
            return
        state = SceneState(epoch=self.snapshot.epoch, revision=self.snapshot.revision)
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "world"
        self.state_pub.publish(state)

    def publish(self, boxes):
        """boxes: [(id, x, y, size_x, size_y)]"""
        revision = (self.snapshot.revision + 1) if self.snapshot else 1
        scene = SceneSnapshot(epoch="fixture", revision=revision)
        scene.header.stamp = rospy.Time.now()
        scene.header.frame_id = "world"
        geometries, obstacles = [], []
        for index, (name, x, y, sx, sy) in enumerate(boxes):
            geometry = SceneGeometry()
            geometry.type, geometry.parameters = encode_geometry({"type": "box", "size": [sx, sy, 0.3]})
            geometries.append(geometry)
            obstacle = SceneObstacle(id=name, name=name, motion_type="hold")
            obstacle.pose.position.x, obstacle.pose.position.y = x, y
            obstacle.pose.orientation.w = 1
            part = ScenePart(id="body", geometry_index=index)
            part.pose.orientation.w = 1
            obstacle.parts = [part]
            obstacles.append(obstacle)
        scene.geometries, scene.obstacles = geometries, obstacles
        self.boxes = boxes
        self.snapshot = scene
        self.snapshot_pub.publish(scene)
        self._state(None)

    def stop(self):
        self.timer.shutdown()


def box_polygon(box):
    _, x, y, sx, sy = box
    return rectangle(x, y, 0.0, sx / 2.0, sy / 2.0)


def min_scene_clearance(vehicle, scene, since=0):
    """Smallest chassis clearance to the boxes of the Scene over the trace from index `since`."""
    with vehicle.lock:
        trace = list(vehicle.trace[since:])
    best = math.inf
    for _, x, y, yaw in trace[::3]:
        body = rectangle(x, y, yaw, *vehicle.chassis)
        for box in scene.boxes:
            best = min(best, clearance(body, box_polygon(box)))
    return best


class ResetProtocolTest(unittest.TestCase):
    def setUp(self):
        self.scenario = rospy.get_param("~scenario")
        self.vehicles = []
        self.scene = None

    def tearDown(self):
        for vehicle in self.vehicles:
            vehicle.click_stop()
            vehicle.stop()
        if self.scene is not None:
            self.scene.stop()

    def vehicle(self, *args, **kwargs):
        v = Vehicle(*args, **kwargs).start()
        self.vehicles.append(v)
        return v

    def ready(self, *vehicles):
        for v in vehicles:
            v.wait(lambda v=v: v.state == "Ready" and v.target_pub.get_num_connections() > 0 and
                   v.command_pub.get_num_connections() > 0, 8.0, "the controller must become Ready")

    def arrived(self, vehicle, count, seconds, target):
        vehicle.wait(lambda: len(vehicle.results) > count, seconds, "Reset must end with a result")
        result = vehicle.results[count]
        self.assertEqual(result.outcome, ResetResult.OUTCOME_ARRIVED, result.detail)
        error, yaw_error = vehicle.error_to(*target)
        self.assertLessEqual(error, 0.05)
        self.assertLessEqual(yaw_error, RAD5)
        self.assertLessEqual(result.final_planar_error, 0.05)
        vehicle.wait(lambda: vehicle.state == "Ready" and vehicle.settled(), 3.0, "Ready and stopped")
        return result

    # ------------------------------------------------------------------------------------------------
    def test_scenario(self):
        getattr(self, "scenario_" + self.scenario)()

    def scenario_mecanum_open(self):
        v = self.vehicle("ugv1", "mecanum", (-1.5, 0.8, 0.5))
        self.ready(v)
        target = (0.0, 0.0, 0.0)
        v.set_target(*target)
        time.sleep(0.2)
        # An idle vehicle already tells the coordinator who and where it is: 5 Hz, phase IDLE.
        before = len(v.sessions)
        time.sleep(2.0)
        sessions = len(v.sessions) - before
        self.assertTrue(8 <= sessions <= 12, "about 5 Hz of sessions, got %d in 2 s" % sessions)
        self.assertEqual(v.sessions[-1][1].phase, 0)
        self.assertEqual(v.sessions[-1][1].model, 1)
        self.assertAlmostEqual(v.sessions[-1][1].half_length, 0.345, places=6)

        # The coordinator sleeps while nobody resets: its CPU is about zero.
        pid = subprocess.check_output(["pgrep", "-f", "ugv_reset_coordinator_node"], text=True).split()[0]
        def cpu():
            with open("/proc/%s/stat" % pid) as stream:
                fields = stream.read().rsplit(")", 1)[1].split()
            return (int(fields[11]) + int(fields[12])) / float(os.sysconf("SC_CLK_TCK"))
        start_cpu, start = cpu(), time.monotonic()
        time.sleep(4.0)
        share = (cpu() - start_cpu) / (time.monotonic() - start)
        self.assertLess(share, 0.05, "an idle coordinator must not spin (%.1f%% of a core)" % (100 * share))

        count = v.click_reset()
        self.arrived(v, count, 60.0, target)
        with v.lock:
            session = v.results[count].session
            self.assertEqual(len([r for r in v.results if r.session == session]), 1)
            self.assertTrue(any(g.state == ResetGrant.STATE_GO for _, g in v.grants if g.session == session))
        # Latched: a late subscriber of the result sees the last one.
        late = rospy.wait_for_message("/ugv1/reset/result", ResetResult, timeout=3.0)
        self.assertEqual(late.session, session)

        # A second click on the target is a new session and ends at once.
        count = v.click_reset()
        self.arrived(v, count, 12.0, target)

        # A heading-only Reset still turns the omnidirectional vehicle.
        turn = wrap(v.pose[2] + 1.0)
        x, y = v.pose[0], v.pose[1]
        v.set_target(x, y, turn)
        time.sleep(0.2)
        count = v.click_reset()
        self.arrived(v, count, 25.0, (x, y, turn))

        # Stop ends a session with its typed result.
        v.set_target(1.0, -1.0, 0.0)
        time.sleep(0.2)
        count = v.click_reset()
        v.wait(lambda: not v.stopped(), 5.0, "it must start moving")
        v.click_stop()
        v.wait(lambda: len(v.results) > count and v.state == "Ready", 3.0, "Stop must end the session")
        self.assertEqual(v.results[count].outcome, ResetResult.OUTCOME_STOPPED)

        # The coordinator dies while the vehicle moves: it ends its own session within the heartbeat.
        count = v.click_reset()
        v.wait(lambda: not v.stopped(), 5.0, "it must move again")
        killed = time.monotonic()
        subprocess.check_call(["rosnode", "kill", "/xgc_reset_coordinator"], stdout=subprocess.DEVNULL)
        v.wait(lambda: len(v.results) > count, 3.0, "a vehicle without a coordinator must stop itself")
        self.assertEqual(v.results[count].outcome, ResetResult.OUTCOME_ABORTED_COORDINATOR_LOST)
        self.assertLess(time.monotonic() - killed, 2.5)
        v.wait(lambda: v.stopped() and v.state == "Ready", 2.0, "exact zero, back to Ready")
        # And a click now finds no coordinator at all: typed, within 2 s.
        count = v.click_reset()
        v.wait(lambda: len(v.results) > count, 4.0, "no coordinator: a typed result")
        self.assertEqual(v.results[count].outcome, ResetResult.OUTCOME_REJECTED_NO_COORDINATOR)

    def scenario_scout_scene(self):
        v = self.vehicle("ugv1", "scout", (-2.0, 0.1, 0.2))
        self.scene = Scene("/reset_fixture/scene")
        self.ready(v)
        target = (2.0, 0.0, 0.0)
        v.set_target(*target)
        time.sleep(0.2)
        # A Scene was authored but its snapshot has not arrived: held, with the reason, never driven.
        count = v.click_reset()
        v.wait(lambda: any(g.state == ResetGrant.STATE_HOLD and g.reason == ResetGrant.REASON_SCENE_UNAVAILABLE
                           for _, g in v.grants), 4.0, "the wait for the Scene must be said")
        self.assertTrue(v.stopped())
        # It arrives: the route goes around the box that sits on the straight line.
        box = ("pillar", 0.0, 0.0, 0.6, 0.6)
        self.scene.publish([box])
        result = self.arrived(v, count, 85.0, target)
        with v.lock:
            vias = max(len(g.via) for _, g in v.grants if g.state == ResetGrant.STATE_GO)
            lateral = max(abs(t[2]) for t in v.trace)
        self.assertGreaterEqual(vias, 1, "a detour is needed")
        self.assertGreater(lateral, 0.5)
        self.assertGreater(min_scene_clearance(v, self.scene), 0.0, "never touched the pillar")
        self.assertLess(result.duration, 90.0)

        # A wall appears under the granted route: the vehicle is braked, the reason is the Scene.
        v.set_target(-2.0, 0.0, 0.0)
        time.sleep(0.2)
        count = v.click_reset()
        v.wait(lambda: not v.stopped(), 10.0, "it must start moving")
        time.sleep(1.0)
        with v.lock:
            x, y = v.pose[0], v.pose[1]
            since = len(v.trace)
        self.scene.publish([box, ("wall", x - 1.0, y, 0.3, 2.5)])
        v.wait(lambda: any(g.state == ResetGrant.STATE_BRAKE and g.reason == ResetGrant.REASON_SCENE_CHANGED
                           for _, g in v.grants), 4.0, "a Scene that changes under the route brakes the vehicle")
        v.wait(lambda: len(v.results) > count, 85.0, "and the session still ends with a result")
        self.assertGreater(min_scene_clearance(v, self.scene, since), 0.0, "the new wall was never touched")

    def scenario_sce_pair(self):
        a = self.vehicle("ugv1", "mecanum", (-2.0, 0.0, 0.0))
        b = self.vehicle("ugv2", "mecanum", (2.0, 0.0, math.pi))
        self.ready(a, b)
        a.set_target(2.0, 0.0, 0.0)
        b.set_target(-2.0, 0.0, math.pi)
        time.sleep(0.3)
        ca, cb = a.click_reset(), b.click_reset()
        self.arrived(a, ca, 80.0, (2.0, 0.0, 0.0))
        self.arrived(b, cb, 80.0, (-2.0, 0.0, math.pi))
        self.assertGreater(min_clearance(a, b), 0.0, "the pair passed each other without contact")
        self.assertGreater(max(abs(t[2]) for t in a.trace + b.trace), 0.3,
                           "they were separated: head-on on one line is a collision course")

    def scenario_mixed_pair(self):
        a = self.vehicle("ugv1", "scout", (-2.0, -1.0, 0.0))
        b = self.vehicle("ugv2", "mecanum", (0.0, -2.5, 1.57))
        self.ready(a, b)
        a.set_target(2.0, 1.0, 0.3)
        b.set_target(0.0, 2.5, -1.0)
        time.sleep(0.3)
        ca, cb = a.click_reset(), b.click_reset()
        self.arrived(a, ca, 85.0, (2.0, 1.0, 0.3))
        self.arrived(b, cb, 85.0, (0.0, 2.5, -1.0))
        self.assertGreater(min_clearance(a, b), 0.0)

        # One vehicle loses its pose stream: it ends alone, the other is not touched.
        a.set_target(-2.0, -1.0, 0.0)
        b.set_target(0.0, -2.5, 1.57)
        time.sleep(0.3)
        ca, cb = a.click_reset(), b.click_reset()
        a.wait(lambda: not a.stopped(), 8.0, "ugv1 must move")
        a.publish_pose = False
        a.wait(lambda: len(a.results) > ca, 5.0, "a vehicle without a pose ends its session")
        self.assertEqual(a.results[ca].outcome, ResetResult.OUTCOME_ABORTED_UNHEALTHY)
        self.arrived(b, cb, 85.0, (0.0, -2.5, 1.57))


if __name__ == "__main__":
    rospy.init_node("reset_protocol_test")
    rostest.rosrun("ugv_integration_tests", "reset_protocol", ResetProtocolTest)
