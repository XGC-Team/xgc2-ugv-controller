#!/usr/bin/env python3
"""One Reset coordinator, a Scout and a Mecanum controller node, synthetic plants.

The vehicles run their own controller nodes and know nothing of each other; the
station's coordinator plans both from their requests. This proves the
vehicle/station split end to end: a joint Reset reaches both targets without the
two bodies ever overlapping, every command stays inside the chassis limits, and a
cut link stops exactly the vehicle whose responses stopped. The plants are
parameterized kinematic surrogates (declared slip and lag for the Scout); there
is no wheel-contact, braking, Gazebo or physical-robot claim.
"""

import math
import subprocess
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from std_msgs.msg import String
from ugv_reset_msgs.msg import ResetRequest, ResetResponse
from xgc2_geometry_msgs.msg import SceneObstacle, SceneObstacleState, ScenePart, SceneSnapshot, SceneState

MAX_V = 0.35
MAX_W = 0.5


def corners(pose, length, width):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx, dy in ((length / 2, width / 2), (length / 2, -width / 2),
                           (-length / 2, -width / 2), (-length / 2, width / 2))]


def overlap(a, b):
    """Separating axis test of two oriented rectangles (touching is not overlap)."""
    for shape in (a, b):
        for i in range(4):
            x0, y0 = shape[i]
            x1, y1 = shape[(i + 1) % 4]
            axis = (y0 - y1, x1 - x0)
            pa = [axis[0] * px + axis[1] * py for px, py in a]
            pb = [axis[0] * px + axis[1] * py for px, py in b]
            if max(pa) <= min(pb) or max(pb) <= min(pa):
                return False
    return True


class Vehicle(object):
    """A synthetic vehicle in the namespace `ns`, behind a controller node."""

    def __init__(self, ns, scout, pose, size):
        self.ns = ns
        self.scout = scout
        self.pose = list(pose)
        self.size = size
        self.command = [0.0, 0.0, 0.0]
        self.actual = [0.0, 0.0, 0.0]
        self.state = ""
        self.requests = []
        self.responses = []
        self.commands = []
        self.deliver = True
        self.lock = threading.RLock()
        name = ns.strip("/")
        self.pose_pub = rospy.Publisher(ns + "/pose", PoseStamped, queue_size=1)
        self.command_pub = rospy.Publisher(ns + "/command", String, queue_size=1)
        self.goal_pub = rospy.Publisher(ns + "/reset_pose", Pose2D, queue_size=1, latch=True)
        self.reply_pub = rospy.Publisher("/reset_pair/" + name + "_response", ResetResponse, queue_size=1)
        self.subscribers = [
            rospy.Subscriber(ns + "/cmd_vel", Twist, self.on_command, queue_size=1),
            rospy.Subscriber(ns + "/reset/request", ResetRequest, self.on_request, queue_size=1),
            rospy.Subscriber(ns + "/reset/response", ResetResponse, self.on_response, queue_size=1),
            rospy.Subscriber(ns + "/custom/statustext", String, self.on_state, queue_size=1),
        ]

    def connected(self):
        return (self.goal_pub.get_num_connections() > 0 and self.command_pub.get_num_connections() > 0
                and self.reply_pub.get_num_connections() > 0)

    def on_command(self, msg):
        with self.lock:
            self.command = [msg.linear.x, msg.linear.y, msg.angular.z]
            self.commands.append(tuple(self.command))

    def on_request(self, msg):
        with self.lock:
            self.requests.append(msg)

    def on_response(self, msg):
        with self.lock:
            self.responses.append(msg)
            deliver = self.deliver
        if deliver:
            self.reply_pub.publish(msg)

    def on_state(self, msg):
        with self.lock:
            self.state = msg.data

    def step(self, dt):
        """Advance the plant by dt and publish the pose."""
        with self.lock:
            x, y, yaw = self.pose
            vx, vy, omega = self.command
            if self.scout:
                # Explicit tested surrogate, not a calibrated tire model:
                # negative yaw-coupled lateral velocity and actuator lag.
                self.actual[0] += (1.0 - math.exp(-dt / 0.12)) * (vx - self.actual[0])
                self.actual[2] += (1.0 - math.exp(-dt / 0.16)) * (omega - self.actual[2])
                self.actual[1] = -0.229 * self.actual[2]
            else:
                self.actual = [vx, vy, omega]
            vx, vy, omega = self.actual
            mid = yaw + omega * dt / 2.0
            x += (math.cos(mid) * vx - math.sin(mid) * vy) * dt
            y += (math.sin(mid) * vx + math.cos(mid) * vy) * dt
            yaw = math.atan2(math.sin(yaw + omega * dt), math.cos(yaw + omega * dt))
            self.pose = [x, y, yaw]
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose)

    def moving(self):
        return max(abs(value) for value in self.command) > 0.015

    def stopped(self):
        return max(abs(value) for value in self.command) < 1e-9

    def reset(self):
        """Ask for a Reset; returns the generation of the request that follows."""
        with self.lock:
            count = len(self.requests)
            previous = self.requests[-1].generation if self.requests else None
        self.command_pub.publish(String(data="reset"))
        return count, previous

    def unsubscribe(self):
        for subscriber in self.subscribers:
            subscriber.unregister()


class TwoVehicleResetTest(unittest.TestCase):
    # (start pose, goal) of each vehicle: the paths cross in the middle.
    SCOUT = ((-1.0, -0.6, 0.0), (1.0, 0.6, 0.0))
    MECANUM = ((1.0, -0.6, math.pi), (-1.0, 0.6, math.pi))

    def setUp(self):
        self.lock = threading.RLock()
        self.scout = Vehicle("/ugv1", True, self.SCOUT[0], (0.90, 0.80))
        self.mecanum = Vehicle("/ugv2", False, self.MECANUM[0], (0.41, 0.36))
        self.vehicles = [self.scout, self.mecanum]
        self.quit = threading.Event()
        self.overlaps = 0
        self.closest = float("inf")
        self.snapshot = None
        self.scene_pub = rospy.Publisher("/reset_pair/scene/snapshot", SceneSnapshot, queue_size=1, latch=True)
        self.scene_state_pub = rospy.Publisher("/reset_pair/scene/state", SceneState, queue_size=1)
        self.scene_timer = rospy.Timer(rospy.Duration(.03), self.publish_scene_state)
        self.thread = threading.Thread(target=self.plants, daemon=True)
        self.thread.start()
        self.publish_scene()
        self.wait(lambda: all(v.state == "Ready" and v.connected() for v in self.vehicles), 10.0,
                  "both native owners must become Ready from canonical poses")
        for vehicle, goal in ((self.scout, self.SCOUT[1]), (self.mecanum, self.MECANUM[1])):
            vehicle.goal_pub.publish(Pose2D(x=goal[0], y=goal[1], theta=goal[2]))
        time.sleep(0.2)

    def tearDown(self):
        for vehicle in self.vehicles:
            vehicle.command_pub.publish(String(data="stop"))
        self.scene_timer.shutdown()
        self.quit.set()
        self.thread.join(timeout=1.0)
        for vehicle in self.vehicles:
            vehicle.unsubscribe()

    def plants(self):
        previous = time.monotonic()
        while not self.quit.is_set() and not rospy.is_shutdown():
            now = time.monotonic()
            dt = min(now - previous, 0.03)
            previous = now
            for vehicle in self.vehicles:
                vehicle.step(dt)
            with self.lock:
                a = corners(self.scout.pose, *self.scout.size)
                b = corners(self.mecanum.pose, *self.mecanum.size)
                if overlap(a, b):
                    self.overlaps += 1
                self.closest = min(self.closest, math.hypot(self.scout.pose[0] - self.mecanum.pose[0],
                                                            self.scout.pose[1] - self.mecanum.pose[1]))
            self.quit.wait(0.01)

    def wait(self, condition, seconds, message):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            if condition():
                return
            time.sleep(0.01)
        self.assertTrue(condition(), message + "; " + self.describe())

    def describe(self):
        return "; ".join(
            "%s state=%s pose=%s command=%s last=%s" % (
                v.ns, v.state, [round(p, 3) for p in v.pose], v.command,
                [(r.generation, r.status, r.reason) for r in v.responses[-2:]])
            for v in self.vehicles)

    def publish_scene_state(self, _event):
        if self.snapshot is None:
            return
        state = SceneState(epoch=self.snapshot.epoch, revision=self.snapshot.revision)
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "world"
        state.obstacles = [SceneObstacleState(id=o.id, pose=o.pose) for o in self.snapshot.obstacles]
        self.scene_state_pub.publish(state)

    def publish_scene(self):
        """A shared scene with one far box: an explicit scene, not an implicit empty one."""
        scene = SceneSnapshot(epoch="pair", revision=1)
        scene.header.stamp = rospy.Time.now()
        scene.header.frame_id = "world"
        obstacle = SceneObstacle(id="far-box", name="far fixture box", motion_type="hold")
        obstacle.pose.position.x = 2.5
        obstacle.pose.position.y = 2.5
        obstacle.pose.orientation.w = 1
        part = ScenePart(id="body")
        part.pose.orientation.w = 1
        part.geometry.type = "box"
        part.geometry.size.x = part.geometry.size.y = part.geometry.size.z = .2
        obstacle.parts = [part]
        scene.obstacles = [obstacle]
        self.snapshot = scene
        self.scene_pub.publish(scene)
        self.publish_scene_state(None)

    def start_joint_reset(self):
        starts = [v.reset() for v in self.vehicles]
        self.wait(lambda: all(len(v.requests) > count and v.requests[-1].generation != previous
                              for v, (count, previous) in zip(self.vehicles, starts)),
                  3.0, "both vehicles must emit a request for a new generation")
        return [v.requests[-1].generation for v in self.vehicles]

    def test_crossing_pair_reaches_the_targets_without_overlap(self):
        generations = self.start_joint_reset()
        # The coordinator schedules the pair: it may hold one vehicle at zero while the
        # other moves, so only that somebody moves is required here.
        self.wait(lambda: self.scout.moving() or self.mecanum.moving(), 5.0,
                  "the coordinator's responses must reach a native cmd_vel")
        self.wait(lambda: all(v.state == "Ready" for v in self.vehicles) and all(
            any(r.generation == g and r.status == ResetResponse.ARRIVED for r in v.responses)
            for v, g in zip(self.vehicles, generations)), 90.0,
            "the coordinator must bring both vehicles to their targets")
        with self.lock:
            self.assertEqual(self.overlaps, 0, "the bodies overlapped; closest centers %.3f m" % self.closest)
        for vehicle, goal in ((self.scout, self.SCOUT[1]), (self.mecanum, self.MECANUM[1])):
            with vehicle.lock:
                self.assertLessEqual(math.hypot(vehicle.pose[0] - goal[0], vehicle.pose[1] - goal[1]), 0.05,
                                     vehicle.ns)
                yaw_error = math.atan2(math.sin(goal[2] - vehicle.pose[2]), math.cos(goal[2] - vehicle.pose[2]))
                self.assertLessEqual(abs(yaw_error), math.radians(5.0), vehicle.ns)
                self.assertTrue(vehicle.stopped(), vehicle.ns)
                self.assertTrue(all(all(math.isfinite(c) for c in cmd) for cmd in vehicle.commands))
                self.assertTrue(all(abs(c[0]) <= MAX_V + 1e-9 and abs(c[2]) <= MAX_W + 1e-9
                                    for c in vehicle.commands), vehicle.ns)
                # Each controller emits its own requests and only its own: the stamps are exact echoes.
                known = {(r.generation, r.header.stamp.to_nsec()) for r in vehicle.requests}
                answers = [r for r in vehicle.responses if r.generation in generations]
                self.assertTrue(answers)
                self.assertTrue(all((r.generation, r.header.stamp.to_nsec()) in known for r in answers))
        self.assertTrue(all(abs(c[1]) < 1e-9 for c in self.scout.commands), "a Scout has no lateral command")

    def test_a_cut_link_stops_exactly_the_vehicle_whose_responses_stopped(self):
        self.start_joint_reset()
        self.wait(lambda: self.scout.moving() or self.mecanum.moving(), 5.0,
                  "a vehicle must move before the link is cut")
        moving, other = (self.scout, self.mecanum) if self.scout.moving() else (self.mecanum, self.scout)
        with moving.lock:
            moving.deliver = False
        self.wait(moving.stopped, 0.5, "an expired lease must stop the cmd_vel of " + moving.ns)
        # Its next request acknowledges the zero it really emitted, not a pending proposal.
        self.wait(lambda: moving.requests[-1].applied_command.linear.x == 0.0
                  and moving.requests[-1].applied_command.linear.y == 0.0
                  and moving.requests[-1].applied_command.angular.z == 0.0, 0.5,
                  "the request must carry the applied zero")
        # The coordinator keeps answering the vehicle whose link is intact.
        answers = len(other.responses)
        self.wait(lambda: len(other.responses) > answers + 5, 2.0,
                  "the coordinator must keep answering " + other.ns)
        # A response to the cut vehicle is never acted on while the link is cut.
        self.assertTrue(moving.stopped())
        for vehicle in self.vehicles:
            vehicle.command_pub.publish(String(data="stop"))
        self.wait(lambda: all(v.state == "Ready" and v.stopped() for v in self.vehicles), 3.0,
                  "Stop must cancel Reset on both")
        with self.lock:
            self.assertEqual(self.overlaps, 0, "the bodies overlapped; closest centers %.3f m" % self.closest)

    def test_coordinator_does_not_own_command_or_cmd_vel(self):
        info = subprocess.check_output(["rosnode", "info", "/reset_pair_coordinator"], text=True)
        self.assertNotIn("/command", info)
        self.assertNotIn("cmd_vel", info)
        for vehicle in self.vehicles:
            self.assertIn(vehicle.ns + "/reset/response", info)
            self.assertIn(vehicle.ns + "/reset/request", info)
        for vehicle in self.vehicles:
            node = "/ugv1/unicycle_ugv_controller" if vehicle.scout else "/ugv2/mecanum_ugv_controller"
            node_info = subprocess.check_output(["rosnode", "info", node], text=True)
            self.assertIn(vehicle.ns + "/cmd_vel", node_info)
            # Neither vehicle knows the other one or the station's scene.
            other = "/ugv2" if vehicle.scout else "/ugv1"
            self.assertNotIn(other + "/", node_info.replace("/reset_pair/", ""))


if __name__ == "__main__":
    rospy.init_node("reset_two_vehicles_test")
    rostest.rosrun("ugv_integration_tests", "reset_two_vehicles", TwoVehicleResetTest)
