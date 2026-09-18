#!/usr/bin/env python3
"""SCE Reset: no Scene publisher, still drive to the frozen target."""

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from std_msgs.msg import String
from ugv_reset_safety.msg import ResetRequest, ResetResponse


class ResetCoordinatorOpenTest(unittest.TestCase):
    NS = "/reset_open_robot"

    def setUp(self):
        self.lock = threading.RLock()
        self.pose = [-0.6, 0.25, 0.35]
        self.command = [0.0, 0.0, 0.0]
        self.state = ""
        self.requests = []
        self.responses = []
        self.quit = threading.Event()
        self.pose_pub = rospy.Publisher(self.NS + "/pose", PoseStamped, queue_size=1)
        self.command_pub = rospy.Publisher(self.NS + "/command", String, queue_size=1)
        self.goal_pub = rospy.Publisher(self.NS + "/reset_pose", Pose2D, queue_size=1, latch=True)
        self.reply_pub = rospy.Publisher("/reset_open/controller_response", ResetResponse, queue_size=1)
        self.subscribers = [
            rospy.Subscriber(self.NS + "/cmd_vel", Twist, self.on_command, queue_size=1),
            rospy.Subscriber(self.NS + "/reset/request", ResetRequest, self.on_request, queue_size=1),
            rospy.Subscriber(self.NS + "/reset/response", ResetResponse, self.on_response, queue_size=1),
            rospy.Subscriber(self.NS + "/custom/statustext", String, self.on_state, queue_size=1),
        ]
        self.thread = threading.Thread(target=self.plant, daemon=True)
        self.thread.start()
        self.wait(lambda: self.state == "Ready" and self.goal_pub.get_num_connections() > 0 and
                  self.command_pub.get_num_connections() > 0 and self.reply_pub.get_num_connections() > 0,
                  5.0, "native owner must become Ready")
        self.goal_pub.publish(Pose2D(x=0.0, y=0.0, theta=0.0))
        time.sleep(0.15)

    def tearDown(self):
        self.command_pub.publish(String(data="stop"))
        self.quit.set()
        self.thread.join(timeout=1.0)

    def wait(self, condition, seconds, message):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            with self.lock:
                if condition():
                    return
            time.sleep(0.01)
        with self.lock:
            self.assertTrue(condition(), message + "; state=" + str(self.state) +
                            "; pose=" + str(self.pose) + "; command=" + str(self.command))

    def on_command(self, msg):
        with self.lock:
            self.command = [msg.linear.x, msg.linear.y, msg.angular.z]

    def on_request(self, msg):
        with self.lock:
            self.requests.append(msg)

    def on_response(self, msg):
        with self.lock:
            self.responses.append(msg)
        self.reply_pub.publish(msg)

    def on_state(self, msg):
        with self.lock:
            self.state = msg.data

    def plant(self):
        previous = time.monotonic()
        while not self.quit.is_set() and not rospy.is_shutdown():
            now = time.monotonic()
            dt = min(now - previous, 0.03)
            previous = now
            with self.lock:
                x, y, yaw = self.pose
                vx, vy, omega = self.command
                midpoint_yaw = yaw + omega * dt / 2.0
                x += (math.cos(midpoint_yaw) * vx - math.sin(midpoint_yaw) * vy) * dt
                y += (math.sin(midpoint_yaw) * vx + math.cos(midpoint_yaw) * vy) * dt
                yaw = math.atan2(math.sin(yaw + omega * dt), math.cos(yaw + omega * dt))
                self.pose = [x, y, yaw]
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"
            pose.pose.position.x = self.pose[0]
            pose.pose.position.y = self.pose[1]
            pose.pose.orientation.z = math.sin(self.pose[2] / 2.0)
            pose.pose.orientation.w = math.cos(self.pose[2] / 2.0)
            self.pose_pub.publish(pose)
            self.quit.wait(0.01)

    def test_reset_without_scene_moves_and_arrives(self):
        with self.lock:
            count = len(self.requests)
        self.command_pub.publish(String(data="reset"))
        self.wait(lambda: len(self.requests) > count, 2.0, "Reset must emit a request")
        generation = self.requests[-1].generation
        time.sleep(0.6)
        with self.lock:
            self.assertFalse(
                any(r.generation == generation and r.status == ResetResponse.REJECTED
                    for r in self.responses),
                "must not reject for a missing Scene")
        self.wait(lambda: max(abs(v) for v in self.command) > 0.015, 3.0,
                  "direct Reset must publish a non-zero command without Scene")
        self.wait(lambda: self.state == "Ready" and any(r.generation == generation and
                  r.status == ResetResponse.ARRIVED for r in self.responses),
                  15.0, "direct Reset must arrive without Scene")
        with self.lock:
            self.assertLessEqual(math.hypot(self.pose[0], self.pose[1]), 0.05)
            self.assertLessEqual(abs(math.atan2(math.sin(self.pose[2]), math.cos(self.pose[2]))),
                                 math.radians(5.0))


if __name__ == "__main__":
    rospy.init_node("reset_coordinator_open_rostest")
    rostest.rosrun("ugv_reset_safety", "reset_coordinator_open_no_scene", ResetCoordinatorOpenTest)
