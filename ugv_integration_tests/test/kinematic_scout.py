"""A kinematic AgileX Scout for the tests of the module chain.

It drives a unicycle with the twist of `<ns>/cmd_vel` (optionally through a first-order actuator lag) and
publishes what a state estimator would: a RigidStateEstimate on `<ns>/alg/state_estimator/state`, or a
PoseStamped on `<ns>/pose`. It follows the ROS clock, so it runs on simulation time too. The pose
it integrates is what the tests compare the controller's work against.
"""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from rigid_state_estimator_msgs.msg import RigidStateEstimate


class KinematicScout:
    def __init__(self, ns, pose=(0.0, 0.0, 0.0), rate=100.0, lag=(0.0, 0.0), source="estimate"):
        self.ns = ns.rstrip("/")
        self.x, self.y, self.yaw = pose
        self.v = 0.0
        self.omega = 0.0
        self.rate = rate
        self.lag = lag
        self.source = source
        self.command = [0.0, 0.0]
        self.commands = []
        self.lock = threading.RLock()
        self.publish_state = True
        self.quit = threading.Event()
        if source == "estimate":
            self.pub = rospy.Publisher(self.ns + "/alg/state_estimator/state", RigidStateEstimate,
                                       queue_size=1)
        else:
            self.pub = rospy.Publisher(self.ns + "/pose", PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber(self.ns + "/cmd_vel", Twist, self._on_command, queue_size=1)
        self.trace = []
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def stop(self):
        self.quit.set()
        self.thread.join(timeout=2.0)
        self.sub.unregister()

    def pose(self):
        with self.lock:
            return (self.x, self.y, self.yaw)

    def _on_command(self, message):
        with self.lock:
            self.command = [message.linear.x, message.angular.z]
            self.commands.append((rospy.get_time(), message.linear.x, message.angular.z))

    def _run(self):
        rate = rospy.Rate(self.rate)
        previous = rospy.get_time()
        while not self.quit.is_set() and not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return
            now = rospy.get_time()
            dt = min(max(now - previous, 0.0), 0.05)
            previous = now
            with self.lock:
                v_cmd, w_cmd = self.command
                if self.lag[0] > 0.0:
                    self.v += (1.0 - math.exp(-dt / self.lag[0])) * (v_cmd - self.v)
                else:
                    self.v = v_cmd
                if self.lag[1] > 0.0:
                    self.omega += (1.0 - math.exp(-dt / self.lag[1])) * (w_cmd - self.omega)
                else:
                    self.omega = w_cmd
                mid = self.yaw + 0.5 * self.omega * dt
                self.x += self.v * math.cos(mid) * dt
                self.y += self.v * math.sin(mid) * dt
                self.yaw = math.atan2(math.sin(self.yaw + self.omega * dt),
                                      math.cos(self.yaw + self.omega * dt))
                x, y, yaw, v, omega = self.x, self.y, self.yaw, self.v, self.omega
                publish = self.publish_state
                self.trace.append((now, x, y, yaw))
            if not publish:
                continue
            stamp = rospy.Time.now()
            if self.source == "estimate":
                estimate = RigidStateEstimate()
                estimate.header.stamp = stamp
                estimate.header.frame_id = "world"
                estimate.position.x = x
                estimate.position.y = y
                estimate.orientation.z = math.sin(0.5 * yaw)
                estimate.orientation.w = math.cos(0.5 * yaw)
                estimate.velocity.x = v * math.cos(yaw)
                estimate.velocity.y = v * math.sin(yaw)
                estimate.angular_velocity.z = omega
                estimate.estimator_state = RigidStateEstimate.STATE_RUNNING
                self.pub.publish(estimate)
            else:
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = "world"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(0.5 * yaw)
                pose.pose.orientation.w = math.cos(0.5 * yaw)
                self.pub.publish(pose)
