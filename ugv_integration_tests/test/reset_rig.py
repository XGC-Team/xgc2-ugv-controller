"""Synthetic vehicles for the Reset integration tests.

A vehicle here is only a plant around a real controller node: it integrates the controller's cmd_vel
(a Scout with actuator lag and the uncommanded lateral velocity -ell * omega, or an ideal
omnidirectional chassis), publishes the canonical `<ns>/pose`, and records everything the Reset
protocol shows on `<ns>/reset/{session,grant,result}` and `<ns>/custom/statustext`. It makes no
wheel-contact, braking, Gazebo or physical-robot claim. Run it only under rostest's isolated master.
"""

import math
import threading
import time

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from std_msgs.msg import String
from ugv_reset_msgs.msg import ResetGrant, ResetResult, ResetSession

OUTCOME_ARRIVED = ResetResult.OUTCOME_ARRIVED


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rectangle(x, y, yaw, half_length, half_width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b)
            for a, b in ((-half_length, -half_width), (half_length, -half_width),
                         (half_length, half_width), (-half_length, half_width))]


def clearance(a, b):
    """Separating-axis clearance of two convex polygons; negative when they overlap."""
    gap = -math.inf
    for polygon in (a, b):
        for i in range(len(polygon)):
            ex, ey = polygon[(i + 1) % len(polygon)][0] - polygon[i][0], polygon[(i + 1) % len(polygon)][1] - polygon[i][1]
            norm = math.hypot(ex, ey)
            nx, ny = -ey / norm, ex / norm
            pa = [nx * p[0] + ny * p[1] for p in a]
            pb = [nx * p[0] + ny * p[1] for p in b]
            gap = max(gap, max(min(pb) - max(pa), min(pa) - max(pb)))
    return gap


class Vehicle:
    registry = []

    def __init__(self, ns, kind, pose, ell=-0.229, chassis=None, scene_free=False):
        Vehicle.registry.append(self)
        self.ns = "/" + ns.strip("/")
        self.kind = kind
        self.scout = kind == "scout"
        self.ell = ell
        self.chassis = chassis or ((0.31, 0.26) if self.scout else (0.205, 0.18))
        self.lock = threading.RLock()
        self.pose = list(pose)
        self.command = [0.0, 0.0, 0.0]
        self.actual = [0.0, 0.0, 0.0]
        self.state = ""
        self.publish_pose = True
        self.trace = []
        self.sessions, self.grants, self.results, self.commands = [], [], [], []
        self.quit = threading.Event()
        self.pose_pub = rospy.Publisher(self.ns + "/pose", PoseStamped, queue_size=1)
        self.command_pub = rospy.Publisher(self.ns + "/command", String, queue_size=1)
        self.target_pub = rospy.Publisher(self.ns + "/reset_pose", Pose2D, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber(self.ns + "/cmd_vel", Twist, self._on_command, queue_size=1),
            rospy.Subscriber(self.ns + "/custom/statustext", String, self._on_state, queue_size=1),
            rospy.Subscriber(self.ns + "/reset/session", ResetSession, self._on_session, queue_size=50),
            rospy.Subscriber(self.ns + "/reset/grant", ResetGrant, self._on_grant, queue_size=50),
            rospy.Subscriber(self.ns + "/reset/result", ResetResult, self._on_result, queue_size=50),
        ]
        self.thread = threading.Thread(target=self._plant, daemon=True)

    # ---- lifecycle -------------------------------------------------------------------------------
    def start(self):
        self.thread.start()
        return self

    def stop(self):
        self.quit.set()
        self.thread.join(timeout=2.0)

    def wait(self, condition, seconds, message):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            with self.lock:
                if condition():
                    return
            time.sleep(0.01)
        with self.lock:
            ok = condition()
        detail = "; " + "; ".join(v.describe() for v in Vehicle.registry) if not ok else ""
        assert ok, message + detail

    def describe(self):
        with self.lock:
            text = "%s state=%s pose=%s command=%s last results=%s" % (
                self.ns, self.state, [round(v, 3) for v in self.pose], self.command,
                [(r.session, r.outcome, r.reason, r.detail) for r in self.results[-2:]])
            if self.sessions:
                last = self.sessions[-1][1]
                text += " last session phase=%s progress=%.2f healthy=%s" % (last.phase, last.progress, last.healthy)
                text += " G=%d ack=%d cert=%s pub=%d" % (last.request_generation, last.acknowledged_grant_id, last.stop_certified, last.header.seq)
                text += " phases=%s" % ["%.1f:%d:%.2f" % (t - self.sessions[0][0], m.phase, m.progress) for t, m in self.sessions[-40:][::5]]
            if self.grants:
                g = self.grants[-1][1]
                text += " last grant state=%s reason=%s s_limit=%.2f via=%d detail=%s" % (
                    g.state, g.reason, g.s_limit, len(g.via), g.detail)
        return text

    # ---- the operator ------------------------------------------------------------------------------
    def set_target(self, x, y, yaw):
        self.target_pub.publish(Pose2D(x=x, y=y, theta=yaw))

    def click_reset(self):
        """Reset on this vehicle only (the namespaced command); returns the number of results so far."""
        with self.lock:
            count = len(self.results)
        self.command_pub.publish(String(data="reset"))
        return count

    def click_stop(self):
        self.command_pub.publish(String(data="stop"))

    # ---- what the protocol shows ----------------------------------------------------------------------
    def result_after(self, count):
        with self.lock:
            return self.results[count] if len(self.results) > count else None

    def stopped(self):
        return max(abs(v) for v in self.command) < 1e-9

    def settled(self):
        return self.stopped() and max(abs(v) for v in self.actual) < 0.01

    def error_to(self, x, y, yaw):
        with self.lock:
            return math.hypot(self.pose[0] - x, self.pose[1] - y), abs(wrap(yaw - self.pose[2]))

    # ---- plumbing -------------------------------------------------------------------------------------
    def _on_command(self, message):
        with self.lock:
            self.command = [message.linear.x, message.linear.y, message.angular.z]
            self.commands.append((time.monotonic(),) + tuple(self.command))

    def _on_state(self, message):
        with self.lock:
            self.state = message.data

    def _on_session(self, message):
        with self.lock:
            self.sessions.append((time.monotonic(), message))

    def _on_grant(self, message):
        with self.lock:
            self.grants.append((time.monotonic(), message))

    def _on_result(self, message):
        with self.lock:
            self.results.append(message)

    def _plant(self):
        previous = time.monotonic()
        while not self.quit.is_set() and not rospy.is_shutdown():
            now = time.monotonic()
            dt = min(now - previous, 0.03)
            previous = now
            with self.lock:
                x, y, yaw = self.pose
                vx, vy, omega = self.command
                if self.scout:
                    self.actual[0] += (1.0 - math.exp(-dt / 0.12)) * (vx - self.actual[0])
                    self.actual[2] += (1.0 - math.exp(-dt / 0.16)) * (omega - self.actual[2])
                    self.actual[1] = -self.ell * self.actual[2]
                else:
                    self.actual = [vx, vy, omega]
                vx, vy, omega = self.actual
                mid = yaw + omega * dt / 2.0
                x += (math.cos(mid) * vx - math.sin(mid) * vy) * dt
                y += (math.sin(mid) * vx + math.cos(mid) * vy) * dt
                yaw = wrap(yaw + omega * dt)
                self.pose = [x, y, yaw]
                self.trace.append((now, x, y, yaw))
                publish = self.publish_pose
            if publish:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "world"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                self.pose_pub.publish(pose)
            self.quit.wait(0.01)


def min_clearance(first, second):
    """Smallest chassis clearance between two vehicles over their traces (paired by time)."""
    import bisect
    with first.lock, second.lock:
        a, b = list(first.trace), list(second.trace)
    times = [t for t, _, _, _ in b]
    best = math.inf
    for t, ax, ay, ayaw in a[::3]:
        j = min(bisect.bisect_left(times, t), len(b) - 1)
        _, bx, by, byaw = b[j]
        best = min(best, clearance(rectangle(ax, ay, ayaw, *first.chassis),
                                   rectangle(bx, by, byaw, *second.chassis)))
    return best
