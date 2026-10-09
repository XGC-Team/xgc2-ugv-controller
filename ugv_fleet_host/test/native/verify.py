#!/usr/bin/env python3
"""Explicit isolated Classic + native fleet host workflow; never launches a GUI.

This verifies native semantics on the supplied binaries. It does not certify a
Focal release ABI or a physical chassis. ROS inputs are user data; every world
and scene action uses the actual world provider's bounded, instance-bound SDK.
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client
from xml.sax.saxutils import escape

from xgc2_xrpc.http import Client, Fault
from xgc2_xrpc.runtime import Runtime


def port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def identity(pid):
    root = Path(f"/proc/{pid}")
    stat = (root / "stat").read_text().rsplit(")", 1)[1].split()
    return {"pid": pid, "start_ticks": int(stat[19]),
            "exe": os.readlink(root / "exe"),
            "argv": (root / "cmdline").read_bytes().decode().split("\0")[:-1]}


def wait_until(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.02)
    raise AssertionError("bounded native observation timed out")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gzserver", required=True)
    parser.add_argument("--plugins", required=True)
    parser.add_argument("--host", required=True)
    parser.add_argument("--acados", required=True)
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--static-scene", action="store_true")
    args = parser.parse_args()
    evidence_path = Path(args.evidence).resolve()
    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    # Linux Unix sockets require a short path even when the checkout is deeply
    # nested. This newly owned test directory is never a product fallback root.
    root = Path(tempfile.mkdtemp(prefix="ugv-native-"))
    root.chmod(0o700)
    environment = dict(os.environ)
    environment.pop("DISPLAY", None)
    environment.pop("WAYLAND_DISPLAY", None)
    ros_port, gazebo_port = port(), port()
    environment.update(HOME=str(root), ROS_HOME=str(root / "ros"),
                       ROS_LOG_DIR=str(root / "ros-log"), ROS_IP="127.0.0.1",
                       ROS_ROOT="/opt/ros/noetic/share/ros", ROS_DISTRO="noetic",
                       ROS_ETC_DIR="/opt/ros/noetic/etc/ros",
                       ROS_PACKAGE_PATH="/opt/ros/noetic/share",
                       CMAKE_PREFIX_PATH="/opt/ros/noetic",
                       ROS_MASTER_URI=f"http://127.0.0.1:{ros_port}",
                       GAZEBO_MASTER_URI=f"http://127.0.0.1:{gazebo_port}",
                       GAZEBO_IP="127.0.0.1",
                       GAZEBO_PLUGIN_PATH=args.plugins, GAZEBO_MODEL_DATABASE_URI="",
                       PATH="/opt/ros/noetic/bin:/usr/bin:/bin",
                       LD_LIBRARY_PATH=f"{args.acados}/lib:/opt/ros/noetic/lib:/opt/ros/noetic/opt/gazebo/lib/x86_64-linux-gnu")
    version_result = subprocess.run([args.gzserver, "--version"], env=environment,
                                    text=True, capture_output=True, timeout=5)
    version = version_result.stdout
    assert version_result.returncode in (0, 255), version_result.stderr
    assert "version 11.15.1" in version, version
    evidence = {"claim": "isolated native semantics; not Focal ABI or physical chassis",
                "gzserver_version": version.strip(), "root": str(root),
                "display_removed": True, "processes": [], "checks": {}}
    evidence["artifacts"] = {str(Path(path).resolve()): hashlib.sha256(Path(path).read_bytes()).hexdigest()
                             for path in (args.host, Path(args.plugins) / "libxgc2_simulation_world.so",
                                          Path(args.plugins) / "libxgc2_simulation_service.so")}
    processes, clients, logs = [], [], []
    runtime = Runtime(blocking_workers=2, max_calls=16)
    stopping = threading.Event()
    pump = None
    errors = []
    latest_scientific_input = {}

    def start(label, argv):
        log = open(root / f"{label}.log", "w")
        logs.append(log)
        process = subprocess.Popen(argv, env=environment, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        proof = identity(process.pid)
        proof["label"] = label
        evidence["processes"].append(proof)
        return process

    def bind(endpoint, target, service):
        discovery = Client(str(endpoint), runtime=runtime)
        clients.append(discovery)
        ref = json.loads(discovery.call("/v1/describe", method="GET", timeout=2).body)["service_ref"]
        assert ref["target_id"] == target and ref["service"] == service
        assert ref["api_version"] == "v1" and ref["profile"] == "http.v1"
        assert ref["endpoint"] == {"kind": "unix", "address": str(endpoint)}
        assert ref["instance_id"]
        client = Client(str(endpoint), runtime=runtime, instance_id=ref["instance_id"])
        clients.append(client)
        return client, ref

    def query(client, route):
        return json.loads(client.call(route, method="GET", timeout=2).body)

    def mutate_world(client, route, body):
        admitted = json.loads(client.call(route, {**body, "operation_timeout_ms": 5000}, timeout=5).body)
        final = json.loads(client.call(f'/v1/operations/{admitted["id"]}/wait', {}, timeout=6).body)
        assert final["state"] == "succeeded", final
        return final

    try:
        core = start("roscore", ["/opt/ros/noetic/bin/roscore", "-p", str(ros_port),
                                 "--sigint-timeout", "2", "--sigterm-timeout", "2"])
        master = xmlrpc.client.ServerProxy(environment["ROS_MASTER_URI"])

        def master_ready():
            if core.poll() is not None:
                raise AssertionError("isolated ROS master exited")
            try:
                return master.getUri("native-test")[0] == 1
            except (OSError, xmlrpc.client.Error):
                return False

        wait_until(master_ready)
        world_endpoint, host_endpoint = root / "world.sock", root / "fleet.sock"
        world = root / "fixture.world"
        world.write_text(f'''<sdf version="1.6"><world name="isolated">
          <physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
          <gravity>0 0 0</gravity>
          <plugin name="simulation" filename="libxgc2_simulation_world.so">
            <socket_path>{escape(str(world_endpoint))}</socket_path><target_id>test-world</target_id>
            <resource_root>{escape(str(root))}</resource_root>
          </plugin></world></sdf>''')
        gazebo = start("gzserver", [args.gzserver, "--verbose", "--pause", str(world)])

        def world_ready():
            if gazebo.poll() is not None:
                raise AssertionError("isolated headless Classic exited")
            return world_endpoint.exists()

        wait_until(world_ready, 15)
        world_client, world_ref = bind(world_endpoint, "test-world", "xgc2.simulation")
        evidence["world_ref"] = world_ref
        mutate_world(world_client, "/v1/extensions/scene/apply", {
            "epoch": "test-scene", "revision": 1,
            "document": {"schema": "xgc2.scene.v1", "id": "native-test", "frame": "world", "obstacles": []}})
        mutate_world(world_client, "/v1/world/resume", {})
        quiet = query(world_client, "/v1/extensions/scene")
        evidence["quiet_scene"] = quiet
        try:
            quiet_observed = world_client.call("/v1/extensions/scene/observe", {
                "after_serial": quiet["serial"]}, timeout=.2)
            evidence["quiet_observe"] = json.loads(quiet_observed.body)
        except Exception as error:
            evidence["quiet_observe"] = {"type": type(error).__name__, "code": getattr(error, "code", None),
                                         "status": getattr(error, "status", None), "message": str(error),
                                         "disposition": getattr(error, "disposition", None)}
        evidence["quiet_scene_after"] = query(world_client, "/v1/extensions/scene")
        if not args.static_scene:
            mutate_world(world_client, "/v1/extensions/scene/motion", {
                "epoch": "test-scene", "revision": 1, "operation": "play"})
        os.environ.update({key: value for key, value in environment.items()
                           if key.startswith("ROS_")})
        import rospy
        from geometry_msgs.msg import Pose2D, PoseStamped, Twist
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        rospy.init_node("native_fleet_verifier", anonymous=True, disable_signals=True)
        rospy.set_param("/use_sim_time", True)
        rospy.set_param("/ugv_fleet_host", {
            "frequency": 50.0, "input_timeout": .15,
            "state_timeout": 1.0, "obstacle_avoidance": False,
            "fence": {"x_min": -20.0, "x_max": 20.0, "y_min": -20.0, "y_max": 20.0},
            "robots": [{"namespace": robot, "type": kind, "length": length, "width": width,
                        "body_offset_x": 0.0, "body_offset_y": 0.0,
                        "max_vx": .35, "max_vy": vy, "max_omega": .5,
                        "accel_vx": .35, "accel_vy": .35, "accel_omega": .6,
                        "lateral_velocity_per_yaw_bound": lateral}
                       for robot, kind, length, width, vy, lateral in
                       [("ugv1", "mecanum", .41, .36, .35, 0.0),
                        ("ugv2", "scout", .9, .8, 0.0, .25)]]})
        for robot, kind, x in [("ugv1", "mecanum", 0.0), ("ugv2", "unicycle", 4.0)]:
            rospy.set_param(f"/{robot}/{kind}_ugv_controller", {
                "state_source": "platform_pose", "auto_start_tracking": False,
                "reset_initial_x": x, "reset_initial_y": 0.0, "reset_initial_yaw": 0.0})
        clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
        pose_pubs = [rospy.Publisher(f"/{robot}/pose", PoseStamped, queue_size=1)
                     for robot in ("ugv1", "ugv2")]
        goal_pubs = [rospy.Publisher(f"/{robot}/reset_pose", Pose2D, queue_size=1, latch=True)
                     for robot in ("ugv1", "ugv2")]
        samples, states = [[], []], ["", ""]
        sample_lock = threading.Lock()

        def received_command(message, robot):
            with sample_lock:
                samples[robot].append((time.monotonic(), message.linear.x, message.linear.y, message.angular.z))

        def received_state(message, robot):
            states[robot] = message.data

        subscribers = []
        for i, robot in enumerate(("ugv1", "ugv2")):
            subscribers.append(rospy.Subscriber(f"/{robot}/cmd_vel", Twist, received_command,
                                                callback_args=i, queue_size=32))
            subscribers.append(rospy.Subscriber(f"/{robot}/custom/statustext", String, received_state,
                                                callback_args=i, queue_size=1))

        def feed_user_data():
            nonlocal latest_scientific_input
            # Test-only world read supplies exact scientific timestamps to original
            # user ROS data inputs. Production math uses its existing cached inputs.
            try:
                while not stopping.is_set():
                    snapshot = query(world_client, "/v1/extensions/scene")
                    stamp = int(snapshot["simulation_time"]["nanoseconds"])
                    latest_scientific_input = {"world": snapshot, "monotonic": time.monotonic(),
                                              "ros_clock_ns": str(stamp)}
                    seconds, nanos = divmod(stamp, 1_000_000_000)
                    clock_pub.publish(Clock(rospy.Time(seconds, nanos)))
                    for pub, x in zip(pose_pubs, (0.0, 4.0)):
                        pose = PoseStamped()
                        pose.header.stamp = rospy.Time(seconds, nanos)
                        pose.header.frame_id = "world"
                        pose.pose.position.x, pose.pose.orientation.w = x, 1.0
                        pub.publish(pose)
                    stopping.wait(.025)
            except Exception as error:
                errors.append(repr(error))

        pump = threading.Thread(target=feed_user_data, name="native-test-user-data")
        pump.start()
        bootstrap = root / "fleet-bootstrap.json"
        bootstrap.write_text(json.dumps({"schema_version": 1, "binding": {
            "schema_version": 1, "target_id": "test-fleet", "service": "ugv-reset",
            "api_version": "v1", "profile": "http.v1",
            "endpoint": {"kind": "unix", "address": str(host_endpoint)},
            "runtime_grant": "fleet.runtime", "authentication": "local_private",
            "secret_handles": {}, "storage_grants": []}, "grants": {},
            "application": {"sceneServiceRef": world_ref, "worldBoundary": None}}))
        bootstrap.chmod(0o600)
        host = start("fleet", [args.host, "--bootstrap-input", str(bootstrap)])
        wait_until(lambda: host_endpoint.exists() or host.poll() is not None)
        assert host.poll() is None, "native fleet host failed startup"
        fleet, fleet_ref = bind(host_endpoint, "test-fleet", "ugv-reset")
        evidence["fleet_ref"] = fleet_ref
        wait_until(lambda: all(samples[i] for i in (0, 1)))
        wait_until(lambda: query(fleet, "/v1/extensions/reset")["scene"]["operational"])
        time.sleep(.2)

        def snapshot():
            assert not errors, errors
            assert host.poll() is None, "native fleet host exited"
            return query(fleet, "/v1/extensions/reset")

        def start_reset():
            current = snapshot()
            scene = query(world_client, "/v1/extensions/scene")
            expected_scene = {"epoch": scene["epoch"], "revision": scene["revision"],
                              "simulation_time_epoch": scene["simulation_time"]["epoch"]}
            try:
                response = fleet.call("/v1/extensions/reset/start", {
                    "expected_revision": current["revision"], "robots": ["ugv1", "ugv2"],
                    "expected_scene": expected_scene}, timeout=2)
            except Exception:
                evidence["failed_start"] = {"before": current, "expected_scene": expected_scene,
                                            "world": scene, "last_user_input": latest_scientific_input,
                                            "after": snapshot()}
                raise
            assert response.status == 202
            return json.loads(response.body), expected_scene

        def terminal():
            current = snapshot()
            return current if current["state"] in ("arrived", "rejected", "expired", "cancelled") else None

        def new_actual_zeros(after):
            with sample_lock:
                return all(any(t > after and vx == 0 and vy == 0 and yaw == 0
                               for t, vx, vy, yaw in robot) for robot in samples)

        before = time.monotonic()
        started, initial_fence = start_reset()
        arrived = wait_until(terminal)
        assert arrived["state"] == "arrived", arrived
        assert all(robot["state"] == "arrived" and robot["generation"] > 0 for robot in arrived["robots"])
        wait_until(lambda: new_actual_zeros(before))
        evidence["checks"]["mixed_native_arrived_with_actual_zero"] = arrived
        try:
            fleet.call("/v1/extensions/reset/start", {
                "expected_revision": arrived["revision"], "robots": ["ugv1"],
                "expected_scene": {**initial_fence, "simulation_time_epoch": initial_fence["simulation_time_epoch"] + 1}})
            raise AssertionError("stale scientific epoch admitted")
        except Fault as fault:
            assert fault.status == 409, fault
        assert snapshot()["operation"] == arrived["operation"]
        evidence["checks"]["stale_scientific_epoch_has_no_effect"] = True
        for pub, x in zip(goal_pubs, (1.0, 5.0)):
            pub.publish(Pose2D(x, 0, 0))
        time.sleep(.2)
        started, _ = start_reset()
        wait_until(lambda: snapshot()["state"] == "running")
        current = snapshot()
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as observer:
            held = observer.submit(fleet.call, "/v1/extensions/reset/observe", {
                "operation": current["operation"], "after_revision": current["revision"]}, timeout=3)
            time.sleep(.08)
            assert not held.done(), "native observer did not hold the current revision"
            before = time.monotonic()
            cancelled_admission = fleet.call("/v1/extensions/reset/cancel", {
                "expected_revision": current["revision"], "operation": current["operation"]}, timeout=2)
            assert cancelled_admission.status == 202
            changed = json.loads(held.result(timeout=3).body)
        assert changed["revision"] > current["revision"]
        cancelled = wait_until(terminal)
        assert cancelled["state"] == "cancelled", cancelled
        wait_until(lambda: new_actual_zeros(before))
        evidence["checks"]["cancel_native_exit_and_new_actual_zero"] = cancelled
        evidence["checks"]["observe_tracks_native_revision"] = changed
        started, old_fence = start_reset()
        wait_until(lambda: snapshot()["state"] == "running")
        before = time.monotonic()
        rewound = mutate_world(world_client, "/v1/world/reset", {"scope": "all_entities", "reset_time": True})
        assert rewound["result"]["time"]["epoch"] == old_fence["simulation_time_epoch"] + 1
        rejected = wait_until(terminal)
        assert rejected["state"] == "rejected", rejected
        wait_until(lambda: new_actual_zeros(before))
        evidence["checks"]["world_reset_time_fences_existing_reset_and_zero"] = rejected
        evidence["passed"] = True
    except Exception as error:
        evidence["passed"] = False
        evidence["error"] = repr(error)
        raise
    finally:
        stopping.set()
        cleanup_errors = []
        if pump:
            pump.join(3)
            if pump.is_alive():
                cleanup_errors.append("test user data owner did not quiesce")
        for client in reversed(clients):
            try:
                client.close()
            except Exception as error:
                cleanup_errors.append(repr(error))
        try:
            runtime.close(timeout=3)
        except Exception as error:
            cleanup_errors.append(repr(error))
        for process, proof in reversed(list(zip(processes, evidence["processes"]))):
            if process.poll() is None:
                assert identity(process.pid)["start_ticks"] == proof["start_ticks"]
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    assert identity(process.pid)["start_ticks"] == proof["start_ticks"]
                    process.terminate()
                    process.wait(timeout=3)
            proof["exit_code"] = process.returncode
            proof["wait_completed"] = True
        for log in logs:
            log.close()
        evidence_path.write_text(json.dumps(evidence, indent=2) + "\n")
        print(json.dumps({"passed": evidence.get("passed", False), "evidence": str(evidence_path),
                          "checks": list(evidence["checks"]), "processes": evidence["processes"]}))
        assert not cleanup_errors, cleanup_errors


if __name__ == "__main__":
    main()
