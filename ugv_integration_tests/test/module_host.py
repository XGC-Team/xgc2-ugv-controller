"""The xgc2-module host as a process of a ROS-level test.

ModuleHost starts `xgc2-module-host` on a manifest derived from a shipped one: the module libraries are
those of the workspace, the control socket is a private one, and a test may edit the manifest (a topic,
a limit) before it is written. The control plane is the XRPC http.v1 profile on a Unix socket.

The host binary is found in $XGC2_MODULE_HOST, else on the PATH. Without one the tests that need it are
not registered (see CMakeLists.txt); ModuleHostUnavailable is raised for a test run by hand.
"""

import http.client
import json
import os
import shutil
import signal
import socket
import subprocess
import tempfile
import threading
import time
import uuid

import toml


class ModuleHostUnavailable(RuntimeError):
    pass


def host_binary():
    path = os.environ.get("XGC2_MODULE_HOST") or shutil.which("xgc2-module-host")
    if not path or not os.access(path, os.X_OK):
        raise ModuleHostUnavailable("xgc2-module-host is not available (set XGC2_MODULE_HOST)")
    return path


def module_directory():
    """lib/ugv_modules of the workspace (devel or install space) that holds the modules."""
    for prefix in os.environ.get("CMAKE_PREFIX_PATH", "").split(os.pathsep):
        candidate = os.path.join(prefix, "lib", "ugv_modules")
        if os.path.exists(os.path.join(candidate, "libugv_ros_edge.so")):
            return candidate
    raise ModuleHostUnavailable("lib/ugv_modules with the module libraries is not on CMAKE_PREFIX_PATH")


class _UnixConnection(http.client.HTTPConnection):
    def __init__(self, path, timeout):
        super().__init__("local", timeout=timeout)
        self._path = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self._path)


class ControlError(RuntimeError):
    def __init__(self, status, body):
        super().__init__("control call failed with %d: %s" % (status, body))
        self.status = status
        self.body = body


class ControlClient:
    """Calls of the control API (docs/control-api.md of xgc2-module)."""

    def __init__(self, socket_path, instance_id=""):
        self.socket_path = socket_path
        self.instance_id = instance_id

    def call(self, method, route, body=None, timeout_ms=10000):
        connection = _UnixConnection(self.socket_path, timeout_ms / 1000.0)
        headers = {
            "Host": "local",
            "X-Request-ID": uuid.uuid4().hex,
            "X-Xrpc-Timeout-Ms": str(timeout_ms),
            "Content-Type": "application/json",
        }
        if self.instance_id and route != "/v1/describe":
            headers["X-Xrpc-Instance-ID"] = self.instance_id
        # every request of http.v1 carries a JSON object, also a read
        payload = json.dumps(body if body is not None else {})
        try:
            connection.request(method, route, body=payload, headers=headers)
            response = connection.getresponse()
            text = response.read().decode("utf-8")
        finally:
            connection.close()
        data = json.loads(text) if text else {}
        if response.status >= 300:
            raise ControlError(response.status, data)
        return data

    def get(self, route):
        return self.call("GET", route)

    def post(self, route, body):
        return self.call("POST", route, body)


def apply_overrides(parsed, assignments):
    """`instance.path.to.key=json` changes a value in the configuration of an instance of the manifest.

    `controller.config.chassis.max_linear_speed=0.35` sets [instance.config.chassis] of the instance
    "controller". The value is JSON, so 1 is an integer, 1.0 a float and "x" a string.
    """
    for assignment in assignments:
        path, _, text = assignment.partition("=")
        name, *keys = path.split(".")
        instance = next(i for i in parsed["instance"] if i["name"] == name)
        table = instance
        for key in keys[:-1]:
            table = table.setdefault(key, {})
        try:
            value = json.loads(text)
        except ValueError:
            value = text  # a bare word is a string
        table[keys[-1]] = value


class ModuleHost:
    def __init__(self, manifest, edit=None, directory=None, log_level="info", echo=False):
        """`manifest` is a shipped manifest; `edit(dict)` may change the parsed manifest."""
        self.echo = echo
        self.binary = host_binary()
        self.modules = directory or module_directory()
        self.directory = tempfile.mkdtemp(prefix="module-host-")
        os.chmod(self.directory, 0o700)
        self.socket = os.path.join(self.directory, "control.sock")
        self.log_level = log_level
        parsed = toml.load(manifest)
        for module in parsed["module"]:
            module["path"] = os.path.join(self.modules, os.path.basename(module["path"]))
        parsed.setdefault("control", {})["socket"] = self.socket
        if edit:
            edit(parsed)
        self.manifest = os.path.join(self.directory, "entity.toml")
        with open(self.manifest, "w") as stream:
            toml.dump(parsed, stream)
        self.process = None
        self.client = None
        self.output = []
        self._threads = []

    def instance(self, name):
        for instance in toml.load(self.manifest)["instance"]:
            if instance["name"] == name:
                return instance
        raise KeyError(name)

    def check(self):
        """--check: load the libraries, plan the channels; returns the plan."""
        result = subprocess.run([self.binary, "--manifest", self.manifest, "--check"],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60,
                                universal_newlines=True)
        if result.returncode != 0:
            raise RuntimeError("--check failed: " + result.stderr)
        return json.loads(result.stdout)

    def start(self):
        self.process = subprocess.Popen(
            [self.binary, "--manifest", self.manifest, "--control-socket", self.socket,
             "--log-level", self.log_level],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        first = self.process.stdout.readline()
        try:
            reference = json.loads(first)["service_ref"]
        except (ValueError, KeyError):
            self.process.kill()
            raise RuntimeError("the host did not announce its service: %r %s" %
                               (first, self.process.stderr.read()))
        self.client = ControlClient(self.socket, reference["instance_id"])
        for stream in (self.process.stdout, self.process.stderr):
            thread = threading.Thread(target=self._collect, args=(stream,), daemon=True)
            thread.start()
            self._threads.append(thread)

    def _collect(self, stream):
        for line in stream:
            self.output.append(line.rstrip("\n"))
            if self.echo:
                print(line.rstrip("\n"), flush=True)

    def logged(self, fragment):
        return any(fragment in line for line in list(self.output))

    def describe(self):
        return ControlClient(self.socket).get("/v1/describe")

    def wait_ready(self, seconds=20.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.process.poll() is not None:
                raise RuntimeError("the host exited with %s: %s" % (self.process.returncode,
                                                                     "\n".join(self.output)))
            try:
                if self.describe().get("ready"):
                    return True
            except (OSError, ControlError, ValueError):
                pass
            time.sleep(0.05)
        return False

    def stop(self, seconds=10.0):
        """SIGTERM, as a supervisor would; returns the exit status."""
        if self.process is None:
            return None
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
        try:
            status = self.process.wait(seconds)
        except subprocess.TimeoutExpired:
            self.process.kill()
            status = self.process.wait()
        for thread in self._threads:
            thread.join(timeout=2.0)
        return status

    def cleanup(self):
        shutil.rmtree(self.directory, ignore_errors=True)
