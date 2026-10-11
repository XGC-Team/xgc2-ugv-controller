#!/usr/bin/env python3
"""Runs an xgc2-module host as a roslaunch node.

    module_host_node.py MANIFEST [instance.path.to.key=json ...]

The manifest is a shipped one (ugv_modules/manifests); the assignments change the configuration of its
instances for this launch (see module_host.apply_overrides). The node ends with the host, and the host
with SIGINT or SIGTERM, as roslaunch ends its nodes.
"""

import os
import signal
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from module_host import ModuleHost, apply_overrides  # noqa: E402


def main(argv):
    arguments = [a for a in argv[1:] if not a.startswith("__") and not a.startswith("_")]
    manifest, assignments = arguments[0], arguments[1:]
    host = ModuleHost(manifest, edit=lambda parsed: apply_overrides(parsed, assignments), echo=True)
    stop = threading.Event()
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stop.set())
    host.start()
    status = 0
    while not stop.wait(0.1):
        if host.process.poll() is not None:
            status = host.process.returncode or 1
            break
    if host.process.poll() is None:
        status = host.stop()
    host.cleanup()
    return status


if __name__ == "__main__":
    sys.exit(main(sys.argv))
