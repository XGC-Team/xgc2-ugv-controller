#!/usr/bin/env python3
"""The entity manifests of ugv_modules.

- They are the configuration of the two ROS nodes: every value the node configurations carry for the
  modules is in the manifest, unchanged, and nothing else is.
- The simulation manifest differs from the vehicle's in the clock and in the edge's sim_time.
- With the host (--host) and the module libraries (--modules): `xgc2-module-host --check` loads the
  libraries, plans the channels and finds nothing to warn about, and every channel has one writer and
  a reader.
"""

import argparse
import copy
import json
import os
import subprocess
import sys
import tempfile
import unittest

import toml
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
REPOSITORY = os.path.dirname(PACKAGE)
MANIFESTS = os.path.join(PACKAGE, "manifests")
ARGS = argparse.Namespace(host=None, modules=None)

# The keys of the node configurations that the modules do not take: topics (the edge keeps the nodes'
# topics), queue sizes, and the loop rates that are the periods of the instances.
NOT_MODULE_KEYS = {"queue_size", "main_frequency", "control_rate_hz", "reference_path_preview_duration",
                   "reference_path_sample_dt"}


def is_topic(key):
    return key.endswith("_topic")


def module_values(table):
    return {k: v for k, v in table.items() if k not in NOT_MODULE_KEYS and not is_topic(k)}


def load(name):
    return toml.load(os.path.join(MANIFESTS, name))


def instance(manifest, name):
    return next(i for i in manifest["instance"] if i["name"] == name)


class ManifestTest(unittest.TestCase):
    def test_the_instances_are_the_chain_in_the_order_that_loses_no_first_event(self):
        for name in ("scout_unicycle.toml", "scout_unicycle_sim.toml"):
            manifest = load(name)
            self.assertEqual(manifest["entity"], "scout")
            self.assertEqual([i["name"] for i in manifest["instance"]],
                             ["reference", "controller", "edge"], name)
            # the consumers of the edge's events are listed before the edge
            self.assertEqual([m["name"] for m in manifest["module"]], ["edge", "reference", "controller"])
            self.assertEqual({i["name"]: i["period_ms"] for i in manifest["instance"]},
                             {"reference": 10, "controller": 2, "edge": 10})
            for module in manifest["module"]:
                self.assertTrue(module["path"].startswith("../../../lib/ugv_modules/lib"), module["path"])
            # the state machine of the cores belongs to the thread that built it: the host keeps each
            # instance on one thread, and the manifest says so
            for item in manifest["instance"]:
                self.assertEqual(item.get("affinity"), "sticky", item["name"])

    def test_the_values_are_the_ones_of_the_nodes(self):
        manifest = load("scout_unicycle.toml")
        pairs = (
            ("reference", os.path.join(REPOSITORY, "unicycle_reference_trajectory", "config",
                                       "unicycle_reference_trajectory.yaml")),
            ("controller", os.path.join(REPOSITORY, "unicycle_ugv_controller", "config",
                                        "unicycle_ugv_controller.yaml")),
        )
        for name, path in pairs:
            with open(path) as stream:
                node = module_values(yaml.safe_load(stream))
            self.assertEqual(instance(manifest, name)["config"], node,
                             "the configuration of the instance %s is not that of %s" % (name, path))

    def test_the_edge_keeps_the_topics_of_the_nodes(self):
        # no topic is renamed in the manifest: the edge's defaults are the nodes' defaults
        manifest = load("scout_unicycle.toml")
        config = instance(manifest, "edge")["config"]
        self.assertEqual(set(config), {"namespace", "state_source", "tracking_strategy"})
        with open(os.path.join(REPOSITORY, "unicycle_ugv_controller", "config",
                               "unicycle_ugv_controller.yaml")) as stream:
            controller = yaml.safe_load(stream)
        self.assertEqual(config["state_source"], controller["state_source"])
        self.assertEqual(config["tracking_strategy"], controller["tracking_strategy"])
        self.assertEqual(instance(manifest, "controller")["config"]["state_source"],
                         config["state_source"])

    def test_the_simulation_manifest_differs_in_the_clock_only(self):
        vehicle = load("scout_unicycle.toml")
        simulation = load("scout_unicycle_sim.toml")
        self.assertEqual(vehicle["clock"], {"mode": "steady"})
        self.assertEqual(simulation["clock"], {"mode": "external", "channel": "clock"})
        edge = instance(simulation, "edge")
        self.assertIs(edge["config"].pop("sim_time"), True)
        self.assertEqual(edge["bind"].pop("clock"), "clock")
        simulation["clock"] = vehicle["clock"]
        self.assertEqual(simulation, vehicle)

    def test_the_host_loads_the_libraries_and_plans_the_channels(self):
        host = ARGS.host or os.environ.get("XGC2_MODULE_HOST")
        if not host:
            self.skipTest("no xgc2-module-host to check with (--host or $XGC2_MODULE_HOST)")
        modules = ARGS.modules
        self.assertTrue(modules and os.path.isdir(modules), "--modules is the directory of the libraries")
        for name in ("scout_unicycle.toml", "scout_unicycle_sim.toml"):
            manifest = load(name)
            for module in manifest["module"]:
                module["path"] = os.path.join(modules, os.path.basename(module["path"]))
            manifest.pop("control", None)
            with tempfile.TemporaryDirectory() as directory:
                path = os.path.join(directory, "entity.toml")
                with open(path, "w") as stream:
                    toml.dump(manifest, stream)
                result = subprocess.run([host, "--manifest", path, "--check"], stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
            self.assertEqual(result.returncode, 0, name + ": " + result.stderr)
            plan = json.loads(result.stdout)
            self.assertEqual(plan["warnings"], [], name)
            self.assertEqual(plan["instances"], ["reference", "controller", "edge"])
            channels = {c["name"]: c for c in plan["channels"]}
            for channel in channels.values():
                self.assertEqual(channel["writers"], 1, channel["name"])
                if not channel["name"].startswith("edge."):
                    # (the edge's private clock has no reader in the vehicle manifest)
                    self.assertGreaterEqual(channel["readers"], 1 if channel["name"] != "clock" else 0,
                                            channel["name"])
            self.assertEqual(channels["state"]["schema"], "xgc2.ugv.planar_state.v1")
            self.assertEqual(channels["command"]["kind"], "event")
            self.assertEqual(channels["cmd_vel"]["readers"], 1)
            self.assertEqual(channels["active_analytic"]["readers"], 2)  # the controller and the edge
            self.assertEqual(("clock" in channels), name.endswith("_sim.toml"))
            self.assertEqual(("edge.clock" in channels), not name.endswith("_sim.toml"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host")
    parser.add_argument("--modules")
    known, rest = parser.parse_known_args()
    ARGS.host, ARGS.modules = known.host, known.modules
    unittest.main(argv=[sys.argv[0]] + rest)
