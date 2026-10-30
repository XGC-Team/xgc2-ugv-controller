import unittest

import numpy as np

from unicycle_nmpc.controller import Bounds, CostWeights, MPCConfig


class UnicycleNmpcConfigTests(unittest.TestCase):
    def test_dimensions_and_defaults(self) -> None:
        cfg = MPCConfig()
        self.assertEqual(cfg.steps, 10)
        self.assertTrue(np.isclose(cfg.dt, 0.1))
        self.assertGreater(Bounds().v_max, 0.0)
        self.assertGreater(Bounds().alpha_max, 0.0)
        self.assertGreater(CostWeights().omega, 1.0)
        self.assertEqual(CostWeights().control.shape, (2,))
        self.assertGreater(CostWeights().control[1], 0.0)

    def test_unicycle_model_dimensions(self) -> None:
        nx = 5
        nu = 2
        np_param = nx + nu
        state = np.zeros(nx)
        control = np.zeros(nu)
        parameters = np.zeros(np_param)
        self.assertEqual(state.shape, (5,))
        self.assertEqual(control.shape, (2,))
        self.assertEqual(parameters.shape, (7,))


if __name__ == "__main__":
    unittest.main()

class SDKOriginTests(unittest.TestCase):
    def test_sdk_validation_is_read_only_and_rejects_a_conflicting_source(self) -> None:
        import os
        import sys
        from unittest import mock

        from unicycle_nmpc import controller as module

        before_path = list(sys.path)
        before_ld = os.environ.get("LD_LIBRARY_PATH")
        with mock.patch.dict(os.environ, {"ACADOS_SOURCE_DIR": "/tmp/other-acados"}):
            status = module._installed_acados_status()
        self.assertIn("conflicts", str(status["error"]))
        self.assertFalse(status["casadi"])
        self.assertFalse(status["acados_template"])
        self.assertEqual(sys.path, before_path)
        self.assertEqual(os.environ.get("LD_LIBRARY_PATH"), before_ld)
