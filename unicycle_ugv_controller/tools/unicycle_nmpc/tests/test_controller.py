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
