from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from dynamic_dual_arm_sim.run import (
    ballistic_position,
    load_config,
    optimize_intercept,
)


CONFIG_PATH = Path(__file__).parents[1] / "configs" / "intercept_demo.json"


class InterceptOptimizerTests(unittest.TestCase):
    def test_ballistic_prediction_applies_gravity(self) -> None:
        position = np.array([0.0, 0.0, 2.0])
        velocity = np.array([1.0, 0.0, 1.0])
        gravity = np.array([0.0, 0.0, -10.0])

        predicted = ballistic_position(position, velocity, gravity, 0.5)

        np.testing.assert_allclose(predicted, [0.5, 0.0, 1.25])

    def test_optimizer_returns_a_sampled_ballistic_point(self) -> None:
        config = load_config(CONFIG_PATH)
        object_position = config.projectile.start_position
        object_velocity = config.projectile.start_velocity
        left_effector = np.array([-0.45, 0.0, 1.8])
        right_effector = np.array([0.45, 0.0, 1.8])

        lead_time, intercept = optimize_intercept(
            config,
            object_position,
            object_velocity,
            left_effector,
            right_effector,
        )

        self.assertGreaterEqual(lead_time, config.intercept.min_lead_time_seconds)
        self.assertLessEqual(lead_time, config.intercept.max_lead_time_seconds)
        self.assertGreaterEqual(intercept[2], config.intercept.stabilize_height_m)
        np.testing.assert_allclose(
            intercept,
            ballistic_position(
                object_position,
                object_velocity,
                config.gravity,
                lead_time,
            ),
        )

    def test_optimizer_uses_closest_apex_when_height_is_infeasible(self) -> None:
        config = load_config(CONFIG_PATH)

        lead_time, _ = optimize_intercept(
            config,
            object_position=np.zeros(3),
            object_velocity=np.zeros(3),
            left_effector=np.zeros(3),
            right_effector=np.zeros(3),
        )

        self.assertEqual(lead_time, config.intercept.min_lead_time_seconds)


if __name__ == "__main__":
    unittest.main()
