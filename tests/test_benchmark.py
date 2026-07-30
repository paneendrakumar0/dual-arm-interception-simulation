from __future__ import annotations

import unittest

from dynamic_dual_arm_sim.benchmark import controller_summary, paired_summary, wilson_interval


class BenchmarkTests(unittest.TestCase):
    def test_wilson_interval_contains_observed_rate(self) -> None:
        lower, upper = wilson_interval(8, 10)

        self.assertLess(lower, 0.8)
        self.assertGreater(upper, 0.8)

    def test_controller_summary_aggregates_trials(self) -> None:
        rows = [
            {
                "captured": True,
                "capture_time_seconds": 0.5,
                "minimum_contact_error_m": 0.01,
                "selected_intercept_time_seconds": 0.45,
            },
            {
                "captured": False,
                "capture_time_seconds": None,
                "minimum_contact_error_m": 0.09,
                "selected_intercept_time_seconds": 0.55,
            },
        ]

        summary = controller_summary(rows)

        self.assertEqual(summary["captures"], 1)
        self.assertEqual(summary["success_rate"], 0.5)
        self.assertEqual(summary["mean_contact_error_m"], 0.05)

    def test_paired_summary_counts_controller_differences(self) -> None:
        rows = [
            {"seed": 1, "trial": 0, "controller": "optimized", "captured": True},
            {"seed": 1, "trial": 0, "controller": "fixed_time", "captured": False},
            {"seed": 1, "trial": 1, "controller": "optimized", "captured": True},
            {"seed": 1, "trial": 1, "controller": "fixed_time", "captured": True},
        ]

        summary = paired_summary(rows)

        self.assertEqual(summary["optimized_only"], 1)
        self.assertEqual(summary["both_capture"], 1)


if __name__ == "__main__":
    unittest.main()
