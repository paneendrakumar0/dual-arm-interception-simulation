from __future__ import annotations

import argparse
import csv
import json
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, pstdev
from typing import Any

import numpy as np

from dynamic_dual_arm_sim.run import ProjectileConfig, SimConfig, load_config, run_sim


def randomized_config(base: SimConfig, rng: np.random.Generator) -> SimConfig:
    start_position = base.projectile.start_position + rng.normal(
        loc=[0.0, 0.0, 0.0],
        scale=[0.08, 0.08, 0.04],
    )
    start_velocity = base.projectile.start_velocity + rng.normal(
        loc=[0.0, 0.0, 0.0],
        scale=[0.16, 0.16, 0.18],
    )
    mass = float(max(0.12, base.projectile.mass + rng.normal(0.0, 0.04)))
    radius = float(max(0.045, base.projectile.radius + rng.normal(0.0, 0.008)))
    projectile = ProjectileConfig(
        start_position=start_position,
        start_velocity=start_velocity,
        radius=radius,
        mass=mass,
    )
    return replace(base, projectile=projectile)


def numeric_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)):
            values.append(float(value))
    return values


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    captured_rows = [row for row in rows if row["captured"]]
    contact_errors = numeric_values(rows, "minimum_contact_error_m")
    capture_times = numeric_values(captured_rows, "capture_time_seconds")
    return {
        "trial_count": len(rows),
        "captured_count": len(captured_rows),
        "success_rate": round(len(captured_rows) / max(len(rows), 1), 4),
        "mean_contact_error_m": round(mean(contact_errors), 5) if contact_errors else None,
        "std_contact_error_m": round(pstdev(contact_errors), 5) if len(contact_errors) > 1 else 0.0,
        "mean_capture_time_seconds": round(mean(capture_times), 5) if capture_times else None,
        "std_capture_time_seconds": round(pstdev(capture_times), 5) if len(capture_times) > 1 else 0.0,
        "best_contact_error_m": round(min(contact_errors), 5) if contact_errors else None,
        "worst_contact_error_m": round(max(contact_errors), 5) if contact_errors else None,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "trial",
        "captured",
        "capture_time_seconds",
        "minimum_contact_error_m",
        "minimum_dual_capture_distance_m",
        "start_position_m",
        "start_velocity_mps",
        "projectile_mass_kg",
        "projectile_radius_m",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def format_vector(values: np.ndarray) -> str:
    return "[" + ", ".join(f"{float(value):.4f}" for value in values) + "]"


def write_report(path: Path, summary: dict[str, Any], rows: list[dict[str, Any]], seed: int) -> None:
    failures = [row for row in rows if not row["captured"]]
    lines = [
        "# Dual-Arm Interception Monte Carlo Report",
        "",
        f"Generated: {datetime.now(timezone.utc).isoformat()}",
        f"Random seed: `{seed}`",
        "",
        "## Summary",
        "",
        f"- Trials: {summary['trial_count']}",
        f"- Captures: {summary['captured_count']}",
        f"- Success rate: {summary['success_rate'] * 100:.2f}%",
        f"- Mean contact error: {summary['mean_contact_error_m']} m",
        f"- Contact error standard deviation: {summary['std_contact_error_m']} m",
        f"- Mean capture time: {summary['mean_capture_time_seconds']} s",
        f"- Capture time standard deviation: {summary['std_capture_time_seconds']} s",
        f"- Best contact error: {summary['best_contact_error_m']} m",
        f"- Worst contact error: {summary['worst_contact_error_m']} m",
        "",
        "## Interpretation",
        "",
        "This report evaluates the current controller against randomized launch",
        "conditions. A strong research simulation should improve this table over",
        "time, not just produce a single cinematic success case.",
        "",
        "## Failure Cases",
        "",
    ]
    if failures:
        for row in failures[:10]:
            lines.append(
                f"- Trial {row['trial']}: contact error "
                f"{row['minimum_contact_error_m']} m, start velocity "
                f"{row['start_velocity_mps']}"
            )
    else:
        lines.append("- No failed captures in this run.")
    lines.append("")
    path.write_text("\n".join(lines))


def run_experiment_suite(config_path: Path, output: Path, trials: int, seed: int) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    base = load_config(config_path)
    rng = np.random.default_rng(seed)
    rows: list[dict[str, Any]] = []

    for trial in range(trials):
        trial_config = randomized_config(base, rng)
        metrics = run_sim(
            config=trial_config,
            output=output / "trials" / f"trial_{trial:04d}",
            render=False,
            gui=False,
            clean=True,
        )
        rows.append(
            {
                "trial": trial,
                "captured": metrics["captured"],
                "capture_time_seconds": metrics["capture_time_seconds"],
                "minimum_contact_error_m": metrics["minimum_contact_error_m"],
                "minimum_dual_capture_distance_m": metrics["minimum_dual_capture_distance_m"],
                "start_position_m": format_vector(trial_config.projectile.start_position),
                "start_velocity_mps": format_vector(trial_config.projectile.start_velocity),
                "projectile_mass_kg": round(trial_config.projectile.mass, 4),
                "projectile_radius_m": round(trial_config.projectile.radius, 4),
            }
        )

    summary = summarize(rows)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    write_csv(output / "trials.csv", rows)
    write_report(output / "report.md", summary, rows, seed)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run randomized dual-arm interception experiments.")
    parser.add_argument("--config", type=Path, default=Path("configs/intercept_demo.json"))
    parser.add_argument("--output", type=Path, default=Path("outputs/experiments/latest"))
    parser.add_argument("--trials", type=int, default=24)
    parser.add_argument("--seed", type=int, default=20260611)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = run_experiment_suite(args.config, args.output, args.trials, args.seed)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
