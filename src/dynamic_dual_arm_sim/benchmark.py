from __future__ import annotations

import argparse
import csv
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from statistics import mean
from typing import Any

from dynamic_dual_arm_sim.experiments import run_experiment_suite


def wilson_interval(successes: int, total: int, z: float = 1.96) -> tuple[float, float]:
    if total == 0:
        return 0.0, 0.0
    rate = successes / total
    denominator = 1.0 + z * z / total
    center = (rate + z * z / (2.0 * total)) / denominator
    margin = (
        z
        * math.sqrt(rate * (1.0 - rate) / total + z * z / (4.0 * total * total))
        / denominator
    )
    return center - margin, center + margin


def read_trial_rows(path: Path, seed: int, controller: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(newline="") as handle:
        for raw in csv.DictReader(handle):
            rows.append(
                {
                    "seed": seed,
                    "trial": int(raw["trial"]),
                    "controller": controller,
                    "captured": raw["captured"].lower() == "true",
                    "capture_time_seconds": (
                        float(raw["capture_time_seconds"])
                        if raw["capture_time_seconds"]
                        else None
                    ),
                    "minimum_contact_error_m": float(raw["minimum_contact_error_m"]),
                    "selected_intercept_time_seconds": float(
                        raw["selected_intercept_time_seconds"]
                    ),
                }
            )
    return rows


def run_seed_pair(
    config_path: str,
    output_path: str,
    trials_per_seed: int,
    seed: int,
) -> int:
    output = Path(output_path) / f"seed_{seed}"
    run_experiment_suite(
        Path(config_path),
        output / "optimized",
        trials_per_seed,
        seed,
        disable_optimizer=False,
    )
    run_experiment_suite(
        Path(config_path),
        output / "fixed_time",
        trials_per_seed,
        seed,
        disable_optimizer=True,
    )
    return seed


def controller_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    captures = [row for row in rows if row["captured"]]
    successes = len(captures)
    interval = wilson_interval(successes, len(rows))
    capture_times = [
        float(row["capture_time_seconds"])
        for row in captures
        if row["capture_time_seconds"] is not None
    ]
    return {
        "trials": len(rows),
        "captures": successes,
        "success_rate": round(successes / max(len(rows), 1), 5),
        "success_rate_ci95": [round(interval[0], 5), round(interval[1], 5)],
        "mean_contact_error_m": round(
            mean(float(row["minimum_contact_error_m"]) for row in rows),
            5,
        ),
        "mean_capture_time_seconds": (
            round(mean(capture_times), 5) if capture_times else None
        ),
        "mean_selected_intercept_time_seconds": round(
            mean(float(row["selected_intercept_time_seconds"]) for row in rows),
            5,
        ),
    }


def paired_summary(rows: list[dict[str, Any]]) -> dict[str, int]:
    outcomes: dict[tuple[int, int], dict[str, bool]] = {}
    for row in rows:
        key = (int(row["seed"]), int(row["trial"]))
        outcomes.setdefault(key, {})[str(row["controller"])] = bool(row["captured"])

    result = {"both_capture": 0, "optimized_only": 0, "fixed_only": 0, "both_fail": 0}
    for outcome in outcomes.values():
        optimized = outcome.get("optimized", False)
        fixed = outcome.get("fixed_time", False)
        if optimized and fixed:
            result["both_capture"] += 1
        elif optimized:
            result["optimized_only"] += 1
        elif fixed:
            result["fixed_only"] += 1
        else:
            result["both_fail"] += 1
    return result


def write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "seed",
        "trial",
        "controller",
        "captured",
        "capture_time_seconds",
        "minimum_contact_error_m",
        "selected_intercept_time_seconds",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_report(path: Path, summary: dict[str, Any]) -> None:
    optimized = summary["controllers"]["optimized"]
    fixed = summary["controllers"]["fixed_time"]
    paired = summary["paired_outcomes"]
    lines = [
        "# Multi-Seed Interception Benchmark",
        "",
        f"- Seeds: {', '.join(str(seed) for seed in summary['seeds'])}",
        f"- Trials per seed and controller: {summary['trials_per_seed']}",
        f"- Total simulated controller runs: {summary['total_controller_runs']}",
        "",
        "## Controller Comparison",
        "",
        "| Controller | Captures | Success rate | 95% CI | Mean contact error | Mean capture time |",
        "|---|---:|---:|---:|---:|---:|",
        (
            f"| Optimized | {optimized['captures']}/{optimized['trials']} | "
            f"{optimized['success_rate'] * 100:.2f}% | "
            f"{optimized['success_rate_ci95'][0] * 100:.2f}–"
            f"{optimized['success_rate_ci95'][1] * 100:.2f}% | "
            f"{optimized['mean_contact_error_m']:.5f} m | "
            f"{optimized['mean_capture_time_seconds']:.5f} s |"
        ),
        (
            f"| Fixed time | {fixed['captures']}/{fixed['trials']} | "
            f"{fixed['success_rate'] * 100:.2f}% | "
            f"{fixed['success_rate_ci95'][0] * 100:.2f}–"
            f"{fixed['success_rate_ci95'][1] * 100:.2f}% | "
            f"{fixed['mean_contact_error_m']:.5f} m | "
            f"{fixed['mean_capture_time_seconds']:.5f} s |"
        ),
        "",
        "## Paired Outcomes",
        "",
        f"- Both captured: {paired['both_capture']}",
        f"- Optimized only: {paired['optimized_only']}",
        f"- Fixed only: {paired['fixed_only']}",
        f"- Both failed: {paired['both_fail']}",
        "",
        "The confidence intervals are Wilson score intervals. Each controller saw",
        "the same randomized launches for a paired comparison.",
        "",
    ]
    path.write_text("\n".join(lines))


def run_benchmark(
    config_path: Path,
    output: Path,
    seeds: list[int],
    trials_per_seed: int,
    workers: int,
) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    with ProcessPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = [
            executor.submit(
                run_seed_pair,
                str(config_path),
                str(output),
                trials_per_seed,
                seed,
            )
            for seed in seeds
        ]
        for future in futures:
            future.result()

    rows: list[dict[str, Any]] = []
    for seed in seeds:
        seed_output = output / f"seed_{seed}"
        rows.extend(read_trial_rows(seed_output / "optimized" / "trials.csv", seed, "optimized"))
        rows.extend(read_trial_rows(seed_output / "fixed_time" / "trials.csv", seed, "fixed_time"))

    optimized_rows = [row for row in rows if row["controller"] == "optimized"]
    fixed_rows = [row for row in rows if row["controller"] == "fixed_time"]
    summary = {
        "seeds": seeds,
        "trials_per_seed": trials_per_seed,
        "total_controller_runs": len(rows),
        "controllers": {
            "optimized": controller_summary(optimized_rows),
            "fixed_time": controller_summary(fixed_rows),
        },
        "paired_outcomes": paired_summary(rows),
    }
    write_rows(output / "all_trials.csv", rows)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    write_report(output / "report.md", summary)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a paired multi-seed controller benchmark.")
    parser.add_argument("--config", type=Path, default=Path("configs/intercept_demo.json"))
    parser.add_argument("--output", type=Path, default=Path("outputs/benchmarks/latest"))
    parser.add_argument(
        "--seeds",
        default=(
            "20260611,20260612,20260613,20260614,20260615,"
            "20260616,20260617,20260618,20260619,20260620"
        ),
        help="Comma-separated random seeds.",
    )
    parser.add_argument("--trials-per-seed", type=int, default=100)
    parser.add_argument("--workers", type=int, default=4)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    seeds = [int(value.strip()) for value in args.seeds.split(",") if value.strip()]
    summary = run_benchmark(
        args.config,
        args.output,
        seeds,
        args.trials_per_seed,
        args.workers,
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
