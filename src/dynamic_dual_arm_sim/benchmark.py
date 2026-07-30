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
from dynamic_dual_arm_sim.run import load_config


def parse_vector(value: str) -> list[float]:
    parsed = json.loads(value)
    return [float(item) for item in parsed]


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
            start_position = parse_vector(raw["start_position_m"])
            start_velocity = parse_vector(raw["start_velocity_mps"])
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
                    "start_position_x_m": start_position[0],
                    "start_position_y_m": start_position[1],
                    "start_position_z_m": start_position[2],
                    "start_velocity_x_mps": start_velocity[0],
                    "start_velocity_y_mps": start_velocity[1],
                    "start_velocity_z_mps": start_velocity[2],
                    "transverse_speed_mps": math.hypot(
                        start_velocity[0], start_velocity[1]
                    ),
                    "projectile_mass_kg": float(raw["projectile_mass_kg"]),
                    "projectile_radius_m": float(raw["projectile_radius_m"]),
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


def exact_mcnemar_p_value(optimized_only: int, fixed_only: int) -> float:
    discordant = optimized_only + fixed_only
    if discordant == 0:
        return 1.0
    smaller = min(optimized_only, fixed_only)
    tail = sum(math.comb(discordant, index) for index in range(smaller + 1))
    return min(1.0, 2.0 * tail / (2**discordant))


def paired_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
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
    result["exact_mcnemar_p_value"] = round(
        exact_mcnemar_p_value(result["optimized_only"], result["fixed_only"]),
        8,
    )
    return result


def failure_analysis(rows: list[dict[str, Any]], speed_limit: float) -> dict[str, Any]:
    category_checks = {
        "outside_optimizer_speed_range": (
            lambda row: float(row["transverse_speed_mps"]) > speed_limit
        ),
        "low_vertical_velocity": (
            lambda row: float(row["start_velocity_z_mps"]) < 3.27
        ),
        "high_forward_velocity": (
            lambda row: float(row["start_velocity_y_mps"]) > 0.94
        ),
        "large_lateral_velocity_error": (
            lambda row: abs(float(row["start_velocity_x_mps"]) - 0.48) > 0.16
        ),
    }
    categories: dict[str, dict[str, int]] = {}
    for controller in ("optimized", "fixed_time"):
        failures = [
            row
            for row in rows
            if row["controller"] == controller and not row["captured"]
        ]
        categories[controller] = {
            name: sum(1 for row in failures if check(row))
            for name, check in category_checks.items()
        }

    speed_bins = [
        ("0.0-0.6", 0.0, 0.6),
        ("0.6-0.8", 0.6, 0.8),
        ("0.8-1.0", 0.8, 1.0),
        ("1.0+", 1.0, float("inf")),
    ]
    success_by_speed: dict[str, dict[str, dict[str, Any]]] = {}
    for controller in ("optimized", "fixed_time"):
        success_by_speed[controller] = {}
        controller_rows = [row for row in rows if row["controller"] == controller]
        for label, lower, upper in speed_bins:
            matching = [
                row
                for row in controller_rows
                if lower < float(row["transverse_speed_mps"]) <= upper
            ]
            captures = sum(1 for row in matching if row["captured"])
            interval = wilson_interval(captures, len(matching))
            success_by_speed[controller][label] = {
                "trials": len(matching),
                "captures": captures,
                "success_rate": round(captures / max(len(matching), 1), 5),
                "success_rate_ci95": [
                    round(interval[0], 5),
                    round(interval[1], 5),
                ],
            }
    return {
        "categories_are_overlapping": True,
        "failure_category_counts": categories,
        "success_by_transverse_speed_bin": success_by_speed,
    }


def write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "seed",
        "trial",
        "controller",
        "captured",
        "capture_time_seconds",
        "minimum_contact_error_m",
        "selected_intercept_time_seconds",
        "start_position_x_m",
        "start_position_y_m",
        "start_position_z_m",
        "start_velocity_x_mps",
        "start_velocity_y_mps",
        "start_velocity_z_mps",
        "transverse_speed_mps",
        "projectile_mass_kg",
        "projectile_radius_m",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_plots(output: Path, rows: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_dir = output / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    colors = {"optimized": "#1f9e89", "fixed_time": "#d97706"}
    labels = {"optimized": "Optimized", "fixed_time": "Fixed time"}

    controllers = ["optimized", "fixed_time"]
    rates = [summary["controllers"][name]["success_rate"] for name in controllers]
    intervals = [
        summary["controllers"][name]["success_rate_ci95"] for name in controllers
    ]
    errors = [
        [rate - interval[0] for rate, interval in zip(rates, intervals)],
        [interval[1] - rate for rate, interval in zip(rates, intervals)],
    ]
    fig, axis = plt.subplots(figsize=(7.5, 4.8))
    axis.bar(
        [labels[name] for name in controllers],
        rates,
        yerr=errors,
        capsize=7,
        color=[colors[name] for name in controllers],
    )
    axis.set_ylim(0.0, 1.0)
    axis.set_ylabel("Capture success rate")
    axis.set_title("Controller Success Rate with 95% Wilson Intervals")
    axis.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(plot_dir / "success_rate.png", dpi=180)
    plt.close(fig)

    contact_data = [
        [
            float(row["minimum_contact_error_m"])
            for row in rows
            if row["controller"] == controller
        ]
        for controller in controllers
    ]
    fig, axis = plt.subplots(figsize=(7.5, 4.8))
    boxes = axis.boxplot(
        contact_data,
        tick_labels=[labels[name] for name in controllers],
        showfliers=False,
        patch_artist=True,
    )
    for patch, controller in zip(boxes["boxes"], controllers):
        patch.set_facecolor(colors[controller])
        patch.set_alpha(0.75)
    axis.set_ylabel("Minimum contact error (m)")
    axis.set_title("Contact Error Distribution")
    axis.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(plot_dir / "contact_error.png", dpi=180)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(7.5, 4.8))
    for controller in controllers:
        capture_times = [
            float(row["capture_time_seconds"])
            for row in rows
            if row["controller"] == controller
            and row["capture_time_seconds"] is not None
        ]
        axis.hist(
            capture_times,
            bins=24,
            alpha=0.55,
            label=labels[controller],
            color=colors[controller],
        )
    axis.set_xlabel("Capture time (s)")
    axis.set_ylabel("Captured trials")
    axis.set_title("Capture-Time Distribution")
    axis.legend()
    axis.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(plot_dir / "capture_time.png", dpi=180)
    plt.close(fig)

    category_counts = summary["failure_analysis"]["failure_category_counts"]
    category_names = list(category_counts["optimized"])
    positions = list(range(len(category_names)))
    width = 0.38
    fig, axis = plt.subplots(figsize=(9.5, 5.2))
    axis.bar(
        [position - width / 2 for position in positions],
        [category_counts["optimized"][name] for name in category_names],
        width,
        label="Optimized",
        color=colors["optimized"],
    )
    axis.bar(
        [position + width / 2 for position in positions],
        [category_counts["fixed_time"][name] for name in category_names],
        width,
        label="Fixed time",
        color=colors["fixed_time"],
    )
    axis.set_xticks(positions, [name.replace("_", "\n") for name in category_names])
    axis.set_ylabel("Failed trials (overlapping categories)")
    axis.set_title("Failure Categories")
    axis.legend()
    axis.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(plot_dir / "failure_categories.png", dpi=180)
    plt.close(fig)


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
        f"- Exact McNemar p-value: {paired['exact_mcnemar_p_value']}",
        "",
        "## Failure Analysis",
        "",
        "Failure categories overlap; one launch can appear in several categories.",
        "",
        "| Category | Optimized failures | Fixed-time failures |",
        "|---|---:|---:|",
    ]
    categories = summary["failure_analysis"]["failure_category_counts"]
    for name in categories["optimized"]:
        lines.append(
            f"| {name.replace('_', ' ')} | {categories['optimized'][name]} | "
            f"{categories['fixed_time'][name]} |"
        )
    speed_bins = summary["failure_analysis"]["success_by_transverse_speed_bin"]
    lines.extend(
        [
            "",
            "## Success by Transverse Speed",
            "",
            "| Speed (m/s) | Optimized | Fixed time |",
            "|---|---:|---:|",
        ]
    )
    for speed_bin in speed_bins["optimized"]:
        optimized_bin = speed_bins["optimized"][speed_bin]
        fixed_bin = speed_bins["fixed_time"][speed_bin]
        lines.append(
            f"| {speed_bin} | {optimized_bin['captures']}/{optimized_bin['trials']} "
            f"({optimized_bin['success_rate'] * 100:.2f}%) | "
            f"{fixed_bin['captures']}/{fixed_bin['trials']} "
            f"({fixed_bin['success_rate'] * 100:.2f}%) |"
        )
    lines.extend(
        [
            "",
            "## Plots",
            "",
            "- [Success rate](plots/success_rate.png)",
            "- [Contact error](plots/contact_error.png)",
            "- [Capture time](plots/capture_time.png)",
            "- [Failure categories](plots/failure_categories.png)",
            "",
            "The confidence intervals are Wilson score intervals. Each controller saw",
            "the same randomized launches for a paired comparison.",
            "",
        ]
    )
    path.write_text("\n".join(lines))


def run_benchmark(
    config_path: Path,
    output: Path,
    seeds: list[int],
    trials_per_seed: int,
    workers: int,
    reuse_results: bool = False,
) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    if not reuse_results:
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
    speed_limit = load_config(config_path).intercept.optimizer_speed_limit_mps
    summary = {
        "seeds": seeds,
        "trials_per_seed": trials_per_seed,
        "total_controller_runs": len(rows),
        "controllers": {
            "optimized": controller_summary(optimized_rows),
            "fixed_time": controller_summary(fixed_rows),
        },
        "paired_outcomes": paired_summary(rows),
        "failure_analysis": failure_analysis(rows, speed_limit),
    }
    write_rows(output / "all_trials.csv", rows)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    write_plots(output, rows, summary)
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
    parser.add_argument(
        "--reuse-results",
        action="store_true",
        help="Rebuild summaries and plots from existing per-seed trial CSV files.",
    )
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
        reuse_results=args.reuse_results,
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
