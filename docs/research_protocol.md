# Research Protocol

## Principle

A single cinematic run is a demo. A research simulation needs repeatable
experiments, measurable hypotheses, and failure analysis.

## Current Baseline

Run the deterministic showcase:

```bash
scripts/run_demo.sh
```

Run randomized Monte Carlo experiments:

```bash
scripts/run_experiments.sh
```

Outputs are written to `outputs/experiments/latest/`:

- `summary.json`: aggregate metrics
- `trials.csv`: per-trial data
- `report.md`: lab-style report with interpretation and failure cases

## Metrics

- Capture success rate
- Capture time
- Minimum contact error
- Minimum dual-arm distance to target
- Failure case launch parameters

## Research Roadmap

1. Add noisy observation streams and a trajectory estimator.
2. Introduce model predictive control for interception timing.
3. Replace the two-pad fixture with articulated grippers and tactile sensing.
4. Add collision constraints and inter-arm safety checks.
5. Port the validated controller into ROS 2.
6. Move rendering and sensor generation into Isaac Sim.
7. Produce a formal report with ablations, plots, and reproducibility notes.
