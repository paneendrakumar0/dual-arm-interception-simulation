# Dynamic Dual-Arm Robotic Interception Simulation

This repository is the starting point for an industrial-grade robotics showcase:
two industrial robot arms track, intercept, and stabilize a moving component in
simulation. The immediate version runs locally with Python and PyBullet. The
research roadmap upgrades the same architecture into ROS 2 + NVIDIA Isaac Sim
for photorealistic rendering and high-fidelity synthetic data.

## What We Are Simulating

The target project is a cinematic robotics R&D demo:

- A mechanical component flies through a manufacturing workcell.
- A dual-arm robot predicts the component trajectory in real time.
- Both arms coordinate to intercept and stabilize the part with explicit
  simulated gripper pads and contact-error metrics. Later dexterous-hand models
  will replace the two-pad fixture with articulated fingers and tactile sensing.
- The system records metrics, rendered frames, and a short showcase video path.

This is a good internship portfolio direction because it combines robot
kinematics, physics simulation, trajectory prediction, multi-agent coordination,
controls, and cinematic visualization.

## Current Stack

- Python 3.10
- PyBullet physics simulator
- NumPy for state and trajectory math
- ROS 2 Humble available on this machine for the next integration phase
- RTX 4060 Laptop GPU available for future Isaac Sim rendering

Isaac Sim is not installed in the current environment yet, so this repo starts
with a portable PyBullet simulation that can later become the control and
validation layer for Isaac Sim.

## Quick Start

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m dynamic_dual_arm_sim.run --config configs/intercept_demo.json --render
```

Outputs are written to `outputs/`:

- `metrics.json` with capture result, distances, and timing
- `frames/` with camera renders when `--render` is enabled
- `dual_arm_interception.mp4` when the video script is used

If you already have the dependencies globally installed, you can run without a
virtual environment:

```bash
PYTHONPATH=src python3 -m dynamic_dual_arm_sim.run --config configs/intercept_demo.json
```

## Create The First Showcase Clip

Run the full local demo pipeline:

```bash
scripts/run_demo.sh
```

This renders the simulation frames and encodes them into:

```text
outputs/dual_arm_interception.mp4
```

For the cinematic 1080p render preset with orbit camera, trajectory markers,
letterboxing, and research HUD overlays:

```bash
scripts/run_cinematic.sh
```

This writes:

```text
outputs/dual_arm_interception_cinematic.mp4
```

## Run Scientific Experiments

Run randomized launch trials and generate a lab-style report:

```bash
scripts/run_experiments.sh
```

The experiment suite writes:

- `outputs/experiments/latest/summary.json`
- `outputs/experiments/latest/trials.csv`
- `outputs/experiments/latest/report.md`

## Development Direction

Phase 1 is the local deterministic simulator. Phase 2 adds ROS 2 nodes for
trajectory prediction, command streaming, telemetry, and replay. Phase 3 ports
the scene into Isaac Sim for photorealistic capture, synthetic sensors, tactile
contact modeling, and a 30-180 second CCA R&D showcase film.

See [docs/architecture.md](docs/architecture.md) and
[docs/showcase_plan.md](docs/showcase_plan.md).

For the research workflow and target hardware profile, see
[docs/research_protocol.md](docs/research_protocol.md) and
[docs/hardware_profile.md](docs/hardware_profile.md). Baseline experiment
results are tracked in [docs/baseline_results.md](docs/baseline_results.md).
