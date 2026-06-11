# Architecture

## System Goal

Build a dual-arm robotic manipulation simulator that looks cinematic but is
driven by real robotics logic: prediction, kinematics, coordination, contact
handling, telemetry, and repeatable evaluation.

## Modules

- `dynamic_dual_arm_sim.run`: local deterministic PyBullet simulation runner.
- `dynamic_dual_arm_sim.experiments`: randomized experiment runner that writes
  aggregate metrics, CSV data, and a Markdown report.
- `configs/intercept_demo.json`: scenario tuning for projectile, arms, camera,
  and capture thresholds.
- `outputs/metrics.json`: repeatable benchmark result for each run.
- `outputs/frames/`: rendered image sequence for showcase editing.
- `outputs/experiments/latest/`: Monte Carlo experiment artifacts.

## Contact Model

The current simulator attaches explicit gripper pad geometry to the KUKA end
effectors. Capture is evaluated using pad-to-object contact target error, not
only wrist distance. For this baseline, pad/projectile collisions are disabled
so the pads act as kinematic measurement fixtures; the next contact-rich phase
will replace this with articulated fingers, force closure checks, and tactile
sensing.

## Near-Term ROS 2 Graph

- `trajectory_predictor`: subscribes to object state and publishes intercept
  poses with confidence.
- `arm_coordinator`: receives intercept poses and publishes synchronized left
  and right arm targets.
- `safety_monitor`: checks joint limits, workspace bounds, and inter-arm
  distances.
- `sim_bridge`: adapts simulator state into ROS 2 messages and applies command
  messages back into the physics scene.
- `telemetry_recorder`: writes bags, CSV metrics, and event markers for video
  editing.

## Research Features To Add

- Model predictive control for intercept timing.
- Learned trajectory correction with noisy visual measurements.
- Domain randomization for object mass, launch velocity, friction, and lighting.
- Contact-rich stabilization once the object is captured.
- Isaac Sim RTX rendering with scripted camera paths and synthetic sensors.

## Quality Bar

Every feature should have a measurable output. Examples:

- capture success rate across randomized launches
- minimum end-effector/object distance
- joint limit violation count
- inter-arm collision count
- time from detection to stable capture
- replayable render frames for the CCA R&D film
