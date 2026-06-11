# Baseline Results

## Explicit Gripper Pad Baseline

Configuration:

- Simulation: PyBullet dual KUKA workcell
- Contact model: kinematic gripper pad geometry
- Capture metric: pad-to-object contact target error
- Capture threshold: 0.045 m
- Gripper half-gap: 0.16 m
- Random seed: 20260611

Deterministic showcase run:

- Captured: true
- Capture time: 0.5167 s
- Minimum contact error: 0.0002 m
- Minimum dual capture distance: 0.147 m
- Rendered frames: 100

Monte Carlo baseline:

- Trials: 24
- Captures: 20
- Success rate: 83.33%
- Mean contact error: 0.05784 m
- Contact error standard deviation: 0.15956 m
- Mean capture time: 0.53146 s
- Best contact error: 0.0001 m
- Worst contact error: 0.6231 m

Failure cases from this baseline are valuable. They show that the controller is
not yet robust to the full randomized launch distribution, especially low
vertical-velocity or high-forward-velocity cases. The next research step is to
replace the fixed intercept timing with an online intercept-time optimizer.
