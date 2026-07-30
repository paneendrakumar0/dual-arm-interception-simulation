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

## Online Intercept-Time Optimizer

The first optimizer iteration adds gravity-consistent online prediction,
sampled intercept-time selection, absolute-deadline locking, and a
calibrated-speed confidence fallback.

Using the same 24 trials and seed:

- Corrected fixed-time ablation: 21 captures (87.50%)
- Confidence-gated online optimizer: 22 captures (91.67%)
- Optimizer mean contact error: 0.05268 m
- Optimizer mean capture time: 0.58012 s
- Mean selected intercept time: 0.50521 s

This small seeded benchmark is evidence for continuing the optimizer work, not
a general robustness claim. The next experiment should use more seeds and
report confidence intervals before expanding the calibrated speed range.

## 1,000-Trial Multi-Seed Robustness Check

The follow-up benchmark used ten seeds, 100 matched launches per seed, and both
controllers for 2,000 total controller runs.

- Online optimizer: 751/1,000 captures (75.10%, Wilson 95% CI 72.33%-77.68%)
- Fixed-time controller: 783/1,000 captures (78.30%, Wilson 95% CI 75.64%-80.74%)
- Both captured: 745 paired launches
- Optimizer only: 6 paired launches
- Fixed time only: 38 paired launches
- Both failed: 211 paired launches

The single-seed optimizer improvement did not generalize. On the larger paired
benchmark, the current optimizer captured 32 fewer launches and produced a
higher mean contact error (0.07137 m versus 0.06627 m). The optimizer should
remain experimental while its reachability score and confidence gate are
revised against this failure set.
