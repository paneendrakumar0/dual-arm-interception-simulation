# Hardware Profile

The current development target is Paneendra's ASUS ROG Strix G16. The local
hardware report and system probes show:

- Model: ROG Strix G16 G614JVR
- CPU platform: 64-bit notebook platform
- Memory: 32 GiB DDR5 SODIMM
- GPU: NVIDIA GeForce RTX 4060 Laptop GPU
- GPU memory: 8188 MiB
- Driver: 580.126.09
- ROS 2: Humble is installed locally

## Simulation Implications

This machine is strong enough for:

- PyBullet Monte Carlo sweeps for controller development
- ROS 2 node orchestration and telemetry recording
- Medium-scale synthetic data generation
- Isaac Sim development after installation and tuning
- Short cinematic renders at 1080p-class resolution

It is not an unlimited cluster. Research quality will come from disciplined
experiment design, metrics, validation, and staged fidelity increases rather
than trying to run every expensive feature at once.
