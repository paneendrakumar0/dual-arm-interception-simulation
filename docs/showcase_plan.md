# Showcase Film Plan

## Target Length

Start with a 30-second version. Once the simulation is stable, expand it into a
2-3 minute technical film.

The current repository can already generate the first raw video artifact:

```bash
scripts/run_demo.sh
```

The generated file is `outputs/dual_arm_interception.mp4`.

For the cinematic render preset:

```bash
scripts/run_cinematic.sh
```

The generated file is `outputs/dual_arm_interception_cinematic.mp4`.

Current cinematic features:

- HD frame export
- orbit/drone-style camera motion
- trajectory markers and intercept beacon
- letterboxed research-footage look
- metric HUD with live capture status and contact error

PyBullet is the fast scientific storyboard renderer. The final photoreal
"Hollywood" pass should use Isaac Sim or Blender after the controller, contact
model, and experiment protocol are validated.

## 30-Second Cut

1. Wide shot of the industrial workcell.
2. Mechanical part enters the workspace in slow motion.
3. Overlay trajectory prediction and intercept point.
4. Dual arms accelerate toward synchronized capture.
5. Close-up of stabilization and final pose.
6. End card with metrics: capture time, minimum distance, success rate.

## 2-3 Minute Cut

- 0:00-0:20: problem setup and cinematic workcell reveal
- 0:20-0:50: perception and trajectory prediction visualization
- 0:50-1:30: dual-arm coordination with multiple camera angles
- 1:30-2:10: randomized test montage
- 2:10-2:40: metrics dashboard and ROS 2 architecture
- 2:40-3:00: final polished hero shot for CCA R&D

## Visual Style

Use an industrial R&D lab look: clean lighting, metal surfaces, clear overlays,
and camera movement that shows the real mechanics. The video should feel like a
research demo, not a game trailer.
