from __future__ import annotations

import argparse
import json
import math
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pybullet as p
import pybullet_data


@dataclass(frozen=True)
class ProjectileConfig:
    start_position: np.ndarray
    start_velocity: np.ndarray
    radius: float
    mass: float


@dataclass(frozen=True)
class ArmConfig:
    left_base_position: np.ndarray
    right_base_position: np.ndarray
    max_joint_velocity: float
    position_gain: float
    velocity_gain: float


@dataclass(frozen=True)
class InterceptConfig:
    target_time_seconds: float
    capture_distance_m: float
    stabilize_height_m: float


@dataclass(frozen=True)
class CameraConfig:
    width: int
    height: int
    frame_stride: int


@dataclass(frozen=True)
class SimConfig:
    time_step: float
    duration_seconds: float
    gravity: np.ndarray
    projectile: ProjectileConfig
    arms: ArmConfig
    intercept: InterceptConfig
    camera: CameraConfig


def _array(values: list[float]) -> np.ndarray:
    return np.array(values, dtype=float)


def load_config(path: Path) -> SimConfig:
    raw = json.loads(path.read_text())
    projectile = raw["projectile"]
    arms = raw["arms"]
    intercept = raw["intercept"]
    camera = raw["camera"]
    return SimConfig(
        time_step=float(raw["time_step"]),
        duration_seconds=float(raw["duration_seconds"]),
        gravity=_array(raw["gravity"]),
        projectile=ProjectileConfig(
            start_position=_array(projectile["start_position"]),
            start_velocity=_array(projectile["start_velocity"]),
            radius=float(projectile["radius"]),
            mass=float(projectile["mass"]),
        ),
        arms=ArmConfig(
            left_base_position=_array(arms["left_base_position"]),
            right_base_position=_array(arms["right_base_position"]),
            max_joint_velocity=float(arms["max_joint_velocity"]),
            position_gain=float(arms["position_gain"]),
            velocity_gain=float(arms["velocity_gain"]),
        ),
        intercept=InterceptConfig(
            target_time_seconds=float(intercept["target_time_seconds"]),
            capture_distance_m=float(intercept["capture_distance_m"]),
            stabilize_height_m=float(intercept["stabilize_height_m"]),
        ),
        camera=CameraConfig(
            width=int(camera["width"]),
            height=int(camera["height"]),
            frame_stride=int(camera["frame_stride"]),
        ),
    )


def predicted_position(config: SimConfig, t: float) -> np.ndarray:
    return (
        config.projectile.start_position
        + config.projectile.start_velocity * t
        + 0.5 * config.gravity * t * t
    )


def connect(render_gui: bool) -> int:
    mode = p.GUI if render_gui else p.DIRECT
    client = p.connect(mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    return client


def create_projectile(config: SimConfig) -> int:
    visual = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=config.projectile.radius,
        rgbaColor=[0.95, 0.58, 0.08, 1.0],
        specularColor=[0.9, 0.8, 0.55],
    )
    collision = p.createCollisionShape(p.GEOM_SPHERE, radius=config.projectile.radius)
    body = p.createMultiBody(
        baseMass=config.projectile.mass,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=config.projectile.start_position.tolist(),
    )
    p.resetBaseVelocity(body, linearVelocity=config.projectile.start_velocity.tolist())
    return body


def create_arm(base_position: np.ndarray, base_yaw: float) -> int:
    orientation = p.getQuaternionFromEuler([0.0, 0.0, base_yaw])
    arm = p.loadURDF(
        "kuka_iiwa/model.urdf",
        basePosition=base_position.tolist(),
        baseOrientation=orientation,
        useFixedBase=True,
    )
    for joint in range(p.getNumJoints(arm)):
        p.changeDynamics(arm, joint, linearDamping=0.05, angularDamping=0.05)
    return arm


def active_joint_indices(body: int) -> list[int]:
    joints: list[int] = []
    for joint in range(p.getNumJoints(body)):
        joint_type = p.getJointInfo(body, joint)[2]
        if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            joints.append(joint)
    return joints


def move_arm_to_target(body: int, target: np.ndarray, config: SimConfig) -> None:
    joints = active_joint_indices(body)
    end_effector_index = joints[-1]
    solution = p.calculateInverseKinematics(
        body,
        end_effector_index,
        target.tolist(),
        maxNumIterations=80,
        residualThreshold=1e-4,
    )
    p.setJointMotorControlArray(
        body,
        joints,
        p.POSITION_CONTROL,
        targetPositions=list(solution[: len(joints)]),
        positionGains=[config.arms.position_gain] * len(joints),
        velocityGains=[config.arms.velocity_gain] * len(joints),
        forces=[220.0] * len(joints),
        targetVelocities=[0.0] * len(joints),
    )


def end_effector_position(body: int) -> np.ndarray:
    joints = active_joint_indices(body)
    return np.array(p.getLinkState(body, joints[-1], computeForwardKinematics=True)[0])


def create_workcell() -> None:
    p.loadURDF("plane.urdf")
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.setPhysicsEngineParameter(numSolverIterations=90, contactBreakingThreshold=0.001)
    p.changeDynamics(0, -1, lateralFriction=0.8, spinningFriction=0.03)
    table_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.55, 1.15, 0.04])
    table_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[1.55, 1.15, 0.04],
        rgbaColor=[0.14, 0.15, 0.16, 1.0],
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=table_collision,
        baseVisualShapeIndex=table_visual,
        basePosition=[0.0, 0.15, 0.62],
    )


def camera_image(config: CameraConfig) -> np.ndarray:
    view = p.computeViewMatrix(
        cameraEyePosition=[2.1, -2.2, 1.55],
        cameraTargetPosition=[0.0, 0.15, 1.0],
        cameraUpVector=[0.0, 0.0, 1.0],
    )
    projection = p.computeProjectionMatrixFOV(
        fov=50.0,
        aspect=config.width / config.height,
        nearVal=0.02,
        farVal=8.0,
    )
    _, _, rgba, _, _ = p.getCameraImage(
        width=config.width,
        height=config.height,
        viewMatrix=view,
        projectionMatrix=projection,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )
    return np.reshape(rgba, (config.height, config.width, 4))[:, :, :3]


def prepare_output_dir(output: Path, clean: bool) -> Path:
    if clean and output.exists():
        shutil.rmtree(output)
    frame_dir = output / "frames"
    frame_dir.mkdir(parents=True, exist_ok=True)
    return frame_dir


def write_frame(path: Path, image: np.ndarray) -> None:
    try:
        import imageio.v2 as imageio

        imageio.imwrite(path, image)
        return
    except ModuleNotFoundError:
        pass

    from PIL import Image

    Image.fromarray(image.astype(np.uint8), mode="RGB").save(path)


def run_sim(config: SimConfig, output: Path, render: bool, gui: bool, clean: bool) -> dict[str, Any]:
    frame_dir = prepare_output_dir(output, clean)
    connect(gui)
    p.resetSimulation()
    p.setTimeStep(config.time_step)
    p.setGravity(*config.gravity.tolist())
    create_workcell()
    projectile = create_projectile(config)
    left_arm = create_arm(config.arms.left_base_position, -math.pi / 2.0)
    right_arm = create_arm(config.arms.right_base_position, math.pi / 2.0)

    steps = int(config.duration_seconds / config.time_step)
    intercept = predicted_position(config, config.intercept.target_time_seconds)
    intercept[2] = max(intercept[2], config.intercept.stabilize_height_m)
    left_target = intercept + np.array([-0.16, 0.0, 0.0])
    right_target = intercept + np.array([0.16, 0.0, 0.0])

    captured = False
    capture_step: int | None = None
    min_left_distance = float("inf")
    min_right_distance = float("inf")
    min_dual_distance = float("inf")
    min_contact_error = float("inf")
    frame_paths: list[str] = []

    for step in range(steps):
        sim_time = step * config.time_step
        object_position = np.array(p.getBasePositionAndOrientation(projectile)[0])
        time_to_intercept = max(config.intercept.target_time_seconds - sim_time, 0.0)
        predicted = object_position + np.array(p.getBaseVelocity(projectile)[0]) * time_to_intercept
        predicted[2] = max(predicted[2], config.intercept.stabilize_height_m)

        blend = min(step / max(steps * 0.25, 1), 1.0)
        move_arm_to_target(left_arm, (1.0 - blend) * left_target + blend * (predicted + [-0.16, 0.0, 0.0]), config)
        move_arm_to_target(right_arm, (1.0 - blend) * right_target + blend * (predicted + [0.16, 0.0, 0.0]), config)

        left_effector = end_effector_position(left_arm)
        right_effector = end_effector_position(right_arm)
        left_contact = object_position + np.array([-0.16, 0.0, 0.0])
        right_contact = object_position + np.array([0.16, 0.0, 0.0])
        left_distance = float(np.linalg.norm(left_effector - object_position))
        right_distance = float(np.linalg.norm(right_effector - object_position))
        contact_error = max(
            float(np.linalg.norm(left_effector - left_contact)),
            float(np.linalg.norm(right_effector - right_contact)),
        )
        dual_distance = max(left_distance, right_distance)
        min_left_distance = min(min_left_distance, left_distance)
        min_right_distance = min(min_right_distance, right_distance)
        min_dual_distance = min(min_dual_distance, dual_distance)
        min_contact_error = min(min_contact_error, contact_error)

        if not captured and contact_error <= config.intercept.capture_distance_m:
            captured = True
            capture_step = step
            p.resetBaseVelocity(projectile, linearVelocity=[0.0, 0.0, 0.0], angularVelocity=[0.0, 0.0, 0.0])

        if captured:
            hold_position = 0.5 * (end_effector_position(left_arm) + end_effector_position(right_arm))
            p.resetBasePositionAndOrientation(projectile, hold_position.tolist(), [0.0, 0.0, 0.0, 1.0])

        p.stepSimulation()

        if render and step % config.camera.frame_stride == 0:
            image = camera_image(config.camera)
            frame_path = frame_dir / f"frame_{len(frame_paths):04d}.png"
            write_frame(frame_path, image)
            frame_paths.append(str(frame_path))

    metrics = {
        "captured": captured,
        "capture_time_seconds": None if capture_step is None else round(capture_step * config.time_step, 4),
        "minimum_left_distance_m": round(min_left_distance, 4),
        "minimum_right_distance_m": round(min_right_distance, 4),
        "minimum_dual_capture_distance_m": round(min_dual_distance, 4),
        "minimum_contact_error_m": round(min_contact_error, 4),
        "intercept_position_m": [round(float(v), 4) for v in intercept],
        "frames_written": len(frame_paths),
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    p.disconnect()
    return metrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the dynamic dual-arm interception simulation.")
    parser.add_argument("--config", type=Path, default=Path("configs/intercept_demo.json"))
    parser.add_argument("--output", type=Path, default=Path("outputs"))
    parser.add_argument("--render", action="store_true", help="Write camera frames to the output folder.")
    parser.add_argument("--gui", action="store_true", help="Open the PyBullet GUI for interactive viewing.")
    parser.add_argument("--keep-output", action="store_true", help="Do not clean the output folder first.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    metrics = run_sim(
        config=load_config(args.config),
        output=args.output,
        render=args.render,
        gui=args.gui,
        clean=not args.keep_output,
    )
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
