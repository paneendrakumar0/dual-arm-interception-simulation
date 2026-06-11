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
class GripperConfig:
    half_gap_m: float
    pad_half_extents_m: np.ndarray
    pad_mass_kg: float
    lateral_friction: float
    contact_stiffness: float
    contact_damping: float


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
    gripper: GripperConfig
    camera: CameraConfig


def _array(values: list[float]) -> np.ndarray:
    return np.array(values, dtype=float)


def load_config(path: Path) -> SimConfig:
    raw = json.loads(path.read_text())
    projectile = raw["projectile"]
    arms = raw["arms"]
    intercept = raw["intercept"]
    gripper = raw["gripper"]
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
        gripper=GripperConfig(
            half_gap_m=float(gripper["half_gap_m"]),
            pad_half_extents_m=_array(gripper["pad_half_extents_m"]),
            pad_mass_kg=float(gripper["pad_mass_kg"]),
            lateral_friction=float(gripper["lateral_friction"]),
            contact_stiffness=float(gripper["contact_stiffness"]),
            contact_damping=float(gripper["contact_damping"]),
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


def end_effector_link_index(body: int) -> int:
    return active_joint_indices(body)[-1]


def active_joint_indices(body: int) -> list[int]:
    joints: list[int] = []
    for joint in range(p.getNumJoints(body)):
        joint_type = p.getJointInfo(body, joint)[2]
        if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            joints.append(joint)
    return joints


def move_arm_to_target(body: int, target: np.ndarray, config: SimConfig) -> None:
    joints = active_joint_indices(body)
    solution = p.calculateInverseKinematics(
        body,
        end_effector_link_index(body),
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
    return np.array(p.getLinkState(body, end_effector_link_index(body), computeForwardKinematics=True)[0])


def create_gripper_pad(arm: int, config: SimConfig, color: list[float]) -> int:
    link_index = end_effector_link_index(arm)
    link_state = p.getLinkState(arm, link_index, computeForwardKinematics=True)
    pad_half_extents = config.gripper.pad_half_extents_m.tolist()
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=pad_half_extents,
        rgbaColor=color,
        specularColor=[0.7, 0.7, 0.7],
    )
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=pad_half_extents)
    pad = p.createMultiBody(
        baseMass=config.gripper.pad_mass_kg,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=link_state[0],
        baseOrientation=link_state[1],
    )
    p.changeDynamics(
        pad,
        -1,
        lateralFriction=config.gripper.lateral_friction,
        spinningFriction=0.08,
        rollingFriction=0.03,
        contactStiffness=config.gripper.contact_stiffness,
        contactDamping=config.gripper.contact_damping,
    )
    for joint in range(-1, p.getNumJoints(arm)):
        p.setCollisionFilterPair(arm, pad, joint, -1, enableCollision=0)
    return pad


def sync_gripper_pad(arm: int, pad: int) -> None:
    link_state = p.getLinkState(arm, end_effector_link_index(arm), computeForwardKinematics=True)
    p.resetBasePositionAndOrientation(pad, link_state[0], link_state[1])


def body_position(body: int) -> np.ndarray:
    return np.array(p.getBasePositionAndOrientation(body)[0])


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
    left_pad = create_gripper_pad(left_arm, config, [0.08, 0.52, 0.95, 1.0])
    right_pad = create_gripper_pad(right_arm, config, [0.95, 0.42, 0.08, 1.0])
    p.setCollisionFilterPair(left_pad, projectile, -1, -1, enableCollision=0)
    p.setCollisionFilterPair(right_pad, projectile, -1, -1, enableCollision=0)
    sync_gripper_pad(left_arm, left_pad)
    sync_gripper_pad(right_arm, right_pad)

    steps = int(config.duration_seconds / config.time_step)
    intercept = predicted_position(config, config.intercept.target_time_seconds)
    intercept[2] = max(intercept[2], config.intercept.stabilize_height_m)
    left_grip_offset = np.array([-config.gripper.half_gap_m, 0.0, 0.0])
    right_grip_offset = np.array([config.gripper.half_gap_m, 0.0, 0.0])
    left_target = intercept + left_grip_offset
    right_target = intercept + right_grip_offset

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
        move_arm_to_target(left_arm, (1.0 - blend) * left_target + blend * (predicted + left_grip_offset), config)
        move_arm_to_target(right_arm, (1.0 - blend) * right_target + blend * (predicted + right_grip_offset), config)

        sync_gripper_pad(left_arm, left_pad)
        sync_gripper_pad(right_arm, right_pad)

        left_effector = end_effector_position(left_arm)
        right_effector = end_effector_position(right_arm)
        left_pad_position = body_position(left_pad)
        right_pad_position = body_position(right_pad)
        left_contact = object_position + left_grip_offset
        right_contact = object_position + right_grip_offset
        left_distance = float(np.linalg.norm(left_effector - object_position))
        right_distance = float(np.linalg.norm(right_effector - object_position))
        contact_error = max(
            float(np.linalg.norm(left_pad_position - left_contact)),
            float(np.linalg.norm(right_pad_position - right_contact)),
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
            hold_position = 0.5 * (body_position(left_pad) + body_position(right_pad))
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
        "capture_threshold_m": config.intercept.capture_distance_m,
        "gripper_half_gap_m": config.gripper.half_gap_m,
        "pad_half_extents_m": [round(float(v), 4) for v in config.gripper.pad_half_extents_m],
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
