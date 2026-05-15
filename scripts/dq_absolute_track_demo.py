#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


DQ_VIZLAB_ROOT = Path(os.environ.get("DQ_VIZLAB_ROOT", "/home/syx/dq_vizlab/dq_vizlab")).expanduser()
SDK_PYTHON = DQ_VIZLAB_ROOT / "python_sdk" / "python"
if SDK_PYTHON.exists():
    sys.path.insert(0, str(SDK_PYTHON))

from sailor_sdk import (  # noqa: E402
    DEFAULT_INITIAL_Q,
    HUMANOID_JOINT_NAMES,
    NullspaceSolverClient,
    dq_pose_to_quaternion_translation,
    interpolate_absolute_pose_waypoints,
    pose_dq_from_quaternion_translation,
)


DEFAULT_SOLVER_BINARY = DQ_VIZLAB_ROOT / "python_sdk" / "build" / "sailor_nullspace_solver"
DEFAULT_SOLVER_URDF = (
    DQ_VIZLAB_ROOT
    / "public"
    / "robots"
    / "sailor_r1_pro_description"
    / "urdf"
    / "sailor_r1_pro_description.urdf"
)


def _topic_prefix(namespace: str) -> str:
    ns = namespace.strip("/")
    return f"/{ns}" if ns else ""


def _parse_dq(text: str) -> list[float]:
    values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if len(values) != 8:
        raise argparse.ArgumentTypeError("--target-dq expects 8 comma-separated values")
    return values


def _require_all_or_none(values: Sequence[float | None], label: str) -> bool:
    present = [value is not None for value in values]
    if any(present) and not all(present):
        raise ValueError(f"{label} must provide all values or none")
    return all(present)


def _quaternion_distance(a: Sequence[float], b: Sequence[float]) -> float:
    dot = sum(float(a[index]) * float(b[index]) for index in range(4))
    dot = max(-1.0, min(1.0, abs(dot)))
    return 2.0 * math.acos(dot)


def _pose_error(target_pose: Sequence[float], actual_pose: Sequence[float]) -> tuple[float, float]:
    target_quat, target_xyz = dq_pose_to_quaternion_translation(target_pose)
    actual_quat, actual_xyz = dq_pose_to_quaternion_translation(actual_pose)
    translation_error = math.sqrt(sum((target_xyz[i] - actual_xyz[i]) ** 2 for i in range(3)))
    rotation_error = _quaternion_distance(target_quat, actual_quat)
    return translation_error, rotation_error


class DQAbsoluteTrackDemo(Node):
    def __init__(self, *, namespace: str) -> None:
        super().__init__("qpin_dq_absolute_track_demo")
        prefix = _topic_prefix(namespace)
        self.joint_state_topic = f"{prefix}/joint_states"
        self.joint_command_topic = f"{prefix}/joint_command"
        self.current_q = [float(value) for value in DEFAULT_INITIAL_Q]
        self.has_joint_state = False
        self.joint_index = {name: index for index, name in enumerate(HUMANOID_JOINT_NAMES)}
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            10,
        )
        self.joint_command_pub = self.create_publisher(JointState, self.joint_command_topic, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        names = list(msg.name or [])
        positions = list(msg.position or [])
        recognized = False
        for name, value in zip(names, positions):
            q_index = self.joint_index.get(str(name))
            if q_index is None:
                continue
            self.current_q[q_index] = float(value)
            recognized = True
        if recognized:
            self.has_joint_state = True

    def wait_for_joint_state(self, *, timeout_sec: float, require: bool) -> tuple[list[float], str]:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            if self.has_joint_state:
                return self.current_q[:], self.joint_state_topic
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if require:
            raise TimeoutError(f"no JointState received on {self.joint_state_topic} within {timeout_sec:.2f}s")
        return self.current_q[:], "DEFAULT_INITIAL_Q"

    def publish_joint_command(self, q: Sequence[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(HUMANOID_JOINT_NAMES)
        msg.position = [float(value) for value in q]
        self.joint_command_pub.publish(msg)

    def wait_seconds(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, float(seconds))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))


def _build_target_pose(args: argparse.Namespace, start_pose: Sequence[float]) -> tuple[list[float], list[float], list[float]]:
    start_quat, start_translation = dq_pose_to_quaternion_translation(start_pose)
    if args.target_dq is not None:
        _, target_translation = dq_pose_to_quaternion_translation(args.target_dq)
        return list(args.target_dq), start_translation, target_translation

    target_xyz_values = [args.target_x, args.target_y, args.target_z]
    target_quat_values = [args.target_qw, args.target_qx, args.target_qy, args.target_qz]
    has_target_xyz = any(value is not None for value in target_xyz_values)
    has_target_quat = _require_all_or_none(target_quat_values, "target quaternion")

    if has_target_xyz or has_target_quat:
        target_translation = [
            float(args.target_x) if args.target_x is not None else start_translation[0],
            float(args.target_y) if args.target_y is not None else start_translation[1],
            float(args.target_z) if args.target_z is not None else start_translation[2],
        ]
    else:
        target_translation = [
            start_translation[0] + float(args.dx),
            start_translation[1] + float(args.dy),
            start_translation[2] + float(args.dz),
        ]

    target_quat = [float(value) for value in target_quat_values] if has_target_quat else start_quat
    target_pose = pose_dq_from_quaternion_translation(
        qw=target_quat[0],
        qx=target_quat[1],
        qy=target_quat[2],
        qz=target_quat[3],
        x=target_translation[0],
        y=target_translation[1],
        z=target_translation[2],
    )
    return target_pose, start_translation, target_translation


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate an absolute-pose trajectory and track it online with dq_vizlab in qpin_sim_env."
    )
    parser.add_argument("--namespace", default="qpin_sim")
    parser.add_argument("--solver-binary", default=str(DEFAULT_SOLVER_BINARY))
    parser.add_argument("--urdf-path", default=str(DEFAULT_SOLVER_URDF))
    parser.add_argument("--joint-state-timeout", type=float, default=2.0)
    parser.add_argument("--require-joint-state", action="store_true")
    parser.add_argument("--sample-count", type=int, default=30)
    parser.add_argument("--step-dt", type=float, default=0.1)
    parser.add_argument("--settle-time", type=float, default=0.05)
    parser.add_argument("--include-waist", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--keep-body-z-vertical", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--joint-limit-avoidance", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dry-run", action="store_true")

    parser.add_argument("--dx", type=float, default=0.0)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=-0.03)

    parser.add_argument("--target-x", type=float)
    parser.add_argument("--target-y", type=float)
    parser.add_argument("--target-z", type=float)
    parser.add_argument("--target-qw", type=float)
    parser.add_argument("--target-qx", type=float)
    parser.add_argument("--target-qy", type=float)
    parser.add_argument("--target-qz", type=float)
    parser.add_argument("--target-dq", type=_parse_dq)
    return parser


def main() -> None:
    args = _build_arg_parser().parse_args()

    rclpy.init()
    node = DQAbsoluteTrackDemo(namespace=args.namespace)
    try:
        q_start, q_source = node.wait_for_joint_state(
            timeout_sec=args.joint_state_timeout,
            require=args.require_joint_state,
        )

        with NullspaceSolverClient(
            solver_binary=args.solver_binary,
            urdf_path=args.urdf_path,
        ) as solver:
            start_state = solver.get_state(q_start)
            target_pose, start_translation, target_translation = _build_target_pose(args, start_state.absolute_pose)
            pose_samples = interpolate_absolute_pose_waypoints(
                start_state.absolute_pose,
                target_pose,
                max(2, int(args.sample_count)),
            )

            print(f"q source: {q_source}", flush=True)
            print(f"start absolute xyz: {[round(value, 6) for value in start_translation]}", flush=True)
            print(f"target absolute xyz: {[round(value, 6) for value in target_translation]}", flush=True)
            print(
                "tracking: "
                f"samples={len(pose_samples)}, step_dt={args.step_dt}, "
                f"include_waist={args.include_waist}, keep_body_z_vertical={args.keep_body_z_vertical}",
                flush=True,
            )

            if args.dry_run:
                print("dry run: trajectory generated but not tracked", flush=True)
                return

            solve_times: list[float] = []
            translation_errors: list[float] = []
            rotation_errors: list[float] = []
            retry_count = 0
            q_command = q_start[:]
            start_time = time.monotonic()

            for index, target_sample in enumerate(pose_samples):
                sample_deadline = start_time + index * float(args.step_dt)
                while rclpy.ok() and time.monotonic() < sample_deadline:
                    rclpy.spin_once(node, timeout_sec=min(0.02, sample_deadline - time.monotonic()))

                t0 = time.perf_counter()
                q_feedback = node.current_q[:]
                result = solver.solve_absolute(
                    q=q_feedback,
                    target_pose=target_sample,
                    include_waist=args.include_waist,
                    keep_body_z_vertical=args.keep_body_z_vertical,
                    joint_limit_avoidance=args.joint_limit_avoidance,
                )
                if not result.apply_result:
                    result = solver.solve_absolute(
                        q=q_command,
                        target_pose=target_sample,
                        include_waist=args.include_waist,
                        keep_body_z_vertical=args.keep_body_z_vertical,
                        joint_limit_avoidance=args.joint_limit_avoidance,
                    )
                    retry_count += 1
                solve_times.append(time.perf_counter() - t0)
                if not result.apply_result:
                    raise RuntimeError(f"tracking solve failed at sample {index}: {result.reason}")

                q_command = result.q[:]
                node.publish_joint_command(result.q)
                node.wait_seconds(args.settle_time)
                actual_state = solver.get_state(node.current_q)
                translation_error, rotation_error = _pose_error(target_sample, actual_state.absolute_pose)
                translation_errors.append(translation_error)
                rotation_errors.append(rotation_error)

            final_state = solver.get_state(node.current_q)
            final_translation_error, final_rotation_error = _pose_error(target_pose, final_state.absolute_pose)

            print(f"mean solve time: {1000.0 * sum(solve_times) / len(solve_times):.3f} ms", flush=True)
            print(f"max solve time: {1000.0 * max(solve_times):.3f} ms", flush=True)
            print(f"retry count: {retry_count}", flush=True)
            print(f"mean translation error: {1000.0 * sum(translation_errors) / len(translation_errors):.3f} mm", flush=True)
            print(f"max translation error: {1000.0 * max(translation_errors):.3f} mm", flush=True)
            print(f"final translation error: {1000.0 * final_translation_error:.3f} mm", flush=True)
            print(f"final rotation error: {math.degrees(final_rotation_error):.3f} deg", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
