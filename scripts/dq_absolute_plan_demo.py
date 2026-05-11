#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time
from typing import Sequence

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DQ_VIZLAB_ROOT = Path(os.environ.get("DQ_VIZLAB_ROOT", "/home/syx/dq_vizlab/dq_vizlab")).expanduser()
SDK_PYTHON = DQ_VIZLAB_ROOT / "python_sdk" / "python"
if SDK_PYTHON.exists():
    sys.path.insert(0, str(SDK_PYTHON))

from sailor_sdk import (  # noqa: E402
    AbsolutePoseTOPPRAPlanner,
    DEFAULT_INITIAL_Q,
    HUMANOID_JOINT_NAMES,
    NullspaceSolverClient,
    dq_pose_to_quaternion_translation,
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


def _duration_from_seconds(seconds: float) -> Duration:
    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    nanos = int(round((seconds - whole) * 1_000_000_000))
    if nanos >= 1_000_000_000:
        whole += 1
        nanos -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanos)


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


class DQAbsolutePlanDemo(Node):
    def __init__(self, *, namespace: str) -> None:
        super().__init__("qpin_dq_absolute_plan_demo")
        prefix = _topic_prefix(namespace)
        self.joint_state_topic = f"{prefix}/joint_states"
        self.trajectory_topic = f"{prefix}/joint_trajectory"
        self.current_q = [float(value) for value in DEFAULT_INITIAL_Q]
        self.has_joint_state = False
        self.joint_index = {name: index for index, name in enumerate(HUMANOID_JOINT_NAMES)}
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            10,
        )
        self.trajectory_pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)

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

    def wait_for_trajectory_subscriber(self, *, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            if self.trajectory_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        return self.trajectory_pub.get_subscription_count() > 0

    def publish_trajectory(self, trajectory) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(HUMANOID_JOINT_NAMES)

        for index, sample_time in enumerate(trajectory.sample_times):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in trajectory.positions[index]]
            if index < len(trajectory.velocities):
                point.velocities = [float(value) for value in trajectory.velocities[index]]
            if index < len(trajectory.accelerations):
                point.accelerations = [float(value) for value in trajectory.accelerations[index]]
            point.time_from_start = _duration_from_seconds(sample_time)
            msg.points.append(point)

        self.trajectory_pub.publish(msg)


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
        description="Plan a dq_vizlab absolute-pose trajectory and publish it to qpin_sim_env."
    )
    parser.add_argument("--namespace", default="qpin_sim")
    parser.add_argument("--solver-binary", default=str(DEFAULT_SOLVER_BINARY))
    parser.add_argument("--urdf-path", default=str(DEFAULT_SOLVER_URDF))
    parser.add_argument("--joint-state-timeout", type=float, default=2.0)
    parser.add_argument("--require-joint-state", action="store_true")
    parser.add_argument("--publisher-match-timeout", type=float, default=2.0)
    parser.add_argument("--waypoint-count", type=int, default=4)
    parser.add_argument("--sample-count", type=int, default=80)
    parser.add_argument("--max-vel", type=float, default=0.45)
    parser.add_argument("--max-acc", type=float, default=0.90)
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
    node = DQAbsolutePlanDemo(namespace=args.namespace)
    try:
        q_start, q_source = node.wait_for_joint_state(
            timeout_sec=args.joint_state_timeout,
            require=args.require_joint_state,
        )

        solver = NullspaceSolverClient(
            solver_binary=args.solver_binary,
            urdf_path=args.urdf_path,
        )
        planner = AbsolutePoseTOPPRAPlanner()
        start_state = solver.get_state(q_start)
        target_pose, start_translation, target_translation = _build_target_pose(args, start_state.absolute_pose)

        print(f"q source: {q_source}", flush=True)
        print(f"start absolute xyz: {[round(value, 6) for value in start_translation]}", flush=True)
        print(f"target absolute xyz: {[round(value, 6) for value in target_translation]}", flush=True)
        print(
            "planning: "
            f"waypoints={args.waypoint_count}, samples={args.sample_count}, "
            f"include_waist={args.include_waist}, keep_body_z_vertical={args.keep_body_z_vertical}",
            flush=True,
        )

        t0 = time.perf_counter()
        plan = planner.plan_absolute_pose(
            solver,
            q_start=q_start,
            target_pose=target_pose,
            include_waist=args.include_waist,
            keep_body_z_vertical=args.keep_body_z_vertical,
            joint_limit_avoidance=args.joint_limit_avoidance,
            waypoint_count=args.waypoint_count,
            max_vel=args.max_vel,
            max_acc=args.max_acc,
            sample_count=args.sample_count,
        )
        planning_seconds = time.perf_counter() - t0

        print(f"planning wall time: {planning_seconds:.3f}s", flush=True)
        print(f"trajectory duration: {plan.trajectory.duration:.3f}s", flush=True)
        print(f"joint waypoints: {len(plan.joint_waypoints)}", flush=True)
        print(f"trajectory samples: {len(plan.trajectory.sample_times)}", flush=True)
        if plan.solve_results:
            print(f"final solve reason: {plan.solve_results[-1].reason}", flush=True)

        if args.dry_run:
            print("dry run: trajectory was not published", flush=True)
            return

        has_subscriber = node.wait_for_trajectory_subscriber(timeout_sec=args.publisher_match_timeout)
        if not has_subscriber:
            print(f"warning: no subscriber matched on {node.trajectory_topic}; publishing once anyway", flush=True)
        node.publish_trajectory(plan.trajectory)
        print(f"published JointTrajectory to {node.trajectory_topic}", flush=True)
        rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
