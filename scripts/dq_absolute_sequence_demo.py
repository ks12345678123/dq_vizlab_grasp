#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import time
from typing import Sequence

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
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
DEFAULT_TARGET_SEQUENCE = (
    "0.56,-0.03,0.86;"
    "0.6939863951,-0.0565121498,0.7430594672;"
    "0.62,0.04,0.81"
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


def _parse_xyz_sequence(text: str) -> list[list[float]]:
    waypoints: list[list[float]] = []
    for chunk in str(text).split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        values = [float(item.strip()) for item in chunk.split(",") if item.strip()]
        if len(values) != 3:
            raise argparse.ArgumentTypeError(
                "target sequence expects triplets like 'x1,y1,z1;x2,y2,z2'"
            )
        waypoints.append(values)
    if not waypoints:
        raise argparse.ArgumentTypeError("target sequence cannot be empty")
    return waypoints


class DQAbsoluteSequenceDemo(Node):
    def __init__(self, *, namespace: str) -> None:
        super().__init__("qpin_dq_absolute_sequence_demo")
        prefix = _topic_prefix(namespace)
        self.joint_state_topic = f"{prefix}/joint_states"
        self.trajectory_topic = f"{prefix}/joint_trajectory"
        self.visual_ready_topic = f"{prefix}/visual_ready"
        self.current_q = [float(value) for value in DEFAULT_INITIAL_Q]
        self.has_joint_state = False
        self.has_visual_ready = False
        self.joint_index = {name: index for index, name in enumerate(HUMANOID_JOINT_NAMES)}
        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(JointState, self.joint_state_topic, self._on_joint_state, 10)
        self.create_subscription(Bool, self.visual_ready_topic, self._on_visual_ready, ready_qos)
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

    def _on_visual_ready(self, msg: Bool) -> None:
        self.has_visual_ready = bool(msg.data)

    def wait_for_visual_ready(self, *, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            if self.has_visual_ready:
                return True
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        return self.has_visual_ready

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

    def wait_seconds(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, float(seconds))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))

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


def _build_target_pose(xyz: Sequence[float], start_pose: Sequence[float]) -> list[float]:
    start_quat, _ = dq_pose_to_quaternion_translation(start_pose)
    return pose_dq_from_quaternion_translation(
        qw=start_quat[0],
        qx=start_quat[1],
        qy=start_quat[2],
        qz=start_quat[3],
        x=float(xyz[0]),
        y=float(xyz[1]),
        z=float(xyz[2]),
    )


def _concatenate_trajectories(trajectories: Sequence[object]) -> object:
    if not trajectories:
        raise ValueError("at least one trajectory is required")

    sample_times: list[float] = []
    positions: list[list[float]] = []
    velocities: list[list[float]] = []
    accelerations: list[list[float]] = []
    cumulative_time = 0.0

    for segment_index, trajectory in enumerate(trajectories):
        start_sample = 0 if segment_index == 0 else 1
        for sample_index in range(start_sample, len(trajectory.sample_times)):
            sample_times.append(cumulative_time + float(trajectory.sample_times[sample_index]))
            positions.append([float(value) for value in trajectory.positions[sample_index]])
            if sample_index < len(trajectory.velocities):
                velocities.append([float(value) for value in trajectory.velocities[sample_index]])
            if sample_index < len(trajectory.accelerations):
                accelerations.append([float(value) for value in trajectory.accelerations[sample_index]])
        cumulative_time += float(trajectory.duration)

    return SimpleNamespace(
        duration=cumulative_time,
        sample_times=sample_times,
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plan and execute multiple absolute-pose trajectories continuously in qpin_sim_env."
    )
    parser.add_argument("--namespace", default="qpin_sim")
    parser.add_argument("--solver-binary", default=str(DEFAULT_SOLVER_BINARY))
    parser.add_argument("--urdf-path", default=str(DEFAULT_SOLVER_URDF))
    parser.add_argument("--joint-state-timeout", type=float, default=2.0)
    parser.add_argument("--require-joint-state", action="store_true")
    parser.add_argument("--publisher-match-timeout", type=float, default=2.0)
    parser.add_argument("--visual-ready-timeout", type=float, default=15.0)
    parser.add_argument("--execution-settle-time", type=float, default=0.1)
    parser.add_argument("--waypoint-count", type=int, default=12)
    parser.add_argument("--sample-count", type=int, default=160)
    parser.add_argument("--max-vel", type=float, default=0.20)
    parser.add_argument("--max-acc", type=float, default=0.35)
    parser.add_argument("--include-waist", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--keep-body-z-vertical", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--joint-limit-avoidance", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--target-sequence", type=_parse_xyz_sequence, default=_parse_xyz_sequence(DEFAULT_TARGET_SEQUENCE))
    return parser


def main() -> None:
    args = _build_arg_parser().parse_args()

    rclpy.init()
    node = DQAbsoluteSequenceDemo(namespace=args.namespace)
    try:
        visual_ready = node.wait_for_visual_ready(timeout_sec=args.visual_ready_timeout)
        print(f"visual ready: {visual_ready} on {node.visual_ready_topic}", flush=True)
        q_start, q_source = node.wait_for_joint_state(
            timeout_sec=args.joint_state_timeout,
            require=args.require_joint_state,
        )

        with NullspaceSolverClient(
            solver_binary=args.solver_binary,
            urdf_path=args.urdf_path,
        ) as solver:
            planner = AbsolutePoseTOPPRAPlanner()
            has_subscriber = node.wait_for_trajectory_subscriber(timeout_sec=args.publisher_match_timeout)
            if not has_subscriber and not args.dry_run:
                print(f"warning: no subscriber matched on {node.trajectory_topic}; publishing anyway", flush=True)

            print(f"q source: {q_source}", flush=True)
            print(f"segments: {len(args.target_sequence)}", flush=True)

            total_planning_seconds = 0.0
            total_trajectory_seconds = 0.0
            trajectories = []
            q_current = q_start[:]

            for index, target_xyz in enumerate(args.target_sequence, start=1):
                start_state = solver.get_state(q_current)
                target_pose = _build_target_pose(target_xyz, start_state.absolute_pose)

                print(
                    f"segment {index}: start_xyz={[round(v, 6) for v in dq_pose_to_quaternion_translation(start_state.absolute_pose)[1]]} "
                    f"target_xyz={[round(v, 6) for v in target_xyz]}",
                    flush=True,
                )

                t0 = time.perf_counter()
                plan = planner.plan_absolute_pose(
                    solver,
                    q_start=q_current,
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
                total_planning_seconds += planning_seconds
                total_trajectory_seconds += float(plan.trajectory.duration)
                q_current = plan.final_state.joint_positions[:]
                trajectories.append(plan.trajectory)

                print(
                    f"segment {index}: planning={planning_seconds:.3f}s duration={plan.trajectory.duration:.3f}s "
                    f"waypoints={len(plan.joint_waypoints)} samples={len(plan.trajectory.sample_times)} "
                    f"reason={plan.solve_results[-1].reason if plan.solve_results else 'n/a'}",
                    flush=True,
                )

            merged_trajectory = _concatenate_trajectories(trajectories)

            print(f"total planning wall time: {total_planning_seconds:.3f}s", flush=True)
            print(f"total trajectory duration: {total_trajectory_seconds:.3f}s", flush=True)
            print(f"merged trajectory samples: {len(merged_trajectory.sample_times)}", flush=True)

            if args.dry_run:
                print("dry run: sequence was planned but not published", flush=True)
            else:
                node.publish_trajectory(merged_trajectory)
                node.wait_seconds(merged_trajectory.duration + args.execution_settle_time)
                print(f"sequence published on {node.trajectory_topic}", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
