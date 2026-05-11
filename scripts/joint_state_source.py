#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


JOINT_ORDER = [
    "left_wheel_joint",
    "right_wheel_joint",
    "ankle_joint",
    "knee_joint",
    "hip_pitch_joint",
    "hip_yaw_joint",
    "ljoint1",
    "ljoint2",
    "ljoint3",
    "ljoint4",
    "ljoint5",
    "ljoint6",
    "ljoint7",
    "rjoint1",
    "rjoint2",
    "rjoint3",
    "rjoint4",
    "rjoint5",
    "rjoint6",
    "rjoint7",
    "head_yaw_joint",
    "head_pitch_joint",
]


DEFAULT_POSITIONS = {
    "left_wheel_joint": 0.0,
    "right_wheel_joint": 0.0,
    "ankle_joint": -0.3,
    "knee_joint": 0.9,
    "hip_pitch_joint": -0.6,
    "hip_yaw_joint": 0.0,
    "ljoint1": -0.5236,
    "ljoint2": 0.0,
    "ljoint3": 0.0,
    "ljoint4": -1.0471,
    "ljoint5": 0.0,
    "ljoint6": 0.0,
    "ljoint7": 0.0,
    "rjoint1": 0.5236,
    "rjoint2": 0.0,
    "rjoint3": 0.0,
    "rjoint4": 1.0471,
    "rjoint5": 0.0,
    "rjoint6": 0.0,
    "rjoint7": 0.0,
    "head_yaw_joint": 0.0,
    "head_pitch_joint": 0.0,
}


@dataclass
class ActiveTrajectory:
    joint_names: list[str]
    start_positions: dict[str, float]
    point_times: list[float]
    point_positions: list[list[float]]
    start_monotonic: float


def _duration_to_seconds(duration) -> float:
    return float(duration.sec) + 1e-9 * float(duration.nanosec)


class JointStateSource(Node):
    def __init__(self, *, namespace: str, rate_hz: float) -> None:
        super().__init__("qpin_sim_joint_state_source")
        ns = namespace.strip("/")
        self.topic_prefix = f"/{ns}" if ns else ""
        self.period = 1.0 / max(1.0, float(rate_hz))
        self.positions = dict(DEFAULT_POSITIONS)
        self.active_trajectory: ActiveTrajectory | None = None

        self.joint_pub = self.create_publisher(JointState, f"{self.topic_prefix}/joint_states", 10)
        self.world_pose_pub = self.create_publisher(PoseStamped, f"{self.topic_prefix}/world_from_base", 10)
        self.legacy_world_pose_pub = self.create_publisher(PoseStamped, f"{self.topic_prefix}/world_from_car", 10)
        self.command_sub = self.create_subscription(
            JointState,
            f"{self.topic_prefix}/joint_command",
            self._on_joint_command,
            10,
        )
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            f"{self.topic_prefix}/joint_trajectory",
            self._on_joint_trajectory,
            10,
        )
        self.timer = self.create_timer(self.period, self._publish)
        self.get_logger().info(
            f"publishing JointState on {self.topic_prefix}/joint_states; "
            f"commands: {self.topic_prefix}/joint_command and {self.topic_prefix}/joint_trajectory"
        )

    def _on_joint_command(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        for name, value in zip(msg.name, msg.position):
            if name in self.positions:
                self.positions[name] = float(value)
        self.active_trajectory = None

    def _on_joint_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.joint_names or not msg.points:
            return
        valid_joint_names = [name for name in msg.joint_names if name in self.positions]
        if not valid_joint_names:
            return

        original_index = {name: idx for idx, name in enumerate(msg.joint_names)}
        point_times: list[float] = []
        point_positions: list[list[float]] = []
        for point in msg.points:
            if len(point.positions) < len(msg.joint_names):
                continue
            point_times.append(max(0.0, _duration_to_seconds(point.time_from_start)))
            point_positions.append([float(point.positions[original_index[name]]) for name in valid_joint_names])

        if not point_positions:
            return
        if point_times[0] > 0.0:
            point_times.insert(0, 0.0)
            point_positions.insert(0, [self.positions[name] for name in valid_joint_names])

        self.active_trajectory = ActiveTrajectory(
            joint_names=valid_joint_names,
            start_positions=dict(self.positions),
            point_times=point_times,
            point_positions=point_positions,
            start_monotonic=time.monotonic(),
        )

    def _apply_active_trajectory(self) -> None:
        traj = self.active_trajectory
        if traj is None:
            return
        elapsed = time.monotonic() - traj.start_monotonic
        if elapsed >= traj.point_times[-1]:
            for name, value in zip(traj.joint_names, traj.point_positions[-1]):
                self.positions[name] = float(value)
            self.active_trajectory = None
            return

        next_index = 1
        while next_index < len(traj.point_times) and traj.point_times[next_index] < elapsed:
            next_index += 1
        prev_index = max(0, next_index - 1)
        t0 = traj.point_times[prev_index]
        t1 = traj.point_times[next_index]
        alpha = 0.0 if t1 <= t0 else (elapsed - t0) / (t1 - t0)
        p0 = traj.point_positions[prev_index]
        p1 = traj.point_positions[next_index]
        for name, start, end in zip(traj.joint_names, p0, p1):
            self.positions[name] = (1.0 - alpha) * float(start) + alpha * float(end)

    def _publish_world_from_base(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.orientation.w = 1.0
        self.world_pose_pub.publish(msg)
        self.legacy_world_pose_pub.publish(msg)

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_ORDER)
        msg.position = [float(self.positions[name]) for name in JOINT_ORDER]
        self.joint_pub.publish(msg)

    def _publish(self) -> None:
        self._apply_active_trajectory()
        self._publish_world_from_base()
        self._publish_joint_state()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="qpin_sim")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = JointStateSource(namespace=args.namespace, rate_hz=args.rate_hz)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
