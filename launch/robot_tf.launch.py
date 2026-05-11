from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _env_dir() -> Path:
    return Path(__file__).resolve().parents[1]


def _robot_urdf_path(env_dir: Path) -> Path:
    return env_dir / "assets" / "sailor_r1_pro_description" / "urdf" / "sailor_r1_pro_description.urdf"


def _resolved_robot_description(urdf_path: Path) -> str:
    description_dir = urdf_path.parents[1]
    robot_description = urdf_path.read_text(encoding="utf-8").replace(
        "package://sailor_r1_pro_description/",
        f"file://{description_dir}/",
    )
    lines = robot_description.splitlines()
    if lines and lines[0].lstrip().startswith("<?xml"):
        robot_description = "\n".join(lines[1:])
    Path("/tmp/qpin_sim_env_resolved.urdf").write_text(robot_description, encoding="utf-8")
    return robot_description


def generate_launch_description():
    env_dir = _env_dir()
    urdf_path = _robot_urdf_path(env_dir)
    robot_description = _resolved_robot_description(urdf_path)
    rviz_path = env_dir / "rviz" / "qpin_sim_env.rviz"

    namespace = LaunchConfiguration("namespace")
    rate_hz = LaunchConfiguration("rate_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="qpin_sim"),
            DeclareLaunchArgument("rate_hz", default_value="30.0"),
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[
                    ("/tf", ["/", namespace, "/tf"]),
                    ("/tf_static", ["/", namespace, "/tf_static"]),
                    ("joint_states", ["/", namespace, "/joint_states"]),
                    ("robot_description", ["/", namespace, "/robot_description"]),
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(env_dir / "scripts" / "joint_state_source.py"),
                    "--namespace",
                    namespace,
                    "--rate-hz",
                    rate_hz,
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                namespace=namespace,
                output="screen",
                arguments=["-d", str(rviz_path)],
                remappings=[
                    ("/tf", ["/", namespace, "/tf"]),
                    ("/tf_static", ["/", namespace, "/tf_static"]),
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
