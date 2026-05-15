from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    world_path = env_dir / "worlds" / "qpin_visual_scene.world"
    rviz_path = env_dir / "rviz" / "qpin_sim_env.rviz"

    namespace = LaunchConfiguration("namespace")
    rate_hz = LaunchConfiguration("rate_hz")
    zero_initial_pose = LaunchConfiguration("zero_initial_pose")
    gazebo_launch = PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="qpin_sim"),
            DeclareLaunchArgument("rate_hz", default_value="30.0"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("visual_follower", default_value="true"),
            DeclareLaunchArgument("zero_initial_pose", default_value="false"),
            IncludeLaunchDescription(
                gazebo_launch,
                launch_arguments={
                    "world": str(world_path),
                    "gui": LaunchConfiguration("gui"),
                    "verbose": "false",
                }.items(),
            ),
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
                    "--zero-initial-pose",
                    zero_initial_pose,
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(env_dir / "scripts" / "gazebo_robot_visual_follower.py"),
                    "--urdf",
                    str(urdf_path),
                    "--root-frame",
                    "base_link",
                    "--world-pose-topic",
                    "/qpin_sim/world_from_base",
                    "--ready-topic",
                    "/qpin_sim/visual_ready",
                    "--exclude-link",
                    "gazebo_camera_link",
                    "--rate-hz",
                    "15",
                    "--ros-args",
                    "-r",
                    "/tf:=/qpin_sim/tf",
                    "-r",
                    "/tf_static:=/qpin_sim/tf_static",
                ],
                condition=IfCondition(LaunchConfiguration("visual_follower")),
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
