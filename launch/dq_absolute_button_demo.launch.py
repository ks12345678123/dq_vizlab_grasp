from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _env_dir() -> Path:
    return Path(__file__).resolve().parents[1]


def generate_launch_description():
    env_dir = _env_dir()
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="qpin_sim"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("visual_follower", default_value="true"),
            DeclareLaunchArgument("zero_initial_pose", default_value="false"),
            DeclareLaunchArgument("target_x", default_value="0.6939863951"),
            DeclareLaunchArgument("target_y", default_value="-0.0565121498"),
            DeclareLaunchArgument("target_z", default_value="0.7430594672"),
            DeclareLaunchArgument("waypoint_count", default_value="12"),
            DeclareLaunchArgument("sample_count", default_value="160"),
            DeclareLaunchArgument("max_vel", default_value="0.20"),
            DeclareLaunchArgument("max_acc", default_value="0.35"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(env_dir / "launch" / "gazebo_visual.launch.py")),
                launch_arguments={
                    "namespace": namespace,
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "visual_follower": LaunchConfiguration("visual_follower"),
                    "zero_initial_pose": LaunchConfiguration("zero_initial_pose"),
                }.items(),
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(env_dir / "scripts" / "dq_absolute_button_ui.py"),
                    "--namespace",
                    namespace,
                    "--target-x",
                    LaunchConfiguration("target_x"),
                    "--target-y",
                    LaunchConfiguration("target_y"),
                    "--target-z",
                    LaunchConfiguration("target_z"),
                    "--waypoint-count",
                    LaunchConfiguration("waypoint_count"),
                    "--sample-count",
                    LaunchConfiguration("sample_count"),
                    "--max-vel",
                    LaunchConfiguration("max_vel"),
                    "--max-acc",
                    LaunchConfiguration("max_acc"),
                ],
                output="screen",
            ),
        ]
    )
