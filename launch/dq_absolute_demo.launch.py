from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
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
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("visual_follower", default_value="true"),
            DeclareLaunchArgument("run_demo", default_value="true"),
            DeclareLaunchArgument("demo_delay", default_value="1.0"),
            DeclareLaunchArgument("dx", default_value="0.0"),
            DeclareLaunchArgument("dy", default_value="0.0"),
            DeclareLaunchArgument("dz", default_value="-0.12"),
            DeclareLaunchArgument("waypoint_count", default_value="8"),
            DeclareLaunchArgument("sample_count", default_value="120"),
            DeclareLaunchArgument("max_vel", default_value="0.20"),
            DeclareLaunchArgument("max_acc", default_value="0.35"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(env_dir / "launch" / "gazebo_visual.launch.py")),
                launch_arguments={
                    "namespace": namespace,
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "visual_follower": LaunchConfiguration("visual_follower"),
                }.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("demo_delay"),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            str(env_dir / "scripts" / "dq_absolute_plan_demo.py"),
                            "--namespace",
                            namespace,
                            "--dx",
                            LaunchConfiguration("dx"),
                            "--dy",
                            LaunchConfiguration("dy"),
                            "--dz",
                            LaunchConfiguration("dz"),
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
                        condition=IfCondition(LaunchConfiguration("run_demo")),
                    )
                ],
            ),
        ]
    )
