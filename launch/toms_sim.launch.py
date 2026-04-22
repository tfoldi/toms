"""toms_sim.launch.py – launch the TOMS stack in simulation mode.

Uses mock adapters for perception and robot bridge.
No hardware required; safe to run on a development machine.

Usage:
    ros2 launch toms_bt toms_sim.launch.py
    ros2 launch toms_bt toms_sim.launch.py task_id:=my_run retry_budget:=5
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config(relative: str) -> str:
    return os.path.join(
        get_package_share_directory("toms_bt"),
        "..", "..", "..", "..", "config", relative,
    )


def generate_launch_description() -> LaunchDescription:
    # Simulation loads the robot schema + cello_star overlay so the same
    # topic/frame names are used, but sim_mode=True substitutes mock adapters.
    robot_arg = DeclareLaunchArgument(
        "robot", default_value="cello_star",
        description="Robot overlay name (file: config/robots/<robot>.yaml)",
    )
    task_id_arg = DeclareLaunchArgument(
        "task_id", default_value="pen_to_holder_sim", description="Unique task run ID"
    )
    retry_budget_arg = DeclareLaunchArgument(
        "retry_budget", default_value="3", description="Max retries per object"
    )
    tick_rate_arg = DeclareLaunchArgument(
        "tick_rate_hz", default_value="10.0", description="BT tick rate"
    )

    robot_name = "cello_star"   # TODO: read from LaunchConfiguration("robot")

    bt_runner_node = Node(
        package="toms_bt",
        executable="run_task",
        name="toms_bt_runner",
        parameters=[
            _config("robot.yaml"),                # 1. base schema
            _config(f"robots/{robot_name}.yaml"), # 2. robot overlay
            _config("planning.yaml"),             # 3. planning
            _config("task.yaml"),                 # 4. task (perception stays mock)
            {
                "task_id": LaunchConfiguration("task_id"),
                "retry_budget": LaunchConfiguration("retry_budget"),
                "tick_rate_hz": LaunchConfiguration("tick_rate_hz"),
                "sim_mode": True,
            },
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_arg,
        task_id_arg,
        retry_budget_arg,
        tick_rate_arg,
        bt_runner_node,
    ])
