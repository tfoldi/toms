"""toms_hardware.launch.py – launch the TOMS stack against real hardware.

Config loading order (later files override earlier ones):
  1. config/robot.yaml          – schema and safe defaults
  2. config/robots/<name>.yaml  – robot-specific overlay  ← set via `robot:=`
  3. config/planning.yaml
  4. config/perception.yaml
  5. config/task.yaml

IMPORTANT: Do NOT run until the following are resolved in cello_follower.yaml:
  [ ] gripper.open_width measured on physical robot
  [ ] gripper.tool_frame TCP offset defined and measured
  [ ] camera.* topics confirmed (sensor mounted and driver running)
  [ ] calibration.method and calibration.transform_file (hand-eye cal done)
  [ ] planning.yaml: moveit2.planner_id confirmed
  [ ] MoveIt2 + ros2_control already running (or add IncludeLaunchDescription)

Usage:
    ros2 launch toms_bt toms_hardware.launch.py
    ros2 launch toms_bt toms_hardware.launch.py robot:=cello_follower task_id:=run_001
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config(relative: str) -> str:
    """Resolve a path relative to the top-level config/ directory."""
    return os.path.join(
        get_package_share_directory("toms_bt"),
        "..", "..", "..", "..", "config", relative,
    )


def generate_launch_description() -> LaunchDescription:
    robot_arg = DeclareLaunchArgument(
        "robot", default_value="cello_follower",
        description="Robot overlay name (file: config/robots/<robot>.yaml)",
    )
    task_id_arg = DeclareLaunchArgument(
        "task_id", default_value="pen_to_holder_hw",
        description="Unique task run ID",
    )
    retry_budget_arg = DeclareLaunchArgument(
        "retry_budget", default_value="3",
        description="Max retries per object",
    )

    # Config files loaded in override order.
    # LaunchConfiguration cannot be used directly in os.path.join; the robot
    # overlay path is resolved at launch time via a helper below.
    # For now, default to cello_follower. To support dynamic selection, use
    # launch.substitutions.PathJoinSubstitution when ament paths are available.
    robot_name = "cello_follower"   # TODO: read from LaunchConfiguration("robot")
                                # once PathJoinSubstitution supports config/ dir

    bt_runner_node = Node(
        package="toms_bt",
        executable="run_task",
        name="toms_bt_runner",
        parameters=[
            _config("robot.yaml"),                        # 1. base schema
            _config(f"robots/{robot_name}.yaml"),         # 2. robot overlay
            _config("planning.yaml"),                     # 3. planning
            _config("perception.yaml"),                   # 4. perception
            _config("task.yaml"),                         # 5. task
            {
                "task_id": LaunchConfiguration("task_id"),
                "retry_budget": LaunchConfiguration("retry_budget"),
                "sim_mode": False,
            },
        ],
        output="screen",
    )

    # TODO: IncludeLaunchDescription for cello_moveit_config demo.launch.py
    # TODO: IncludeLaunchDescription for ros2_control hardware interface
    # TODO: IncludeLaunchDescription for perception node (camera driver + detector)

    return LaunchDescription([
        robot_arg,
        task_id_arg,
        retry_budget_arg,
        bt_runner_node,
    ])
