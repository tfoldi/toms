"""toms_preflight.launch.py – hardware preflight + optional smoke test.

Runs the TOMS preflight checker against a live ROS2 graph and optionally
executes a minimal smoke test (gripper cycle + MoveIt2 plan probe).

PRE-FLIGHT REQUIREMENTS for live checks (--ros bindings phase):
  [ ] cello_moveit_config demo.launch.py is running (MoveIt2 + ros2_control)
  [ ] /joint_states publishing at >= 10 Hz
  [ ] /arm_controller/follow_joint_trajectory action server is available
  [ ] /hand_controller/gripper_cmd action server is available
  [ ] /plan_kinematic_path service is available
  [ ] robot_state_publisher is publishing TF

PRE-FLIGHT REQUIREMENTS for smoke test phase (smoke_test:=true):
  All of the above PLUS:
  [ ] Robot is in a safe, unobstructed position
  [ ] Gripper is unobstructed (no object in jaws)

Usage::

    # Config check + ROS binding check only (no gripper motion):
    ros2 launch toms_bt toms_preflight.launch.py smoke_test:=false

    # Full preflight including gripper cycle + MoveIt2 plan probe:
    ros2 launch toms_bt toms_preflight.launch.py

    # Full preflight including a small safe motion execution:
    ros2 launch toms_bt toms_preflight.launch.py execute_motion:=true

    # Different robot overlay:
    ros2 launch toms_bt toms_preflight.launch.py robot:=cello_follower

Config loading order (later entries override earlier):
  1. config/robot.yaml         – base schema + null defaults
  2. config/robots/<robot>.yaml – robot-specific overlay
  3. config/planning.yaml      – MoveIt2 planning parameters
  4. config/task.yaml          – task runner parameters
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
    robot_arg = DeclareLaunchArgument(
        "robot", default_value="cello_follower",
        description="Robot overlay name (loads config/robots/<robot>.yaml)",
    )
    runs_base_dir_arg = DeclareLaunchArgument(
        "runs_base_dir", default_value="runs",
        description="Base directory for preflight run artifact subdirectories",
    )
    smoke_test_arg = DeclareLaunchArgument(
        "smoke_test", default_value="true",
        description="Run gripper cycle + MoveIt2 plan probe after ROS binding checks",
    )
    execute_motion_arg = DeclareLaunchArgument(
        "execute_motion", default_value="false",
        description=(
            "Actually execute a small safe motion during smoke test. "
            "Requires a clear workspace and home_joints set in the robot overlay."
        ),
    )

    robot_name = "cello_follower"   # TODO: read from LaunchConfiguration("robot")

    snapshot_paths = ",".join([
        _config("robot.yaml"),
        _config(f"robots/{robot_name}.yaml"),
        _config("planning.yaml"),
        _config("task.yaml"),
    ])

    preflight_node = Node(
        package="toms_bt",
        executable="run_preflight",
        name="toms_preflight",
        parameters=[{
            "robot": LaunchConfiguration("robot"),
            "runs_base_dir": LaunchConfiguration("runs_base_dir"),
            "smoke_test": LaunchConfiguration("smoke_test"),
            "execute_motion": LaunchConfiguration("execute_motion"),
            "snapshot_configs": snapshot_paths,
        }],
        output="screen",
    )

    return LaunchDescription([
        robot_arg,
        runs_base_dir_arg,
        smoke_test_arg,
        execute_motion_arg,
        preflight_node,
    ])
