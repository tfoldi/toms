"""toms_hardware_test.launch.py – hardware run with full diagnostics.

Uses InstrumentedBtRunnerNode so every tick is recorded.  Run artifacts go to:
  runs/<task_id>_<YYYYMMDD_HHMMSS>/

PRE-FLIGHT CHECKLIST (must pass before launching):
  [ ] cello_moveit_config demo.launch.py is running (MoveIt2 + ros2_control)
  [ ] /joint_states is publishing at ≥ 10 Hz
  [ ] /arm_controller/follow_joint_trajectory action server is available
  [ ] /hand_controller/gripper_cmd action server is available
  [ ] /plan_kinematic_path service is available
  [ ] gripper.open_width measured and set in cello_follower.yaml
  [ ] camera.* topics set and verified (perception not integrated yet)
  [ ] calibration.transform_file set (hand-eye cal completed)

Config loading order (later entries override earlier):
  1. config/robot.yaml
  2. config/robots/<robot>.yaml
  3. config/planning.yaml
  4. config/perception.yaml
  5. config/task.yaml

Usage::

    ros2 launch toms_bt toms_hardware_test.launch.py
    ros2 launch toms_bt toms_hardware_test.launch.py task_id:=hw_run_001
    ros2 launch toms_bt toms_hardware_test.launch.py runs_base_dir:=/data/toms_runs
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
        description="Robot overlay name (file: config/robots/<robot>.yaml)",
    )
    task_id_arg = DeclareLaunchArgument(
        "task_id", default_value="hw_test",
        description="Unique task run ID (used as run directory prefix)",
    )
    retry_budget_arg = DeclareLaunchArgument(
        "retry_budget", default_value="3",
        description="Max grasp+place retries per object",
    )
    runs_base_dir_arg = DeclareLaunchArgument(
        "runs_base_dir", default_value="runs",
        description="Base directory for run artifact subdirectories",
    )

    robot_name = "cello_follower"  # TODO: read from LaunchConfiguration("robot")

    _snapshot_paths = ",".join([
        _config("robot.yaml"),
        _config(f"robots/{robot_name}.yaml"),
        _config("planning.yaml"),
        _config("perception.yaml"),
        _config("task.yaml"),
    ])

    bt_runner_node = Node(
        package="toms_bt",
        executable="run_task_instrumented",
        name="toms_bt_runner",
        parameters=[{
            "task_id": LaunchConfiguration("task_id"),
            "retry_budget": LaunchConfiguration("retry_budget"),
            "sim_mode": False,
            "runs_base_dir": LaunchConfiguration("runs_base_dir"),
            "snapshot_configs": _snapshot_paths,
        }],
        output="screen",
    )

    # TODO: add IncludeLaunchDescription for:
    #   cello_moveit_config/launch/demo.launch.py
    #   ros2_control hardware interface
    #   camera driver (once sensor is mounted)

    return LaunchDescription([
        robot_arg,
        task_id_arg,
        retry_budget_arg,
        runs_base_dir_arg,
        bt_runner_node,
    ])
