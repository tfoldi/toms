"""toms_diagnostics_dryrun.launch.py – sim run with full diagnostics.

Uses InstrumentedBtRunnerNode (run_task_instrumented executable) so that
every tick emits BT node transitions, validation results, and world-state
snapshots to a structured JSONL log inside a unique run directory.

Run artifacts are written to: runs/<task_id>_<YYYYMMDD_HHMMSS>/
  events.jsonl          – structured event log
  config_snapshot.yaml  – merged config at launch time
  rosout.txt            – Python-level console output
  run_metadata.json     – task_id, start time, git hash, outcome

Config loading order (later entries override earlier):
  1. config/robot.yaml
  2. config/robots/<robot>.yaml
  3. config/planning.yaml
  4. config/task.yaml  (sim_mode=True override below)

Usage::

    ros2 launch toms_bt toms_diagnostics_dryrun.launch.py
    ros2 launch toms_bt toms_diagnostics_dryrun.launch.py task_id:=my_test_001
    ros2 launch toms_bt toms_diagnostics_dryrun.launch.py runs_base_dir:=/tmp/toms_runs
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
        "task_id", default_value="dryrun",
        description="Unique task run ID (used as run directory prefix)",
    )
    retry_budget_arg = DeclareLaunchArgument(
        "retry_budget", default_value="3",
        description="Max grasp+place retries per object",
    )
    tick_rate_arg = DeclareLaunchArgument(
        "tick_rate_hz", default_value="10.0",
        description="BT tick rate (Hz)",
    )
    runs_base_dir_arg = DeclareLaunchArgument(
        "runs_base_dir", default_value="runs",
        description="Base directory for run artifact subdirectories",
    )

    robot_name = "cello_follower"  # TODO: read from LaunchConfiguration("robot")

    # Comma-separated list of config file paths for the config snapshot.
    # These are the files that will be deep-merged into config_snapshot.yaml.
    _snapshot_paths = ",".join([
        _config("robot.yaml"),
        _config(f"robots/{robot_name}.yaml"),
        _config("planning.yaml"),
        _config("task.yaml"),
    ])

    bt_runner_node = Node(
        package="toms_bt",
        executable="run_task_instrumented",
        name="toms_bt_runner",
        parameters=[{
            "task_id": LaunchConfiguration("task_id"),
            "retry_budget": LaunchConfiguration("retry_budget"),
            "tick_rate_hz": LaunchConfiguration("tick_rate_hz"),
            "sim_mode": True,
            "runs_base_dir": LaunchConfiguration("runs_base_dir"),
            "snapshot_configs": _snapshot_paths,
        }],
        output="screen",
    )

    return LaunchDescription([
        robot_arg,
        task_id_arg,
        retry_budget_arg,
        tick_rate_arg,
        runs_base_dir_arg,
        bt_runner_node,
    ])
