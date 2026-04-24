"""Tests for toms_robot.preflight – ConfigChecker, PreflightReport, format_report.

Pure-Python; no rclpy or ROS2 required.  RosChecker and SmokeTestRunner are
not tested here (they need a live graph).
"""
from __future__ import annotations

import os

import pytest
from toms_robot.config_loader import GripperConfig, KinematicsConfig, RobotConfig
from toms_robot.preflight import (
    CATEGORY_CONFIG,
    CATEGORY_MOVEIT,
    CATEGORY_ROS,
    CheckResult,
    ConfigChecker,
    PreflightReport,
    format_report,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_config(**kwargs) -> RobotConfig:
    """Return a minimal valid RobotConfig for testing.

    Override any field via keyword arguments.
    """
    defaults: dict = dict(
        model="test_arm",
        variant="test",
        dof=6,
        planning_group="arm",
        gripper_group="hand",
        base_frame="base_link",
        end_effector_link="link6",
        tool_frame=None,
        joint_names=["j1", "j2", "j3", "j4", "j5", "j6"],
        joint_state_topic="/joint_states",
        trajectory_action="/arm_controller/follow_joint_trajectory",
        gripper=GripperConfig(
            interface_type="gripper_action",
            action="/hand_controller/gripper_cmd",
            joint_name="joint7_left",
            open_position=-0.025,
            close_position=0.0,
            max_force=15.0,
            open_width=None,
        ),
        kinematics=KinematicsConfig(
            plugin="kdl_kinematics_plugin/KDLKinematicsPlugin",
        ),
        moveit_package="cello_moveit_config",
        urdf_package="cello_description",
        urdf_file="urdf/cello_description.urdf",
        srdf_package="cello_moveit_config",
        srdf_file="config/cello_description.srdf",
    )
    defaults.update(kwargs)
    return RobotConfig(**defaults)


# ---------------------------------------------------------------------------
# ConfigChecker – identity and frames
# ---------------------------------------------------------------------------


def test_config_checker_reports_model():
    results = ConfigChecker().check_all(_make_config())
    r = next(r for r in results if r.name == "robot.model")
    assert r.passed
    assert "test_arm" in r.message


def test_config_checker_dof_positive_passes():
    results = ConfigChecker().check_all(_make_config(dof=6))
    r = next(r for r in results if r.name == "robot.dof")
    assert r.passed


def test_config_checker_tool_frame_null_is_warning():
    results = ConfigChecker().check_all(_make_config(tool_frame=None))
    r = next(r for r in results if r.name == "robot.tool_frame")
    assert r.passed          # not a hard failure
    assert r.warning          # but flagged as a caveat


def test_config_checker_tool_frame_set_no_warning():
    results = ConfigChecker().check_all(_make_config(tool_frame="tcp_frame"))
    r = next(r for r in results if r.name == "robot.tool_frame")
    assert r.passed
    assert not r.warning


# ---------------------------------------------------------------------------
# ConfigChecker – joint names
# ---------------------------------------------------------------------------


def test_config_checker_joint_names_count_matches_dof():
    results = ConfigChecker().check_all(_make_config(dof=6,
                                                     joint_names=["a", "b", "c", "d", "e", "f"]))
    r = next(r for r in results if r.name == "robot.joint_names.count")
    assert r.passed


def test_config_checker_joint_names_count_mismatch_fails():
    results = ConfigChecker().check_all(_make_config(dof=6, joint_names=["j1", "j2", "j3"]))
    r = next(r for r in results if r.name == "robot.joint_names.count")
    assert not r.passed
    assert "3" in r.message
    assert "6" in r.message


def test_config_checker_duplicate_joint_names_fails():
    results = ConfigChecker().check_all(
        _make_config(joint_names=["j1", "j1", "j3", "j4", "j5", "j6"])
    )
    r = next(r for r in results if r.name == "robot.joint_names.unique")
    assert not r.passed


def test_config_checker_unique_joint_names_passes():
    results = ConfigChecker().check_all(_make_config())
    r = next(r for r in results if r.name == "robot.joint_names.unique")
    assert r.passed


# ---------------------------------------------------------------------------
# ConfigChecker – gripper
# ---------------------------------------------------------------------------


def test_config_checker_gripper_open_width_null_warns():
    results = ConfigChecker().check_all(_make_config())   # open_width=None by default
    r = next(r for r in results if r.name == "robot.gripper.open_width")
    assert r.passed          # not a hard failure
    assert r.warning


def test_config_checker_gripper_open_width_set_no_warning():
    config = _make_config()
    config.gripper.open_width = 0.05
    results = ConfigChecker().check_all(config)
    r = next(r for r in results if r.name == "robot.gripper.open_width")
    assert r.passed
    assert not r.warning


def test_config_checker_gripper_positions_reported():
    results = ConfigChecker().check_all(_make_config())
    r = next(r for r in results if r.name == "robot.gripper.positions")
    assert r.passed
    assert "-0.025" in r.message


# ---------------------------------------------------------------------------
# ConfigChecker – all pass with complete config
# ---------------------------------------------------------------------------


def test_config_checker_complete_config_no_failures():
    config = _make_config(tool_frame="tcp_frame")
    config.gripper.open_width = 0.05
    results = ConfigChecker().check_all(config)
    failures = [r for r in results if not r.passed]
    assert failures == [], [r.message for r in failures]


def test_config_checker_complete_config_no_warnings():
    config = _make_config(tool_frame="tcp_frame")
    config.gripper.open_width = 0.05
    results = ConfigChecker().check_all(config)
    warnings = [r for r in results if r.passed and r.warning]
    assert warnings == []


# ---------------------------------------------------------------------------
# CheckResult defaults
# ---------------------------------------------------------------------------


def test_check_result_warning_defaults_false():
    r = CheckResult(name="x", category=CATEGORY_CONFIG, passed=True, message="ok")
    assert r.warning is False


def test_check_result_details_defaults_empty():
    r = CheckResult(name="x", category=CATEGORY_CONFIG, passed=True, message="ok")
    assert r.details == {}


# ---------------------------------------------------------------------------
# PreflightReport properties
# ---------------------------------------------------------------------------


def test_preflight_report_passed_all_pass():
    results = [
        CheckResult(name="a", category=CATEGORY_CONFIG, passed=True, message="ok"),
        CheckResult(name="b", category=CATEGORY_ROS, passed=True, message="ok"),
    ]
    assert PreflightReport(results=results).passed


def test_preflight_report_not_passed_if_any_fail():
    results = [
        CheckResult(name="a", category=CATEGORY_CONFIG, passed=True, message="ok"),
        CheckResult(name="b", category=CATEGORY_ROS, passed=False, message="fail"),
    ]
    assert not PreflightReport(results=results).passed


def test_preflight_report_warnings_do_not_block_passed():
    results = [
        CheckResult(name="a", category=CATEGORY_CONFIG, passed=True, message="ok",
                    warning=True),
    ]
    assert PreflightReport(results=results).passed


def test_preflight_report_failures_list():
    results = [
        CheckResult(name="a", category=CATEGORY_CONFIG, passed=True, message="ok"),
        CheckResult(name="b", category=CATEGORY_ROS, passed=False, message="fail"),
        CheckResult(name="c", category=CATEGORY_MOVEIT, passed=False, message="fail2"),
    ]
    failures = PreflightReport(results=results).failures()
    assert len(failures) == 2
    assert {f.name for f in failures} == {"b", "c"}


def test_preflight_report_warnings_list():
    results = [
        CheckResult(name="a", category=CATEGORY_CONFIG, passed=True,
                    message="ok", warning=True),
        CheckResult(name="b", category=CATEGORY_CONFIG, passed=True, message="ok"),
        CheckResult(name="c", category=CATEGORY_CONFIG, passed=False, message="fail"),
    ]
    warnings = PreflightReport(results=results).warnings()
    assert len(warnings) == 1
    assert warnings[0].name == "a"


def test_preflight_report_by_category():
    results = [
        CheckResult(name="a", category=CATEGORY_CONFIG, passed=True, message="ok"),
        CheckResult(name="b", category=CATEGORY_ROS, passed=True, message="ok"),
        CheckResult(name="c", category=CATEGORY_CONFIG, passed=True, message="ok"),
    ]
    by_cat = PreflightReport(results=results).by_category()
    assert len(by_cat[CATEGORY_CONFIG]) == 2
    assert len(by_cat[CATEGORY_ROS]) == 1


# ---------------------------------------------------------------------------
# format_report
# ---------------------------------------------------------------------------


def test_format_report_all_pass_shows_passed():
    results = [CheckResult(name="robot.model", category=CATEGORY_CONFIG,
                           passed=True, message="ok")]
    text = format_report(PreflightReport(results=results))
    assert "PASSED" in text
    assert "ALL" in text


def test_format_report_failure_shows_fail():
    results = [CheckResult(name="topic:/joint_states", category=CATEGORY_ROS,
                           passed=False, message="not found")]
    text = format_report(PreflightReport(results=results))
    assert "FAIL" in text
    assert "topic:/joint_states" in text


def test_format_report_warning_shows_warn():
    results = [CheckResult(name="robot.gripper.open_width", category=CATEGORY_CONFIG,
                           passed=True, warning=True, message="null")]
    text = format_report(PreflightReport(results=results))
    assert "WARN" in text


def test_format_report_groups_by_category():
    results = [
        CheckResult(name="a", category=CATEGORY_CONFIG, passed=True, message="ok"),
        CheckResult(name="b", category=CATEGORY_MOVEIT, passed=True, message="ok"),
    ]
    text = format_report(PreflightReport(results=results))
    assert "CONFIG" in text
    assert "MOVEIT" in text


def test_format_report_shows_run_dir():
    report = PreflightReport(results=[], run_dir="/runs/preflight_20240101_120000")
    text = format_report(report)
    assert "/runs/preflight_20240101_120000" in text


def test_format_report_failure_section_lists_category():
    results = [CheckResult(name="action:/arm_controller", category=CATEGORY_ROS,
                           passed=False, message="server not found")]
    text = format_report(PreflightReport(results=results))
    assert "ros_bindings" in text
    assert "action:/arm_controller" in text


def test_format_report_warning_count_in_summary():
    results = [
        CheckResult(name="a", category=CATEGORY_CONFIG, passed=True,
                    message="ok", warning=True),
        CheckResult(name="b", category=CATEGORY_CONFIG, passed=True, message="ok"),
    ]
    text = format_report(PreflightReport(results=results))
    assert "1 warning" in text


# ---------------------------------------------------------------------------
# Integration: real config YAML files (skipped if files not present)
# ---------------------------------------------------------------------------


def _config_path(*parts: str) -> str:
    # test file is src/toms_robot/test/ → go up 3 levels to reach repo root
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "..", "..", "config", *parts)


def test_config_checker_cello_yaml_loads_and_warns_open_width():
    """Real cello_follower.yaml has open_width=null → ConfigChecker warns."""
    robot_yaml = _config_path("robot.yaml")
    cello_yaml = _config_path("robots", "cello_follower.yaml")
    if not (os.path.exists(robot_yaml) and os.path.exists(cello_yaml)):
        pytest.skip("config files not found relative to test directory")
    config = RobotConfig.from_yaml_files(robot_yaml, cello_yaml)
    results = ConfigChecker().check_all(config)
    gw = next(r for r in results if r.name == "robot.gripper.open_width")
    assert gw.passed and gw.warning   # open_width is null in cello_follower.yaml


def test_config_checker_cello_yaml_warns_tool_frame():
    """Real cello_follower.yaml has tool_frame=null → ConfigChecker warns."""
    robot_yaml = _config_path("robot.yaml")
    cello_yaml = _config_path("robots", "cello_follower.yaml")
    if not (os.path.exists(robot_yaml) and os.path.exists(cello_yaml)):
        pytest.skip("config files not found relative to test directory")
    config = RobotConfig.from_yaml_files(robot_yaml, cello_yaml)
    results = ConfigChecker().check_all(config)
    tf = next(r for r in results if r.name == "robot.tool_frame")
    assert tf.passed and tf.warning


def test_config_checker_cello_yaml_joint_names_match_dof():
    """Cello has 6 joints and dof=6 → joint_names.count passes."""
    robot_yaml = _config_path("robot.yaml")
    cello_yaml = _config_path("robots", "cello_follower.yaml")
    if not (os.path.exists(robot_yaml) and os.path.exists(cello_yaml)):
        pytest.skip("config files not found relative to test directory")
    config = RobotConfig.from_yaml_files(robot_yaml, cello_yaml)
    results = ConfigChecker().check_all(config)
    r = next(r for r in results if r.name == "robot.joint_names.count")
    assert r.passed
