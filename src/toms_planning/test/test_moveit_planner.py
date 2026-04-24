"""Tests for MoveIt2RosPlanner construction and request building.

Pure-Python; no rclpy or moveit_msgs imports.
"""
from __future__ import annotations

import pytest
from toms_core.models import GraspCandidate, GraspStatus, PlanRequest, PlanResult
from toms_planning.moveit_wrapper import MoveIt2RosPlanner, PlanningConfig
from toms_robot.config_loader import RobotConfig

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

CELLO_PARAMS: dict = {
    "robot.model": "cello_description",
    "robot.variant": "cello",
    "robot.dof": 6,
    "robot.planning_group": "arm",
    "robot.gripper_group": "hand",
    "robot.base_frame": "base_link",
    "robot.end_effector_link": "link6",
    "robot.tool_frame": None,
    "robot.joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "robot.joint_state_topic": "/joint_states",
    "robot.trajectory_action": "/arm_controller/follow_joint_trajectory",
    "robot.gripper.interface_type": "gripper_action",
    "robot.gripper.action": "/hand_controller/gripper_cmd",
    "robot.gripper.joint_name": "joint7_left",
    "robot.gripper.open_position": -0.025,
    "robot.gripper.close_position": 0.0,
    "robot.gripper.max_force": 15.0,
    "robot.gripper.open_width": None,
    "robot.gripper.close_width": 0.0,
    "robot.kinematics.plugin": "kdl_kinematics_plugin/KDLKinematicsPlugin",
    "robot.kinematics.search_resolution": 0.005,
    "robot.kinematics.timeout": 0.005,
    "robot.moveit_package": "cello_moveit_config",
    "robot.urdf_package": "cello_description",
    "robot.urdf_file": "urdf/cello_description.urdf",
    "robot.srdf_package": "cello_moveit_config",
    "robot.srdf_file": "config/cello_description.srdf",
}

PLANNING_PARAMS: dict = {
    "planning.moveit2.planner_id": "",
    "planning.moveit2.planning_time": 5.0,
    "planning.moveit2.max_planning_attempts": 3,
    "planning.moveit2.goal_position_tolerance": 0.005,
    "planning.moveit2.goal_orientation_tolerance": 0.01,
}


def cello_config() -> RobotConfig:
    return RobotConfig.from_dict(CELLO_PARAMS)


def cello_planning() -> PlanningConfig:
    return PlanningConfig.from_dict(PLANNING_PARAMS)


def make_plan_request() -> PlanRequest:
    candidate = GraspCandidate(
        candidate_id="g0",
        object_id="pen_0",
        status=GraspStatus.FEASIBLE,
    )
    return PlanRequest(
        object_id="pen_0",
        grasp_candidate=candidate,
        planning_group="arm",
        end_effector_link="link6",
        base_frame="base_link",
    )


# ---------------------------------------------------------------------------
# PlanningConfig
# ---------------------------------------------------------------------------


def test_planning_config_defaults():
    pc = PlanningConfig()
    assert pc.planner_id == ""
    assert pc.planning_time == pytest.approx(5.0)
    assert pc.max_attempts == 3


def test_planning_config_from_dict():
    pc = cello_planning()
    assert pc.planner_id == ""
    assert pc.planning_time == pytest.approx(5.0)
    assert pc.max_attempts == 3
    assert pc.goal_position_tolerance == pytest.approx(0.005)
    assert pc.goal_orientation_tolerance == pytest.approx(0.01)


def test_planning_config_planner_id_empty_string():
    """planner_id="" is intentional: MoveIt2 uses the default OMPL planner."""
    pc = cello_planning()
    assert pc.planner_id == ""
    assert pc.planner_id is not None
    # Must not be a TODO sentinel
    assert "TODO" not in pc.planner_id


# ---------------------------------------------------------------------------
# MoveIt2RosPlanner – construction (dry-run, no node)
# ---------------------------------------------------------------------------


def test_planner_constructs_dry_run():
    cfg = cello_config()
    planner = MoveIt2RosPlanner(cfg, node=None)
    assert planner is not None


def test_planner_plan_without_node_returns_failure():
    cfg = cello_config()
    planner = MoveIt2RosPlanner(cfg, node=None)
    result = planner.plan(make_plan_request())
    assert isinstance(result, PlanResult)
    assert result.success is False
    assert "NO_NODE" in result.error_code


def test_planner_plan_without_node_error_message():
    cfg = cello_config()
    planner = MoveIt2RosPlanner(cfg, node=None)
    result = planner.plan(make_plan_request())
    assert result.error_message is not None
    assert len(result.error_message) > 0


# ---------------------------------------------------------------------------
# MoveIt2RosPlanner – planning_params() returns config-driven values
# ---------------------------------------------------------------------------


def test_planning_params_uses_config_planning_group():
    cfg = cello_config()
    planner = MoveIt2RosPlanner(cfg, node=None)
    params = planner.planning_params()
    assert params["planning_group"] == "arm"


def test_planning_params_uses_config_base_frame():
    cfg = cello_config()
    planner = MoveIt2RosPlanner(cfg, node=None)
    params = planner.planning_params()
    assert params["base_frame"] == "base_link"


def test_planning_params_uses_config_ee_link():
    cfg = cello_config()
    planner = MoveIt2RosPlanner(cfg, node=None)
    params = planner.planning_params()
    assert params["end_effector_link"] == "link6"


def test_planning_params_tool_frame_fallback():
    """tool_frame=null in config → effective_tool_frame falls back to link6."""
    cfg = cello_config()
    assert cfg.tool_frame is None
    planner = MoveIt2RosPlanner(cfg, node=None)
    params = planner.planning_params()
    assert params["tool_frame"] == "link6"


def test_planning_params_planner_id_from_planning_config():
    cfg = cello_config()
    pc = cello_planning()
    planner = MoveIt2RosPlanner(cfg, planning=pc, node=None)
    params = planner.planning_params()
    assert params["planner_id"] == ""
    assert "TODO" not in params["planner_id"]


def test_planning_params_tolerances():
    cfg = cello_config()
    pc = cello_planning()
    planner = MoveIt2RosPlanner(cfg, planning=pc, node=None)
    params = planner.planning_params()
    assert params["goal_position_tolerance"] == pytest.approx(0.005)
    assert params["goal_orientation_tolerance"] == pytest.approx(0.01)


def test_planning_params_not_hardcoded():
    """Changing config values changes what the planner would use."""
    import copy
    p = copy.deepcopy(CELLO_PARAMS)
    p["robot.planning_group"] = "custom_arm"
    p["robot.base_frame"] = "custom_base"
    cfg = RobotConfig.from_dict(p)
    planner = MoveIt2RosPlanner(cfg, node=None)
    params = planner.planning_params()
    assert params["planning_group"] == "custom_arm"
    assert params["base_frame"] == "custom_base"


# ---------------------------------------------------------------------------
# MockMotionPlanner (regression – still works after moveit_wrapper rewrite)
# ---------------------------------------------------------------------------


def test_mock_planner_succeeds():
    from toms_planning.moveit_wrapper import MockMotionPlanner
    planner = MockMotionPlanner(always_succeed=True)
    result = planner.plan(make_plan_request())
    assert result.success is True
    assert result.trajectory is not None


def test_mock_planner_can_fail():
    from toms_planning.moveit_wrapper import MockMotionPlanner
    planner = MockMotionPlanner(always_succeed=False)
    result = planner.plan(make_plan_request())
    assert result.success is False
    assert result.error_code == "MOCK_FAILURE"
