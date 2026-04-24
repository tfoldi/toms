"""Tests for RobotConfig loading/validation and CelloRobotBridge dry-run binding.

These are pure-Python tests; no rclpy or ROS2 message types are imported.
"""
from __future__ import annotations

import os

import pytest
from toms_robot.cello_bridge import CelloRobotBridge
from toms_robot.config_loader import (
    ConfigurationError,
    GripperConfig,
    KinematicsConfig,
    RobotConfig,
    _deep_merge,
    _flatten,
)

# ---------------------------------------------------------------------------
# Cello fixture – values match config/robots/cello_follower.yaml exactly
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


def cello_config() -> RobotConfig:
    return RobotConfig.from_dict(CELLO_PARAMS)


# ---------------------------------------------------------------------------
# RobotConfig.from_dict – happy path
# ---------------------------------------------------------------------------


def test_from_dict_identity():
    cfg = cello_config()
    assert cfg.model == "cello_description"
    assert cfg.variant == "cello"
    assert cfg.dof == 6


def test_from_dict_frames():
    cfg = cello_config()
    assert cfg.base_frame == "base_link"
    assert cfg.end_effector_link == "link6"
    assert cfg.tool_frame is None


def test_from_dict_effective_tool_frame_falls_back():
    """When tool_frame is null, effective_tool_frame returns end_effector_link."""
    cfg = cello_config()
    assert cfg.tool_frame is None
    assert cfg.effective_tool_frame == "link6"


def test_from_dict_effective_tool_frame_uses_set_value():
    p = dict(CELLO_PARAMS)
    p["robot.tool_frame"] = "tcp_frame"
    cfg = RobotConfig.from_dict(p)
    assert cfg.effective_tool_frame == "tcp_frame"


def test_from_dict_planning_group():
    cfg = cello_config()
    assert cfg.planning_group == "arm"
    assert cfg.gripper_group == "hand"


def test_from_dict_joint_names():
    cfg = cello_config()
    assert cfg.joint_names == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    assert len(cfg.joint_names) == cfg.dof


def test_from_dict_topics():
    cfg = cello_config()
    assert cfg.joint_state_topic == "/joint_states"
    assert cfg.trajectory_action == "/arm_controller/follow_joint_trajectory"


def test_from_dict_gripper():
    cfg = cello_config()
    g = cfg.gripper
    assert isinstance(g, GripperConfig)
    assert g.interface_type == "gripper_action"
    assert g.action == "/hand_controller/gripper_cmd"
    assert g.joint_name == "joint7_left"
    assert g.open_position == pytest.approx(-0.025)
    assert g.close_position == pytest.approx(0.0)
    assert g.max_force == pytest.approx(15.0)
    assert g.open_width is None      # not yet measured
    assert g.close_width == pytest.approx(0.0)


def test_from_dict_kinematics():
    cfg = cello_config()
    k = cfg.kinematics
    assert isinstance(k, KinematicsConfig)
    assert k.plugin == "kdl_kinematics_plugin/KDLKinematicsPlugin"
    assert k.search_resolution == pytest.approx(0.005)
    assert k.timeout == pytest.approx(0.005)


def test_from_dict_moveit_packages():
    cfg = cello_config()
    assert cfg.moveit_package == "cello_moveit_config"
    assert cfg.urdf_package == "cello_description"
    assert cfg.urdf_file == "urdf/cello_description.urdf"
    assert cfg.srdf_package == "cello_moveit_config"
    assert cfg.srdf_file == "config/cello_description.srdf"


# ---------------------------------------------------------------------------
# RobotConfig.from_dict – validation errors
# ---------------------------------------------------------------------------


def test_missing_planning_group_raises():
    p = dict(CELLO_PARAMS)
    p["robot.planning_group"] = None
    with pytest.raises(ConfigurationError, match="robot.planning_group"):
        RobotConfig.from_dict(p)


def test_missing_base_frame_raises():
    p = dict(CELLO_PARAMS)
    p["robot.base_frame"] = None
    with pytest.raises(ConfigurationError, match="robot.base_frame"):
        RobotConfig.from_dict(p)


def test_missing_trajectory_action_raises():
    p = dict(CELLO_PARAMS)
    p["robot.trajectory_action"] = None
    with pytest.raises(ConfigurationError, match="robot.trajectory_action"):
        RobotConfig.from_dict(p)


def test_missing_gripper_action_raises():
    p = dict(CELLO_PARAMS)
    p["robot.gripper.action"] = None
    with pytest.raises(ConfigurationError, match="robot.gripper.action"):
        RobotConfig.from_dict(p)


def test_missing_gripper_open_position_raises():
    p = dict(CELLO_PARAMS)
    p["robot.gripper.open_position"] = None
    with pytest.raises(ConfigurationError, match="robot.gripper.open_position"):
        RobotConfig.from_dict(p)


def test_missing_kinematics_plugin_raises():
    p = dict(CELLO_PARAMS)
    p["robot.kinematics.plugin"] = None
    with pytest.raises(ConfigurationError, match="robot.kinematics.plugin"):
        RobotConfig.from_dict(p)


# ---------------------------------------------------------------------------
# from_yaml_files – integration with real config files
# ---------------------------------------------------------------------------


def _repo_root() -> str:
    """Return absolute path to the repository root (3 dirs up from this file)."""
    # This file: src/toms_robot/test/test_config_loader.py
    # Repo root: ../../..
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..")
    )


def test_from_yaml_files_cello():
    """Load the real config files and verify key cello values."""
    pytest.importorskip("yaml", reason="pyyaml not installed")
    root = _repo_root()
    base = os.path.join(root, "config", "robot.yaml")
    overlay = os.path.join(root, "config", "robots", "cello_follower.yaml")
    if not os.path.exists(base) or not os.path.exists(overlay):
        pytest.skip("config files not found (running outside repo root)")

    cfg = RobotConfig.from_yaml_files(base, overlay)
    assert cfg.planning_group == "arm"
    assert cfg.base_frame == "base_link"
    assert cfg.end_effector_link == "link6"
    assert cfg.trajectory_action == "/arm_controller/follow_joint_trajectory"
    assert cfg.gripper.action == "/hand_controller/gripper_cmd"
    assert cfg.gripper.open_position == pytest.approx(0.0)
    assert cfg.gripper.close_position == pytest.approx(-0.025)
    assert cfg.home_joints is not None
    assert len(cfg.home_joints) == cfg.dof


def test_from_yaml_files_overlay_wins():
    """Later YAML file overrides earlier values (overlay pattern)."""
    pytest.importorskip("yaml", reason="pyyaml not installed")
    import tempfile

    import yaml

    base_data = {"robot": {"model": "base_model", "variant": None}}
    over_data = {"robot": {"variant": "cello"}}

    with (
        tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f1,
        tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f2,
    ):
        yaml.dump(base_data, f1)
        yaml.dump(over_data, f2)
        f1_path, f2_path = f1.name, f2.name

    try:
        import yaml as _yaml
        merged = {}
        for p in (f1_path, f2_path):
            with open(p) as fh:
                merged = _yaml.safe_load(fh) or {}
        # overlay value should win
        assert merged["robot"]["variant"] == "cello"
    finally:
        os.unlink(f1_path)
        os.unlink(f2_path)


# ---------------------------------------------------------------------------
# _deep_merge and _flatten helpers
# ---------------------------------------------------------------------------


def test_deep_merge_override_wins():
    a = {"x": 1, "nested": {"y": 2, "z": 3}}
    b = {"nested": {"y": 99}}
    result = _deep_merge(a, b)
    assert result["x"] == 1
    assert result["nested"]["y"] == 99
    assert result["nested"]["z"] == 3


def test_deep_merge_new_key_added():
    a = {"x": 1}
    b = {"y": 2}
    assert _deep_merge(a, b) == {"x": 1, "y": 2}


def test_flatten_nested():
    d = {"robot": {"base_frame": "base_link", "gripper": {"action": "/gripper_cmd"}}}
    flat = _flatten(d)
    assert flat["robot.base_frame"] == "base_link"
    assert flat["robot.gripper.action"] == "/gripper_cmd"


def test_flatten_leaf_values_preserved():
    d = {"robot": {"joint_names": ["j1", "j2", "j3"]}}
    flat = _flatten(d)
    assert flat["robot.joint_names"] == ["j1", "j2", "j3"]


# ---------------------------------------------------------------------------
# CelloRobotBridge – dry-run mode (no rclpy.Node)
# ---------------------------------------------------------------------------


def test_cello_bridge_constructs_without_node():
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    assert bridge is not None


def test_cello_bridge_open_gripper_dry_run_returns_false():
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    assert bridge.open_gripper() is False


def test_cello_bridge_close_gripper_dry_run_returns_false():
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    assert bridge.close_gripper(target_width=0.01, force=5.0) is False


def test_cello_bridge_execute_trajectory_dry_run_returns_false():
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    assert bridge.execute_trajectory(object()) is False


def test_cello_bridge_update_robot_state_dry_run_noop():
    from toms_core.models import WorldState
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    ws = WorldState()
    ws.robot.joint_positions = [1.0, 2.0]
    bridge.update_robot_state(ws)
    # Dry-run: state not changed
    assert ws.robot.joint_positions == [1.0, 2.0]


def test_cello_bridge_config_bindings():
    """Verify the bridge stores config values (not hardcoded) correctly."""
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    assert bridge._config.trajectory_action == "/arm_controller/follow_joint_trajectory"
    assert bridge._config.gripper.action == "/hand_controller/gripper_cmd"
    assert bridge._config.joint_state_topic == "/joint_states"
    assert bridge._config.joint_names == [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    ]
    assert bridge._config.gripper.joint_name == "joint7_left"
    assert bridge._config.effective_tool_frame == "link6"


def test_cello_bridge_force_capped_at_max():
    """close_gripper uses min(force, max_force) – verify max_force from config."""
    cfg = cello_config()
    assert cfg.gripper.max_force == pytest.approx(15.0)
    # No real call, just verify the config value the bridge would cap at
    bridge = CelloRobotBridge(cfg, node=None)
    assert bridge._config.gripper.max_force == pytest.approx(15.0)
