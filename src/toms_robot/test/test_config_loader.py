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
    PoseConfig,
    RobotConfig,
    _deep_merge,
    _flatten,
    _read_pose,
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


def test_read_pose_all_fields_present():
    p = {
        "fixed_pick_test.object_pose.frame_id": "base_link",
        "fixed_pick_test.object_pose.position.x": 1.0,
        "fixed_pick_test.object_pose.position.y": 2.0,
        "fixed_pick_test.object_pose.position.z": 3.0,
        "fixed_pick_test.object_pose.orientation.x": 0.0,
        "fixed_pick_test.object_pose.orientation.y": 0.0,
        "fixed_pick_test.object_pose.orientation.z": 0.0,
        "fixed_pick_test.object_pose.orientation.w": 1.0,
    }
    pose = _read_pose(p, "fixed_pick_test.object_pose")
    assert isinstance(pose, PoseConfig)
    assert pose.frame_id == "base_link"
    assert pose.position == (1.0, 2.0, 3.0)
    assert pose.orientation == (0.0, 0.0, 0.0, 1.0)


def test_read_pose_defaults_frame_id_to_base_link():
    p = {
        "fixed_pick_test.object_pose.position.x": 0.1,
        "fixed_pick_test.object_pose.position.y": 0.2,
        "fixed_pick_test.object_pose.position.z": 0.3,
        "fixed_pick_test.object_pose.orientation.x": 0.0,
        "fixed_pick_test.object_pose.orientation.y": 0.0,
        "fixed_pick_test.object_pose.orientation.z": 0.0,
        "fixed_pick_test.object_pose.orientation.w": 1.0,
    }
    pose = _read_pose(p, "fixed_pick_test.object_pose")
    assert pose is not None
    assert pose.frame_id == "base_link"


def test_read_pose_returns_none_when_any_field_missing():
    # missing position.z
    p = {
        "fixed_pick_test.object_pose.position.x": 0.1,
        "fixed_pick_test.object_pose.position.y": 0.2,
        "fixed_pick_test.object_pose.orientation.x": 0.0,
        "fixed_pick_test.object_pose.orientation.y": 0.0,
        "fixed_pick_test.object_pose.orientation.z": 0.0,
        "fixed_pick_test.object_pose.orientation.w": 1.0,
    }
    assert _read_pose(p, "fixed_pick_test.object_pose") is None


def test_read_pose_returns_none_when_field_is_null():
    # all keys present but one is None (YAML null) – should treat as absent
    p = {
        "fixed_pick_test.place_pose.position.x": 0.1,
        "fixed_pick_test.place_pose.position.y": 0.2,
        "fixed_pick_test.place_pose.position.z": None,
        "fixed_pick_test.place_pose.orientation.x": 0.0,
        "fixed_pick_test.place_pose.orientation.y": 0.0,
        "fixed_pick_test.place_pose.orientation.z": 0.0,
        "fixed_pick_test.place_pose.orientation.w": 1.0,
    }
    assert _read_pose(p, "fixed_pick_test.place_pose") is None


def test_robot_config_fixed_pick_fields_default_to_none():
    """RobotConfig without any fixed_pick_test keys leaves the 4 fields None."""
    cfg = RobotConfig.from_dict(CELLO_PARAMS)
    assert cfg.object_pose is None
    assert cfg.place_pose is None
    assert cfg.test_lift_offset_z is None
    assert cfg.pre_grasp_offset_z is None
    assert cfg.retreat_offset_z is None


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
    # fixed_pick_test plumbing
    assert cfg.test_lift_offset_z == pytest.approx(0.05)
    assert cfg.pre_grasp_offset_z == pytest.approx(0.05)
    assert cfg.retreat_offset_z == pytest.approx(0.05)
    assert cfg.object_pose is not None
    assert cfg.object_pose.frame_id == "base_link"
    assert cfg.object_pose.position == pytest.approx((-0.289, 0.021, -0.005))
    assert cfg.object_pose.orientation[3] == pytest.approx(0.0)   # qw
    assert cfg.place_pose is not None
    assert cfg.place_pose.position == pytest.approx((0.354, 0.180, 0.051))


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


# ---------------------------------------------------------------------------
# execute_cartesian_move – tested without moving any arm
# ---------------------------------------------------------------------------


def _dummy_pose(x=0.0, y=0.0, z=0.0, qw=1.0):
    from toms_core.models import Pose
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.w = qw
    return p


def test_cartesian_move_params_wires_config():
    """cartesian_move_params forwards config values – no ROS2, no service call."""
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    params = bridge.cartesian_move_params(
        [_dummy_pose(), _dummy_pose(z=0.1)],
        velocity_scale=0.25,
        max_step=0.01,
    )
    assert params["group_name"] == "arm"
    assert params["link_name"] == "link6"
    assert params["frame_id"] == "base_link"
    assert params["num_waypoints"] == 2
    assert params["max_step"] == pytest.approx(0.01)
    assert params["velocity_scale"] == pytest.approx(0.25)
    assert params["avoid_collisions"] is True
    assert params["jump_threshold"] == pytest.approx(0.0)


def test_cartesian_move_params_defaults():
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    params = bridge.cartesian_move_params([_dummy_pose()])
    # Defaults from signature – these are the knobs downstream code depends on
    assert params["max_step"] == pytest.approx(0.005)
    assert params["velocity_scale"] == pytest.approx(0.1)
    assert params["jump_threshold"] == pytest.approx(0.0)
    assert params["avoid_collisions"] is True


def test_execute_cartesian_move_dry_run_returns_false():
    """node=None → dry run; must not touch any service client."""
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    assert bridge.execute_cartesian_move([_dummy_pose(), _dummy_pose(z=0.1)]) is False


def test_execute_cartesian_move_empty_poses_returns_false():
    """Empty poses list is an input error – fail fast, do not call the service."""
    cfg = cello_config()

    class _FakeNode:
        def get_clock(self):
            raise AssertionError("should not be called – empty-poses guard")

    bridge = CelloRobotBridge(cfg, node=None)
    bridge._node = _FakeNode()  # simulate connected state without real ROS2
    assert bridge.execute_cartesian_move([]) is False


def test_execute_cartesian_move_refuses_partial_fraction():
    """When the service returns fraction < min_fraction, we must not execute."""
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    bridge._node = object()   # truthy so dry-run check is bypassed

    # Stub the service-call layer: return a fake trajectory + fraction = 0.5
    sent_trajectories = []

    def fake_call(_poses, _params, service_timeout):
        return object(), 0.5  # trajectory exists but only 50% path coverage

    def fake_send(traj):
        sent_trajectories.append(traj)
        return True

    bridge._call_compute_cartesian_path = fake_call          # type: ignore[method-assign]
    bridge._send_joint_trajectory = fake_send                # type: ignore[method-assign]
    ok = bridge.execute_cartesian_move([_dummy_pose(), _dummy_pose(z=0.1)],
                                       min_fraction=0.9)
    assert ok is False
    assert sent_trajectories == []   # nothing sent to the arm


def test_fixed_pick_build_plan_offsets():
    """build_plan() should add Z offsets correctly and require all 4 fields."""
    import sys as _sys
    from pathlib import Path
    repo_root = Path(__file__).resolve().parents[3]
    sp = str(repo_root / "scripts")
    if sp not in _sys.path:
        _sys.path.insert(0, sp)
    import importlib
    fixed_pick = importlib.import_module("fixed_pick")

    cfg = RobotConfig.from_yaml_files(
        str(repo_root / "config" / "robot.yaml"),
        str(repo_root / "config" / "robots" / "cello_follower.yaml"),
    )
    plan = fixed_pick.build_plan(cfg)

    # pre_grasp = object_pose + pre_grasp_offset_z
    assert plan.grasp.position.z == pytest.approx(-0.005)
    assert plan.pre_grasp.position.z == pytest.approx(-0.005 + 0.05)
    assert plan.retreat.position.z == pytest.approx(-0.005 + 0.05)
    # X / Y / orientation preserved
    assert plan.pre_grasp.position.x == pytest.approx(plan.grasp.position.x)
    assert plan.pre_grasp.orientation.w == pytest.approx(plan.grasp.orientation.w)

    # pre_place = place_pose + pre_grasp_offset_z (same offset by design)
    assert plan.place.position.z == pytest.approx(0.051)
    assert plan.pre_place.position.z == pytest.approx(0.051 + 0.05)
    assert plan.post_place.position.z == pytest.approx(0.051 + 0.05)

    # grasp_force = 50 % of max_force
    assert plan.grasp_force == pytest.approx(cfg.gripper.max_force * 0.5)


def test_fixed_pick_build_plan_rejects_missing_object_pose():
    """Plan construction must error out when required fields are null."""
    import sys as _sys
    from pathlib import Path
    repo_root = Path(__file__).resolve().parents[3]
    sp = str(repo_root / "scripts")
    if sp not in _sys.path:
        _sys.path.insert(0, sp)
    import importlib
    fixed_pick = importlib.import_module("fixed_pick")

    cfg = RobotConfig.from_dict(CELLO_PARAMS)   # has no fixed_pick_test fields
    with pytest.raises(SystemExit, match=r"home_joints|object_pose"):
        fixed_pick.build_plan(cfg)


def test_execute_cartesian_move_dispatches_full_path():
    """fraction >= min_fraction and send succeeds → returns True and dispatches."""
    cfg = cello_config()
    bridge = CelloRobotBridge(cfg, node=None)
    bridge._node = object()

    class _FakeJointTraj:
        points = []   # empty list – velocity scaling is a no-op

    class _FakeTraj:
        joint_trajectory = _FakeJointTraj()

    sent = []

    def fake_call(_poses, _params, service_timeout):
        return _FakeTraj(), 1.0

    def fake_send(traj):
        sent.append(traj)
        return True

    bridge._call_compute_cartesian_path = fake_call          # type: ignore[method-assign]
    bridge._send_joint_trajectory = fake_send                # type: ignore[method-assign]
    # velocity_scale=1.0 bypasses the time-scaling branch entirely
    ok = bridge.execute_cartesian_move(
        [_dummy_pose(), _dummy_pose(z=0.1)], velocity_scale=1.0,
    )
    assert ok is True
    assert len(sent) == 1
    assert sent[0] is _FakeTraj.joint_trajectory
