"""Tests for validation state machines and failure classification."""
from __future__ import annotations

import pytest

from toms_core.models import (
    FailureType,
    ObjectState,
    ObjectStatus,
    Pose,
    Position,
    Quaternion,
    RobotState,
    TaskState,
    WorldState,
)
from toms_validation.grasp_validator import GraspValidationConfig, GraspValidator
from toms_validation.placement_validator import PlacementValidationConfig, PlacementValidator


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_grasp_ws(
    gripper_width: float = 0.012,
    gripper_effort: float = 0.0,
    object_id: str = "pen_0",
    obj_x: float = 0.30,
    ee_x: float = 0.30,
) -> WorldState:
    ws = WorldState()
    ws.robot = RobotState(
        gripper_width=gripper_width,
        gripper_effort=gripper_effort,
        ee_pose=Pose(position=Position(x=ee_x, y=0.0, z=0.15)),
    )
    ws.task = TaskState(task_id="t0")
    ws.task.metadata = {"selected_object_id": object_id}  # type: ignore[attr-defined]
    ws.objects[object_id] = ObjectState(
        object_id=object_id,
        label="pen",
        pose=Pose(position=Position(x=obj_x, y=0.0, z=0.15)),
        status=ObjectStatus.GRASPED,
    )
    return ws


# ---------------------------------------------------------------------------
# GraspValidator tests
# ---------------------------------------------------------------------------


class TestGraspValidator:
    def test_valid_grasp(self):
        cfg = GraspValidationConfig(
            max_empty_gripper_width=0.07,
            min_grasp_gripper_width=0.005,
            vision_proximity_threshold=0.05,
        )
        ws = _make_grasp_ws(gripper_width=0.012, obj_x=0.30, ee_x=0.30)
        v = GraspValidator(cfg)
        result = v.validate(ws)
        assert result.success is True
        assert result.failure_type is None
        assert "gripper_width" in result.signals

    def test_empty_grasp_wide_gripper(self):
        cfg = GraspValidationConfig(max_empty_gripper_width=0.07)
        ws = _make_grasp_ws(gripper_width=0.08)
        result = GraspValidator(cfg).validate(ws)
        assert result.success is False
        assert result.failure_type == FailureType.EMPTY_GRASP

    def test_empty_grasp_fully_closed(self):
        cfg = GraspValidationConfig(
            max_empty_gripper_width=0.07,
            min_grasp_gripper_width=0.005,
        )
        ws = _make_grasp_ws(gripper_width=0.003)
        result = GraspValidator(cfg).validate(ws)
        assert result.success is False
        assert result.failure_type == FailureType.EMPTY_GRASP

    def test_slip_detected_via_effort(self):
        cfg = GraspValidationConfig(
            max_empty_gripper_width=0.07,
            min_grasp_gripper_width=0.005,
            min_grasp_effort=0.3,
            vision_proximity_threshold=0.05,
        )
        ws = _make_grasp_ws(gripper_width=0.012, gripper_effort=0.1)
        result = GraspValidator(cfg).validate(ws)
        assert result.success is False
        assert result.failure_type == FailureType.SLIP

    def test_vision_uncertainty_when_object_far(self):
        cfg = GraspValidationConfig(
            max_empty_gripper_width=0.07,
            min_grasp_gripper_width=0.005,
            vision_proximity_threshold=0.05,
        )
        # Object at x=0.30 but EE at x=0.50 → distance = 0.20 > threshold
        ws = _make_grasp_ws(gripper_width=0.012, obj_x=0.30, ee_x=0.50)
        result = GraspValidator(cfg).validate(ws)
        assert result.success is False
        assert result.failure_type == FailureType.UNCERTAIN

    def test_signals_always_populated(self):
        ws = _make_grasp_ws(gripper_width=0.08)  # empty grasp
        result = GraspValidator().validate(ws)
        assert "gripper_width" in result.signals
        assert "gripper_width_check" in result.signals

    def test_all_failure_types_are_distinct(self):
        assert FailureType.EMPTY_GRASP != FailureType.SLIP
        assert FailureType.SLIP != FailureType.DROPPED
        assert FailureType.DROPPED != FailureType.PLACEMENT_FAILED
        assert FailureType.PLACEMENT_FAILED != FailureType.UNCERTAIN


# ---------------------------------------------------------------------------
# PlacementValidator tests
# ---------------------------------------------------------------------------


def _make_place_ws(
    gripper_width: float = 0.06,
    obj_x: float = 0.40,
    container_x: float = 0.40,
    object_id: str = "pen_0",
) -> WorldState:
    ws = WorldState()
    ws.robot = RobotState(gripper_width=gripper_width)
    ws.objects[object_id] = ObjectState(
        object_id=object_id,
        label="pen",
        pose=Pose(position=Position(x=obj_x, y=0.0, z=0.10)),
        status=ObjectStatus.GRASPED,
    )
    ws.container = ObjectState(
        object_id="holder_0",
        label="pen_holder",
        pose=Pose(position=Position(x=container_x, y=0.0, z=0.05)),
    )
    return ws


class TestPlacementValidator:
    def test_successful_placement(self):
        cfg = PlacementValidationConfig(
            min_released_gripper_width=0.04,
            max_container_distance_xy=0.05,
        )
        ws = _make_place_ws(gripper_width=0.06, obj_x=0.40, container_x=0.40)
        v = PlacementValidator(cfg)
        result = v.validate("pen_0", ws)
        assert result.success is True
        assert ws.objects["pen_0"].status == ObjectStatus.PLACED

    def test_gripper_still_closed_after_release(self):
        cfg = PlacementValidationConfig(min_released_gripper_width=0.04)
        ws = _make_place_ws(gripper_width=0.01)
        result = PlacementValidator(cfg).validate("pen_0", ws)
        assert result.success is False
        assert result.failure_type == FailureType.PLACEMENT_FAILED

    def test_object_outside_holder_zone(self):
        cfg = PlacementValidationConfig(
            min_released_gripper_width=0.04,
            max_container_distance_xy=0.05,
        )
        ws = _make_place_ws(gripper_width=0.06, obj_x=0.30, container_x=0.40)
        result = PlacementValidator(cfg).validate("pen_0", ws)
        assert result.success is False
        assert result.failure_type == FailureType.PLACEMENT_FAILED

    def test_missing_object_in_world_state(self):
        ws = _make_place_ws()
        result = PlacementValidator().validate("nonexistent_pen", ws)
        assert result.success is False
        assert result.failure_type == FailureType.UNCERTAIN

    def test_missing_container(self):
        ws = _make_place_ws()
        ws.container = None
        result = PlacementValidator().validate("pen_0", ws)
        assert result.success is False
        assert result.failure_type == FailureType.UNCERTAIN

    def test_signals_always_populated(self):
        ws = _make_place_ws(gripper_width=0.01)
        result = PlacementValidator().validate("pen_0", ws)
        assert "gripper_width_after_release" in result.signals
        assert "release_check" in result.signals
