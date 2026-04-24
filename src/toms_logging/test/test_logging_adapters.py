"""Tests for logging adapter wrappers.

Pure-Python; no rclpy or ROS2 required.
"""
from __future__ import annotations

import json
import tempfile
from pathlib import Path
from unittest.mock import MagicMock

from toms_core.models import (
    GraspCandidate,
    GraspStatus,
    ObjectState,
    ObjectStatus,
    PlanRequest,
    PlanResult,
    Pose,
    Position,
    RobotState,
    TaskState,
    ValidationResult,
    WorldState,
)
from toms_logging.logging_adapters import (
    LoggingExecutor,
    LoggingGraspGenerator,
    LoggingGraspValidator,
    LoggingPlacementValidator,
    LoggingPlanner,
)
from toms_logging.run_manager import RunLogger

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _make_logger(run_dir: str) -> RunLogger:
    return RunLogger(run_dir, "test_task", also_print=False)


def _make_world_state() -> WorldState:
    ws = WorldState(task=TaskState(task_id="t0"))
    ws.robot = RobotState(gripper_width=0.05)
    ws.objects["pen_0"] = ObjectState(
        object_id="pen_0", label="pen", status=ObjectStatus.GRASPED,
        detection_confidence=0.9,
        pose=Pose(position=Position(x=0.3, y=0.0, z=0.02)),
    )
    return ws


def _make_plan_request() -> PlanRequest:
    candidate = GraspCandidate(
        candidate_id="g0", object_id="pen_0",
        pose=Pose(), score=0.9, status=GraspStatus.FEASIBLE,
    )
    return PlanRequest(
        object_id="pen_0",
        grasp_candidate=candidate,
        planning_group="arm",
        end_effector_link="link6",
        base_frame="base_link",
    )


def _read_events(run_dir: str) -> list:
    p = Path(run_dir) / "events.jsonl"
    if not p.exists():
        return []
    return [json.loads(line) for line in p.read_text().splitlines() if line.strip()]


# ---------------------------------------------------------------------------
# LoggingPlanner
# ---------------------------------------------------------------------------


def test_logging_planner_delegates_to_inner():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.plan.return_value = PlanResult(success=True, planning_time=0.1)
        log = _make_logger(run_dir)
        wrapper = LoggingPlanner(inner, log)
        req = _make_plan_request()
        result = wrapper.plan(req)
        inner.plan.assert_called_once_with(req)
        assert result.success is True
        log.close()


def test_logging_planner_emits_plan_result_event():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.plan.return_value = PlanResult(success=True, planning_time=0.05)
        log = _make_logger(run_dir)
        wrapper = LoggingPlanner(inner, log)
        wrapper.plan(_make_plan_request())
        log.close()
        events = _read_events(run_dir)
        assert any(e["event"] == "plan_result" for e in events)


def test_logging_planner_emits_failure_result():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.plan.return_value = PlanResult(
            success=False, error_code="IK_FAILED", error_message="No IK solution"
        )
        log = _make_logger(run_dir)
        wrapper = LoggingPlanner(inner, log)
        result = wrapper.plan(_make_plan_request())
        log.close()
        assert result.success is False
        events = _read_events(run_dir)
        plan_ev = [e for e in events if e["event"] == "plan_result"][0]
        assert plan_ev["success"] is False


def test_logging_planner_returns_inner_result_unchanged():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        expected = PlanResult(success=True, trajectory={"steps": 5}, planning_time=0.3)
        inner.plan.return_value = expected
        log = _make_logger(run_dir)
        result = LoggingPlanner(inner, log).plan(_make_plan_request())
        log.close()
        assert result.trajectory == {"steps": 5}


# ---------------------------------------------------------------------------
# LoggingGraspValidator
# ---------------------------------------------------------------------------


def test_logging_grasp_validator_delegates():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.validate.return_value = ValidationResult(success=True)
        log = _make_logger(run_dir)
        ws = _make_world_state()
        result = LoggingGraspValidator(inner, log).validate(ws)
        inner.validate.assert_called_once_with(ws)
        assert result.success is True
        log.close()


def test_logging_grasp_validator_emits_validation_event():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.validate.return_value = ValidationResult(
            success=False,
            failure_type=None,
            signals={"gripper_width": 0.072},
            message="Too wide",
        )
        log = _make_logger(run_dir)
        LoggingGraspValidator(inner, log).validate(_make_world_state())
        log.close()
        events = _read_events(run_dir)
        val_ev = [e for e in events if e["event"] == "validation_result"][0]
        assert val_ev["phase"] == "grasp"
        assert val_ev["success"] is False


def test_logging_grasp_validator_phase_is_grasp():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.validate.return_value = ValidationResult(success=True)
        log = _make_logger(run_dir)
        LoggingGraspValidator(inner, log).validate(_make_world_state())
        log.close()
        events = _read_events(run_dir)
        val_ev = [e for e in events if e["event"] == "validation_result"][0]
        assert val_ev["phase"] == "grasp"


# ---------------------------------------------------------------------------
# LoggingPlacementValidator
# ---------------------------------------------------------------------------


def test_logging_placement_validator_delegates():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.validate.return_value = ValidationResult(success=True)
        log = _make_logger(run_dir)
        ws = _make_world_state()
        result = LoggingPlacementValidator(inner, log).validate("pen_0", ws)
        inner.validate.assert_called_once_with("pen_0", ws)
        assert result.success is True
        log.close()


def test_logging_placement_validator_phase_is_placement():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.validate.return_value = ValidationResult(success=True)
        log = _make_logger(run_dir)
        LoggingPlacementValidator(inner, log).validate("pen_0", _make_world_state())
        log.close()
        events = _read_events(run_dir)
        val_ev = [e for e in events if e["event"] == "validation_result"][0]
        assert val_ev["phase"] == "placement"


# ---------------------------------------------------------------------------
# LoggingExecutor
# ---------------------------------------------------------------------------


def test_logging_executor_execute_grasp_delegates():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.execute_grasp.return_value = True
        log = _make_logger(run_dir)
        ws = _make_world_state()
        plan = PlanResult(success=True)
        result = LoggingExecutor(inner, log).execute_grasp(plan, ws)
        inner.execute_grasp.assert_called_once_with(plan, ws)
        assert result is True
        log.close()


def test_logging_executor_execute_place_delegates():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.execute_place.return_value = True
        log = _make_logger(run_dir)
        ws = _make_world_state()
        plan = PlanResult(success=True)
        result = LoggingExecutor(inner, log).execute_place(plan, ws)
        inner.execute_place.assert_called_once_with(plan, ws)
        assert result is True
        log.close()


def test_logging_executor_grasp_emits_execution_step():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.execute_grasp.return_value = True
        log = _make_logger(run_dir)
        LoggingExecutor(inner, log).execute_grasp(PlanResult(success=True), _make_world_state())
        log.close()
        events = _read_events(run_dir)
        exec_ev = [e for e in events if e["event"] == "execution_step"][0]
        assert exec_ev["step"] == "grasp"
        assert exec_ev["success"] is True


def test_logging_executor_place_emits_execution_step():
    with tempfile.TemporaryDirectory() as run_dir:
        inner = MagicMock()
        inner.execute_place.return_value = False
        log = _make_logger(run_dir)
        LoggingExecutor(inner, log).execute_place(PlanResult(success=True), _make_world_state())
        log.close()
        events = _read_events(run_dir)
        exec_ev = [e for e in events if e["event"] == "execution_step"][0]
        assert exec_ev["step"] == "place"
        assert exec_ev["success"] is False


# ---------------------------------------------------------------------------
# LoggingGraspGenerator
# ---------------------------------------------------------------------------


def test_logging_grasp_generator_delegates():
    with tempfile.TemporaryDirectory() as run_dir:
        candidates = [
            GraspCandidate(candidate_id="g0", object_id="pen_0",
                           status=GraspStatus.CANDIDATE)
        ]
        inner = MagicMock()
        inner.generate.return_value = candidates
        log = _make_logger(run_dir)
        ws = _make_world_state()
        result = LoggingGraspGenerator(inner, log).generate("pen_0", ws)
        inner.generate.assert_called_once_with("pen_0", ws)
        assert result == candidates
        log.close()


def test_logging_grasp_generator_emits_candidates_event():
    with tempfile.TemporaryDirectory() as run_dir:
        candidates = [
            GraspCandidate(candidate_id="g0", object_id="pen_0",
                           status=GraspStatus.CANDIDATE)
        ]
        inner = MagicMock()
        inner.generate.return_value = candidates
        log = _make_logger(run_dir)
        LoggingGraspGenerator(inner, log).generate("pen_0", _make_world_state())
        log.close()
        events = _read_events(run_dir)
        assert any(e["event"] == "grasp_candidates" for e in events)
