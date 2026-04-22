"""Tests for toms_core typed data models."""


from toms_core.models import (
    FailureType,
    GraspCandidate,
    GraspStatus,
    ObjectState,
    ObjectStatus,
    TaskOutcome,
    TaskState,
    TaskStatus,
    ValidationResult,
    WorldState,
)

# ---------------------------------------------------------------------------
# ObjectState
# ---------------------------------------------------------------------------


def test_object_state_defaults():
    obj = ObjectState(object_id="pen_0", label="pen")
    assert obj.status == ObjectStatus.UNKNOWN
    assert obj.grasp_attempts == 0
    assert obj.detection_confidence == 0.0
    assert isinstance(obj.last_seen, float)


def test_object_state_status_transitions():
    obj = ObjectState(object_id="pen_0", label="pen")
    obj.status = ObjectStatus.DETECTED
    assert obj.status == ObjectStatus.DETECTED
    obj.status = ObjectStatus.SELECTED
    assert obj.status == ObjectStatus.SELECTED
    obj.status = ObjectStatus.GRASPED
    assert obj.status == ObjectStatus.GRASPED
    obj.status = ObjectStatus.PLACED
    assert obj.status == ObjectStatus.PLACED


def test_object_state_grasp_attempts_increment():
    obj = ObjectState(object_id="pen_1", label="pen")
    obj.grasp_attempts += 1
    assert obj.grasp_attempts == 1
    obj.grasp_attempts += 1
    assert obj.grasp_attempts == 2


# ---------------------------------------------------------------------------
# TaskState
# ---------------------------------------------------------------------------


def test_task_state_defaults():
    task = TaskState(task_id="task_001")
    assert task.status == TaskStatus.IDLE
    assert task.retry_budget == 3
    assert task.retries_used == 0
    assert task.objects_pending == []
    assert task.objects_complete == []
    assert task.objects_failed == []


def test_task_state_retry_budget_exhaustion():
    task = TaskState(task_id="task_002", retry_budget=2)
    task.retries_used = 1
    assert task.retries_used < task.retry_budget
    task.retries_used = 2
    assert task.retries_used >= task.retry_budget


def test_task_state_object_movement():
    task = TaskState(task_id="task_003")
    task.objects_pending = ["pen_0", "pen_1"]
    # simulate completing pen_0
    task.objects_pending.remove("pen_0")
    task.objects_complete.append("pen_0")
    assert "pen_0" not in task.objects_pending
    assert "pen_0" in task.objects_complete
    assert "pen_1" in task.objects_pending


def test_task_state_object_to_failed():
    task = TaskState(task_id="task_004")
    task.objects_pending = ["pen_0"]
    task.objects_pending.remove("pen_0")
    task.objects_failed.append("pen_0")
    assert task.objects_failed == ["pen_0"]
    assert task.objects_pending == []


# ---------------------------------------------------------------------------
# WorldState
# ---------------------------------------------------------------------------


def test_world_state_defaults():
    ws = WorldState()
    assert ws.objects == {}
    assert ws.container is None
    assert ws.task is None
    assert ws.sequence == 0


def test_world_state_add_object():
    ws = WorldState()
    pen = ObjectState(object_id="pen_0", label="pen")
    ws.objects["pen_0"] = pen
    assert "pen_0" in ws.objects
    assert ws.objects["pen_0"].label == "pen"


def test_world_state_sequence_increments():
    ws = WorldState()
    ws.sequence += 1
    ws.sequence += 1
    assert ws.sequence == 2


def test_world_state_container_assignment():
    ws = WorldState()
    holder = ObjectState(object_id="holder_0", label="pen_holder")
    ws.container = holder
    assert ws.container is not None
    assert ws.container.label == "pen_holder"


# ---------------------------------------------------------------------------
# GraspCandidate
# ---------------------------------------------------------------------------


def test_grasp_candidate_defaults():
    cand = GraspCandidate(candidate_id="g0", object_id="pen_0")
    assert cand.status == GraspStatus.CANDIDATE
    assert cand.is_feasible is None
    assert cand.ik_solution is None
    assert cand.score == 0.0


def test_grasp_candidate_feasibility_update():
    cand = GraspCandidate(candidate_id="g0", object_id="pen_0")
    cand.is_feasible = True
    cand.ik_solution = [0.0] * 6
    cand.status = GraspStatus.FEASIBLE
    assert cand.status == GraspStatus.FEASIBLE
    assert len(cand.ik_solution) == 6


# ---------------------------------------------------------------------------
# ValidationResult
# ---------------------------------------------------------------------------


def test_validation_result_success():
    result = ValidationResult(success=True, confidence=0.95)
    assert result.success is True
    assert result.failure_type is None


def test_validation_result_failure_types():
    for ft in FailureType:
        result = ValidationResult(success=False, failure_type=ft)
        assert result.failure_type == ft
        assert result.success is False


def test_validation_result_signals():
    result = ValidationResult(
        success=False,
        failure_type=FailureType.EMPTY_GRASP,
        signals={"gripper_width": 0.08, "expected_width": 0.01},
    )
    assert result.signals["gripper_width"] == 0.08


# ---------------------------------------------------------------------------
# TaskOutcome
# ---------------------------------------------------------------------------


def test_task_outcome():
    outcome = TaskOutcome(
        task_id="task_001",
        success=True,
        objects_placed=["pen_0", "pen_1"],
        total_attempts=3,
        duration_sec=42.5,
    )
    assert outcome.success is True
    assert len(outcome.objects_placed) == 2
    assert outcome.objects_failed == []
