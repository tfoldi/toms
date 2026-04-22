"""Tests for WorldStateManager state transitions."""
from __future__ import annotations

import pytest

from toms_core.models import ObjectState, ObjectStatus, TaskStatus, WorldState
from toms_world.world_state_manager import WorldStateManager


def make_manager() -> WorldStateManager:
    m = WorldStateManager()
    m.initialize_task("task_001", retry_budget=3, object_ids=["pen_0", "pen_1"])
    return m


# ---------------------------------------------------------------------------
# Initialization
# ---------------------------------------------------------------------------


def test_initialize_task_creates_task_state():
    m = make_manager()
    ws = m.snapshot()
    assert ws.task is not None
    assert ws.task.task_id == "task_001"
    assert ws.task.status == TaskStatus.RUNNING
    assert set(ws.task.objects_pending) == {"pen_0", "pen_1"}
    assert ws.task.objects_complete == []
    assert ws.task.objects_failed == []


def test_initialize_task_sets_retry_budget():
    m = make_manager()
    assert m.snapshot().task.retry_budget == 3


# ---------------------------------------------------------------------------
# Object upsert / remove
# ---------------------------------------------------------------------------


def test_upsert_adds_object():
    m = make_manager()
    obj = ObjectState(object_id="pen_0", label="pen", status=ObjectStatus.DETECTED)
    m.upsert_object(obj)
    ws = m.snapshot()
    assert "pen_0" in ws.objects
    assert ws.objects["pen_0"].label == "pen"


def test_upsert_replaces_object():
    m = make_manager()
    m.upsert_object(ObjectState(object_id="pen_0", label="pen", detection_confidence=0.5))
    m.upsert_object(ObjectState(object_id="pen_0", label="pen", detection_confidence=0.9))
    assert m.snapshot().objects["pen_0"].detection_confidence == 0.9


def test_remove_object():
    m = make_manager()
    m.upsert_object(ObjectState(object_id="pen_0", label="pen"))
    m.remove_object("pen_0")
    assert "pen_0" not in m.snapshot().objects


def test_remove_nonexistent_object_is_safe():
    m = make_manager()
    m.remove_object("does_not_exist")  # should not raise


# ---------------------------------------------------------------------------
# Object status update
# ---------------------------------------------------------------------------


def test_update_object_status():
    m = make_manager()
    m.upsert_object(ObjectState(object_id="pen_0", label="pen", status=ObjectStatus.DETECTED))
    m.update_object_status("pen_0", ObjectStatus.GRASPED)
    assert m.snapshot().objects["pen_0"].status == ObjectStatus.GRASPED


def test_update_status_missing_object_is_safe():
    m = make_manager()
    m.update_object_status("ghost", ObjectStatus.PLACED)  # should not raise


# ---------------------------------------------------------------------------
# Task bookkeeping
# ---------------------------------------------------------------------------


def test_mark_object_complete():
    m = make_manager()
    m.mark_object_complete("pen_0")
    ws = m.snapshot()
    assert "pen_0" not in ws.task.objects_pending
    assert "pen_0" in ws.task.objects_complete


def test_mark_object_complete_idempotent():
    m = make_manager()
    m.mark_object_complete("pen_0")
    m.mark_object_complete("pen_0")
    assert m.snapshot().task.objects_complete.count("pen_0") == 1


def test_mark_object_failed():
    m = make_manager()
    m.mark_object_failed("pen_1")
    ws = m.snapshot()
    assert "pen_1" not in ws.task.objects_pending
    assert "pen_1" in ws.task.objects_failed


def test_record_retry_increments_counter():
    m = make_manager()
    m.record_retry()
    m.record_retry()
    assert m.snapshot().task.retries_used == 2


def test_finalize_task_success():
    m = make_manager()
    m.finalize_task(success=True)
    ws = m.snapshot()
    assert ws.task.status == TaskStatus.SUCCESS
    assert ws.task.end_time is not None


def test_finalize_task_failure():
    m = make_manager()
    m.finalize_task(success=False)
    assert m.snapshot().task.status == TaskStatus.FAILED


# ---------------------------------------------------------------------------
# Sequence counter and history
# ---------------------------------------------------------------------------


def test_sequence_increments_on_each_mutation():
    m = make_manager()
    seq0 = m.snapshot().sequence
    m.upsert_object(ObjectState(object_id="pen_0", label="pen"))
    seq1 = m.snapshot().sequence
    m.mark_object_complete("pen_0")
    seq2 = m.snapshot().sequence
    assert seq1 > seq0
    assert seq2 > seq1


def test_history_grows_with_mutations():
    m = make_manager()
    initial_len = len(m.history())
    m.upsert_object(ObjectState(object_id="pen_0", label="pen"))
    m.upsert_object(ObjectState(object_id="pen_1", label="pen"))
    assert len(m.history()) == initial_len + 2


def test_snapshot_is_independent_copy():
    m = make_manager()
    m.upsert_object(ObjectState(object_id="pen_0", label="pen", detection_confidence=0.5))
    ws = m.snapshot()
    # Mutate the live state
    m.update_object_status("pen_0", ObjectStatus.GRASPED)
    # Snapshot should be unaffected
    assert ws.objects["pen_0"].status == ObjectStatus.UNKNOWN
