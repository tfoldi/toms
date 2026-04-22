"""Tests for BT node status propagation and retry budget handling."""
from __future__ import annotations

from typing import List
from unittest.mock import MagicMock

import pytest

from toms_bt.node_base import (
    BTNode,
    NodeStatus,
    ParallelAll,
    RepeatUntilFailure,
    Selector,
    Sequence,
)
from toms_bt.nodes.find_container import FindContainer
from toms_bt.nodes.mark_complete import MarkObjectComplete
from toms_bt.nodes.select_next_object import SELECTED_OBJECT_KEY, SelectNextObject
from toms_bt.nodes.update_world_state import UpdateWorldState
from toms_core.models import (
    ObjectState,
    ObjectStatus,
    TaskState,
    TaskStatus,
    WorldState,
)


# ---------------------------------------------------------------------------
# Helper leaf nodes
# ---------------------------------------------------------------------------


class AlwaysSuccess(BTNode):
    def tick(self, world_state: WorldState) -> NodeStatus:
        return NodeStatus.SUCCESS


class AlwaysFailure(BTNode):
    def tick(self, world_state: WorldState) -> NodeStatus:
        return NodeStatus.FAILURE


class AlwaysRunning(BTNode):
    def tick(self, world_state: WorldState) -> NodeStatus:
        return NodeStatus.RUNNING


class CountingNode(BTNode):
    """Returns SUCCESS after N ticks."""

    def __init__(self, name: str, succeed_after: int) -> None:
        super().__init__(name)
        self.ticks = 0
        self.succeed_after = succeed_after

    def tick(self, ws: WorldState) -> NodeStatus:
        self.ticks += 1
        return NodeStatus.SUCCESS if self.ticks >= self.succeed_after else NodeStatus.RUNNING


# ---------------------------------------------------------------------------
# Sequence tests
# ---------------------------------------------------------------------------


def _ws() -> WorldState:
    return WorldState()


def test_sequence_all_success():
    seq = Sequence("seq", [AlwaysSuccess("a"), AlwaysSuccess("b")])
    assert seq.tick(_ws()) == NodeStatus.SUCCESS


def test_sequence_first_fails():
    seq = Sequence("seq", [AlwaysFailure("a"), AlwaysSuccess("b")])
    assert seq.tick(_ws()) == NodeStatus.FAILURE


def test_sequence_stops_at_first_failure():
    executed = []

    class Track(BTNode):
        def __init__(self, n, result):
            super().__init__(n)
            self._result = result

        def tick(self, ws):
            executed.append(self.name)
            return self._result

    seq = Sequence("seq", [Track("a", NodeStatus.FAILURE), Track("b", NodeStatus.SUCCESS)])
    seq.tick(_ws())
    assert executed == ["a"]


def test_sequence_running_pauses():
    seq = Sequence("seq", [AlwaysSuccess("a"), AlwaysRunning("b"), AlwaysSuccess("c")])
    assert seq.tick(_ws()) == NodeStatus.RUNNING


def test_sequence_resumes_after_running():
    node_b = CountingNode("b", succeed_after=2)
    seq = Sequence("seq", [AlwaysSuccess("a"), node_b, AlwaysSuccess("c")])
    assert seq.tick(_ws()) == NodeStatus.RUNNING  # b needs 2 ticks
    assert seq.tick(_ws()) == NodeStatus.SUCCESS


# ---------------------------------------------------------------------------
# Selector tests
# ---------------------------------------------------------------------------


def test_selector_first_succeeds():
    sel = Selector("sel", [AlwaysSuccess("a"), AlwaysFailure("b")])
    assert sel.tick(_ws()) == NodeStatus.SUCCESS


def test_selector_all_fail():
    sel = Selector("sel", [AlwaysFailure("a"), AlwaysFailure("b")])
    assert sel.tick(_ws()) == NodeStatus.FAILURE


def test_selector_skips_to_second():
    sel = Selector("sel", [AlwaysFailure("a"), AlwaysSuccess("b")])
    assert sel.tick(_ws()) == NodeStatus.SUCCESS


# ---------------------------------------------------------------------------
# RepeatUntilFailure tests
# ---------------------------------------------------------------------------


def test_repeat_until_failure_exits_on_child_failure():
    loop = RepeatUntilFailure("loop", child=AlwaysFailure("inner"))
    # First tick: inner fails → loop returns SUCCESS (loop ended cleanly)
    assert loop.tick(_ws()) == NodeStatus.SUCCESS


def test_repeat_until_failure_loops_on_success():
    count = [0]

    class CountThenFail(BTNode):
        def tick(self, ws):
            count[0] += 1
            return NodeStatus.FAILURE if count[0] >= 3 else NodeStatus.SUCCESS

    loop = RepeatUntilFailure("loop", child=CountThenFail("inner"), max_iterations=10)
    # ticks 1 and 2: child SUCCESS → loop running
    assert loop.tick(_ws()) == NodeStatus.RUNNING
    assert loop.tick(_ws()) == NodeStatus.RUNNING
    # tick 3: child FAILURE → loop returns SUCCESS
    assert loop.tick(_ws()) == NodeStatus.SUCCESS
    assert count[0] == 3


def test_repeat_until_failure_max_iterations():
    loop = RepeatUntilFailure("loop", child=AlwaysSuccess("inner"), max_iterations=3)
    loop.tick(_ws())  # iter 1 → RUNNING
    loop.tick(_ws())  # iter 2 → RUNNING
    loop.tick(_ws())  # iter 3 → RUNNING (hits max next tick)
    assert loop.tick(_ws()) == NodeStatus.FAILURE  # budget exhausted


# ---------------------------------------------------------------------------
# UpdateWorldState node
# ---------------------------------------------------------------------------


def test_update_world_state_no_adapters():
    node = UpdateWorldState()
    ws = WorldState()
    ws.sequence = 0
    result = node.tick(ws)
    assert result == NodeStatus.SUCCESS
    assert ws.sequence == 1


def test_update_world_state_adapter_called():
    mock_perception = MagicMock()
    mock_robot = MagicMock()
    node = UpdateWorldState(perception=mock_perception, robot=mock_robot)
    ws = WorldState()
    node.tick(ws)
    mock_perception.update_world_state.assert_called_once_with(ws)
    mock_robot.update_robot_state.assert_called_once_with(ws)


def test_update_world_state_adapter_raises():
    mock_perception = MagicMock()
    mock_perception.update_world_state.side_effect = RuntimeError("sensor down")
    node = UpdateWorldState(perception=mock_perception)
    result = node.tick(WorldState())
    assert result == NodeStatus.FAILURE


# ---------------------------------------------------------------------------
# FindContainer node
# ---------------------------------------------------------------------------


def test_find_container_found():
    ws = WorldState()
    holder = ObjectState(
        object_id="holder_0",
        label="pen_holder",
        status=ObjectStatus.DETECTED,
        detection_confidence=0.9,
    )
    ws.objects["holder_0"] = holder
    node = FindContainer()
    assert node.tick(ws) == NodeStatus.SUCCESS
    assert ws.container is holder


def test_find_container_not_found():
    ws = WorldState()
    node = FindContainer()
    assert node.tick(ws) == NodeStatus.FAILURE
    assert ws.container is None


def test_find_container_picks_highest_confidence():
    ws = WorldState()
    ws.objects["h0"] = ObjectState(
        object_id="h0", label="pen_holder", status=ObjectStatus.DETECTED,
        detection_confidence=0.5,
    )
    ws.objects["h1"] = ObjectState(
        object_id="h1", label="pen_holder", status=ObjectStatus.DETECTED,
        detection_confidence=0.95,
    )
    node = FindContainer()
    node.tick(ws)
    assert ws.container is not None
    assert ws.container.object_id == "h1"


# ---------------------------------------------------------------------------
# SelectNextObject node
# ---------------------------------------------------------------------------


def _ws_with_pens(*statuses: ObjectStatus) -> WorldState:
    ws = WorldState()
    ws.task = TaskState(task_id="t0")
    for i, status in enumerate(statuses):
        oid = f"pen_{i}"
        ws.objects[oid] = ObjectState(
            object_id=oid,
            label="pen",
            status=status,
            detection_confidence=float(i + 1) / len(statuses),
        )
        ws.task.objects_pending.append(oid)
    return ws


def test_select_next_object_picks_detected():
    ws = _ws_with_pens(ObjectStatus.DETECTED, ObjectStatus.DETECTED)
    node = SelectNextObject()
    assert node.tick(ws) == NodeStatus.SUCCESS
    selected_id = ws.task.metadata[SELECTED_OBJECT_KEY]  # type: ignore[attr-defined]
    assert selected_id in ["pen_0", "pen_1"]
    assert ws.objects[selected_id].status == ObjectStatus.SELECTED


def test_select_next_object_no_pending():
    ws = WorldState()
    ws.task = TaskState(task_id="t0")
    node = SelectNextObject()
    assert node.tick(ws) == NodeStatus.FAILURE


def test_select_next_object_skips_failed():
    ws = _ws_with_pens(ObjectStatus.FAILED, ObjectStatus.DETECTED)
    # pen_0 is FAILED, so not eligible
    ws.objects["pen_0"].status = ObjectStatus.FAILED
    node = SelectNextObject()
    result = node.tick(ws)
    assert result == NodeStatus.SUCCESS
    assert ws.task.metadata[SELECTED_OBJECT_KEY] == "pen_1"  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# MarkObjectComplete node
# ---------------------------------------------------------------------------


def test_mark_object_complete_moves_to_complete():
    ws = WorldState()
    ws.task = TaskState(task_id="t0")
    ws.task.objects_pending = ["pen_0"]
    ws.task.metadata = {SELECTED_OBJECT_KEY: "pen_0"}  # type: ignore[attr-defined]
    ws.objects["pen_0"] = ObjectState(object_id="pen_0", label="pen", status=ObjectStatus.PLACED)

    node = MarkObjectComplete()
    assert node.tick(ws) == NodeStatus.SUCCESS
    assert "pen_0" not in ws.task.objects_pending
    assert "pen_0" in ws.task.objects_complete
    assert SELECTED_OBJECT_KEY not in ws.task.metadata  # type: ignore[attr-defined]


def test_mark_object_complete_idempotent():
    ws = WorldState()
    ws.task = TaskState(task_id="t0")
    ws.task.objects_pending = []
    ws.task.objects_complete = ["pen_0"]
    ws.task.metadata = {}  # type: ignore[attr-defined]

    node = MarkObjectComplete()
    assert node.tick(ws) == NodeStatus.SUCCESS
    assert ws.task.objects_complete == ["pen_0"]


# ---------------------------------------------------------------------------
# Retry budget integration
# ---------------------------------------------------------------------------


def test_retry_budget_tracking():
    """Simulate exhausting the retry budget via TaskState."""
    task = TaskState(task_id="t0", retry_budget=2)
    task.objects_pending = ["pen_0"]

    # Simulate two failed grasp attempts
    task.retries_used += 1
    assert task.retries_used < task.retry_budget
    task.retries_used += 1
    assert task.retries_used >= task.retry_budget

    # At this point, the BT outer loop should stop re-selecting pen_0
    task.objects_pending.remove("pen_0")
    task.objects_failed.append("pen_0")
    assert task.objects_pending == []
    assert task.objects_failed == ["pen_0"]
