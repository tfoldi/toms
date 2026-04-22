"""ReportOutcome – emit a structured TaskOutcome at end of task run."""
from __future__ import annotations

import time
from typing import Optional, Protocol

from toms_bt.node_base import BTNode, NodeStatus
from toms_core.models import TaskOutcome, TaskStatus, WorldState


class LoggerAdapter(Protocol):
    def log_task_outcome(self, outcome: TaskOutcome) -> None:
        """Persist / publish the final task outcome."""
        ...


class ReportOutcome(BTNode):
    """Build a TaskOutcome from world state and hand it to the logger.

    Inputs:  world_state.task
    Outputs: logs task outcome via logger adapter
    Success: always (reporting cannot block task completion)
    Failure: never
    """

    def __init__(
        self,
        name: str = "ReportOutcome",
        logger: Optional[LoggerAdapter] = None,
    ) -> None:
        super().__init__(name)
        self._logger = logger

    def tick(self, world_state: WorldState) -> NodeStatus:
        task = world_state.task
        if task is None:
            self._status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        duration = (
            time.time() - task.start_time
            if task.end_time is None
            else task.end_time - task.start_time
        )

        outcome = TaskOutcome(
            task_id=task.task_id,
            success=task.status == TaskStatus.SUCCESS,
            objects_placed=list(task.objects_complete),
            objects_failed=list(task.objects_failed),
            total_attempts=task.retries_used + len(task.objects_complete),
            duration_sec=duration,
        )

        if self._logger is not None:
            self._logger.log_task_outcome(outcome)

        self._status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS
