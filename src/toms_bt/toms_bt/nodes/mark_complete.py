"""MarkObjectComplete – bookkeeping after a successful place."""
from __future__ import annotations

from toms_bt.node_base import BTNode, NodeStatus
from toms_bt.nodes.select_next_object import SELECTED_OBJECT_KEY
from toms_core.models import ObjectStatus, WorldState


class MarkObjectComplete(BTNode):
    """Move the placed object from objects_pending to objects_complete.

    Inputs:  world_state.task.metadata[SELECTED_OBJECT_KEY]
             object must have status == PLACED
    Outputs: task.objects_pending shortened, task.objects_complete extended
    Success: always (bookkeeping only; downstream ReportOutcome summarises)
    Failure: never raised – if task is None we silently succeed to avoid
             masking earlier logic errors
    Retry:   idempotent
    """

    def __init__(self, name: str = "MarkObjectComplete") -> None:
        super().__init__(name)

    def tick(self, world_state: WorldState) -> NodeStatus:
        task = world_state.task
        if task is None:
            self._status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        object_id = getattr(task, "metadata", {}).get(SELECTED_OBJECT_KEY)
        if object_id and object_id in task.objects_pending:
            task.objects_pending.remove(object_id)
            if object_id not in task.objects_complete:
                task.objects_complete.append(object_id)

        # Clear selection so next tick of SelectNextObject starts fresh.
        if hasattr(task, "metadata") and SELECTED_OBJECT_KEY in task.metadata:  # type: ignore[attr-defined]
            del task.metadata[SELECTED_OBJECT_KEY]  # type: ignore[attr-defined]

        self._status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS
