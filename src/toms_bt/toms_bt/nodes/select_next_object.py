"""SelectNextObject – choose which pen to attempt next."""
from __future__ import annotations

from toms_core.models import ObjectState, ObjectStatus, TaskState, WorldState

from toms_bt.node_base import BTNode, NodeStatus

# Key used to store the currently selected object in WorldState.task metadata.
SELECTED_OBJECT_KEY = "selected_object_id"


class SelectNextObject(BTNode):
    """Pick the next reachable, pending pen from the world state.

    Selection policy (v1): highest detection_confidence among DETECTED pens
    whose object_id is still in task.objects_pending.

    Inputs:  world_state.objects, world_state.task.objects_pending
    Outputs: world_state.task metadata["selected_object_id"] set
             selected object status → SELECTED
    Success: a pending, reachable object was found and marked
    Failure: no more pending objects → outer RepeatUntil ends the loop
    Retry:   safe; re-selection after a failed grasp is expected
    """

    def __init__(
        self,
        name: str = "SelectNextObject",
        object_label: str = "pen",
    ) -> None:
        super().__init__(name)
        self._object_label = object_label

    def tick(self, world_state: WorldState) -> NodeStatus:
        task: TaskState | None = world_state.task
        if task is None or not task.objects_pending:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        candidates: list[ObjectState] = [
            obj
            for obj in world_state.objects.values()
            if obj.object_id in task.objects_pending
            and obj.label == self._object_label
            and obj.status == ObjectStatus.DETECTED
        ]

        if not candidates:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        chosen = max(candidates, key=lambda o: o.detection_confidence)
        chosen.status = ObjectStatus.SELECTED
        task.metadata = getattr(task, "metadata", {})  # type: ignore[attr-defined]
        # Store selection in task metadata so downstream nodes can find it.
        task.metadata[SELECTED_OBJECT_KEY] = chosen.object_id  # type: ignore[attr-defined]

        self._status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS
