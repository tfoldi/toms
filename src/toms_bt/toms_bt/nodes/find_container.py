"""FindContainer – locate the pen holder in the world state."""
from __future__ import annotations

from toms_core.models import ObjectStatus, WorldState

from toms_bt.node_base import BTNode, NodeStatus


class FindContainer(BTNode):
    """Locate the pen holder object and set world_state.container.

    Inputs:  world_state.objects (populated by UpdateWorldState)
             container_label – label string to search for (default: "pen_holder")
    Outputs: world_state.container set to the detected holder ObjectState
    Success: at least one object with container_label found and is DETECTED
    Failure: no holder found → FailureType.CONTAINER_NOT_FOUND
    Retry:   safe to re-tick after UpdateWorldState refreshes the scene
    """

    def __init__(
        self,
        name: str = "FindContainer",
        container_label: str = "pen_holder",
    ) -> None:
        super().__init__(name)
        self._container_label = container_label

    def tick(self, world_state: WorldState) -> NodeStatus:
        candidates = [
            obj
            for obj in world_state.objects.values()
            if obj.label == self._container_label
            and obj.status != ObjectStatus.FAILED
        ]
        if not candidates:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        # Pick highest-confidence detection if multiple exist.
        best = max(candidates, key=lambda o: o.detection_confidence)
        world_state.container = best
        self._status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS
