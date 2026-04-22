"""UpdateWorldState – refresh world state from perception and robot adapters."""
from __future__ import annotations

from typing import Protocol

from toms_bt.node_base import BTNode, NodeStatus
from toms_core.models import WorldState


class PerceptionAdapter(Protocol):
    """Minimal interface expected from the perception backend."""

    def update_world_state(self, world_state: WorldState) -> None:
        """Merge latest detections into world_state.objects in-place."""
        ...


class RobotAdapter(Protocol):
    """Minimal interface expected from the robot backend."""

    def update_robot_state(self, world_state: WorldState) -> None:
        """Refresh world_state.robot in-place from hardware/sim."""
        ...


class UpdateWorldState(BTNode):
    """Tick perception and robot adapters to refresh the world state.

    Inputs:  world_state (passed to tick)
    Outputs: world_state.objects updated, world_state.robot updated,
             world_state.sequence incremented, world_state.timestamp updated
    Success: both adapters returned without raising
    Failure: either adapter raises PerceptionError or hardware fault
    Retry:   immediate re-tick is safe; upstream RepeatUntil handles budget
    """

    def __init__(
        self,
        name: str = "UpdateWorldState",
        perception: PerceptionAdapter | None = None,
        robot: RobotAdapter | None = None,
    ) -> None:
        super().__init__(name)
        self._perception = perception
        self._robot = robot

    def tick(self, world_state: WorldState) -> NodeStatus:
        import time

        try:
            if self._perception is not None:
                self._perception.update_world_state(world_state)
            if self._robot is not None:
                self._robot.update_robot_state(world_state)
            world_state.sequence += 1
            world_state.timestamp = time.time()
            self._status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS
        except Exception:  # noqa: BLE001
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE
