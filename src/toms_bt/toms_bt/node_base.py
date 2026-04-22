"""toms_bt.node_base – base classes for all behavior tree nodes.

Design principles:
  - Nodes are pure Python; no ROS2 spin inside a tick().
  - All I/O goes through WorldState and injected adapter interfaces.
  - Every node documents its inputs, outputs, success condition, and failure types.
  - Retry logic lives in the tree topology (RepeatUntil), NOT inside leaf nodes.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from typing import List, Optional

from toms_core.models import WorldState


class NodeStatus(str, Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"


class BTNode(ABC):
    """Abstract base for a single behavior tree node."""

    def __init__(self, name: str) -> None:
        self.name = name
        self._status: NodeStatus = NodeStatus.RUNNING

    @property
    def status(self) -> NodeStatus:
        return self._status

    @abstractmethod
    def tick(self, world_state: WorldState) -> NodeStatus:
        """Execute one tick.

        Must not block; call ROS2 services/actions through non-blocking
        adapter methods that return immediately with RUNNING while in flight.

        Returns:
            NodeStatus.SUCCESS  – node goal achieved.
            NodeStatus.FAILURE  – node goal cannot be achieved now.
            NodeStatus.RUNNING  – still in progress; re-tick next cycle.
        """

    def reset(self) -> None:
        """Reset internal state so the node can be re-ticked from scratch."""
        self._status = NodeStatus.RUNNING

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.name!r})"


# ---------------------------------------------------------------------------
# Composite nodes
# ---------------------------------------------------------------------------


class Sequence(BTNode):
    """Ticks children left-to-right; succeeds when all succeed.

    Stops and returns FAILURE as soon as any child fails (AND semantics).
    """

    def __init__(self, name: str, children: List[BTNode]) -> None:
        super().__init__(name)
        self.children = children
        self._current_index: int = 0

    def tick(self, world_state: WorldState) -> NodeStatus:
        while self._current_index < len(self.children):
            child = self.children[self._current_index]
            status = child.tick(world_state)
            if status == NodeStatus.RUNNING:
                self._status = NodeStatus.RUNNING
                return NodeStatus.RUNNING
            if status == NodeStatus.FAILURE:
                self._status = NodeStatus.FAILURE
                return NodeStatus.FAILURE
            # SUCCESS → advance
            self._current_index += 1
        self._current_index = 0
        self._status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS

    def reset(self) -> None:
        super().reset()
        self._current_index = 0
        for child in self.children:
            child.reset()


class Selector(BTNode):
    """Ticks children left-to-right; succeeds when any child succeeds.

    Returns FAILURE only when all children have failed (OR semantics).
    """

    def __init__(self, name: str, children: List[BTNode]) -> None:
        super().__init__(name)
        self.children = children
        self._current_index: int = 0

    def tick(self, world_state: WorldState) -> NodeStatus:
        while self._current_index < len(self.children):
            child = self.children[self._current_index]
            status = child.tick(world_state)
            if status == NodeStatus.RUNNING:
                self._status = NodeStatus.RUNNING
                return NodeStatus.RUNNING
            if status == NodeStatus.SUCCESS:
                self._current_index = 0
                self._status = NodeStatus.SUCCESS
                return NodeStatus.SUCCESS
            # FAILURE → try next
            self._current_index += 1
        self._current_index = 0
        self._status = NodeStatus.FAILURE
        return NodeStatus.FAILURE

    def reset(self) -> None:
        super().reset()
        self._current_index = 0
        for child in self.children:
            child.reset()


class RepeatUntilFailure(BTNode):
    """Repeat child until it returns FAILURE or the retry budget is exhausted.

    This is the outer retry loop for the pen-to-holder task.  The retry
    budget is tracked in TaskState, not here, so the node checks the world
    state on each tick.
    """

    def __init__(self, name: str, child: BTNode, max_iterations: int = 100) -> None:
        super().__init__(name)
        self.child = child
        self.max_iterations = max_iterations
        self._iterations: int = 0

    def tick(self, world_state: WorldState) -> NodeStatus:
        if self._iterations >= self.max_iterations:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        status = self.child.tick(world_state)
        self._iterations += 1

        if status == NodeStatus.FAILURE:
            self._status = NodeStatus.SUCCESS  # loop ended cleanly
            return NodeStatus.SUCCESS
        if status == NodeStatus.RUNNING:
            self._status = NodeStatus.RUNNING
            return NodeStatus.RUNNING
        # child SUCCESS → keep looping
        self.child.reset()
        self._status = NodeStatus.RUNNING
        return NodeStatus.RUNNING

    def reset(self) -> None:
        super().reset()
        self._iterations = 0
        self.child.reset()


class ParallelAll(BTNode):
    """Tick all children each cycle; succeed when all succeed.

    Fails immediately if any child fails.
    """

    def __init__(self, name: str, children: List[BTNode]) -> None:
        super().__init__(name)
        self.children = children

    def tick(self, world_state: WorldState) -> NodeStatus:
        all_success = True
        for child in self.children:
            status = child.tick(world_state)
            if status == NodeStatus.FAILURE:
                self._status = NodeStatus.FAILURE
                return NodeStatus.FAILURE
            if status == NodeStatus.RUNNING:
                all_success = False
        if all_success:
            self._status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS
        self._status = NodeStatus.RUNNING
        return NodeStatus.RUNNING
