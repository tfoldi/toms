"""toms_world.world_state_manager – canonical world state storage and mutation.

Rules:
  - Only this module should mutate WorldState fields.
  - All other packages call methods here; they do NOT modify WorldState directly.
  - Thread-safety: acquire self._lock for any mutation in a multi-threaded context.
"""
from __future__ import annotations

import copy
import threading
import time
import uuid
from typing import List, Optional

from toms_core.models import (
    ObjectState,
    ObjectStatus,
    TaskState,
    TaskStatus,
    WorldState,
)


class WorldStateManager:
    """Thread-safe manager for the canonical WorldState.

    Usage::

        manager = WorldStateManager()
        manager.initialize_task(task_id="task_001", objects=detected_objects)
        ws = manager.snapshot()   # immutable copy for decision-making
    """

    def __init__(self) -> None:
        self._state = WorldState()
        self._lock = threading.Lock()
        self._history: List[WorldState] = []  # for replay/debugging
        self._max_history: int = 100

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def snapshot(self) -> WorldState:
        """Return a deep copy of the current world state for read-only use."""
        with self._lock:
            return copy.deepcopy(self._state)

    def current(self) -> WorldState:
        """Return a reference to the live state (mutable; use with care)."""
        return self._state

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def initialize_task(
        self,
        task_id: str,
        retry_budget: int = 3,
        object_ids: Optional[List[str]] = None,
    ) -> None:
        with self._lock:
            self._state.task = TaskState(
                task_id=task_id,
                status=TaskStatus.RUNNING,
                retry_budget=retry_budget,
                objects_pending=list(object_ids or []),
            )
            self._commit()

    # ------------------------------------------------------------------
    # Object updates
    # ------------------------------------------------------------------

    def upsert_object(self, obj: ObjectState) -> None:
        """Add or replace an object in the world state."""
        with self._lock:
            self._state.objects[obj.object_id] = obj
            self._commit()

    def remove_object(self, object_id: str) -> None:
        with self._lock:
            self._state.objects.pop(object_id, None)
            self._commit()

    def update_object_status(self, object_id: str, status: ObjectStatus) -> None:
        with self._lock:
            obj = self._state.objects.get(object_id)
            if obj is not None:
                obj.status = status
                obj.last_seen = time.time()
            self._commit()

    def set_container(self, obj: Optional[ObjectState]) -> None:
        with self._lock:
            self._state.container = obj
            self._commit()

    # ------------------------------------------------------------------
    # Task bookkeeping
    # ------------------------------------------------------------------

    def mark_object_complete(self, object_id: str) -> None:
        with self._lock:
            task = self._state.task
            if task is None:
                return
            if object_id in task.objects_pending:
                task.objects_pending.remove(object_id)
            if object_id not in task.objects_complete:
                task.objects_complete.append(object_id)
            self._commit()

    def mark_object_failed(self, object_id: str) -> None:
        with self._lock:
            task = self._state.task
            if task is None:
                return
            if object_id in task.objects_pending:
                task.objects_pending.remove(object_id)
            if object_id not in task.objects_failed:
                task.objects_failed.append(object_id)
            self._commit()

    def record_retry(self) -> None:
        with self._lock:
            if self._state.task is not None:
                self._state.task.retries_used += 1
            self._commit()

    def finalize_task(self, success: bool) -> None:
        with self._lock:
            if self._state.task is not None:
                self._state.task.status = (
                    TaskStatus.SUCCESS if success else TaskStatus.FAILED
                )
                self._state.task.end_time = time.time()
            self._commit()

    # ------------------------------------------------------------------
    # History / replay
    # ------------------------------------------------------------------

    def _commit(self) -> None:
        """Snapshot current state into history (call while holding lock)."""
        self._state.sequence += 1
        self._state.timestamp = time.time()
        if len(self._history) >= self._max_history:
            self._history.pop(0)
        self._history.append(copy.deepcopy(self._state))

    def history(self) -> List[WorldState]:
        """Return a copy of the snapshot history for replay/debugging."""
        with self._lock:
            return list(self._history)
