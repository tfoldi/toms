"""toms_logging.task_logger – structured logging and replay support.

Logs:
  - world state snapshots
  - selected objects
  - grasp candidates
  - plans
  - execution steps
  - validation results
  - failures and retries

Output format: JSONL (one JSON object per line) for easy grep/replay.
TODO: add ROS2 bag integration for full sensor replay.
"""
from __future__ import annotations

import dataclasses
import json
import logging
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from toms_core.models import (
    GraspCandidate,
    PlanResult,
    TaskOutcome,
    ValidationResult,
    WorldState,
)


def _to_dict(obj: Any) -> Any:
    """Recursively convert dataclasses / enums to JSON-serialisable dicts."""
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return {k: _to_dict(v) for k, v in dataclasses.asdict(obj).items()}
    if hasattr(obj, "value"):  # Enum
        return obj.value
    if isinstance(obj, (list, tuple)):
        return [_to_dict(i) for i in obj]
    if isinstance(obj, dict):
        return {k: _to_dict(v) for k, v in obj.items()}
    return obj


class TomsLogger:
    """Structured event logger with replay support.

    Usage::

        logger = TomsLogger(log_dir="/tmp/toms_logs", task_id="task_001")
        logger.log_world_state(world_state)
        logger.log_validation_result("grasp", result)
        logger.log_task_outcome(outcome)
    """

    def __init__(
        self,
        log_dir: str = "/tmp/toms_logs",
        task_id: str = "unknown",
        also_print: bool = True,
    ) -> None:
        self._task_id = task_id
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)

        log_file = self._log_dir / f"{task_id}_{int(time.time())}.jsonl"
        self._file = open(log_file, "a", encoding="utf-8")

        self._python_logger = logging.getLogger(f"toms.{task_id}")
        if also_print and not self._python_logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter("[TOMS] %(levelname)s %(message)s"))
            self._python_logger.addHandler(handler)
            self._python_logger.setLevel(logging.DEBUG)

    # ------------------------------------------------------------------
    # Public logging methods
    # ------------------------------------------------------------------

    def log_world_state(self, world_state: WorldState) -> None:
        self._emit("world_state", _to_dict(world_state))

    def log_object_selected(self, object_id: str) -> None:
        self._emit("object_selected", {"object_id": object_id})
        self._python_logger.info(f"Selected object: {object_id}")

    def log_grasp_candidates(self, candidates: List[GraspCandidate]) -> None:
        self._emit("grasp_candidates", {"candidates": [_to_dict(c) for c in candidates]})

    def log_plan_result(self, step: str, result: PlanResult) -> None:
        self._emit("plan_result", {"step": step, **_to_dict(result)})
        status = "OK" if result.success else f"FAIL ({result.error_code})"
        self._python_logger.info(f"Plan [{step}]: {status}")

    def log_execution_step(self, step: str, success: bool, details: Optional[Dict] = None) -> None:
        payload: Dict = {"step": step, "success": success}
        if details:
            payload.update(details)
        self._emit("execution_step", payload)
        self._python_logger.info(f"Exec [{step}]: {'OK' if success else 'FAIL'}")

    def log_validation_result(self, phase: str, result: ValidationResult) -> None:
        self._emit("validation_result", {"phase": phase, **_to_dict(result)})
        if result.success:
            self._python_logger.info(f"Validation [{phase}]: PASS")
        else:
            self._python_logger.warning(
                f"Validation [{phase}]: FAIL type={result.failure_type} {result.message}"
            )

    def log_retry(self, object_id: str, attempt: int, reason: str) -> None:
        self._emit("retry", {"object_id": object_id, "attempt": attempt, "reason": reason})
        self._python_logger.warning(f"Retry {attempt} for {object_id}: {reason}")

    def log_task_outcome(self, outcome: TaskOutcome) -> None:
        self._emit("task_outcome", _to_dict(outcome))
        status = "SUCCESS" if outcome.success else "FAILED"
        self._python_logger.info(
            f"Task {outcome.task_id} {status} | "
            f"placed={len(outcome.objects_placed)} "
            f"failed={len(outcome.objects_failed)} "
            f"duration={outcome.duration_sec:.1f}s"
        )

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _emit(self, event_type: str, payload: Dict) -> None:
        record = {
            "ts": time.time(),
            "task_id": self._task_id,
            "event": event_type,
            **payload,
        }
        self._file.write(json.dumps(record) + "\n")
        self._file.flush()

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> "TomsLogger":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
