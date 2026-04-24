"""toms_logging.logging_adapters – pass-through adapter wrappers with event logging.

Each wrapper accepts an inner adapter (the real or mock implementation) and a
RunLogger.  All public adapter methods are delegated to the inner adapter;
results are forwarded to the logger before being returned to the caller.

This means the logging layer is entirely outside the core architecture:
  - No changes to any existing adapter or BT node.
  - Inject the wrapper at tree construction time, just like any other adapter.

Example::

    from toms_logging.run_manager import RunManager
    from toms_logging.logging_adapters import LoggingPlanner, LoggingGraspValidator

    mgr = RunManager("runs", task_id="run_001")
    log = mgr.make_logger()

    planner     = LoggingPlanner(MockMotionPlanner(), log)
    gv          = LoggingGraspValidator(GraspValidator(config), log)
    pv          = LoggingPlacementValidator(PlacementValidator(config), log)
    executor    = LoggingExecutor(ManipulationExecutor(robot), log)
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING, List

from toms_core.models import (
    GraspCandidate,
    PlanRequest,
    PlanResult,
    ValidationResult,
    WorldState,
)

if TYPE_CHECKING:
    from toms_logging.run_manager import RunLogger


# ---------------------------------------------------------------------------
# Motion planner
# ---------------------------------------------------------------------------


class LoggingPlanner:
    """Wraps any MotionPlannerAdapter to log PlanRequest/PlanResult pairs.

    Compatible with both pick_subtree.MotionPlannerAdapter and
    place_subtree.MotionPlannerAdapter (both require only .plan()).
    """

    def __init__(self, inner: object, logger: "RunLogger") -> None:
        self._inner = inner
        self._logger = logger

    def plan(self, request: PlanRequest) -> PlanResult:
        t0 = time.monotonic()
        result = self._inner.plan(request)  # type: ignore[attr-defined]
        elapsed = time.monotonic() - t0
        self._logger.log_grasp_candidates(
            getattr(request.grasp_candidate, "_candidates", [request.grasp_candidate])
        )
        self._logger.log_plan_result(request.object_id, result)
        if result.planning_time == 0.0:
            # Fill in wall-clock time if the planner didn't measure it
            object.__setattr__(result, "planning_time", elapsed)
        return result


# ---------------------------------------------------------------------------
# Grasp validator
# ---------------------------------------------------------------------------


class LoggingGraspValidator:
    """Wraps any GraspValidatorAdapter to log ValidationResult events."""

    def __init__(self, inner: object, logger: "RunLogger") -> None:
        self._inner = inner
        self._logger = logger

    def validate(self, world_state: WorldState) -> ValidationResult:
        result = self._inner.validate(world_state)  # type: ignore[attr-defined]
        self._logger.log_validation_result("grasp", result)
        return result


# ---------------------------------------------------------------------------
# Placement validator
# ---------------------------------------------------------------------------


class LoggingPlacementValidator:
    """Wraps any PlacementValidatorAdapter to log ValidationResult events."""

    def __init__(self, inner: object, logger: "RunLogger") -> None:
        self._inner = inner
        self._logger = logger

    def validate(self, object_id: str, world_state: WorldState) -> ValidationResult:
        result = self._inner.validate(object_id, world_state)  # type: ignore[attr-defined]
        self._logger.log_validation_result("placement", result)
        return result


# ---------------------------------------------------------------------------
# Executor (grasp + place)
# ---------------------------------------------------------------------------


class LoggingExecutor:
    """Wraps an executor that has both execute_grasp and execute_place.

    Logs execution_step events for both pick and place phases.
    Compatible with ManipulationExecutor and any mock with the same interface.
    """

    def __init__(self, inner: object, logger: "RunLogger") -> None:
        self._inner = inner
        self._logger = logger

    def execute_grasp(self, plan: PlanResult, world_state: WorldState) -> bool:
        ok = self._inner.execute_grasp(plan, world_state)  # type: ignore[attr-defined]
        self._logger.log_execution_step(
            "grasp",
            ok,
            {"gripper_width": world_state.robot.gripper_width},
        )
        return ok

    def execute_place(self, plan: PlanResult, world_state: WorldState) -> bool:
        ok = self._inner.execute_place(plan, world_state)  # type: ignore[attr-defined]
        self._logger.log_execution_step(
            "place",
            ok,
            {"gripper_width": world_state.robot.gripper_width},
        )
        return ok


# ---------------------------------------------------------------------------
# Grasp generator (optional instrumentation)
# ---------------------------------------------------------------------------


class LoggingGraspGenerator:
    """Wraps a GraspGeneratorAdapter to log generated candidates."""

    def __init__(self, inner: object, logger: "RunLogger") -> None:
        self._inner = inner
        self._logger = logger

    def generate(self, object_id: str, world_state: WorldState) -> List[GraspCandidate]:
        candidates = self._inner.generate(object_id, world_state)  # type: ignore[attr-defined]
        self._logger.log_grasp_candidates(candidates)
        return candidates
