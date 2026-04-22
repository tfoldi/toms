"""PickObject – sub-tree that grasps the selected object.

Flow:
  GenerateGrasps → FilterByIK → PlanMotion → ExecuteGrasp → ValidateGrasp

Each step is an injected adapter call; no hardware specifics here.
"""
from __future__ import annotations

from typing import List, Optional, Protocol

from toms_bt.node_base import BTNode, NodeStatus
from toms_bt.nodes.select_next_object import SELECTED_OBJECT_KEY
from toms_core.models import (
    GraspCandidate,
    GraspStatus,
    ObjectStatus,
    PlanRequest,
    PlanResult,
    ValidationResult,
    WorldState,
)


# ---------------------------------------------------------------------------
# Adapter protocols (implementations live in toms_planning / toms_execution /
# toms_validation; injected at construction time)
# ---------------------------------------------------------------------------


class GraspGeneratorAdapter(Protocol):
    def generate(self, object_id: str, world_state: WorldState) -> List[GraspCandidate]:
        """Return a ranked list of grasp candidates for object_id."""
        ...


class FeasibilityAdapter(Protocol):
    def filter(self, candidates: List[GraspCandidate]) -> List[GraspCandidate]:
        """Return only IK-feasible candidates; sets candidate.is_feasible."""
        ...


class MotionPlannerAdapter(Protocol):
    def plan(self, request: PlanRequest) -> PlanResult:
        """Plan a trajectory for the given grasp; returns PlanResult."""
        ...


class ExecutionAdapter(Protocol):
    def execute_grasp(self, plan: PlanResult, world_state: WorldState) -> bool:
        """Execute the planned trajectory; returns True on success."""
        ...


class GraspValidatorAdapter(Protocol):
    def validate(self, world_state: WorldState) -> ValidationResult:
        """Multi-signal grasp validation: width + lift + vision."""
        ...


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class PickObject(BTNode):
    """Full pick sub-tree for a single object.

    Inputs:  world_state.task.metadata[SELECTED_OBJECT_KEY]
    Outputs: selected object status → GRASPED (on success) or FAILED
             world_state.robot.is_holding_object set
    Success: grasp validation passes
    Failure: no feasible grasp, planning failed, execution error, or
             grasp validation returned failure
    Retry:   caller (RepeatUntil) increments retry budget before re-selecting
    """

    def __init__(
        self,
        name: str = "PickObject",
        grasp_generator: Optional[GraspGeneratorAdapter] = None,
        feasibility: Optional[FeasibilityAdapter] = None,
        planner: Optional[MotionPlannerAdapter] = None,
        executor: Optional[ExecutionAdapter] = None,
        grasp_validator: Optional[GraspValidatorAdapter] = None,
        # TODO: planning_group and frame IDs must come from robot.yaml
        planning_group: str = "TODO_SET_IN_CONFIG",
        end_effector_link: str = "TODO_SET_IN_CONFIG",
        base_frame: str = "TODO_SET_IN_CONFIG",
    ) -> None:
        super().__init__(name)
        self._grasp_generator = grasp_generator
        self._feasibility = feasibility
        self._planner = planner
        self._executor = executor
        self._grasp_validator = grasp_validator
        self._planning_group = planning_group
        self._end_effector_link = end_effector_link
        self._base_frame = base_frame

    def tick(self, world_state: WorldState) -> NodeStatus:
        task = world_state.task
        if task is None:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        object_id: Optional[str] = getattr(task, "metadata", {}).get(SELECTED_OBJECT_KEY)
        if object_id is None or object_id not in world_state.objects:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        obj = world_state.objects[object_id]

        # --- 1. Generate grasp candidates ---
        if self._grasp_generator is None:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE
        candidates = self._grasp_generator.generate(object_id, world_state)
        if not candidates:
            obj.status = ObjectStatus.FAILED
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        # --- 2. Filter by IK feasibility ---
        if self._feasibility is not None:
            candidates = self._feasibility.filter(candidates)
        feasible = [c for c in candidates if c.is_feasible is not False]
        if not feasible:
            obj.status = ObjectStatus.FAILED
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        # Highest-score feasible candidate
        best = max(feasible, key=lambda c: c.score)

        # --- 3. Plan motion ---
        if self._planner is None:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE
        plan_req = PlanRequest(
            object_id=object_id,
            grasp_candidate=best,
            planning_group=self._planning_group,
            end_effector_link=self._end_effector_link,
            base_frame=self._base_frame,
        )
        plan_result = self._planner.plan(plan_req)
        if not plan_result.success:
            best.status = GraspStatus.INFEASIBLE
            obj.grasp_attempts += 1
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        # --- 4. Execute grasp ---
        if self._executor is None:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE
        exec_ok = self._executor.execute_grasp(plan_result, world_state)
        if not exec_ok:
            best.status = GraspStatus.FAILED
            obj.grasp_attempts += 1
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        # --- 5. Validate grasp ---
        if self._grasp_validator is None:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE
        validation = self._grasp_validator.validate(world_state)
        obj.grasp_attempts += 1
        if not validation.success:
            obj.status = ObjectStatus.FAILED
            best.status = GraspStatus.FAILED
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        # Grasp confirmed.
        obj.status = ObjectStatus.GRASPED
        best.status = GraspStatus.SUCCESS
        world_state.robot.is_holding_object = True
        self._status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS
