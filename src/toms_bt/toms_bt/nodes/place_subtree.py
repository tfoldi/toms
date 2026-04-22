"""PlaceObjectInContainer – sub-tree that places the grasped object.

Flow:
  GeneratePlacePose → PlanMotion → ExecutePlace → ValidatePlacement

Each step is an injected adapter call.
"""
from __future__ import annotations

from typing import Optional, Protocol

from toms_core.models import (
    GraspCandidate,
    ObjectStatus,
    PlanRequest,
    PlanResult,
    ValidationResult,
    WorldState,
)

from toms_bt.node_base import BTNode, NodeStatus
from toms_bt.nodes.select_next_object import SELECTED_OBJECT_KEY


class PlacePoseAdapter(Protocol):
    def generate_place_pose(
        self, object_id: str, container_id: str, world_state: WorldState
    ) -> GraspCandidate:
        """Return a placement pose (reuses GraspCandidate struct for pose)."""
        ...


class MotionPlannerAdapter(Protocol):
    def plan(self, request: PlanRequest) -> PlanResult:
        ...


class ExecutionAdapter(Protocol):
    def execute_place(self, plan: PlanResult, world_state: WorldState) -> bool:
        """Execute approach, release, retreat sequence. Returns True on success."""
        ...


class PlacementValidatorAdapter(Protocol):
    def validate(self, object_id: str, world_state: WorldState) -> ValidationResult:
        """Validate: not in gripper, inside holder zone, stable after retreat."""
        ...


class PlaceObjectInContainer(BTNode):
    """Full place sub-tree for a single object.

    Inputs:  world_state.task.metadata[SELECTED_OBJECT_KEY]
             world_state.container set by FindContainer
             world_state.robot.is_holding_object == True
    Outputs: object status → PLACED (success) or FAILED
             world_state.robot.is_holding_object → False
    Success: placement validation passes
    Failure: no place pose, planning failed, execution error, or
             placement validation returned failure
    Retry:   caller increments retry budget; PickObject must run first
    """

    def __init__(
        self,
        name: str = "PlaceObjectInContainer",
        place_pose_adapter: Optional[PlacePoseAdapter] = None,
        planner: Optional[MotionPlannerAdapter] = None,
        executor: Optional[ExecutionAdapter] = None,
        placement_validator: Optional[PlacementValidatorAdapter] = None,
        # TODO: from robot.yaml
        planning_group: str = "TODO_SET_IN_CONFIG",
        end_effector_link: str = "TODO_SET_IN_CONFIG",
        base_frame: str = "TODO_SET_IN_CONFIG",
    ) -> None:
        super().__init__(name)
        self._place_pose_adapter = place_pose_adapter
        self._planner = planner
        self._executor = executor
        self._placement_validator = placement_validator
        self._planning_group = planning_group
        self._end_effector_link = end_effector_link
        self._base_frame = base_frame

    def tick(self, world_state: WorldState) -> NodeStatus:
        task = world_state.task
        if task is None or world_state.container is None:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        object_id: Optional[str] = getattr(task, "metadata", {}).get(SELECTED_OBJECT_KEY)
        if object_id is None or object_id not in world_state.objects:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        obj = world_state.objects[object_id]
        container = world_state.container

        # --- 1. Generate place pose ---
        if self._place_pose_adapter is None:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE
        place_candidate = self._place_pose_adapter.generate_place_pose(
            object_id, container.object_id, world_state
        )

        # --- 2. Plan motion to place pose ---
        if self._planner is None:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE
        plan_req = PlanRequest(
            object_id=object_id,
            grasp_candidate=place_candidate,
            planning_group=self._planning_group,
            end_effector_link=self._end_effector_link,
            base_frame=self._base_frame,
        )
        plan_result = self._planner.plan(plan_req)
        if not plan_result.success:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        # --- 3. Execute place ---
        if self._executor is None:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE
        exec_ok = self._executor.execute_place(plan_result, world_state)
        if not exec_ok:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        # --- 4. Validate placement ---
        if self._placement_validator is None:
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE
        validation = self._placement_validator.validate(object_id, world_state)
        if not validation.success:
            obj.status = ObjectStatus.FAILED
            world_state.robot.is_holding_object = False
            self._status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        obj.status = ObjectStatus.PLACED
        world_state.robot.is_holding_object = False
        self._status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS
