"""pen_to_holder.py – assembled behavior tree for the pen-to-holder task.

Tree structure (matches AGENTS.md spec):

  Sequence("Task")
    UpdateWorldState
    FindContainer
    RepeatUntilFailure("PickPlaceLoop")
      Sequence("PickPlaceOnce")
        UpdateWorldState
        SelectNextObject
        PickObject            ← pick sub-tree (grasp + validate)
        PlaceObjectInContainer ← place sub-tree (place + validate)
        MarkObjectComplete
    ReportOutcome

The loop exits (returns SUCCESS) when SelectNextObject fails because
objects_pending is empty, or when the retry budget in TaskState is exceeded
(SelectNextObject returns FAILURE after marking all reachable objects FAILED).
"""
from __future__ import annotations

from typing import Optional

from toms_bt.node_base import BTNode, RepeatUntilFailure, Sequence
from toms_bt.nodes.find_container import FindContainer
from toms_bt.nodes.mark_complete import MarkObjectComplete
from toms_bt.nodes.pick_subtree import (
    ExecutionAdapter as PickExecutionAdapter,
)
from toms_bt.nodes.pick_subtree import (
    FeasibilityAdapter,
    GraspGeneratorAdapter,
    GraspValidatorAdapter,
    MotionPlannerAdapter,
    PickObject,
)
from toms_bt.nodes.place_subtree import (
    ExecutionAdapter as PlaceExecutionAdapter,
)
from toms_bt.nodes.place_subtree import (
    PlacementValidatorAdapter,
    PlaceObjectInContainer,
    PlacePoseAdapter,
)
from toms_bt.nodes.report_outcome import LoggerAdapter, ReportOutcome
from toms_bt.nodes.select_next_object import SelectNextObject
from toms_bt.nodes.update_world_state import (
    PerceptionAdapter,
    RobotAdapter,
    UpdateWorldState,
)


class PenToHolderConfig:
    """All tunable parameters for the pen-to-holder task.

    TODO: load from task.yaml at runtime instead of using defaults.
    """

    container_label: str = "pen_holder"
    object_label: str = "pen"
    max_loop_iterations: int = 50   # hard ceiling on pick-place iterations
    # TODO: planning_group, end_effector_link, base_frame → from robot.yaml
    planning_group: str = "TODO_SET_IN_CONFIG"
    end_effector_link: str = "TODO_SET_IN_CONFIG"
    base_frame: str = "TODO_SET_IN_CONFIG"


def build_pen_to_holder_tree(
    perception: Optional[PerceptionAdapter] = None,
    robot: Optional[RobotAdapter] = None,
    grasp_generator: Optional[GraspGeneratorAdapter] = None,
    feasibility: Optional[FeasibilityAdapter] = None,
    planner: Optional[MotionPlannerAdapter] = None,
    pick_executor: Optional[PickExecutionAdapter] = None,
    grasp_validator: Optional[GraspValidatorAdapter] = None,
    place_pose_adapter: Optional[PlacePoseAdapter] = None,
    place_executor: Optional[PlaceExecutionAdapter] = None,
    placement_validator: Optional[PlacementValidatorAdapter] = None,
    logger: Optional[LoggerAdapter] = None,
    config: Optional[PenToHolderConfig] = None,
) -> BTNode:
    """Assemble and return the pen-to-holder behavior tree root node.

    All adapters are injected here; swap in mock adapters for simulation,
    real adapters for hardware.
    """
    cfg = config or PenToHolderConfig()

    pick_place_once = Sequence(
        "PickPlaceOnce",
        children=[
            UpdateWorldState(
                "UpdateWorldState_Loop",
                perception=perception,
                robot=robot,
            ),
            SelectNextObject(
                object_label=cfg.object_label,
            ),
            PickObject(
                grasp_generator=grasp_generator,
                feasibility=feasibility,
                planner=planner,
                executor=pick_executor,
                grasp_validator=grasp_validator,
                planning_group=cfg.planning_group,
                end_effector_link=cfg.end_effector_link,
                base_frame=cfg.base_frame,
            ),
            PlaceObjectInContainer(
                place_pose_adapter=place_pose_adapter,
                planner=planner,
                executor=place_executor,
                placement_validator=placement_validator,
                planning_group=cfg.planning_group,
                end_effector_link=cfg.end_effector_link,
                base_frame=cfg.base_frame,
            ),
            MarkObjectComplete(),
        ],
    )

    pick_place_loop = RepeatUntilFailure(
        "PickPlaceLoop",
        child=pick_place_once,
        max_iterations=cfg.max_loop_iterations,
    )

    root = Sequence(
        "Task_PenToHolder",
        children=[
            UpdateWorldState(
                "UpdateWorldState_Init",
                perception=perception,
                robot=robot,
            ),
            FindContainer(container_label=cfg.container_label),
            pick_place_loop,
            ReportOutcome(logger=logger),
        ],
    )

    return root
