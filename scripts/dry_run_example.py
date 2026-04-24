#!/usr/bin/env python3
"""Dry-run example: full pen-to-holder BT with mock adapters + full diagnostics.

Runs entirely without ROS2.  Creates a real run directory under runs/ and
writes structured events, config snapshot, and rosout.txt just like a
hardware run would.

Usage::

    # From repo root:
    PYTHONPATH=src/toms_core:src/toms_bt:src/toms_world:src/toms_validation:\\
    src/toms_perception:src/toms_planning:src/toms_execution:\\
    src/toms_robot:src/toms_logging \\
    python3 scripts/dry_run_example.py

    # Summarise the result:
    toms-run-summary runs/
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

# ── Minimal PYTHONPATH guard ─────────────────────────────────────────────────
_REPO_ROOT = Path(__file__).resolve().parent.parent
_PATHS = [
    "src/toms_core",
    "src/toms_bt",
    "src/toms_world",
    "src/toms_validation",
    "src/toms_perception",
    "src/toms_planning",
    "src/toms_execution",
    "src/toms_robot",
    "src/toms_logging",
]
for _p in _PATHS:
    _abs = str(_REPO_ROOT / _p)
    if _abs not in sys.path:
        sys.path.insert(0, _abs)

# ── Imports ──────────────────────────────────────────────────────────────────
from toms_bt.node_base import NodeStatus
from toms_bt.templates.pen_to_holder import PenToHolderConfig, build_pen_to_holder_tree
from toms_core.models import (
    BoundingBox,
    GraspCandidate,
    GraspStatus,
    ObjectState,
    ObjectStatus,
    Position,
    Pose,
    TaskState,
    TaskStatus,
    ValidationResult,
    WorldState,
)
from toms_logging.logging_adapters import (
    LoggingExecutor,
    LoggingGraspGenerator,
    LoggingGraspValidator,
    LoggingPlacementValidator,
    LoggingPlanner,
)
from toms_logging.run_manager import RunManager
from toms_logging.run_summary import format_summary, parse_run
from toms_planning.feasibility import MockFeasibilityFilter
from toms_planning.grasp_generator import HeuristicGraspGenerator
from toms_planning.moveit_wrapper import MockMotionPlanner
from toms_robot.robot_bridge import MockRobotBridge
from toms_validation.grasp_validator import GraspValidator, GraspValidationConfig
from toms_validation.placement_validator import (
    PlacementValidationConfig,
    PlacementValidator,
)


# ── Mock adapters ────────────────────────────────────────────────────────────

class MockPerception:
    """Returns the objects already in world_state (they were seeded below)."""

    def update_world_state(self, world_state: WorldState) -> None:
        world_state.sequence += 1


class MockPlacePoseAdapter:
    """Returns a fixed place pose above the container."""

    def generate_place_pose(
        self, object_id: str, container_id: str, world_state: WorldState
    ) -> GraspCandidate:
        container = world_state.container
        px = container.pose.position.x if container else 0.0
        py = container.pose.position.y if container else 0.0
        return GraspCandidate(
            candidate_id=f"place_{object_id}",
            object_id=object_id,
            pose=Pose(position=Position(x=px, y=py, z=0.25)),
            score=1.0,
            status=GraspStatus.CANDIDATE,
        )


class _WrappedExecutor:
    """Minimal executor that succeeds and updates gripper state."""

    def __init__(self, robot: MockRobotBridge) -> None:
        self._robot = robot

    def execute_grasp(self, plan, world_state: WorldState) -> bool:
        self._robot._simulate_grasp_success()
        self._robot.update_robot_state(world_state)
        return True

    def execute_place(self, plan, world_state: WorldState) -> bool:
        self._robot.open_gripper()
        self._robot.update_robot_state(world_state)
        return True


# ── World state setup ────────────────────────────────────────────────────────

def _build_world_state(task_id: str) -> WorldState:
    ws = WorldState(
        task=TaskState(
            task_id=task_id,
            retry_budget=3,
            status=TaskStatus.RUNNING,
        )
    )
    # 3 pens
    for i in range(3):
        oid = f"pen_{i}"
        ws.objects[oid] = ObjectState(
            object_id=oid,
            label="pen",
            status=ObjectStatus.DETECTED,
            detection_confidence=0.8 + i * 0.05,
            pose=Pose(position=Position(x=0.3 + i * 0.05, y=0.0, z=0.02)),
            bounding_box=BoundingBox(
                center=Position(x=0.3 + i * 0.05, y=0.0, z=0.02),
                size_x=0.01, size_y=0.01, size_z=0.15,
            ),
        )
        ws.task.objects_pending.append(oid)
    # 1 pen holder
    ws.objects["holder_0"] = ObjectState(
        object_id="holder_0",
        label="pen_holder",
        status=ObjectStatus.DETECTED,
        detection_confidence=0.95,
        pose=Pose(position=Position(x=0.5, y=0.0, z=0.01)),
        bounding_box=BoundingBox(
            center=Position(x=0.5, y=0.0, z=0.01),
            size_x=0.06, size_y=0.06, size_z=0.12,
        ),
    )
    return ws


# ── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    task_id = f"dry_run_{int(time.time())}"
    print(f"\nTOMS dry-run example  task_id={task_id}")
    print("─" * 60)

    # Run directory + logger
    mgr = RunManager("runs", task_id)
    mgr.setup_rosout_handler()
    mgr.snapshot_configs(
        str(_REPO_ROOT / "config" / "robot.yaml"),
        str(_REPO_ROOT / "config" / "robots" / "cello_follower.yaml"),
        str(_REPO_ROOT / "config" / "planning.yaml"),
        str(_REPO_ROOT / "config" / "task.yaml"),
    )
    log = mgr.make_logger(also_print=True)
    log.log_run_start(str(mgr.run_dir), ["robot.yaml", "cello_follower.yaml"])

    # Build adapters
    robot = MockRobotBridge(gripper_open_width=0.08)
    inner_executor = _WrappedExecutor(robot)

    grasp_cfg = GraspValidationConfig(
        max_empty_gripper_width=0.07,
        min_grasp_gripper_width=0.005,
    )
    place_cfg = PlacementValidationConfig(
        min_released_gripper_width=0.04,
        max_container_distance_xy=0.05,
    )

    # Wrap each adapter with a logging wrapper
    planner = LoggingPlanner(MockMotionPlanner(always_succeed=True), log)
    grasp_gen = LoggingGraspGenerator(HeuristicGraspGenerator(), log)
    feasibility = MockFeasibilityFilter()
    grasp_val = LoggingGraspValidator(GraspValidator(grasp_cfg), log)
    place_val = LoggingPlacementValidator(PlacementValidator(place_cfg), log)
    executor = LoggingExecutor(inner_executor, log)

    # Build BT tree
    cfg = PenToHolderConfig()
    cfg.planning_group = "arm"
    cfg.end_effector_link = "link6"
    cfg.base_frame = "base_link"

    tree = build_pen_to_holder_tree(
        perception=MockPerception(),
        robot=robot,
        grasp_generator=grasp_gen,
        feasibility=feasibility,
        planner=planner,
        pick_executor=executor,
        grasp_validator=grasp_val,
        place_pose_adapter=MockPlacePoseAdapter(),
        place_executor=executor,
        placement_validator=place_val,
        logger=log,
        config=cfg,
    )

    world_state = _build_world_state(task_id)

    # ── Tick loop ─────────────────────────────────────────────────────────
    print(f"Ticking at 100 Hz (max 500 ticks)…")
    prev_statuses: dict = {}
    t0 = time.monotonic()
    final_status = NodeStatus.RUNNING

    for tick in range(500):
        status = tree.tick(world_state)

        # Detect and log BT transitions
        from toms_bt.instrumented_runner import _walk_statuses
        after = _walk_statuses(tree)
        for node_id, (name, after_val) in after.items():
            prev_val = prev_statuses.get(node_id)
            if prev_val != after_val:
                log.log_bt_transition(name, prev_val or "RUNNING", after_val, world_state.sequence)
        prev_statuses = {nid: val for nid, (_, val) in after.items()}

        log.log_world_state(world_state)

        if status in (NodeStatus.SUCCESS, NodeStatus.FAILURE):
            final_status = status
            break

    elapsed = time.monotonic() - t0
    success = final_status == NodeStatus.SUCCESS
    log.log_run_end(success=success, duration_sec=elapsed)
    mgr.write_run_metadata({"outcome": final_status.value, "sim_mode": True})
    mgr.teardown()

    # ── Summary ──────────────────────────────────────────────────────────
    print()
    print(format_summary(parse_run(mgr.run_dir)))
    print(f"\nArtifact directory: {mgr.run_dir}")
    print(f"  events.jsonl        – {(mgr.run_dir / 'events.jsonl').stat().st_size} bytes")
    print(f"  config_snapshot.yaml– {(mgr.run_dir / 'config_snapshot.yaml').stat().st_size} bytes")
    print(f"  rosout.txt          – {(mgr.run_dir / 'rosout.txt').stat().st_size} bytes")
    print(f"  run_metadata.json   – {(mgr.run_dir / 'run_metadata.json').stat().st_size} bytes")


if __name__ == "__main__":
    main()
