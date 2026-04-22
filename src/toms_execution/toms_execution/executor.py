"""toms_execution.executor – action execution sequences for pick and place.

Implements the physical execution steps (approach, close gripper, lift,
place, retreat) by composing calls to the robot bridge.

All motion parameters (velocity scales, approach offsets, gripper force)
come from planning.yaml / robot.yaml.
"""
from __future__ import annotations

from typing import List, Optional, Protocol

from toms_core.models import PlanResult, Pose, WorldState


class RobotBridgeProtocol(Protocol):
    """Minimal robot interface needed by the executor."""

    def open_gripper(self) -> bool:
        ...

    def close_gripper(self, target_width: float, force: float) -> bool:
        ...

    def execute_trajectory(self, trajectory: object) -> bool:
        ...

    def execute_cartesian_move(self, poses: List[Pose], velocity_scale: float) -> bool:
        ...


class ManipulationExecutor:
    """Sequences robot bridge calls for pick and place primitives.

    TODO: All numeric parameters (velocity, force, offsets) from robot.yaml
          and planning.yaml.  Do NOT hardcode these for the real robot.
    """

    def __init__(
        self,
        robot: Optional[RobotBridgeProtocol] = None,
        # TODO: all the below from robot.yaml / planning.yaml
        approach_velocity_scale: float = 0.1,
        retreat_velocity_scale: float = 0.15,
        gripper_close_width: float = 0.01,  # TODO: per-object from config
        gripper_close_force: float = 10.0,  # N; TODO: from robot.yaml
    ) -> None:
        self._robot = robot
        self._approach_vel = approach_velocity_scale
        self._retreat_vel = retreat_velocity_scale
        self._gripper_close_width = gripper_close_width
        self._gripper_close_force = gripper_close_force

    # ------------------------------------------------------------------
    # Pick
    # ------------------------------------------------------------------

    def execute_grasp(self, plan: PlanResult, world_state: WorldState) -> bool:
        """Run the full grasp sequence: open → approach → close → lift.

        Returns True if all steps succeed.
        Validation is NOT performed here; that is toms_validation's job.
        """
        if self._robot is None or plan.trajectory is None:
            return False

        # 1. Open gripper
        if not self._robot.open_gripper():
            return False

        # 2. Execute pre-grasp joint trajectory (plan to pre-grasp pose)
        if not self._robot.execute_trajectory(plan.trajectory):
            return False

        # 3. Cartesian approach (final few cm)
        # TODO: approach_waypoints computed from grasp candidate pose
        #       using tool_frame offset from robot.yaml
        # approach_poses = _compute_approach_waypoints(plan)
        # if not self._robot.execute_cartesian_move(approach_poses, self._approach_vel):
        #     return False

        # 4. Close gripper
        if not self._robot.close_gripper(
            target_width=self._gripper_close_width,
            force=self._gripper_close_force,
        ):
            return False

        # 5. Small lift (Cartesian, ~3 cm)
        # TODO: lift_pose from grasp pose + lift_offset_z from planning.yaml
        # lift_poses = _compute_lift_waypoints(plan)
        # if not self._robot.execute_cartesian_move(lift_poses, self._approach_vel):
        #     return False

        return True

    # ------------------------------------------------------------------
    # Place
    # ------------------------------------------------------------------

    def execute_place(self, plan: PlanResult, world_state: WorldState) -> bool:
        """Run the full place sequence: approach → release → retreat.

        Returns True if all steps succeed.
        """
        if self._robot is None or plan.trajectory is None:
            return False

        # 1. Execute joint trajectory to placement pre-pose
        if not self._robot.execute_trajectory(plan.trajectory):
            return False

        # 2. Cartesian descent into holder
        # TODO: descent_poses computed from place candidate + robot.yaml offsets

        # 3. Open gripper to release
        if not self._robot.open_gripper():
            return False

        # 4. Cartesian retreat
        # TODO: retreat_poses from planning.yaml retreat_distance

        return True
