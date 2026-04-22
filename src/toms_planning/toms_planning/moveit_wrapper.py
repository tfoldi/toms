"""toms_planning.moveit_wrapper – MoveIt2 motion planning interface.

Wraps MoveIt2 MoveGroup or the MoveIt2 Python API.
All robot-specific parameters (planning_group, end_effector_link, base_frame,
trajectory action server, kinematics plugin) come from robot.yaml.

TODO: implement once robot.yaml "Arm Integration" section is filled in.
      Reference: https://github.com/Welt-liu/star-arm-moveit2
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from toms_core.models import PlanRequest, PlanResult


class MotionPlannerBase(ABC):
    """Interface for motion planning backends."""

    @abstractmethod
    def plan(self, request: PlanRequest) -> PlanResult:
        """Plan a collision-free trajectory to the grasp/place pose.

        Returns PlanResult.success=False if planning fails after max_attempts.
        Must NOT execute the trajectory; execution is toms_execution's job.
        """


class MockMotionPlanner(MotionPlannerBase):
    """Mock planner that returns a trivial successful plan.

    Does not compute real kinematics.  Replace with MoveIt2RosPlanner.
    """

    def __init__(self, always_succeed: bool = True) -> None:
        self._always_succeed = always_succeed

    def plan(self, request: PlanRequest) -> PlanResult:
        if not self._always_succeed:
            return PlanResult(
                success=False,
                error_code="MOCK_FAILURE",
                error_message="MockMotionPlanner configured to fail.",
            )
        # TODO: return a real trajectory_msgs/JointTrajectory here.
        return PlanResult(
            success=True,
            trajectory={"mock": True, "request": request},
            planning_time=0.001,
        )


class MoveIt2RosPlanner(MotionPlannerBase):
    """MoveIt2 planner via ROS2 action/service.

    TODO: implement this class once the following are known from robot.yaml:
      - planning_group
      - end_effector_link
      - base_frame
      - trajectory action server name
      - kinematics plugin (KDL / TRAC-IK / BioIK)
      - URDF / SRDF paths

    Suggested implementation steps:
      1. Import moveit_py or use the MoveGroup ROS2 action directly.
      2. Set planning parameters from robot.yaml config node.
      3. Convert PlanRequest.grasp_candidate.pose to PoseStamped in base_frame.
      4. Call plan_to_pose(); capture error_code.
      5. On success, return PlanResult with the JointTrajectory message.
    """

    def __init__(self) -> None:
        raise NotImplementedError(
            "MoveIt2RosPlanner is not yet implemented. "
            "Fill in robot.yaml first, then implement this class. "
            "See AGENTS.md 'Arm Integration' section."
        )

    def plan(self, request: PlanRequest) -> PlanResult:  # pragma: no cover
        raise NotImplementedError
