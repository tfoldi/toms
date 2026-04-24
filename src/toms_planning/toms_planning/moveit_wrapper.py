"""toms_planning.moveit_wrapper – MoveIt2 motion planning interface.

All robot-specific parameters (planning_group, end_effector_link, base_frame,
kinematics plugin) come from RobotConfig (loaded from robot.yaml overlay).
Planning tuning parameters (planner_id, planning_time, tolerances) come from
planning.yaml via the PlanningConfig dataclass.

In dry-run mode (node=None) plan() returns PlanResult(success=False) without
touching ROS2; this is the mode used in offline CI tests.
"""
from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

from toms_core.models import PlanRequest, PlanResult

if TYPE_CHECKING:
    import rclpy.node
    from toms_robot.config_loader import RobotConfig

_LOG = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Planning configuration (from planning.yaml)
# ---------------------------------------------------------------------------


@dataclass
class PlanningConfig:
    """Motion planning parameters sourced from planning.yaml.

    All values have safe defaults so the class can be used without loading
    the full YAML in offline tests.
    """

    planner_id: str = ""               # "" → MoveIt2 default (OMPL RRTConnect)
    planning_time: float = 5.0         # seconds allowed per planning call
    max_attempts: int = 3
    goal_position_tolerance: float = 0.005    # metres
    goal_orientation_tolerance: float = 0.01  # radians
    max_velocity_scale: float = 1.0
    max_acceleration_scale: float = 1.0

    @classmethod
    def from_dict(cls, p: dict) -> "PlanningConfig":
        """Build from flat dict with planning.* prefix keys."""
        def get(key: str, default):
            return p.get(key, default)

        return cls(
            planner_id=get("planning.moveit2.planner_id", ""),
            planning_time=get("planning.moveit2.planning_time", 5.0),
            max_attempts=get("planning.moveit2.max_planning_attempts", 3),
            goal_position_tolerance=get(
                "planning.moveit2.goal_position_tolerance", 0.005
            ),
            goal_orientation_tolerance=get(
                "planning.moveit2.goal_orientation_tolerance", 0.01
            ),
        )


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------


class MotionPlannerBase(ABC):
    """Interface for motion planning backends."""

    @abstractmethod
    def plan(self, request: PlanRequest) -> PlanResult:
        """Plan a collision-free trajectory to the grasp/place pose.

        Returns PlanResult.success=False if planning fails after max_attempts.
        Must NOT execute the trajectory; execution is toms_execution's job.
        """


# ---------------------------------------------------------------------------
# Mock planner
# ---------------------------------------------------------------------------


class MockMotionPlanner(MotionPlannerBase):
    """Mock planner that returns a trivial successful plan.

    Does not compute real kinematics.  Replace with MoveIt2RosPlanner for hw.
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
        return PlanResult(
            success=True,
            trajectory={"mock": True, "request": request},
            planning_time=0.001,
        )


# ---------------------------------------------------------------------------
# Real MoveIt2 planner
# ---------------------------------------------------------------------------


class MoveIt2RosPlanner(MotionPlannerBase):
    """MoveIt2 planner via the GetMotionPlan ROS2 service.

    Service: moveit_msgs/srv/GetMotionPlan at /plan_kinematic_path
    The service returns a planned trajectory without executing it.

    Construction:
        config = RobotConfig.from_yaml_files("config/robot.yaml",
                                              "config/robots/cello_follower.yaml")
        planning = PlanningConfig()
        planner = MoveIt2RosPlanner(config, planning)       # dry-run (no node)
        planner.connect(node)                               # attach ROS2 node

    Cello binding (from cello_follower.yaml + planning.yaml):
        planning_group     : arm
        base_frame         : base_link
        end_effector_link  : link6  (tool_frame falls back here if null)
        planner_id         : ""     (MoveIt2 default OMPL planner)
        planning_time      : 5.0 s
    """

    def __init__(
        self,
        config: "RobotConfig",
        planning: Optional[PlanningConfig] = None,
        node: Optional["rclpy.node.Node"] = None,
    ) -> None:
        self._config = config
        self._planning = planning or PlanningConfig()
        self._node = node
        self._plan_client = None

        _LOG.info(
            "MoveIt2RosPlanner binding:\n"
            "  planning_group     : %s\n"
            "  base_frame         : %s\n"
            "  end_effector_link  : %s  (tool=%s)\n"
            "  planner_id         : '%s'  (empty = MoveIt2 default)\n"
            "  planning_time      : %.1f s  max_attempts=%d",
            config.planning_group,
            config.base_frame,
            config.end_effector_link,
            config.effective_tool_frame,
            self._planning.planner_id,
            self._planning.planning_time,
            self._planning.max_attempts,
        )

        if node is not None:
            self._connect(node)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def connect(self, node: "rclpy.node.Node") -> None:
        """Attach a live rclpy.Node and create the planning service client."""
        self._node = node
        self._connect(node)

    def plan(self, request: PlanRequest) -> PlanResult:
        """Plan to the pose in request.grasp_candidate.pose using MoveIt2.

        Returns PlanResult(success=False) when:
          - no node is connected (dry-run mode)
          - the planning service is unavailable
          - MoveIt2 returns a non-SUCCESS error code
        """
        if self._node is None:
            return PlanResult(
                success=False,
                error_code="NO_NODE",
                error_message=(
                    "MoveIt2RosPlanner: no rclpy.Node connected. "
                    "Call connect(node) before plan()."
                ),
            )
        return self._call_plan_service(request)

    def planning_params(self) -> dict:
        """Return the planning parameters that will be used in the next call.

        Useful for introspection and dry-run tests without ROS2.
        """
        return {
            "planning_group": self._config.planning_group,
            "base_frame": self._config.base_frame,
            "end_effector_link": self._config.end_effector_link,
            "tool_frame": self._config.effective_tool_frame,
            "planner_id": self._planning.planner_id,
            "planning_time": self._planning.planning_time,
            "max_attempts": self._planning.max_attempts,
            "goal_position_tolerance": self._planning.goal_position_tolerance,
            "goal_orientation_tolerance": self._planning.goal_orientation_tolerance,
        }

    # ------------------------------------------------------------------
    # Internal – ROS2 connection
    # ------------------------------------------------------------------

    def _connect(self, node: "rclpy.node.Node") -> None:
        """Create the GetMotionPlan service client."""
        try:
            from moveit_msgs.srv import GetMotionPlan  # type: ignore[import]
        except ImportError as exc:
            raise ImportError(
                "moveit_msgs not found. "
                "Source /opt/ros/humble/setup.bash before calling connect()."
            ) from exc

        self._plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        _LOG.info("MoveIt2RosPlanner: service client created → /plan_kinematic_path")

    # ------------------------------------------------------------------
    # Internal – planning service call
    # ------------------------------------------------------------------

    def _call_plan_service(self, request: PlanRequest) -> PlanResult:
        """Build MotionPlanRequest and call /plan_kinematic_path."""
        import time  # noqa: PLC0415

        import rclpy  # type: ignore[import]
        from geometry_msgs.msg import PoseStamped  # type: ignore[import]
        from moveit_msgs.msg import (  # type: ignore[import]
            BoundingVolume,
            Constraints,
            MoveItErrorCodes,
            OrientationConstraint,
            PositionConstraint,
            RobotState,
        )
        from moveit_msgs.srv import GetMotionPlan  # type: ignore[import]
        from shape_msgs.msg import SolidPrimitive  # type: ignore[import]

        if not self._plan_client.wait_for_service(timeout_sec=5.0):
            _LOG.error("MoveIt2 planning service not available: /plan_kinematic_path")
            return PlanResult(
                success=False,
                error_code="SERVICE_UNAVAILABLE",
                error_message="/plan_kinematic_path service not available",
            )

        # Build goal PoseStamped in base_frame
        grasp_pose = request.grasp_candidate.pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = self._config.base_frame
        target_pose.header.stamp = self._node.get_clock().now().to_msg()
        target_pose.pose.position.x = grasp_pose.position.x
        target_pose.pose.position.y = grasp_pose.position.y
        target_pose.pose.position.z = grasp_pose.position.z
        target_pose.pose.orientation.x = grasp_pose.orientation.x
        target_pose.pose.orientation.y = grasp_pose.orientation.y
        target_pose.pose.orientation.z = grasp_pose.orientation.z
        target_pose.pose.orientation.w = grasp_pose.orientation.w

        # Position constraint: small sphere around target
        tol = self._planning.goal_position_tolerance
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self._config.end_effector_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [tol]
        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [target_pose.pose]
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        # Orientation constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = self._config.end_effector_link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_tol = self._planning.goal_orientation_tolerance
        ori_constraint.absolute_x_axis_tolerance = ori_tol
        ori_constraint.absolute_y_axis_tolerance = ori_tol
        ori_constraint.absolute_z_axis_tolerance = ori_tol
        ori_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pos_constraint]
        goal_constraints.orientation_constraints = [ori_constraint]

        # Assemble the service request
        srv_req = GetMotionPlan.Request()
        mp_req = srv_req.motion_plan_request
        mp_req.group_name = self._config.planning_group
        mp_req.planner_id = self._planning.planner_id
        mp_req.allowed_planning_time = self._planning.planning_time
        mp_req.num_planning_attempts = self._planning.max_attempts
        mp_req.max_velocity_scaling_factor = self._planning.max_velocity_scale
        mp_req.max_acceleration_scaling_factor = self._planning.max_acceleration_scale
        mp_req.goal_constraints = [goal_constraints]
        mp_req.start_state = RobotState()   # empty = use current state

        t0 = time.monotonic()
        future = self._plan_client.call_async(srv_req)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=self._planning.planning_time + 2.0
        )
        elapsed = time.monotonic() - t0

        response = future.result()
        if response is None:
            return PlanResult(
                success=False,
                error_code="TIMEOUT",
                error_message="GetMotionPlan service call timed out",
                planning_time=elapsed,
            )

        error_val = response.motion_plan_response.error_code.val
        if error_val != MoveItErrorCodes.SUCCESS:
            _LOG.warning(
                "MoveIt2 planning failed: error_code=%d  group=%s",
                error_val,
                self._config.planning_group,
            )
            return PlanResult(
                success=False,
                error_code=str(error_val),
                error_message=f"MoveItErrorCodes={error_val}",
                planning_time=response.motion_plan_response.planning_time,
            )

        return PlanResult(
            success=True,
            trajectory=response.motion_plan_response.trajectory,
            planning_time=response.motion_plan_response.planning_time,
        )
