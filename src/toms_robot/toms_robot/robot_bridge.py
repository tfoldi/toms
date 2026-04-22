"""toms_robot.robot_bridge – abstract robot adapter and mock implementation.

IMPORTANT: All topic names, joint names, controller names, and frame IDs
must come from robot.yaml.  Do NOT hardcode them here.

Reference hardware: see AGENTS.md "Arm Integration" section for the full
list of fields that must be filled before this connects to real hardware.
TODO: https://github.com/Welt-liu/star-arm-moveit2
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Optional

from toms_core.models import Pose, RobotState, WorldState


class RobotBridgeBase(ABC):
    """Interface every robot backend must implement."""

    @abstractmethod
    def update_robot_state(self, world_state: WorldState) -> None:
        """Refresh world_state.robot from the current hardware/sim state."""

    @abstractmethod
    def open_gripper(self) -> bool:
        """Command gripper fully open.  Returns True when confirmed open."""

    @abstractmethod
    def close_gripper(self, target_width: float, force: float) -> bool:
        """Command gripper to close to target_width (m) at given force (N).

        Returns True when gripper reaches a stable closed state.
        Does NOT validate grasp success – that is toms_validation's job.
        """

    @abstractmethod
    def execute_trajectory(self, trajectory: object) -> bool:
        """Send a pre-planned trajectory to the controller.

        trajectory: a trajectory_msgs/JointTrajectory (or equivalent) object.
        Returns True when execution completes without error.

        TODO: trajectory type to be confirmed once arm ROS2 driver is known.
        """

    @abstractmethod
    def execute_cartesian_move(
        self, poses: List[Pose], velocity_scale: float = 0.1
    ) -> bool:
        """Execute a Cartesian waypoint path (final approach / retreat).

        velocity_scale: fraction of max velocity (0-1).
        TODO: frame ID comes from robot.yaml (tool_frame / base_frame).
        """


class MockRobotBridge(RobotBridgeBase):
    """Simulated robot bridge for testing.

    Tracks a simple internal state; does NOT simulate kinematics or dynamics.
    """

    def __init__(
        self,
        # TODO: gripper_open_width and gripper_close_width from robot.yaml
        gripper_open_width: float = 0.08,
        gripper_close_width_empty: float = 0.08,  # no object in gripper
        gripper_close_width_with_pen: float = 0.012,  # pen-sized object
    ) -> None:
        self._open_width = gripper_open_width
        self._close_width_empty = gripper_close_width_empty
        self._close_width_pen = gripper_close_width_with_pen
        self._gripper_width = gripper_open_width
        self._holding_object = False

    # ------------------------------------------------------------------
    # RobotBridgeBase implementation
    # ------------------------------------------------------------------

    def update_robot_state(self, world_state: WorldState) -> None:
        world_state.robot.gripper_width = self._gripper_width
        world_state.robot.is_holding_object = self._holding_object
        # TODO: populate joint_positions and ee_pose from sim kinematics
        #       or from /joint_states topic once robot is known.

    def open_gripper(self) -> bool:
        self._gripper_width = self._open_width
        self._holding_object = False
        return True

    def close_gripper(self, target_width: float, force: float) -> bool:
        # In mock: close width reflects whether an object would be held.
        # Actual width reported depends on simulation state, not target.
        self._gripper_width = target_width
        return True

    def execute_trajectory(self, trajectory: object) -> bool:
        # Mock: always succeeds instantly.
        # TODO: integrate with ros2_control / FollowJointTrajectory action.
        return True

    def execute_cartesian_move(
        self, poses: List[Pose], velocity_scale: float = 0.1
    ) -> bool:
        # Mock: always succeeds instantly.
        # TODO: integrate with MoveIt2 Cartesian path execution.
        return True

    # ------------------------------------------------------------------
    # Test helpers (not part of the public interface)
    # ------------------------------------------------------------------

    def _simulate_grasp_success(self) -> None:
        """Call in tests to simulate a successful grasp."""
        self._gripper_width = self._close_width_pen
        self._holding_object = True

    def _simulate_empty_grasp(self) -> None:
        """Call in tests to simulate missed grasp (gripper closed but empty)."""
        self._gripper_width = self._close_width_empty
        self._holding_object = False
