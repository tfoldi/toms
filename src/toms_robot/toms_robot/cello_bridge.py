"""toms_robot.cello_bridge – RobotBridge for the Cello (starai-arm) robot.

All topic/action/frame names come from RobotConfig (loaded from robot.yaml +
cello_follower.yaml).  Nothing is hardcoded here.

ROS2 communication is deferred until a live rclpy.Node is provided.
In dry-run mode (node=None) the bridge validates config and logs bindings
but does not open any sockets; this is the mode used in offline CI tests.

Hardware endpoints (from cello_follower.yaml):
  joint states:     robot.joint_state_topic       → /joint_states
  arm trajectory:   robot.trajectory_action        → /arm_controller/follow_joint_trajectory
  gripper:          robot.gripper.action            → /hand_controller/gripper_cmd
                    (control_msgs/action/GripperCommand)
  gripper joint:    robot.gripper.joint_name        → joint7_left (prismatic, metres)
  open position:    robot.gripper.open_position     → -0.025 m
  close position:   robot.gripper.close_position    → 0.0 m
  tool frame:       robot.tool_frame                → null, falls back to link6
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, List, Optional

from toms_core.models import WorldState

from toms_robot.config_loader import RobotConfig
from toms_robot.robot_bridge import RobotBridgeBase

if TYPE_CHECKING:
    # rclpy is only imported at runtime inside _connect(); this block keeps
    # mypy and IDEs happy without pulling in ROS2 at module load time.
    import rclpy.node
    from toms_core.models import Pose  # also used in execute_cartesian_move sig

_LOG = logging.getLogger(__name__)


class CelloRobotBridge(RobotBridgeBase):
    """Concrete RobotBridge for the Cello 6-DOF arm.

    Pass node=None (default) for dry-run / offline use.
    Pass a live rclpy.Node to enable real hardware communication.
    """

    def __init__(
        self,
        config: RobotConfig,
        node: Optional["rclpy.node.Node"] = None,
    ) -> None:
        self._config = config
        self._node = node

        # ROS2 client handles – created in _connect()
        self._joint_state_sub = None
        self._trajectory_client = None
        self._gripper_client = None

        # Cached hardware state
        self._joint_positions: List[float] = [0.0] * config.dof
        self._gripper_width: float = abs(config.gripper.open_position)
        self._gripper_position: float = 0.0   # joint7_left position (m), updated from /joint_states

        _LOG.info(
            "CelloRobotBridge binding:\n"
            "  joint_state_topic  : %s\n"
            "  trajectory_action  : %s\n"
            "  gripper_action     : %s\n"
            "    joint=%s  open=%.4f m  close=%.4f m  max_force=%.1f N\n"
            "  planning_group     : %s\n"
            "  frames             : base=%s  EE=%s  tool=%s\n"
            "  kinematics         : %s",
            config.joint_state_topic,
            config.trajectory_action,
            config.gripper.action,
            config.gripper.joint_name,
            config.gripper.open_position,
            config.gripper.close_position,
            config.gripper.max_force,
            config.planning_group,
            config.base_frame,
            config.end_effector_link,
            config.effective_tool_frame,
            config.kinematics.plugin,
        )

        if node is not None:
            self._connect(node)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def connect(self, node: "rclpy.node.Node") -> None:
        """Attach a live rclpy.Node and create action/topic clients."""
        self._node = node
        self._connect(node)

    def update_robot_state(self, world_state: WorldState) -> None:
        """Refresh world_state.robot from latest /joint_states message.

        In dry-run mode (no node), leaves state unchanged.
        """
        if self._node is None:
            return
        world_state.robot.joint_positions = list(self._joint_positions)
        world_state.robot.gripper_width = self._gripper_width
        world_state.robot.is_moving = False
        # TODO: populate ee_pose via forward kinematics or /tf once TF listener added

    def open_gripper(self) -> bool:
        """Send GripperCommand(position=open_position, max_effort=0).

        open_position for cello = -0.025 m (prismatic, finger out from centre).
        Returns False in dry-run mode.
        """
        target = self._config.gripper.open_position
        _LOG.debug(
            "open_gripper  action=%s  position=%.4f m",
            self._config.gripper.action,
            target,
        )
        if self._node is None:
            _LOG.warning("open_gripper: dry-run mode, no node connected")
            return False
        return self._send_gripper_command(position=target, max_effort=0.0)

    def close_gripper(self, target_width: float, force: float) -> bool:
        """Send GripperCommand(position=close_position, max_effort=capped_force).

        close_position for cello = 0.0 m (hard stop).
        force is capped at robot.gripper.max_force from config.
        target_width is accepted for interface compatibility but not used for
        commanding (GripperActionController uses the joint position directly).
        Returns False in dry-run mode.
        """
        position = self._config.gripper.close_position
        max_effort = min(force, self._config.gripper.max_force)
        _LOG.debug(
            "close_gripper  action=%s  position=%.4f m  max_effort=%.1f N",
            self._config.gripper.action,
            position,
            max_effort,
        )
        if self._node is None:
            _LOG.warning("close_gripper: dry-run mode, no node connected")
            return False
        return self._send_gripper_command(position=position, max_effort=max_effort)

    def move_to_joint_positions(
        self,
        positions: List[float],
        duration_sec: float = 3.0,
        num_waypoints: int = 25,
    ) -> bool:
        """Build an interpolated multi-point JointTrajectory to positions.

        positions: one float per joint in self._config.joint_names order.
        duration_sec: total time for the motion (end-to-end).
        num_waypoints: how many intermediate steps to interpolate.

        A single-waypoint trajectory causes the servo driver to command the
        goal position immediately with max speed, resulting in abrupt
        "jump-scare" motion even for large deltas.  Splitting the motion
        into many small waypoints (each spaced duration_sec/num_waypoints
        apart) forces the controller to step gradually.

        Linear interpolation from the latest cached joint state. Falls back
        to a single-waypoint command if the cache is empty (first run,
        before /joint_states arrived).

        No collision checking – the caller is responsible for a clear path.
        Returns False in dry-run mode (no node).
        """
        if self._node is None:
            _LOG.warning("move_to_joint_positions: dry-run mode, no node connected")
            return False
        if len(positions) != len(self._config.joint_names):
            _LOG.error(
                "move_to_joint_positions: %d positions but %d joint names",
                len(positions),
                len(self._config.joint_names),
            )
            return False
        try:
            from builtin_interfaces.msg import Duration  # type: ignore[import]  # noqa: PLC0415
            from trajectory_msgs.msg import (  # type: ignore[import]  # noqa: PLC0415
                JointTrajectory,
                JointTrajectoryPoint,
            )
        except ImportError as exc:
            _LOG.error("trajectory_msgs not available: %s", exc)
            return False

        start = list(self._joint_positions)
        if len(start) != len(positions) or all(abs(v) < 1e-9 for v in start):
            # Joint state not yet populated – fall back to single waypoint
            _LOG.warning(
                "move_to_joint_positions: _joint_positions unavailable, "
                "using single waypoint (motion may be abrupt)"
            )
            num_waypoints = 1
            start = list(positions)   # unused, just a safe placeholder

        traj = JointTrajectory()
        traj.header.stamp = self._node.get_clock().now().to_msg()
        traj.joint_names = list(self._config.joint_names)

        points = []
        for i in range(1, num_waypoints + 1):
            frac = i / num_waypoints
            t = duration_sec * frac
            if num_waypoints == 1:
                interp = list(positions)
            else:
                interp = [s + (p - s) * frac for s, p in zip(start, positions)]
            pt = JointTrajectoryPoint()
            pt.positions = interp
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            points.append(pt)
        traj.points = points

        _LOG.info(
            "move_to_joint_positions: %d waypoints over %.1fs (%.0f ms/step) → %s",
            num_waypoints,
            duration_sec,
            (duration_sec / num_waypoints) * 1000.0,
            self._config.trajectory_action,
        )
        return self._send_joint_trajectory(traj)

    def execute_trajectory(self, trajectory: object) -> bool:
        """Send trajectory to the FollowJointTrajectory action server.

        trajectory: trajectory_msgs/JointTrajectory (or compatible message).
        action server: robot.trajectory_action (/arm_controller/follow_joint_trajectory)
        Returns False in dry-run mode.
        """
        _LOG.debug("execute_trajectory  action=%s", self._config.trajectory_action)
        if self._node is None:
            _LOG.warning("execute_trajectory: dry-run mode, no node connected")
            return False
        return self._send_joint_trajectory(trajectory)

    def execute_cartesian_move(
        self, poses: List["Pose"], velocity_scale: float = 0.1
    ) -> bool:
        """Execute Cartesian waypoints via MoveIt2 ComputeCartesianPath.

        frame_id = effective_tool_frame (falls back to end_effector_link if
        robot.tool_frame is null in config).
        Returns False in dry-run mode.

        TODO: implement ComputeCartesianPath service call once confirmed
              that /compute_cartesian_path service is available on hardware.
        """
        _LOG.debug(
            "execute_cartesian_move  %d waypoints  vel=%.2f  frame=%s",
            len(poses),
            velocity_scale,
            self._config.effective_tool_frame,
        )
        if self._node is None:
            _LOG.warning("execute_cartesian_move: dry-run mode, no node connected")
            return False
        raise NotImplementedError(
            "execute_cartesian_move: ComputeCartesianPath service call not yet"
            " implemented. See toms_robot/cello_bridge.py TODO."
        )

    # ------------------------------------------------------------------
    # Internal – ROS2 connection
    # ------------------------------------------------------------------

    def _connect(self, node: "rclpy.node.Node") -> None:
        """Create ROS2 subscriber and action clients from config values."""
        try:
            from control_msgs.action import (
                FollowJointTrajectory,  # type: ignore[import]
                GripperCommand,  # type: ignore[import]
            )
            from rclpy.action import ActionClient  # type: ignore[import]
            from sensor_msgs.msg import JointState  # type: ignore[import]
        except ImportError as exc:
            raise ImportError(
                "ROS2 packages not found. "
                "Source /opt/ros/humble/setup.bash before calling connect()."
            ) from exc

        self._joint_state_sub = node.create_subscription(
            JointState,
            self._config.joint_state_topic,
            self._on_joint_state,
            10,
        )
        self._trajectory_client = ActionClient(
            node,
            FollowJointTrajectory,
            self._config.trajectory_action,
        )
        self._gripper_client = ActionClient(
            node,
            GripperCommand,
            self._config.gripper.action,
        )
        _LOG.info(
            "CelloRobotBridge: ROS2 clients created (traj=%s  gripper=%s)",
            self._config.trajectory_action,
            self._config.gripper.action,
        )

    # ------------------------------------------------------------------
    # Internal – subscriber callback
    # ------------------------------------------------------------------

    def _on_joint_state(self, msg: object) -> None:
        """Cache latest joint positions in config-specified order."""
        try:
            name_to_pos = dict(zip(msg.name, msg.position))  # type: ignore[attr-defined]
            self._joint_positions = [
                name_to_pos.get(n, 0.0) for n in self._config.joint_names
            ]
            self._gripper_position = name_to_pos.get(
                self._config.gripper.joint_name, self._gripper_position
            )
        except Exception as exc:
            _LOG.warning("_on_joint_state error: %s", exc)

    # ------------------------------------------------------------------
    # Internal – action helpers (synchronous wrappers for bring-up)
    # ------------------------------------------------------------------

    def _send_gripper_command(self, position: float, max_effort: float) -> bool:
        """Build and send a GripperCommand goal; block until result."""
        import rclpy  # type: ignore[import]
        from control_msgs.action import GripperCommand  # type: ignore[import]

        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            _LOG.error(
                "Gripper action server not available: %s",
                self._config.gripper.action,
            )
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            _LOG.error("Gripper goal rejected by server")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=10.0)
        result = result_future.result()
        if result is None:
            _LOG.error("Gripper action timed out waiting for result")
            return False
        r = result.result
        self._node.get_logger().info(
            f"Gripper result: position={r.position:.4f} effort={r.effort:.2f} "
            f"reached_goal={r.reached_goal} stalled={r.stalled}"
        )
        # Success = action completed without an abnormal stall.
        # (Some controllers leave reached_goal=False on no-op goals – tolerate it.)
        return not r.stalled

    def _send_joint_trajectory(self, trajectory: object) -> bool:
        """Send a FollowJointTrajectory goal and block until result."""
        import rclpy  # type: ignore[import]
        from control_msgs.action import FollowJointTrajectory  # type: ignore[import]

        if not self._trajectory_client.wait_for_server(timeout_sec=10.0):
            _LOG.error(
                "Trajectory action server not available: %s",
                self._config.trajectory_action,
            )
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        future = self._trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            _LOG.error("Trajectory goal rejected by server")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=60.0)
        result = result_future.result()
        if result is None:
            _LOG.error("Trajectory action timed out waiting for result")
            return False
        # FollowJointTrajectory.Result.error_code: 0 = SUCCESSFUL
        return result.result.error_code == 0
