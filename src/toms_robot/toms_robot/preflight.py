"""toms_robot.preflight – hardware preflight checker for TOMS.

Provides three classes:

  ConfigChecker   – pure Python, no ROS2; validates a loaded RobotConfig
  RosChecker      – requires rclpy + a live ROS2 graph; checks topics, actions,
                    services, and TF frames
  SmokeTestRunner – requires a live CelloRobotBridge; open/close gripper,
                    read joint state, and optionally call the MoveIt2 planner

All check results use CheckResult / PreflightReport dataclasses.
format_report() returns a human-readable multi-line summary grouped by category.

Failure categories:
  config       – YAML fields missing or logically inconsistent
  ros_bindings – topics, action servers, or services not visible in the graph
  moveit       – MoveIt2 service unreachable or planning call failed
  execution    – gripper or joint-state hardware check failed

Usage (offline config check – no ROS2 needed)::

    from toms_robot.config_loader import RobotConfig
    from toms_robot.preflight import ConfigChecker, PreflightReport, format_report

    config = RobotConfig.from_yaml_files("config/robot.yaml",
                                         "config/robots/cello_follower.yaml")
    results = ConfigChecker().check_all(config)
    report  = PreflightReport(results=results)
    print(format_report(report))

Usage (live hardware, from a ROS2 node)::

    from toms_robot.preflight import RosChecker, SmokeTestRunner

    ros  = RosChecker(node)
    results += ros.check_all(config)

    smoke = SmokeTestRunner(config, bridge)
    results += smoke.run_all(planner=planner)
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Dict, List, Optional

if TYPE_CHECKING:
    import rclpy.node

    from toms_robot.config_loader import RobotConfig

# ---------------------------------------------------------------------------
# Categories
# ---------------------------------------------------------------------------

CATEGORY_CONFIG = "config"
CATEGORY_ROS = "ros_bindings"
CATEGORY_MOVEIT = "moveit"
CATEGORY_EXECUTION = "execution"

_CATEGORY_ORDER = [CATEGORY_CONFIG, CATEGORY_ROS, CATEGORY_MOVEIT, CATEGORY_EXECUTION]

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass
class CheckResult:
    """Outcome of a single preflight check."""

    name: str
    category: str           # one of CATEGORY_* constants
    passed: bool
    message: str
    warning: bool = False   # True = passed but with a notable caveat
    details: Dict[str, Any] = field(default_factory=dict)


@dataclass
class PreflightReport:
    """Aggregated results of a full preflight run."""

    results: List[CheckResult]
    run_dir: Optional[str] = None
    timestamp: str = ""

    @property
    def passed(self) -> bool:
        """True only when every check passed (warnings are allowed)."""
        return all(r.passed for r in self.results)

    def failures(self) -> List[CheckResult]:
        return [r for r in self.results if not r.passed]

    def warnings(self) -> List[CheckResult]:
        return [r for r in self.results if r.passed and r.warning]

    def by_category(self) -> Dict[str, List[CheckResult]]:
        out: Dict[str, List[CheckResult]] = {}
        for r in self.results:
            out.setdefault(r.category, []).append(r)
        return out


# ---------------------------------------------------------------------------
# Config checker  (pure Python, no ROS2)
# ---------------------------------------------------------------------------


class ConfigChecker:
    """Validate a loaded RobotConfig for hardware-readiness.

    All checks run offline without ROS2.  Call before attempting any live
    connection to catch missing fields early.
    """

    def check_all(self, config: "RobotConfig") -> List[CheckResult]:
        """Run the full config check suite. Returns one result per check."""
        results: List[CheckResult] = []
        results += self._check_identity(config)
        results += self._check_frames(config)
        results += self._check_joint_names(config)
        results += self._check_actions(config)
        results += self._check_gripper(config)
        results += self._check_moveit_packages(config)
        results += self._check_kinematics(config)
        return results

    # ------------------------------------------------------------------
    # Individual check groups
    # ------------------------------------------------------------------

    def _check_identity(self, config: "RobotConfig") -> List[CheckResult]:
        results = []
        results.append(CheckResult(
            name="robot.model",
            category=CATEGORY_CONFIG,
            passed=True,
            message=f"robot.model = {config.model!r}  (variant={config.variant!r})",
        ))
        ok_dof = config.dof > 0
        results.append(CheckResult(
            name="robot.dof",
            category=CATEGORY_CONFIG,
            passed=ok_dof,
            message=(
                f"robot.dof = {config.dof}"
                if ok_dof
                else f"robot.dof = {config.dof!r}; must be a positive integer"
            ),
        ))
        return results

    def _check_frames(self, config: "RobotConfig") -> List[CheckResult]:
        results = []
        results.append(CheckResult(
            name="robot.base_frame",
            category=CATEGORY_CONFIG,
            passed=True,
            message=f"base_frame = {config.base_frame!r}",
        ))
        results.append(CheckResult(
            name="robot.end_effector_link",
            category=CATEGORY_CONFIG,
            passed=True,
            message=f"end_effector_link = {config.end_effector_link!r}",
        ))
        tool_set = config.tool_frame is not None
        results.append(CheckResult(
            name="robot.tool_frame",
            category=CATEGORY_CONFIG,
            passed=True,
            warning=not tool_set,
            message=(
                f"tool_frame = {config.tool_frame!r}"
                if tool_set
                else (
                    f"tool_frame is null; falling back to end_effector_link "
                    f"({config.end_effector_link!r}). "
                    "Define a TCP frame child of that link for full task support."
                )
            ),
        ))
        return results

    def _check_joint_names(self, config: "RobotConfig") -> List[CheckResult]:
        results = []
        n = len(config.joint_names)
        count_ok = n == config.dof
        results.append(CheckResult(
            name="robot.joint_names.count",
            category=CATEGORY_CONFIG,
            passed=count_ok,
            message=(
                f"joint_names has {n} entries matching dof={config.dof}"
                if count_ok
                else f"joint_names has {n} entries but dof={config.dof} – counts must match"
            ),
        ))
        has_dups = len(config.joint_names) != len(set(config.joint_names))
        results.append(CheckResult(
            name="robot.joint_names.unique",
            category=CATEGORY_CONFIG,
            passed=not has_dups,
            message=(
                f"joint_names unique: {config.joint_names}"
                if not has_dups
                else f"duplicate joint names detected: {config.joint_names}"
            ),
        ))
        return results

    def _check_actions(self, config: "RobotConfig") -> List[CheckResult]:
        return [
            CheckResult(
                name="robot.joint_state_topic",
                category=CATEGORY_CONFIG,
                passed=True,
                message=f"joint_state_topic = {config.joint_state_topic!r}",
            ),
            CheckResult(
                name="robot.trajectory_action",
                category=CATEGORY_CONFIG,
                passed=True,
                message=f"trajectory_action = {config.trajectory_action!r}",
            ),
        ]

    def _check_gripper(self, config: "RobotConfig") -> List[CheckResult]:
        results = []
        g = config.gripper
        results.append(CheckResult(
            name="robot.gripper.action",
            category=CATEGORY_CONFIG,
            passed=True,
            message=f"gripper.action = {g.action!r}  (interface={g.interface_type!r})",
        ))
        results.append(CheckResult(
            name="robot.gripper.positions",
            category=CATEGORY_CONFIG,
            passed=True,
            message=(
                f"gripper: open={g.open_position} m  close={g.close_position} m  "
                f"max_force={g.max_force} N"
            ),
        ))
        width_set = g.open_width is not None
        results.append(CheckResult(
            name="robot.gripper.open_width",
            category=CATEGORY_CONFIG,
            passed=True,
            warning=not width_set,
            message=(
                f"gripper.open_width = {g.open_width} m"
                if width_set
                else (
                    "gripper.open_width is null – needed for grasp validation. "
                    "Measure fingertip gap and set in config/robots/<name>.yaml."
                )
            ),
        ))
        return results

    def _check_moveit_packages(self, config: "RobotConfig") -> List[CheckResult]:
        return [
            CheckResult(
                name="robot.moveit_package",
                category=CATEGORY_CONFIG,
                passed=True,
                message=(
                    f"moveit_package={config.moveit_package!r}  "
                    f"planning_group={config.planning_group!r}  "
                    f"gripper_group={config.gripper_group!r}"
                ),
            ),
            CheckResult(
                name="robot.urdf_srdf",
                category=CATEGORY_CONFIG,
                passed=True,
                message=(
                    f"urdf={config.urdf_package}/{config.urdf_file}  "
                    f"srdf={config.srdf_package}/{config.srdf_file}"
                ),
            ),
        ]

    def _check_kinematics(self, config: "RobotConfig") -> List[CheckResult]:
        return [CheckResult(
            name="robot.kinematics.plugin",
            category=CATEGORY_CONFIG,
            passed=True,
            message=(
                f"plugin={config.kinematics.plugin!r}  "
                f"res={config.kinematics.search_resolution}  "
                f"timeout={config.kinematics.timeout}s"
            ),
        )]


# ---------------------------------------------------------------------------
# ROS2 checker  (requires rclpy + live graph)
# ---------------------------------------------------------------------------


class RosChecker:
    """Live ROS2 graph checks.

    All methods require a running rclpy.Node and an active ROS2 graph.
    rclpy is imported lazily inside each method so the module loads without
    ROS2 installed (offline tests and preflight_offline.py).
    """

    def __init__(self, node: "rclpy.node.Node") -> None:
        self._node = node

    def check_all(
        self,
        config: "RobotConfig",
        pre_received_topics: Optional[Dict[str, bool]] = None,
    ) -> List[CheckResult]:
        """Run the full live-graph check suite from config.

        pre_received_topics: mapping of topic name → True/False indicating whether
        a message was already received by an external subscriber (avoids reentrant
        spin_once inside a timer callback).
        """
        results: List[CheckResult] = []
        topic = config.joint_state_topic
        if pre_received_topics and topic in pre_received_topics:
            ok = pre_received_topics[topic]
            results.append(CheckResult(
                name=f"topic:{topic}",
                category=CATEGORY_ROS,
                passed=ok,
                message=(
                    f"Topic {topic!r} is publishing"
                    if ok
                    else f"No messages received on {topic!r} within the observation window"
                ),
            ))
        else:
            results.append(self.check_topic_active(topic, timeout=3.0))
        results.append(self.check_action_server(
            config.trajectory_action, "FollowJointTrajectory", timeout=5.0,
        ))
        results.append(self.check_action_server(
            config.gripper.action, "GripperCommand", timeout=5.0,
        ))
        results.append(self.check_service(
            "/plan_kinematic_path", "GetMotionPlan", timeout=5.0,
        ))
        results += self.check_tf_frames(
            [config.base_frame, config.end_effector_link], timeout=4.0,
        )
        return results

    def check_topic_active(self, topic: str, timeout: float = 3.0) -> CheckResult:
        """Subscribe and expect at least one message within *timeout* seconds."""
        try:
            import rclpy  # type: ignore[import]  # noqa: PLC0415
            from sensor_msgs.msg import (
                JointState,  # type: ignore[import]  # noqa: PLC0415
            )
        except ImportError as exc:
            return CheckResult(
                name=f"topic:{topic}",
                category=CATEGORY_ROS,
                passed=False,
                message=f"sensor_msgs not installed: {exc}",
            )

        received: List[bool] = []

        def _cb(_msg: object) -> None:
            received.append(True)

        sub = self._node.create_subscription(JointState, topic, _cb, 10)
        deadline = time.monotonic() + timeout
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._node.destroy_subscription(sub)

        ok = bool(received)
        return CheckResult(
            name=f"topic:{topic}",
            category=CATEGORY_ROS,
            passed=ok,
            message=(
                f"Topic {topic!r} is publishing"
                if ok
                else f"No messages received on {topic!r} within {timeout:.0f}s"
            ),
        )

    def check_action_server(
        self,
        action_name: str,
        action_type_label: str,
        timeout: float = 5.0,
    ) -> CheckResult:
        """Wait for an action server to become available.

        action_type_label: "FollowJointTrajectory" | "GripperCommand"
        """
        try:
            from control_msgs.action import (  # type: ignore[import]  # noqa: PLC0415
                FollowJointTrajectory,
                GripperCommand,
            )
            from rclpy.action import (
                ActionClient,  # type: ignore[import]  # noqa: PLC0415
            )
        except ImportError as exc:
            return CheckResult(
                name=f"action:{action_name}",
                category=CATEGORY_ROS,
                passed=False,
                message=f"control_msgs / rclpy.action not installed: {exc}",
            )

        _type_map = {
            "FollowJointTrajectory": FollowJointTrajectory,
            "GripperCommand": GripperCommand,
        }
        action_type = _type_map.get(action_type_label)
        if action_type is None:
            return CheckResult(
                name=f"action:{action_name}",
                category=CATEGORY_ROS,
                passed=False,
                message=f"Unknown action type label: {action_type_label!r}",
            )

        client = ActionClient(self._node, action_type, action_name)
        available = client.wait_for_server(timeout_sec=timeout)
        client.destroy()

        return CheckResult(
            name=f"action:{action_name}",
            category=CATEGORY_ROS,
            passed=available,
            message=(
                f"Action server {action_name!r} is available"
                if available
                else (
                    f"Action server {action_name!r} not found within {timeout:.0f}s. "
                    "Is ros2_control / MoveIt2 running?"
                )
            ),
        )

    def check_service(
        self,
        service_name: str,
        service_type_label: str,
        timeout: float = 5.0,
    ) -> CheckResult:
        """Wait for a ROS2 service to become available."""
        try:
            from moveit_msgs.srv import (
                GetMotionPlan,  # type: ignore[import]  # noqa: PLC0415
            )
        except ImportError as exc:
            return CheckResult(
                name=f"service:{service_name}",
                category=CATEGORY_MOVEIT,
                passed=False,
                message=f"moveit_msgs not installed: {exc}",
            )

        client = self._node.create_client(GetMotionPlan, service_name)
        available = client.wait_for_service(timeout_sec=timeout)
        self._node.destroy_client(client)

        return CheckResult(
            name=f"service:{service_name}",
            category=CATEGORY_MOVEIT,
            passed=available,
            message=(
                f"Service {service_name!r} is available"
                if available
                else (
                    f"Service {service_name!r} not found within {timeout:.0f}s. "
                    "Is MoveIt2 (demo.launch.py) running?"
                )
            ),
        )

    def check_tf_frames(
        self,
        frames: List[str],
        timeout: float = 4.0,
    ) -> List[CheckResult]:
        """Check that each frame is reachable from the first frame in the list.

        The first element is treated as the root (base_frame).
        A transform from frames[0] to itself is always valid; others check the
        full kinematic chain through robot_state_publisher.
        """
        try:
            import rclpy  # type: ignore[import]  # noqa: PLC0415
            from tf2_ros import (  # type: ignore[import]  # noqa: PLC0415
                Buffer,
                TransformListener,
            )
        except ImportError as exc:
            return [CheckResult(
                name="tf_frames",
                category=CATEGORY_ROS,
                passed=False,
                message=f"tf2_ros not available: {exc}",
            )]

        tf_buffer = Buffer()
        _listener = TransformListener(tf_buffer, self._node)  # noqa: F841  # keep alive

        # Spin briefly to populate the TF buffer
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        root = frames[0] if frames else "world"
        results = []
        for frame in frames:
            if frame == root:
                # A frame can always transform to itself
                results.append(CheckResult(
                    name=f"tf:{frame}",
                    category=CATEGORY_ROS,
                    passed=True,
                    message=f"TF root frame {frame!r} (self-transform always valid)",
                ))
                continue
            ok = tf_buffer.can_transform(root, frame, rclpy.time.Time())
            results.append(CheckResult(
                name=f"tf:{frame}",
                category=CATEGORY_ROS,
                passed=ok,
                message=(
                    f"TF {root!r} → {frame!r} available"
                    if ok
                    else (
                        f"TF {root!r} → {frame!r} not available. "
                        "Is robot_state_publisher running?"
                    )
                ),
            ))
        return results


# ---------------------------------------------------------------------------
# Smoke test runner  (requires live CelloRobotBridge)
# ---------------------------------------------------------------------------


class SmokeTestRunner:
    """Execute a minimal robot smoke-test sequence.

    Sequence (all steps independent; later steps run even if earlier ones fail):
      1. Read joint state (verify /joint_states data arrived)
      2. Open gripper
      3. Close gripper (30 % of max_force – safe for unloaded gripper)
      4. Re-open gripper (leave in safe open state)
      5. Plan smoke-test (call MoveIt2 with a probe request; tests service health)

    The plan check (step 5) is optional: pass planner=None to skip.
    The plan check passes if the service responds at all; it warns if the
    plan itself is infeasible (expected for the identity-pose probe).
    Motion is NEVER executed unless execute_motion=True is set explicitly.
    """

    def __init__(
        self,
        config: "RobotConfig",
        bridge: object,   # CelloRobotBridge with live node attached
        *,
        execute_motion: bool = False,
        node: Optional["rclpy.node.Node"] = None,
    ) -> None:
        self._config = config
        self._bridge = bridge
        self._execute_motion = execute_motion
        self._node = node   # needed for TF lookup in lift test

    def run_all(self, planner: object = None) -> List[CheckResult]:
        """Run the full smoke-test sequence. Returns one result per step."""
        results: List[CheckResult] = []
        results.append(self._check_joint_state())
        results += self._check_gripper_cycle()
        if planner is None:
            results.append(CheckResult(
                name="smoke:moveit_plan_service",
                category=CATEGORY_MOVEIT,
                passed=True,
                warning=True,
                message="MoveIt2 plan check skipped: no planner provided",
            ))
        elif self._execute_motion:
            # Real lift test – exercises plan+execute end-to-end
            results += self._check_plan_and_execute_lift(planner)
        else:
            # Cheap service-health probe (unreachable pose; expected to warn)
            results.append(self._check_plan(planner))
        results.append(self._check_move_to_home())
        return results

    def _check_plan_and_execute_lift(self, planner: object) -> List[CheckResult]:
        """Plan + execute a small straight-up lift from the current EE pose.

        Reads current base_frame→end_effector_link TF, builds a goal pose with
        +test_lift_offset_z in Z (same orientation), calls the planner, then
        executes the resulting trajectory via the bridge.

        Returns two CheckResults: one for planning, one for execution.
        """
        offset = getattr(self._config, "test_lift_offset_z", None)
        if offset is None:
            return [CheckResult(
                name="smoke:plan_and_execute_lift",
                category=CATEGORY_MOVEIT,
                passed=True,
                warning=True,
                message=(
                    "plan_and_execute_lift skipped: "
                    "fixed_pick_test.test_lift_offset_z is null in config"
                ),
            )]
        if self._node is None:
            return [CheckResult(
                name="smoke:plan_and_execute_lift",
                category=CATEGORY_MOVEIT,
                passed=False,
                message="plan_and_execute_lift: no node passed to SmokeTestRunner",
            )]

        # ---- Read current EE pose via TF ----
        try:
            import rclpy  # type: ignore[import]  # noqa: PLC0415
            from tf2_ros import (  # type: ignore[import]  # noqa: PLC0415
                Buffer,
                TransformListener,
            )
        except ImportError as exc:
            return [CheckResult(
                name="smoke:plan_and_execute_lift",
                category=CATEGORY_MOVEIT,
                passed=False,
                message=f"tf2_ros not available: {exc}",
            )]

        buf = Buffer()
        _listener = TransformListener(buf, self._node)  # noqa: F841  # keep alive
        deadline = time.monotonic() + 3.0
        base = self._config.base_frame
        ee = self._config.end_effector_link
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if buf.can_transform(base, ee, rclpy.time.Time()):
                break
        try:
            tf = buf.lookup_transform(base, ee, rclpy.time.Time())
        except Exception as exc:
            return [CheckResult(
                name="smoke:plan_and_execute_lift",
                category=CATEGORY_MOVEIT,
                passed=False,
                message=f"TF lookup {base!r}→{ee!r} failed: {exc}",
            )]

        # ---- Build goal = current pose + Z offset ----
        from toms_core.models import (  # noqa: PLC0415
            GraspCandidate,
            GraspStatus,
            PlanRequest,
            Pose,
        )
        t = tf.transform.translation
        r = tf.transform.rotation
        goal = Pose()
        goal.position.x = t.x
        goal.position.y = t.y
        goal.position.z = t.z + offset
        goal.orientation.x = r.x
        goal.orientation.y = r.y
        goal.orientation.z = r.z
        goal.orientation.w = r.w

        candidate = GraspCandidate(
            candidate_id="lift_test",
            object_id="lift_test",
            pose=goal,
            score=1.0,
            status=GraspStatus.CANDIDATE,
        )
        request = PlanRequest(
            object_id="lift_test",
            grasp_candidate=candidate,
            planning_group=self._config.planning_group,
            end_effector_link=self._config.end_effector_link,
            base_frame=self._config.base_frame,
        )

        # ---- Plan ----
        results: List[CheckResult] = []
        try:
            plan_result = planner.plan(request)  # type: ignore[attr-defined]
        except Exception as exc:
            return [CheckResult(
                name="smoke:plan_lift",
                category=CATEGORY_MOVEIT,
                passed=False,
                message=f"Planner raised exception: {exc}",
            )]

        plan_ok = getattr(plan_result, "success", False)
        results.append(CheckResult(
            name="smoke:plan_lift",
            category=CATEGORY_MOVEIT,
            passed=plan_ok,
            message=(
                f"Planned +{offset*100:.1f}cm lift from EE ({t.x:.3f},{t.y:.3f},{t.z:.3f}) "
                f"in {getattr(plan_result, 'planning_time', 0):.3f}s"
                if plan_ok
                else (
                    f"Plan FAILED (error_code={getattr(plan_result, 'error_code', '?')}) "
                    f"from EE ({t.x:.3f},{t.y:.3f},{t.z:.3f}) to z+{offset:.3f}m"
                )
            ),
        ))
        if not plan_ok:
            results.append(CheckResult(
                name="smoke:execute_lift",
                category=CATEGORY_EXECUTION,
                passed=True,
                warning=True,
                message="execute_lift skipped: planning failed",
            ))
            return results

        # ---- Execute via bridge ----
        joint_traj = getattr(plan_result.trajectory, "joint_trajectory", None)
        if joint_traj is None:
            results.append(CheckResult(
                name="smoke:execute_lift",
                category=CATEGORY_EXECUTION,
                passed=False,
                message="Planned trajectory has no joint_trajectory field",
            ))
            return results

        try:
            exec_ok = self._bridge.execute_trajectory(joint_traj)  # type: ignore[attr-defined]
        except Exception as exc:
            results.append(CheckResult(
                name="smoke:execute_lift",
                category=CATEGORY_EXECUTION,
                passed=False,
                message=f"execute_trajectory raised: {exc}",
            ))
            return results

        results.append(CheckResult(
            name="smoke:execute_lift",
            category=CATEGORY_EXECUTION,
            passed=exec_ok,
            message=(
                f"Executed +{offset*100:.1f}cm lift successfully"
                if exec_ok
                else "Trajectory execution returned False (controller rejected/aborted)"
            ),
        ))
        return results

    def _check_move_to_home(self) -> CheckResult:
        """Send a single-point trajectory to home_joints via the arm controller.

        Only runs when execute_motion=True AND config.home_joints is set.
        Otherwise returns a skip (PASS, warning) result.
        """
        home = getattr(self._config, "home_joints", None)
        if home is None:
            return CheckResult(
                name="smoke:move_to_home",
                category=CATEGORY_EXECUTION,
                passed=True,
                warning=True,
                message="move_to_home skipped: smoke_test.home_joints is null in config",
            )
        if not self._execute_motion:
            return CheckResult(
                name="smoke:move_to_home",
                category=CATEGORY_EXECUTION,
                passed=True,
                warning=True,
                message=(
                    f"move_to_home skipped: execute_motion=False. "
                    f"Target home_joints={[round(v, 4) for v in home]}."
                ),
            )
        try:
            ok = self._bridge.move_to_joint_positions(home, duration_sec=3.0)  # type: ignore[attr-defined]
        except Exception as exc:
            return CheckResult(
                name="smoke:move_to_home",
                category=CATEGORY_EXECUTION,
                passed=False,
                message=f"move_to_home exception: {exc}",
            )
        return CheckResult(
            name="smoke:move_to_home",
            category=CATEGORY_EXECUTION,
            passed=ok,
            message=(
                f"Arm reached home_joints={[round(v, 4) for v in home]}"
                if ok
                else (
                    f"move_to_home FAILED – trajectory action rejected/aborted. "
                    f"Target: {[round(v, 4) for v in home]}"
                )
            ),
            details={"home_joints": list(home)},
        )

    # ------------------------------------------------------------------

    def _check_joint_state(self) -> CheckResult:
        """Verify /joint_states data populated the bridge's cached positions."""
        positions = getattr(self._bridge, "_joint_positions", [])
        populated = any(abs(p) > 1e-9 for p in positions)
        return CheckResult(
            name="smoke:joint_state_read",
            category=CATEGORY_EXECUTION,
            # Treat all-zero as a warning (robot may genuinely be at zero)
            passed=True,
            warning=not populated,
            message=(
                f"Joint positions: {[round(p, 4) for p in positions]}"
                if populated
                else (
                    "Joint positions are all zero – /joint_states may not be publishing "
                    "or robot is at exact zero pose. Verify before proceeding."
                )
            ),
            details={"joint_positions": list(positions)},
        )

    def _check_gripper_cycle(self) -> List[CheckResult]:
        results = []

        def _read_pos() -> Optional[float]:
            """Read the cached gripper joint position after a short spin window.

            Short spin is just to let a fresh /joint_states message arrive –
            the upstream cello_controller now waits for the servo to settle
            before returning from the action, so we don't need to sleep here.
            """
            if self._node is None:
                return getattr(self._bridge, "_gripper_position", None)
            try:
                import rclpy  # type: ignore[import]  # noqa: PLC0415
            except ImportError:
                return getattr(self._bridge, "_gripper_position", None)
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            return getattr(self._bridge, "_gripper_position", None)

        def _fmt(p: Optional[float]) -> str:
            return "n/a" if p is None else f"{p:.4f} m"

        def _moved(pre: Optional[float], post: Optional[float]) -> bool:
            return (pre is not None and post is not None
                    and abs(post - pre) >= 0.002)  # ≥ 2 mm delta

        # ---- open ----
        pre = _read_pos()
        ok_open = self._bridge.open_gripper()  # type: ignore[attr-defined]
        post = _read_pos()
        results.append(CheckResult(
            name="smoke:gripper_open",
            category=CATEGORY_EXECUTION,
            passed=ok_open,
            warning=ok_open and not _moved(pre, post)
                and pre is not None and abs(pre - self._config.gripper.open_position) > 0.002,
            message=(
                f"Gripper opened: joint7_left {_fmt(pre)} → {_fmt(post)} "
                f"(target {self._config.gripper.open_position:.4f} m)"
                if ok_open
                else (
                    "open_gripper() returned False – check gripper action server "
                    f"({self._config.gripper.action}) and hardware"
                )
            ),
            details={"pre": pre, "post": post, "target": self._config.gripper.open_position},
        ))

        # ---- close ----
        pre = _read_pos()
        ok_close = self._bridge.close_gripper(  # type: ignore[attr-defined]
            target_width=0.0,
            force=self._config.gripper.max_force * 0.3,  # 30 % – safe for unloaded
        )
        post = _read_pos()
        results.append(CheckResult(
            name="smoke:gripper_close",
            category=CATEGORY_EXECUTION,
            passed=ok_close,
            warning=ok_close and not _moved(pre, post)
                and pre is not None and abs(pre - self._config.gripper.close_position) > 0.002,
            message=(
                f"Gripper closed: joint7_left {_fmt(pre)} → {_fmt(post)} "
                f"(target {self._config.gripper.close_position:.4f} m)"
                if ok_close
                else (
                    "close_gripper() returned False – check gripper action server "
                    f"({self._config.gripper.action}) and hardware"
                )
            ),
            details={"pre": pre, "post": post, "target": self._config.gripper.close_position},
        ))

        # ---- reopen ----
        pre = _read_pos()
        ok_reopen = self._bridge.open_gripper()  # type: ignore[attr-defined]
        post = _read_pos()
        results.append(CheckResult(
            name="smoke:gripper_reopen",
            category=CATEGORY_EXECUTION,
            passed=ok_reopen,
            warning=ok_reopen and not _moved(pre, post)
                and pre is not None and abs(pre - self._config.gripper.open_position) > 0.002,
            message=(
                f"Gripper re-opened: joint7_left {_fmt(pre)} → {_fmt(post)} "
                f"(target {self._config.gripper.open_position:.4f} m)"
                if ok_reopen
                else (
                    "Re-open gripper FAILED – gripper may be stuck. "
                    "Inspect manually before continuing."
                )
            ),
            details={"pre": pre, "post": post, "target": self._config.gripper.open_position},
        ))
        return results

    def _check_plan(self, planner: object) -> CheckResult:
        """Call MoveIt2 with a smoke-test probe request to verify service health.

        The goal pose (identity) is intentionally unreachable; we only check
        that the service responds.  A planning failure is a WARNING, not a FAIL.
        Motion is never executed here.
        """
        from toms_core.models import (  # noqa: PLC0415
            GraspCandidate,
            GraspStatus,
            PlanRequest,
            Pose,
        )

        candidate = GraspCandidate(
            candidate_id="preflight_probe",
            object_id="preflight_probe",
            pose=Pose(),          # identity – likely infeasible; service health only
            score=1.0,
            status=GraspStatus.CANDIDATE,
        )
        request = PlanRequest(
            object_id="preflight_probe",
            grasp_candidate=candidate,
            planning_group=self._config.planning_group,
            end_effector_link=self._config.end_effector_link,
            base_frame=self._config.base_frame,
        )

        try:
            result = planner.plan(request)  # type: ignore[attr-defined]
            # Service responded – pass regardless of planning outcome
            plan_ok = getattr(result, "success", False)
            return CheckResult(
                name="smoke:moveit_plan_service",
                category=CATEGORY_MOVEIT,
                passed=True,
                warning=not plan_ok,
                message=(
                    "MoveIt2 service responded; plan "
                    + ("SUCCEEDED" if plan_ok else f"FAILED ({getattr(result, 'error_code', '?')})")
                    + " (probe pose was intentional – service health confirmed)"
                ),
                details={"planning_time": getattr(result, "planning_time", None)},
            )
        except Exception as exc:
            return CheckResult(
                name="smoke:moveit_plan_service",
                category=CATEGORY_MOVEIT,
                passed=False,
                message=f"MoveIt2 service error: {exc}",
            )


# ---------------------------------------------------------------------------
# Report formatting
# ---------------------------------------------------------------------------

_WIDTH = 72
_LABEL: Dict[bool, str] = {True: "PASS", False: "FAIL"}


def format_report(report: PreflightReport) -> str:
    """Return a human-readable preflight report grouped by category."""
    lines = []
    lines.append("=" * _WIDTH)
    lines.append("TOMS PREFLIGHT REPORT")
    if report.timestamp:
        lines.append(f"  {report.timestamp}")
    if report.run_dir:
        lines.append(f"  Artifacts: {report.run_dir}")
    lines.append("=" * _WIDTH)

    by_cat = report.by_category()
    for cat in _CATEGORY_ORDER:
        checks = by_cat.get(cat, [])
        if not checks:
            continue
        lines.append(f"\n[{cat.upper()}]")
        for r in checks:
            label = "WARN" if (r.passed and r.warning) else _LABEL[r.passed]
            lines.append(f"  {label:<4}  {r.name}")
            lines.append(f"        {r.message}")

    # Summary
    failures = report.failures()
    warnings = report.warnings()
    lines.append("")
    lines.append("─" * _WIDTH)
    if not failures:
        suffix = f" ({len(warnings)} warning(s))" if warnings else ""
        lines.append(f"RESULT: ALL {len(report.results)} CHECKS PASSED{suffix}")
    else:
        lines.append(
            f"RESULT: {len(failures)} FAILURE(S) out of {len(report.results)} checks"
        )
        lines.append("")
        lines.append("FAILURES:")
        for r in failures:
            lines.append(f"  [{r.category}]  {r.name}")
            lines.append(f"    {r.message}")
    lines.append("=" * _WIDTH)
    return "\n".join(lines)
