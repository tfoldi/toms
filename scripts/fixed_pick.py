#!/usr/bin/env python3
"""scripts/fixed_pick.py – Linear pick-and-place sequence using fixed poses.

Reads `home_joints`, `object_pose`, `place_pose`, and the approach/retreat
offsets from config/robots/<robot>.yaml and runs a hardcoded grasp+place
sequence.  No perception, no behaviour tree, no retries – this is the
hardware bring-up validation that comes before integrating either of those.

Usage:
    # See the planned sequence without commanding hardware:
    python3 scripts/fixed_pick.py --dry-run

    # Run for real (driver.launch.py and move_group must be up):
    source /opt/ros/humble/setup.bash
    source ~/Developer/ros2/starai_ws/install/setup.bash
    # Append (don't clobber) PYTHONPATH so ROS2 packages stay visible:
    PYTHONPATH=src/toms_core:src/toms_robot:src/toms_planning:$PYTHONPATH \\
        python3 scripts/fixed_pick.py

Sequence:
    1. home_joints                 (joint-space, interpolated 25 wp / 4 s)
    2. pre_grasp = object + Z      (planner: /plan_kinematic_path)
    3. grasp pose                  (cartesian descent)
    4. close gripper               (50 % of max_force)
    5. retreat = grasp + Z         (cartesian ascent)
    6. pre_place = place + Z       (planner: large swing across workspace)
    7. place pose                  (cartesian descent)
    8. open gripper                (release)
    9. post_place = place + Z      (cartesian ascent)
   10. home_joints                 (joint-space)

Bails on the first failure with a clear message.  Use --dry-run first to
sanity-check the computed waypoint poses.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional

# Ensure src/toms_* are importable when the script is run from repo root
_REPO_ROOT = Path(__file__).resolve().parent.parent
for sub in ("toms_core", "toms_robot", "toms_planning"):
    p = _REPO_ROOT / "src" / sub
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------


def _pose_from_config(pc) -> "Pose":  # noqa: F821 (forward ref)
    from toms_core.models import Pose
    p = Pose()
    p.position.x, p.position.y, p.position.z = pc.position
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = pc.orientation
    return p


def _pose_with_z_offset(p, dz: float) -> "Pose":  # noqa: F821
    from toms_core.models import Pose
    out = Pose()
    out.position.x = p.position.x
    out.position.y = p.position.y
    out.position.z = p.position.z + dz
    out.orientation.x = p.orientation.x
    out.orientation.y = p.orientation.y
    out.orientation.z = p.orientation.z
    out.orientation.w = p.orientation.w
    return out


def _fmt_pose(p) -> str:
    return (f"({p.position.x:+.3f}, {p.position.y:+.3f}, {p.position.z:+.3f}) "
            f"q=({p.orientation.x:+.3f}, {p.orientation.y:+.3f}, "
            f"{p.orientation.z:+.3f}, {p.orientation.w:+.3f})")


# ---------------------------------------------------------------------------
# Plan structure
# ---------------------------------------------------------------------------


@dataclass
class PickPlan:
    """All poses needed to run the fixed pick-and-place."""
    home_joints: List[float]
    pre_grasp: object   # toms_core.models.Pose
    grasp: object
    retreat: object
    pre_place: object
    place: object
    post_place: object
    grasp_force: float


def build_plan(config) -> PickPlan:
    """Translate RobotConfig into the Pose objects we'll send."""
    if config.home_joints is None:
        raise SystemExit("ERROR: smoke_test.home_joints is null in config")
    if config.object_pose is None:
        raise SystemExit("ERROR: fixed_pick_test.object_pose is null in config")
    if config.place_pose is None:
        raise SystemExit("ERROR: fixed_pick_test.place_pose is null in config")
    if config.pre_grasp_offset_z is None or config.retreat_offset_z is None:
        raise SystemExit("ERROR: pre_grasp_offset_z / retreat_offset_z are null")

    grasp = _pose_from_config(config.object_pose)
    place = _pose_from_config(config.place_pose)
    return PickPlan(
        home_joints=list(config.home_joints),
        pre_grasp=_pose_with_z_offset(grasp, config.pre_grasp_offset_z),
        grasp=grasp,
        retreat=_pose_with_z_offset(grasp, config.retreat_offset_z),
        pre_place=_pose_with_z_offset(place, config.pre_grasp_offset_z),
        place=place,
        post_place=_pose_with_z_offset(place, config.retreat_offset_z),
        grasp_force=config.gripper.max_force * 0.5,
    )


def print_plan(plan: PickPlan) -> None:
    print("\n────  FIXED PICK PLAN  ────────────────────────────────────────────")
    print(f"  home_joints = {[round(v, 4) for v in plan.home_joints]}")
    print(f"  pre_grasp    = {_fmt_pose(plan.pre_grasp)}")
    print(f"  grasp        = {_fmt_pose(plan.grasp)}")
    print(f"  retreat      = {_fmt_pose(plan.retreat)}")
    print(f"  pre_place    = {_fmt_pose(plan.pre_place)}")
    print(f"  place        = {_fmt_pose(plan.place)}")
    print(f"  post_place   = {_fmt_pose(plan.post_place)}")
    print(f"  grasp_force  = {plan.grasp_force:.2f} N "
          f"(50 % of max_force)")
    print("─────────────────────────────────────────────────────────────────────\n")


# ---------------------------------------------------------------------------
# Step runner – bail on first failure
# ---------------------------------------------------------------------------


_STEP_PAUSE = 0.5  # seconds between steps for visual confirmation


def _run_step(label: str, fn: Callable[[], object]) -> None:
    """Execute fn() and bail with sys.exit(1) on False / exception."""
    print(f"  [STEP] {label} ...")
    t0 = time.monotonic()
    try:
        result = fn()
    except Exception as exc:
        print(f"  [FAIL] {label}: {exc}")
        sys.exit(1)
    elapsed = time.monotonic() - t0
    if result is False:
        print(f"  [FAIL] {label}: returned False ({elapsed:.2f}s)")
        sys.exit(1)
    print(f"  [OK]   {label} ({elapsed:.2f}s)")
    time.sleep(_STEP_PAUSE)


# ---------------------------------------------------------------------------
# Plan / execute helpers
# ---------------------------------------------------------------------------


def _read_current_ee_pose(node, base_frame: str, ee_link: str):
    """Look up the current TF base→ee.  Returns the geometry_msgs Transform or None."""
    import rclpy   # noqa: PLC0415
    from tf2_ros import Buffer, TransformListener   # noqa: PLC0415
    buf = Buffer()
    _listener = TransformListener(buf, node)  # noqa: F841
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform(base_frame, ee_link, rclpy.time.Time()):
            break
    try:
        return buf.lookup_transform(base_frame, ee_link, rclpy.time.Time())
    except Exception as exc:
        print(f"        TF lookup {base_frame!r}→{ee_link!r} failed: {exc}")
        return None


def _pose_from_tf(tf):
    """Convert a geometry_msgs/TransformStamped to a toms_core.models.Pose."""
    from toms_core.models import Pose
    p = Pose()
    p.position.x = tf.transform.translation.x
    p.position.y = tf.transform.translation.y
    p.position.z = tf.transform.translation.z
    p.orientation.x = tf.transform.rotation.x
    p.orientation.y = tf.transform.rotation.y
    p.orientation.z = tf.transform.rotation.z
    p.orientation.w = tf.transform.rotation.w
    return p


def _cartesian_from_current(bridge, node, target_pose, velocity_scale: float = 0.2) -> bool:
    """TF-read current EE pose, then cartesian-execute [current, target]."""
    config = bridge._config   # noqa: SLF001
    tf = _read_current_ee_pose(node, config.base_frame, config.end_effector_link)
    if tf is None:
        return False
    start = _pose_from_tf(tf)
    return bridge.execute_cartesian_move([start, target_pose], velocity_scale=velocity_scale)


def _plan_to_position_with_current_orientation(planner, bridge, node,
                                               target_pose, label: str) -> bool:
    """Joint-space plan using *current* EE orientation, not the config orientation.

    Avoids the orientation-tolerance mismatch between captured YAML poses and
    URDF forward-kinematics.  The captured XY/Z position is still respected.
    """
    config = bridge._config   # noqa: SLF001
    tf = _read_current_ee_pose(node, config.base_frame, config.end_effector_link)
    if tf is None:
        return False

    from toms_core.models import GraspCandidate, GraspStatus, Pose, PlanRequest
    goal = Pose()
    goal.position.x = target_pose.position.x
    goal.position.y = target_pose.position.y
    goal.position.z = target_pose.position.z
    goal.orientation.x = tf.transform.rotation.x
    goal.orientation.y = tf.transform.rotation.y
    goal.orientation.z = tf.transform.rotation.z
    goal.orientation.w = tf.transform.rotation.w

    candidate = GraspCandidate(
        candidate_id=label,
        object_id=label,
        pose=goal,
        score=1.0,
        status=GraspStatus.CANDIDATE,
    )
    request = PlanRequest(
        object_id=label,
        grasp_candidate=candidate,
        planning_group=config.planning_group,
        end_effector_link=config.end_effector_link,
        base_frame=config.base_frame,
    )
    result = planner.plan(request)
    if not result.success:
        print(f"        Plan failed (error_code={result.error_code})")
        return False
    return bridge.execute_trajectory(result.trajectory.joint_trajectory)


# ---------------------------------------------------------------------------
# ROS2 setup + sequence execution
# ---------------------------------------------------------------------------


def _setup_ros(config):
    """Init rclpy, build node + bridge + planner, wait for joint state."""
    try:
        import rclpy   # noqa: PLC0415
    except ImportError:
        print(
            "\nERROR: rclpy not found.  Source ROS2 before running, and append\n"
            "(don't clobber) PYTHONPATH so the rclpy paths stay visible:\n"
            "    source /opt/ros/humble/setup.bash\n"
            "    source ~/Developer/ros2/starai_ws/install/setup.bash\n"
            "    PYTHONPATH=src/toms_core:src/toms_robot:src/toms_planning:$PYTHONPATH \\\n"
            "        python3 scripts/fixed_pick.py\n"
            "Use --dry-run to inspect the plan without ROS2."
        )
        sys.exit(2)
    rclpy.init()
    node = rclpy.create_node("toms_fixed_pick")

    # Subscribe to /joint_states by creating the bridge (it sets up its own sub)
    from toms_robot.cello_bridge import CelloRobotBridge   # noqa: PLC0415
    from toms_planning.moveit_wrapper import (             # noqa: PLC0415
        MoveIt2RosPlanner,
        PlanningConfig,
    )
    bridge = CelloRobotBridge(config, node=node)
    planner = MoveIt2RosPlanner(config, PlanningConfig(), node=node)

    # Spin briefly so joint_states cache populates
    print("  Waiting for /joint_states (2 s) ...")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if all(abs(v) < 1e-9 for v in bridge._joint_positions):  # noqa: SLF001
        print("  WARNING: all joint positions still 0.0 – /joint_states may be stale")
    else:
        print(f"  Live joint_positions: "
              f"{[round(v, 3) for v in bridge._joint_positions]}")  # noqa: SLF001
    return rclpy, node, bridge, planner


def _run_sequence(bridge, planner, node, plan: PickPlan) -> None:
    """The 10-step pick-and-place itself.

    Notes on planner choice per step:
      * Steps 2/3/5/7/9 are short (10 cm) cartesian segments – go through
        /compute_cartesian_path with the actual current EE pose as the start.
        Avoids the orientation-tolerance mismatch between captured YAML pose
        and URDF FK that bites /plan_kinematic_path.
      * Step 6 is the only large free-space motion (≈64 cm cross-workspace).
        Planner is fed the current TF orientation as the goal orientation,
        same workaround the smoke test's plan_lift uses.
    """
    cv = 0.2   # cartesian velocity scale – conservative for first run
    home_dur = 4.0

    print("\n────  EXECUTING  ────────────────────────────────────────────────────\n")

    _run_step("1. Move to home_joints",
              lambda: bridge.move_to_joint_positions(plan.home_joints, duration_sec=home_dur))

    # Joint-space plan: cartesian fails at this offset because lifting straight
    # up from home_joints hits a joint limit halfway.  Joint-space can curve.
    _run_step("2. Plan to pre_grasp (joint-space, TF orientation)",
              lambda: _plan_to_position_with_current_orientation(
                  planner, bridge, node, plan.pre_grasp, "pre_grasp"))

    _run_step("3. Cartesian descent to grasp",
              lambda: bridge.execute_cartesian_move(
                  [plan.pre_grasp, plan.grasp], velocity_scale=cv))

    _run_step("4. Close gripper (50 % max_force)",
              lambda: bridge.close_gripper(target_width=0.0, force=plan.grasp_force))

    _run_step("5. Cartesian retreat (lift)",
              lambda: bridge.execute_cartesian_move(
                  [plan.grasp, plan.retreat], velocity_scale=cv))

    _run_step("6. Plan + execute to pre_place (big swing)",
              lambda: _plan_to_position_with_current_orientation(
                  planner, bridge, node, plan.pre_place, "pre_place"))

    _run_step("7. Cartesian descent to place",
              lambda: bridge.execute_cartesian_move(
                  [plan.pre_place, plan.place], velocity_scale=cv))

    _run_step("8. Open gripper (release)",
              lambda: bridge.open_gripper())

    _run_step("9. Cartesian retreat from place",
              lambda: bridge.execute_cartesian_move(
                  [plan.place, plan.post_place], velocity_scale=cv))

    _run_step("10. Move to home_joints",
              lambda: bridge.move_to_joint_positions(plan.home_joints, duration_sec=home_dur))

    print("\n────  PICK COMPLETE  ────────────────────────────────────────────────")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def _resolve_config_paths(robot: str, config_dir: str) -> List[str]:
    base = Path(config_dir) / "robot.yaml"
    overlay = Path(config_dir) / "robots" / f"{robot}.yaml"
    paths: List[str] = []
    if base.exists():
        paths.append(str(base))
    if overlay.exists():
        paths.append(str(overlay))
    if not paths:
        raise SystemExit(
            f"ERROR: no config files found under {config_dir!r} for robot={robot!r}"
        )
    return paths


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the plan and exit without commanding hardware.")
    parser.add_argument("--robot", default="cello_follower",
                        help="Robot overlay name under config/robots/")
    parser.add_argument("--config-dir", default="config",
                        help="Path to config directory (default: ./config)")
    args = parser.parse_args(argv)

    paths = _resolve_config_paths(args.robot, args.config_dir)
    print(f"Loading config from: {paths}")
    from toms_robot.config_loader import RobotConfig   # noqa: PLC0415
    config = RobotConfig.from_yaml_files(*paths)

    plan = build_plan(config)
    print_plan(plan)

    if args.dry_run:
        print("DRY RUN – no hardware commands sent.\n")
        return 0

    rclpy, node, bridge, planner = _setup_ros(config)
    try:
        _run_sequence(bridge, planner, node, plan)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
