#!/usr/bin/env python3
"""scripts/goto_captured_poses.py – diagnose pick-and-place plan failures.

Standalone test that ONLY moves to the captured object_pose and place_pose
from config — no offsets, no derived waypoints.  Use this to isolate
"can MoveIt plan to the literal captured poses?" from "is the offset
arithmetic correct?".

Sequence:
    1. Move to home_joints (joint-space, interpolated)
    2. Plan + execute to object_pose
    3. Move to home_joints
    4. Plan + execute to place_pose
    5. Move to home_joints

Each step prints PASS/FAIL with planner error code on failure.

Two orientation modes (--orientation):
    config  (default) – use the captured quaternion verbatim
    tf      – use the current TF orientation (matches smoke test trick)

If config-orientation FAILS but tf-orientation PASSES, the captured
quaternions in YAML have drifted from URDF FK and exceed the planner's
goal_orientation_tolerance (0.01 rad ≈ 0.6°).  Either bump the tolerance
in planning.yaml or recapture the poses with `tf2_echo` after settling.

Usage:
    source /opt/ros/humble/setup.bash
    source ~/Developer/ros2/starai_ws/install/setup.bash
    PYTHONPATH=src/toms_core:src/toms_robot:src/toms_planning:$PYTHONPATH \\
        python3 scripts/goto_captured_poses.py [--orientation tf]
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import List, Optional

_REPO_ROOT = Path(__file__).resolve().parent.parent
for sub in ("toms_core", "toms_robot", "toms_planning"):
    p = _REPO_ROOT / "src" / sub
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))


def _resolve_paths(robot: str, config_dir: str) -> List[str]:
    base = Path(config_dir) / "robot.yaml"
    overlay = Path(config_dir) / "robots" / f"{robot}.yaml"
    paths = []
    if base.exists():
        paths.append(str(base))
    if overlay.exists():
        paths.append(str(overlay))
    if not paths:
        raise SystemExit(f"ERROR: no config files under {config_dir!r} for {robot!r}")
    return paths


def _read_tf_orientation(node, base_frame, ee_link):
    """Returns (qx,qy,qz,qw) from current TF or None on failure."""
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
        tf = buf.lookup_transform(base_frame, ee_link, rclpy.time.Time())
    except Exception as exc:
        print(f"        TF lookup failed: {exc}")
        return None
    r = tf.transform.rotation
    return (r.x, r.y, r.z, r.w)


def _plan_to_pose(planner, bridge, position_xyz, orientation_xyzw, label):
    """Plan to (position, orientation) and execute.  Returns True/False."""
    from toms_core.models import GraspCandidate, GraspStatus, Pose, PlanRequest
    goal = Pose()
    goal.position.x, goal.position.y, goal.position.z = position_xyz
    (goal.orientation.x, goal.orientation.y,
     goal.orientation.z, goal.orientation.w) = orientation_xyzw

    config = bridge._config   # noqa: SLF001
    candidate = GraspCandidate(
        candidate_id=label, object_id=label, pose=goal,
        score=1.0, status=GraspStatus.CANDIDATE,
    )
    request = PlanRequest(
        object_id=label, grasp_candidate=candidate,
        planning_group=config.planning_group,
        end_effector_link=config.end_effector_link,
        base_frame=config.base_frame,
    )
    result = planner.plan(request)
    if not result.success:
        print(f"        Plan FAILED  error_code={result.error_code}  "
              f"target=({position_xyz[0]:+.3f},{position_xyz[1]:+.3f},"
              f"{position_xyz[2]:+.3f})  q=({orientation_xyzw})")
        return False
    return bridge.execute_trajectory(result.trajectory.joint_trajectory)


def _step(label, fn) -> bool:
    print(f"  [STEP] {label} ...")
    t0 = time.monotonic()
    try:
        ok = fn()
    except Exception as exc:
        elapsed = time.monotonic() - t0
        print(f"  [FAIL] {label}: {exc} ({elapsed:.2f}s)")
        return False
    elapsed = time.monotonic() - t0
    if ok is False:
        print(f"  [FAIL] {label}  ({elapsed:.2f}s)")
        return False
    print(f"  [OK]   {label}  ({elapsed:.2f}s)")
    time.sleep(0.5)
    return True


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--orientation", choices=["config", "tf"], default="config",
                        help="Source for goal orientation (default: config)")
    parser.add_argument("--robot", default="cello_follower")
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--skip-place", action="store_true",
                        help="Test only object_pose, skip place_pose")
    args = parser.parse_args(argv)

    paths = _resolve_paths(args.robot, args.config_dir)
    print(f"Loading config from: {paths}")
    from toms_robot.config_loader import RobotConfig   # noqa: PLC0415
    config = RobotConfig.from_yaml_files(*paths)

    if config.home_joints is None:
        raise SystemExit("ERROR: smoke_test.home_joints is null")
    if config.object_pose is None:
        raise SystemExit("ERROR: fixed_pick_test.object_pose is null")
    if config.place_pose is None and not args.skip_place:
        raise SystemExit("ERROR: fixed_pick_test.place_pose is null "
                         "(use --skip-place to test only object_pose)")

    print(f"\nOrientation source: {args.orientation}")
    print(f"home_joints:  {[round(v, 4) for v in config.home_joints]}")
    print(f"object_pose:  pos={config.object_pose.position}  "
          f"q={config.object_pose.orientation}")
    if config.place_pose:
        print(f"place_pose:   pos={config.place_pose.position}  "
              f"q={config.place_pose.orientation}")
    print()

    try:
        import rclpy   # noqa: PLC0415
    except ImportError:
        print("ERROR: rclpy not found. Source ROS2 first.")
        return 2
    rclpy.init()
    node = rclpy.create_node("toms_goto_captured_poses")

    from toms_robot.cello_bridge import CelloRobotBridge   # noqa: PLC0415
    from toms_planning.moveit_wrapper import (             # noqa: PLC0415
        MoveIt2RosPlanner, PlanningConfig,
    )
    bridge = CelloRobotBridge(config, node=node)
    planner = MoveIt2RosPlanner(config, PlanningConfig(), node=node)

    print("Waiting 2s for /joint_states ...")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    summary = []   # list of (label, ok)

    try:
        # ---- Step 1: home ----
        ok = _step("1. Move to home_joints",
                   lambda: bridge.move_to_joint_positions(config.home_joints,
                                                          duration_sec=4.0))
        summary.append(("home_joints (start)", ok))
        if not ok:
            return 1

        # ---- Step 2: object_pose ----
        if args.orientation == "tf":
            ori = _read_tf_orientation(node, config.base_frame,
                                       config.end_effector_link)
            if ori is None:
                summary.append(("object_pose", False))
                return 1
            label_suffix = " (TF orientation)"
        else:
            ori = config.object_pose.orientation
            label_suffix = " (config orientation)"

        ok = _step(f"2. Plan + execute to object_pose{label_suffix}",
                   lambda: _plan_to_pose(planner, bridge,
                                         config.object_pose.position,
                                         ori, "object_pose_test"))
        summary.append(("object_pose", ok))
        # Continue to test place_pose even if object_pose failed; both
        # data points are useful.

        # ---- Step 3: home ----
        ok = _step("3. Move to home_joints",
                   lambda: bridge.move_to_joint_positions(config.home_joints,
                                                          duration_sec=4.0))
        summary.append(("home_joints (mid)", ok))
        if not ok:
            return 1

        # ---- Step 4: place_pose (optional) ----
        if config.place_pose and not args.skip_place:
            if args.orientation == "tf":
                ori = _read_tf_orientation(node, config.base_frame,
                                           config.end_effector_link)
                if ori is None:
                    summary.append(("place_pose", False))
                else:
                    ok = _step(f"4. Plan + execute to place_pose{label_suffix}",
                               lambda: _plan_to_pose(planner, bridge,
                                                     config.place_pose.position,
                                                     ori, "place_pose_test"))
                    summary.append(("place_pose", ok))
            else:
                ori = config.place_pose.orientation
                ok = _step(f"4. Plan + execute to place_pose{label_suffix}",
                           lambda: _plan_to_pose(planner, bridge,
                                                 config.place_pose.position,
                                                 ori, "place_pose_test"))
                summary.append(("place_pose", ok))

            # ---- Step 5: home ----
            ok = _step("5. Move to home_joints",
                       lambda: bridge.move_to_joint_positions(config.home_joints,
                                                              duration_sec=4.0))
            summary.append(("home_joints (end)", ok))

    finally:
        node.destroy_node()
        rclpy.shutdown()

    # ---- Summary ----
    print("\n────  SUMMARY  ──────────────────────────────────────────────────────")
    for label, ok in summary:
        print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    print("─────────────────────────────────────────────────────────────────────\n")
    failures = [lbl for lbl, ok in summary if not ok]
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
