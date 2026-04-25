# TOMS follow-ups

Running list of things deferred during bring-up. Short-form — bullet per item,
note the why and the blocker if any.

## Hardware bring-up

- **Gripper settle polling in the bridge** (defensive)
  Upstream fix landed in the cello fork (`fix/gripper-settle-time`: waits
  `travel_time_ms + 100 ms` before returning action success). TOMS should
  still add a client-side settle poll in `_send_gripper_command()` so we
  don't break against older starai builds or non-cello controllers.
  Shape: after action result, poll `_gripper_position` until `abs(pos -
  target) < tolerance` or `timeout`; return False on timeout.
  Config: `gripper.settle_tolerance_m`, `gripper.settle_timeout_s`.

- **Tune gripper controller PID / verify `effort` reporting**
  We use `max_effort` as the close force in `close_gripper()`, but the
  returned `effort` field is always 0 right now (see the fix above).
  Need this reliable before `GraspValidator` can distinguish "gripped
  something" from "gripped air" via force feedback.

- **Verify `FollowJointTrajectory` `goal_time_tolerance`**
  The controller currently accepts all trajectories and reports success
  at the end. Confirm what happens when a trajectory is physically
  infeasible (hit a limit, exceed velocity) — does it abort or silently
  truncate? Blocker for reliable error reporting during pick.

## Config / calibration

- **gripper.open_width** in `config/robots/cello_follower.yaml`
  Measure fingertip-to-fingertip distance at `open_position=0.0` with
  calipers; set the value. Blocker for grasp validation.

- **tool_frame** in `config/robots/cello_follower.yaml`
  Define a TCP frame (URDF joint from `link6` to fingertip midpoint),
  publish from `robot_state_publisher`. Cleaner than using `link6` as
  the grasp reference point.

- **Hand-eye calibration**
  `calibration.method` and `calibration.transform_file` are both null.
  Not needed for fixed-pose picking. Required before perception.

- **Revisit `pre_grasp_offset_z`** (currently 5 cm)
  Capped at 5 cm because lifting 10 cm from the current `home_joints`
  exceeds the cello's reachable workspace from that config (both
  joint-space and cartesian planners refuse). When we have a different
  home pose with more headroom (e.g. arm pointing forward instead of
  folded back), increase to 8–10 cm for a safer pre-grasp hover.

## fixed_pick.py – debugging step 2

`scripts/fixed_pick.py` step 2 still fails with `error_code=99999`
(generic FAILURE) even at 5 cm offset, while the smoke test's
`plan_lift` succeeds at the same offset.  Hypothesis (untested):

The smoke test uses `current TF EE position + Z offset` as the goal.
Fixed_pick uses `config'd object_pose.position + Z offset`.  After
`move_to_home`, the actual EE position drifts from the captured
`object_pose` by a few mm (cello_controller is open-loop, no
goal-position verification).  The captured orientation also has slight
drift from URDF FK at `home_joints`.  Together those exceed the
planner's `goal_position_tolerance=0.005 m` / `goal_orientation_tolerance=0.01 rad`.

Next things to try (in order):
1. Increase `goal_position_tolerance` to 0.01 m and `goal_orientation_tolerance`
   to 0.05 rad in `planning.yaml` — likely to fix without other changes.
2. Read CURRENT EE pose via TF after move_to_home, compute pre_grasp =
   `current + Z`.  Same trick as the smoke test.  Loses "absolute pen
   position" semantics but matches what the smoke test proved works.
3. Populate `start_state` explicitly in the GetMotionPlan request from
   the cached `_joint_positions` instead of leaving RobotState() empty.
   move_group's planning_scene_monitor may have a sync lag.
4. Capture move_group's terminal output to see the actual rejection
   reason (99999 is generic — internal logs usually have detail).

## MoveIt2

- **Implement `execute_cartesian_move`** in `cello_bridge.py`
  Currently raises `NotImplementedError`. Use the
  `/compute_cartesian_path` service. Needed for straight-line approach
  and retreat during grasp.

- **Investigate default planner selection**
  `planning.moveit2.planner_id = ""` uses MoveIt2's default (OMPL
  RRTConnect). Worth comparing against Pilz LIN for straight-line
  motions during approach.

## Upstream (starai fork, branch `fix/gripper-settle-time`)

- [x] Gripper action waits for servo settle + populates result fields
- [ ] Swap `"open"` / `"close"` SRDF group state labels to match physical
      behaviour (cosmetic; no functional impact on TOMS since we don't
      use SRDF named states). Could be bundled into the same PR.

## Test coverage

- **Integration test for `move_to_joint_positions`** — currently covered
  only by live hardware smoke test. Consider a rosbag-driven fake-node
  fixture that exercises `_send_joint_trajectory` end-to-end.
