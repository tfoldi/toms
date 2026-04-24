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
