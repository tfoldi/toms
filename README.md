# TOMS – Task-Oriented Manipulation System

Behavior-tree-driven tabletop robot manipulation.  Initial task: **pens → pen holder**.

## Architecture

```
perception → world state → behavior tree → pose generation → execution → validation → retry
```

All execution goes through the behavior tree.  No hidden state; no fake autonomy.

## Packages

| Package | Role |
|---|---|
| `toms_core` | Typed data models (no ROS2 dep) |
| `toms_bt` | BT runtime, all action/condition nodes, pen-to-holder template |
| `toms_world` | Canonical WorldState manager, history |
| `toms_perception` | Perception adapter (mock + abstract base) |
| `toms_planning` | Grasp generation, IK feasibility, MoveIt2 wrapper |
| `toms_execution` | Pick/place execution sequences |
| `toms_validation` | Grasp + placement validators with structured failures |
| `toms_robot` | Robot bridge (mock + abstract base) |
| `toms_logging` | JSONL structured logging + replay |

## Quickstart (simulation)

```bash
# Build
cd /path/to/ros2_ws
colcon build --packages-select toms_core toms_bt toms_world \
  toms_perception toms_planning toms_execution toms_validation \
  toms_robot toms_logging
source install/setup.bash

# Run tests
colcon test --packages-select toms_core toms_bt toms_world toms_validation

# Launch (mock adapters, no hardware)
ros2 launch toms_bt toms_sim.launch.py
```

## Hardware bring-up

Before running on real hardware, fill in **all TODO fields** in:
- `config/robot.yaml`   – arm model, topics, frames, gripper
- `config/perception.yaml` – real detection backend
- `config/planning.yaml` – confirmed planner and thresholds

Then run: `ros2 launch toms_bt toms_hardware.launch.py`

See `launch/toms_hardware.launch.py` for the TODO checklist.

## Behavior tree structure

```
Sequence("Task_PenToHolder")
  UpdateWorldState          ← initial perception + robot sync
  FindContainer             ← locate pen holder
  RepeatUntilFailure        ← loop until no more pens
    Sequence("PickPlaceOnce")
      UpdateWorldState      ← refresh before each object
      SelectNextObject      ← pick highest-confidence pending pen
      PickObject            ← generate grasps → IK → plan → exec → validate
      PlaceObjectInContainer ← place pose → plan → exec → validate
      MarkObjectComplete    ← bookkeeping
  ReportOutcome             ← emit TaskOutcome to logger
```

## Key principles

1. **World state is canonical** – all modules read/write through `WorldStateManager`
2. **Validation is mandatory** – every action has post-conditions; no "assume success"
3. **Retry is first-class** – failures branch in the BT; they do not crash
4. **Config-driven** – no hardcoded robot topics, joint names, or frame IDs
5. **Local-first** – no cloud dependency in the control loop

## What is still needed from hardware

See [AGENTS.md](AGENTS.md) "Arm Integration" section.  Summary:
- Robot model, DOF, planning group, end-effector link, base frame
- Joint state topic, trajectory action server
- Gripper interface type and topics
- Camera topics and frame IDs
- Hand-eye calibration result
- MoveIt2 package, URDF/SRDF paths, kinematics plugin
