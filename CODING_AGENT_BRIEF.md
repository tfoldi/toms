# Coding Agent Brief

## Role
You are implementing a ROS2 Python-first manipulation stack for a tabletop robot arm.

Your job is to scaffold and implement a modular, inspectable system for this execution flow:

perception -> world state -> behavior tree -> pose generation -> execution -> validation -> retry

Initial task:
- Put my pens into my pen holder

## What matters most
- Clear package boundaries
- Typed interfaces
- Canonical world state
- Behavior tree runtime
- Validation and retries as first-class concepts
- Swap-friendly grasp generation backend
- Local-first execution

## First implementation pass
Create:
1. ROS2 workspace/package layout
2. Typed data models
3. Config files
4. Behavior tree node interfaces
5. Pen-to-holder BT template
6. Placeholder perception adapter
7. Placeholder robot bridge
8. MoveIt planning interface wrapper
9. Validator scaffolding
10. Logging/replay scaffolding
11. Unit and integration tests
12. Example launch files
13. Readme files per package

## Do not do these things
- Do not implement fake “AI planner” logic that bypasses BT/runtime
- Do not rely on cloud APIs
- Do not create an end-to-end policy interface
- Do not hide important assumptions in comments only
- Do not guess concrete robot topics/actions unless clearly marked as TODO/config-driven

## Expected package set
- robot_bridge
- leader_teleop
- perception
- world_model
- task_planner
- bt_runtime
- grasp_generation
- feasibility
- motion_planning
- executor
- validator
- logging_replay

## Key internal models
Implement explicit typed models for:
- ObjectState
- RobotState
- TaskState
- WorldState
- GraspCandidate
- ValidationResult
- PlanRequest / PlanResult
- TaskOutcome

## Expected BT behavior
Top-level pattern:
- UpdateWorldState
- FindContainer
- RepeatUntil(no reachable pens / retry exceeded)
  - UpdateWorldState
  - SelectNextObject
  - PickObject subtree
  - PlaceObjectInContainer subtree
  - MarkObjectComplete
- ReportOutcome

## Required validation semantics
Grasp validation must combine multiple signals, even in placeholder form:
- gripper state
- small lift verification
- vision/world-state consistency

Placement validation must check:
- no longer in gripper
- object inside target zone
- object remains after retreat

## Testing requirements
Add tests for:
- world state transitions
- object selection logic
- grasp validation state machine
- BT node status propagation
- retry budget handling

## Implementation style
- Python with type hints
- dataclasses or pydantic-like clear models
- modular and readable
- explicit TODO markers where arm-specific details are needed
- avoid over-abstraction in v1

## Output quality bar
Produce a repo that is structurally correct for robotics software work, even if some backends are still placeholders.
