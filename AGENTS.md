# AGENTS.md

## Project
TOMS – Task-Oriented Manipulation System

A modular, behavior-tree-driven execution system for tabletop robot manipulation.

Initial target:
- “Put my pens into my pen holder”

Core idea:
- Structured execution, not end-to-end policy
- Explicit world state
- Deterministic control + validation + retry
- Modular components, swappable backends

---

## System Goal

Build a reliable execution loop:

perception → world state → behavior tree → pose generation → execution → validation → retry

This system should:
- work locally (RTX 4060)
- be inspectable and debuggable
- support incremental upgrades (AnyGrasp, VLMs, etc.)

---

## Naming Conventions

All core modules must follow toms_* naming:

- toms_core
- toms_bt
- toms_world
- toms_perception
- toms_planning
- toms_execution
- toms_validation
- toms_robot
- toms_logging

Do not introduce inconsistent naming.

---

## Core Principles

1. Hybrid system
   - Do not build end-to-end policies
      - Combine planning, control, and learned components

      2. Behavior tree as execution backbone
         - No freeform agent control at runtime
            - All actions go through BT nodes

            3. World state is canonical
               - All modules read/write through structured state
                  - No hidden implicit state

                  4. Validation is mandatory
                     - Every action must have postconditions
                        - No “assume success”

                        5. Retry and recovery are first-class
                           - Failures must branch, not crash

                           6. Local-first execution
                              - No cloud dependency in control loop

                              7. Swappable components
                                 - Grasp generation must support replacement (e.g. AnyGrasp)

                                 ---

## Initial Task: Pens → Holder

Execution flow:

1. Detect pen holder
2. Detect pens
3. Select next pen
4. Generate grasp candidates
5. Filter with IK
6. Plan motion (MoveIt2)
7. Execute grasp
8. Validate grasp
9. Generate placement pose
10. Execute place
11. Validate placement
12. Repeat

---

## Behavior Tree (Required Pattern)

Root:
- UpdateWorldState
- FindContainer
- RepeatUntil(no reachable pens / retry exceeded)
  - UpdateWorldState
    - SelectNextObject
      - PickObject subtree
        - PlaceObjectInContainer subtree
          - MarkObjectComplete
          - ReportOutcome

          All nodes must define:
          - inputs
          - outputs
          - success condition
          - failure types
          - retry behavior

          ---

## Required Data Models

Implement explicit typed models:

- ObjectState
- RobotState
- TaskState
- WorldState
- GraspCandidate
- ValidationResult
- TaskOutcome

No unstructured dicts in core logic.

---

## Validation Requirements

### Grasp validation must combine:
- gripper width
- optional effort/current
- small lift test
- vision/world consistency

### Placement validation must confirm:
- object not in gripper
- object inside holder zone
- object remains after retreat

Return structured failure types:
- empty_grasp
- slip
- dropped
- placement_failed
- uncertain

---

## Package Responsibilities

### toms_robot
Robot interface (joint state, motion, gripper)

### toms_perception
Detection, segmentation, pose estimation

### toms_world
Canonical world state + tracking

### toms_bt
Behavior tree runtime and node definitions

### toms_planning
Grasp generation + IK filtering + planning interfaces

### toms_execution
Action execution (approach, grasp, place)

### toms_validation
All validators and failure classification

### toms_logging
Structured logging + replay

---

## Motion Planning

Two-stage approach:

1. Fast IK filtering
2. MoveIt2 planning

Use:
- MoveIt2 for global motion
- Cartesian for final approach/retreat

---

## Logging (Mandatory)

Log:
- world state snapshots
- selected objects
- grasp candidates
- plans
- execution steps
- validation results
- failures and retries

Must support replay/debugging.

---

## Non-Goals (v1)

Do NOT implement:
- end-to-end VLA control
- cloud-dependent runtime
- unconstrained LLM planning
- general household manipulation
- large world models

---

## Arm Integration (Fill This In)

Reference:
https://github.com/Welt-liu/star-arm-moveit2

Fill with actual values:

- robot model:
- DOF:
- ROS2 distro:
- planning group:
- end effector link:
- base frame:
- tool frame:
- joint state topic:
- trajectory action:
- gripper interface:
- MoveIt package:
- URDF/SRDF paths:
- kinematics plugin:
- controller names:
- camera topics:
- calibration method:
- known quirks:

Do not guess these in code. Use config.

---

## Agent Constraints

The coding agent may:
- scaffold packages
- implement typed models
- build BT runtime
- create configs and tests

The agent must NOT:
- bypass validation
- hardcode fake robot behavior
- assume cloud usage
- remove retries for simplicity
- invent robot interfaces without config/TODO

---

## First Milestone

A runnable system with:
- mocked perception
- mocked robot bridge
- BT execution
- grasp + place flow
- validation logic
- structured logging

Hardware integration comes AFTER this works.

---

## Key Rule

If something is uncertain:
- make it explicit
- put it in config
- add a TODO

Do not guess and hide it in code.
