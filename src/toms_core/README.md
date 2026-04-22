# toms_core

Shared typed data models for all TOMS packages.  No ROS2 runtime dependency.

## Contents
- `models.py` – `ObjectState`, `RobotState`, `TaskState`, `WorldState`, `GraspCandidate`, `PlanRequest`, `PlanResult`, `ValidationResult`, `TaskOutcome`
- `exceptions.py` – TOMS exception hierarchy

## Rules
- No unstructured dicts in core logic; use these dataclasses.
- All `TODO` fields must be filled from config before use on hardware.
