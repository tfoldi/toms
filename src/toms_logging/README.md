# toms_logging

Structured logging and replay support.

## Contents
- `task_logger.py` – JSONL event logger (`TomsLogger`)

## Output
One JSONL file per task run in `/tmp/toms_logs/` (configurable via `task.yaml`).
Events: `world_state`, `object_selected`, `grasp_candidates`, `plan_result`,
`execution_step`, `validation_result`, `retry`, `task_outcome`.

## TODO
- Add ROS2 bag integration for full sensor replay.
