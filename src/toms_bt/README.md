# toms_bt

Behavior tree runtime and manipulation task templates.

## Contents
- `node_base.py` – `BTNode`, `Sequence`, `Selector`, `RepeatUntilFailure`, `ParallelAll`
- `nodes/` – individual action/condition nodes
- `templates/pen_to_holder.py` – assembled pen-to-holder BT
- `runner.py` – ROS2 node that drives the tick loop

## Adding a new task
1. Create a new template in `templates/` following `pen_to_holder.py`.
2. Wire in your adapters in `runner.py` or a new runner.
