# toms_planning

Grasp generation, IK feasibility filter, motion planning interface.

## Contents
- `grasp_generator.py` – `HeuristicGraspGenerator` (v1); swap for AnyGrasp later
- `feasibility.py` – `MockFeasibilityFilter`; TODO: real IK solver
- `moveit_wrapper.py` – `MockMotionPlanner` + `MoveIt2RosPlanner` stub
- `place_pose.py` – `HeuristicPlacePoseAdapter`

## TODO (hardware)
- Fill in `MoveIt2RosPlanner` once `robot.yaml` arm integration section is complete.
- Replace `MockFeasibilityFilter` with KDL/TRAC-IK client.
