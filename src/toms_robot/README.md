# toms_robot

Robot bridge: joint state, motion execution, gripper interface.

## Contents
- `robot_bridge.py` – `RobotBridgeBase` (abstract) + `MockRobotBridge`

## TODO (hardware)
Fill in `robot.yaml` "Arm Integration" section, then implement a concrete
`RobotBridgeBase` subclass for the actual arm driver.
Reference: https://github.com/Welt-liu/star-arm-moveit2
