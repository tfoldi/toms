"""toms_bt.runner – minimal BT execution loop (ROS2 node wrapper).

This node drives the tick loop.  Adapters are resolved from ROS2 service/action
clients at startup; the tick loop itself is ROS-agnostic.

TODO: wire real adapters once hardware details from robot.yaml are filled in.
"""
from __future__ import annotations

import time
import uuid

import rclpy
from rclpy.node import Node

from toms_bt.node_base import NodeStatus
from toms_bt.templates.pen_to_holder import PenToHolderConfig, build_pen_to_holder_tree
from toms_core.models import TaskState, TaskStatus, WorldState


class BtRunnerNode(Node):
    """ROS2 node that drives the behavior tree tick loop."""

    # TODO: replace these rates with values from task.yaml
    TICK_RATE_HZ: float = 10.0

    def __init__(self) -> None:
        super().__init__("toms_bt_runner")

        self.declare_parameter("task_id", str(uuid.uuid4()))
        self.declare_parameter("retry_budget", 3)
        self.declare_parameter("tick_rate_hz", self.TICK_RATE_HZ)

        task_id: str = self.get_parameter("task_id").value
        retry_budget: int = self.get_parameter("retry_budget").value
        tick_rate: float = self.get_parameter("tick_rate_hz").value

        # TODO: instantiate real perception / robot / planning adapters here.
        #       See toms_perception, toms_robot, toms_planning for adapter classes.
        perception = None   # TODO: PerceptionRosAdapter(self)
        robot = None        # TODO: RobotBridgeAdapter(self)
        grasp_generator = None   # TODO: GraspGeneratorAdapter(self)
        feasibility = None       # TODO: FeasibilityAdapter(self)
        planner = None           # TODO: MoveItPlannerAdapter(self)
        pick_executor = None     # TODO: ExecutionAdapter(self)
        grasp_validator = None   # TODO: GraspValidatorAdapter(self)
        place_pose_adapter = None  # TODO: PlacePoseAdapter(self)
        place_executor = None      # TODO: PlaceExecutionAdapter(self)
        placement_validator = None  # TODO: PlacementValidatorAdapter(self)
        logger = None              # TODO: TomsLoggerAdapter(self)

        cfg = PenToHolderConfig()
        # TODO: load cfg from task.yaml parameter

        self._world_state = WorldState(
            task=TaskState(
                task_id=task_id,
                retry_budget=retry_budget,
                status=TaskStatus.RUNNING,
            )
        )

        self._tree = build_pen_to_holder_tree(
            perception=perception,
            robot=robot,
            grasp_generator=grasp_generator,
            feasibility=feasibility,
            planner=planner,
            pick_executor=pick_executor,
            grasp_validator=grasp_validator,
            place_pose_adapter=place_pose_adapter,
            place_executor=place_executor,
            placement_validator=placement_validator,
            logger=logger,
            config=cfg,
        )

        period = 1.0 / tick_rate
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(f"BT runner started (task_id={task_id})")

    def _tick(self) -> None:
        status = self._tree.tick(self._world_state)
        if status == NodeStatus.SUCCESS:
            self.get_logger().info("Task completed: SUCCESS")
            self._timer.cancel()
        elif status == NodeStatus.FAILURE:
            self.get_logger().error("Task completed: FAILURE")
            self._timer.cancel()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BtRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
