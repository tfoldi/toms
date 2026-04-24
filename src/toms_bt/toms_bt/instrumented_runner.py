"""toms_bt.instrumented_runner – BT runner with per-run artifact collection.

Extends BtRunnerNode without changing any core architecture.  On each tick:
  - Walks the tree before/after and emits bt_transition events for every node
    whose status changed.
  - Snapshots WorldState to events.jsonl on every tick.
  - On task completion, writes the final run_metadata.json.

Run directory is created under ``runs_base_dir``/<task_id>_<YYYYMMDD_HHMMSS>/

Additional ROS2 parameters (beyond those in BtRunnerNode):
  runs_base_dir    (string,  default "runs")     – base directory for run artifacts
  snapshot_configs (string,  default "")          – comma-separated config YAML paths to snapshot
  sim_mode         (bool,    default False)        – recorded in run metadata
"""
from __future__ import annotations

import time
from typing import Dict, Optional

import rclpy
from toms_logging.run_manager import RunLogger, RunManager

from toms_bt.node_base import BTNode, NodeStatus
from toms_bt.runner import BtRunnerNode


class InstrumentedBtRunnerNode(BtRunnerNode):
    """BtRunnerNode with per-run artifact collection.

    Adds three behaviours on top of the base runner:
      1. Creates a run directory and a structured RunLogger on startup.
      2. Detects BT node status transitions on every tick and logs them.
      3. Captures all Python-level log output to rosout.txt.
    """

    def __init__(self) -> None:
        super().__init__()  # builds tree, declares core params, starts timer

        # ── Additional parameters ────────────────────────────────────────
        self.declare_parameter("runs_base_dir", "runs")
        self.declare_parameter("snapshot_configs", "")
        self.declare_parameter("sim_mode", False)

        runs_base_dir: str = self.get_parameter("runs_base_dir").value
        snapshot_str: str = self.get_parameter("snapshot_configs").value
        sim_mode: bool = self.get_parameter("sim_mode").value

        task_id: str = self.get_parameter("task_id").value

        # ── Run directory + logger ────────────────────────────────────────
        self._run_manager = RunManager(runs_base_dir, task_id)
        self._run_manager.setup_rosout_handler()
        self._run_logger: RunLogger = self._run_manager.make_logger()
        self._run_start_wall: float = time.monotonic()

        # ── Config snapshot ──────────────────────────────────────────────
        config_paths = [p.strip() for p in snapshot_str.split(",") if p.strip()]
        if config_paths:
            try:
                self._run_manager.snapshot_configs(*config_paths)
            except Exception as exc:
                self.get_logger().warning(f"Config snapshot failed: {exc}")

        # ── run_metadata.json ────────────────────────────────────────────
        self._run_manager.write_run_metadata(
            {
                "sim_mode": sim_mode,
                "config_paths": config_paths,
                "runs_base_dir": runs_base_dir,
            }
        )

        # ── run_start event ──────────────────────────────────────────────
        self._run_logger.log_run_start(
            str(self._run_manager.run_dir), config_paths
        )

        # ── Node-status tracking (for transition detection) ───────────────
        self._prev_node_statuses: Dict[int, str] = {}

        self.get_logger().info(
            f"Run artifacts → {self._run_manager.run_dir}"
        )

    # ------------------------------------------------------------------
    # Overridden tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        # Execute the tree
        status = self._tree.tick(self._world_state)

        # Capture node statuses after tick and emit transitions
        after = _walk_statuses(self._tree)
        seq = self._world_state.sequence
        for node_id, (name, after_val) in after.items():
            prev_val = self._prev_node_statuses.get(node_id)
            if prev_val != after_val:
                self._run_logger.log_bt_transition(name, prev_val or "RUNNING", after_val, seq)
        self._prev_node_statuses = {nid: val for nid, (_, val) in after.items()}

        # Snapshot world state on every tick
        self._run_logger.log_world_state(self._world_state)

        # Handle terminal states
        if status == NodeStatus.SUCCESS:
            self.get_logger().info("Task completed: SUCCESS")
            self._timer.cancel()
            self._run_logger.log_run_end(
                success=True, duration_sec=time.monotonic() - self._run_start_wall
            )
            self._run_manager.write_run_metadata(
                {"outcome": "SUCCESS", "sim_mode": self.get_parameter("sim_mode").value}
            )
            self._run_manager.teardown()
        elif status == NodeStatus.FAILURE:
            self.get_logger().error("Task completed: FAILURE")
            self._timer.cancel()
            self._run_logger.log_run_end(
                success=False, duration_sec=time.monotonic() - self._run_start_wall
            )
            self._run_manager.write_run_metadata(
                {"outcome": "FAILURE", "sim_mode": self.get_parameter("sim_mode").value}
            )
            self._run_manager.teardown()


# ---------------------------------------------------------------------------
# Tree-walking helper
# ---------------------------------------------------------------------------


def _walk_statuses(node: BTNode) -> Dict[int, tuple]:
    """Return {id(node): (name, status_str)} for node and all descendants."""
    result: Dict[int, tuple] = {id(node): (node.name, node.status.value)}
    for child in getattr(node, "children", []):
        result.update(_walk_statuses(child))
    single: Optional[BTNode] = getattr(node, "child", None)
    if single is not None:
        result.update(_walk_statuses(single))
    return result


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InstrumentedBtRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
