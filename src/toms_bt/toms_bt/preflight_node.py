"""toms_bt.preflight_node – ROS2 node that runs the TOMS hardware preflight
and optional smoke test.

Phases (run in order):
  1. Config check  – offline, runs immediately after node init
  2. ROS bindings  – topics, action servers, MoveIt2 service, TF frames
  3. Smoke test    – joint state read, gripper cycle, MoveIt2 plan probe
                     (only when smoke_test param is True)

All results are:
  • Emitted as preflight_check events to events.jsonl in the run directory
  • Written as preflight_report.txt (human-readable) and
    preflight_report.json (structured) in the same run directory
  • Printed to the console

Parameters (declare via launch or command line):
  robot           (str,  default "cello_follower")   – overlay name
  runs_base_dir   (str,  default "runs")             – artifact base dir
  smoke_test      (bool, default True)               – run gripper + plan checks
  execute_motion  (bool, default False)              – actually execute a plan
  snapshot_configs (str, default "")                 – comma-separated YAML paths
                                                       (empty = auto-detect from robot param)

Usage::

    ros2 run toms_bt run_preflight
    ros2 run toms_bt run_preflight --ros-args -p smoke_test:=false
    ros2 launch toms_bt toms_preflight.launch.py

Exit behaviour: the node prints the report and shuts down cleanly after the
last check completes.  Exit code 0 even on failures (failures are in the report).
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import List

import rclpy
import rclpy.node
from toms_logging.run_manager import RunManager
from toms_robot.config_loader import ConfigurationError, RobotConfig
from toms_robot.preflight import (
    CheckResult,
    PreflightReport,
    RosChecker,
    SmokeTestRunner,
    format_report,
)


class PreflightNode(rclpy.node.Node):
    """ROS2 node that runs the TOMS preflight + optional smoke test."""

    def __init__(self) -> None:
        super().__init__("toms_preflight")

        self.declare_parameter("robot", "cello_follower")
        self.declare_parameter("runs_base_dir", "runs")
        self.declare_parameter("smoke_test", True)
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("snapshot_configs", "")

        self._done = False
        self._results: List[CheckResult] = []
        self._joint_state_received = False
        # Set by timer callback to trigger smoke test + finalization from main loop
        self._smoke_pending = False
        self._smoke_execute_motion = False
        self._run_mgr: object = None
        self._log: object = None

        # Phase 1: config check (offline, immediate)
        self._config, config_errors = self._load_config()
        self._results.extend(config_errors)

        # Subscribe to joint states NOW so the main spin loop (not a callback)
        # delivers messages before the live-check timer fires.
        if self._config is not None:
            try:
                from sensor_msgs.msg import JointState  # type: ignore[import]  # noqa: PLC0415
                self.create_subscription(
                    JointState,
                    self._config.joint_state_topic,
                    lambda _msg: setattr(self, "_joint_state_received", True),
                    10,
                )
            except ImportError:
                pass

        # 3 s window to collect messages before running live checks
        self._timer = self.create_timer(3.0, self._run_live_checks)

    # ------------------------------------------------------------------
    # Timer callback – ROS binding checks only (no action futures here)
    # ------------------------------------------------------------------

    def _run_live_checks(self) -> None:
        """Phase 2: ROS binding checks.  Must NOT call spin_until_future_complete
        (reentrant spin).  Smoke test is deferred to the main loop via
        _smoke_pending so that action futures can complete normally."""
        self._timer.cancel()

        smoke_test = self.get_parameter("smoke_test").value
        execute_motion = self.get_parameter("execute_motion").value
        runs_base_dir = self.get_parameter("runs_base_dir").value

        # Create run artifact directory
        run_mgr = RunManager(runs_base_dir, "preflight")
        run_mgr.setup_rosout_handler()
        log = run_mgr.make_logger(also_print=False)

        config_paths = self._resolve_config_paths()
        if config_paths:
            try:
                run_mgr.snapshot_configs(*config_paths)
            except Exception as exc:
                self.get_logger().warning(f"Config snapshot failed: {exc}")

        run_mgr.write_run_metadata({"phase": "preflight"})
        log.log_run_start(str(run_mgr.run_dir), config_paths)

        if self._config is not None:
            ros_checker = RosChecker(self)
            ros_results = ros_checker.check_all(
                self._config,
                pre_received_topics={
                    self._config.joint_state_topic: self._joint_state_received,
                },
            )
            self._results.extend(ros_results)
        else:
            self.get_logger().error(
                "Config failed to load – skipping ROS and smoke-test checks"
            )

        if smoke_test and self._config is not None:
            # Defer to main loop – spin_until_future_complete cannot run here
            self._run_mgr = run_mgr
            self._log = log
            self._smoke_execute_motion = execute_motion
            self._smoke_pending = True
        else:
            self._finalize_preflight(run_mgr, log)

    # ------------------------------------------------------------------
    # Finalization – called from main loop (safe for action futures)
    # ------------------------------------------------------------------

    def _finalize_preflight(self, run_mgr: object, log: object) -> None:
        """Emit results, write report files, print summary, signal done."""
        for r in self._results:
            log.log_event("preflight_check", {  # type: ignore[attr-defined]
                "name": r.name,
                "category": r.category,
                "passed": r.passed,
                "warning": r.warning,
                "message": r.message,
            })

        report = PreflightReport(
            results=self._results,
            run_dir=str(run_mgr.run_dir),  # type: ignore[attr-defined]
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )

        log.log_run_end(  # type: ignore[attr-defined]
            success=report.passed,
            duration_sec=time.time() - run_mgr.start_time,  # type: ignore[attr-defined]
        )
        log.close()  # type: ignore[attr-defined]

        self._write_report_files(run_mgr.run_dir, report)  # type: ignore[attr-defined]
        run_mgr.teardown()  # type: ignore[attr-defined]

        print(format_report(report))
        self.get_logger().info(f"Preflight complete → {run_mgr.run_dir}")  # type: ignore[attr-defined]
        self._done = True

    def _run_smoke_test(self, execute_motion: bool) -> None:
        """Create bridge + planner and run SmokeTestRunner."""
        try:
            from toms_robot.cello_bridge import CelloRobotBridge  # noqa: PLC0415
        except ImportError as exc:
            self._results.append(CheckResult(
                name="smoke:bridge_import",
                category="execution",
                passed=False,
                message=f"Cannot import CelloRobotBridge: {exc}",
            ))
            return

        bridge = CelloRobotBridge(self._config, node=self)

        # Brief spin to let the joint-state subscriber populate
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Build planner (needs live node)
        planner = None
        try:
            from toms_planning.moveit_wrapper import (  # noqa: PLC0415
                MoveIt2RosPlanner,
                PlanningConfig,
            )
            planning_cfg = PlanningConfig()   # uses defaults; override via planning.yaml
            planner = MoveIt2RosPlanner(self._config, planning_cfg, node=self)
        except ImportError:
            pass   # planner unavailable; smoke test will skip plan check

        runner = SmokeTestRunner(
            self._config, bridge, execute_motion=execute_motion, node=self,
        )
        smoke_results = runner.run_all(planner=planner)
        self._results.extend(smoke_results)

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------

    def _load_config(self):
        """Try to load RobotConfig; return (config, [error_results])."""
        paths = self._resolve_config_paths()
        errors: List[CheckResult] = []

        if not paths:
            errors.append(CheckResult(
                name="config:load",
                category="config",
                passed=False,
                message=(
                    "snapshot_configs param is empty and auto-detection failed. "
                    "Pass snapshot_configs or launch via toms_preflight.launch.py."
                ),
            ))
            return None, errors

        try:
            config = RobotConfig.from_yaml_files(*paths)
            errors.append(CheckResult(
                name="config:load",
                category="config",
                passed=True,
                message=f"Loaded RobotConfig from {len(paths)} file(s): {paths}",
            ))
            # Run offline config checks
            from toms_robot.preflight import ConfigChecker  # noqa: PLC0415
            errors.extend(ConfigChecker().check_all(config))
            return config, errors
        except ConfigurationError as exc:
            errors.append(CheckResult(
                name="config:load",
                category="config",
                passed=False,
                message=f"ConfigurationError: {exc}",
            ))
            return None, errors
        except FileNotFoundError as exc:
            errors.append(CheckResult(
                name="config:load",
                category="config",
                passed=False,
                message=f"Config file not found: {exc}",
            ))
            return None, errors

    def _resolve_config_paths(self) -> List[str]:
        """Return the ordered list of YAML config file paths."""
        raw = self.get_parameter("snapshot_configs").value or ""
        if raw:
            return [p.strip() for p in raw.split(",") if p.strip()]

        # Auto-detect relative to cwd (works when launched from workspace root)
        robot = self.get_parameter("robot").value
        candidates = [
            "config/robot.yaml",
            f"config/robots/{robot}.yaml",
            "config/planning.yaml",
            "config/task.yaml",
        ]
        return [p for p in candidates if os.path.exists(p)]

    # ------------------------------------------------------------------
    # Report file writers
    # ------------------------------------------------------------------

    @staticmethod
    def _write_report_files(run_dir: Path, report: PreflightReport) -> None:
        txt_path = run_dir / "preflight_report.txt"
        txt_path.write_text(format_report(report), encoding="utf-8")

        json_path = run_dir / "preflight_report.json"
        import dataclasses  # noqa: PLC0415
        json_path.write_text(
            json.dumps(dataclasses.asdict(report), indent=2, default=str),
            encoding="utf-8",
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PreflightNode()
    try:
        while not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
            # Phase 3 runs here – outside any callback – so spin_until_future_complete works
            if node._smoke_pending:
                node._smoke_pending = False
                node._run_smoke_test(node._smoke_execute_motion)
                node._finalize_preflight(node._run_mgr, node._log)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
