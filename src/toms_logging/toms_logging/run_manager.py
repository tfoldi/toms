"""toms_logging.run_manager – per-run artifact directory and structured logger.

Every execution run creates:
  <runs_base_dir>/<task_id>_<YYYYMMDD_HHMMSS>/
    events.jsonl          – all structured events (BT transitions, planner,
                             execution, validation, world state snapshots)
    config_snapshot.yaml  – deep-merged config at time of run
    rosout.txt            – Python logging output (all loggers)
    run_metadata.json     – task_id, start time, config paths, outcome

No ROS2 imports; safe for pure-Python tests and offline analysis.

Usage::

    mgr = RunManager("runs", task_id="pen_to_holder_001")
    mgr.snapshot_configs("config/robot.yaml", "config/robots/cello_follower.yaml")
    log = mgr.make_logger()
    log.log_run_start(str(mgr.run_dir), [...])
    # … run the task …
    log.log_run_end(success=True, duration_sec=47.3)
    mgr.write_run_metadata({"outcome": "SUCCESS"})
"""
from __future__ import annotations

import json
import logging
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from toms_logging.task_logger import TomsLogger


class RunManager:
    """Creates and manages the per-run artifact directory.

    Thread-safe for concurrent log writes; directory creation is idempotent.
    """

    def __init__(self, base_dir: str, task_id: str) -> None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir: Path = Path(base_dir) / f"{task_id}_{ts}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.task_id = task_id
        self.start_time: float = time.time()
        self._rosout_handler: Optional[logging.FileHandler] = None

    # ------------------------------------------------------------------
    # Config snapshot
    # ------------------------------------------------------------------

    def snapshot_configs(self, *yaml_paths: str) -> Path:
        """Deep-merge YAML files and save as config_snapshot.yaml.

        Later files override earlier ones (same order as launch file loading).
        Requires PyYAML.
        """
        import yaml  # noqa: PLC0415

        merged: dict = {}
        for path in yaml_paths:
            with open(path) as fh:
                data = yaml.safe_load(fh) or {}
            merged = _deep_merge(merged, data)

        snapshot_path = self.run_dir / "config_snapshot.yaml"
        with open(snapshot_path, "w") as fh:
            yaml.dump(merged, fh, default_flow_style=False, sort_keys=True)
        return snapshot_path

    # ------------------------------------------------------------------
    # rosout.txt capture
    # ------------------------------------------------------------------

    def setup_rosout_handler(self) -> logging.FileHandler:
        """Add a FileHandler to the root Python logger → writes rosout.txt.

        Call once after creating the RunManager.  Returns the handler so the
        caller can remove it at shutdown if needed.
        """
        if self._rosout_handler is not None:
            return self._rosout_handler

        fh = logging.FileHandler(str(self.run_dir / "rosout.txt"), encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(
            logging.Formatter("%(asctime)s %(name)-30s %(levelname)-8s %(message)s")
        )
        logging.getLogger().addHandler(fh)
        self._rosout_handler = fh
        return fh

    # ------------------------------------------------------------------
    # Logger factory
    # ------------------------------------------------------------------

    def make_logger(self, also_print: bool = True) -> "RunLogger":
        """Return a RunLogger that writes events.jsonl inside this run_dir."""
        return RunLogger(str(self.run_dir), self.task_id, also_print=also_print)

    # ------------------------------------------------------------------
    # Metadata
    # ------------------------------------------------------------------

    def write_run_metadata(self, extra: Optional[Dict[str, Any]] = None) -> Path:
        """Write run_metadata.json with standard fields + optional extras."""
        meta: Dict[str, Any] = {
            "task_id": self.task_id,
            "run_dir": str(self.run_dir),
            "started_at": datetime.fromtimestamp(self.start_time).isoformat(),
            "git_hash": _git_hash(),
        }
        if extra:
            meta.update(extra)
        out = self.run_dir / "run_metadata.json"
        with open(out, "w") as fh:
            json.dump(meta, fh, indent=2)
        return out

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def teardown(self) -> None:
        """Remove the rosout FileHandler from the root logger."""
        if self._rosout_handler is not None:
            logging.getLogger().removeHandler(self._rosout_handler)
            self._rosout_handler.close()
            self._rosout_handler = None


# ---------------------------------------------------------------------------
# RunLogger
# ---------------------------------------------------------------------------


class RunLogger(TomsLogger):
    """Structured event logger that writes to a fixed run directory.

    Subclasses TomsLogger; all existing log_* methods work unchanged.
    Adds:
        log_bt_transition(node_name, before, after, world_seq)
        log_run_start(run_dir, config_paths)
        log_run_end(success, duration_sec)

    The log file is ``events.jsonl`` (not timestamp-named) since the
    timestamp is already encoded in the parent run directory name.
    """

    def __init__(
        self,
        run_dir: str,
        task_id: str,
        also_print: bool = True,
    ) -> None:
        # Bypass TomsLogger.__init__ – we set up identical internals but with
        # a fixed filename inside the run directory.
        self._task_id = task_id
        self._log_dir = Path(run_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)

        log_file = self._log_dir / "events.jsonl"
        self._file = open(log_file, "a", encoding="utf-8")  # noqa: SIM115

        self._python_logger = logging.getLogger(f"toms.{task_id}")
        if also_print and not self._python_logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(
                logging.Formatter("[TOMS] %(levelname)s %(message)s")
            )
            self._python_logger.addHandler(handler)
            self._python_logger.setLevel(logging.DEBUG)

    # ------------------------------------------------------------------
    # New events
    # ------------------------------------------------------------------

    def log_bt_transition(
        self,
        node_name: str,
        before: str,
        after: str,
        world_seq: int = 0,
    ) -> None:
        """Log a BT node status transition.

        Args:
            node_name: Name of the node (e.g. "PickObject", "PickPlaceOnce").
            before:    Previous status value string ("RUNNING", "SUCCESS", "FAILURE").
            after:     New status value string.
            world_seq: WorldState.sequence at time of transition.
        """
        self._emit(
            "bt_transition",
            {"node": node_name, "before": before, "after": after, "world_seq": world_seq},
        )
        if after == "FAILURE":
            self._python_logger.warning("BT FAILURE: %s  (seq=%d)", node_name, world_seq)

    def log_run_start(self, run_dir: str, config_paths: List[str]) -> None:
        """Emit the opening run_start record."""
        self._emit("run_start", {"run_dir": run_dir, "config_paths": config_paths})
        self._python_logger.info("Run started → %s", run_dir)

    def log_run_end(self, success: bool, duration_sec: float) -> None:
        """Emit the closing run_end record."""
        self._emit("run_end", {"success": success, "duration_sec": duration_sec})
        status = "SUCCESS" if success else "FAILED"
        self._python_logger.info("Run ended: %s in %.1f s", status, duration_sec)

    def log_event(self, event_type: str, payload: Dict[str, Any]) -> None:
        """Emit an arbitrary structured event to events.jsonl.

        Use for event types not covered by the named log_* methods, e.g.
        preflight_check results.
        """
        self._emit(event_type, payload)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge override into base; override values win."""
    result = dict(base)
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def _git_hash() -> Optional[str]:
    """Return the current git commit hash, or None if unavailable."""
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL,
            timeout=2,
        )
        return out.decode().strip()
    except Exception:
        return None
