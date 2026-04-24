"""Tests for RunManager and RunLogger.

Pure-Python; no rclpy or ROS2 required.
"""
from __future__ import annotations

import json
import logging
import tempfile
import time
from pathlib import Path

import pytest
from toms_logging.run_manager import RunLogger, RunManager, _deep_merge

# ---------------------------------------------------------------------------
# RunManager – directory creation
# ---------------------------------------------------------------------------


def test_run_manager_creates_directory():
    with tempfile.TemporaryDirectory() as base:
        mgr = RunManager(base, "test_task")
        assert mgr.run_dir.exists()
        assert mgr.run_dir.is_dir()


def test_run_manager_directory_name_contains_task_id():
    with tempfile.TemporaryDirectory() as base:
        mgr = RunManager(base, "my_task_123")
        assert "my_task_123" in mgr.run_dir.name


def test_run_manager_directory_name_contains_timestamp():
    with tempfile.TemporaryDirectory() as base:
        mgr = RunManager(base, "t")
        # Timestamp portion is YYYYMMDD_HHMMSS (15 chars after underscore)
        parts = mgr.run_dir.name.split("_")
        # Last two parts are date and time
        assert len(parts) >= 3
        assert parts[-2].isdigit()
        assert parts[-1].isdigit()


def test_run_manager_unique_dirs_for_same_task():
    with tempfile.TemporaryDirectory() as base:
        mgr1 = RunManager(base, "task")
        time.sleep(1.1)  # ensure different second
        mgr2 = RunManager(base, "task")
        assert mgr1.run_dir != mgr2.run_dir


# ---------------------------------------------------------------------------
# RunManager – config snapshot
# ---------------------------------------------------------------------------


def test_snapshot_configs_creates_file():
    pytest.importorskip("yaml")
    import yaml
    with tempfile.TemporaryDirectory() as base:
        mgr = RunManager(base, "snap_task")
        # Write two temp yaml files
        f1 = Path(base) / "a.yaml"
        f2 = Path(base) / "b.yaml"
        f1.write_text(yaml.dump({"x": 1, "nested": {"y": 2}}))
        f2.write_text(yaml.dump({"nested": {"y": 99, "z": 3}}))

        snap = mgr.snapshot_configs(str(f1), str(f2))
        assert snap.exists()
        assert snap.name == "config_snapshot.yaml"


def test_snapshot_configs_deep_merges():
    pytest.importorskip("yaml")
    import yaml
    with tempfile.TemporaryDirectory() as base:
        mgr = RunManager(base, "merge_task")
        f1 = Path(base) / "base.yaml"
        f2 = Path(base) / "overlay.yaml"
        f1.write_text(yaml.dump({"robot": {"model": "base", "dof": 6}}))
        f2.write_text(yaml.dump({"robot": {"model": "cello"}}))

        snap = mgr.snapshot_configs(str(f1), str(f2))
        data = yaml.safe_load(snap.read_text())
        assert data["robot"]["model"] == "cello"  # overlay wins
        assert data["robot"]["dof"] == 6           # base preserved


def test_snapshot_configs_overlay_wins_at_top_level():
    pytest.importorskip("yaml")
    import yaml
    with tempfile.TemporaryDirectory() as base:
        mgr = RunManager(base, "t")
        f1 = Path(base) / "f1.yaml"
        f2 = Path(base) / "f2.yaml"
        f1.write_text(yaml.dump({"key": "first"}))
        f2.write_text(yaml.dump({"key": "second"}))
        snap = mgr.snapshot_configs(str(f1), str(f2))
        data = yaml.safe_load(snap.read_text())
        assert data["key"] == "second"


# ---------------------------------------------------------------------------
# RunManager – rosout handler
# ---------------------------------------------------------------------------


def test_setup_rosout_handler_creates_file():
    with tempfile.TemporaryDirectory() as base:
        mgr = RunManager(base, "rosout_task")
        handler = mgr.setup_rosout_handler()
        assert handler is not None
        # Write a log message and verify it appears in rosout.txt
        logging.getLogger("toms.test").info("test rosout entry")
        handler.flush()
        rosout = mgr.run_dir / "rosout.txt"
        assert rosout.exists()
        # Clean up
        mgr.teardown()


def test_setup_rosout_handler_idempotent():
    with tempfile.TemporaryDirectory() as base:
        mgr = RunManager(base, "t")
        h1 = mgr.setup_rosout_handler()
        h2 = mgr.setup_rosout_handler()
        assert h1 is h2  # same handler returned on second call
        mgr.teardown()


# ---------------------------------------------------------------------------
# RunManager – metadata
# ---------------------------------------------------------------------------


def test_write_run_metadata_creates_json():
    with tempfile.TemporaryDirectory() as base:
        mgr = RunManager(base, "meta_task")
        mgr.write_run_metadata({"outcome": "SUCCESS"})
        meta_path = mgr.run_dir / "run_metadata.json"
        assert meta_path.exists()
        meta = json.loads(meta_path.read_text())
        assert meta["task_id"] == "meta_task"
        assert meta["outcome"] == "SUCCESS"
        assert "started_at" in meta


def test_write_run_metadata_contains_task_id():
    with tempfile.TemporaryDirectory() as base:
        mgr = RunManager(base, "specific_task")
        mgr.write_run_metadata()
        meta = json.loads((mgr.run_dir / "run_metadata.json").read_text())
        assert meta["task_id"] == "specific_task"


# ---------------------------------------------------------------------------
# RunLogger – events.jsonl
# ---------------------------------------------------------------------------


def test_run_logger_creates_events_jsonl():
    with tempfile.TemporaryDirectory() as run_dir:
        log = RunLogger(run_dir, "log_task", also_print=False)
        log.close()
        assert (Path(run_dir) / "events.jsonl").exists()


def test_run_logger_fixed_filename():
    """RunLogger always writes to events.jsonl (not a timestamp-named file)."""
    with tempfile.TemporaryDirectory() as run_dir:
        log = RunLogger(run_dir, "t", also_print=False)
        log.close()
        files = list(Path(run_dir).iterdir())
        assert any(f.name == "events.jsonl" for f in files)
        assert not any(f.name.endswith(".jsonl") and f.name != "events.jsonl" for f in files)


def test_run_logger_log_plan_result():
    from toms_core.models import PlanResult
    with tempfile.TemporaryDirectory() as run_dir:
        log = RunLogger(run_dir, "t", also_print=False)
        log.log_plan_result("pick", PlanResult(success=True, planning_time=0.12))
        log.close()
        events = _read_events(run_dir)
        assert any(e["event"] == "plan_result" for e in events)


def test_run_logger_log_bt_transition_failure():
    with tempfile.TemporaryDirectory() as run_dir:
        log = RunLogger(run_dir, "t", also_print=False)
        log.log_bt_transition("PickObject", "RUNNING", "FAILURE", world_seq=5)
        log.close()
        events = _read_events(run_dir)
        tr = [e for e in events if e["event"] == "bt_transition"][0]
        assert tr["node"] == "PickObject"
        assert tr["before"] == "RUNNING"
        assert tr["after"] == "FAILURE"
        assert tr["world_seq"] == 5


def test_run_logger_log_run_start():
    with tempfile.TemporaryDirectory() as run_dir:
        log = RunLogger(run_dir, "t", also_print=False)
        log.log_run_start(run_dir, ["robot.yaml"])
        log.close()
        events = _read_events(run_dir)
        assert any(e["event"] == "run_start" for e in events)


def test_run_logger_log_run_end():
    with tempfile.TemporaryDirectory() as run_dir:
        log = RunLogger(run_dir, "t", also_print=False)
        log.log_run_end(success=True, duration_sec=12.5)
        log.close()
        events = _read_events(run_dir)
        ev = [e for e in events if e["event"] == "run_end"][0]
        assert ev["success"] is True
        assert abs(ev["duration_sec"] - 12.5) < 0.01


def test_run_logger_records_have_ts_and_task_id():
    with tempfile.TemporaryDirectory() as run_dir:
        log = RunLogger(run_dir, "tid_123", also_print=False)
        log.log_run_start(run_dir, [])
        log.close()
        events = _read_events(run_dir)
        for ev in events:
            assert "ts" in ev
            assert ev["task_id"] == "tid_123"


# ---------------------------------------------------------------------------
# _deep_merge helper
# ---------------------------------------------------------------------------


def test_deep_merge_leaf_override():
    a = {"a": 1}
    b = {"a": 2}
    assert _deep_merge(a, b)["a"] == 2


def test_deep_merge_nested_partial():
    a = {"outer": {"x": 1, "y": 2}}
    b = {"outer": {"x": 99}}
    r = _deep_merge(a, b)
    assert r["outer"]["x"] == 99
    assert r["outer"]["y"] == 2


def test_deep_merge_new_top_level_key():
    a = {"x": 1}
    b = {"y": 2}
    r = _deep_merge(a, b)
    assert r == {"x": 1, "y": 2}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _read_events(run_dir: str) -> list:
    events = []
    p = Path(run_dir) / "events.jsonl"
    if p.exists():
        with open(p) as fh:
            for line in fh:
                line = line.strip()
                if line:
                    events.append(json.loads(line))
    return events
