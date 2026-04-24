"""Tests for run_summary.py – CLI parsing and formatting.

Pure-Python; no rclpy or ROS2 required.
"""
from __future__ import annotations

import json
import tempfile
from pathlib import Path

import pytest
from toms_logging.run_summary import (
    FailureDetail,
    RunSummary,
    _find_most_recent_run,
    _infer_task_id,
    _load_events,
    format_summary,
    parse_run,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _write_events(run_dir: Path, events: list) -> None:
    with open(run_dir / "events.jsonl", "w") as fh:
        for ev in events:
            fh.write(json.dumps(ev) + "\n")


def _write_metadata(run_dir: Path, meta: dict) -> None:
    with open(run_dir / "run_metadata.json", "w") as fh:
        json.dump(meta, fh)


def _make_run_dir(parent: Path, name: str = "task_20240423_143022") -> Path:
    d = parent / name
    d.mkdir(parents=True, exist_ok=True)
    return d


# ---------------------------------------------------------------------------
# _load_events
# ---------------------------------------------------------------------------


def test_load_events_empty_file():
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "events.jsonl"
        p.write_text("")
        assert _load_events(p) == []


def test_load_events_missing_file():
    p = Path("/nonexistent/events.jsonl")
    assert _load_events(p) == []


def test_load_events_parses_jsonl():
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "events.jsonl"
        p.write_text('{"event":"run_start"}\n{"event":"run_end"}\n')
        events = _load_events(p)
        assert len(events) == 2
        assert events[0]["event"] == "run_start"


def test_load_events_skips_malformed_lines():
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "events.jsonl"
        p.write_text('{"event":"ok"}\nNOT_JSON\n{"event":"also_ok"}\n')
        events = _load_events(p)
        assert len(events) == 2


# ---------------------------------------------------------------------------
# _infer_task_id
# ---------------------------------------------------------------------------


def test_infer_task_id_standard_name():
    d = Path("pen_to_holder_001_20240423_143022")
    assert _infer_task_id(d) == "pen_to_holder_001"


def test_infer_task_id_fallback():
    d = Path("some_random_name")
    assert _infer_task_id(d) == "some_random_name"


# ---------------------------------------------------------------------------
# parse_run – empty run
# ---------------------------------------------------------------------------


def test_parse_run_empty_dir():
    with tempfile.TemporaryDirectory() as base:
        run_dir = _make_run_dir(Path(base))
        summary = parse_run(run_dir)
        assert summary.success is None
        assert summary.objects_placed == []
        assert summary.objects_failed == []
        assert summary.last_failure is None


def test_parse_run_reads_run_metadata():
    with tempfile.TemporaryDirectory() as base:
        run_dir = _make_run_dir(Path(base), "my_task_20240423_120000")
        _write_metadata(run_dir, {
            "task_id": "my_task",
            "started_at": "2024-04-23T12:00:00",
            "git_hash": "abc1234",
        })
        _write_events(run_dir, [])
        summary = parse_run(run_dir)
        assert summary.task_id == "my_task"
        assert summary.started_at == "2024-04-23T12:00:00"
        assert summary.git_hash == "abc1234"


# ---------------------------------------------------------------------------
# parse_run – task_outcome event
# ---------------------------------------------------------------------------


def test_parse_run_task_outcome_success():
    with tempfile.TemporaryDirectory() as base:
        run_dir = _make_run_dir(Path(base))
        _write_events(run_dir, [
            {"event": "task_outcome", "success": True,
             "objects_placed": ["pen_0", "pen_1"], "objects_failed": [],
             "duration_sec": 45.2},
        ])
        s = parse_run(run_dir)
        assert s.success is True
        assert s.objects_placed == ["pen_0", "pen_1"]
        assert s.duration_sec == pytest.approx(45.2)


def test_parse_run_task_outcome_failure():
    with tempfile.TemporaryDirectory() as base:
        run_dir = _make_run_dir(Path(base))
        _write_events(run_dir, [
            {"event": "task_outcome", "success": False,
             "objects_placed": ["pen_0"], "objects_failed": ["pen_1"],
             "duration_sec": 23.0},
        ])
        s = parse_run(run_dir)
        assert s.success is False
        assert s.objects_failed == ["pen_1"]


# ---------------------------------------------------------------------------
# parse_run – run_end event
# ---------------------------------------------------------------------------


def test_parse_run_end_overrides_duration():
    with tempfile.TemporaryDirectory() as base:
        run_dir = _make_run_dir(Path(base))
        _write_events(run_dir, [
            {"event": "run_end", "success": True, "duration_sec": 30.0},
        ])
        s = parse_run(run_dir)
        assert s.success is True
        assert s.duration_sec == pytest.approx(30.0)


# ---------------------------------------------------------------------------
# parse_run – BT transitions
# ---------------------------------------------------------------------------


def test_parse_run_counts_bt_failure_transitions():
    with tempfile.TemporaryDirectory() as base:
        run_dir = _make_run_dir(Path(base))
        _write_events(run_dir, [
            {"event": "bt_transition", "node": "PickObject", "before": "RUNNING",
             "after": "FAILURE", "world_seq": 5},
            {"event": "bt_transition", "node": "PickObject", "before": "RUNNING",
             "after": "SUCCESS", "world_seq": 10},
            {"event": "bt_transition", "node": "PlaceObjectInContainer",
             "before": "RUNNING", "after": "FAILURE", "world_seq": 7},
        ])
        s = parse_run(run_dir)
        assert len(s.failure_transitions) == 2


def test_parse_run_last_failure_node():
    with tempfile.TemporaryDirectory() as base:
        run_dir = _make_run_dir(Path(base))
        _write_events(run_dir, [
            {"event": "bt_transition", "node": "PickObject",
             "before": "RUNNING", "after": "FAILURE", "world_seq": 3},
            {"event": "bt_transition", "node": "PlaceObjectInContainer",
             "before": "RUNNING", "after": "FAILURE", "world_seq": 8},
        ])
        s = parse_run(run_dir)
        assert s.last_failure is not None
        assert s.last_failure.node_name == "PlaceObjectInContainer"


# ---------------------------------------------------------------------------
# parse_run – last validation failure detail
# ---------------------------------------------------------------------------


def test_parse_run_last_validation_failure():
    with tempfile.TemporaryDirectory() as base:
        run_dir = _make_run_dir(Path(base))
        _write_events(run_dir, [
            {"event": "validation_result", "phase": "grasp", "success": False,
             "failure_type": "EMPTY_GRASP",
             "signals": {"gripper_width": 0.072},
             "message": "Gripper too wide"},
        ])
        s = parse_run(run_dir)
        assert s.last_failure is not None
        lf = s.last_failure
        assert lf.phase == "grasp"
        assert lf.failure_type == "EMPTY_GRASP"
        assert lf.signals["gripper_width"] == pytest.approx(0.072)
        assert "Gripper" in lf.message


def test_parse_run_ignores_successful_validation():
    with tempfile.TemporaryDirectory() as base:
        run_dir = _make_run_dir(Path(base))
        _write_events(run_dir, [
            {"event": "validation_result", "phase": "grasp", "success": True,
             "failure_type": None, "signals": {}, "message": ""},
        ])
        s = parse_run(run_dir)
        assert s.last_failure is None


# ---------------------------------------------------------------------------
# format_summary
# ---------------------------------------------------------------------------


def _make_summary(**kwargs) -> RunSummary:
    defaults = {
        "task_id": "test_task",
        "run_dir": "/runs/test_task_20240423_120000",
        "started_at": "2024-04-23T12:00:00",
        "duration_sec": 45.0,
        "success": True,
        "objects_placed": ["pen_0"],
        "objects_failed": [],
        "total_bt_ticks": 10,
        "failure_transitions": [],
        "last_failure": None,
        "git_hash": "abc1234",
    }
    defaults.update(kwargs)
    return RunSummary(**defaults)


def test_format_summary_contains_task_id():
    s = _make_summary(task_id="my_task")
    text = format_summary(s)
    assert "my_task" in text


def test_format_summary_success():
    s = _make_summary(success=True)
    text = format_summary(s)
    assert "SUCCESS" in text


def test_format_summary_failure():
    s = _make_summary(success=False, objects_failed=["pen_1"])
    text = format_summary(s)
    assert "FAILED" in text
    assert "pen_1" in text


def test_format_summary_shows_last_failure():
    lf = FailureDetail(
        node_name="PickObject",
        phase="grasp",
        failure_type="EMPTY_GRASP",
        signals={"gripper_width": 0.072},
        message="Too wide",
    )
    s = _make_summary(success=False, last_failure=lf)
    text = format_summary(s)
    assert "PickObject" in text
    assert "EMPTY_GRASP" in text
    assert "gripper_width" in text


def test_format_summary_unknown_result():
    s = _make_summary(success=None)
    text = format_summary(s)
    assert "UNKNOWN" in text


# ---------------------------------------------------------------------------
# _find_most_recent_run
# ---------------------------------------------------------------------------


def test_find_most_recent_run():
    import time as _time
    with tempfile.TemporaryDirectory() as base:
        p = Path(base)
        d1 = p / "run_a"
        d1.mkdir()
        _time.sleep(0.05)
        d2 = p / "run_b"
        d2.mkdir()
        result = _find_most_recent_run(p)
        assert result == d2


def test_find_most_recent_run_empty_dir():
    with tempfile.TemporaryDirectory() as base:
        assert _find_most_recent_run(Path(base)) is None
