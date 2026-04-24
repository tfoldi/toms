"""toms_logging.run_summary – CLI tool to summarize a TOMS run directory.

Reads events.jsonl, run_metadata.json, and config_snapshot.yaml from a run
directory and prints a human-readable summary with the final task outcome,
the last failed BT node, and the failure type with diagnostic signals.

Usage::

    toms-run-summary runs/pen_to_holder_001_20240423_143022
    toms-run-summary --json runs/pen_to_holder_001_20240423_143022
    toms-run-summary runs/   # summarise the most recent run in the directory

No ROS2 required.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

# ---------------------------------------------------------------------------
# Data structures for the parsed summary
# ---------------------------------------------------------------------------


@dataclass
class FailureDetail:
    node_name: Optional[str]
    phase: Optional[str]      # "grasp" | "placement"
    failure_type: Optional[str]
    signals: Dict[str, Any]
    message: str


@dataclass
class RunSummary:
    task_id: str
    run_dir: str
    started_at: Optional[str]
    duration_sec: Optional[float]
    success: Optional[bool]
    objects_placed: List[str]
    objects_failed: List[str]
    total_bt_ticks: int
    failure_transitions: List[Dict[str, Any]]
    last_failure: Optional[FailureDetail]
    git_hash: Optional[str]


# ---------------------------------------------------------------------------
# Core parsing
# ---------------------------------------------------------------------------


def parse_run(run_dir: Path) -> RunSummary:
    """Parse a run directory and return a structured RunSummary."""
    task_id = _infer_task_id(run_dir)
    started_at = None
    git_hash = None

    # Load run_metadata.json if present
    meta_path = run_dir / "run_metadata.json"
    if meta_path.exists():
        with open(meta_path) as fh:
            meta = json.load(fh)
        task_id = meta.get("task_id", task_id)
        started_at = meta.get("started_at")
        git_hash = meta.get("git_hash")

    # Parse events.jsonl
    events = _load_events(run_dir / "events.jsonl")

    duration_sec: Optional[float] = None
    success: Optional[bool] = None
    objects_placed: List[str] = []
    objects_failed: List[str] = []
    failure_transitions: List[Dict[str, Any]] = []
    last_validation_failure: Optional[Dict[str, Any]] = None
    last_failure_node: Optional[str] = None
    bt_ticks = 0

    for ev in events:
        etype = ev.get("event", "")

        if etype == "run_end":
            success = ev.get("success")
            duration_sec = ev.get("duration_sec")

        elif etype == "task_outcome":
            objects_placed = ev.get("objects_placed", [])
            objects_failed = ev.get("objects_failed", [])
            if success is None:
                success = ev.get("success")
            if duration_sec is None:
                duration_sec = ev.get("duration_sec")

        elif etype == "bt_transition":
            if ev.get("after") == "FAILURE":
                failure_transitions.append(ev)
                last_failure_node = ev.get("node")
            if ev.get("after") in ("SUCCESS", "FAILURE"):
                bt_ticks += 1

        elif etype == "validation_result" and not ev.get("success", True):
            last_validation_failure = ev

    # Build last_failure detail
    last_failure: Optional[FailureDetail] = None
    if last_validation_failure is not None:
        last_failure = FailureDetail(
            node_name=last_failure_node,
            phase=last_validation_failure.get("phase"),
            failure_type=last_validation_failure.get("failure_type"),
            signals=last_validation_failure.get("signals", {}),
            message=last_validation_failure.get("message", ""),
        )
    elif last_failure_node is not None:
        last_failure = FailureDetail(
            node_name=last_failure_node,
            phase=None,
            failure_type=None,
            signals={},
            message="",
        )

    return RunSummary(
        task_id=task_id,
        run_dir=str(run_dir.resolve()),
        started_at=started_at,
        duration_sec=duration_sec,
        success=success,
        objects_placed=objects_placed,
        objects_failed=objects_failed,
        total_bt_ticks=bt_ticks,
        failure_transitions=failure_transitions[-10:],  # last 10 only
        last_failure=last_failure,
        git_hash=git_hash,
    )


# ---------------------------------------------------------------------------
# Formatting
# ---------------------------------------------------------------------------


def format_summary(s: RunSummary) -> str:
    """Return a human-readable multi-line summary string."""
    lines = []

    # Header
    run_name = Path(s.run_dir).name
    lines.append(f"Run Summary: {run_name}")
    lines.append("─" * (len("Run Summary: ") + len(run_name)))
    lines.append(f"  Task ID:        {s.task_id}")

    if s.started_at:
        lines.append(f"  Started:        {s.started_at}")
    if s.duration_sec is not None:
        lines.append(f"  Duration:       {s.duration_sec:.1f} s")
    if s.git_hash:
        lines.append(f"  Git hash:       {s.git_hash}")

    lines.append("")

    # Outcome
    if s.success is None:
        outcome_str = "UNKNOWN (run may be incomplete)"
    elif s.success:
        outcome_str = "SUCCESS"
    else:
        outcome_str = "FAILED"

    lines.append(f"  Result:         {outcome_str}")
    if s.objects_placed:
        lines.append(f"  Placed:         {len(s.objects_placed)}  {s.objects_placed}")
    if s.objects_failed:
        lines.append(f"  Failed:         {len(s.objects_failed)}  {s.objects_failed}")
    lines.append(f"  BT transitions: {s.total_bt_ticks}")

    # Last failure
    if s.last_failure is not None:
        lf = s.last_failure
        lines.append("")
        lines.append("Last failure:")
        if lf.node_name:
            lines.append(f"  BT node:        {lf.node_name}")
        if lf.phase:
            lines.append(f"  Stage:          {lf.phase}_validation")
        if lf.failure_type:
            lines.append(f"  Failure type:   {lf.failure_type}")
        if lf.signals:
            for k, v in lf.signals.items():
                lines.append(f"  Signal [{k}]: {v}")
        if lf.message:
            lines.append(f"  Message:        {lf.message}")

    # Recent FAILURE transitions (last ≤ 10)
    if s.failure_transitions:
        lines.append("")
        lines.append(f"BT FAILURE transitions (last {len(s.failure_transitions)}):")
        for tr in s.failure_transitions:
            seq = tr.get("world_seq", "?")
            node = tr.get("node", "?")
            lines.append(f"  seq={seq:>4}  {node}")

    lines.append("")
    lines.append(f"Artifact dir:   {s.run_dir}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _load_events(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    events = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return events


def _infer_task_id(run_dir: Path) -> str:
    """Extract task_id from directory name <task_id>_<YYYYMMDD_HHMMSS>."""
    import re
    name = run_dir.name
    m = re.match(r'^(.+)_\d{8}_\d{6}$', name)
    if m:
        return m.group(1)
    return name


def _find_most_recent_run(directory: Path) -> Optional[Path]:
    """Return the most recently modified subdirectory, or None."""
    candidates = [d for d in directory.iterdir() if d.is_dir()]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.stat().st_mtime)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        prog="toms-run-summary",
        description=(
            "Summarise a TOMS run artifact directory.\n\n"
            "If the given path is a parent directory (not a run dir itself),\n"
            "the most recently modified subdirectory is used."
        ),
    )
    parser.add_argument(
        "run_dir",
        metavar="RUN_DIR",
        help="Path to a run directory (e.g. runs/pen_to_holder_001_20240423_143022)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output raw JSON instead of human-readable text",
    )
    args = parser.parse_args(argv)

    run_path = Path(args.run_dir)
    if not run_path.exists():
        print(f"error: path does not exist: {run_path}", file=sys.stderr)
        sys.exit(1)

    # Auto-select most recent child if the user pointed at a parent dir
    if not (run_path / "events.jsonl").exists() and run_path.is_dir():
        candidate = _find_most_recent_run(run_path)
        if candidate is None:
            print(f"error: no run subdirectories found in {run_path}", file=sys.stderr)
            sys.exit(1)
        run_path = candidate

    summary = parse_run(run_path)

    if args.json:
        import dataclasses
        print(json.dumps(dataclasses.asdict(summary), indent=2, default=str))
    else:
        print(format_summary(summary))


if __name__ == "__main__":
    main()
