#!/usr/bin/env python3
"""preflight_offline.py – offline config completeness check for TOMS.

Loads robot.yaml + the chosen overlay (default: cello_follower) and runs
ConfigChecker.  No ROS2 or hardware connection needed.

Usage::

    python3 scripts/preflight_offline.py
    python3 scripts/preflight_offline.py --robot cello_follower
    python3 scripts/preflight_offline.py --robot cello_follower --json

Exit codes:
  0 – all checks passed (warnings allowed)
  1 – one or more checks FAILED
  2 – config files not found or YAML parse error
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


def _setup_path() -> Path:
    """Add src/* packages to sys.path and return repo root."""
    here = Path(__file__).resolve().parent
    repo = here.parent
    for pkg in ["toms_core", "toms_robot"]:
        candidate = repo / "src" / pkg
        if candidate.is_dir() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))
    return repo


def main() -> None:
    repo = _setup_path()

    parser = argparse.ArgumentParser(
        prog="preflight_offline",
        description=(
            "Offline TOMS config preflight – no ROS2 needed.\n\n"
            "Validates config/robot.yaml + the robot overlay for missing or "
            "inconsistent fields.  Run this before starting ROS2 to catch "
            "config problems early."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--robot",
        default="cello_follower",
        metavar="NAME",
        help="Robot overlay name (loads config/robots/<name>.yaml). Default: cello_follower",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        dest="json_output",
        help="Output raw JSON instead of human-readable text",
    )
    args = parser.parse_args()

    # Resolve config paths
    config_dir = repo / "config"
    robot_yaml = config_dir / "robot.yaml"
    overlay_yaml = config_dir / "robots" / f"{args.robot}.yaml"

    missing = [str(p) for p in [robot_yaml, overlay_yaml] if not p.exists()]
    if missing:
        print(f"error: config file(s) not found: {missing}", file=sys.stderr)
        print(
            f"  Expected: {robot_yaml}\n  Expected: {overlay_yaml}\n"
            "  Run from the repository root or check --robot name.",
            file=sys.stderr,
        )
        sys.exit(2)

    from toms_robot.config_loader import ConfigurationError, RobotConfig
    from toms_robot.preflight import (
        CheckResult,
        ConfigChecker,
        PreflightReport,
        format_report,
    )

    # Load config
    results: list[CheckResult] = []
    config = None
    try:
        config = RobotConfig.from_yaml_files(str(robot_yaml), str(overlay_yaml))
        results.append(CheckResult(
            name="config:load",
            category="config",
            passed=True,
            message=f"Loaded: {robot_yaml.name} + {overlay_yaml.name}",
        ))
    except ConfigurationError as exc:
        results.append(CheckResult(
            name="config:load",
            category="config",
            passed=False,
            message=f"ConfigurationError: {exc}",
        ))
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(2)

    if config is not None:
        results.extend(ConfigChecker().check_all(config))

    import time
    report = PreflightReport(
        results=results,
        timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
    )

    if args.json_output:
        import dataclasses
        print(json.dumps(dataclasses.asdict(report), indent=2, default=str))
    else:
        print(format_report(report))

    sys.exit(0 if report.passed else 1)


if __name__ == "__main__":
    main()
