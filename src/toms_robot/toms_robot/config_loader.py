"""toms_robot.config_loader – typed robot configuration from merged YAML.

Loads config/robot.yaml + config/robots/<name>.yaml into validated dataclasses.
No ROS2 imports; safe for pure-Python unit tests.

Usage (offline / test):
    config = RobotConfig.from_yaml_files(
        "config/robot.yaml",
        "config/robots/cello_follower.yaml",
    )

Usage (from a ROS2 node):
    flat = {
        k: node.get_parameter(k).value
        for k in node.get_parameters_by_prefix("robot")
    }
    config = RobotConfig.from_dict(flat)
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


class ConfigurationError(ValueError):
    """Raised when a required config field is null or missing."""


# ---------------------------------------------------------------------------
# Sub-dataclasses
# ---------------------------------------------------------------------------


@dataclass
class GripperConfig:
    """Gripper-specific settings from robot.yaml / overlay."""

    interface_type: str          # "gripper_action" | "joint_trajectory" | "custom"
    action: str                  # action server topic, e.g. /hand_controller/gripper_cmd
    joint_name: str              # actuated joint, e.g. joint7_left
    open_position: float         # joint position at open (m for prismatic)
    close_position: float        # joint position at close (hard stop)
    max_force: float             # Newtons
    open_width: Optional[float] = None   # fingertip gap; null until physically measured
    close_width: float = 0.0


@dataclass
class KinematicsConfig:
    plugin: str
    search_resolution: float = 0.005
    timeout: float = 0.005


# ---------------------------------------------------------------------------
# Top-level config dataclass
# ---------------------------------------------------------------------------


@dataclass
class RobotConfig:
    """Validated, typed robot configuration.

    All fields have concrete values; null sentinels from robot.yaml are
    rejected by from_dict() unless they are explicitly Optional here.
    """

    model: str
    variant: str
    dof: int
    planning_group: str
    gripper_group: str
    base_frame: str
    end_effector_link: str
    tool_frame: Optional[str]          # null → falls back to end_effector_link
    joint_names: List[str]
    joint_state_topic: str
    trajectory_action: str
    gripper: GripperConfig
    kinematics: KinematicsConfig
    moveit_package: str
    urdf_package: str
    urdf_file: str
    srdf_package: str
    srdf_file: str
    home_joints: Optional[List[float]] = None   # smoke_test.home_joints; null = skip move-to-home

    @property
    def effective_tool_frame(self) -> str:
        """tool_frame if configured, otherwise end_effector_link."""
        return self.tool_frame if self.tool_frame else self.end_effector_link

    # ------------------------------------------------------------------
    # Factory methods
    # ------------------------------------------------------------------

    @classmethod
    def from_dict(cls, p: dict) -> "RobotConfig":
        """Build from a flat dict with dot-separated keys (robot.* prefix).

        Keys mirror the YAML field paths after flattening, e.g.:
          p["robot.planning_group"] = "arm"
          p["robot.gripper.action"] = "/hand_controller/gripper_cmd"

        Raises ConfigurationError for any null required field.
        """

        def require(key: str):
            val = p.get(key)
            if val is None:
                raise ConfigurationError(
                    f"Required config field '{key}' is null or missing. "
                    "Set it in config/robots/<name>.yaml before running."
                )
            return val

        def get(key: str, default=None):
            return p.get(key, default)

        gripper = GripperConfig(
            interface_type=require("robot.gripper.interface_type"),
            action=require("robot.gripper.action"),
            joint_name=require("robot.gripper.joint_name"),
            open_position=require("robot.gripper.open_position"),
            close_position=require("robot.gripper.close_position"),
            max_force=require("robot.gripper.max_force"),
            open_width=get("robot.gripper.open_width"),
            close_width=get("robot.gripper.close_width", 0.0),
        )
        kinematics = KinematicsConfig(
            plugin=require("robot.kinematics.plugin"),
            search_resolution=get("robot.kinematics.search_resolution", 0.005),
            timeout=get("robot.kinematics.timeout", 0.005),
        )
        return cls(
            model=require("robot.model"),
            variant=require("robot.variant"),
            dof=require("robot.dof"),
            planning_group=require("robot.planning_group"),
            gripper_group=require("robot.gripper_group"),
            base_frame=require("robot.base_frame"),
            end_effector_link=require("robot.end_effector_link"),
            tool_frame=get("robot.tool_frame"),
            joint_names=require("robot.joint_names"),
            joint_state_topic=require("robot.joint_state_topic"),
            trajectory_action=require("robot.trajectory_action"),
            gripper=gripper,
            kinematics=kinematics,
            moveit_package=require("robot.moveit_package"),
            urdf_package=require("robot.urdf_package"),
            urdf_file=require("robot.urdf_file"),
            srdf_package=require("robot.srdf_package"),
            srdf_file=require("robot.srdf_file"),
            home_joints=get("smoke_test.home_joints"),
        )

    @classmethod
    def from_yaml_files(cls, *paths: str) -> "RobotConfig":
        """Load and deep-merge YAML files; later files override earlier ones.

        Requires PyYAML (pip install pyyaml).  Does not require ROS2.
        """
        import yaml  # noqa: PLC0415

        merged: dict = {}
        for path in paths:
            with open(path) as fh:
                data = yaml.safe_load(fh) or {}
            merged = _deep_merge(merged, data)
        return cls.from_dict(_flatten(merged))


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge override into base; override values win on conflict."""
    result = dict(base)
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def _flatten(d: dict, prefix: str = "") -> dict:
    """Flatten nested dict to dot-separated keys."""
    out: dict = {}
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(_flatten(v, key))
        else:
            out[key] = v
    return out
