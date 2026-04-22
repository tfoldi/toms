"""toms_core.models – canonical typed data models.

Rules:
  - No unstructured dicts in core logic; use these dataclasses.
  - No ROS2 runtime imports here; convert at package boundaries.
  - All TODO fields must be filled from config before use.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------


class ObjectStatus(str, Enum):
    UNKNOWN = "unknown"
    DETECTED = "detected"
    SELECTED = "selected"
    GRASPED = "grasped"
    PLACED = "placed"
    FAILED = "failed"


class TaskStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    SUCCESS = "success"
    FAILED = "failed"
    ABORTED = "aborted"


class FailureType(str, Enum):
    EMPTY_GRASP = "empty_grasp"
    SLIP = "slip"
    DROPPED = "dropped"
    PLACEMENT_FAILED = "placement_failed"
    UNCERTAIN = "uncertain"
    IK_FAILED = "ik_failed"
    PLANNING_FAILED = "planning_failed"
    EXECUTION_FAILED = "execution_failed"
    PERCEPTION_FAILED = "perception_failed"
    CONTAINER_NOT_FOUND = "container_not_found"


class GraspStatus(str, Enum):
    CANDIDATE = "candidate"
    FEASIBLE = "feasible"
    INFEASIBLE = "infeasible"
    ATTEMPTED = "attempted"
    SUCCESS = "success"
    FAILED = "failed"


# ---------------------------------------------------------------------------
# Primitive geometry types (ROS-agnostic)
# Convert to/from geometry_msgs at the ROS2 package boundary only.
# ---------------------------------------------------------------------------


@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Pose:
    position: Position = field(default_factory=Position)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class BoundingBox:
    """3-D axis-aligned bounding box in world frame (metres)."""
    center: Position = field(default_factory=Position)
    size_x: float = 0.0
    size_y: float = 0.0
    size_z: float = 0.0


# ---------------------------------------------------------------------------
# Core domain models
# ---------------------------------------------------------------------------


@dataclass
class ObjectState:
    """State of a single tracked object (pen, holder, …)."""

    object_id: str
    label: str
    pose: Pose = field(default_factory=Pose)
    bounding_box: Optional[BoundingBox] = None
    status: ObjectStatus = ObjectStatus.UNKNOWN
    detection_confidence: float = 0.0
    grasp_attempts: int = 0
    last_seen: float = field(default_factory=time.time)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class RobotState:
    """Current robot hardware state.

    All topic/controller/joint identifiers come from robot.yaml.
    Do NOT hardcode joint names, topic paths, or frame IDs here.
    """

    # TODO: fill from robot.yaml (arm model, DOF, end-effector link)
    is_moving: bool = False
    gripper_width: float = 0.0       # metres; TODO: calibrate per gripper model
    gripper_effort: float = 0.0      # normalised 0-1; TODO: source from hardware
    joint_positions: List[float] = field(default_factory=list)
    ee_pose: Pose = field(default_factory=Pose)
    is_holding_object: bool = False
    timestamp: float = field(default_factory=time.time)


@dataclass
class TaskState:
    """Mutable task-level bookkeeping (retry budget, object lists)."""

    task_id: str
    status: TaskStatus = TaskStatus.IDLE
    retry_budget: int = 3
    retries_used: int = 0
    objects_pending: List[str] = field(default_factory=list)
    objects_complete: List[str] = field(default_factory=list)
    objects_failed: List[str] = field(default_factory=list)
    start_time: float = field(default_factory=time.time)
    end_time: Optional[float] = None


@dataclass
class WorldState:
    """Canonical world snapshot – single source of truth for all modules.

    Only toms_world.WorldStateManager should mutate this; all other
    packages receive a copy (or reference) and must write back via the manager.
    """

    timestamp: float = field(default_factory=time.time)
    objects: Dict[str, ObjectState] = field(default_factory=dict)
    robot: RobotState = field(default_factory=RobotState)
    task: Optional[TaskState] = None
    container: Optional[ObjectState] = None   # the pen holder
    sequence: int = 0                          # monotonic update counter


# ---------------------------------------------------------------------------
# Planning / motion models
# ---------------------------------------------------------------------------


@dataclass
class GraspCandidate:
    """A single candidate grasp pose with feasibility metadata."""

    candidate_id: str
    object_id: str
    pose: Pose = field(default_factory=Pose)
    score: float = 0.0               # higher = better
    status: GraspStatus = GraspStatus.CANDIDATE
    is_feasible: Optional[bool] = None
    ik_solution: Optional[List[float]] = None
    # TODO: approach vector, pre-grasp offset once arm kinematics are known
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class PlanRequest:
    """Request to the MoveIt2 motion planner."""

    object_id: str
    grasp_candidate: GraspCandidate
    # TODO: all three fields must be overridden from robot.yaml at runtime
    planning_group: str = "TODO_SET_IN_CONFIG"
    end_effector_link: str = "TODO_SET_IN_CONFIG"
    base_frame: str = "TODO_SET_IN_CONFIG"
    allow_replanning: bool = True
    max_attempts: int = 3


@dataclass
class PlanResult:
    """Result from the MoveIt2 motion planner."""

    success: bool
    trajectory: Optional[Any] = None  # trajectory_msgs/JointTrajectory when real
    planning_time: float = 0.0
    error_code: Optional[str] = None
    error_message: Optional[str] = None


# ---------------------------------------------------------------------------
# Validation models
# ---------------------------------------------------------------------------


@dataclass
class ValidationResult:
    """Structured outcome from any validator (grasp or placement)."""

    success: bool
    failure_type: Optional[FailureType] = None
    confidence: float = 1.0
    signals: Dict[str, Any] = field(default_factory=dict)
    # Common signal keys: gripper_width, lift_displacement, vision_match,
    #                     in_holder_zone, still_present_after_retreat
    message: str = ""


# ---------------------------------------------------------------------------
# Task outcome
# ---------------------------------------------------------------------------


@dataclass
class TaskOutcome:
    """Final summary emitted when a task run ends."""

    task_id: str
    success: bool
    objects_placed: List[str] = field(default_factory=list)
    objects_failed: List[str] = field(default_factory=list)
    total_attempts: int = 0
    duration_sec: float = 0.0
    failure_reasons: List[str] = field(default_factory=list)
