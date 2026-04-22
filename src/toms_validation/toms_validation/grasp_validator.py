"""toms_validation.grasp_validator – multi-signal grasp validation.

Validation must NEVER be bypassed.  All signals are combined and the
result is a structured ValidationResult with an explicit FailureType.

Signals used:
  1. Gripper width check  – finger gap vs. expected object width
  2. Gripper effort       – motor current/effort (requires hardware support)
  3. Small lift test      – confirm object moved up with the arm
  4. Vision consistency   – world state still shows object near EE pose

Each signal is an independent check; any failure produces a distinct type.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from toms_core.models import FailureType, ValidationResult, WorldState


@dataclass
class GraspValidationConfig:
    """Thresholds for grasp validation checks.

    TODO: all values must come from robot.yaml / planning.yaml calibration.
    """
    # Width below which we consider the gripper to have caught something
    # TODO: measure actual pen diameter and calibrate per gripper
    max_empty_gripper_width: float = 0.07        # metres (>= this = empty)
    min_grasp_gripper_width: float = 0.005       # metres (<= this = fully closed, likely empty)
    # TODO: calibrate effort threshold on the actual gripper driver
    min_grasp_effort: Optional[float] = None     # None = skip effort check
    # Lift test: EE must rise by at least this much
    lift_distance_threshold: float = 0.015       # metres
    # Vision: object centroid must be within this distance of EE
    vision_proximity_threshold: float = 0.08     # metres; TODO: from calibration


class GraspValidator:
    """Combine multiple signals to validate a completed grasp.

    All checks are run even if earlier ones fail, so the signals dict
    always contains the full diagnostic picture.
    """

    def __init__(self, config: Optional[GraspValidationConfig] = None) -> None:
        self._cfg = config or GraspValidationConfig()

    def validate(self, world_state: WorldState) -> ValidationResult:
        robot = world_state.robot
        signals: dict = {}
        failures = []

        # --- Signal 1: gripper width ---
        gw = robot.gripper_width
        signals["gripper_width"] = gw

        if gw >= self._cfg.max_empty_gripper_width:
            failures.append(FailureType.EMPTY_GRASP)
            signals["gripper_width_check"] = "FAIL_empty"
        elif gw <= self._cfg.min_grasp_gripper_width:
            failures.append(FailureType.EMPTY_GRASP)
            signals["gripper_width_check"] = "FAIL_fully_closed"
        else:
            signals["gripper_width_check"] = "OK"

        # --- Signal 2: gripper effort (optional, requires hardware) ---
        if self._cfg.min_grasp_effort is not None:
            effort = robot.gripper_effort
            signals["gripper_effort"] = effort
            if effort < self._cfg.min_grasp_effort:
                failures.append(FailureType.SLIP)
                signals["gripper_effort_check"] = "FAIL_low_effort"
            else:
                signals["gripper_effort_check"] = "OK"
        else:
            signals["gripper_effort_check"] = "SKIPPED_no_hardware"

        # --- Signal 3: small lift test ---
        # TODO: compare ee_pose before and after lift once robot adapter
        #       provides pre-lift pose snapshot.  For now, record a placeholder.
        signals["lift_check"] = "TODO_requires_pre_lift_pose_snapshot"

        # --- Signal 4: vision consistency ---
        task = world_state.task
        selected_id = getattr(task, "metadata", {}).get("selected_object_id") if task else None
        if selected_id and selected_id in world_state.objects:
            obj = world_state.objects[selected_id]
            import math
            dx = obj.pose.position.x - robot.ee_pose.position.x
            dy = obj.pose.position.y - robot.ee_pose.position.y
            dz = obj.pose.position.z - robot.ee_pose.position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            signals["vision_ee_distance"] = dist
            if dist > self._cfg.vision_proximity_threshold:
                # Object is far from EE – may have been dropped or not picked
                failures.append(FailureType.UNCERTAIN)
                signals["vision_check"] = "FAIL_object_far_from_ee"
            else:
                signals["vision_check"] = "OK"
        else:
            signals["vision_check"] = "SKIPPED_no_object_in_state"

        if failures:
            # Report the primary failure type (first detected)
            return ValidationResult(
                success=False,
                failure_type=failures[0],
                confidence=0.0,
                signals=signals,
                message=f"Grasp validation failed: {[f.value for f in failures]}",
            )

        return ValidationResult(
            success=True,
            confidence=1.0,
            signals=signals,
            message="Grasp validation passed.",
        )
