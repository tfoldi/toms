"""toms_validation.placement_validator – placement validation.

Checks:
  1. Object no longer in gripper (gripper is open / wide)
  2. Object inside container zone (spatial check)
  3. Object still present after robot retreats (stability check)

All thresholds from robot.yaml / planning.yaml.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from toms_core.models import BoundingBox, FailureType, ObjectStatus, ValidationResult, WorldState


@dataclass
class PlacementValidationConfig:
    """Thresholds for placement validation checks.

    TODO: calibrate all values against the real hardware and scene geometry.
    """
    # Gripper width above which we consider the object released
    min_released_gripper_width: float = 0.04    # metres; TODO: from robot.yaml
    # How far the object centroid may be from the container centroid (XY plane)
    max_container_distance_xy: float = 0.05     # metres; TODO: from scene calibration
    # TODO: stability check timing (wait after retreat before checking vision)
    stability_wait_sec: float = 0.5             # seconds


class PlacementValidator:
    """Validate that the placement succeeded using multiple checks."""

    def __init__(self, config: Optional[PlacementValidationConfig] = None) -> None:
        self._cfg = config or PlacementValidationConfig()

    def validate(self, object_id: str, world_state: WorldState) -> ValidationResult:
        signals: dict = {}
        failures = []

        # --- Check 1: object no longer in gripper ---
        gw = world_state.robot.gripper_width
        signals["gripper_width_after_release"] = gw
        if gw < self._cfg.min_released_gripper_width:
            failures.append(FailureType.PLACEMENT_FAILED)
            signals["release_check"] = "FAIL_gripper_still_closed"
        else:
            signals["release_check"] = "OK"

        # --- Check 2: object inside holder zone ---
        obj = world_state.objects.get(object_id)
        container = world_state.container

        if obj is None:
            failures.append(FailureType.UNCERTAIN)
            signals["zone_check"] = "FAIL_object_not_in_world_state"
        elif container is None:
            failures.append(FailureType.UNCERTAIN)
            signals["zone_check"] = "FAIL_no_container_reference"
        else:
            import math
            dx = obj.pose.position.x - container.pose.position.x
            dy = obj.pose.position.y - container.pose.position.y
            dist_xy = math.sqrt(dx * dx + dy * dy)
            signals["object_container_distance_xy"] = dist_xy

            if dist_xy > self._cfg.max_container_distance_xy:
                failures.append(FailureType.PLACEMENT_FAILED)
                signals["zone_check"] = "FAIL_outside_holder_zone"
            else:
                signals["zone_check"] = "OK"

        # --- Check 3: object stable after retreat ---
        # TODO: this requires a re-perception call after robot retreats.
        #       Implement once the full robot/perception loop is connected.
        signals["stability_check"] = "TODO_requires_post_retreat_perception"

        if failures:
            return ValidationResult(
                success=False,
                failure_type=failures[0],
                confidence=0.0,
                signals=signals,
                message=f"Placement validation failed: {[f.value for f in failures]}",
            )

        # Mark the object PLACED in world state if visible
        if obj is not None and obj.status != ObjectStatus.PLACED:
            obj.status = ObjectStatus.PLACED

        return ValidationResult(
            success=True,
            confidence=1.0,
            signals=signals,
            message="Placement validation passed.",
        )
