"""toms_perception.adapter – abstract perception adapter and mock implementation.

Real adapters (e.g. using YOLO, FoundationPose, or a custom ROS service)
implement PerceptionAdapterBase and are injected into BT nodes.
"""
from __future__ import annotations

import time
import uuid
from abc import ABC, abstractmethod
from typing import List

from toms_core.models import (
    BoundingBox,
    ObjectState,
    ObjectStatus,
    Pose,
    Position,
    Quaternion,
    WorldState,
)


class PerceptionAdapterBase(ABC):
    """Interface that all perception backends must satisfy."""

    @abstractmethod
    def update_world_state(self, world_state: WorldState) -> None:
        """Merge latest detections into world_state.objects in-place.

        Implementations must:
          - Set detection_confidence on each detection
          - Update last_seen timestamps
          - Not remove objects that are still tracked (debounce in world state)
          - Raise toms_core.exceptions.PerceptionError on unrecoverable failure
        """

    @abstractmethod
    def detect_objects(self, labels: List[str]) -> List[ObjectState]:
        """Return all detected objects matching any of the given labels."""


class MockPerceptionAdapter(PerceptionAdapterBase):
    """Deterministic mock for simulation and unit tests.

    Configured via a list of ObjectState fixtures injected at construction.
    Does NOT simulate physics, motion, or real sensor data.
    """

    def __init__(self, fixtures: List[ObjectState] | None = None) -> None:
        # Default: three pens and one holder on a flat table
        # TODO: replace fixture positions with values from perception.yaml
        self._fixtures: List[ObjectState] = fixtures or _default_fixtures()

    def update_world_state(self, world_state: WorldState) -> None:
        for obj in self._fixtures:
            obj.last_seen = time.time()
            obj.status = ObjectStatus.DETECTED
            world_state.objects[obj.object_id] = obj

    def detect_objects(self, labels: List[str]) -> List[ObjectState]:
        return [o for o in self._fixtures if o.label in labels]


def _default_fixtures() -> List[ObjectState]:
    """Three pens and a holder at hardcoded positions for simulation only.

    TODO: These positions must come from perception.yaml or a calibration
          procedure once the real camera and table setup is known.
    """
    holder = ObjectState(
        object_id="holder_0",
        label="pen_holder",
        pose=Pose(
            position=Position(x=0.40, y=0.00, z=0.05),
            orientation=Quaternion(w=1.0),
        ),
        status=ObjectStatus.DETECTED,
        detection_confidence=0.95,
        bounding_box=BoundingBox(
            center=Position(x=0.40, y=0.00, z=0.05),
            size_x=0.06, size_y=0.06, size_z=0.10,
        ),
    )
    pens = [
        ObjectState(
            object_id=f"pen_{i}",
            label="pen",
            pose=Pose(
                position=Position(x=0.25 + i * 0.06, y=-0.10, z=0.01),
                orientation=Quaternion(w=1.0),
            ),
            status=ObjectStatus.DETECTED,
            detection_confidence=0.85 + i * 0.03,
            bounding_box=BoundingBox(
                center=Position(x=0.25 + i * 0.06, y=-0.10, z=0.01),
                size_x=0.015, size_y=0.015, size_z=0.14,
            ),
        )
        for i in range(3)
    ]
    return [holder, *pens]
