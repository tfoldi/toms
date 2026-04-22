"""toms_planning.grasp_generator – grasp candidate generation.

v1: simple heuristic generator (top-down grasps along object z-axis).
Future: swap in AnyGrasp or another learned backend by replacing this class.

TODO: All frame IDs, approach offsets, and gripper parameters must come
      from planning.yaml and robot.yaml.
"""
from __future__ import annotations

import uuid
from abc import ABC, abstractmethod
from typing import List

from toms_core.models import (
    GraspCandidate,
    GraspStatus,
    ObjectState,
    Pose,
    Position,
    Quaternion,
    WorldState,
)


class GraspGeneratorBase(ABC):
    """Interface for all grasp generation backends."""

    @abstractmethod
    def generate(self, object_id: str, world_state: WorldState) -> List[GraspCandidate]:
        """Return ranked grasp candidates for the specified object.

        Candidates are ordered best-first (highest score first).
        Returns an empty list if no candidates can be generated.
        """


class HeuristicGraspGenerator(GraspGeneratorBase):
    """Top-down heuristic grasp generator.

    Generates a small set of approach poses above the object centroid.
    Suitable as a starting point; replace with AnyGrasp for learned grasps.

    TODO: pre_grasp_offset_z and num_candidates should come from planning.yaml.
    TODO: approach orientation must account for actual gripper geometry and
          end-effector frame once robot.yaml is filled in.
    """

    def __init__(
        self,
        pre_grasp_offset_z: float = 0.12,   # metres above object top
        num_candidates: int = 4,
    ) -> None:
        self._pre_grasp_offset_z = pre_grasp_offset_z
        self._num_candidates = num_candidates

    def generate(self, object_id: str, world_state: WorldState) -> List[GraspCandidate]:
        obj: ObjectState | None = world_state.objects.get(object_id)
        if obj is None:
            return []

        height = (
            obj.bounding_box.size_z if obj.bounding_box is not None else 0.0
        )
        grasp_z = obj.pose.position.z + height / 2.0

        candidates: List[GraspCandidate] = []
        for i in range(self._num_candidates):
            # Rotate approach orientation around Z by i * 90°.
            # TODO: proper approach-angle grid from planning.yaml once
            #       gripper symmetry and collision geometry are known.
            angle_step = i * (3.14159 / self._num_candidates)
            import math
            qz = math.sin(angle_step / 2.0)
            qw = math.cos(angle_step / 2.0)

            candidate = GraspCandidate(
                candidate_id=str(uuid.uuid4()),
                object_id=object_id,
                pose=Pose(
                    position=Position(
                        x=obj.pose.position.x,
                        y=obj.pose.position.y,
                        z=grasp_z + self._pre_grasp_offset_z,
                    ),
                    orientation=Quaternion(z=qz, w=qw),
                ),
                score=1.0 - (i * 0.1),  # simple ranking placeholder
                status=GraspStatus.CANDIDATE,
            )
            candidates.append(candidate)

        return candidates
