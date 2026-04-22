"""toms_planning.place_pose – generate placement poses inside the container.

v1: simple heuristic (place above container center).
TODO: replace with a bin-packing or learned placement policy if needed.
"""
from __future__ import annotations

import uuid

from toms_core.models import (
    GraspCandidate,
    GraspStatus,
    Pose,
    Position,
    Quaternion,
    WorldState,
)


class HeuristicPlacePoseAdapter:
    """Generate a placement pose above the container centroid.

    TODO: place_height_above_container from planning.yaml.
    TODO: account for object height (avoid collision with container walls).
    """

    def __init__(self, place_height_above_container: float = 0.15) -> None:
        # TODO: load from planning.yaml
        self._height = place_height_above_container

    def generate_place_pose(
        self,
        object_id: str,
        container_id: str,
        world_state: WorldState,
    ) -> GraspCandidate:
        container = world_state.container
        if container is None:
            # Fallback: return an identity pose so the tree can still fail
            # gracefully through PlanResult rather than raising.
            cx, cy, cz = 0.0, 0.0, 0.0
        else:
            cx = container.pose.position.x
            cy = container.pose.position.y
            cz = container.pose.position.z

        return GraspCandidate(
            candidate_id=str(uuid.uuid4()),
            object_id=object_id,
            pose=Pose(
                position=Position(x=cx, y=cy, z=cz + self._height),
                orientation=Quaternion(w=1.0),  # TODO: align to container
            ),
            score=1.0,
            status=GraspStatus.CANDIDATE,
        )
