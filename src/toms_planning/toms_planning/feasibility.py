"""toms_planning.feasibility – IK feasibility filter for grasp candidates.

Stage 1 of the two-stage planning pipeline (fast IK check before full MoveIt2).

TODO: Integrate with an actual IK solver (KDL, TRAC-IK, BioIK, etc.) once
      the robot URDF and planning_group name are available from robot.yaml.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from toms_core.models import GraspCandidate, GraspStatus


class FeasibilityFilterBase(ABC):
    """Interface for IK feasibility filters."""

    @abstractmethod
    def filter(self, candidates: List[GraspCandidate]) -> List[GraspCandidate]:
        """Mark each candidate as feasible/infeasible and return the full list.

        Implementations must set:
          - candidate.is_feasible = True/False
          - candidate.ik_solution = [joint angles] if feasible
          - candidate.status = GraspStatus.FEASIBLE or INFEASIBLE
        """


class MockFeasibilityFilter(FeasibilityFilterBase):
    """Mock IK filter that marks all candidates feasible.

    Replace with a real IK client once robot.yaml is filled in.
    TODO: connect to MoveIt2 IK service or run KDL chain directly.
    """

    def __init__(self, num_dof: int = 6) -> None:
        # TODO: num_dof from robot.yaml (DOF field)
        self._num_dof = num_dof

    def filter(self, candidates: List[GraspCandidate]) -> List[GraspCandidate]:
        for candidate in candidates:
            # Placeholder: all candidates are feasible with zero-position solution.
            candidate.is_feasible = True
            candidate.ik_solution = [0.0] * self._num_dof
            candidate.status = GraspStatus.FEASIBLE
        return candidates
