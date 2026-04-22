"""toms_core.exceptions – TOMS exception hierarchy."""
from __future__ import annotations


class TomsError(Exception):
    """Base class for all TOMS exceptions."""


class ValidationError(TomsError):
    """Raised when a post-condition validator detects a failure."""


class PlanningError(TomsError):
    """Raised when motion planning fails definitively (not retryable)."""


class ExecutionError(TomsError):
    """Raised when robot execution fails."""


class PerceptionError(TomsError):
    """Raised when perception cannot return a usable result."""


class RetryBudgetExceeded(TomsError):
    """Raised when all retry attempts for an object/action are exhausted."""


class ConfigurationError(TomsError):
    """Raised when a TODO config value has not been filled in."""
