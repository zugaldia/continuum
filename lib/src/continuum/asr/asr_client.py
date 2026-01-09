from abc import ABC

from continuum.models import ContinuumExecutor


class ContinuumAsrClient(ContinuumExecutor, ABC):
    """Base class for ASR clients with shared utilities."""

    pass
