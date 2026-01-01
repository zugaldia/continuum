from abc import ABC

from continuum.models import ContinuumExecutor


class ContinuumAsrClient(ContinuumExecutor, ABC):
    pass
