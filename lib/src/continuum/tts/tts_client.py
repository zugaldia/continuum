from abc import ABC

from continuum.models import ContinuumExecutor


class ContinuumTtsClient(ContinuumExecutor, ABC):
    pass
