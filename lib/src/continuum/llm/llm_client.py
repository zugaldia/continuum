from abc import ABC

from continuum.models import ContinuumExecutor


class ContinuumLlmClient(ContinuumExecutor, ABC):
    pass
