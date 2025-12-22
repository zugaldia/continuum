from abc import ABC

from continuum.models import ContinuumClient


class ContinuumLlmClient(ContinuumClient, ABC):
    pass
