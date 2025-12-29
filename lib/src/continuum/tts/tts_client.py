from abc import ABC

from continuum.models import ContinuumClient


class ContinuumTtsClient(ContinuumClient, ABC):
    pass
