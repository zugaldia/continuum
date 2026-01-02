from abc import ABC

from continuum.models import ContinuumExecutor


class AgentRunner(ContinuumExecutor, ABC):
    pass
