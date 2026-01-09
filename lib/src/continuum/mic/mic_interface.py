import logging
from abc import abstractmethod, ABC

from continuum.mic.models import MicState
from continuum.models import AudioComponent


class MicInterface(ABC):
    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._state = MicState.IDLE

    @property
    def state(self) -> MicState:
        return self._state

    @state.setter
    def state(self, value: MicState):
        self._state = value

    @abstractmethod
    def start(self, session_id: str) -> None:
        pass

    @abstractmethod
    def stop(self) -> AudioComponent:
        pass

    @abstractmethod
    def mute(self) -> None:
        pass

    @abstractmethod
    def unmute(self) -> None:
        pass

    def shutdown(self) -> None:
        pass  # Optional
