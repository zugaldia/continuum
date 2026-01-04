"""

We want to keep the fields in the models easily portable to ROS 2 types, which means we should limit them to the
following Python types (source: https://docs.ros.org/en/kilted/Concepts/Basic/About-Interfaces.html#field-types):

- builtins.bool (including arrays)
- builtins.int (including arrays)
- builtins.float (including arrays)
- builtins.string (including arrays)
- builtins.bytes (only a set/sequence of bytes)

Similarly, instead of using optionals, we set up default values, like the language and language-probability fields
in the ASR models.

"""

from abc import abstractmethod, ABC
from enum import Enum
from typing import Any, Callable, Dict, Optional

from pydantic import BaseModel, Field

from continuum.constants import (
    ERROR_CODE_SUCCESS,
    DEFAULT_AUDIO_FORMAT,
    DEFAULT_AUDIO_CHANNELS,
    DEFAULT_AUDIO_SAMPLE_RATE,
    DEFAULT_AUDIO_SAMPLE_WIDTH,
)
from continuum.utils import generate_unique_id, generate_order_id, generate_timestamp


class ContinuumRequest(BaseModel):
    timestamp: int = Field(default_factory=generate_timestamp)
    session_id: str = Field(default_factory=generate_unique_id)

    @classmethod
    def from_ros(cls, msg: Dict[str, Any]) -> "ContinuumRequest":
        return cls.model_validate(msg)


class ContinuumStreamingRequest(BaseModel):
    timestamp: int = Field(default_factory=generate_timestamp)
    session_id: str = Field(default_factory=generate_unique_id)

    @classmethod
    def from_ros(cls, msg: Dict[str, Any]) -> "ContinuumStreamingRequest":
        return cls.model_validate(msg)


class ContinuumResponse(BaseModel):
    timestamp: int = Field(default_factory=generate_timestamp)
    session_id: str = Field(default_factory=generate_unique_id)
    error_code: int = ERROR_CODE_SUCCESS
    error_message: str = "All systems nominal"

    @classmethod
    def from_ros(cls, msg: Dict[str, Any]) -> "ContinuumResponse":
        return cls.model_validate(msg)


class ContinuumStreamingResponse(BaseModel):
    timestamp: int = Field(default_factory=generate_timestamp)
    session_id: str = Field(default_factory=generate_unique_id)
    is_initial: bool = False
    is_final: bool = False
    order_id: int = Field(default_factory=generate_order_id)
    error_code: int = ERROR_CODE_SUCCESS
    error_message: str = "All systems nominal"

    @classmethod
    def from_ros(cls, msg: Dict[str, Any]) -> "ContinuumStreamingResponse":
        return cls.model_validate(msg)


class ContinuumExecutor(ABC):
    @abstractmethod
    async def execute_request(
        self,
        request: ContinuumRequest,
        streaming_callback: Optional[Callable[[ContinuumStreamingResponse], None]] = None,
    ) -> ContinuumResponse:
        pass

    def shutdown(self):
        pass


#
# Audio
#


class AudioComponent(BaseModel):
    audio_data: list[int] = []
    format: str = DEFAULT_AUDIO_FORMAT
    channels: int = DEFAULT_AUDIO_CHANNELS
    sample_rate: int = DEFAULT_AUDIO_SAMPLE_RATE
    sample_width: int = DEFAULT_AUDIO_SAMPLE_WIDTH

    def get_audio_bytes(self) -> bytes:
        return bytes(self.audio_data)

    def set_audio_bytes(self, audio_bytes: bytes) -> None:
        self.audio_data = list(audio_bytes)


#
# Joysticks
#


class JoystickButtonState(Enum):
    PRESSED = "PRESSED"
    RELEASED = "RELEASED"


class JoystickButton(Enum):
    BUTTON_A = "BUTTON_A"
    BUTTON_B = "BUTTON_B"
    BUTTON_X = "BUTTON_X"
    BUTTON_Y = "BUTTON_Y"
    BUTTON_L = "BUTTON_L"
    BUTTON_R = "BUTTON_R"
    BUTTON_SELECT = "BUTTON_SELECT"
    BUTTON_START = "BUTTON_START"
    BUTTON_HOME = "BUTTON_HOME"


class JoystickAxisState(Enum):
    PRESSED = "PRESSED"
    RELEASED = "RELEASED"


class JoystickAxisDirection(Enum):
    LEFT_RIGHT = "LEFT_RIGHT"
    UP_DOWN = "UP_DOWN"


class JoystickAxis(Enum):
    AXIS_LEFT = "AXIS_LEFT"
    AXIS_RIGHT = "AXIS_RIGHT"
    AXIS_UP = "AXIS_UP"
    AXIS_DOWN = "AXIS_DOWN"


#
# Echo
#


class EchoRequest(ContinuumRequest):
    message: str


class EchoResponse(ContinuumResponse):
    message: str
