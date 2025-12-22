"""

We want to keep the fields in the models easily portable to ROS 2 types, which means we should limit them to the
following Python types (source: https://docs.ros.org/en/kilted/Concepts/Basic/About-Interfaces.html#field-types):

- builtins.bool
- builtins.bytes*
- builtins.float*
- builtins.int*
- builtins.str

Similarly, instead of using optionals, we set up default values, like the language and language probability fields
in the ASR models.

"""

from abc import abstractmethod, ABC
from collections import OrderedDict
from typing import Callable, Optional

from pydantic import BaseModel, Field

from continuum.utils import generate_session_id


class ContinuumRequest(BaseModel):
    session_id: str = Field(default_factory=generate_session_id)

    @classmethod
    def from_ros(cls, msg: OrderedDict) -> "ContinuumRequest":
        return cls.model_validate(msg)


class ContinuumStreamingRequest(BaseModel):
    session_id: str = Field(default_factory=generate_session_id)

    @classmethod
    def from_ros(cls, msg: OrderedDict) -> "ContinuumStreamingRequest":
        return cls.model_validate(msg)


class ContinuumResponse(BaseModel):
    session_id: str = Field(default_factory=generate_session_id)


class ContinuumStreamingResponse(BaseModel):
    session_id: str = Field(default_factory=generate_session_id)


class ContinuumClient(ABC):
    @abstractmethod
    async def execute_request(
            self,
            request: ContinuumRequest,
            streaming_callback: Optional[Callable[[ContinuumStreamingResponse], None]] = None,
    ) -> ContinuumResponse:
        pass

    def shutdown(self):
        pass
