"""

We want to keep the fields in the models easily portable to ROS 2 types, which means we should limit them to the
following Python types:
- builtins.bool
- builtins.bytes*
- builtins.float*
- builtins.int*
- builtins.str

Refs: https://docs.ros.org/en/kilted/Concepts/Basic/About-Interfaces.html#field-types

"""

from abc import abstractmethod, ABC

from pydantic import BaseModel


class ContinuumRequest(BaseModel):
    pass


class ContinuumResponse(BaseModel):
    pass


class ContinuumClient(ABC):
    @abstractmethod
    async def execute_request(self, request: ContinuumRequest) -> ContinuumResponse:
        pass

    def shutdown(self):
        pass
