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
