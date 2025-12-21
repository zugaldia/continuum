from collections import OrderedDict

from continuum.models import ContinuumResponse, ContinuumRequest


#
# Match the definitions in
# ./workspace/src/continuum_interfaces/msg/AsrRequest.msg
# ./workspace/src/continuum_interfaces/msg/AsrResponse.msg
#

class ContinuumAsrRequest(ContinuumRequest):
    session_id: str

    @classmethod
    def from_ros(cls, msg: OrderedDict) -> "ContinuumAsrRequest":
        return cls.model_validate(msg)


class ContinuumAsrResponse(ContinuumResponse):
    session_id: str
    transcription: str
