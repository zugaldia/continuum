from pydantic import BaseModel, Field

from continuum.constants import (
    DEFAULT_AUDIO_CHANNELS,
    DEFAULT_AUDIO_SAMPLE_RATE,
    DEFAULT_AUDIO_SAMPLE_WIDTH,
)
from continuum.utils import generate_order_id


class ReachyAudio(BaseModel):
    audio_data: bytes
    sample_rate: int = DEFAULT_AUDIO_SAMPLE_RATE
    channels: int = DEFAULT_AUDIO_CHANNELS
    sample_width: int = DEFAULT_AUDIO_SAMPLE_WIDTH
    order_id: int = Field(default_factory=generate_order_id)
    is_initial: bool = False
