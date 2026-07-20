"""Concrete LLM + TTS providers shipped with rob_box_llm."""

from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_llm.providers.mimo import MiMoProvider
from rob_box_llm.providers.fake import FakeLLMProvider, FakeCall
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider

__all__ = [
    "DeepSeekProvider",
    "MiMoProvider",
    "FakeLLMProvider",
    "FakeCall",
    "MiniMaxTTSProvider",
]
