"""rob_box_llm — shared LLM + TTS provider abstraction.

P0 foundation per ADR-0001. Replaces the duplicated LLM-client code that
historically lived in rob_box_voice.dialogue_node, rob_box_telegram.llm_chat,
rob_box_telegram.mcp_bridge and rob_box_mcp_tools.llm_adapter.

LLM public surface:
    LLMProvider        — ABC; complete() and stream()
    LLMMessage, LLMResponse, LLMChunk, ToolCall, ToolResult — value objects
    DeepSeekProvider, MiMoProvider, FakeLLMProvider — concrete impls
    errors             — RateLimitError, TimeoutError, ContentFilterError, AuthError

TTS public surface (added in P0.5 — see ADR-0002):
    TTSProvider        — ABC; synthesize() and stream()
    TTSAudio, TTSChunk, TTSSettings, TTSFormat — value objects
    MiniMaxTTSProvider — concrete impl over MiniMax T2A v2 HTTP
    errors             — TTSError, TTSRateLimitError, TTSTimeoutError,
                         TTSAuthError, TTSBadRequestError
"""

from rob_box_llm.provider import (
    LLMProvider,
    LLMMessage,
    LLMResponse,
    LLMChunk,
    LLMSettings,
    ToolCall,
    ToolResult,
)
from rob_box_llm.tts import (
    TTSProvider,
    TTSAudio,
    TTSChunk,
    TTSSettings,
    TTSFormat,
)
from rob_box_llm import errors
from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_llm.providers.mimo import MiMoProvider
from rob_box_llm.providers.fake import FakeLLMProvider
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider

__all__ = [
    # LLM
    "LLMProvider",
    "LLMMessage",
    "LLMResponse",
    "LLMChunk",
    "LLMSettings",
    "ToolCall",
    "ToolResult",
    "DeepSeekProvider",
    "MiMoProvider",
    "FakeLLMProvider",
    # TTS
    "TTSProvider",
    "TTSAudio",
    "TTSChunk",
    "TTSSettings",
    "TTSFormat",
    "MiniMaxTTSProvider",
    # Errors module (re-export for convenience)
    "errors",
]

__version__ = "0.2.0"
