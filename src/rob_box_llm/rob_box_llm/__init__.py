"""rob_box_llm — shared LLM + TTS provider abstraction.

P0 foundation per ADR-0001. Replaces the duplicated LLM-client code that
historically lived in rob_box_voice.dialogue_node, rob_box_telegram.llm_chat,
rob_box_telegram.mcp_bridge and rob_box_mcp_tools.llm_adapter.

LLM public surface:
    LLMProvider        — ABC; complete() and stream()
    LLMMessage, LLMResponse, LLMChunk, ToolCall, ToolResult — value objects
    TextPart, ImagePart, MessageContent — multimodal content (P1 / M0)
    ProviderCapabilities                 — capability introspection (P1 / M0)
    DeepSeekProvider, MiMoProvider, MiniMaxProvider, FakeLLMProvider — concrete impls
    errors             — RateLimitError, TimeoutError, ContentFilterError,
                         AuthError, CapabilityUnavailableError, ProviderError

TTS public surface (added in P0.5 — see ADR-0003):
    TTSProvider        — ABC; synthesize() and stream()
    TTSAudio, TTSChunk, TTSSettings, TTSFormat — value objects
    MiniMaxTTSProvider — concrete impl over MiniMax T2A v2 HTTP
    FakeTTSProvider    — deterministic in-memory impl, for tests
    errors             — TTSError, TTSRateLimitError, TTSTimeoutError,
                         TTSAuthError, TTSBadRequestError
"""

from __future__ import annotations

from rob_box_llm.errors import (
    AuthError,
    ContentFilterError,
    ProviderError,
    RateLimitError,
    TimeoutError,
    TTSAuthError,
    TTSBadRequestError,
    TTSError,
    TTSRateLimitError,
    TTSTimeoutError,
)
from rob_box_llm.provider import (
    ImagePart,
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    MessageContent,
    ProviderCapabilities,
    TextPart,
    ToolCall,
    ToolResult,
    TextPart,
    ImagePart,
    MessagePart,
    MessageContent,
    ProviderCapabilities,
)
from rob_box_llm.tts import (
    TTSProvider,
    TTSAudio,
    TTSChunk,
    TTSSettings,
    TTSFormat,
    FakeTTSProvider,
)
from rob_box_llm import errors
from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_llm.providers.fake import FakeCall, FakeLLMProvider
from rob_box_llm.providers.mimo import MiMoProvider
from rob_box_llm.providers.minimax import MiniMaxProvider
from rob_box_llm.providers.fake import FakeLLMProvider
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider

__all__ = [
    # LLM
    "LLMProvider",
    "LLMMessage",
    "LLMResponse",
    "LLMChunk",
    "ToolCall",
    "ToolResult",
    "TextPart",
    "ImagePart",
    "MessagePart",
    "MessageContent",
    "ProviderCapabilities",
    "errors",
    "DeepSeekProvider",
    "MiMoProvider",
    "MiniMaxProvider",
    "FakeLLMProvider",
    # TTS
    "TTSProvider",
    "TTSAudio",
    "TTSChunk",
    "TTSSettings",
    "TTSFormat",
    "MiniMaxTTSProvider",
    "FakeTTSProvider",
    # Errors module (re-export for convenience)
    "errors",
]

__version__ = "0.2.1"
