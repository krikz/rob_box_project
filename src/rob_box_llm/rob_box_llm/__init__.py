"""rob_box_llm — shared LLM provider abstraction.

P0 foundation per ADR-0001. Replaces the duplicated LLM-client code that
historically lived in rob_box_voice.dialogue_node, rob_box_telegram.llm_chat,
rob_box_telegram.mcp_bridge and rob_box_mcp_tools.llm_adapter.

Public surface:
    LLMProvider        — ABC; complete() and stream()
    LLMMessage, LLMResponse, LLMChunk, ToolCall, ToolResult — value objects
    DeepSeekProvider, MiMoProvider, FakeLLMProvider — concrete impls
    errors             — RateLimitError, TimeoutError, ContentFilterError, ProviderError
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
from rob_box_llm import errors
from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_llm.providers.mimo import MiMoProvider
from rob_box_llm.providers.fake import FakeLLMProvider

__all__ = [
    "LLMProvider",
    "LLMMessage",
    "LLMResponse",
    "LLMChunk",
    "LLMSettings",
    "ToolCall",
    "ToolResult",
    "errors",
    "DeepSeekProvider",
    "MiMoProvider",
    "FakeLLMProvider",
]

__version__ = "0.1.0"
