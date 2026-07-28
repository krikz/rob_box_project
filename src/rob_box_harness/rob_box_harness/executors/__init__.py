"""Concrete tool executors used by dialog and Telegram harness adapters."""

from rob_box_harness.executors.core_adapter import (
    LegacyToolProviderAdapter,
    LocalSkillProviderAdapter,
    MCPBridgeProviderAdapter,
    adapt_tool_provider,
)
from rob_box_harness.executors.local import (
    HarnessIntrospection,
    LocalSkillProvider,
    LocalToolProvider,
    build_default_local_skill_provider,
)
from rob_box_harness.executors.mcp_bridge import (
    MCPBackpressureError,
    MCPBridgeExecutor,
    MCPBridgeProvider,
    MCPRateLimit,
    MCPRetryPolicy,
    MCPTransportError,
    build_default_mcp_bridge_provider,
)

from rob_box_harness.executors.ros_mcp import (
    ROSMCPBridge,
    ROSMCPToolProvider,
    descriptor_from_function_tool,
)

__all__ = [
    "LegacyToolProviderAdapter",
    "LocalSkillProviderAdapter",
    "MCPBridgeProviderAdapter",
    "adapt_tool_provider",
    "HarnessIntrospection",
    "LocalSkillProvider",
    "LocalToolProvider",
    "build_default_local_skill_provider",
    "MCPBackpressureError",
    "MCPBridgeExecutor",
    "MCPBridgeProvider",
    "MCPRateLimit",
    "MCPRetryPolicy",
    "MCPTransportError",
    "build_default_mcp_bridge_provider",
    "ROSMCPBridge",
    "ROSMCPToolProvider",
    "descriptor_from_function_tool",
]
