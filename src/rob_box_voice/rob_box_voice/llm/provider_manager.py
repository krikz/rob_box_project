"""
LLM Provider Management Module

Handles configuration and initialization of different LLM providers
(OpenAI-compatible APIs like Qwen, DeepSeek, OpenAI, etc.)

This module is pure Python with no ROS dependencies, making it:
- Testable in isolation
- Reusable across different contexts
- Easy to extend with new providers
"""

import os
from dataclasses import dataclass
from typing import Dict, Optional, Any
from openai import OpenAI
from httpx import Timeout


@dataclass
class ProviderConfig:
    """Configuration for an LLM provider."""
    name: str
    base_url: str
    model: str
    env_var: str
    fallback_env: str


class ProviderManager:
    """
    Manages LLM provider configuration and client initialization.

    Features:
    - Support for multiple OpenAI-compatible providers
    - API key resolution from multiple sources
    - Provider switching (fallback support)
    - Client initialization with timeout configuration

    Example:
        ```python
        # Define providers
        providers = {
            "qwen": ProviderConfig(
                name="Qwen (Алибаба)",
                base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
                model="qwen-plus",
                env_var="LLM_API_KEY",
                fallback_env="DASHSCOPE_API_KEY"
            ),
            "deepseek": ProviderConfig(
                name="DeepSeek",
                base_url="https://api.deepseek.com",
                model="deepseek-chat",
                env_var="LLM_API_KEY",
                fallback_env="DEEPSEEK_API_KEY"
            )
        }

        # Create manager
        manager = ProviderManager(providers, default_provider="qwen")

        # Initialize client
        client = manager.initialize_client(
            api_key="your-api-key",  # or None to use env vars
            base_url=None,  # or custom URL
            model=None  # or custom model
        )

        # Try fallback if needed
        if manager.try_fallback():
            client = manager.get_client()
        ```
    """

    def __init__(
        self,
        providers: Dict[str, ProviderConfig],
        default_provider: str = "qwen",
        enable_fallback: bool = True,
        timeout_total: float = 60.0,
        timeout_connect: float = 10.0
    ):
        """
        Initialize ProviderManager.

        Args:
            providers: Dictionary mapping provider IDs to their configs
            default_provider: Default provider ID to use
            enable_fallback: Whether to allow fallback to other providers
            timeout_total: Total timeout for API requests (seconds)
            timeout_connect: Connection timeout (seconds)
        """
        self.providers = providers
        self.current_provider = default_provider
        self.enable_fallback = enable_fallback
        self.timeout_total = timeout_total
        self.timeout_connect = timeout_connect

        self.client: Optional[OpenAI] = None
        self.model: Optional[str] = None
        self.base_url: Optional[str] = None

        if default_provider not in providers:
            raise ValueError(f"Default provider '{default_provider}' not in providers")

    def get_provider_config(self, provider_id: Optional[str] = None) -> ProviderConfig:
        """
        Get configuration for a provider.

        Args:
            provider_id: Provider ID, or None for current provider

        Returns:
            ProviderConfig for the specified provider

        Raises:
            ValueError: If provider ID not found
        """
        provider_id = provider_id or self.current_provider
        if provider_id not in self.providers:
            raise ValueError(f"Unknown provider: {provider_id}")
        return self.providers[provider_id]

    def resolve_api_key(
        self,
        api_key: Optional[str] = None,
        provider_id: Optional[str] = None
    ) -> str:
        """
        Resolve API key from multiple sources.

        Priority:
        1. Explicitly provided api_key parameter
        2. Unified env variable (LLM_API_KEY)
        3. Provider-specific env variable (e.g., DASHSCOPE_API_KEY)

        Args:
            api_key: Explicitly provided API key (highest priority)
            provider_id: Provider ID, or None for current provider

        Returns:
            Resolved API key

        Raises:
            ValueError: If no API key found
        """
        provider_config = self.get_provider_config(provider_id)

        # Try provided key
        if api_key:
            return api_key

        # Try unified env variable
        api_key = os.getenv(provider_config.env_var)
        if api_key:
            return api_key

        # Try provider-specific env variable
        api_key = os.getenv(provider_config.fallback_env)
        if api_key:
            return api_key

        raise ValueError(
            f"API key not found for {provider_config.name}. "
            f"Set {provider_config.env_var} or {provider_config.fallback_env}"
        )

    def initialize_client(
        self,
        api_key: Optional[str] = None,
        base_url: Optional[str] = None,
        model: Optional[str] = None,
        provider_id: Optional[str] = None
    ) -> OpenAI:
        """
        Initialize OpenAI client for a provider.

        Args:
            api_key: API key (or None to use env vars)
            base_url: Base URL (or None to use provider default)
            model: Model name (or None to use provider default)
            provider_id: Provider ID (or None to use current)

        Returns:
            Initialized OpenAI client

        Raises:
            ValueError: If API key not found or provider unknown
        """
        provider_config = self.get_provider_config(provider_id)

        # Resolve API key
        resolved_key = self.resolve_api_key(api_key, provider_id)

        # Use provided or default base_url
        resolved_url = base_url or provider_config.base_url

        # Use provided or default model
        resolved_model = model or provider_config.model

        # Create client
        self.client = OpenAI(
            api_key=resolved_key,
            base_url=resolved_url,
            timeout=Timeout(self.timeout_total, connect=self.timeout_connect)
        )

        self.model = resolved_model
        self.base_url = resolved_url

        return self.client

    def get_client(self) -> Optional[OpenAI]:
        """Get the current OpenAI client."""
        return self.client

    def get_model(self) -> Optional[str]:
        """Get the current model name."""
        return self.model

    def get_base_url(self) -> Optional[str]:
        """Get the current base URL."""
        return self.base_url

    def get_current_provider(self) -> str:
        """Get the current provider ID."""
        return self.current_provider

    def get_fallback_provider(self) -> Optional[str]:
        """
        Get the fallback provider ID for the current provider.

        Currently uses simple logic:
        - qwen → deepseek
        - deepseek → qwen
        - others → None

        Returns:
            Fallback provider ID, or None if no fallback available
        """
        if self.current_provider == "qwen":
            return "deepseek" if "deepseek" in self.providers else None
        elif self.current_provider == "deepseek":
            return "qwen" if "qwen" in self.providers else None
        else:
            return None

    def try_fallback(
        self,
        api_key: Optional[str] = None,
        base_url: Optional[str] = None,
        model: Optional[str] = None
    ) -> bool:
        """
        Try to switch to fallback provider.

        Args:
            api_key: API key for fallback provider (or None)
            base_url: Base URL for fallback provider (or None)
            model: Model for fallback provider (or None)

        Returns:
            True if successfully switched, False otherwise
        """
        if not self.enable_fallback:
            return False

        fallback_id = self.get_fallback_provider()
        if not fallback_id:
            return False

        old_provider = self.current_provider

        try:
            # Switch provider
            self.current_provider = fallback_id

            # Try to initialize
            self.initialize_client(api_key, base_url, model)

            return True

        except Exception:
            # Restore old provider on failure
            self.current_provider = old_provider
            return False
