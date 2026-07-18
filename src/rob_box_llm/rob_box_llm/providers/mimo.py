"""MiMo provider (Xiaomi MiMo, OpenAI-compatible).

See ``deepseek.py`` for the shared implementation; this module just exposes the
public ``MiMoProvider`` class with the right ``name`` and default model so the
config in `rob_box_voice` and `rob_box_telegram` can keep using
``MiMoProvider()`` without any base_url boilerplate.
"""

from __future__ import annotations

from typing import Optional

from openai import AsyncOpenAI

from rob_box_llm.providers.deepseek import _OpenAICompatibleProvider


class MiMoProvider(_OpenAICompatibleProvider):
    """Xiaomi MiMo (OpenAI-compatible).

    The deployment lives behind a custom base_url — pass it explicitly if you
    need to point at a private gateway. The default model name matches the
    value `rob_box` is currently configured for.
    """

    DEFAULT_BASE_URL = "https://api.xiaomimimo.com/v1"
    DEFAULT_MODEL = "mimo-v2.5-pro"

    def __init__(
        self,
        *,
        base_url: str = DEFAULT_BASE_URL,
        api_key: Optional[str] = None,
        model: str = DEFAULT_MODEL,
        timeout: float = 30.0,
        client: Optional[AsyncOpenAI] = None,
    ) -> None:
        super().__init__(
            name="mimo",
            base_url=base_url,
            default_model=model,
            api_key=api_key,
            timeout=timeout,
            client=client,
        )
