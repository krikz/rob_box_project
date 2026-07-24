"""Concrete providers shipped with the harness framework.

This package stays intentionally small: only the implementations that
the dummy harnesses and the smoke test need at the framework level.
Real-world providers (DeepSeek, MiniMax, MiMo, ROS2Transport, …) live
in their own packages and are wired in via the registry.
"""

from __future__ import annotations

from rob_box_harness.providers.dummy import DummyLLMProvider
from rob_box_harness.providers.fake_llm import HarnessFakeLLMProvider

__all__ = ["DummyLLMProvider", "HarnessFakeLLMProvider"]
