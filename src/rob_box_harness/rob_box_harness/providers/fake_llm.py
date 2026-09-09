"""``HarnessFakeLLMProvider`` — thin re-export of FakeLLMProvider.

Bridges :class:`rob_box_llm.providers.fake.FakeLLMProvider` into the
harness world. Useful when a test wants scripted responses
(``FakeCall``) or the call-recording plumbing that ships with the
rich fake from ``rob_box_llm``.

The class IS-A :class:`rob_box_llm.provider.LLMProvider` so the
framework treats it identically to the home-grown
:class:`DummyLLMProvider`; the only difference is the underlying
scripted behaviour.
"""

from __future__ import annotations

from rob_box_llm.providers.fake import FakeLLMProvider  # re-export

# Re-export under the harness package's name so test code can
# import everything from a single namespace without coupling to the
# internal sub-package path.
HarnessFakeLLMProvider = FakeLLMProvider

__all__ = ["HarnessFakeLLMProvider", "FakeLLMProvider"]
