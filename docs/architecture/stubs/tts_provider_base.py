"""TTS Provider extension ports and capability metadata — DESIGN ONLY (t_8d714ff0).

**STATUS: design-only stub. NOT imported by production code.**

This file exists purely to document the proposed extension surface for
future TTS providers (ElevenLabs / Google / Azure / local Piper). It is
kept under ``docs/architecture/stubs/`` rather than the production source
tree (``src/rob_box_llm/``) so it cannot accidentally be imported.

When this design is accepted (ADR-0007 → Accepted and t_25b8e221 lands),
this file will move to ``src/rob_box_llm/rob_box_llm/tts_provider_base.py``
and acquire real implementations.

Public contract (frozen once implemented):

    * ``BaseTTSProvider``           — abstract port
    * ``TTSCapabilities``           — frozen dataclass of 8 boolean flags
    * ``TTSVoice``                  — normalized voice catalogue entry
    * ``TTSHealth``                 — frozen pre-flight health snapshot
    * ``ProviderBuilder``           — factory callback type

Subclassing rules:

    * Override ``_build_request_payload`` (mandatory) — pure mapping
      ``(text, settings, voice_meta) -> dict``.
    * Override ``_http_client_factory`` (mandatory unless default httpx works)
      to customise transport / TLS / proxy / OAuth.
    * Override ``capabilities``, ``list_voices``, ``healthcheck`` only if the
      provider supports them; default impls are honest no-ops.

Backward-compat with PR #907:

    * ``BaseTTSProvider`` IS-A ``TTSProvider`` — every existing call site
      that type-annotates ``TTSProvider`` keeps working unchanged.
    * ``MiniMaxTTSProvider(TTSProvider)`` stays untouched until the second
      opt-in provider lands; migration is a single-line
      ``class MiniMaxTTSProvider(BaseTTSProvider)``.

See also:
    * ``docs/architecture/tts-extension-points.md`` — full design doc
    * ``docs/adr/0007-minimax-tts-integration-final.md`` — parent ADR
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Mapping

if TYPE_CHECKING:
    import httpx

    from rob_box_llm.tts import TTSSettings


# ---------------------------------------------------------------------------
# Capability metadata — frozen dataclass, 8 boolean flags
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TTSCapabilities:
    """Static capability declaration for a TTS provider.

    Providers override :meth:`BaseTTSProvider.capabilities` to return an
    instance with only the flags they actually support; every flag is
    ``False`` by default, so an unconfigured provider is always
    capability-honest (no silent degradation).
    """

    streaming: bool = False
    voice_cloning: bool = False
    ssml: bool = False
    pronunciation_dict: bool = False
    audio_format_pcm: bool = False
    audio_format_mp3: bool = False
    audio_format_ogg: bool = False
    custom_endpoint: bool = False  # for local / on-prem providers


# ---------------------------------------------------------------------------
# Normalized voice catalogue entry
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TTSVoice:
    """Provider-neutral voice description.

    Different providers expose wildly different voice metadata (MiniMax
    uses ``"Calm_Woman"``, ElevenLabs uses UUIDs, Google uses
    ``"en-US-Wavenet-A"``, local Piper uses ``.onnx`` filenames).
    This dataclass is the smallest common subset the rest of rob_box
    can rely on. Provider-specific extras go in ``extra``.
    """

    id: str
    name: str
    language: str  # BCP-47, e.g. "ru", "en-US"
    gender: str = "unknown"  # "male" | "female" | "neutral" | "unknown"
    preview_url: str | None = None
    supports_cloning: bool = False
    extra: Mapping[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Pre-flight health snapshot
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TTSHealth:
    """Snapshot returned by :meth:`BaseTTSProvider.healthcheck`.

    Default impl returns ``(ok=True, provider=self.name, latency_ms=0.0)`` —
    no upstream call. Production providers override this to do a cheap
    pre-flight (e.g. list-voices with TTL cache, OAuth-token check,
    subprocess ``--version``). Logging secrets is forbidden — only
    ``provider / ok / latency_ms / reason`` are exposed.
    """

    ok: bool
    provider: str
    latency_ms: float = 0.0
    reason: str | None = None  # short human-readable string when ok=False


# ---------------------------------------------------------------------------
# Factory callback type — registry input
# ---------------------------------------------------------------------------


# Forward-declared so the type alias is valid even before TTSProviderRegistry
# imports this module.
if TYPE_CHECKING:
    ProviderBuilder = Callable[[Mapping[str, Any]], "BaseTTSProvider"]
else:
    ProviderBuilder = Callable[[Mapping[str, Any]], "BaseTTSProvider"]
"""Factory callback used by :class:`TTSProviderRegistry`.

Takes a config dict (already validated by pydantic-settings in CLI path,
or ROS-params dict in ROS path) and returns a fully constructed
:class:`BaseTTSProvider`. Builders SHOULD be idempotent — calling twice
with the same config returns equivalent instances.
"""


# ---------------------------------------------------------------------------
# Extension port
# ---------------------------------------------------------------------------


class BaseTTSProvider(abc.ABC):
    """Extension port for future TTS providers.

    Inherits all behaviour from :class:`rob_box_llm.tts.TTSProvider`
    (synthesize / stream / aclose contract — frozen at P0.5,
    see ``tts.py:121-177``) and adds 5 override hooks:

    * :meth:`capabilities`         — optional, default empty
    * :meth:`list_voices`          — optional, default empty list
    * :meth:`healthcheck`          — optional, default always-ok
    * :meth:`_build_request_payload` — mandatory, pure mapping
    * :meth:`_http_client_factory`   — mandatory unless default httpx works

    .. note::

        Existing ``MiniMaxTTSProvider(TTSProvider)`` does NOT migrate to
        this base in this PR — that change is deferred until the second
        opt-in provider lands (ADR-0004 §2.8). Migrating earlier is YAGNI.

    Subclassing example::

        class ElevenLabsTTSProvider(BaseTTSProvider):
            name = "elevenlabs"

            async def _http_client_factory(self) -> "httpx.AsyncClient":
                import httpx
                return httpx.AsyncClient(
                    base_url="https://api.elevenlabs.io",
                    headers={"xi-api-key": self._api_key},
                    timeout=httpx.Timeout(self._timeout),
                )

            def _build_request_payload(
                self,
                text: str,
                settings: "TTSSettings",
                voice_meta: "TTSVoice | None",
            ) -> dict[str, Any]:
                return {
                    "text": text,
                    "voice_settings": {"stability": 0.5, "similarity_boost": 0.75},
                    "model_id": settings.model or "eleven_monolingual_v1",
                    "output_format": "mp3_44100_128",
                }
    """

    name: str = "abstract-base"

    # ---- optional extension points (default = honest no-op) ----

    def capabilities(self) -> TTSCapabilities:
        """Static capability declaration.

        Override if your provider supports streaming / voice-cloning /
        SSML / specific audio formats. Default: all flags False.
        """
        return TTSCapabilities()

    async def list_voices(self) -> list[TTSVoice]:
        """Return normalized voice catalogue. Default: empty list."""
        return []

    async def healthcheck(self) -> TTSHealth:
        """Pre-flight health check. Default: always ok."""
        return TTSHealth(ok=True, provider=self.name)

    # ---- mandatory extension points (override required) ----

    @abc.abstractmethod
    def _build_request_payload(
        self,
        text: str,
        settings: "TTSSettings",
        voice_meta: TTSVoice | None,
    ) -> dict[str, Any]:
        """Pure mapping ``TTSSettings → provider-specific JSON body``.

        MUST be a pure function: no HTTP calls, no logging of secrets, no
        global state mutation. Tested in isolation via ``unittest.mock``,
        no transport mocking needed.
        """

    def _http_client_factory(self) -> "httpx.AsyncClient":
        """Construct the HTTP client this provider uses.

        Default = ``httpx.AsyncClient(timeout=self._timeout)``. Override
        for OAuth (Google), custom headers (ElevenLabs), proxy / TLS, etc.
        The provider DOES NOT own the returned client — caller (or
        :meth:`aclose`) handles teardown.
        """
        import httpx  # local import keeps tts.py import-cheap

        timeout: float = getattr(self, "_timeout", 30.0)
        return httpx.AsyncClient(timeout=timeout)


__all__ = [
    "BaseTTSProvider",
    "TTSCapabilities",
    "TTSVoice",
    "TTSHealth",
    "ProviderBuilder",
]