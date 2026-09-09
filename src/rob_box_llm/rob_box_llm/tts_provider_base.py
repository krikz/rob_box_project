"""TTS Provider extension ports and capability metadata.

Landed in P0.5 / ADR-0008 (commit t_25b8e221 on the ``wt/t_25b8e221``
branch, mirrored here as part of t_8cbf9995). Provides:

* :class:`BaseTTSProvider` — abstract port
* :class:`TTSCapabilities` — frozen dataclass of 8 boolean flags
* :class:`TTSVoice`        — normalized voice catalogue entry
* :class:`TTSHealth`       — frozen pre-flight health snapshot
* :class:`ProviderBuilder` — factory callback type

Subclassing rules:

* :class:`BaseTTSProvider` IS-A :class:`rob_box_llm.tts.TTSProvider` —
  every existing call site that type-annotates ``TTSProvider`` keeps
  working unchanged.
* Override :meth:`BaseTTSProvider._build_request_payload` (mandatory) —
  pure mapping ``(text, settings, voice_meta) -> dict``.
* Override :meth:`BaseTTSProvider._http_client_factory` (mandatory unless
  the default ``httpx`` works) to customise transport / TLS / proxy /
  OAuth.
* Override :meth:`capabilities`, :meth:`list_voices`, :meth:`healthcheck`
  only if the provider supports them; default impls are honest no-ops.

Backward-compat with PR #907:

* The public contract :meth:`synthesize` / :meth:`stream` / :meth:`aclose`
  on :class:`TTSProvider` is unchanged — existing call sites and tests
  continue to work.
* Code that type-annotates ``TTSProvider`` keeps resolving to the same
  runtime instance; only providers that opt-in to
  :class:`BaseTTSProvider` get the extension surface.

See also:

* ``docs/architecture/tts-extension-points.md`` — full design doc
* ``docs/adr/0008-tts-provider-extension-points-landed.md`` — Accepted ADR
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Callable, Mapping

if TYPE_CHECKING:
    import httpx

    from rob_box_llm.tts import TTSSettings, TTSProvider as _TTSProvider  # noqa: F401

# Import the runtime ABC at module load time. We do this here (instead
# of using a ``TYPE_CHECKING`` guard only) so ``BaseTTSProvider`` is a
# concrete runtime subclass — important for ``isinstance(x, TTSProvider)``
# checks performed by existing call sites after the migration.
from rob_box_llm.tts import TTSProvider  # noqa: E402


# ---------------------------------------------------------------------------
# Default connection / content limits for TTS providers
# ---------------------------------------------------------------------------

DEFAULT_MAX_CONNECTIONS: int = 10
"""Maximum concurrent HTTP connections per TTS provider pool."""

DEFAULT_MAX_CONTENT_SIZE: int = 50 * 1024 * 1024  # 50 MB
"""Maximum response body size (bytes) before a streaming provider aborts."""

DEFAULT_MAX_KEEPALIVE_CONNECTIONS: int = 5
"""Maximum idle keep-alive connections retained per provider pool."""

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

    ``extra`` is wrapped in :class:`types.MappingProxyType` at construction
    time so callers cannot mutate the catalogue entry after the provider
    published it.
    """

    id: str
    name: str
    language: str  # BCP-47, e.g. "ru", "en-US"
    gender: str = "unknown"  # "male" | "female" | "neutral" | "unknown"
    preview_url: str | None = None
    supports_cloning: bool = False
    extra: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # ``frozen=True`` blocks normal assignment; use object.__setattr__.
        if isinstance(self.extra, dict):
            object.__setattr__(self, "extra", MappingProxyType(self.extra))


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


class BaseTTSProvider(TTSProvider):
    """Extension port for future TTS providers.

    Inherits all behaviour from :class:`rob_box_llm.tts.TTSProvider`
    (``synthesize`` / ``stream`` / ``aclose`` contract — frozen at P0.5,
    see ``tts.py``) and adds 5 override hooks:

    * :meth:`capabilities`            — optional, default empty
    * :meth:`list_voices`             — optional, default empty list
    * :meth:`healthcheck`             — optional, default always-ok
    * :meth:`_build_request_payload`  — mandatory, pure mapping
    * :meth:`_http_client_factory`    — mandatory unless default httpx works

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
        SSML / specific audio formats. Default: all flags ``False``.
        """
        return TTSCapabilities()

    async def list_voices(self) -> list[TTSVoice]:
        """Return normalized voice catalogue. Default: empty list."""
        return []

    async def healthcheck(self) -> TTSHealth:
        """Pre-flight health check. Default: always ok.

        The default implementation does NOT call upstream — providers
        that need a real probe (OAuth check, ``/version`` endpoint) must
        override and cache the result.
        """
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

        Default = ``httpx.AsyncClient(timeout=<self._timeout>)``. Override
        for OAuth (Google), custom headers (ElevenLabs), proxy / TLS, etc.
        The provider DOES NOT own the returned client — caller (or
        :meth:`aclose`) handles teardown.

        ``self._timeout`` may be a plain ``float`` (applied to every
        phase — httpx semantics) or a per-phase :class:`httpx.Timeout`.
        The BLK-5 fix in :mod:`rob_box_llm.providers.minimax_tts` sets
        a per-phase value by default so a slow DNS / TLS handshake on
        the connect phase no longer burns the whole 30 s budget. This
        base method is shape-agnostic: it passes the attribute straight
        through to ``httpx.AsyncClient`` and only fills in a per-phase
        default if the subclass never set ``self._timeout`` at all.
        """
        import httpx  # local import keeps tts.py import-cheap

        timeout = getattr(self, "_timeout", None)
        if timeout is None:
            # Per-phase default applied only when a subclass skipped
            # ``self._timeout`` entirely. Matches the MiniMax LLM
            # provider's DEFAULT_TIMEOUT (see providers/minimax.py).
            timeout = httpx.Timeout(connect=5.0, read=20.0, write=10.0, pool=5.0)
        return httpx.AsyncClient(
            timeout=timeout,
            limits=httpx.Limits(
                max_connections=DEFAULT_MAX_CONNECTIONS,
                max_keepalive_connections=DEFAULT_MAX_KEEPALIVE_CONNECTIONS,
            ),
        )


__all__ = [
    "BaseTTSProvider",
    "TTSCapabilities",
    "TTSVoice",
    "TTSHealth",
    "ProviderBuilder",
]