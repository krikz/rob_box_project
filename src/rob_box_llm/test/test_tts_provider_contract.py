"""Tests for the :class:`TTSProvider` ABC contract itself + the bundled.
:class:`FakeTTSProvider`.

These tests guard the *contract*, not a particular provider implementation
(see ``test_minimax_tts_provider.py`` for that). They exist because we
explicitly want the ABC to reject incomplete subclasses — without the test
nothing prevents a partial implementation from instantiating and then
silently ``NotImplementedError`` at call time.
"""

from __future__ import annotations

import pytest

from rob_box_llm.errors import TTSBadRequestError
from rob_box_llm.tts import (
    FakeTTSProvider,
    TTSProvider,
    TTSSettings,
)

# ---------------------------------------------------------------------------
# ABC contract — incomplete subclasses must fail at instantiation
# ---------------------------------------------------------------------------


class _IncompleteProvider(TTSProvider):
    """Subclass that forgets to implement ``synthesize`` / ``stream``."""

    name = "incomplete"


class TestABCRejectsIncompleteSubclasses:
    """Python's ``abc.ABC`` should refuse to instantiate subclasses that
    don't implement every abstract method. If this test ever flips to
    "instantiates OK", someone removed the ``@abc.abstractmethod`` decorator
    — undo that change.
    """

    def test_cannot_instantiate_subclass_missing_methods(self):
        with pytest.raises(TypeError):
            _IncompleteProvider()  # noqa: F841

    def test_fake_tts_provider_is_a_complete_subclass(self):
        # Sanity check: a complete implementation MUST instantiate without
        # raising. If this raises TypeError, the @abstractmethod decorator
        # on stream() is leaking into FakeTTSProvider (i.e. someone marked
        # it abstract on the parent and forgot to override).
        FakeTTSProvider()


# ---------------------------------------------------------------------------
# FakeTTSProvider — deterministic in-memory implementation
# ---------------------------------------------------------------------------


class TestFakeTTSProvider:
    @pytest.mark.asyncio
    async def test_synthesize_returns_pcm_samples(self):
        p = FakeTTSProvider(sample_rate=24_000)
        out = await p.synthesize("hello")
        # Same input → same byte length: len("hello") == 5 samples.
        assert len(out.samples) == 5 * 2  # 16-bit mono → 2 bytes/sample
        assert out.sample_rate == 24_000
        assert out.format.value == "pcm"
        assert out.raw == {"fake": True, "text": "hello"}

    @pytest.mark.asyncio
    async def test_synthesize_is_deterministic(self):
        p = FakeTTSProvider()
        a = await p.synthesize("same")
        b = await p.synthesize("same")
        assert a.samples == b.samples
        # And different from different input.
        c = await p.synthesize("different")
        assert c.samples != a.samples

    @pytest.mark.asyncio
    async def test_synthesize_uses_settings_sample_rate(self):
        p = FakeTTSProvider(sample_rate=8_000)  # default overridden
        out = await p.synthesize("x", settings=TTSSettings(sample_rate=16_000))
        assert out.sample_rate == 16_000  # settings wins over constructor default

    @pytest.mark.asyncio
    async def test_synthesize_raises_on_empty_text(self):
        p = FakeTTSProvider()
        with pytest.raises(TTSBadRequestError):
            await p.synthesize("")
        with pytest.raises(TTSBadRequestError):
            await p.synthesize("   ")

    @pytest.mark.asyncio
    async def test_stream_yields_single_finish_reason_stop_chunk(self):
        p = FakeTTSProvider()
        chunks = [c async for c in p.stream("hi")]
        assert len(chunks) == 1
        c = chunks[0]
        assert c.finish_reason == "stop"
        assert c.samples  # non-empty
        assert c.sample_rate > 0

    @pytest.mark.asyncio
    async def test_stream_matches_synthesize_bytes(self):
        p = FakeTTSProvider(sample_rate=16_000)
        s = TTSSettings(sample_rate=16_000)
        full = await p.synthesize("hi", settings=s)
        streamed = [c async for c in p.stream("hi", settings=s)]
        assert len(streamed) == 1
        assert streamed[0].samples == full.samples

    @pytest.mark.asyncio
    async def test_aclose_is_noop_and_idempotent(self):
        # aclose() must be safe to call multiple times — callers in finally
        # blocks rely on this.
        p = FakeTTSProvider()
        await p.aclose()
        await p.aclose()  # second call must not raise
        await p.aclose()

    def test_name_is_fake(self):
        # Canonical provider name — lets downstream code branch on
        # ``provider.name == "fake"`` for tests / offline mode.
        assert FakeTTSProvider().name == "fake"
