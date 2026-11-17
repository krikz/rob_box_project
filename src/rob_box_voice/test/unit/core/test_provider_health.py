"""Состояния TTS/STT для /voice/providers/state и реакция репортёра на чужие виды (PR-4 ч.2, #3693)."""

from __future__ import annotations

from types import SimpleNamespace

from rob_box_core.provider_publisher import ProviderStatePublisher
from rob_box_dialog.degrade import ProviderBoard, ProviderState
from rob_box_llm.errors import TTSAuthError, TTSConnectionError, TTSRateLimitError, TTSTimeoutError
from rob_box_voice.core.provider_health import (
    record_tts_death, stt_provider_states, tts_failure_class, tts_provider_states,
)
from rob_box_voice.core.provider_reporter import ProviderStateReporter, providers_context, reporter_context
from rob_box_voice.stt_fallback import ProviderDeadCache, STTAuthError, STTQuotaError, STTTimeoutError, failure_class


def test_tts_failure_classes():
    assert tts_failure_class(TTSRateLimitError("429"), permanent=True) == "quota"
    assert tts_failure_class(TTSAuthError("401"), permanent=True) == "auth"
    assert tts_failure_class(RuntimeError("PERMISSION_DENIED"), permanent=True) == "auth"
    assert tts_failure_class(TTSConnectionError("dns"), permanent=False) == "network"
    assert tts_failure_class(TTSTimeoutError("t"), permanent=False) == "timeout"
    assert tts_failure_class(RuntimeError("boom"), permanent=False) == "down"


def _tts_node(dead):
    return SimpleNamespace(provider_chain=["minimax", "yandex", "silero"], _provider_is_dead=lambda n: n in dead)


def test_tts_states_report_class_of_dead_provider():
    node = _tts_node({"minimax"})
    record_tts_death(node, "minimax", TTSRateLimitError("429"), True)
    got = {s.name: s.health for s in tts_provider_states(node)}
    assert got == {"minimax": "quota", "yandex": "ok", "silero": "ok"}
    assert all(s.kind == "tts" for s in tts_provider_states(node))


def test_tts_dead_via_shared_llm_cache_takes_its_class():
    shared = SimpleNamespace(get=lambda n: SimpleNamespace(health_class="quota"))
    node = SimpleNamespace(provider_chain=["minimax"], _provider_is_dead=lambda n: True, _shared_health_cache=shared)
    assert tts_provider_states(node)[0].health == "quota"


def test_stt_dead_cache_keeps_failure_class():
    clock = [0.0]
    cache = ProviderDeadCache(clock=lambda: clock[0])
    cache.mark_dead("minimax", "x", transient=False, health_class=failure_class(None, STTQuotaError("q")))
    cache.mark_dead("yandex", "x", transient=False, health_class=failure_class(None, STTAuthError("a")))
    assert failure_class("timeout>5s", STTTimeoutError("t")) == "timeout" and failure_class("e", None) == "down"
    node = SimpleNamespace(provider_chain=["minimax", "yandex", "vosk"], _provider_dead_cache=cache)
    assert {s.name: s.health for s in stt_provider_states(node)} == {"minimax": "quota", "yandex": "auth", "vosk": "ok"}
    clock[0] = 10_000.0
    assert {s.health for s in stt_provider_states(node)} == {"ok"}


def test_reporter_announces_tts_failure_from_board_once_and_adds_context():
    now = [0.0]
    board = ProviderBoard()
    spoken, published = [], []
    rep = ProviderStateReporter(object(), published.append, spoken.append, board=board, clock=lambda: now[0])
    board.update([ProviderState("minimax", "tts", "quota"), ProviderState("yandex", "tts", "ok")], now[0])
    rep.tick()
    rep.tick()
    assert published == [] and len(spoken) == 1 and "minimax" in spoken[0] and "yandex" in spoken[0]
    assert reporter_context(rep) == providers_context(rep.all_states()) == [
        {"kind": "tts", "name": "minimax", "health": "quota"}]
    assert reporter_context(None) == []


def test_context_is_empty_when_everything_is_fine():
    assert providers_context([ProviderState("minimax", "tts", "ok"), ProviderState("kie", "music_gen", "unknown")]) == []


def test_publisher_republishes_every_tick_and_skips_empty(monkeypatch):
    import rob_box_core.provider_publisher as pp

    sent = []
    monkeypatch.setattr(pp, "make_publisher", lambda node: "pub")
    monkeypatch.setattr(pp, "publish_provider_states", lambda pub, states: sent.append((pub, list(states))))
    node = SimpleNamespace(create_timer=lambda period, cb: (period, cb))
    states = [[ProviderState("a", "tts", "ok")], []]
    publisher = ProviderStatePublisher(node, lambda: states.pop(0))
    publisher.tick()
    publisher.tick()
    assert len(sent) == 1 and sent[0][0] == "pub"
    assert node.create_timer(2.0, None)[0] == 2.0
