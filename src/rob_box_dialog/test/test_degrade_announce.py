"""phrase_on_change / ProviderAnnouncer: одна фраза на смену состояния, не чаще 1/10 мин (ADR-0150 §8, #3693)."""

from __future__ import annotations

import pytest

from rob_box_dialog.degrade import ANNOUNCE_MIN_INTERVAL_S, ProviderAnnouncer, ProviderState, phrase_on_change


@pytest.mark.parametrize(
    "prev, cur, key",
    [
        ("ok", "quota", "provider.exhausted.quota"),
        ("ok", "network", "provider.exhausted.network"),
        ("degraded", "timeout", "provider.exhausted.timeout"),
        ("ok", "auth", "provider.exhausted.auth"),
        ("ok", "down", "provider.exhausted.error"),
        ("quota", "quota", None),  # ничего не изменилось
        ("quota", "ok", None),  # выздоровление молчит
        ("ok", "degraded", None),  # лимит частоты - ещё пробуем
    ],
)
def test_phrase_on_change_key(prev, cur, key):
    assert phrase_on_change(ProviderState("minimax", "llm", prev), ProviderState("minimax", "llm", cur)) == key


def test_first_sight_of_dead_provider_speaks_and_of_healthy_does_not():
    assert phrase_on_change(None, ProviderState("minimax", "llm", "quota")) == "provider.exhausted.quota"
    assert phrase_on_change(None, ProviderState("minimax", "llm", "ok")) is None


def test_quota_phrase_names_provider_and_fallback():
    out = ProviderAnnouncer().observe(
        [ProviderState("minimax", "llm", "quota"), ProviderState("deepseek", "llm", "ok")],
        ["minimax", "deepseek"], now=0.0)
    assert len(out) == 1
    assert "minimax" in out[0] and "кончились деньги" in out[0] and "deepseek" in out[0]


def test_without_fallback_says_so():
    out = ProviderAnnouncer().observe([ProviderState("minimax", "llm", "quota")], ["minimax"], now=0.0)
    assert len(out) == 1 and "Запасного нет" in out[0]


def test_dead_alternative_is_not_offered():
    states = [ProviderState("minimax", "llm", "quota"), ProviderState("deepseek", "llm", "auth")]
    out = ProviderAnnouncer().observe(states, ["minimax", "deepseek"], now=0.0)
    assert any("Запасного нет" in t for t in out)


def test_one_phrase_per_change_and_throttled_ten_minutes():
    a = ProviderAnnouncer()
    dead = [ProviderState("minimax", "llm", "quota"), ProviderState("deepseek", "llm", "ok")]
    ok = [ProviderState("minimax", "llm", "ok"), ProviderState("deepseek", "llm", "ok")]
    prio = ["minimax", "deepseek"]
    assert len(a.observe(dead, prio, now=0.0)) == 1
    assert a.observe(dead, prio, now=5.0) == []  # то же состояние - тишина
    assert a.observe(ok, prio, now=60.0) == []
    assert a.observe(dead, prio, now=120.0) == []  # новая смена, но < 10 мин
    assert a.observe(ok, prio, now=130.0) == []
    assert len(a.observe(dead, prio, now=ANNOUNCE_MIN_INTERVAL_S + 1)) == 1


def test_kinds_do_not_mix():
    states = [ProviderState("minimax", "tts", "quota"), ProviderState("deepseek", "llm", "ok")]
    out = ProviderAnnouncer().observe(states, ["minimax", "yandex"], now=0.0)
    assert len(out) == 1 and "yandex" in out[0]


def test_same_provider_same_reason_on_two_kinds_is_one_phrase():
    """Тариф MiniMax общий для LLM и TTS: оба вида упали по квоте - человек слышит одну фразу."""
    states = [ProviderState("minimax", "llm", "quota"), ProviderState("deepseek", "llm", "ok"),
              ProviderState("minimax", "tts", "quota"), ProviderState("yandex", "tts", "ok")]
    out = ProviderAnnouncer().observe(states, ["minimax", "deepseek", "yandex"], now=0.0)
    assert len(out) == 1


def test_stt_failure_names_the_kind_and_takes_fallback_of_same_kind():
    """#3755: отказ распознавания - про распознавание и с запасным STT, а не «переключаюсь на deepseek»."""
    states = [ProviderState("deepseek", "llm", "ok"), ProviderState("minimax", "stt", "timeout"),
              ProviderState("vosk", "stt", "ok")]
    out = ProviderAnnouncer().observe(states, {"llm": ["deepseek"], "stt": ["minimax", "vosk"]}, now=0.0)
    assert len(out) == 1 and "распознавани" in out[0] and "vosk" in out[0] and "deepseek" not in out[0]


def test_only_active_provider_of_the_kind_is_announced():
    """Сбой запасного при живом основном человеку не мешает: фразы нет."""
    prio = {"llm": ["deepseek", "minimax"]}
    states = [ProviderState("deepseek", "llm", "ok"), ProviderState("minimax", "llm", "quota")]
    assert ProviderAnnouncer().observe(states, prio, now=0.0) == []
    states = [ProviderState("deepseek", "llm", "quota"), ProviderState("minimax", "llm", "ok")]
    out = ProviderAnnouncer().observe(states, prio, now=0.0)
    assert len(out) == 1 and "переключаюсь на minimax" in out[0].lower()


def test_second_provider_becomes_active_after_first_is_dead():
    a = ProviderAnnouncer()
    prio = {"stt": ["yandex", "minimax", "vosk"]}
    s = [ProviderState("yandex", "stt", "quota"), ProviderState("minimax", "stt", "ok"), ProviderState("vosk", "stt", "ok")]
    assert len(a.observe(s, prio, now=0.0)) == 1
    s[1] = ProviderState("minimax", "stt", "timeout")
    out = a.observe(s, prio, now=1.0)
    assert len(out) == 1 and "minimax" in out[0] and "vosk" in out[0]
