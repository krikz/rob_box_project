"""Unit-тесты SessionVoiceState + rate limiter."""

import time

import pytest

from rob_box_quest.voice import (
    PREVIEW_RATE_LIMIT,
    PREVIEW_WINDOW_S,
    SessionVoiceState,
    VoiceStateRegistry,
)


class TestSessionVoiceState:
    def test_initial_defaults(self):
        s = SessionVoiceState(session_id="abc")
        assert s.session_id == "abc"
        assert s.active_voice_id is None
        assert s.active_preset == "standard"
        assert s.listening is False
        assert s.last_error is None

    def test_apply_voice(self):
        s = SessionVoiceState(session_id="abc")
        s.apply_voice("anton", "friendly")
        assert s.active_voice_id == "anton"
        assert s.active_preset == "friendly"
        assert s.last_error is None  # успех сбрасывает прошлую ошибку

    def test_apply_voice_after_error_clears_error(self):
        s = SessionVoiceState(session_id="abc")
        s.set_error("boom")
        s.apply_voice("anton", "standard")
        assert s.last_error is None

    def test_set_error(self):
        s = SessionVoiceState(session_id="abc")
        s.set_error("voice-pipeline offline")
        assert s.last_error == "voice-pipeline offline"

    def test_to_state_payload_keys(self):
        s = SessionVoiceState(session_id="abc")
        s.apply_voice("alena", "whisper")
        s.set_error("oops")
        payload = s.to_state_payload(ts_ms=123)
        assert payload["active_voice_id"] == "alena"
        assert payload["active_preset"] == "whisper"
        assert payload["last_error"] == "oops"
        assert payload["ts_ms"] == 123
        assert payload["listening"] is False

    def test_to_state_payload_empty_voice_id(self):
        # До первого set_voice → active_voice_id=None → "" в payload.
        s = SessionVoiceState(session_id="abc")
        payload = s.to_state_payload(ts_ms=0)
        assert payload["active_voice_id"] == ""


class TestRateLimiter:
    def test_first_call_allowed(self):
        s = SessionVoiceState(session_id="abc")
        assert s.check_preview_quota(now_monotonic=100.0) is True

    def test_within_limit_allowed(self):
        s = SessionVoiceState(session_id="abc")
        # PREVIEW_RATE_LIMIT=3 запросов подряд.
        assert s.check_preview_quota(now_monotonic=100.0) is True
        assert s.check_preview_quota(now_monotonic=101.0) is True
        assert s.check_preview_quota(now_monotonic=102.0) is True

    def test_fourth_call_denied(self):
        s = SessionVoiceState(session_id="abc")
        s.check_preview_quota(now_monotonic=100.0)
        s.check_preview_quota(now_monotonic=101.0)
        s.check_preview_quota(now_monotonic=102.0)
        assert s.check_preview_quota(now_monotonic=103.0) is False

    def test_window_resets(self):
        s = SessionVoiceState(session_id="abc")
        # 3 запроса в t=100..102.
        for t in (100.0, 101.0, 102.0):
            assert s.check_preview_quota(now_monotonic=t) is True
        # На t=110 (через 10 сек после первого) — старые timestamps
        # выпадают из окна, новый пропускаем.
        assert s.check_preview_quota(now_monotonic=112.0) is True

    def test_window_partial_reset(self):
        s = SessionVoiceState(session_id="abc")
        # t=100, 105, 109 — все 3 внутри [now-10, now] окна.
        s.check_preview_quota(now_monotonic=100.0)
        s.check_preview_quota(now_monotonic=105.0)
        s.check_preview_quota(now_monotonic=109.0)
        # На t=110 окно [100, 110] — все 3 ещё внутри, denied.
        assert s.check_preview_quota(now_monotonic=110.0) is False
        # На t=110.5 окно [100.5, 110.5] — 100 выпало, осталось 105 и 109 (2).
        # 2 < 3 → разрешаем и добавляем 110.5.
        assert s.check_preview_quota(now_monotonic=110.5) is True
        # Сразу ещё один — 3 внутри [100.5, 110.5] → denied.
        assert s.check_preview_quota(now_monotonic=110.5) is False

    def test_limit_constants(self):
        # Pin the values — задача t_7eba64d9: "3/10 sec".
        assert PREVIEW_RATE_LIMIT == 3
        assert PREVIEW_WINDOW_S == 10.0


class TestVoiceStateRegistry:
    def test_get_or_create(self):
        r = VoiceStateRegistry()
        s = r.get_or_create("s1")
        assert s.session_id == "s1"
        # Повторный get — тот же объект.
        assert r.get_or_create("s1") is s

    def test_remove(self):
        r = VoiceStateRegistry()
        r.get_or_create("s1")
        assert "s1" in r
        r.remove("s1")
        assert "s1" not in r

    def test_remove_unknown_is_noop(self):
        r = VoiceStateRegistry()
        r.remove("ghost")  # не должен raise

    def test_len(self):
        r = VoiceStateRegistry()
        assert len(r) == 0
        r.get_or_create("a")
        r.get_or_create("b")
        assert len(r) == 2