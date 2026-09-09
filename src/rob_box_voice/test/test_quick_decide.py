"""Unit tests for S4.1 :mod:`rob_box_voice.scheduler.quick_decide`.

Ревизия v5 (SCHEDULER_DESIGN.md §4.7, §554-604): **никакой второй LLM**.
Правила ловят только явный мусор (междометия, < 2 слов, повторы,
низкий ASR-confidence) и явные императивы («хватит», «стоп»,
«замолчи», «отставить») — кроме music-stop, который уже разруливается
``_MUSIC_STOP_OVERRIDES`` (dialogue_node.py:1778-1784). Всё остальное,
включая любые союзы/местоимения («и ещё про X», «а потом Y», «про
него»), уходит в основной LLM-цикл как ``PENDING_LLM`` — это
контекстно-зависимая классификация, правила её НИКОГДА не решают.
"""

from __future__ import annotations

import time

import pytest

from rob_box_voice.scheduler.quick_decide import QuickVerdict, quick_decide


# ---------------------------------------------------------------------------
# IGNORE — явный мусор
# ---------------------------------------------------------------------------


class TestIgnore:
    @pytest.mark.parametrize("text", ["угу", "ага", "мхм", "Угу.", "ага!"])
    def test_interjections_are_ignored(self, text):
        assert quick_decide(text, source="stt", active_group=None, clock=time.monotonic) is QuickVerdict.IGNORE

    @pytest.mark.parametrize("text", ["да", "ну", "хм"])
    def test_short_fillers_are_ignored(self, text):
        assert quick_decide(text, source="stt", active_group=None, clock=time.monotonic) is QuickVerdict.IGNORE

    def test_fewer_than_two_words_is_ignored(self):
        assert quick_decide("погоди", source="stt", active_group=None, clock=time.monotonic) is QuickVerdict.IGNORE

    def test_empty_text_is_ignored(self):
        assert quick_decide("   ", source="stt", active_group=None, clock=time.monotonic) is QuickVerdict.IGNORE

    def test_low_confidence_is_ignored(self):
        verdict = quick_decide(
            "расскажи мне что-нибудь интересное",
            source="stt", active_group=None, clock=time.monotonic,
            confidence=0.2,
        )
        assert verdict is QuickVerdict.IGNORE

    def test_confidence_at_or_above_threshold_is_not_forced_ignore(self):
        verdict = quick_decide(
            "расскажи мне что-нибудь интересное",
            source="stt", active_group=None, clock=time.monotonic,
            confidence=0.4,
        )
        assert verdict is not QuickVerdict.IGNORE

    def test_exact_repeat_within_window_is_ignored(self):
        t0 = 100.0
        verdict = quick_decide(
            "расскажи анекдот",
            source="stt", active_group=None, clock=lambda: t0 + 0.3,
            previous_text="расскажи анекдот", previous_ts=t0,
        )
        assert verdict is QuickVerdict.IGNORE

    def test_repeat_case_insensitive_and_whitespace_tolerant(self):
        t0 = 100.0
        verdict = quick_decide(
            "  Расскажи Анекдот  ",
            source="stt", active_group=None, clock=lambda: t0 + 0.1,
            previous_text="расскажи анекдот", previous_ts=t0,
        )
        assert verdict is QuickVerdict.IGNORE

    def test_repeat_outside_window_is_not_forced_ignore(self):
        t0 = 100.0
        verdict = quick_decide(
            "расскажи анекдот",
            source="stt", active_group=None, clock=lambda: t0 + 5.0,
            previous_text="расскажи анекдот", previous_ts=t0,
        )
        assert verdict is not QuickVerdict.IGNORE

    def test_different_text_is_not_treated_as_repeat(self):
        t0 = 100.0
        verdict = quick_decide(
            "расскажи сказку",
            source="stt", active_group=None, clock=lambda: t0 + 0.1,
            previous_text="расскажи анекдот", previous_ts=t0,
        )
        assert verdict is not QuickVerdict.IGNORE


# ---------------------------------------------------------------------------
# REPLACE — явные императивы (кроме music-stop)
# ---------------------------------------------------------------------------


class TestReplace:
    @pytest.mark.parametrize("text", ["хватит", "стоп", "замолчи", "отставить"])
    def test_explicit_imperatives_are_replace(self, text):
        assert quick_decide(text, source="stt", active_group="g1", clock=time.monotonic) is QuickVerdict.REPLACE

    def test_imperative_with_extra_words_is_still_replace(self):
        verdict = quick_decide("так, хватит уже", source="stt", active_group="g1", clock=time.monotonic)
        assert verdict is QuickVerdict.REPLACE

    @pytest.mark.parametrize("text", [
        "стоп музыку", "выключи музыку", "останови музыку", "хватит диджеить",
    ])
    def test_music_stop_overrides_are_not_replace(self, text):
        """§0.2 — music-stop phrases must reach the LLM (which calls
        stop_music), not get swallowed as a REPLACE verdict here."""
        verdict = quick_decide(text, source="stt", active_group="g1", clock=time.monotonic)
        assert verdict is not QuickVerdict.REPLACE


# ---------------------------------------------------------------------------
# PENDING_LLM — всё остальное, включая союзы/местоимения (НИКОГДА правила)
# ---------------------------------------------------------------------------


class TestPendingLlm:
    @pytest.mark.parametrize("text", [
        "и ещё про енота",
        "а потом спой колыбельную",
        "расскажи про него",
        "добавь енота в куплет",
        "и ещё один анекдот про енота",
        "а можно ещё раз",
        "тоже самое но про кота",
    ])
    def test_rules_never_decide_conjunctions_or_pronouns(self, text):
        """Ревизия v5, §4.7.2: 'Никаких regex на союзы/«и ещё»' — это
        контекстно-зависимо, решает только основная LLM."""
        verdict = quick_decide(text, source="stt", active_group="g1", clock=time.monotonic)
        assert verdict is QuickVerdict.PENDING_LLM

    def test_ordinary_new_request_is_pending_llm(self):
        verdict = quick_decide("спой песню про комара", source="stt", active_group=None, clock=time.monotonic)
        assert verdict is QuickVerdict.PENDING_LLM


# ---------------------------------------------------------------------------
# Speed — правила решают за < 50мс
# ---------------------------------------------------------------------------


class TestSpeed:
    def test_decision_under_50ms(self):
        t0 = time.perf_counter()
        quick_decide("и ещё про енота", source="stt", active_group="g1", clock=time.monotonic)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        assert elapsed_ms < 50, f"quick_decide took {elapsed_ms:.2f}ms"
