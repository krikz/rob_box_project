"""Unit tests for the music stop-command grammar (:mod:`rob_box_voice.core.media_command_grammar`).

Раньше жили в ``test_dialogue_guards.py``: ``dialogue_guards`` только реэкспортировал ``is_music_stop_command`` и
``MUSIC_STOP_OVERRIDES`` из грамматики медиакоманд. PR-15 часть 15-core (#3711) удалила модуль-реэкспорт; тесты
остались при настоящем владельце. Детектор отказа TTS на куске системного шаблона (#2175) — в
``test_tts_text_clean.py``, разметка вызова тула (#2760) — ``rob_box_core/test/test_tool_call_markup.py``.

These are pure-Python functions — no ROS2 node required.
"""

from __future__ import annotations

from rob_box_voice.core.media_command_grammar import (
    MUSIC_STOP_OVERRIDES,
    is_music_stop_command,
)


# ---------------------------------------------------------------------------
# is_music_stop_command
# ---------------------------------------------------------------------------


class TestIsMusicStopCommand:
    def test_stop_phrases(self) -> None:
        assert is_music_stop_command("хватит диджеить") is True
        assert is_music_stop_command("выключи музыку") is True
        assert is_music_stop_command("стоп музыку") is True
        assert is_music_stop_command("убери музыку") is True

    def test_case_insensitive(self) -> None:
        assert is_music_stop_command("ВЫКЛЮЧИ МУЗЫКУ") is True

    def test_start_phrases_are_not_stop(self) -> None:
        assert is_music_stop_command("включи музыку") is False
        assert is_music_stop_command("спой песню") is False

    def test_empty(self) -> None:
        assert is_music_stop_command("") is False
        assert is_music_stop_command(None) is False  # type: ignore[arg-type]

    def test_stop_verb_plus_dj_still_matches(self) -> None:
        """Issue #2971 регресс: «диджея»/«диджеить»/«диджей режим» больше
        не в ``MUSIC_STOP_OVERRIDES`` как голые подстроки, но со стоп-
        глаголом рядом их по-прежнему ловит ``MUSIC_STOP_COMMAND_RE``."""
        assert is_music_stop_command("хватит диджея") is True
        assert is_music_stop_command("выключи диджея") is True
        assert is_music_stop_command("хватит диджеить") is True
        assert is_music_stop_command("выключи диджей режим") is True
        assert is_music_stop_command("стоп диджей") is True

    def test_bare_dj_noun_without_stop_verb_is_not_stop(self) -> None:
        """Issue #2971 — живой инцидент 24.09.2026: юзер продиктовал
        роботу системный промпт «Ты диджей PAUL OAKENFOLD …», который
        заканчивался словами «…системный промт для робота-диджея». Ни
        «для диджея», ни «робота-диджея», ни «у диджея» не содержат
        стоп-глагол — это НЕ стоп-команда, а хвост обычной реплики."""
        assert is_music_stop_command("это промпт для диджея") is False
        assert (
            is_music_stop_command("системный промт для робота-диджея")
            is False
        )
        assert is_music_stop_command("вопрос у диджея") is False
        assert is_music_stop_command("поставь диджея") is False
        assert is_music_stop_command("включи диджей режим") is False

    def test_live_incident_long_dj_prompt_is_not_stop(self) -> None:
        """Issue #2971 — точный хвост промпта из живого лога 24.09.2026
        11:10 UTC (сокращённая версия «промпт 143 строки» из issue)."""
        prompt = (
            "Ты диджей PAUL OAKENFOLD и у нас сегодня вечеринка. "
            "Скопируй весь блок выше и вставь как системный промт "
            "для робота-диджея."
        )
        assert is_music_stop_command(prompt) is False


# ---------------------------------------------------------------------------
# Smoke — keyword tuples must not silently shrink
# ---------------------------------------------------------------------------


def test_keyword_tuples_non_empty() -> None:
    assert MUSIC_STOP_OVERRIDES


def test_stop_overrides_are_caught_by_stop_detector() -> None:
    """Every fixed stop phrase is caught by ``is_music_stop_command``."""
    for stop in MUSIC_STOP_OVERRIDES:
        assert is_music_stop_command(stop) is True


class TestWatchdogStopReachesTheDialogueLive3108:
    """Музыку останавливает не только диалог — он обязан это узнавать (живой лог 31.08: через 139 с после
    остановки по watchdog диалог считал, что трек играет). Источник «играет» — снимок плеера
    ``/voice/music/state`` (#3133, ADR-0141); флаг TRACK-режима удалён в ADR-0150 PR-7c ч.4б (#3700).
    PR-15 (#3711, часть 15p): нода диалога — ``dialog_node`` (v2), нода v1 удалена.
    """

    def _dialog_node_source(self) -> str:
        from pathlib import Path

        here = Path(__file__).resolve()
        for parent in here.parents:
            candidate = parent / "rob_box_voice" / "dialog_node.py"
            if candidate.is_file():
                return candidate.read_text(encoding="utf-8")
        raise AssertionError("dialog_node.py not found")

    def test_node_subscribes_to_the_server_music_state(self) -> None:
        # Issue #3133: имя топика — константа контракта (ADR-0141).
        from rob_box_voice.core.music_player_state import MUSIC_STATE_TOPIC

        src = self._dialog_node_source()
        assert MUSIC_STATE_TOPIC == "/voice/music/state"
        assert "MUSIC_STATE_TOPIC, self._on_music_state" in src, (
            "диалог не слушает /voice/music/state — про остановку по "
            "watchdog он не узнает"
        )
