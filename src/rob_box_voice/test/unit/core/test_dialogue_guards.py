"""Unit tests for :mod:`rob_box_voice.core.dialogue_guards` (TD-1 extraction).

Covers the Issue #992 guard heuristics that were extracted from
``dialogue_node.py``:

* music stop-command classification,
* service-syntax detector for TTS (``<system>`` in SSML).

ADR-0150 PR-7 ч.7d-4 (#3698): action-claim detectors (#992 Bug E, #2549, #2559) and
their retry prompts / fallbacks were removed together with their tests.

ADR-0150 PR-7 ч.7d-1 (#3698): babble (Bug D), planning-narration (#1882) and
tool-skipped (#1777) guards were removed together with their tests.

ADR-0149 PR-13a: music-guard detectors (``user_wants_music``,
``is_vocal_request``, ``is_music_state_query``, music ``ActionClaimRule``-s,
Renardo-code / unknown-melody detectors, music retry prompts) were removed
together with their tests.

These are pure-Python functions — no ROS2 node required.
"""

from __future__ import annotations

import pytest
from typing import Optional

from rob_box_voice.core.dialogue_guards import (
    MUSIC_STOP_OVERRIDES,
    is_music_stop_command,
    is_system_template_regurgitated_in_ssml,
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


def test_stop_music_is_a_stop_tool() -> None:
    from rob_box_voice.core.dialogue_guards import MUSIC_STOP_TOOLS

    assert "stop_music" in MUSIC_STOP_TOOLS


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


class TestSystemTemplateRegurgitateInSsml:
    """Defence-in-depth для tts_node — отдельный detector для SSML.

    ``is_system_template_regurgitated_in_ssml`` ловит regurgitates
    на СЫРОМ SSML (до strip'а тегов в ``_extract_text_from_ssml``).
    Более узкий, чем для extracted spoken: только полный ``<speak>``
    обрамлённый блок ИЛИ plain ``<system>...</system>``.
    """

    @pytest.mark.parametrize(
        "ssml",
        [
            # SSML-обёртка с regurgitates внутри
            "<speak><system>foo</system></speak>",
            "<speak><system>\n[получатель ответа забыл указать антропоморфные "
            "атрибуты]\n</system></speak>",
            # Без SSML-обёртки (edge-case)
            "<system>x</system>",
            # Whitespace внутри тегов (после ``<speak>`` / перед ``</speak>``)
            "<speak>\n  <system>foo</system>  \n</speak>",
        ],
    )
    def test_ssml_regurgitate_is_detected(self, ssml: str) -> None:
        assert is_system_template_regurgitated_in_ssml(ssml) is True

    @pytest.mark.parametrize(
        "ssml",
        [
            # Нормальная фраза с серединным тегом — НЕ regurgitates
            "<speak>Согласно <system>инструкции</system>, отвечу.</speak>",
            # Обычная русская речь
            "<speak>Привет! Как дела?</speak>",
            # Пустая строка / None
            "",
            None,  # type: ignore[list-item]
            # Неполные теги
            "<speak><system>foo</speak>",
            "<speak>foo</system></speak>",
        ],
    )
    def test_normal_ssml_passes(self, ssml: Optional[str]) -> None:
        """Серединные ссылки, нормальная речь, неполные теги."""
        assert is_system_template_regurgitated_in_ssml(ssml) is False
