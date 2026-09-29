"""Issue #3169 — «Робот ты остановил музыку» не должно исполняться как стоп.

Живой прогон 29.09.2026 01:34-01:36 UTC (деплой develop ``d68021e34``):
вопрос в прошедшем времени («ты остановил музыку», без пунктуации — Yandex
STT не ставит «?») роутер медиакоманд (#3134, ``media_command_grammar``)
и параллельно ``command_node`` (через ``core.command_parser``) исполнили
как приказ «стоп»: робот сказал «Останавливаюсь» (command_node,
``handle_stop`` — навигационный стоп, независимая подписка на
``/voice/stt/result``, НЕ гейт stt_admission) и следом «Сейчас ничего не
играет — на всякий случай всё остановил.» (media_router, т.к. музыка и
так стояла).

Причина в обоих местах — глагольный стемминг ловил прошедшее время:

* ``command_parser.py`` (``IntentType.STOP``) — ``останови`` без границы
  слова матчился и ВНУТРИ ``остановил`` (иск. #2971 не касался этого пути
  — та фраза шла через ``media_command_grammar``, не ``CommandParser``).
* ``media_command_grammar.py`` (``_MUSIC_STOP_VERBS``) — ``останов\\w*`` /
  ``выключ\\w*`` и т.п. считали любое окончание, включая ``-ил``.

Фикс — оба паттерна ПРАВЯТ существующие группы (без новых
``re.compile``, мораторий #3132): только повелительная форма проходит
как приказ; прошедшее время должно уйти в LLM как обычная реплика/вопрос.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.command_parser import CommandParser, IntentType
from rob_box_voice.core.media_command_grammar import (
    MediaIntent,
    is_music_stop_command,
    parse_media_command,
)

# ---------------------------------------------------------------------------
# command_parser.py — та же нода, что стоит за command_node.py (issue #3169:
# command_node подписан на /voice/stt/result НАПРЯМУЮ, минуя stt_admission
# и media_command_grammar, — «Останавливаюсь» пришло именно отсюда).
# ---------------------------------------------------------------------------


class TestCommandParserPastTenseNotStop:
    """«остановил» (прошедшее время) — не приказ «стоп»."""

    @pytest.mark.parametrize(
        "text",
        [
            "ты остановил музыку",
            "робот ты остановил музыку",
            "ты остановил",
        ],
    )
    def test_past_tense_is_not_stop(self, text: str) -> None:
        parser = CommandParser()
        command = parser.parse(text)
        assert command.intent is not IntentType.STOP, (
            f"CommandParser.parse({text!r}) не должен классифицировать "
            "прошедшее время как STOP (issue #3169)"
        )

    @pytest.mark.parametrize(
        "text",
        [
            "остановись",
            "останови",
            "останови музыку",
            "стоп",
            "стой",
            "halt",
            "хватит",
            "замри",
        ],
    )
    def test_imperative_forms_still_stop(self, text: str) -> None:
        """Повелительная форма (приказ) — регрессия быть не должна."""
        parser = CommandParser()
        command = parser.parse(text)
        assert command.intent is IntentType.STOP, (
            f"CommandParser.parse({text!r}) должен остаться STOP "
            "(повелительная форма) — issue #3169 не трогает этот путь"
        )


# ---------------------------------------------------------------------------
# media_command_grammar.py — роутер медиакоманд до LLM (#3134).
# ---------------------------------------------------------------------------


class TestMediaGrammarPastTenseNotStop:
    """Грамматика роутера: прошедшее время не закрывает реплику как STOP."""

    @pytest.mark.parametrize(
        "text",
        [
            "ты остановил музыку",
            "робот ты остановил музыку",
            "ты выключил музыку",
        ],
    )
    def test_past_tense_falls_through_to_llm(self, text: str) -> None:
        cmd = parse_media_command(text)
        assert cmd.intent is MediaIntent.NONE, (
            f"parse_media_command({text!r}).intent должен быть NONE "
            "(вопрос в прошедшем времени уходит в LLM) — issue #3169, "
            f"получено {cmd.intent!r}"
        )

    @pytest.mark.parametrize(
        "text",
        [
            "ты остановил музыку",
            "робот ты остановил музыку",
            "ты выключил музыку",
        ],
    )
    def test_past_tense_is_not_music_stop_command(self, text: str) -> None:
        """Широкий детектор (силенс-/command-гейт) тоже не должен ловить
        прошедшее время — иначе фраза всё ещё обходит command-гейт молча
        (даже если роутер её не закрывает)."""
        assert is_music_stop_command(text) is False, (
            f"is_music_stop_command({text!r}) должен быть False — "
            "issue #3169 (прошедшее время — не приказ)"
        )

    @pytest.mark.parametrize(
        ("text", "intent"),
        [
            ("выключи музыку", MediaIntent.STOP),
            ("останови музыку", MediaIntent.STOP),
            ("останови трек пожалуйста", MediaIntent.STOP),
            ("стоп музыку", MediaIntent.STOP),
            ("убери музыку", MediaIntent.STOP),
            ("хватит диджеить", MediaIntent.STOP),
            ("стоп диджей", MediaIntent.STOP),
        ],
    )
    def test_imperative_forms_still_close_stop(
        self, text: str, intent: MediaIntent
    ) -> None:
        """Регрессия: повелительные стоп-фразы (#2834/#2971/#3134) не
        должны сломаться правкой стеммингового паттерна."""
        cmd = parse_media_command(text)
        assert cmd.intent is intent
        assert cmd.closed is True

    @pytest.mark.parametrize(
        "text",
        [
            "выключи музыку",
            "останови музыку",
            "стоп музыку",
            "убери музыку",
            "хватит диджеить",
            "стоп диджей блядь",
            "выключи музыку и включи Oakenfold",
        ],
    )
    def test_imperative_forms_still_music_stop_command(self, text: str) -> None:
        assert is_music_stop_command(text) is True
