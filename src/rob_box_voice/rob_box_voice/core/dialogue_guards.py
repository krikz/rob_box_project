"""dialogue_guards.py — что осталось от гардов текста ответа LLM в v1-ноде (ADR-0150 PR-7, #3698).

Pure Python: no rclpy, no I/O. Только:

* **Music tool names** — :data:`MUSIC_STARTING_TOOLS` & Co. from the tool catalog.
* **Нормализация текста для TTS** — служебный синтаксис вместо речи (#2175 ``<system>``, #2760 разметка вызова
  тула): его не озвучивают ``tts_node`` и ``turn_speech.final_text_for_speech``. Переезжает в tts_node в PR-15.

Гардов, которые по тексту ответа решают «повторить вызов LLM» или «подменить фразу», здесь больше нет: babble,
планирование, tool-skipped (ч.7d-1), ретраи на разметку и ``<system>`` (ч.7d-2), «заявил действие без тула» —
#992 Bug E ``ACTION_CLAIM_RULES``, #2549, #2559, fallback #2780/#2949 (ч.7d-4). В режиме команд речь при действиях
не звучит, а фразу об исходе строит код по событию владельца (ADR-0150 §4.1, §5.2–5.4).
"""

from __future__ import annotations

import re
from typing import Optional

from rob_box_core.music_tools import MUSIC_STOP_TOOLS as _MUSIC_STOP_TOOLS
from rob_box_core.tool_catalog import TOOL_CATALOG

from .media_command_grammar import (  # noqa: F401 — реэкспорт, issue #3134
    MUSIC_STOP_COMMAND_RE,
    MUSIC_STOP_OVERRIDES,
    is_music_stop_command,
)


# ---------------------------------------------------------------------------
# Music tool names — single source of truth
# ---------------------------------------------------------------------------

#: Инструменты, запускающие слышимую музыку (Renardo или mp3).
#:
#: Единственный источник — capability-флаг ``starts_music`` на ``MCPTool``,
#: проецируемый в ``rob_box_core.tool_catalog``. Раньше это знание жило в
#: двух рукописных frozenset'ах (``RENARDO_MUSIC_TOOLS`` /
#: ``GENERATED_MUSIC_TOOLS``), и каждый новый музыкальный тул про него
#: забывал: ``compose_music`` вышел и был авто-заглушен через 1.5 с,
#: ``music_library`` — в третий раз, а ``lookup_melody`` играл
#: мелодию, но гуард его не видел и гнал LLM в повторный
#: запуск музыки (старый путь стирал мелодию через Clock.clear()).
MUSIC_STARTING_TOOLS: frozenset = frozenset(
    entry.name for entry in TOOL_CATALOG if entry.starts_music
)

#: Stop tools (``stop_music``). The TRACK-mode cleanup-policy flag they used to
#: clear in dialogue_node was removed with the music cleanup (ADR-0150 PR-7c ч.4б).
#:
#: Issue #3133 (ADR-0141): ``set_dj_mode`` removed. It sat here AND in
#: the starters set at once; the starters branch runs after the stop
#: branch and re-armed the flag anyway, so its presence here never cleared
#: anything — it only made the set lie about what «stop» means. Whether
#: music is playing is no longer derived from tool names at all: dialogue
#: reads the player snapshot on ``/voice/music/state``.
#:
MUSIC_STOP_TOOLS: frozenset = _MUSIC_STOP_TOOLS


# ---------------------------------------------------------------------------
# Issue #2175 — MiniMax-M3 regurgitates ``<system>...</system>`` template
# instead of producing a user-facing reply.
#
# Live 08.09 (Vision Pi, 14:52): три запроса подряд после ``set_voice``
# + multi-voice user_input + новая DJ-skill context дали в ``spoken``
# ровно строку вида::
#
#     <system>
#     [получатель ответа забыл указать антропоморфные атрибуты]
#     </system>
#
# Это кусок СИСТЕМНОГО шаблона из master_prompt_compact.txt (см.
# ``RULE #RESPONSE_FORMAT``), который модель regurgitates буквально.
# TTS озвучивал эту директиву через Yandex→MiniMax fallback, юзер слышал
# «получатель ответа забыл указать антропоморфные атрибуты» поверх
# только что сменённого голоса.
#
# Защита — отказ озвучки в tts_node (:func:`is_system_template_regurgitated_in_ssml`): в SSML
# ``<system>...</system>`` может быть обрамлён ``<speak>...</speak>``. ADR-0150 PR-7 ч.7d-2 (#3698): ретрай
# LLM в dialogue_node (``build_system_regurgitate_retry_prompt``) и детектор на извлечённом тексте удалены.
# ---------------------------------------------------------------------------
# Regex для SSML (defense-in-depth) — парный ``<system>...</system>``
# внутри ``<speak>...</speak>``. Также ловит ``<system>...</system>``
# как единственный верхнеуровневый блок (без ``<speak>``-обёртки).
# search() — потому что блок внутри SSML.
SYSTEM_TEMPLATE_REGURGITATE_SSML_RE = re.compile(
    r"^<speak>\s*<system>.*?</system>\s*</speak>$"
    r"|^<system>.*?</system>$",
    re.DOTALL | re.IGNORECASE,
)


def is_system_template_regurgitated_in_ssml(ssml: Optional[str]) -> bool:
    """Issue #2175 — defense-in-depth для tts_node.

    Работает на СЫРОМ SSML (входе в ``dialogue_callback``). Ловит
    regurgitates в двух форматах:

    1. ``<speak><system>...</system></speak>`` — типичный случай
       после ``_extract_text_from_ssml`` (когда strip ещё не прошёл).
    2. ``<system>...</system>`` без SSML-обёртки — на случай если
       producer забыл обернуть в ``<speak>``.

    Возвращает ``False`` для серединных ссылок в обычной фразе
    («<speak>Согласно <system>инструкции</system>, отвечу.</speak>»)
    — match() ограничивает всю строку.
    """
    if not ssml:
        return False
    return bool(SYSTEM_TEMPLATE_REGURGITATE_SSML_RE.match(ssml))


# ---------------------------------------------------------------------------
# Issue #2760 — модель ПЕЧАТАЕТ вызов тула вместо того, чтобы его сделать.
#
# Live Vision Pi, прогон 35704637846 (акт 2, шаг n204_boris_intro_long)::
#
#   spoken='<function_calls>\n<invoke name="register_speaker">\n
#           <parameter name="name">Борис</parameter>\n</invoke>\n
#           <invoke name="memory_save">…</invoke>\n</function_calls>'
#   tools=[] finish_reason='stop'
#
# Дальше без задержки: ``📤 LLM OUTPUT`` → два TTS-чанка → робот читает
# разметку вслух, а текст оседает в истории как реплика ассистента
# (следующий ход учится на этом примере).
#
# Отличие от #2175: там модель возвращает кусок СИСТЕМНОГО промпта, здесь —
# синтаксис ВЫЗОВА инструмента. Отличие от #2549/#2559: там модель
# ОПИСЫВАЕТ действие прозой («сделала», «перезапущу»), и детектор ищет
# глаголы; здесь глаголов нет вообще — есть теги.
#
# Разбор причины (модель MiniMax-M3 отправляет решение в канал content
# при исправно переданных тулах), восстановление намерения и сам регекс —
# в :mod:`rob_box_harness.core.tool_loop.markup_recovery` и
# ``text_classify``. Здесь — последний рубеж для tts_node: не дать тегам прозвучать (ретрай LLM в
# dialogue_node удалён в ADR-0150 PR-7 ч.7d-2, #3698).
# ---------------------------------------------------------------------------
# ⚠️ Регекс намеренно ДУБЛИРУЕТСЯ с
# ``rob_box_harness.core.tool_loop.text_classify.TOOL_CALL_MARKUP_RE``.
# Свести в один модуль нельзя: harness — нижний слой, он не имеет права
# импортировать voice, а этот модуль обязан оставаться pure-Python без
# зависимостей (см. шапку файла). Слои решают РАЗНЫЕ задачи: там —
# восстановить намерение внутри цикла тулов, здесь — не дать тегам
# прозвучать. Правку одного регекса переносить во второй; эквивалентность
# закреплена тестом ``test_issue_2760_tool_call_markup.py``.
TOOL_CALL_MARKUP_RE = re.compile(
    # Родной диалект MiniMax (официальный tool_calling_guide.md):
    # <minimax:tool_call><invoke name="..."><parameter name="...">.
    r"</?minimax:tool_call\s*>"
    # Диалект, который робот получил живьём 22.09 (Anthropic-style).
    r"|</?function_?calls\s*>"
    r"|</?tool_?calls?\s*>"
    r"|<\s*/?\s*(?:minimax:|antml:)?invoke(?:\s+name\s*=|\s*>)"
    r"|<\s*/?\s*(?:minimax:|antml:)?parameter(?:\s+name\s*=|\s*>)"
    r"|<\s*antml:",
    re.IGNORECASE,
)


def is_tool_call_markup(spoken_text: Optional[str]) -> bool:
    """Issue #2760 — в ``spoken`` лежит разметка вызова тула, а не речь.

    ``True``, если текст содержит хоть один маркер из
    :data:`TOOL_CALL_MARKUP_RE`. Пустая строка / ``None`` → ``False``.
    """
    if not spoken_text:
        return False
    return bool(TOOL_CALL_MARKUP_RE.search(spoken_text))
