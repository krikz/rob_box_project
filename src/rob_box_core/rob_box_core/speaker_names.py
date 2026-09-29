"""Единый набор «мусорных» имён спикера.

architecture audit 2026-09-29, ADR-0145: раньше набор дублировался в трёх
местах и разошёлся:

* ``rob_box_voice.utils.speaker_embeddings._INVALID_SPEAKER_NAMES``
* ``rob_box_voice.core.dialogue_helpers.INVALID_SPEAKER_NAMES``
* ``rob_box_mcp_tools.tools.dialogue.RegisterSpeakerTool._NOISE_NAMES``

Здесь — объединение всех трёх. Сравнение выполняют вызывающие: точное
совпадение ``name.strip().lower()`` (НЕ substring/prefix, поэтому реальные
имена вроде «Юзеф» или «Гостомысл» фильтр не задевает).
"""

from __future__ import annotations

INVALID_SPEAKER_NAMES: frozenset[str] = frozenset(
    {
        # — trigger-слова из фразы «меня зовут X» (issue #1101) —
        "зовут",
        "имя",
        "меня",
        "зовут-это",
        "зовут меня",
        "это",
        "называю",
        "зовут-меня",
        "моё",
        "мое",
        "моё имя",
        "мое имя",
        "имя мне",
        "имя моё",
        "имя мое",
        # — junk-значения из resemblyzer / битых JSON-полей (#1077/#1101) —
        "null",
        "none",
        "undefined",
        "unknown",
        "",
        # — заглушечные/служебные имена-плейсхолдеры (issue #2932) —
        "неизвестный",
        "неизвестная",
        "неизвестно",
        "незнакомец",
        "незнакомка",
        "гость",
        "user",
        "пользователь",
        "speaker",
        "name",
        "-",
        "?",
    }
)
