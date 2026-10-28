"""tts_text_clean - очистка текста и отбраковка служебного ввода перед синтезом (ADR-0150 PR-15, #3711, часть 15-core).

Один модуль знания «что в тексте для TTS не озвучивается» рядом с ``tts_node`` - единственным потребителем. Раньше это
жило в ``core/speak_helpers.py`` и ``core/dialogue_guards.py`` вперемешку с чисткой ответа LLM старой ноды диалога
(метки ``[Spkr:…]``, маркер ``done``, ``<think>``, чанкование, SSML-обёртка), которой v2 не пользуется: фразу и её
разбиение на куски строит код (``rob_box_dialog``, ``speech_client``), а модель маркеров не пишет.

Что здесь:

* :func:`strip_markdown` - звёздочки и решётки не читаются как «звёздочка» (#988);
* :func:`strip_bracket_asides` - вставки ``[…]`` не звучат: модель иногда пишет свои рассуждения в ``content``
  (``[… §2 ANTI-DUP …]``, живой прогон 06.10, PR #3474);
* :func:`is_system_template_regurgitated_in_ssml` - ответ целиком из ``<system>…</system>`` (кусок системного шаблона,
  который модель выдала буквально, #2175) - не речь, ``tts_node`` отказывает.

Разметку вызова тула текстом (#2760) ``tts_node`` отбраковывает через ``rob_box_core.tool_call_markup`` - там одна
копия на ``tts_node`` и harness. Письменность текста проверяет :mod:`.tts_text_guard`, язык - ``tts_voice_registry``.

Чистый Python, без ROS.
"""

from __future__ import annotations

import re
from typing import List, Optional

#: Правила :func:`strip_markdown` по порядку: ``(скомпилированный шаблон, замена)``. Выделение требует ПАРНЫХ
#: разделителей, поэтому одинокие ``*`` и ``_`` (``2 * 3``, ``под_черкивание``) не трогаются.
_MARKDOWN_STRIP_RULES: List[tuple] = [
    # Inline code spans: `code` → code
    (re.compile(r"`([^`]*)`"), r"\1"),
    # Bold / italic: **text** → text, *text* → text
    (re.compile(r"\*\*([^*]+)\*\*"), r"\1"),
    (re.compile(r"\*([^*]+)\*"), r"\1"),
    # __bold__ / _italic_
    (re.compile(r"__([^_]+)__"), r"\1"),
    (re.compile(r"_([^_]+)_"), r"\1"),
    # Strikethrough: ~~text~~ → text
    (re.compile(r"~~([^~]+)~~"), r"\1"),
    # Headings at line start: # text / ## text → text
    (re.compile(r"(?m)^[#]{1,6}\s*"), ""),
    # Blockquote at line start: > text → text
    (re.compile(r"(?m)^\s*>\s?"), ""),
    # List markers at line start: - text, * text, + text, 1. text → text
    (re.compile(r"(?m)^\s*[-*+]\s+"), ""),
    (re.compile(r"(?m)^\s*\d+\.\s+"), ""),
    # Links: [text](url) → text
    (re.compile(r"\[([^\]]+)\]\([^)]*\)"), r"\1"),
]

#: Вставка ``[...]`` в любом месте реплики. ``](`` исключено: ссылки Markdown ``[текст](url)`` доживают до
#: :func:`strip_markdown`, который оставляет видимый текст.
_BRACKET_ASIDE_RE = re.compile(r"\[[^\[\]]*\](?!\()")
#: Незакрытая ``[`` - рассуждение оборвано ``max_tokens``: хвост выбрасывается.
_BRACKET_TAIL_RE = re.compile(r"\[[^\]]*$")
_SPACE_RUN_RE = re.compile(r"[ \t]{2,}")

#: Issue #2175 - MiniMax-M3 выдаёт ``<system>…</system>`` из системного шаблона вместо ответа. Парный блок внутри
#: ``<speak>…</speak>`` или единственный верхнеуровневый блок без обёртки; ``match`` ограничивает всю строку, поэтому
#: ссылка на тег посреди обычной фразы не срабатывает.
_SYSTEM_TEMPLATE_RE = re.compile(
    r"^<speak>\s*<system>.*?</system>\s*</speak>$|^<system>.*?</system>$",
    re.DOTALL | re.IGNORECASE,
)


def strip_markdown(text: str) -> str:
    """Снять разметку Markdown перед синтезом (issue #988).

    LLM часто оборачивает стихи и рэп в Markdown (``*Жил да был енот весёлый,*``), а TTS читает ``*`` как «звёздочка».
    Здесь снимаются МАРКЕРЫ, слова остаются. Правила консервативные: выделение (``**`` / ``*`` / ``__`` / ``_``) - только
    парами; ``#``, ``>`` и маркеры списков - только в начале строки; код (`` ` ``) и ссылки ``[текст](url)`` оставляют
    видимый текст. Не-строка возвращается как есть.
    """
    if not isinstance(text, str):
        return text
    for pattern, repl in _MARKDOWN_STRIP_RULES:
        text = pattern.sub(repl, text)
    return text.strip()


def strip_bracket_asides(text: str) -> str:
    """Убрать из ``text`` каждую вставку ``[...]`` - где бы она ни стояла.

    Единственное место правила «вставки в квадратных скобках не произносятся». Вложенные скобки снимаются изнутри
    наружу (не больше четырёх уровней), незакрытая ``[`` срезает хвост. Не-строка и текст без ``[`` возвращаются как есть.
    """
    if not isinstance(text, str) or "[" not in text:
        return text
    out = text
    for _ in range(4):
        new = _BRACKET_ASIDE_RE.sub(" ", out)
        if new == out:
            break
        out = new
    out = _BRACKET_TAIL_RE.sub("", out)
    return _SPACE_RUN_RE.sub(" ", out).strip()


def is_system_template_regurgitated_in_ssml(ssml: Optional[str]) -> bool:
    """Issue #2175 - в СЫРОМ SSML лежит только кусок системного шаблона, а не реплика.

    Смотреть надо на сырой SSML: после снятия тегов ``<system>…</system>`` исчезает, и содержимое блока выглядит обычным
    текстом. Ловит ``<speak><system>…</system></speak>`` и ``<system>…</system>`` без обёртки; пустая строка / ``None`` -
    ``False``.
    """
    if not ssml:
        return False
    return bool(_SYSTEM_TEMPLATE_RE.match(ssml))


__all__ = [
    "is_system_template_regurgitated_in_ssml",
    "strip_bracket_asides",
    "strip_markdown",
]
