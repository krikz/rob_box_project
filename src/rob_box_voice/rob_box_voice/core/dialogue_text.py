"""Pure helpers for wake-word / silence-command text classification.

Extracted from :class:`DialogueManager` and the per-method regex in
:mod:`rob_box_harness.harnesses.dialog` so the same logic can be used
by:

* the legacy ``DialogueNode`` (OpenAI Agents SDK pipeline)
* the new ``DialogHarness`` (harness framework)
* unit tests that don't want to spin up the full state machine

Everything in this module is **pure**: no I/O, no ROS2, no time
side-effects (callers pass in the wake-word list / lowercased text).
That makes the helpers cheap to test in isolation (see
``test/unit/core/test_dialogue_text.py``).
"""

from __future__ import annotations

import os
import re
from typing import Sequence

# Default trigger phrases — match what DialogueManager used historically
# so behaviour is preserved when the legacy node switches to these
# helpers. Includes spelling variants that the dialog harness originally
# supported inline (роббокс, роб бокс, робокс).
# 🔴 fix(voice #1252): синхронизировано с dialogue_node.yaml — 12 вариантов
# + исторический «робик» (потерян при 9ca7fb29, 21.02). STT реально выдаёт
# кривые варианты («робок», «роберт», «рыбок», «роботс») — все покрываем.
# NB: порядок ВАЖЕН для strip_wake_word (regex-альтернация leftmost-first) —
# более длинные/специфичные варианты идут ПЕРВЫМИ, иначе «роб» съест «роб бокс».
DEFAULT_WAKE_WORDS: tuple[str, ...] = (
    "роб бокс",
    "роббокс",
    "робокос",
    "rob box",
    "робокс",
    # STT-искажения «робокс» (e2e ww01, run 32595628905): vosk/yandex
    # слышат «робэкс», «робекс», «робакс», «рабокс», «рубокс» — покрываем
    # замены гласной. «роблокс» уже был в docker-конфиге — синхронизируем.
    "робэкс",
    "робекс",
    "робакс",
    "рабокс",
    "рубокс",
    "роблокс",
    "роберт",
    "роббос",
    "robbox",
    "робот",
    "робок",
    # STT-искажения «робок»: гласная первого/второго слога.
    "рабок",
    "робак",
    "рыбок",
    "робик",
    "робо",
    "рома",
    "бот",
    "роб",
)
# Namespace оператора (целевая §7.3, issue #1990): вейк «ТАРС» для агента
# оператора. Слышит ТОЛЬКО микрофон шлема (wake-поток /audio/quest_wake);
# из ReSpeaker-канала игнорируется (namespace привязан к источнику аудио).
# STT-искажения («тарз», «тас», «target»...) НЕ придумываем — наполняем по
# логам e2e (целевая §14.1), как собирали для «роббокс».
DEFAULT_OPERATOR_WAKE_WORDS: tuple[str, ...] = ("тарс", "tars")
DEFAULT_SILENCE_COMMANDS: tuple[str, ...] = ("помолч", "замолч", "хватит")
DEFAULT_UNSILENCE_COMMANDS: tuple[str, ...] = (
    "говори",
    "включ",
    "работ",
    "отвеч",
    "разговар",
)


# ---------------------------------------------------------------------------
# Wake-word
# ---------------------------------------------------------------------------


_WAKE_WORD_PATTERN_CACHE: dict[tuple[str, ...], "re.Pattern[str]"] = {}


def has_wake_word(text_lower: str, wake_words: Sequence[str]) -> bool:
    """Return True if any ``wake_word`` appears as a *word* in ``text_lower``.

    Empty ``wake_words`` means "bypass mode" — accept every input.

    🔴 FIX (issue #1292): раньше использовалась подстрока
    (``any(w in text_lower ...)``) — «бот» ∈ «работает» давал ложный wake
    word → LLM вызывался на фоновую речь и повторял старые команды из
    истории диалога. Теперь, как и :func:`strip_wake_word`, матчим
    отдельные слова через ``\\b`` (word boundary). Это сохраняет кейс
    «робот» в середине фразы («денчик ой фу робот меня зовут» — фикс
    10.08) и не ловит «работник», «заработок», «работает».
    """
    if not wake_words:
        return True
    key = tuple(wake_words)
    pattern = _WAKE_WORD_PATTERN_CACHE.get(key)
    if pattern is None:
        pattern = re.compile(
            r"\b(" + "|".join(re.escape(w) for w in wake_words) + r")\b",
            re.IGNORECASE,
        )
        _WAKE_WORD_PATTERN_CACHE[key] = pattern
    return bool(pattern.search(text_lower))


def strip_wake_word(text: str, wake_words: Sequence[str] | None = None) -> str:
    """Remove the *first* matching wake word from ``text`` (any position).

    Used by the harness wake-word gate (``DialogHarness._strip_wake_word``)
    AND by the legacy node's ``DialogueManager.remove_wake_word``.

    🔴 FIX (live 10.08): regex was anchored ``^`` — пропускал wake-word
    в середине фразы (напр. «денчик ой фу робот меня зовут...»).
    ``on_user_input()`` в AgentCore видел «робот» → WAKE_WORD вместо
    STT_RESULT → guard ``event==STT_RESULT`` пропускал LLM → тишина.
    Теперь удаляем из ЛЮБОГО места в тексте.

    Args:
        text: Raw user input (case-insensitive).
        wake_words: Optional override; defaults to
            :data:`DEFAULT_WAKE_WORDS`.
    """
    words = wake_words if wake_words is not None else DEFAULT_WAKE_WORDS
    if not words:
        return text.strip()
    # Case-insensitive, any-position match.  Consume trailing
    # punctuation/whitespace so the cleaned text is usable.
    pattern = re.compile(
        r"\b(" + "|".join(re.escape(w) for w in words) + r")[.,!?\s]*",
        re.IGNORECASE,
    )
    return pattern.sub("", text, count=1).strip()


# ---------------------------------------------------------------------------
# SSoT wake-слов из YAML (целевая §7.3, issue #1990)
# ---------------------------------------------------------------------------


def load_wake_word_namespaces(path: str | os.PathLike[str] | None) -> dict[str, list[str]]:
    """Прочитать ``config/wake_words.yaml`` (SSoT) в ``{namespace: [words]}``.

    Namespaces: ``personality`` (только ReSpeaker → ``/voice/stt/result``) и
    ``operator`` (только микрофон шлема, wake-поток). Порядок списка
    сохраняется КАК В ФАЙЛЕ — от него зависит :func:`strip_wake_word`
    (regex leftmost-first, длинные варианты первыми).

    Безопасная функция: при пустом пути / отсутствующем файле / битом YAML /
    незнакомой структуре возвращает ``{}`` (а не падает) — вызывающий решает,
    откатиться ли на кодовые дефолты (:data:`DEFAULT_WAKE_WORDS` /
    :data:`DEFAULT_OPERATOR_WAKE_WORDS`).
    """
    if not path:
        return {}
    import yaml  # noqa: PLC0415 — лениво: модуль остаётся чистым без пути

    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(data, dict):
        return {}
    out: dict[str, list[str]] = {}
    for namespace in ("personality", "operator"):
        words = data.get(namespace)
        if isinstance(words, list):
            cleaned = [w for w in words if isinstance(w, str) and w.strip()]
            if cleaned:
                out[namespace] = cleaned
    return out


def resolve_wake_word_namespaces(
    path: str | os.PathLike[str] | None,
    personality_fallback: Sequence[str] = DEFAULT_WAKE_WORDS,
    operator_fallback: Sequence[str] = DEFAULT_OPERATOR_WAKE_WORDS,
) -> tuple[list[str], list[str]]:
    """Вернуть ``(personality, operator)`` списки вейк-слов (порядок важен).

    Файл — единственный SSoT в проде (docker: ``/config/wake_words.yaml``).
    Если файл не задан / не читается / не содержит namespace — берём
    переданные фолбеки (в юнит-тестах и dev-env без файла поведение
    не меняется; в проде список копируется из кода в YAML байт-в-байт).
    """
    loaded = load_wake_word_namespaces(path)
    personality = loaded.get("personality") or list(personality_fallback)
    operator = loaded.get("operator") or list(operator_fallback)
    return personality, operator



# ---------------------------------------------------------------------------
# Silence / unsilence commands
# ---------------------------------------------------------------------------


def is_silence_command(text_lower: str, commands: Sequence[str] | None = None) -> bool:
    """Return True if ``text_lower`` matches any silence trigger phrase."""
    cmds = commands if commands is not None else DEFAULT_SILENCE_COMMANDS
    return any(cmd in text_lower for cmd in cmds)


def is_unsilence_command(text_lower: str, commands: Sequence[str] | None = None) -> bool:
    """Return True if ``text_lower`` matches any unsilence trigger phrase."""
    cmds = commands if commands is not None else DEFAULT_UNSILENCE_COMMANDS
    return any(cmd in text_lower for cmd in cmds)


__all__ = [
    "DEFAULT_WAKE_WORDS",
    "DEFAULT_OPERATOR_WAKE_WORDS",
    "DEFAULT_SILENCE_COMMANDS",
    "DEFAULT_UNSILENCE_COMMANDS",
    "has_wake_word",
    "strip_wake_word",
    "is_silence_command",
    "is_unsilence_command",
    "load_wake_word_namespaces",
    "resolve_wake_word_namespaces",
]
