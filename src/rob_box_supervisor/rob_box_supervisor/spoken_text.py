"""Речевая версия ответа ТАРС (issue #3296): коротко и без разметки.

Чистая функция без ROS. Полный ответ (markdown, списки, PromQL) остаётся
на экране (``/avatar/command_result``); в TTS уходит только очищенное начало:
первые предложения в пределах лимита длины. Правила общие, без условий под
конкретные фразы.
"""

from __future__ import annotations

import re

DEFAULT_MAX_CHARS = 200
DEFAULT_MAX_SENTENCES = 2
# Добавляется, когда речь короче полного ответа.
DETAILS_SUFFIX = "Подробности на экране."

_FENCE_RE = re.compile(r"```.*?(?:```|\Z)", re.DOTALL)
_INLINE_CODE_RE = re.compile(r"`[^`\n]*`")
_URL_RE = re.compile(r"https?://\S+|www\.\S+")
_LINK_RE = re.compile(r"\[([^\]]*)\]\([^)]*\)")
_LINE_MARK_RE = re.compile(r"^\s*(?:#{1,6}\s+|>\s*|[-*+•]\s+|\d+[.)]\s+)", re.MULTILINE)
_EMPHASIS_RE = re.compile(r"[*_~]{1,3}(?=\S)|(?<=\S)[*_~]{1,3}")
_TABLE_RE = re.compile(r"^\s*\|.*$", re.MULTILINE)
_SENTENCE_END_RE = re.compile(r"(?<=[.!?…])\s+")


def _strip_markup(text: str) -> str:
    text = _FENCE_RE.sub(" ", text)
    text = _INLINE_CODE_RE.sub(" ", text)
    text = _TABLE_RE.sub(" ", text)
    text = _LINK_RE.sub(r"\1", text)
    text = _URL_RE.sub(" ", text)
    text = _LINE_MARK_RE.sub("", text)
    text = _EMPHASIS_RE.sub("", text)
    return text


def _clean_sentences(text: str) -> list[str]:
    out: list[str] = []
    for line in text.splitlines():
        line = re.sub(r"\s+", " ", line).strip()
        if not line:
            continue
        out.extend(p for p in _SENTENCE_END_RE.split(line) if p.strip())
    return out


def _fit(sentences: list[str], max_chars: int, max_sentences: int) -> str:
    picked: list[str] = []
    for s in sentences[:max_sentences]:
        if len(" ".join(picked + [s])) > max_chars:
            break
        picked.append(s)
    if picked:
        return " ".join(picked)
    # Первое предложение само длиннее лимита: режем по границе слова.
    first = sentences[0][:max_chars]
    cut = first.rsplit(" ", 1)[0] if " " in first else first
    return cut.rstrip(" ,;:-—") + "…"


def to_spoken(
    text: str,
    max_chars: int = DEFAULT_MAX_CHARS,
    max_sentences: int = DEFAULT_MAX_SENTENCES,
    details_suffix: str = DETAILS_SUFFIX,
) -> str:
    """Речевая версия ``text``: без markdown/кода/URL, 1-2 предложения, лимит."""
    if not text or not text.strip():
        return ""
    sentences = _clean_sentences(_strip_markup(text))
    if not sentences:
        return ""
    spoken = _fit(sentences, max_chars, max_sentences)
    truncated = len(spoken.rstrip("…")) < len(" ".join(sentences))
    if truncated and details_suffix:
        return f"{spoken} {details_suffix}"
    return spoken
