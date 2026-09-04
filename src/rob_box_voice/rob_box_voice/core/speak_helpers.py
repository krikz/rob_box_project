"""
speak_helpers.py — Sentence splitting and TTS publish helpers.

Extracted from the legacy ``dialogue_node.py`` so the ROS2 shell stays
≤350 LOC while keeping the same race-protected TTS / sound awaiters
and SSML framing that the production pipeline needs.

These helpers are used by :class:`rob_box_voice.dialogue_node.DialogueNode`
to publish fallback responses (``_speak_direct``) and to release the
``speak_text`` / ``play_sound`` awaiters on ``/voice/tts/finished`` and
``/voice/sound/state``.

The eventual destination of this code is :mod:`rob_box_harness.effects`
(SpeakEffect / PlaySoundEffect); until that migration lands the
helpers live here to keep the W5 commit scoped.
"""

from __future__ import annotations

import asyncio
import json
import re
import threading
import uuid
from typing import Any, Callable, Dict, List, Optional

# Strip history marker prefix that some LLMs copy into output.
_HISTORY_MARKER_RE = re.compile(
    r"^\[(?:выполнено через|executed via):[^\]]*\]\s*",
    flags=re.IGNORECASE,
)

# Strip interleaved-thinking blocks (MiniMax M3 + Anthropic format).
# After removal, the trailing "done" marker (if present) is what's left
# — which the dialogue_node correctly recognises and skips from auto-TTS.
_THINK_BLOCK_RE = re.compile(r"<think>.*?</think>\s*", flags=re.DOTALL)

# Strip a leading speaker routing marker (``[Spkr:<имя>]``) that some
# providers echo back from the voice-channel input format. Internal
# routing marker — never meant to be voiced. Stripping it also unmasks a
# following ``[CRITICAL]`` / ``[SYSTEM ...]`` marker so the
# service-text guard in dialogue_node can catch it. The ``+`` consumes
# repeated leading tags in one match.
_SPEAKER_TAG_RE = re.compile(r"^(?:\s*\[Spkr:[^\]]*\])+", flags=re.IGNORECASE)

# Strip a trailing ``done`` / ``task complete`` / Russian equivalents
# that the LLM adds AFTER the final ``speak_text`` per the master-prompt
# cycle-end contract. The marker is **internal** (signals turn end) and
# MUST NOT reach TTS — the user would hear «...дан» (literal Russian
# pronunciation of "done") at the end of every reply.
#
# Anchored at end-of-string with optional whitespace/newlines BEFORE the
# marker (LLM commonly writes ``\n\ndone``) and trailing punctuation
# (``.``, no other chars). Case-insensitive. The marker set is identical
# to the equality check in ``dialogue_node._handle_result`` so the
# downstream "skip auto-TTS" path stays a single source of truth.
_DONE_MARKER_RE = re.compile(
    r"\s*[\r\n]+\s*"
    r"(?:done|task[ _]?complete|готов[оа]?|всё|выполнено|завершен[оа]?)\s*\.?\s*$",
    flags=re.IGNORECASE,
)


def strip_history_marker(text: str) -> str:
    """Return *text* with the leading ``[выполнено через: ...]`` marker removed."""
    if not text:
        return text
    return _HISTORY_MARKER_RE.sub("", text).strip()


def strip_speaker_tag(text: str) -> str:
    """Strip leading ``[Spkr:<имя>]`` routing markers copied into output.

    The voice channel tags user input as ``[Spkr:<имя>] <текст>``
    (master prompt RULE #SRC). Some providers copy that marker back into
    their final reply; it is an internal routing marker and must never
    reach TTS. Also exposes a following ``[CRITICAL]`` / ``[SYSTEM ...]``
    marker so the dialogue_node service-text guard fires instead of the
    internal instruction being spoken aloud.
    """
    if not isinstance(text, str):
        return text
    return _SPEAKER_TAG_RE.sub("", text, count=1).strip()


def strip_thinking_blocks(text: str) -> str:
    """Remove ``<think>...</think>`` blocks so internal reasoning does not
    leak into spoken output.

    MiniMax M3 returns a ``<think>...</think>`` block in ``content`` BEFORE
    the actual reply. If we don't strip it, the model literally says things
    like ``«Music started successfully. Now return "done" — no speak_text,
    no follow-up.»`` over TTS, and the user hears the model's internal
    monologue (often in English, even when the system prompt says Russian).
    """
    if not text:
        return text
    return _THINK_BLOCK_RE.sub("", text).strip()


def strip_done_marker(text: str) -> str:
    """Remove a trailing ``done`` / ``task complete`` / Russian-equivalent
    marker (issue #1564).

    The master prompt tells the LLM that, after the LAST ``speak_text``
    in a turn, its next response MUST be plain text ``"done"`` (no
    tool calls) — that text is the agentic-cycle terminator and is
    **internal**, not user-facing. But the model sometimes writes the
    marker AFTER the actual answer (``\\n\\ndone`` appended to ``spoken``)
    instead of replacing the answer with it. Without this strip the TTS
    engine reads the trailing word aloud as Russian «дан» («...дон»),
    so every reply ends with a robotic artifact.

    The marker set here mirrors the equality check in
    ``dialogue_node._handle_result`` (post-strip) so the downstream
    "skip auto-TTS" path stays the single source of truth for deciding
    whether the LLM actually spoke anything user-facing.

    Rules (anchored at end-of-string):

    * optional whitespace + one-or-more newlines BEFORE the marker
      (covers ``\\ndone``, ``\\n\\ndone``, ``\\r\\ndone``);
    * the marker itself — ``done``, ``task complete``, ``task_complete``,
      ``готово``/``готова``, ``всё``, ``выполнено``, ``завершено``/``завершена``;
    * trailing punctuation (``.``) and whitespace allowed;
    * case-insensitive (``Done`` / ``DONE`` / ``Done.`` all match).

    Pure Python — no ROS, no heavy deps. Non-string input is returned
    as-is (matches ``strip_markdown`` contract).
    """
    if not isinstance(text, str) or not text:
        return text
    return _DONE_MARKER_RE.sub("", text).rstrip()


#: Regexes applied by :func:`strip_markdown` in order. Each tuple is
#: ``(compiled_pattern, replacement)``. The emphasis patterns intentionally
#: require *paired* delimiters so a lone ``*`` or ``_`` (e.g. ``2 * 3``,
#: ``под_черкивание``) survives untouched.
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


def strip_markdown(text: str) -> str:
    """Strip common Markdown formatting before TTS synthesis (issue #988).

    The LLM frequently wraps poems / rap / emphasis in Markdown
    (``*Жил да был енот весёлый,*``). TTS engines read the literal
    ``*`` as «звёздочка», producing «звёздочка звёздочка лалала».
    This strips the *markers* while keeping the words, so the robot
    sings the song instead of spelling out punctuation.

    Rules are intentionally conservative:

    * emphasis markers (``**`` / ``*`` / ``__`` / ``_``) are removed
      only in *pairs* — a single ``*`` (multiplication) or ``_``
      (underscore inside a word) is left alone;
    * ``#`` headings, ``>`` blockquotes and list bullets are removed
      only at the start of a line;
    * inline code spans (`` ` ``) and links (``[text](url)``) keep the
      visible text.

    Returns the cleaned text; non-string input is returned as-is.
    """
    if not isinstance(text, str):
        return text
    for pattern, repl in _MARKDOWN_STRIP_RULES:
        text = pattern.sub(repl, text)
    return text.strip()



def split_into_chunks(text: str, max_len: int = 200) -> List[str]:
    """Sentence-aware splitter — keeps TTS requests under the SSML limit."""
    raw = re.split(r"(?<=[.!?;])\s+", text.strip())
    chunks: List[str] = []
    buf = ""
    for part in raw:
        part = part.strip()
        if not part:
            continue
        candidate = (buf + " " + part).strip() if buf else part
        if len(candidate) <= max_len:
            buf = candidate
        else:
            if buf:
                chunks.append(buf)
            if len(part) > max_len:
                sub_parts = re.split(r"(?<=,)\s+", part)
                sub_buf = ""
                for sp in sub_parts:
                    sub_c = (sub_buf + " " + sp).strip() if sub_buf else sp
                    if len(sub_c) <= max_len:
                        sub_buf = sub_c
                    else:
                        if sub_buf:
                            chunks.append(sub_buf)
                        sub_buf = sp
                buf = sub_buf
            else:
                buf = part
    if buf:
        chunks.append(buf)
    return [c for c in chunks if c.strip()] or [text]


# ── AV-28: язык произношения (per-utterance override) ──────────────────
#
# `ros2-audio-contract-spec.md` §2.2 объявляет `language` варьирующимся
# параметром («ROS-param minimax_language ИЛИ override»), но override
# никогда не был реализован: dialogue_node переписывал реплику на
# французский, а tts_node синтезировал её со статическим
# `minimax_language="ru"`. Здесь — недостающее поле payload'а.
#
# Кто какой язык умеет:
#   minimax — любой из `_LANGUAGE_ALIASES` (minimax_tts.py), язык задаёт
#             `language_boost`, голос при этом остаётся тем же → ограничений нет;
#   yandex  — весь наш каталог голосов ru-RU (tts_voice_registry.py);
#   silero  — загружена модель `v5_ru`, она умеет ТОЛЬКО русский.
#
# `None` в таблице = ограничений нет.
TTS_PROVIDER_LANGUAGES: Dict[str, Optional[frozenset]] = {
    "minimax": None,
    "yandex": frozenset({"ru"}),
    "silero": frozenset({"ru"}),
}

# Как назвать язык в честной фразе-отказе.
_LANGUAGE_NAMES_RU: Dict[str, str] = {
    "ru": "русском",
    "en": "английском",
    "fr": "французском",
    "de": "немецком",
    "zh": "китайском",
    "hi": "хинди",
}


def unsupported_language_notice(
    provider: str, language: Optional[str]
) -> Optional[str]:
    """Фраза-отказ, если ``provider`` не умеет ``language``; иначе ``None``.

    Зачем отказ, а не транслит: Silero с моделью ``v5_ru`` читает по
    русским правилам, и «beaucoup», переписанное кириллицей, звучит
    «беаукоуп», а не «боку». Это молчаливая деградация — оператор слышит
    речь, считает, что робот говорит по-французски, и узнаёт правду от
    собеседника. Честная короткая фраза лучше (ADR-0018).

    Правильное решение для offline — своя модель Silero на язык
    (``v3_fr``, ``v3_de``, …); это отдельная карточка: модели качаются на
    Pi поштучно, а китайского у Silero нет вовсе.

    ``language=None`` (обычный диалог робота) ограничений не имеет — эта
    функция для него всегда возвращает ``None``.
    """
    if not language:
        return None
    lang = str(language).strip().lower()
    if not lang:
        return None
    allowed = TTS_PROVIDER_LANGUAGES.get(str(provider).strip().lower())
    if allowed is None or lang in allowed:
        return None
    name = _LANGUAGE_NAMES_RU.get(lang, lang)
    return (
        f"Не могу сказать это на {name}: сейчас работает голосовой движок "
        f"{provider}, он умеет только русский."
    )


def build_ssml_payload(
    text: str,
    animation: str = "neutral",
    *,
    batch_id: Optional[str] = None,
    batch_index: Optional[int] = None,
    batch_total: Optional[int] = None,
    tg_chat_id: Optional[int] = None,
    voice: Optional[str] = None,
    language: Optional[str] = None,
) -> str:
    """Build the JSON string consumed by ``tts_node`` on ``/voice/dialogue/response``.

    Issue #980 — chunked long responses (rap, poetry) publish one TTS request
    per chunk. To let ``dialogue_node`` know when *all* chunks of a single
    assistant turn finished (and only then trigger music cleanup), each chunk
    in a batch carries a shared ``batch_id`` plus 1-based ``batch_index`` and
    ``batch_total`` counters. tts_node echoes those on ``/voice/tts/finished``
    and publishes a dedicated ``/voice/tts/batch_complete`` once the last
    chunk lands. Single-chunk turns simply reuse the chunk as both speech
    and batch identifiers — back-compat behaviour.

    Issue #1195 — ``tg_chat_id`` (Telegram chat the reply should be echoed
    into) is an extra routing hint for telegram_node; tts_node ignores
    unknown fields.
    """
    payload: Dict[str, Any] = {
        "ssml": f"<speak>{text}</speak>",
        "speech_id": str(uuid.uuid4()),
        "emotion": animation,
    }
    if batch_id is not None:
        payload["batch_id"] = batch_id
    if batch_index is not None:
        payload["batch_index"] = int(batch_index)
    if batch_total is not None:
        payload["batch_total"] = int(batch_total)
    if tg_chat_id is not None:
        payload["tg_chat_id"] = int(tg_chat_id)
    if voice is not None:
        payload["voice"] = voice
    if language is not None:
        payload["language"] = language
    return json.dumps(payload, ensure_ascii=False)


class EffectAwaiterRegistry:
    """Tracks in-flight TTS / sound awaiters keyed by speech_id.

    Mirrors the field set that lived inline in the legacy
    ``DialogueNode``:

    * ``tts_events`` — speech_id → ``asyncio.Event`` released on
      ``/voice/tts/finished``.
    * ``sound_done_event`` — single ``asyncio.Event`` released on
      ``/voice/sound/state`` ``"ready"``.

    The registry owns its own locks so the shell doesn't need to
    juggle threading.Lock instances inline. Callers wire up two
    callbacks (``release_tts`` / ``release_sound``) to schedule
    ``event.set()`` back on the asyncio loop thread-safely.
    """

    def __init__(
        self,
        *,
        release_tts: Callable[[Any], None],
        release_sound: Callable[[asyncio.Event], None],
    ) -> None:
        self._tts_events: Dict[str, Any] = {}
        self._tts_lock = threading.Lock()
        self._tts_pending_count: int = 0  # issue #935 v2: deferred cleanup counter
        self._sound_done_event: Optional[Any] = None
        self._sound_lock = threading.Lock()
        self._release_tts = release_tts
        self._release_sound = release_sound

    # ── TTS side ────────────────────────────────────────────────────

    def register_tts(self, speech_id: str, event: asyncio.Event) -> None:  # type: ignore[type-arg]
        with self._tts_lock:
            self._tts_events[speech_id] = event

    @property
    def has_pending_tts(self) -> bool:
        """True while at least one TTS chunk is still playing/synthesising."""
        with self._tts_lock:
            return self._tts_pending_count > 0 or len(self._tts_events) > 0

    def increment_tts_pending(self, count: int = 1) -> None:
        with self._tts_lock:
            self._tts_pending_count += count

    def pop_tts(self, speech_id: str) -> Optional[asyncio.Event]:  # type: ignore[type-arg]
        with self._tts_lock:
            return self._tts_events.pop(speech_id, None)

    def release_all_tts(self) -> None:
        with self._tts_lock:
            for event in self._tts_events.values():
                self._release_tts(event)
            self._tts_events.clear()

    def handle_tts_finished(self, payload: str) -> None:
        try:
            speech_id = json.loads(payload).get("speech_id", "")
        except (json.JSONDecodeError, TypeError, AttributeError):
            speech_id = (payload or "").strip()
        with self._tts_lock:
            if self._tts_pending_count > 0:
                self._tts_pending_count -= 1
        if not speech_id:
            return
        event = self.pop_tts(speech_id)
        if event is not None:
            self._release_tts(event)

    # ── Sound side ──────────────────────────────────────────────────

    def set_sound_event(self, event: asyncio.Event) -> None:  # type: ignore[type-arg]
        with self._sound_lock:
            self._sound_done_event = event

    def clear_sound_event(self) -> None:
        with self._sound_lock:
            event = self._sound_done_event
            self._sound_done_event = None
        if event is not None:
            self._release_sound(event)

    def handle_sound_state(self, payload: str) -> None:
        if (payload or "") != "ready":
            return
        with self._sound_lock:
            event = self._sound_done_event
        if event is not None:
            self._release_sound(event)


__all__ = [
    "strip_history_marker",
    "strip_markdown",
    "strip_done_marker",
    "split_into_chunks",
    "build_ssml_payload",
    "EffectAwaiterRegistry",
]
