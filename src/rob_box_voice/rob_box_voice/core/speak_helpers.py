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

# voice-vr 12 (issue #2197, ADR-0080 §1.3 / §2.3): единое место сборки
# SSML — раньше здесь был ``f"<speak>{text}</speak>"`` без экранирования.
# Теперь текст проходит через ``Utterance.ssml`` (XML-escape для ``&``/`<`/`>`).
from rob_box_core.utterance import Sink, Utterance

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

# Strip leading meta-markers the LLM occasionally emits in
# ``spoken`` — internal section headers that were meant for the
# assistant's own reasoning but leaked past the system prompt. TTS
# reads ``spoken`` verbatim, so leaking these prefixes produces
# audible noise (live 15.09: «[Мнение ассистента] Вплела тему
# Грига…»).
#
# Anchored at start-of-string, optionally preceded by whitespace, and
# consumes **one** prefix at a time — the caller (``strip_meta_markers``
# below) loops so stacked prefixes («[Мнение ассистента] **Итог:** …»)
# collapse cleanly. Two prefix shapes:
#
# * ``[Anything]`` — bracketed section header (issue #2547 live case:
#   ``[Мнение ассистента]``, ``[Примечание]``, ``[Note]``, ``[Answer]``).
#   Square brackets inside the marker are not allowed (the regex
#   stops at the first ``]``).
#
#     **Excluded** (must remain visible to the service-text guard
#     running downstream in ``_handle_result``):
#
#     * ``[CRITICAL]`` — internal retry prompt (suppressed by
#       ``_check_babble_and_retry`` / service-text guard).
#     * ``[SYSTEM ...]`` — internal system template regurgitated
#       (issue #2175, suppressed by ``is_system_template_regurgitated``).
#     * ``[Spkr:<name>]`` — speaker routing marker (owned by
#       :func:`strip_speaker_tag` which runs first).
#
# * ``**Anything**`` — Markdown-bold section header (``**Итог:**``,
#   ``**Answer:**``).
_META_PREFIX_RE = re.compile(
    r"^\s*(?:"
    # Bracketed meta-markers — only when the inner content is NOT one of
    # the reserved service-text prefixes. Case-insensitive (CRITICAL /
    # Critical / critical are all reserved).
    r"(?!\s*\[(?:CRITICAL|Spkr:[^\]]*|SYSTEM\b)[^\]]*\])"
    r"\[[^\]]+\]"
    r"|"
    # Markdown-bold meta-markers. Bold-form of CRITICAL doesn't exist
    # in practice; no exclusion needed here.
    r"\*\*[^*]+\*\*"
    r")"
    r"\s*(?:[:\-—]\s*)?",
    flags=re.UNICODE | re.IGNORECASE,
)

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


def strip_meta_markers(text: str) -> str:
    """Remove leading meta-markers (``[…]``, ``**…**``) from ``spoken`` (issue #2547).

    MiniMax-M1 sometimes prefixes its reply with internal section headers
    like ``[Мнение ассистента]``, ``[Примечание]``, ``[Note]`` or
    ``**Итог:**``. These were meant for the assistant's own reasoning
    (structured answer sections), but they leaked past the system prompt
    into the ``spoken`` field that is read by TTS verbatim. The user hears
    the prefix as part of the reply, which is confusing and breaks the
    conversational tone (live 15.09 DJ test: 21 occurrences on a single
    session; round 3 regression: 193 cases/hour).

    Strip is applied BEFORE ``strip_markdown`` so the bold-form prefix
    (``**…**``) is removed before markdown rules can corrupt it. The
    bracketed-form prefix (``[Spkr:<имя>]``) is **not** matched here —
    that one is owned by :func:`strip_speaker_tag` which runs first in
    the pipeline (``_handle_result``).

    Rules (anchored at start-of-string, looped to consume stacks):

    * optional leading whitespace;
    * one bracketed marker ``[<non-bracket>+]`` or one Markdown-bold
      marker ``**<non-asterisk>+**``;
    * optional trailing whitespace;
    * optional trailing colon/dash separator (``[Мнение ассистента]:``,
      ``**Итог** —``).

    Loop is bounded to ``_MAX_STACK`` iterations — more than 4 stacked
    meta prefixes in a single response is pathological and most likely
    a model hallucination; falling through unchanged is the right
    behaviour (the downstream equality check still recognises
    done-markers and the chunking layer survives the prefix).

    Pure Python — no ROS, no heavy deps. Non-string input is returned
    as-is (matches the contract of :func:`strip_thinking_blocks`).
    """
    if not isinstance(text, str) or not text:
        return text
    out = text
    for _ in range(4):
        new = _META_PREFIX_RE.sub("", out, count=1).lstrip()
        if new == out:
            break
        out = new
    return out


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


# Issue #2557 (DJ live round 3, 2026-09-15): the LLM sometimes returns
# ``spoken='done'`` (or ``\n\ndone`` / «готово» / «всё» / …) AFTER calling
# music tools WITHOUT ``speak_text`` — the user hears music start but no
# audible acknowledgement. The cycle-end equality check in
# ``_handle_result`` correctly suppresses the marker from auto-TTS, but
# the user is left with silence after a music action. The DJ fallback
# (issue #2547) only fired when ``spoken`` was already empty post-strip
# AND the turn was NOT ``is_dj_auto`` — which is exactly the opposite of
# what we want here: on a DJ transition the announcement IS the
# information (track changed), not noise. This helper is called by
# ``_handle_result`` to replace the spoken text with a short
# DJ-appropriate phrase when the model called music tools but produced
# a degenerate (``done`` / empty) answer.
#
# The set below mirrors ``agent_core._MUSIC_LAUNCH_TOOLS`` plus the
# pure-DJ controls (``set_dj_mode``, ``stop_music``, ``lookup_melody``,
# ``load_skill``). Keep both lists in sync when adding a new music
# tool — otherwise the fallback won't fire and the user will hear
# silence again.
_DJ_MUSIC_TOOLS: frozenset[str] = frozenset({
    # Music launchers (agent_core._MUSIC_LAUNCH_TOOLS).
    "execute_music_code", "generate_music",
    "gen_play_from_library", "set_vibe_preset", "load_track",
    # Pure-DJ controls.
    "compose_music", "set_dj_mode", "stop_music",
    "lookup_melody", "load_skill",
})
#: Degenerate marker that the master-prompt cycle-end contract tells the
#: LLM to emit after the LAST ``speak_text``. Same set as the equality
#: check in ``_handle_result`` (``done`` / «готово» / «всё» / …). Used
#: here to recognise the «tools called but marker only» shape that
#: falls through to silence.
_DJ_DEGENERATE_MARKERS: frozenset[str] = frozenset({
    "done", "task complete", "task_complete",
    "готово", "готов", "готова",
    "всё", "выполнено", "завершено", "завершена",
})
#: Short DJ announcement when the model changed the music without
#: speaking. Tuned to be informative without claiming a specific
#: action: «Готово, играю.» confirms the music change reached TTS, then
#: the rest of the ``spoken`` (when present and non-degenerate) follows.
#: No emoji / no exclamation — keeps the tone neutral and matches the
#: other DJ hooks (``Принял.``, «Понял.»).
_DJ_FALLBACK_PHRASE: str = "Готово, играю."


def ensure_dj_music_response(
    spoken: str,
    tools_called: Optional[List[str]],
    *,
    is_dj_auto: bool = False,
    track_name: Optional[str] = None,
    theme: Optional[str] = None,
    persona: Optional[str] = None,
) -> str:
    """Return a DJ-style fallback when music tools ran without real
    user-facing text (issue #2557; #2857 extends it — see below).

    Contract:

    * ``spoken`` is the post-strip, post-cycle-marker-equal text
      (already passed through ``strip_done_marker`` etc.). May be
      empty, whitespace, or one of :data:`_DJ_DEGENERATE_MARKERS`.
    * ``tools_called`` is the list of tool names the LLM called this
      turn (may be ``None``).

    Returns the cleaned ``spoken`` if it looks like a real reply;
    otherwise returns a fallback when ``tools_called`` intersects
    :data:`_DJ_MUSIC_TOOLS`.

    Issue #2857 — live 23.09.2026: ``speak_text`` was called AND
    voiced a real DJ line this turn, but the post-strip ``spoken``
    field still ended up empty/``done`` (the LLM's cycle-end
    contract), and the fallback stomped the already-spoken line with
    a second, duller phrase. If ``speak_text`` is in ``tools_called``
    at all, this turn already had its say — never override it here,
    regardless of what ``spoken`` looks like.

    Issue #2857 also replaces the flat ``"Готово, играю."`` on
    autonomous DJ transitions (``is_dj_auto=True``): the generic
    phrase is only appropriate when the USER directly asked for music
    and got no reply text. On a DJ auto-transition, a short
    track-specific line is more informative — built from whatever is
    cheaply available this turn (``track_name``, else ``theme``, else
    ``persona``). If none of those are available, stay silent
    (``""``) rather than repeat the dull phrase every transition.

    Pure / no ROS, no side effects — caller decides whether to publish.
    Designed to be the single source of truth so the dialogue_node
    change is a one-liner and stays under the CC budget.
    """
    if tools_called is None:
        return spoken
    # Defensive: mirror ``strip_done_marker`` contract — non-string
    # ``spoken`` is the caller's problem (the dialogue_node already
    # does ``result.spoken_text or ""`` upstream), but if it slips
    # through here we pass it through untouched rather than crash on
    # ``.strip()``.
    if not isinstance(spoken, str):
        return spoken
    called = set(tools_called)
    if "speak_text" in called:
        # Issue #2857 — speak_text already voiced this turn's line (the
        # live bug: 'Йоу, народ, гангста-драйв качает!' via speak_text,
        # then 'Готово, играю.' stomped on top of it). Never publish a
        # second, generic line over an already-spoken one.
        return spoken
    if not (called & _DJ_MUSIC_TOOLS):
        return spoken
    # Real user-facing reply? Leave it alone — the master-prompt contract
    # allows the model to call music tools AND speak (e.g. «Запускаю
    # Баха!»). Only intervene when the reply is empty or one of the
    # cycle-end markers the LLM uses to signal turn end without
    # producing speech.
    stripped = spoken.strip()
    if stripped and stripped.lower() not in _DJ_DEGENERATE_MARKERS:
        return spoken
    if not is_dj_auto:
        # Direct user request ("сыграй что-нибудь") with no reply text —
        # the generic confirmation is still the right call here.
        return _DJ_FALLBACK_PHRASE
    # DJ auto-transition without any spoken line: prefer a short,
    # track-specific announcement over the generic phrase; silence
    # beats a robotic "Готово, играю." repeated every ~45s.
    name = (track_name or theme or "").strip()
    if not name:
        return ""
    persona = (persona or "").strip()
    if persona:
        return f"{persona}: дальше — {name}!"
    return f"Дальше — {name}!"


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
# `minimax_language="ru"`. Здесь — недостающее поле payload'а плюс ответ
# на вопрос «а этот провайдер вообще так умеет?».
#
# Кто какой язык умеет — считается ПО КАТАЛОГУ ГОЛОСОВ
# (`tts_voice_registry.languages_for`), а не по отдельной таблице:
# вторая таблица разъехалась бы с каталогом ровно так же, как разъехались
# whitelist'ы AV-28. Исключение ровно одно и оно явное — MiniMax.

# MiniMax задаёт язык полем `language_boost`, а не выбором голоса, поэтому
# он умеет ВСЁ из `minimax_tts._LANGUAGE_ALIASES` независимо от того, на
# каком языке говорят голоса каталога (там ru и zh). Дублировать сюда весь
# список смысла нет — достаточно знать, что ограничений по каталогу нет.
LANGUAGE_AGNOSTIC_PROVIDERS: frozenset = frozenset({"minimax"})

# Как назвать язык в честной фразе-отказе.
_LANGUAGE_NAMES_RU: Dict[str, str] = {
    "ru": "русском",
    "en": "английском",
    "fr": "французском",
    "de": "немецком",
    "zh": "китайском",
    "hi": "хинди",
}


def provider_speaks(provider: str, language: Optional[str]) -> bool:
    """Умеет ли ``provider`` говорить на ``language``.

    ``language`` пустой/``None`` (обычный диалог робота) — всегда ``True``.
    """
    if not language:
        return True
    lang = str(language).strip().lower().split("-")[0]
    if not lang:
        return True
    prov = str(provider).strip().lower()
    if prov in LANGUAGE_AGNOSTIC_PROVIDERS:
        return True
    from ..tts_voice_registry import languages_for  # локально: избегаем цикла

    known = languages_for(prov)
    # Неизвестный провайдер не ограничиваем: пустой каталог — это «мы про
    # него ничего не знаем», а не «он ничего не умеет».
    return not known or lang in known


def unsupported_language_notice(
    provider: str, language: Optional[str]
) -> Optional[str]:
    """Фраза-отказ, если ``provider`` не умеет ``language``; иначе ``None``.

    Зачем отказ, а не транслит кириллицей: Silero с моделью ``v5_ru``
    читает по русским правилам, и «beaucoup», переписанное кириллицей,
    звучит «беаукоуп», а не «боку». Это молчаливая деградация — оператор
    слышит речь, считает, что робот говорит по-французски, и узнаёт правду
    от собеседника. Честная короткая фраза лучше (ADR-0018).

    Правильное решение — не транслит, а НАСТОЯЩИЙ голос на этом языке;
    из шести языков UI он есть почти везде:
      * MiniMax — все шесть (`language_boost`);
      * Yandex  — ru, en (`john`), de (`lea`); fr/zh/hi у него нет вовсе;
      * Silero  — сейчас загружена только `v5_ru`, но upstream отдаёт
        `v3_en`, `v3_de`, `v3_fr` и indic (`hindi_male`/`hindi_female`).
        Догрузить их — отдельная карточка (модели качаются на Pi
        поштучно); китайского у Silero нет.

    То есть транслит нужен ровно для одного пересечения — китайский
    офлайн, — и именно там он бесполезнее всего: палладица, прочитанная
    русским голосом, китайцу непонятна.
    """
    if provider_speaks(provider, language):
        return None
    lang = str(language).strip().lower().split("-")[0]
    name = _LANGUAGE_NAMES_RU.get(lang, lang)
    return (
        f"Не могу сказать это на {name}: сейчас работает голосовой движок "
        f"{provider}, у него нет голоса на этом языке."
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
    payload: Dict[str, Any] = Utterance(
        text=text,
        sink=Sink.SPEAKERS,
        emotion=animation or "neutral",
        extra={
            "speech_id": str(uuid.uuid4()),
            "batch_id": batch_id,
            "batch_index": batch_index,
            "batch_total": batch_total,
            "tg_chat_id": tg_chat_id,
            "voice": voice,
            "language": language,
        },
    ).to_request()
    # ``Utterance.to_request`` ставит ``emotion="neutral"`` по умолчанию и
    # выкидывает ``None``-поля из extra; здесь чистим ``None`` для обратной
    # совместимости со старыми подписчиками, которые ждут отсутствие ключей.
    for k in (
        "batch_id",
        "batch_index",
        "batch_total",
        "tg_chat_id",
        "voice",
        "language",
    ):
        if k in payload and payload[k] is None:
            payload.pop(k)
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
    "strip_meta_markers",
    "strip_done_marker",
    "split_into_chunks",
    "build_ssml_payload",
    "EffectAwaiterRegistry",
]
