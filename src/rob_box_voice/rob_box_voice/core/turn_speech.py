"""Что озвучивать из свободного текста хода (issue #2874).

Живой прогон 23.09.2026 17:30 (DJ «Снупдог»): music-гуард (Bug C) три раза
подряд отправлял синхронный ретрай, а каждый ход до ретрая успевал уйти в
TTS — юзер услышал три разные фразы подряд, а потом, когда общий
retry-budget (#1881) кончился, ещё и сырой ответ модели целиком: 11
TTS-чанков, ~40 с монолога про «нет mp3-библиотеки».

Внутри ``_run_turn`` свободный текст хода придерживается в
:class:`TurnSpeechHold` и выпускается в конце хода (вопрос о личности звучит
последним). Что именно выпустить — решает чистая :func:`decide_turn_speech`:

* ответ отозван (``discard_last_reply`` + фраза кода через ``_speak_direct``)
  → молчим, фраза кода уже звучит;
* музыкальный/DJ-контекст и ответ длиннее :data:`MUSIC_MAX_CHUNKS`
  TTS-чанков → первая фраза. Исключение — юзер сам просил текст
  (рэп/стих/песню, :func:`wants_lyrics`): там длинный ответ и есть исполнение
  (issue #980).

ADR-0150 PR-7 ч.7d-4 (#3698): синтетических ретраев LLM больше нет — ветки
«за ходом ретрай», «бюджет ретраев исчерпан» и отказ по процитированному
``[CRITICAL]``-промпту (#2999) удалены.

Модуль без ROS2 — тестируется напрямую.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional

from .dialogue_guards import is_system_template_regurgitated_in_ssml, is_tool_call_markup

#: Потолок «первой фразы» для fallback-озвучки (acceptance #2874: ~120).
FIRST_SENTENCE_LIMIT: int = 120

#: Сколько TTS-чанков свободного текста допустимо в музыкальном/DJ-контексте.
#: Больше — это монолог поверх (или вместо) музыки, режем до первой фразы.
MUSIC_MAX_CHUNKS: int = 3

#: Юзер просит именно ТЕКСТ (исполнение словами) — длинный ответ легитимен.
_LYRICS_KEYWORDS: tuple = (
    "рэп", "rap", "зачита", "стих", "поэм", "песн", "спой", "пой ",
    "частушк", "куплет",
)

_SENTENCE_END_RE = re.compile(r"(?<=[.!?…])\s+")


def wants_lyrics(user_input: Optional[str]) -> bool:
    """Юзер просил исполнить текст (рэп/стих/песню) — не резать ответ."""
    low = (user_input or "").lower()
    return any(kw in low for kw in _LYRICS_KEYWORDS)


def first_sentence(text: str, limit: int = FIRST_SENTENCE_LIMIT) -> str:
    """Первая фраза ``text``, не длиннее ``limit`` символов.

    Длинная первая фраза режется по последнему пробелу до лимита и
    закрывается многоточием — TTS не зачитывает обрывок слова.
    """
    stripped = (text or "").strip()
    if not stripped:
        return ""
    head = _SENTENCE_END_RE.split(stripped, maxsplit=1)[0].strip()
    if len(head) <= limit:
        return head
    cut = head[:limit].rsplit(" ", 1)[0].rstrip(" ,;:—-")
    return (cut or head[:limit]) + "…"


@dataclass
class TurnSpeechHold:
    """Придержанный свободный текст одного хода (issue #2874).

    ``text`` — что ``_handle_result`` собирался озвучить; ``retracted`` —
    гуард отозвал ответ хода (``_discard_last_music_reply``) и сам сказал
    короткую фразу вместо него.
    """

    text: Optional[str] = None
    user_input: Optional[str] = None
    retracted: bool = False

    def hold(self, text: str, user_input: Optional[str] = None) -> None:
        self.text = text
        self.user_input = user_input

    def retract(self) -> None:
        self.retracted = True


def decide_turn_speech(
    text: Optional[str],
    *,
    n_chunks: int,
    retracted: bool = False,
    music_context: bool = False,
    lyrics_requested: bool = False,
) -> Optional[str]:
    """Что озвучить из свободного текста хода; ``None`` — ничего.

    Args:
        text: свободный текст хода после всех strip'ов ``_handle_result``.
        n_chunks: сколько TTS-чанков из него получится (``split_into_chunks``).
        retracted: ответ отозван, вместо него звучит фраза кода.
        music_context: DJ-сессия или музыкальный запрос юзера.
        lyrics_requested: юзер просил исполнить текст (:func:`wants_lyrics`).
    """
    if not text or retracted:
        return None
    if music_context and not lyrics_requested and n_chunks > MUSIC_MAX_CHUNKS:
        return first_sentence(text) or None
    return text


def final_text_for_speech(result: object, warn) -> str:
    """Issue #3598 — финальный текст хода, который можно отдавать в авто-TTS.

    Ядро (``AgentCore``) помечает ``internal_text_marker``, когда текст
    цитирует служебное из контекста хода (``<music_state>``) или имя тула
    (``speak_text``) — это рассуждение модели, не реплика человеку. Такой
    текст не озвучивается и не уходит дублем в Telegram: возвращается
    ``""``, и дальше ``_handle_result`` идёт штатным путём пустого ответа
    (ход без тулов → честная фраза кода :data:`EMPTY_REPLY_PHRASE`, с тулами
    → тишина, о действии говорит код). ``warn`` — логгер для строки
    «🤐 [issue 3598] …».

    Тем же путём уходит служебный синтаксис вместо речи — разметка вызова тула (#2760) и ``<system>``-шаблон
    (#2175): дальше по тракту ``build_ssml_payload`` снимает теги, и отказ tts_node их уже не узнал бы (робот прочёл
    бы значения параметров). Детекторы — те же, что у tts_node (одна таблица в ``dialogue_guards``); ретрай LLM
    на такой ответ удалён в ADR-0150 PR-7 ч.7d-2 (#3698), нормализация переезжает в tts_node в PR-15.
    """
    text = getattr(result, "spoken_text", "") or ""
    service = _service_syntax(text)
    if service:
        warn(f"🤐 [{service}] финальный текст хода — служебный синтаксис, не озвучиваю: {text[:160]!r}")
        return ""
    marker = getattr(result, "internal_text_marker", None)
    if not marker:
        return text
    warn(
        f"🤐 [issue 3598] финальный текст хода — рассуждение модели "
        f"(цитирует {marker!r}), не озвучиваю: {text[:160]!r}"
    )
    return ""


def _service_syntax(text: str) -> str:
    """Метка служебного синтаксиса в тексте хода (``issue 2760`` / ``issue 2175``) или ``""``."""
    if is_tool_call_markup(text):
        return "issue 2760"
    if is_system_template_regurgitated_in_ssml(text.strip()):
        return "issue 2175"
    return ""


def final_text_skipped_log(text: str, verbose: bool) -> str:
    """Issue #988 — строка лога: ``speak_text`` уже прозвучал, финальный текст хода не озвучиваем."""
    if verbose:
        return f"🔇 [issue 988] speak_text called in cycle — skipping auto-TTS of final text: {text[:200]!r}"
    return f"🔇 [issue 988] speak_text called — final text skipped (anti-duplicate): {text[:80]!r}"


__all__ = [
    "FIRST_SENTENCE_LIMIT",
    "MUSIC_MAX_CHUNKS",
    "TurnSpeechHold",
    "decide_turn_speech",
    "final_text_for_speech",
    "final_text_skipped_log",
    "first_sentence",
    "wants_lyrics",
]
