"""Что озвучивать из свободного текста хода (issue #2874).

Живой прогон 23.09.2026 17:30 (DJ «Снупдог»): music-гуард (Bug C) три раза
подряд отправлял синхронный ретрай, а каждый ход до ретрая успевал уйти в
TTS — юзер услышал три разные фразы подряд, а потом, когда общий
retry-budget (#1881) кончился, ещё и сырой ответ модели целиком: 11
TTS-чанков, ~40 с монолога про «нет mp3-библиотеки».

Корень — порядок: ``_handle_result`` публиковал текст СРАЗУ, а post-turn
гуарды (music Bug B/C, tool-skipped) решали про ретрай уже ПОСЛЕ этого, в
``finally`` хода. Теперь внутри ``_run_turn`` свободный текст хода
придерживается в :class:`TurnSpeechHold` и выпускается только когда
гуарды сказали своё. Что именно выпустить — решает чистая
:func:`decide_turn_speech`:

* за ходом последовал синхронный ретрай → молчим: озвучится итог цепочки;
* гуард отозвал ответ (``discard_last_reply`` + своя короткая
  fallback-фраза через ``_speak_direct``) → молчим, фраза гуарда уже звучит;
* retry-budget исчерпан на этом ходе («отдаю как есть») → не сырой ответ, а
  первая фраза ≤ :data:`FIRST_SENTENCE_LIMIT` символов;
* музыкальный/DJ-контекст и ответ длиннее :data:`MUSIC_MAX_CHUNKS`
  TTS-чанков → первая фраза. Исключение — юзер сам просил текст
  (рэп/стих/песню, :func:`wants_lyrics`): там длинный ответ и есть исполнение
  (issue #980).

Модуль без ROS2 — тестируется напрямую.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional

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
    retry_dispatched: bool = False,
    retracted: bool = False,
    budget_exhausted: bool = False,
    music_context: bool = False,
    lyrics_requested: bool = False,
) -> Optional[str]:
    """Что озвучить из свободного текста хода; ``None`` — ничего.

    Args:
        text: свободный текст хода после всех strip'ов ``_handle_result``.
        n_chunks: сколько TTS-чанков из него получится (``split_into_chunks``).
        retry_dispatched: за ходом последовал синхронный ретрай.
        retracted: гуард отозвал ответ и озвучил свой fallback.
        budget_exhausted: на этом ходе кончился общий retry-budget (#1881).
        music_context: DJ-сессия или музыкальный запрос юзера.
        lyrics_requested: юзер просил исполнить текст (:func:`wants_lyrics`).
    """
    if not text or retry_dispatched or retracted:
        return None
    if budget_exhausted:
        return first_sentence(text) or None
    if music_context and not lyrics_requested and n_chunks > MUSIC_MAX_CHUNKS:
        return first_sentence(text) or None
    return text


__all__ = [
    "FIRST_SENTENCE_LIMIT",
    "MUSIC_MAX_CHUNKS",
    "TurnSpeechHold",
    "decide_turn_speech",
    "first_sentence",
    "wants_lyrics",
]
