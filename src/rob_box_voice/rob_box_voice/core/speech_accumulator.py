#!/usr/bin/env python3
"""SpeechAccumulator — аккумулятор фоновой речи без wake-слова.

Чистый модуль: без I/O, без ROS2. DialogueNode держит один экземпляр,
кладет сюда распознанные фразы без wake-word и сливает их в
``<speech_backlog>`` при следующем wake-word (см. docs/plans/
2026-08-20-voice-backlog-accumulator-design.md).
"""

from __future__ import annotations

import time
from typing import List, Optional

DEFAULT_MAX_ENTRIES = 30

_INSTRUCTION = (
    "[URGENT_BACKLOG] Ниже — фразы, которые пользователь(и) произнёс(ли) без "
    "обращённого wake-слова ДО текущего обращения. Это либо отложенный запрос "
    "пользователя, который плохо распознался, либо разговор, который ты подслушал. "
    "ВАЖНО: backlog ИМЕЕТ ПРИОРИТЕТ над историей диалога для явных команд "
    "(«включи X», «поставь Y», «сделай Z»). Если в текущей фразе нет своей "
    "явной команды (только wake-слово «робот», «повтори», «что думаешь», "
    "«давай зарегистрируем» и т.п.) — ВЫПОЛНИ последнюю явную команду из "
    "бэклога или ответь по сути. НЕ приветствуй, НЕ переспрашивай, не "
    "пересказывай фразы дословно. Если в бэклоге НЕСКОЛЬКО явных команд — "
    "выполни ПОСЛЕДНЮЮ (LRU)."
)


def _xml_escape(value: str) -> str:
    return value.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;")


def _is_entry_foreign(
    entry: dict,
    current_speaker_known: Optional[bool],
    current_speaker_name: Optional[str],
) -> bool:
    """Issue #2779 — запись принадлежит опознанному человеку, который не
    является текущим собеседником.

    ``current_speaker_known is None`` значит «вызывающий код не передал
    сигнал о текущем собеседнике» (старые/юнит-тестовые вызовы формата
    напрямую) — тогда фильтрацию не делаем вовсе, поведение 1-в-1 со
    старым (нет regressions в существующих тестах).

    Запись БЕЗ имени (``None``/``"незнакомец"``) никогда не считается
    чужой: анонимность — это не конкурирующая идентичность, ей нечего
    "утекать". Чужой может быть только ИМЕНОВАННАЯ запись, и только
    когда это имя не совпадает с текущим (тоже опознанным) собеседником.

    Ровно это остановило бы утечку #2779: биометрия дважды подряд
    сказала ``Speaker: unknown`` для реплики Гриши, но запись в
    аккумуляторе (добавленная СИНХРОННО, до того как identify()
    досчитал именно эту реплику) успела получить ``speaker_name="Борис"``
    от устаревшего ``_current_speaker``. К моменту слива бэклога текущий
    собеседник уже точно известен (``_apply_speaker_identity`` ждёт
    inference), и это несоответствие («Борис» ≠ unknown) ловится здесь —
    запись остаётся в аккумуляторе для СЛЕДУЮЩЕГО раза, когда Борис
    действительно вернётся, а не пересказывается случайному человеку.
    """
    if current_speaker_known is None:
        return False
    entry_name = entry.get("speaker_name")
    if not entry_name or entry_name == "незнакомец":
        return False
    return not (current_speaker_known and entry_name == current_speaker_name)


def format_ago_s(seconds: float) -> str:
    """Человекочитаемое «сколько назад» для записи бэклога."""
    seconds = max(0, int(seconds))
    if seconds < 60:
        return f"{seconds}с"
    minutes = seconds // 60
    if minutes < 60:
        return f"{minutes}м{seconds % 60}с"
    hours = minutes // 60
    return f"{hours}ч{minutes % 60}м"


class SpeechAccumulator:
    """Скользящее окно распознанной речи без wake-слова."""

    def __init__(
        self,
        window_sec: float = 180.0,
        max_entries: int = DEFAULT_MAX_ENTRIES,
    ) -> None:
        self.window_sec = window_sec
        self.max_entries = max_entries
        self._entries: List[dict] = []

    def add(
        self,
        text: str,
        speaker_tag: Optional[str] = None,
        speaker_name: Optional[str] = None,
    ) -> None:
        text = (text or "").strip()
        if not text:
            return
        self._entries.append(
            {
                "ts": time.time(),
                "text": text,
                "speaker_tag": speaker_tag,
                "speaker_name": speaker_name or "незнакомец",
            }
        )
        self._trim()

    def prune(self, now: Optional[float] = None) -> None:
        now = time.time() if now is None else now
        cutoff = now - self.window_sec
        self._entries = [e for e in self._entries if e["ts"] >= cutoff]
        self._trim()

    def _trim(self) -> None:
        if len(self._entries) > self.max_entries:
            self._entries = self._entries[-self.max_entries :]

    def is_empty(self) -> bool:
        return not self._entries

    def clear(self) -> None:
        self._entries.clear()

    def _visible_entries(
        self,
        current_speaker_known: Optional[bool],
        current_speaker_name: Optional[str],
    ) -> List[dict]:
        """Записи, которые МОЖНО показать текущему собеседнику сейчас.

        Issue #2779 — записи чужого (по имени) диктора не попадают в
        контекст текущего хода (критерий приёмки §2): они остаются в
        ``self._entries`` для следующего слива, когда за wake-словом
        придёт тот самый диктор.
        """
        return [
            e
            for e in self._entries
            if not _is_entry_foreign(e, current_speaker_known, current_speaker_name)
        ]

    def discard_visible(
        self,
        current_speaker_known: Optional[bool] = None,
        current_speaker_name: Optional[str] = None,
        now: Optional[float] = None,
    ) -> None:
        """Убрать из аккумулятора только записи, слитые в этот ход.

        Issue #2779 — раньше вызывающий код делал безусловный
        :meth:`clear` сразу после :meth:`format_block`, стирая заодно и
        записи ЧУЖОГО (по имени) диктора, которые в блок не попали. Это
        были бы потерянные навсегда факты/вопросы человека, если бы
        подряд говорил кто-то ещё, не прошедший wake-гейт. Теперь
        стираем ровно то множество, что реально ушло в LLM;
        неадресованное текущему собеседнику остаётся ждать своего часа
        (и всё равно будет вычищено по TTL — см. :meth:`prune`).
        """
        self.prune(now)
        if current_speaker_known is None:
            self.clear()
            return
        self._entries = [
            e
            for e in self._entries
            if _is_entry_foreign(e, current_speaker_known, current_speaker_name)
        ]

    def format_block(
        self,
        now: Optional[float] = None,
        *,
        current_speaker_known: Optional[bool] = None,
        current_speaker_name: Optional[str] = None,
    ) -> Optional[str]:
        """XML-блок ``<speech_backlog>`` или ``None``, если нечего показать.

        ``current_speaker_known``/``current_speaker_name`` — issue #2779:
        сигнал «кто сейчас говорит» (после разрешения биометрии).
        ``current_speaker_known=None`` (по умолчанию) — сигнала нет,
        фильтрация не делается (обратная совместимость со старыми
        вызовами/тестами). Иначе записи ИМЕНОВАННОГО чужого диктора не
        попадают в блок вовсе (см. :func:`_is_entry_foreign`).
        """
        self.prune(now)
        entries = self._visible_entries(current_speaker_known, current_speaker_name)
        if not entries:
            return None
        now = time.time() if now is None else now
        lines = [
            "<speech_backlog>",
            f"  <instruction>{_xml_escape(_INSTRUCTION)}</instruction>",
        ]
        for entry in entries:
            ago_s = format_ago_s(now - entry["ts"])
            tag = entry["speaker_tag"] or "?"
            speaker = _xml_escape(entry["speaker_name"] or "незнакомец")
            text = _xml_escape(entry["text"])
            lines.append(f'  <entry speaker_tag="{tag}" speaker="{speaker}" ' f'ago_s="{ago_s}">{text}</entry>')
        lines.append("</speech_backlog>")
        return "\n".join(lines)

    def format_user_hint(
        self,
        now: Optional[float] = None,
        *,
        current_speaker_known: Optional[bool] = None,
        current_speaker_name: Optional[str] = None,
    ) -> Optional[str]:
        """Plain-text подсказка для user-сообщения (не телеметрия).

        В отличие от :meth:`format_block`, который уходит в
        ``<system_context>`` как телеметрия, этот текст подставляется в
        user-turn, чтобы модель восприняла фоновую речь как отложенный
        запрос пользователя, а не метаданные.

        См. :meth:`format_block` про ``current_speaker_known``/
        ``current_speaker_name`` (issue #2779) — та же фильтрация чужих
        (по имени) записей применяется и здесь.
        """
        self.prune(now)
        entries = self._visible_entries(current_speaker_known, current_speaker_name)
        if not entries:
            return None
        lines = [
            "[URGENT_BACKLOG] [ФОНОВЫЙ ЗАПРОС] До этого обращения (без wake-слова) "
            "прозвучало:",
        ]
        for entry in entries:
            speaker = entry["speaker_name"] or "незнакомец"
            lines.append(f'- {speaker}: «{entry["text"]}»')
        lines.append(
            "ВАЖНО: backlog ИМЕЕТ ПРИОРИТЕТ над историей диалога. Если в текущей "
            "фразе есть своя явная команда («включи X», «поставь Y», «сделай Z») — "
            "выполни её. Если в текущей фразе НЕТ своей явной команды (только "
            "wake-слово «робот», «повтори», «что думаешь», «давай зарегистрируем» "
            "и т.п.) — ВЫПОЛНИ последнюю явную команду из списка выше или ответь "
            "по сути. Если в бэклоге НЕСКОЛЬКО явных команд — выполни ПОСЛЕДНЮЮ "
            "(LRU). НЕ здоровайся, НЕ переспрашивай, не пересказывай фразы "
            "дословно."
        )
        return "\n".join(lines)
