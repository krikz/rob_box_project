"""SpeechReceipts - квитанция по ``/say`` строится кодом из ``SpeechEvent`` той же реплики (#3727, ADR-0150 PR-3/PR-12b).

Раньше ``say_handler`` отвечал «Озвучиваю» сразу после публикации, а владелец речи (``tts_node``) мог отказать
(``rejected{unsupported_script}``) - человек читал ложный успех. Теперь хендлер молчит, а итог приходит по
``speech_id``: ``finished`` -> «Озвучил», ``rejected{reason}`` -> фраза по причине, ``cancelled`` -> «перебили»,
тишина дольше ``TIMEOUT_S`` -> «не знаю, прозвучало ли». Тексты - ``phrases/ru.yaml`` (ключи ``speak_aloud.*``).
Чистый Python без ROS: узел отдаёт события и часы.
"""

from __future__ import annotations

import time
from typing import Callable, Dict, List, Optional, Tuple

#: Сколько ждать итога реплики; озвучка длинного текста идёт десятки секунд, но тишина дольше этого - потеря события.
TIMEOUT_S = 120.0

#: ``SpeechEvent.kind`` (uint8) -> завершающие виды; остальные (queued/started/progress/...) квитанцию не закрывают.
KIND_FINISHED, KIND_CANCELLED, KIND_REJECTED = 4, 5, 7

Receipt = Tuple[int, str]  # (chat_id, текст для чата)


class SpeechReceipts:
    """Реплики, итог которых ждёт чат: ``speech_id -> (chat_id, срок)``."""

    def __init__(self, clock: Callable[[], float] = time.monotonic, timeout_s: float = TIMEOUT_S) -> None:
        self._clock = clock
        self._timeout_s = timeout_s
        self._pending: Dict[str, Tuple[int, float]] = {}

    def expect(self, speech_id: str, chat_id: int) -> None:
        self._pending[speech_id] = (int(chat_id), self._clock() + self._timeout_s)

    def on_event(self, kind: int, speech_id: str, reason: str = "", seconds: float = 0.0) -> Optional[Receipt]:
        """Квитанция для чата или ``None`` (чужая реплика / не итоговое событие)."""
        if kind not in (KIND_FINISHED, KIND_CANCELLED, KIND_REJECTED) or speech_id not in self._pending:
            return None
        chat_id, _ = self._pending.pop(speech_id)
        return chat_id, _phrase(kind, reason, seconds)

    def expired(self) -> List[Receipt]:
        """Реплики без итога дольше срока: честное «не знаю», а не вечная тишина."""
        now = self._clock()
        gone = [sid for sid, (_, until) in self._pending.items() if now >= until]
        return [(self._pending.pop(sid)[0], _text("speak_aloud.timeout")) for sid in gone]


def _text(key: str, **slots: object) -> str:
    from rob_box_dialog import phrases  # noqa: PLC0415 - пакет диалога нужен только квитанции

    return phrases.text(key, **slots)


def _phrase(kind: int, reason: str, seconds: float) -> str:
    from rob_box_dialog import phrases  # noqa: PLC0415

    if kind == KIND_FINISHED:
        return phrases.text("speak_aloud.done", seconds=f"{seconds:.0f}")
    if kind == KIND_CANCELLED:
        return phrases.text("speak_aloud.cancelled")
    key = phrases.first_key(f"speak_aloud.rejected.{reason}", "speak_aloud.rejected")
    return phrases.text(key or "speak_aloud.rejected")
