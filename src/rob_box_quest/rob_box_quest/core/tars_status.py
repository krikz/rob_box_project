"""Служебная информация и контекст для экрана ТАРС 1 (issue #3253, Ш3).

Чистая логика без ROS: собирает то, что quest_node реально слышит в графе
(``/voice/dj_mode``, ``/voice/speaker/result``, ``/voice/tts/provider_state``,
``/voice/llm_status``, ``/voice/wake_words``), в событие ``tars_status`` и решает, пора ли его слать (троттлинг).

Правило ADR-0018: нет источника — нет ключа в событии, клиент рисует прочерк.
llm и wake приходят latched-топиками dialogue_node (Ш3б); пока их никто не
опубликовал (нода не стартовала), ключей нет.
"""

from __future__ import annotations

import json
import time
from typing import Any, Callable, Optional

# Не чаще одного события в 1.5 с при изменениях.
MIN_INTERVAL_S = 1.5
# Повтор без изменений — чтобы клиент, подключившийся позже, получил статус.
HEARTBEAT_S = 10.0
# «Рядом» устаревает: речь была давно — человек мог уйти.
NEARBY_TTL_S = 120.0


def _clean(value: Any) -> Optional[str]:
    if not isinstance(value, str):
        return None
    value = value.strip()
    return value or None


def format_tts(provider: Optional[str], voice: Optional[str]) -> Optional[str]:
    """«provider · voice»; без провайдера значения нет (None → прочерк)."""
    p = _clean(provider)
    v = _clean(voice)
    if not p:
        return None
    return f"{p} · {v}" if v else p


def format_llm(payload: Any) -> Optional[str]:
    """``/voice/llm_status`` → «provider · model»; без провайдера None."""
    if not isinstance(payload, dict):
        return None
    return format_tts(payload.get("provider"), payload.get("model"))


def format_wake(payload: Any) -> Optional[str]:
    """``/voice/wake_words`` → «слово, слово»; пустой список — None."""
    if not isinstance(payload, dict) or not isinstance(payload.get("words"), list):
        return None
    words = [w for w in (_clean(x) for x in payload["words"]) if w]
    return ", ".join(words) if words else None


class TarsStatusAggregator:
    """Копит сигналы и отдаёт ``tars_status`` с троттлингом."""

    def __init__(self) -> None:
        self._dj_enabled: Optional[bool] = None  # None — dj_mode ещё не слышали
        self._dj_theme: Optional[str] = None
        self._llm: Optional[str] = None
        self._wake: Optional[str] = None
        self._nearby: Optional[str] = None
        self._nearby_at: float = 0.0
        self._last_sent: Optional[dict[str, Any]] = None
        self._last_sent_at: float = 0.0

    def note_dj_mode(self, payload: Any) -> None:
        """``/voice/dj_mode``: ``{enabled, theme?, ...}``."""
        if not isinstance(payload, dict) or not isinstance(payload.get("enabled"), bool):
            return
        self._dj_enabled = payload["enabled"]
        self._dj_theme = _clean(payload.get("theme")) if self._dj_enabled else None

    def note_llm(self, payload: Any) -> None:
        """``/voice/llm_status``: ``{provider, model?, ...}`` (latched)."""
        value = format_llm(payload)
        if value:
            self._llm = value

    def note_wake(self, payload: Any) -> None:
        """``/voice/wake_words``: ``{words: [...]}`` (latched)."""
        value = format_wake(payload)
        if value:
            self._wake = value

    def note_speaker(self, payload: Any, now: float) -> None:
        """``/voice/speaker/result``: узнанный по голосу человек.

        Неузнанный и «inconclusive» кадры не стирают узнанного: это «не знаю»,
        а не «ушёл».
        """
        if not isinstance(payload, dict) or payload.get("inconclusive"):
            return
        if payload.get("is_known") is not True:
            return
        name = _clean(payload.get("name"))
        if name:
            self._nearby = name
            self._nearby_at = now

    def _topic(self) -> Optional[str]:
        if self._dj_enabled is None:
            return None
        if not self._dj_enabled:
            return "обычный режим"
        return f"DJ · {self._dj_theme}" if self._dj_theme else "DJ"

    def snapshot(
        self, now: float, tts_provider: Optional[str], tts_voice: Optional[str]
    ) -> dict[str, Any]:
        """Поля, для которых есть данные. Остальных ключей нет (→ прочерк)."""
        snap: dict[str, Any] = {}
        if self._llm:
            snap["llm"] = self._llm
        if self._wake:
            snap["wake"] = self._wake
        tts = format_tts(tts_provider, tts_voice)
        if tts:
            snap["tts"] = tts
        topic = self._topic()
        if topic:
            snap["topic"] = topic
        if self._nearby and now - self._nearby_at <= NEARBY_TTL_S:
            snap["nearby"] = self._nearby
        return snap

    def _should_send(self, snap: dict[str, Any], now: float) -> bool:
        if self._last_sent is None:
            return bool(snap)  # нечего сообщать — молчим
        since = now - self._last_sent_at
        if snap != self._last_sent:
            return since >= MIN_INTERVAL_S
        return since >= HEARTBEAT_S

    def poll(
        self,
        now: float,
        ts_ms: int,
        tts_provider: Optional[str],
        tts_voice: Optional[str],
    ) -> Optional[dict[str, Any]]:
        """Событие к отправке или None. Вызывать по таймеру."""
        snap = self.snapshot(now, tts_provider, tts_voice)
        if not self._should_send(snap, now):
            return None
        self._last_sent = dict(snap)
        self._last_sent_at = now
        return {"type": "tars_status", **snap, "ts_ms": ts_ms}


class TarsStatusRelay:
    """Связка агрегатора с ROS/WS: колбэки подписок и таймера (ADR-0145 —
    вынесено из QuestNode, чтобы не раздувать класс)."""

    def __init__(
        self,
        get_tts: Callable[[], tuple[Optional[str], Optional[str]]],
        broadcast: Callable[[dict[str, Any]], None],
        log_debug: Callable[[str], None],
    ) -> None:
        self._agg = TarsStatusAggregator()
        self._get_tts = get_tts
        self._broadcast = broadcast
        self._log_debug = log_debug

    @staticmethod
    def _parse(msg: Any) -> Any:
        try:
            return json.loads(msg.data or "")
        except (json.JSONDecodeError, TypeError, AttributeError):
            return None

    def on_dj_mode(self, msg: Any) -> None:
        """Колбэк ``/voice/dj_mode``."""
        self._agg.note_dj_mode(self._parse(msg))

    def on_llm_status(self, msg: Any) -> None:
        """Колбэк ``/voice/llm_status``."""
        self._agg.note_llm(self._parse(msg))

    def on_wake_words(self, msg: Any) -> None:
        """Колбэк ``/voice/wake_words``."""
        self._agg.note_wake(self._parse(msg))

    def on_speaker_result(self, msg: Any) -> None:
        """Колбэк ``/voice/speaker/result``."""
        self._agg.note_speaker(self._parse(msg), time.monotonic())

    def on_timer(self) -> None:
        """Колбэк таймера 1 Гц: событие уходит только если пора."""
        provider, voice = self._get_tts()
        event = self._agg.poll(
            time.monotonic(), int(time.time() * 1000), provider, voice
        )
        if event is None:
            return
        try:
            self._broadcast(event)
        except Exception as e:  # noqa: BLE001
            self._log_debug(f"tars_status broadcast failed: {e}")
