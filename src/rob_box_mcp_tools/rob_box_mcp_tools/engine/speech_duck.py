"""Приглушение музыки под речь робота (ADR-0150 §9.1 О8 (а), В16, PR-7d #3701).

``tts_node`` — владелец речи — публикует ``SpeechEvent`` на каждом переходе реплики. Здесь — один писатель ручки
``duck`` мастер-шины (``masterfilter.scd``, узел 999): пока звучит хоть одна реплика, ``duck 1`` — музыка уходит на
``duckDb`` за ``duckAttack``; последняя реплика закончилась — ``duck 0``, возврат за ``duckRelease``. Огибающую
ведёт scsynth (``Slew``), отсюда только переключение 0/1 — без потока ``/n_set``.

Какая речь считается: ``started`` у ``SpeechEvent`` бывает только перед локальным воспроизведением в динамики
(``tts_node._sap_play_local``); реплики в шлем оператора (``sink=headset``) идут мимо владельца и событий не дают.
Поэтому поле ``sink`` в событии не нужно: каждый ``started`` — речь из динамиков (Личность, ТАРС в динамики,
Telegram ``say_aloud``).

Несколько реплик одновременно — множество ``speech_id``: ``duck 0`` только когда оно пустое. Финал потерялся
(нода упала, сообщение не дошло) — реплика снимается по сроку ``duration_ms + grace_s`` в :meth:`SpeechDuck.tick`,
музыка не остаётся приглушённой навсегда. Модуль без ROS: тесты гоняют его на фейковой отправке.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Dict, Mapping

from rob_box_music import knowledge as kn

from .renardo_adapter import MASTER_NODE

_LOG = logging.getLogger(__name__)

#: Запас к длительности реплики, после которого потерянный финал снимает приглушение, с.
GRACE_S = 3.0
#: Срок реплики без известной длительности (``duration_ms = 0``), с.
UNKNOWN_DURATION_S = 60.0
#: Финалы реплики (имена ``SpeechEvent.KIND_*`` в нижнем регистре).
_ENDS = ("finished", "cancelled", "rejected")


def duck_controls(depth_db: float, attack_ms: float, release_ms: float) -> Dict[str, float]:
    """Ручки огибающей для ``/n_set 999``: глубина только вниз, времена не меньше 1 мс."""
    return {"duckDb": min(0.0, float(depth_db)), "duckAttack": max(1.0, float(attack_ms)) / 1000.0,
            "duckRelease": max(1.0, float(release_ms)) / 1000.0}


class SpeechDuck:
    """Счётчик звучащих реплик → ``/n_set 999 duck 0|1`` с ручками огибающей.

    Args:
        send_osc: ``RenardoRuntime.send_osc`` или фейк.
        controls: ``duckDb``/``duckAttack``/``duckRelease`` (:func:`duck_controls`); по умолчанию —
            :data:`rob_box_music.knowledge.SPEECH_DUCK`.
        clock: монотонные часы для срока потерянных финалов.
    """

    def __init__(self, send_osc: Callable[..., None], controls: Mapping[str, float] = kn.SPEECH_DUCK, *,
                 logger: Any = None, clock: Callable[[], float] = time.monotonic, grace_s: float = GRACE_S) -> None:
        self._send = send_osc
        self._controls = dict(controls)
        self._log = logger or _LOG
        self._clock = clock
        self._grace_s = grace_s
        self._lock = threading.Lock()
        self._active: Dict[str, float] = {}  # speech_id → срок, после которого финал считается потерянным
        self._ducked = False

    @property
    def ducked(self) -> bool:
        """Последнее отправленное состояние ручки ``duck``."""
        return self._ducked

    def on_event(self, kind: str, speech_id: str, duration_ms: int = 0) -> None:
        """``SpeechEvent``: ``started`` — реплика звучит; ``finished``/``cancelled``/``rejected`` — нет."""
        if not speech_id:
            return
        with self._lock:
            if kind == "started":
                span = duration_ms / 1000.0 if duration_ms > 0 else UNKNOWN_DURATION_S
                self._active[speech_id] = self._clock() + span + self._grace_s
            elif kind in _ENDS:
                self._active.pop(speech_id, None)
            else:
                return
        self._apply(f"{kind} speech_id={speech_id[:8]}")

    def tick(self) -> None:
        """Снять реплики, чей финал не пришёл к сроку (громко: это сбой владельца речи)."""
        now = self._clock()
        with self._lock:
            lost = [sid for sid, due in self._active.items() if due <= now]
            for sid in lost:
                del self._active[sid]
        if lost:
            self._log.warning(f"⚠️ [duck] финал реплики не пришёл к сроку: {', '.join(s[:8] for s in lost)} — снято")
            self._apply("lost")

    def _apply(self, why: str) -> None:
        with self._lock:
            want = bool(self._active)
            if want == self._ducked:
                return
            self._ducked = want
            count = len(self._active)
        args = ["duck", 1.0 if want else 0.0]
        for name, value in self._controls.items():  # ручки — с каждым переключением: узел мог пересоздаться
            args += [name, float(value)]
        try:
            self._send("/n_set", MASTER_NODE, *args)
        except Exception as exc:  # noqa: BLE001 — SC недоступен: речь важнее, музыку не роняем
            self._log.warning(f"⚠️ [duck] /n_set не отправлен: {exc}")
            with self._lock:
                self._ducked = not want  # следующее событие попробует снова
            return
        self._log.info(f"🔉 [duck] {'on' if want else 'off'} ({why}, реплик={count}) "
                       + " ".join(f"{k}={v:g}" for k, v in self._controls.items()))


__all__ = ["SpeechDuck", "duck_controls"]
