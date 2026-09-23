"""Реплика ``speak_text`` не обгоняет исход регистрации (issue #2913).

Механизм «придержать и заменить ответ хода» (#2828 «вы разные люди?»,
#2888 «<Имя>, это ты?», #2908 «не расслышал») заменял только ИТОГОВЫЙ
текст хода. Реплика, которую LLM сказала тулом ``speak_text``, уходила в
TTS сразу — мимо механизма. Живой лог (E2E акт 2c, run 35923157899, n722)::

    .642 [mcp_server]      register_speaker {'name': 'Борис', ...}
    .659 [speaker_id_node] голос похож на уже известного 'Саша' ...
    .679 [mcp_server]      speak_text {'text': 'Здравствуй, Борис! Приятно познакомиться.'}
    .686 [tts_node]        TTS: 'Здравствуй, Борис! Приятно познакомиться.'
    .696 [dialogue_node]   переспрашиваю про личность: ... held_until_turn_end=True
    ...  [tts_node]        TTS: 'Твой голос очень похож на голос ... Вы разные люди ...'

:class:`TurnSpeechGate` — одна точка решения «звучит ли эта ``speak_text``»:

* в ходе уже придержан вопрос о личности / отказ (:meth:`mute`) — реплика
  хода не звучит, вместо неё прозвучит придержанный вопрос;
* в ходе отправлен ``register_speaker`` и исход ещё не пришёл
  (:meth:`registration_sent` без :meth:`registration_settled`) — реплика
  ждёт исхода не дольше ``timeout_s``, потом решается по нему;
* иначе (обычный ход, DJ-ход) — звучит сразу, без единого ``await``.

Ход без ``register_speaker`` и без придержанного вопроса не ждёт ничего:
латентность обычных и музыкальных ходов не меняется.

Состояние — на ход (:meth:`begin_turn`): реплика, поставленная в очередь
одним ходом, судится по состоянию СВОЕГО хода, даже если голосовой канал
выпустил её, когда уже начался следующий. Реплика, которая ждала исхода, а
её ход закончился, не звучит: её ход уже ответил (вопросом или ответом), а
приветствие поверх следующего хода было бы невпопад.

Модуль без ROS2 — тестируется напрямую.
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Callable, Optional

#: Имя тула, после которого исход приходит асинхронно (ack speaker_id_node).
REGISTER_TOOL = "register_speaker"

#: Сколько реплика ждёт исхода регистрации. speaker_id_node ждёт эмбеддинг
#: фразы до ``_REGISTER_UTTERANCE_WAIT_SEC`` = 1.5 с, плюс сама запись в БД.
#: Обычно ack приходит за десятки миллисекунд (run 35923157899: 17 мс).
REGISTER_OUTCOME_TIMEOUT_S: float = 2.5

_POLL_S: float = 0.01


class _TurnState:
    __slots__ = ("pending", "muted")

    def __init__(self) -> None:
        self.pending = 0
        self.muted = False


class TurnSpeechGate:
    """Решает, звучит ли ``speak_text`` хода (см. модуль)."""

    def __init__(
        self,
        timeout_s: float = REGISTER_OUTCOME_TIMEOUT_S,
        *,
        log: Optional[Callable[[str], None]] = None,
        clock: Callable[[], float] = time.monotonic,
        poll_s: float = _POLL_S,
    ) -> None:
        self._lock = threading.Lock()
        self._timeout_s = timeout_s
        self._log = log
        self._clock = clock
        self._poll_s = poll_s
        self._turn = _TurnState()

    # ----- сторона хода (dialogue_node) ---------------------------------

    def begin_turn(self) -> None:
        """Новый ход: своё состояние, прошлые решения его не касаются."""
        with self._lock:
            self._turn = _TurnState()

    def mute(self) -> None:
        """В ходе придержан вопрос/отказ — его речь заменяется им."""
        with self._lock:
            self._turn.muted = True

    def registration_sent(self) -> None:
        """``register_speaker`` ушёл, исход (ack) придёт асинхронно."""
        with self._lock:
            self._turn.pending += 1

    def registration_settled(self) -> None:
        """Исход регистрации известен (ack ``registered``/``register_error``
        обработан, либо тул не отправил запрос)."""
        with self._lock:
            self._turn.pending = max(0, self._turn.pending - 1)

    # ----- сторона голосового канала (SchedulerToolExecutor) -------------

    def ticket(self) -> _TurnState:
        """Ход, которому принадлежит реплика; брать при постановке в очередь."""
        with self._lock:
            return self._turn

    def _verdict(self, ticket: _TurnState, waited: bool) -> Optional[bool]:
        with self._lock:
            if ticket.muted:
                return False
            if ticket.pending == 0:
                return True
            if ticket is not self._turn:
                # Ход кончился, так и не узнав исход: его ответ уже выдан.
                return False if waited else None
            return None

    async def admit(self, ticket: _TurnState, text: str = "") -> bool:
        """``True`` — реплику озвучить, ``False`` — не озвучивать."""
        verdict = self._verdict(ticket, waited=False)
        if verdict is None:
            deadline = self._clock() + self._timeout_s
            while verdict is None and self._clock() < deadline:
                await asyncio.sleep(self._poll_s)
                verdict = self._verdict(ticket, waited=True)
            if verdict is None:
                self._emit(
                    "⏱ [issue #2913] исход register_speaker не пришёл за "
                    f"{self._timeout_s:.1f}с — озвучиваю speak_text хода: "
                    f"{text[:60]!r}"
                )
                verdict = True
        if not verdict:
            self._emit(
                "🔇 [issue #2913] speak_text хода не озвучен — ход отвечает "
                f"вопросом о личности / просьбой повторить: {text[:60]!r}"
            )
        return verdict

    def _emit(self, line: str) -> None:
        if self._log is not None:
            try:
                self._log(line)
            except Exception:  # noqa: BLE001 — лог не должен ронять речь
                pass


__all__ = [
    "REGISTER_OUTCOME_TIMEOUT_S",
    "REGISTER_TOOL",
    "TurnSpeechGate",
]
