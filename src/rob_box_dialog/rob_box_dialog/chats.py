"""Телеграм-агент на общем движке: сессия, история и ход на каждый чат (ADR-0150 §9.2, решение В10; PR-12a #3706).

Чат — не вторая голова голосового диалога: у него своя :class:`~rob_box_dialog.session.Session`
(``agent="telegram"``, ``session_id="telegram:<chat_id>"``), своё окно ходов и свой :class:`TurnEngine`. Адресация —
та же таблица (:func:`rob_box_dialog.address.decide`): текст в чате адресован всегда, «Клод …» — координатору
(``NotAddressed(coordinator)``). Команды роботу идут к тем же владельцам (исполнитель хоста), а фразы — и слова модели,
и фраза кода об исходе по событию — уходят текстом в чат (:class:`ChatPorts`, ``DialogOutput``), а не в динамики.

Озвучка из чата (PR-12b, §9.2.3): ``say_aloud`` — действие класса ``action.speak_aloud``. Тул ждёт только начала
реплики (``pending`` с ``event_ref=speech_id``), а «Озвучил (12 с)» приходит позже событием ``SpeechEvent finished``
(:meth:`ChatTurns.on_speech` → :meth:`ChatPorts.late`): дедлайн хода рассказ не ждёт, «отправил» без события нет.
Тишина — кадр Личности и чата не касается (О2); ``OperatorHold`` — отказ ``operator_hold`` кодом (:meth:`ChatPorts.execute`).

Чистый модуль: транспорт (LLM, исполнитель, публикация ``DialogOutput``) — снаружи, у ROS-хоста.
"""

from __future__ import annotations

import logging
import time
import uuid
from typing import Any, Callable, Dict, Optional, Sequence

from rob_box_core import speech_client
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import SOURCE_TELEGRAM, Decision, decide
from rob_box_dialog.backlog import Heard
from rob_box_dialog.execute import OwnerEvent
from rob_box_dialog.respond import ActionResult, phrase_from_result

_LOG = logging.getLogger(__name__)
#: Строка лога телеграм-агента.
LOG_MARK = "💬 [tg-agent]"
#: ``DialogOutput.channel_ref`` чата: ``telegram:<chat_id>``.
CHANNEL_PREFIX = "telegram:"
#: ``Send(channel_ref, text, speech_id)`` — публикация ``DialogOutput`` хостом.
Send = Callable[[str, str, str], None]


def channel_ref(chat_id: int) -> str:
    return f"{CHANNEL_PREFIX}{int(chat_id)}"


def chat_of(ref: str) -> Optional[int]:
    """``telegram:<chat_id>`` → ``chat_id``; чужой канал или мусор — ``None``."""
    if not isinstance(ref, str) or not ref.startswith(CHANNEL_PREFIX):
        return None
    try:
        return int(ref[len(CHANNEL_PREFIX):])
    except ValueError:
        return None


def _result_term_s(duration_ms: Any) -> float:
    """Срок позднего итога реплики: длительность + запас, не больше потолка; длительность неизвестна - потолок."""
    seconds = float(duration_ms or 0) / 1000.0
    if seconds <= 0:
        return kn.SPEECH_RESULT_CEILING_S
    return min(kn.SPEECH_RESULT_CEILING_S, seconds + kn.SPEECH_RESULT_SLACK_S)


class ChatPorts:
    """Порты хода одного чата: LLM и исполнение — общие порты хоста (``base``), речь — текст в чат.

    Стадии хода лицо и кольцо не трогают (чат — не разговор с роботом вслух), отменять в чате нечего, выступление
    личности стоп из чата не рвёт; стоп движения — тот же интент, что у голоса."""

    def __init__(self, base: Any, chat_id: int, send: Send, log: Callable[[str], None] = _LOG.info,
                 hold: Callable[[], bool] = lambda: False) -> None:
        self._base = base
        self._ref = channel_ref(chat_id)
        self._send = send
        self._log = log
        self._hold = hold  # держит ли оператор Личность (кадр ``OperatorHold`` её сессии)

    def ask(self, messages: Sequence[Any], tools: Sequence[Any], timeout_s: float) -> Any:
        return self._base.ask(messages, tools, timeout_s)

    async def execute(self, actions: Sequence[Any], snapshot: Any, turn_id: str) -> Any:
        """Действия владельцам. ``say_aloud`` при ``OperatorHold`` — ``rejected{operator_hold}`` без запроса (§9.2.3);
        пользовательская тишина Личности сюда не входит: её кадр в сессии Личности, не чата (О2)."""
        held = {i for i, a in enumerate(actions) if a.tool == kn.ALOUD_TOOL} if self._hold() else set()
        if not held:
            return await self._base.execute(actions, snapshot, turn_id)
        rest = [a for i, a in enumerate(actions) if i not in held]
        done = iter(await self._base.execute(rest, snapshot, turn_id) if rest else [])
        self._log(f"{LOG_MARK} {self._ref} say_aloud отклонён: оператор держит робота (operator_hold)")
        return [self._held(a, snapshot, f"{turn_id}/{i}") if i in held else next(done) for i, a in enumerate(actions)]

    @staticmethod
    def _held(call: Any, snapshot: Any, request_id: str) -> ActionResult:
        return ActionResult(request_id, snapshot.epoch, snapshot.agent, call.tool, kn.TOOL_CLASSES[call.tool],
                            "rejected", "operator_hold")

    def late(self, result: ActionResult) -> None:
        """Поздний итог действия (``Executor.on_late``): фраза кода по событию владельца — в чат («Озвучил (12 с)»)."""
        said = phrase_from_result(result)
        if said is None:
            return
        self._send(self._ref, said.text, uuid.uuid4().hex)
        self._log(f"{LOG_MARK} → {self._ref} итог {result.tool} {result.status} {result.reason or '-'}")

    def say(self, turn_id: str, epoch: int, text: str, voice: Optional[str] = None,
            emotion: Optional[str] = None) -> str:
        speech_id = uuid.uuid4().hex
        self._send(self._ref, text, speech_id)
        self._log(f"{LOG_MARK} → {self._ref} turn={turn_id} epoch={epoch} chars={len(text)}")
        return speech_id

    def stage(self, turn_id: str, epoch: int, stage: str, status: str, reason: str = "", cls: Optional[str] = None,
              llm_calls: int = 0, provider: str = "", source: str = "") -> None:
        if status != "started":
            self._log(f"{LOG_MARK} {self._ref} turn={turn_id} stage={stage} status={status} reason={reason or '-'} "
                      f"llm_calls={llm_calls} provider={provider or '-'}")

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested") -> None:
        return None

    def stop_perform(self, how: str) -> bool:
        return False

    def intent(self, turn_id: str, epoch: int, kind: str) -> None:
        self._base.intent(turn_id, epoch, kind)


class ChatTurns:
    """Ходы телеграм-агента: движок на чат (создаётся при первом сообщении чата), ходы одного чата — по очереди."""

    def __init__(self, make_engine: Callable[[int], Any], wake_words: Sequence[str] = (),
                 log: Callable[[str], None] = _LOG.info, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._due: Dict[str, float] = {}  # ``speech_id`` обещания -> срок позднего итога
        self._make = make_engine
        self._wake = tuple(wake_words)
        self._log = log
        self._engines: Dict[int, Any] = {}
        self._locks: Dict[int, Any] = {}
        self._executors: list = []

    def decide(self, chat_id: int, text: str, now: float) -> Decision:
        """Адресация реплики чата той же таблицей, что у голоса (``source=telegram``)."""
        decision = decide(Heard(text=(text or "").strip(), t=now, source=SOURCE_TELEGRAM), None, self._wake)
        self._log(f"{LOG_MARK} {channel_ref(chat_id)} decision={type(decision).__name__} "
                  f"reason={getattr(decision, 'reason', '-')}")
        return decision

    def engine(self, chat_id: int) -> Any:
        """Движок чата; первый раз — новая сессия ``telegram:<chat_id>`` со своей историей."""
        chat_id = int(chat_id)
        if chat_id not in self._engines:
            self._engines[chat_id] = self._make(chat_id)
            self._log(f"{LOG_MARK} новая сессия {channel_ref(chat_id)} (чатов: {len(self._engines)})")
        return self._engines[chat_id]

    def adopt(self, executor: Any) -> None:
        """Исполнитель чата, чьи ``pending``-обещания ждут событий владельца (хост зовёт при создании движка)."""
        self._executors.append(executor)

    def on_speech(self, event: Any) -> bool:
        """``SpeechEvent`` реплики агента ``telegram``: конец реплики — итог ``say_aloud`` тому исполнителю, что его
        ждёт; чужая реплика (``/say``, квитанция ``telegram_node``) и не итоговые виды — ``False``."""
        status, reason, seconds = speech_client.event_outcome(event)
        ref = str(getattr(event, "speech_id", "") or "")
        if status == "pending":
            if any(ex.expects(ref) for ex in self._executors):  # звучит: срок итога - длительность реплики + запас
                self._due[ref] = self._clock() + _result_term_s(getattr(event, "duration_ms", 0))
            return False
        self._due.pop(ref, None)
        for executor in self._executors:
            if executor.expects(ref):
                executor.on_event(OwnerEvent(ref, status, reason, {"seconds": seconds}))
                return True
        return False

    def tick(self) -> None:
        """Таймер хоста: обещание без итога дольше срока - поздний ``timeout`` («Не знаю, договорил ли»). Срок
        обещания без ``started`` - потолок; ``started`` с длительностью сокращает его (:meth:`on_speech`)."""
        now = self._clock()
        for executor in self._executors:
            for ref in executor.promised():
                if self._due.setdefault(ref, now + _result_term_s(0)) <= now:
                    self._due.pop(ref, None)
                    executor.expire(ref)

    async def run(self, chat_id: int, addressed: Any) -> Any:
        """Ход чата; новый ход того же чата ждёт конца текущего (другие чаты — не ждут). Упавший ход — громкая
        строка лога и ``None`` (задачу ``run_coroutine_threadsafe`` никто не ждёт: иначе ошибка пропала бы молча)."""
        import asyncio  # noqa: PLC0415 — asyncio тянет socket: импорт пакета чистый (test_purity)

        lock = self._locks.setdefault(int(chat_id), asyncio.Lock())
        async with lock:
            try:
                return await self.engine(chat_id).run(addressed)
            except Exception as exc:  # noqa: BLE001 — упавший ход чата не роняет ноду, но громко
                self._log(f"{LOG_MARK} {channel_ref(chat_id)} ход упал: {type(exc).__name__}: {exc}")
                return None


__all__ = ["CHANNEL_PREFIX", "ChatPorts", "ChatTurns", "LOG_MARK", "channel_ref", "chat_of"]
