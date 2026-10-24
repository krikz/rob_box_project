"""Стадии хода диалога v1 и рефлексы на них (ADR-0150 §2.3, §5.5.3; ADR-0148 «код решает, LLM говорит»).

``addressed -> thinking -> executing -> speaking -> done | failed | timeout``. На каждой стадии :class:`TurnStages`
публикует ``TurnEvent`` (его читает кольцо LED, метрики хода, e2e) и один ``ExpressionRequest`` по
``rob_box_dialog.expression.reflex()`` - лицо и earcon; LLM в этом не участвует. Эмоция команды (v1 - аргумент
``animation`` у ``speak_text`` / ``play_animation``) красит ``speaking`` и ``done`` через ``expression.merge()``, на
``failed`` рефлекс побеждает.

Откуда стадии (источники - события, а не пересказ LLM):

* ``addressed`` - ``InterruptRequest`` с вейком (ранний сигнал ~0,4-0,6 с от начала фразы, место бывшего «булька»
  ``stt_node``) либо допуск реплики (``AddressStep``: Telegram и фразы без раннего сигнала); один раз на фразу;
* ``thinking`` - перед вызовом LLM; ``executing{cls}`` - перед ``/mcp/execute`` (класс из ``knowledge.TOOL_CLASSES``);
* ``speaking`` - ``SpeechEvent started`` речи личности; ``done`` - тело хода закончено и речь дочитана;
* ``failed`` - тул вернул ошибку, LLM упала, речь отклонена; ``timeout`` - речь, которую ход обещал, не началась.

Без ROS: публикаторы, часы и нормализатор имён сцен приходят снаружи. Вызывается из потоков ROS и из цикла
asyncio, поэтому состояние под одним замком. Выход рефлексов - :mod:`.turn_reflexes`, тулы - :mod:`.turn_tools`.
"""

from __future__ import annotations

import functools
import threading
import time
import uuid
from typing import Any, Callable, Dict, Optional

from rob_box_dialog import expression as ex
from rob_box_dialog import knowledge as kn

from .turn_reflexes import ReflexOut, Turn
from .turn_tools import ToolWatch, error_reason, scene_for_animation, show_seconds

#: Сколько ждать речь, обещанную ходом, после конца тела хода; не началась - ``timeout``.
SPEECH_GRACE_S = 10.0
#: Сколько ждать речь у хода, закрытого кодом (команда «замолчи», медиароутер): она может прозвучать, а может нет.
HANDLED_SETTLE_S = 1.5
#: Реплика в очереди без терминального события дольше этого срока считается потерянной (самая длинная речь - минуты).
SPEECH_ID_TTL_S = 300.0
#: Запас сверх ``ADDRESS_WAIT_S``, после которого «слушаю» без реплики закрывается (STT оборвался, голое «робот»).
HOLD_MARGIN_S = 4.0
#: Сцена и срок, на которые возвращается лицо, если фраза с вейком оказалась не для робота.
CALM_SCENE, CALM_TTL_MS = "idle", 100
#: ``SpeechEvent.kind`` (rob_box_dialog_msgs/SpeechEvent).
SPEECH_QUEUED, SPEECH_STARTED, SPEECH_FINISHED, SPEECH_CANCELLED, SPEECH_REJECTED = 0, 2, 4, 5, 7
SPEECH_DONE_KINDS = (SPEECH_FINISHED, SPEECH_CANCELLED, SPEECH_REJECTED)


def _locked(method):
    """Метод под замком хода (как ``with self._lock`` в остальных)."""
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return wrapper


#: Заглушка вместо открытого хода: удержание без хода никого не трогает.
_NO_TURN = Turn("", None, "", 0.0, 0)


class SpeechLedger:
    """Реплики личности в очереди (по ``speech_id``) и дочитана ли речь, которую обещал ход.

    Потерянное терминальное событие не держит ход вечно: запись старше ``SPEECH_ID_TTL_S`` не считается.
    """

    def __init__(self) -> None:
        self._open: Dict[str, float] = {}

    def note(self, kind: int, speech_id: str, now: float) -> None:
        if kind == SPEECH_QUEUED:
            self._open[speech_id] = now
        elif kind in SPEECH_DONE_KINDS:
            self._open.pop(speech_id, None)

    def pending(self, now: float) -> int:
        self._open = {sid: at for sid, at in self._open.items() if now - at < SPEECH_ID_TTL_S}
        return len(self._open)

    def verdict(self, turn: Turn, now: float) -> str:
        """``wait`` | ``done`` | ``timeout``: речь дочитана / ещё идёт или не началась."""
        if turn.expects_speech:
            if turn.speech_started:
                return "wait" if self.pending(now) else "done"
            return "timeout" if now - (turn.body_done_at or now) >= SPEECH_GRACE_S else "wait"
        return "wait" if (self.pending(now) or now < turn.settle_until) else "done"


class TurnStages:
    """Стадии хода личности и рефлексы на них.

    ``express(**поля)`` и ``turn_event(**поля)`` - привязанные к ноде ``expression_client.express`` /
    ``expression_client.turn_event``; ``epoch()`` - текущая эпоха сессии; ``normalize(имя)`` - имя анимации тула в имя
    сцены лица (``""`` - неизвестно).
    """

    def __init__(
        self,
        express: Callable[..., Any],
        turn_event: Callable[..., Any],
        *,
        agent: str = "personality",
        epoch: Callable[[], int] = lambda: 0,
        normalize: Callable[[str], str] = lambda name: name,
        clock: Callable[[], float] = time.monotonic,
        log: Callable[[str], None] = lambda text: None,
    ) -> None:
        self._agent, self._clock, self._log = agent, clock, log
        self._out = ReflexOut(express, turn_event, agent, epoch, clock, log)
        self._tools = ToolWatch(self._out, normalize)
        self._speech = SpeechLedger()
        self._lock = threading.RLock()
        self._turn: Optional[Turn] = None
        self._count = 0

    # -- открытие и закрытие хода ---------------------------------------------------------------------------------
    def turn_id_for(self, uid: Optional[str]) -> Optional[str]:
        """``turn_id`` открытого хода этой фразы (или ``None``): ход LLM берёт тот же id, что у ``addressed``.

        Фраза без id (Telegram, оператор, инъекция): её ход - открытый ``addressed`` тоже без id; ретрай гуарда
        (стадия уже дальше) своего id не получает - он продолжает ход.
        """
        with self._lock:
            turn = self._live()
            return turn.turn_id if (turn is not None and turn.takes_phrase(uid)) else None

    def addressed(self, uid: Optional[str], source: str = "respeaker") -> bool:
        """Фраза адресована роботу: открыть ход и дать рефлекс ``addressed``; ``False`` - ход этой фразы уже открыт.

        Зовут дважды: ранний сигнал (``InterruptRequest``) и допуск реплики - второй вызов того же ``uid`` (или без
        ``uid`` в пределах ``SAME_PHRASE_S``) ничего не делает, кроме привязки ``uid``.
        """
        with self._lock:
            now = self._clock()
            turn = self._turn
            if turn is not None and turn.same_phrase(uid, source, now):
                turn.uid = turn.uid or uid
                return False
            if turn is not None and not turn.terminal:
                self._out.close(turn, "skipped", "superseded")
            self._count += 1
            self._turn = turn = Turn(uuid.uuid4().hex, uid or None, source, now, self._count)
            self._out.enter(turn, "addressed")
            return True

    def dropped(self, uid: Optional[str], reason: str) -> None:
        """Фраза с вейком оказалась не для робота (тишина, пустая после вейка): ход закрыт, лицо успокоено."""
        with self._lock:
            turn = self._live()
            if turn is None or (uid and turn.uid and turn.uid != uid) or turn.stage != "addressed":
                return
            self._calm(turn, reason)

    def handled(self, uid: Optional[str]) -> None:
        """Реплику закрыл код без LLM («замолчи», медиароутер): речь, если будет, ход дочитает, иначе ``done``."""
        with self._lock:
            turn = self._live()
            if turn is None or (uid and turn.uid and turn.uid != uid):
                return
            now = self._clock()
            turn.end_body(now)
            turn.settle_until = now + HANDLED_SETTLE_S
            self._maybe_complete(turn)

    @_locked
    def hold(self, reopen: Optional[bool] = None) -> None:
        """Медиароутер ищет мелодию (``reopen=None``), итог хода не решать; нашёл - ``False``, промах - ``True`` (#3794)."""
        (self._live() or _NO_TURN).hold(reopen)  # открытого хода нет - удерживать нечего

    def finish(self, turn_id: Optional[str], ok: bool = True, reason: str = "", cancelled: bool = False) -> None:
        """Тело хода закончено (``_run_turn``): исход решают результаты тулов и LLM; речь ход дочитает."""
        with self._lock:
            turn = self._live()
            if turn is None or (turn_id and turn.turn_id != turn_id):
                return
            if cancelled:
                self._out.close(turn, "skipped", "cancelled")
                return
            if not ok:
                turn.failure = reason or "error"
            turn.end_body(self._clock())
            self._maybe_complete(turn)

    def tick(self) -> None:
        """Раз в секунду: закрыть ход, которому не дождались речи, и «слушаю» без реплики (голое «робот», обрыв STT)."""
        with self._lock:
            turn = self._live()
            if turn is None:
                return
            if turn.body_done_at is not None:
                self._maybe_complete(turn)
            elif turn.stage == "addressed" and self._clock() - turn.began > kn.ADDRESS_WAIT_S + HOLD_MARGIN_S:
                self._calm(turn, "no_utterance")

    # -- события хода --------------------------------------------------------------------------------------------
    def thinking(self, provider: str = "") -> None:
        """Перед вызовом LLM."""
        with self._lock:
            turn = self._live()
            if turn is not None:
                turn.llm_calls += 1
                turn.llm_provider = provider or turn.llm_provider
                self._out.enter(turn, "thinking")

    def tool_call(self, name: str, args: Optional[Dict[str, Any]] = None) -> None:
        """Перед ``/mcp/execute``: ``executing{cls}``; ``speak_text`` обещает речь, ``animation`` красит ход."""
        with self._lock:
            turn = self._live()
            if turn is not None:
                self._tools.call(turn, name, args or {})

    def tool_result(self, name: str, is_error: bool, reason: str = "") -> None:
        """Итог тула: ошибка действия - причина ``failed``; успех того же тула позже её снимает."""
        with self._lock:
            turn = self._live()
            if turn is not None:
                self._tools.result(turn, name, is_error, reason)

    def expect_speech(self) -> None:
        """Ход ответит речью не через ``speak_text`` (авто-озвучка ответа, прямая фраза): её нужно дождаться."""
        with self._lock:
            turn = self._live()
            if turn is not None:
                turn.expects_speech = True

    def on_speech(self, kind: int, agent: str, reason: str = "", speech_id: str = "") -> None:
        """``SpeechEvent`` речи личности: ``started`` -> ``speaking``; конец очереди может закрыть ход."""
        if agent != self._agent:
            return
        with self._lock:
            self._speech.note(kind, speech_id, self._clock())
            turn = self._live()
            if turn is None:
                return
            if kind == SPEECH_STARTED and not turn.speech_started:
                turn.speech_started = True
                self._out.enter(turn, "speaking")
            elif kind == SPEECH_REJECTED:
                turn.errors["speech"] = f"rejected:{reason}"[:60]
            if turn.body_done_at is not None:
                self._maybe_complete(turn)

    def on_expression_event(self, executor: str, turn_id: str, started: bool, reason: str) -> str:
        """Ответ исполнителя: пропуск виден в логе (молчаливых пропусков рефлексов нет); вернуть строку лога."""
        with self._lock:
            return self._out.answer(self._turn, executor, turn_id, started, reason)

    # -- внутреннее ------------------------------------------------------------------------------------------------
    def _live(self) -> Optional[Turn]:
        turn = self._turn
        return None if (turn is None or turn.terminal) else turn

    def _calm(self, turn: Turn, reason: str) -> None:
        """Фраза не состоялась: ход закрыт ``skipped``, лицо возвращено в покой."""
        self._out.close(turn, "skipped", reason)
        self._out.send(turn, ex.ExpressionPlan(emotion=CALM_SCENE, ttl_ms=CALM_TTL_MS))

    def _maybe_complete(self, turn: Turn) -> None:
        """Тело хода закончено: ``failed`` - сразу; иначе ``done`` после речи (или ``timeout``, если речь не началась)."""
        if turn.terminal or turn.body_done_at is None:
            return
        failure = turn.failure or next(iter(turn.errors.values()), "")
        verdict = "failed" if failure else self._speech.verdict(turn, self._clock())
        if verdict == "wait":
            return
        reason = failure or ("speech_not_started" if verdict == "timeout" else "")
        self._out.enter(turn, verdict, status="finished" if verdict == "done" else "failed", reason=reason)
        turn.terminal = True
        self._log(f"[turn] {verdict} turn={turn.turn_id[:8]} reason={reason or '-'} "
                  f"latency_ms={int((self._clock() - turn.began) * 1000)}")


__all__ = ["TurnStages", "error_reason", "scene_for_animation", "show_seconds"]
