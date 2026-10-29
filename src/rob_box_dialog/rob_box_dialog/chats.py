"""Телеграм-агент на общем движке: сессия, история и ход на каждый чат (ADR-0150 §9.2, решение В10; PR-12a).

Чат — не вторая голова голосового диалога: у него своя :class:`~rob_box_dialog.session.Session`
(``agent="telegram"``, ``session_id="telegram:<chat_id>"``), своё окно ходов и свой :class:`TurnEngine`. Адресация —
та же таблица (:func:`rob_box_dialog.address.decide`): текст в чате адресован всегда, «Клод …» — координатору
(``NotAddressed(coordinator)``). Команды роботу идут к тем же владельцам (исполнитель хоста), а фразы — и слова модели,
и фраза кода об исходе по событию — уходят текстом в чат (:class:`ChatPorts`, ``DialogOutput``), а не в динамики.

Пульт робота (решение Шифу 28.10): просьбу действия на роботе исполняет РОБОТ от своего имени — выступление
(``perform``: «спой», рэп, сказка) уходит хосту выступления Личности через пульт хоста (:class:`Robot`), ответ лицом и
сцены ``play_animation`` — на лицо робота; служебного текста агента там нет. В чат — фраза кода из результата («Пою на
роботе: «…»», «Показываю на роботе: радость.», отказ — «Не вышло.», лицо в тишине Личности — «Не показал…»), а не
текст песни. «Спой» из чата решает тот же путь пения, что у голоса (хост даёт движку чата тот же
``SingRouteHost`` — тот же выбиратель и доска провайдеров, строка лога с меткой чата; музыку робота —
:meth:`ChatTurns.set_music`); конец выступления закрывает его запись и в окне чата (:meth:`ChatTurns.performance_ended`).

Озвучка из чата (PR-12b, §9.2.3): ``say_aloud`` — действие класса ``action.speak_aloud``. Тул ждёт только начала
реплики (``pending`` с ``event_ref=speech_id``), а «Озвучил (12 с)» приходит позже событием ``SpeechEvent finished``
(:meth:`ChatTurns.on_speech` → :meth:`ChatPorts.late`): дедлайн хода рассказ не ждёт, «отправил» без события нет.
Тишина — кадр Личности и чата не касается (О2); ``OperatorHold`` — отказ ``operator_hold`` кодом (:meth:`ChatPorts.execute`).

Чистый модуль: транспорт (LLM, исполнитель, публикация ``DialogOutput``, пульт робота) — снаружи, у ROS-хоста.
"""

from __future__ import annotations

import logging
import time
import uuid
from typing import Any, Callable, Dict, Mapping, Optional, Protocol, Sequence

from rob_box_core import speech_client
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import HELD, SOURCE_TELEGRAM, Decision, decide
from rob_box_dialog.backlog import Heard
from rob_box_dialog.execute import OwnerEvent
from rob_box_dialog.expression import face_label
from rob_box_dialog.respond import ActionResult, phrase_from_result
from rob_box_dialog.turn import Shown

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


class Robot(Protocol):
    """Пульт робота для агента чата: то, что чат исполняет РОБОТОМ. Хост (``dialog_node.RobotRemote``) отдаёт
    это портам Личности — её эпоха, её голос и лицо: со стороны робот делает это сам."""

    def perform(self, turn_id: str, perf: Any, edit: bool) -> Optional[Mapping[str, Any]]:
        """Выступление хосту выступления робота; исход старта (``status``: ``pending``/``done``/``rejected``) или
        ``None`` — хоста выступления нет."""

    def show(self, turn_id: str, expression: Any) -> str:
        """Лицо — на лицо робота (без стадии хода и earcon'а). Исход показа: «» — лицо ушло лицу робота, иначе причина,
        по которой его там не будет (``knowledge.REMOTE_SHOW_REFUSALS``: ``silenced`` — у Личности тишина)."""


def quote(text: str) -> str:
    """Строка выступления цитатой для чата: пробелы схлопнуты, не длиннее ``REMOTE_QUOTE_MAX_CHARS`` (дальше «…»)."""
    line = " ".join(str(text or "").split())
    limit = kn.REMOTE_QUOTE_MAX_CHARS
    return line if len(line) <= limit else line[:limit - 1].rstrip() + "…"


def perform_phrase(perf: Any, outcome: Mapping[str, Any]) -> Optional[str]:
    """Фраза кода в чат об исходе выступления на роботе (ADR-0148: из ответа его хоста, а не из слов модели): старт
    (``pending``) — по виду выступления с цитатой первой строки, правка идущего (``done``: хост влил куплеты) —
    «добавил», «добавь бит» без куплетов (``backing``) — «добавил бит». Отказ — ``None``: его фразу говорит ход
    (``turn.hand_perform``: «Не вышло.»)."""
    status = str(outcome.get("status") or "")
    segments = tuple(perf.segments or ())
    if status == "done" and not segments:
        return phrases.text(kn.REMOTE_PERFORM_BACKING_PHRASE) if outcome.get("backing") else None
    key = {"pending": kn.REMOTE_PERFORM_PHRASES.get(perf.kind), "done": kn.REMOTE_PERFORM_EDIT_PHRASE}.get(status)
    if key is None or not segments:
        return None
    return phrases.text(key, line=quote(segments[0]))


def robot_face(expression: Any, said: bool) -> Optional[Any]:
    """Лицо итога хода чата, которое показывает робот: сцены исполненных ``play_animation`` (``turn.Shown``) — всегда,
    это действие на роботе; лицо команды — только как ответ лицом, когда текста в чат в этом ходе не было (``said``:
    реплика или фраза кода о выступлении): рядом с текстом это настроение текста, а не просьба к роботу (на
    «который час» лицо робота не меняется, «спой» с ``expression`` — одно сообщение «Пою на роботе…»). Показывать
    нечего — ``None``."""
    if expression is None:
        return None
    return expression if isinstance(expression, Shown) or not said else None


def face_words(expression: Any) -> str:
    """Что показал робот, словами кода для фразы чата: сцены по очереди (пачка ``play_animation`` — ``Shown.then``),
    русскими словами одного словаря сцен (``knowledge.SCENE_WORDS``), а не именами сцен."""
    scenes = (expression, *getattr(expression, "then", ()))
    labels = (face_label(s.emotion, s.gesture, s.face, words=kn.SCENE_WORDS) for s in scenes)
    return ", ".join(dict.fromkeys(filter(None, labels)))


def show_phrase(refused: str, face: Any) -> Optional[str]:
    """Фраза кода в чат об исходе показа лица на роботе (ADR-0148: из исхода, а не из слов модели): показано —
    «Показываю на роботе: …» (словами :func:`face_words`; назвать нечего — ``None``), не показано — фраза причины
    ``remote.show.rejected.<причина>`` (причины без ключа — «Не вышло.»)."""
    if not refused:
        words = face_words(face)
        return phrases.text(kn.REMOTE_SHOW_PHRASE, face=words) if words else None
    key = phrases.first_key(f"{kn.REMOTE_SHOW_REJECTED_PHRASE}.{refused}", "local.rejected")
    return phrases.text(str(key))


def _result_term_s(duration_ms: Any) -> float:
    """Срок позднего итога реплики: длительность + запас, не больше потолка; длительность неизвестна - потолок."""
    seconds = float(duration_ms or 0) / 1000.0
    if seconds <= 0:
        return kn.SPEECH_RESULT_CEILING_S
    return min(kn.SPEECH_RESULT_CEILING_S, seconds + kn.SPEECH_RESULT_SLACK_S)


class ChatPorts:
    """Порты хода одного чата: LLM и исполнение — общие порты хоста (``base``), речь — текст в чат, действия на роботе
    — пульт хоста (``robot``): выступление и ответ лицом исполняет робот, в чат — фраза кода об исполнении.

    Стадии хода кольцо не трогают (чат — не разговор с роботом вслух), отменять в чате нечего, выступление
    личности стоп из чата не рвёт; стоп движения — тот же интент, что у голоса."""

    def __init__(self, base: Any, chat_id: int, send: Send, log: Callable[[str], None] = _LOG.info,
                 hold: Callable[[], bool] = lambda: False, robot: Optional[Robot] = None) -> None:
        self._base = base
        self._ref = channel_ref(chat_id)
        self._send = send
        self._log = log
        self._hold = hold  # держит ли оператор Личность (кадр ``OperatorHold`` её сессии)
        self._robot = robot
        self._said = ""  # ход, в котором в чат ушёл текст: ответ лицом на роботе — только без него (:func:`robot_face`)

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
                            "rejected", HELD)

    def late(self, result: ActionResult) -> None:
        """Поздний итог действия (``Executor.on_late``): фраза кода по событию владельца — в чат («Озвучил (12 с)»)."""
        said = phrase_from_result(result)
        if said is None:
            return
        self._send(self._ref, said.text, uuid.uuid4().hex)
        self._log(f"{LOG_MARK} → {self._ref} итог {result.tool} {result.status} {result.reason or '-'}")

    def say(self, turn_id: str, epoch: int, text: str, voice: Optional[str] = None,
            emotion: Optional[str] = None) -> str:
        """Реплика хода (слова модели, фраза кода об исходе) — текстом в чат; в динамики робота она не идёт."""
        speech_id = uuid.uuid4().hex
        self._said = turn_id
        self._send(self._ref, text, speech_id)
        self._log(f"{LOG_MARK} → {self._ref} turn={turn_id} epoch={epoch} chars={len(text)}")
        return speech_id

    def stage(self, turn_id: str, epoch: int, stage: str, status: str, reason: str = "", cls: Optional[str] = None,
              llm_calls: int = 0, provider: str = "", source: str = "", expression: Optional[Any] = None) -> bool:
        """Стадия хода чата — строка лога; лицо итога ``done`` (``turn.finish_stage``) — на лицо робота, когда это
        действие на роботе (:func:`robot_face`), и фраза кода в чат об исходе показа (:func:`show_phrase`).
        ``True`` — робот показал лицо; ``False`` — нет (пульта нет, лицо — настроение текста в чат; оператор держит
        робота или у Личности тишина — тогда фраза кода об отказе вместо «Показываю»): в окне его нет."""
        if status != "started":
            self._log(f"{LOG_MARK} {self._ref} turn={turn_id} stage={stage} status={status} reason={reason or '-'} "
                      f"llm_calls={llm_calls} provider={provider or '-'}")
        face = robot_face(expression, said=self._said == turn_id)
        if face is None or self._robot is None:
            return False
        refused = HELD if self._hold() else self._robot.show(turn_id, face)
        self._log(f"{LOG_MARK} {self._ref} → робот: лицо {face_words(face) or '-'} "
                  f"{'не показано: ' + refused if refused else 'показано'} turn={turn_id}")
        said = show_phrase(refused, face)
        if said:
            self._send(self._ref, said, uuid.uuid4().hex)
        return not refused

    def cancel_speech(self, epoch: int, reason: str, boundary: str = "as_requested") -> None:
        return None

    def stop_perform(self, how: str) -> bool:
        return False

    def perform(self, turn_id: str, epoch: int, perf: Any, edit: bool) -> Optional[Mapping[str, Any]]:
        """Выступление агента чата исполняет робот: пульт хоста отдаёт его хосту выступления Личности в ЕЁ эпохе
        (``epoch`` чата сюда не идёт: «Робот, хватит» голосом гасит и это выступление), сегменты звучат голосом робота,
        а не текстом в чат. В чат — фраза кода из исхода старта (:func:`perform_phrase`): она — текст хода в чат, и лицо
        команды рядом с ней — настроение, а не второе сообщение «Показываю…» (:func:`robot_face`); отказ
        хоста — фраза хода по причине (``turn.hand_perform``). Оператор держит робота — отказ ``operator_hold`` без
        запроса хосту (как ``say_aloud``, §9.2.3). Пульта или хоста выступления нет — отказ кодом ``no_robot``: текст
        песни в чат не уходит и тогда."""
        if self._hold():
            outcome: Optional[Mapping[str, Any]] = {"status": "rejected", "reason": HELD}
        else:
            outcome = self._robot.perform(turn_id, perf, edit) if self._robot is not None else None
        result = dict(outcome) if outcome is not None else {"status": "rejected", "reason": kn.REMOTE_NO_ROBOT}
        self._log(f"{LOG_MARK} {self._ref} → робот: выступление {'правка' if edit else 'старт'} kind={perf.kind} "
                  f"segments={len(perf.segments)} → {result.get('status')} {result.get('reason') or '-'} "
                  f"turn={turn_id}")
        said = perform_phrase(perf, result)
        if said:
            self._said = turn_id
            self._send(self._ref, said, uuid.uuid4().hex)
        return result

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
        self._music: Any = None  # последний снимок музыки робота (:meth:`set_music`): новый чат начинает с него

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
            if self._music is not None:
                self._engines[chat_id].session.set_music(self._music)
            self._log(f"{LOG_MARK} новая сессия {channel_ref(chat_id)} (чатов: {len(self._engines)})")
        return self._engines[chat_id]

    def set_music(self, music: Any) -> None:
        """Снимок музыки РОБОТА (``session.MusicStateRef`` от хоста, тот же, что у Личности) — в сессию каждого чата:
        чат — пульт робота, и его путь пения и срез тулов судят по тому, что играет на роботе («под этот бит
        зачитай» — поверх играющего, «сделай бит потяжелее» — правка играющего, как у голоса)."""
        self._music = music
        for engine in self._engaged():
            engine.session.set_music(music)

    def performance_ended(self, turn_id: str, end_reason: str, segments_done: int = 0) -> bool:
        """Выступление на роботе кончилось (``block_end`` его хоста): запись «идёт» закрывается и в окне каждого чата
        (иначе модель чата видела «выступление идёт», пока ход не выпадал из окна). Выступление на роботе
        одно за раз (``TurnLog.performance_ended`` закрывает все записи «идёт»). Закрыта хоть одна — ``True``."""
        closed = [engine.log.performance_ended(turn_id, end_reason, segments_done) for engine in self._engaged()]
        return any(closed)

    def _engaged(self) -> list:
        """Движки чатов копией: ``set_music`` и ``performance_ended`` зовут ROS-колбэки музыки и ``block_end``, а
        :meth:`engine` добавляет чат из колбэка реплик — при многопоточном исполнителе обход самого словаря упал бы
        «dictionary changed size during iteration»."""
        return list(self._engines.values())

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


__all__ = ["CHANNEL_PREFIX", "ChatPorts", "ChatTurns", "LOG_MARK", "Robot", "channel_ref", "chat_of", "face_words",
           "perform_phrase", "quote", "robot_face", "show_phrase"]
