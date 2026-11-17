"""Вереница и прерывание, этап 4 (ADR-0165 §6, §7, §12 пп. 5-7, 10; #4403): стоп Tier-1 гасит вереницу и робот
говорит итог кодом - что успел, что не стал; «замолчи» - вереница идёт молча, итог в окне; новая сессия - итог не
звучит; стоп из чата - итог строкой в чат. LLM нет: ответы модели - заготовки, владельцы - заглушки."""

import asyncio

from rob_box_dialog import action_chain as ac
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.chats import ChatPorts, channel_ref
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.execute import Executor, OwnerEvent
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine, settle

POINTS = ("хил-вход", "хил-кухня", "хил-диван", "хил-окно")


def deletes(names=POINTS):
    return tuple(ToolCall("delete_waypoint", {"name": n}) for n in names)


def trips_and_save():
    return (ToolCall("navigate_to_waypoint", {"waypoint": "склад"}), ToolCall("save_waypoint", {"name": "диван"}))


class Ports:
    """Модель - заготовки; владельцы отвечают ``done`` (``outcome`` - тул → (статус, причина)). ``hold`` - номер
    запроса (с 1), который ждёт события теста: реплика посреди шага. ``epochs`` - эпоха каждого запроса."""

    def __init__(self, answers, hold=None, outcome=None):
        self.answers, self.hold, self.outcome = list(answers), hold, dict(outcome or {})
        self.gate = asyncio.Event()
        self.reached = asyncio.Event()
        self.said, self.batches, self.epochs, self.intents = [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.batches.append([c.args.get("name") or c.args.get("waypoint") or c.tool for c in actions])
        self.epochs.append(snapshot.epoch)
        if len(self.batches) == self.hold:
            self.reached.set()
            await self.gate.wait()
        return [
            ActionResult(
                f"{turn_id}/{n}",
                snapshot.epoch,
                snapshot.agent,
                c.tool,
                kn.TOOL_CLASSES[c.tool],
                *self.outcome.get(c.tool, ("done", None)),
                dict(c.args),
            )
            for n, c in enumerate(actions)
        ]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        self.intents.append(kind)

    def perform(self, turn_id, epoch, perf, edit):
        return None


def engine_for(ports, session=None):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(session or Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))


def answer(actions, speech=None):
    return Answer(Command(speech=speech, actions=tuple(actions)), "stub")


def interrupted(ports, said_mid, first="Робот, удали четыре точки", session=None):
    """Вереница, на шаге ``ports.hold`` - реплика ``said_mid`` (ход Tier-1), затем шаг отпускается."""

    async def main():
        engine = engine_for(ports, session)
        await engine.run(Addressed(first, VOICE))
        await ports.reached.wait()
        await engine.run(Addressed(said_mid, VOICE))
        ports.gate.set()
        await settle(engine)
        return engine

    return asyncio.run(main())


def plan_of(names=POINTS):
    return phrases.text("chain.plan.delete", items=", ".join(names))


# --- стоп гасит вереницу, итог звучит (§6, §7, §12 пп. 5, 7) ------------------------------------------------------


def test_halt_mid_step_says_what_was_done_and_what_was_not_started():
    """«Робот, стой» посреди второго удаления: точка доудалена (непрерываемый шаг), третья и четвёртая не начинались."""
    ports = Ports([answer(deletes())], hold=2)
    interrupted(ports, "Робот, стой")
    assert ports.batches == [["хил-вход"], ["хил-кухня"]]
    assert ports.said == [plan_of(), "Остановился. Удалил: хил-вход, хил-кухня. Не стал: хил-диван, хил-окно."]


def test_enough_stops_the_chain_too():
    """«Робот, хватит» (политика ``stop``, тот же вид ``stop_all``) гасит вереницу так же, как «стой»."""
    ports = Ports([answer(deletes())], hold=1)
    interrupted(ports, "Робот, хватит")
    assert ports.batches == [["хил-вход"]]
    assert ports.said[-1] == "Остановился. Удалил: хил-вход. Не стал: хил-кухня, хил-диван, хил-окно."


def test_summary_after_a_stop_goes_in_the_stop_epoch():
    """Владелец речи отбрасывает фразы старой эпохи: итог - в эпохе стопа, а не вереницы."""
    session = Session("personality")
    ports = Ports([answer(deletes())], hold=1)
    epochs = []
    original = ports.say
    ports.say = lambda turn_id, epoch, text, *a, **k: (epochs.append(epoch), original(turn_id, epoch, text))[1]
    interrupted(ports, "Робот, стой", session=session)
    assert epochs == [0, session.snapshot.epoch] and session.snapshot.epoch == 1


def test_halt_during_a_trip_stops_motion_and_names_the_trip_as_not_reached():
    """§12 п. 6, HIL п. 3: стоп во время поездки - ``stop_motion``, поездка не доехала, остаток «не стал» (не «дальше
    не еду»: остаток отменил человек); причина отмены («движение отменили») не повторяет «Остановился»."""
    ports = Ports([answer(trips_and_save())], hold=1, outcome={"navigate_to_waypoint": ("rejected", "canceled")})
    interrupted(ports, "Робот, стой", first="Робот, поезжай на склад и запомни там диван")
    assert "stop_motion" in ports.intents and ports.batches == [["склад"]]
    assert ports.said[-1] == "Остановился. Не доехал: склад. Не стал: диван."


def test_stop_after_the_last_step_still_answers_the_stop():
    ports = Ports([answer(deletes())], hold=4)
    interrupted(ports, "Робот, стой")
    assert len(ports.batches) == 4
    assert ports.said[-1] == f"Остановился. Удалил: {', '.join(POINTS)}."


# --- «замолчи» и «хватит говорить»: вереница идёт (В4) ------------------------------------------------------------


def test_silence_keeps_the_chain_going_quietly_and_puts_the_summary_into_the_window():
    """В4: «замолчи» - объезд идёт молча: все шаги, в новой эпохе; итог не звучит, а в окне хода вереницы есть."""
    session = Session("personality")
    ports = Ports([answer(deletes())], hold=2)
    engine = interrupted(ports, "Робот, замолчи", session=session)
    assert [b[0] for b in ports.batches] == list(POINTS)
    assert ports.epochs == [0, 0, 1, 1]  # после «замолчи» шаги - в новой эпохе, исполнитель их не отбросит
    assert ports.said == [plan_of()]
    done = phrases.text("chain.did.delete", items=", ".join(POINTS))
    mark = kn.CHAIN_UNVOICED_MARK.format(summary=f"Готово. {done}")
    assert mark in [e.text() for e in engine.log.window()]


def test_stop_speech_keeps_the_chain_and_the_summary_is_spoken():
    """«Хватит говорить» гасит речь, а не действия (движение и музыку - тоже нет): вереница доходит до конца."""
    ports = Ports([answer(deletes())], hold=2)
    interrupted(ports, "Робот, хватит говорить")
    assert len(ports.batches) == 4 and ports.said[-1].startswith("Готово.")


# --- новая сессия: остаток отменён, итог не звучит ----------------------------------------------------------------


def test_new_session_cancels_the_rest_and_says_no_summary():
    ports = Ports([answer(deletes())], hold=2)
    session = Session("personality")
    interrupted(ports, "Робот, новая сессия", session=session)
    assert ports.batches == [["хил-вход"], ["хил-кухня"]]
    assert all(not s.startswith("Остановился") for s in ports.said)


# --- стоп из чата Telegram: итог строкой в чат --------------------------------------------------------------------


def test_stop_from_the_chat_stops_its_chain_and_the_summary_goes_to_the_chat():
    base = Ports([answer(deletes())], hold=2)
    sent = []

    async def main():
        session = Session("telegram", session_id=channel_ref(7))
        chat = ChatPorts(base, 7, lambda ref, text, sid: sent.append((ref, text)), lambda line: None)
        engine = TurnEngine(session, chat, system_prompt="Ты ассистент в чате.")
        await engine.run(Addressed("удали четыре точки", "telegram"))
        await base.reached.wait()
        await engine.run(Addressed("стоп", "telegram"))
        base.gate.set()
        await settle(engine)

    asyncio.run(main())
    assert base.batches == [["хил-вход"], ["хил-кухня"]] and "stop_motion" in base.intents
    assert sent[-1] == ("telegram:7", "Остановился. Удалил: хил-вход, хил-кухня. Не стал: хил-диван, хил-окно.")


# --- настоящий исполнитель: шаг, что шёл при смене эпохи, засчитан по исходу владельца ----------------------------


class ExecutorPorts(Ports):
    """Порты с настоящим ``Executor`` (эпоха - из сессии, как у хоста): владелец отвечает ``done`` на каждый запрос,
    на запросе ``hold`` - после события теста. Исход старой эпохи исполнитель подменяет на ``cancelled{stale_epoch}``.
    """

    def __init__(self, answers, session, hold):
        super().__init__(answers, hold=hold)
        self.requests = []
        self.executor = Executor(self._send, epoch=lambda: session.snapshot.epoch)

    async def execute(self, actions, snapshot, turn_id):
        self.batches.append([c.args.get("name") for c in actions])
        self.epochs.append(snapshot.epoch)
        return await self.executor.run(actions, snapshot, turn_id)

    async def _send(self, request):
        self.requests.append(request)
        if len(self.requests) == self.hold:
            self.reached.set()
            await self.gate.wait()
        self.executor.on_event(OwnerEvent(request.turn_id, "done", None, {"name": request.args["name"]}))


def test_with_the_real_executor_a_step_that_lived_through_the_stop_counts_as_done():
    """§12 п. 7 целиком: «стой» посреди удаления, владелец ответил ``done`` уже в новой эпохе - «удалил»."""
    session = Session("personality")
    ports = ExecutorPorts([answer(deletes())], session, hold=2)
    interrupted(ports, "Робот, стой", session=session)
    assert ports.said[-1] == "Остановился. Удалил: хил-вход, хил-кухня. Не стал: хил-диван, хил-окно."


def test_with_the_real_executor_steps_after_silence_are_not_stale():
    session = Session("personality")
    ports = ExecutorPorts([answer(deletes())], session, hold=2)
    engine = interrupted(ports, "Робот, замолчи", session=session)
    assert [r.epoch for r in ports.requests] == [0, 0, 1, 1]
    done = phrases.text("chain.did.delete", items=", ".join(POINTS))
    assert kn.CHAIN_UNVOICED_MARK.format(summary=f"Готово. {done}") in [e.text() for e in engine.log.window()]


# --- итог: шаг, что шёл при стопе (action_chain) -------------------------------------------------------------------


def _outcome(call, status, note="", cut=False):
    cls = kn.call_class(call.tool, call.args)
    return ac.StepOutcome(ac.Step(call, cls, ac.expected_s(cls, call)), status, note, cut)


def test_cut_step_reason_is_not_spoken_and_the_rest_is_not_started():
    trip, save = trips_and_save()
    outcomes = [_outcome(trip, "cancelled", "Остановился.", cut=True), _outcome(save, kn.CHAIN_STEP_SKIPPED)]
    assert ac.summary(outcomes, stopped=True) == "Остановился. Не доехал: склад. Не стал: диван."


def test_failed_trip_without_a_stop_keeps_its_reason_and_motion_failed_phrase():
    trip, save = trips_and_save()
    outcomes = [_outcome(trip, "rejected", "Впереди препятствие."), _outcome(save, kn.CHAIN_STEP_SKIPPED)]
    assert ac.summary(outcomes) == "Не доехал: склад. Впереди препятствие. Дальше не еду: диван."


def test_only_stop_all_stops_a_chain():
    assert kn.CHAIN_STOP_KINDS == ("stop_all",)
