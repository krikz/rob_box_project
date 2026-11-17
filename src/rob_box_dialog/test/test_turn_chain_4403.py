"""Вереница в ходе, этап 3 (ADR-0165 §12 «Ход», #4403): план до шагов, ход закрыт после плана, шаги по одному,
эпоха между шагами, доживший шаг честно, итог кодом. LLM нет: ответы модели - заготовки, владельцы - заглушки."""

import asyncio

from rob_box_dialog import action_chain
from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall, parse, validate
from rob_box_dialog.execute import Executor, OwnerEvent
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine, settle

POINTS = ("хил-вход", "хил-кухня", "хил-диван", "хил-окно")
TOOLS = ["delete_waypoint", "save_waypoint", "navigate_to_waypoint", "set_volume", "register_speaker"]


def deletes(names=POINTS):
    return tuple(ToolCall("delete_waypoint", {"name": n}) for n in names)


class Ports:
    """Модель - заготовки; владельцы отвечают ``done``, кроме ``fail`` (тул → причина); ``gate`` - шаг ждёт события
    теста (новая реплика посреди вереницы); ``log`` - порядок речи и запросов."""

    def __init__(self, answers, fail=None, gate=None, on_step=None):
        self.answers, self.fail, self.gate, self.on_step = list(answers), dict(fail or {}), gate, on_step
        self.log, self.said, self.batches = [], [], []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.batches.append([c.args.get("name") or c.args.get("waypoint") or c.tool for c in actions])
        self.log.append(("execute", turn_id))
        if self.gate is not None:
            await self.gate.wait()
        out = []
        for n, call in enumerate(actions):
            reason = self.fail.get(call.tool)
            out.append(
                ActionResult(
                    f"{turn_id}/{n}",
                    snapshot.epoch,
                    snapshot.agent,
                    call.tool,
                    kn.TOOL_CLASSES[call.tool],
                    "rejected" if reason else "done",
                    reason,
                    dict(call.args),
                )
            )
        if self.on_step is not None:
            self.on_step(len(self.batches))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        self.log.append(("say", text))
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def engine_for(ports, session=None):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(session or Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))


def run(engine, text="Робот, удали четыре точки"):
    async def main():
        agg = await engine.run(Addressed(text, VOICE))
        await settle(engine)
        return agg

    return asyncio.run(main())


def answer(actions, speech=None):
    return Answer(Command(speech=speech, actions=tuple(actions)), "stub")


def test_plan_is_said_before_the_first_step_and_steps_go_one_by_one():
    ports = Ports([answer(deletes())])
    agg = run(engine_for(ports))
    assert agg.reason == "chain"
    assert ports.log[0] == ("say", phrases.text("chain.plan.delete", items=", ".join(POINTS)))
    assert ports.batches == [[p] for p in POINTS]
    assert [ref for kind, ref in ports.log if kind == "execute"] == [f"t1/c{k}" for k in range(4)]
    assert ports.said[-1] == f"Готово. {phrases.text('chain.did.delete', items=', '.join(POINTS))}"


def test_model_plan_is_spoken_as_is():
    ports = Ports([answer(deletes(), speech="Сейчас уберу четыре точки.")])
    run(engine_for(ports))
    assert ports.said[0] == "Сейчас уберу четыре точки."


def test_turn_closes_after_the_plan_and_a_new_reply_gets_its_own_answer():
    """Ход закрыт после плана: пока шаг вереницы ждёт владельца, новая реплика получает ответ (§9.2)."""

    async def main():
        gate = asyncio.Event()
        ports = Ports([answer(deletes()), Answer(Command(speech="Двенадцать часов."), "stub")], gate=gate)
        engine = engine_for(ports)
        await engine.run(Addressed("Робот, удали четыре точки", VOICE))
        for _ in range(5):  # задача вереницы стартует на следующих тактах цикла
            await asyncio.sleep(0)
        assert engine.chains and ports.batches == [["хил-вход"]]  # вереница идёт, ход уже закрыт
        await engine.run(Addressed("Робот, сколько времени?", VOICE))
        assert "Двенадцать часов." in ports.said
        gate.set()
        await settle(engine)
        return ports

    ports = asyncio.run(main())
    assert len(ports.batches) == 4 and ports.said[-1].startswith("Готово.")


def test_failed_trip_cancels_the_rest():
    calls = (ToolCall("navigate_to_waypoint", {"waypoint": "склад"}), ToolCall("save_waypoint", {"name": "диван"}))
    ports = Ports([answer(calls)], fail={"navigate_to_waypoint": "aborted"})
    run(engine_for(ports), "Робот, поезжай на склад и запомни там диван")
    assert ports.batches == [["склад"]]
    assert ports.said[-1].endswith(phrases.text(kn.CHAIN_MOTION_FAILED_PHRASE, items="диван"))


def test_failed_local_step_does_not_stop_the_chain():
    ports = Ports([answer(deletes())], fail={})
    original = ports.execute

    async def execute(actions, snapshot, turn_id):
        out = await original(actions, snapshot, turn_id)
        if actions[0].args["name"] == "хил-диван":
            r = out[0]
            out = [ActionResult(r.turn_id, r.epoch, r.agent, r.tool, r.cls, "rejected", "not_found", r.data)]
        return out

    ports.execute = execute
    run(engine_for(ports))
    assert len(ports.batches) == 4
    assert "Не удалил: хил-диван." in ports.said[-1] and not ports.said[-1].startswith("Готово.")


def test_epoch_change_between_steps_starts_no_new_step_and_says_no_summary():
    """Эпоха + 1 без стопа Tier-1 (сброс сессии) после первого шага: следующих запросов нет, итог не звучит. Стоп
    Tier-1 и его итог «Остановился. …» - этап 4, ``test_turn_chain_stop_4403.py``."""
    session = Session("personality")
    ports = Ports([answer(deletes())], on_step=lambda n: session.replace_epoch() if n == 1 else None)
    run(engine_for(ports, session))
    assert ports.batches == [["хил-вход"]]
    assert ports.said == [phrases.text("chain.plan.delete", items=", ".join(POINTS))]


def test_summary_is_not_spoken_in_silence():
    session = Session("personality")
    ports = Ports([answer(deletes())], on_step=lambda n: session.push_scene("Silence", "user") if n == 2 else None)
    run(engine_for(ports, session))
    assert len(ports.batches) == 4 and len(ports.said) == 1  # только план; итог - в журнале и окне


def test_chain_over_its_task_limit_does_not_start(monkeypatch):
    tasks = dict(kn.CHAIN_TASKS)
    tasks[kn.CHAIN_ERRAND] = kn.ChainTask(steps_max=3, trips_max=1, term_s=None)
    monkeypatch.setattr(kn, "CHAIN_TASKS", tasks)
    ports = Ports([answer(deletes())])
    run(engine_for(ports))
    assert ports.batches == [] and ports.said == [phrases.text(kn.CHAIN_TOO_LONG_PHRASE)]


def test_survived_step_after_a_stop_is_counted_by_the_owners_outcome():
    """§12 п. 7: эпоха сменилась, пока владелец удалял точку - исход ``cancelled{stale_epoch}`` несёт ``done``."""
    epoch = [0]

    async def main():
        async def send(request):
            epoch[0] = 1  # «стой» посреди шага
            ex.on_event(OwnerEvent(request.turn_id, "done", None, {"name": "хил-вход"}))

        ex = Executor(send, epoch=lambda: epoch[0])
        return await ex.run([ToolCall("delete_waypoint", {"name": "хил-вход"})], Session("personality").snapshot, "t")

    [res] = asyncio.run(main())
    assert (res.status, res.reason) == ("cancelled", "stale_epoch")
    assert action_chain.step_status(res.status, res.reason, res.data) == "done"


def test_short_commands_run_at_once_as_before():
    ports = Ports([answer(deletes(POINTS[:3]))])
    agg = run(engine_for(ports))
    assert agg.reason != "chain" and ports.batches == [list(POINTS[:3])] and ports.said == ["Готово."]


# --- валидатор: речь вереницы - план ---------------------------------------------------------------------------


def test_plan_speech_beside_a_chain_is_valid_and_short_command_speech_is_still_a_violation():
    chain = {
        "speech": "Сейчас уберу четыре точки.",
        "actions": [{"tool": "delete_waypoint", "args": {"name": p}} for p in POINTS],
    }
    assert isinstance(validate(parse(chain), TOOLS), Command)
    short = {**chain, "actions": chain["actions"][:2]}
    res = validate(parse(short), TOOLS)
    assert isinstance(res, cm.Invalid) and res.reason == kn.VIOLATION_SPEECH_WITH_ACTIONS


def test_plan_claiming_done_is_a_done_claim_even_with_the_tool():
    chain = {
        "speech": "Удалил четыре точки.",
        "actions": [{"tool": "delete_waypoint", "args": {"name": p}} for p in POINTS],
    }
    res = validate(parse(chain), TOOLS)
    assert isinstance(res, cm.Invalid) and res.reason == kn.VIOLATION_DONE_CLAIM
    assert "в плане до шагов" in res.detail


def test_retry_that_keeps_the_claim_runs_the_chain_with_the_code_plan():
    """§3: повтор не помог - план говорит код, ложь не звучит."""
    bad = validate(
        parse(
            {
                "speech": "Удалил четыре точки.",
                "actions": [{"tool": "delete_waypoint", "args": {"name": p}} for p in POINTS],
            }
        ),
        TOOLS,
    )
    ports = Ports([Answer(bad, "stub")])
    run(engine_for(ports))
    assert ports.said[0] == phrases.text("chain.plan.delete", items=", ".join(POINTS))
    assert "Удалил четыре точки." not in ports.said and len(ports.batches) == 4
