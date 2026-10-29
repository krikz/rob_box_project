"""#4218: после переименования профиля голоса (#4054) прежнее имя не звучит из ``recent_actions`` (ADR-0148).

Марафон-6 29.10 (dev-980c053), акт 2c, n723: голос назвался Ниной (n721, ``register_speaker("Нина")`` ``done``) → тот же
голос: «я Тамара, мы не знакомы» (n722, ``done{renamed}``, профиль переименован) → «как меня зовут?» (n723). В снимке n723
лежали обе записи - ``register_speaker {"name":"Нина"}`` и ``register_speaker {"name":"Тамара"}`` - и модель сказала
«сначала я услышал „Нина“». Теперь при ``renamed`` журнал убирает регистрации прежнего имени (``turn_log.forget_old_name``):
в снимке следующего хода - одна, с текущим именем. Тест проверяет, что уходит модели, а не текст промпта.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.turn import RECENT_ACTIONS, Answer, TurnEngine
from rob_box_dialog.turn_log import TOOL, TurnLog, forget_old_name

NINA = SpeakerRef(person_ref="spk-nina", name="Нина", name_state="confirmed")
STRANGER = SpeakerRef()
GUESS = SpeakerRef(person_ref="spk-nina", name="Нина", name_state="tentative")


def _result(name, reason=None, turn_id="t1/0", status="done"):
    return ActionResult(turn_id, 0, "personality", "register_speaker", kn.TOOL_CLASSES["register_speaker"], status,
                        reason, {"name": name})


def _names(log):
    return [e.content["data"]["name"] for e in log.window() if e.role == TOOL and e.content["tool"] == "register_speaker"]


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход ``register_speaker`` - из ``outcomes`` (как его отдаёт тул)."""

    def __init__(self, answers, outcomes):
        self.answers, self.outcomes, self.asked, self.said = list(answers), list(outcomes), [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                             "done", self.outcomes.pop(0), {"name": a.args.get("name")})
                for n, a in enumerate(actions)]

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
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _recent(request):
    last = request[-1]["content"]
    start = last.find(f"<{RECENT_ACTIONS}>")
    return last[start:last.find(f"</{RECENT_ACTIONS}>")] if start >= 0 else ""


def _register(name):
    return Answer(Command(actions=(ToolCall("register_speaker", {"name": name}),)), "deepseek")


def _n721_n723(second_speaker):
    """n721 «я Нина» (голос неизвестен) → n722 «я Тамара, мы не знакомы» (голос ``second_speaker``) → n723."""
    session = Session("personality")
    ports = Ports([_register("Нина"), _register("Тамара"), Answer(Command(speech="Тебя зовут Тамара."), "deepseek")],
                  [None, kn.RENAMED_REASON])
    engine = TurnEngine(session, ports, system_prompt="Ты робот.")
    for speaker, text in ((STRANGER, "робот я нина запомни мой голос"),
                          (second_speaker, "робот я тамара мы с тобой еще не знакомы"),
                          (SpeakerRef(person_ref="spk-nina", name="Тамара", name_state="confirmed"),
                           "а скажи-ка как меня зовут")):
        session.set_speaker(speaker)
        asyncio.run(engine.run(Addressed(text, VOICE)))
    return ports


def test_n723_snapshot_after_rename_has_no_old_name():
    ports = _n721_n723(NINA)
    recent = _recent(ports.asked[2])
    assert "Нина" not in recent, recent
    assert recent.count('"tool":"register_speaker"') == 1 and "Тамара" in recent and "renamed" in recent


def test_n723_old_name_dropped_even_when_snapshot_speaker_was_not_named():
    """Голос в снимке n722 без подтверждённого имени (гипотеза, неизвестный): прежнее имя неизвестно - уходят регистрации
    под любым другим именем, кроме нового."""
    for speaker in (STRANGER, GUESS):
        recent = _recent(_n721_n723(speaker).asked[2])
        assert "Нина" not in recent and "Тамара" in recent, speaker.name_state


def test_plain_registration_next_turn_is_kept():
    """Без переименования записи регистраций остаются как были."""
    session = Session("personality")
    ports = Ports([_register("Нина"), Answer(Command(speech="Нина."), "deepseek")], [None])
    engine = TurnEngine(session, ports, system_prompt="Ты робот.")
    for text in ("робот я нина запомни мой голос", "а как меня зовут"):
        asyncio.run(engine.run(Addressed(text, VOICE)))
    assert "Нина" in _recent(ports.asked[1])


def test_forget_old_name_drops_only_the_old_names_registrations():
    log = TurnLog()
    for turn, name in (("t1", "Нина"), ("t2", "Гриша"), ("t3", "Тамара")):
        log.begin(turn, 0, "я " + name, "")
        log.result(turn, _result(name, turn_id=turn + "/0"))
    log.result("t3", _result("Тамара", kn.RENAMED_REASON, turn_id="t3/1"))
    assert forget_old_name(log, "Нина", "Тамара") == 1
    assert _names(log) == ["Гриша", "Тамара", "Тамара"], "чужая регистрация и свежие записи нового имени на месте"
    assert forget_old_name(log, None, "Тамара") == 1, "имя в снимке неизвестно - любая регистрация другого имени уходит"
    assert _names(log) == ["Тамара", "Тамара"]


def test_forget_old_name_matches_name_by_key_and_leaves_other_tools():
    log = TurnLog()
    log.begin("t1", 0, "я нина", "")
    log.result("t1", _result("нина"))  # тот же ключ имени, что «Нина»
    log.result("t1", ActionResult("t1/1", 0, "personality", "play_animation", kn.TOOL_CLASSES["play_animation"],
                                  "done", None, {"animation": "happy"}))
    assert forget_old_name(log, "Нина", "Тамара") == 1
    assert [e.content["tool"] for e in log.window() if e.role == TOOL] == ["play_animation"]
