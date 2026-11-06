"""Пульт лиц в Telegram, вторая часть (#4505): потолок снимков за ход, перенос группы ``faces`` на следующий ход,
``say_aloud`` в чате только по явной просьбе озвучить (ADR-0148: решает код, а не правило в промпте)."""

import asyncio
import logging

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import SOURCE_TELEGRAM, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools, groups_for
from rob_box_dialog.turn import Answer, TurnEngine, capped, carried_groups, unasked_aloud

FACES = {"people_list", "person_photo", "name_person"}
LOG = logging.getLogger("t4505")


class _Turn:
    def __init__(self, text="", pickup=False):
        self.text, self.pickup, self.turn_id, self.capped = text, pickup, "t1", ()


class Ports:
    def __init__(self, answers):
        self.answers, self.asked, self.executed, self.said = list(answers), [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append({t.name for t in tools})
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed += [(c.tool, dict(c.args)) for c in actions]
        return [
            ActionResult(
                f"{turn_id}/{n}",
                snapshot.epoch,
                snapshot.agent,
                c.tool,
                kn.TOOL_CLASSES[c.tool],
                "done",
                None,
                dict(c.args),
            )
            for n, c in enumerate(actions)
        ]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *a, **k):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _engine(ports):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(Session("telegram"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))


def _run(engine, text):
    asyncio.run(engine.run(Addressed(text, SOURCE_TELEGRAM)))


# --- 4. потолок снимков ---------------------------------------------------------------------------------------


def _photos(n):
    return tuple(ToolCall("person_photo", {"id": f"{i:08x}"}) for i in range(n))


def test_ten_photos_are_all_executed_and_the_eleventh_is_named():
    turn = _Turn()
    kept = capped(LOG, turn, Command(actions=_photos(kn.PHOTOS_MAX + 1)))
    assert len(kept.actions) == kn.PHOTOS_MAX and len(turn.capped) == 1


def test_five_photos_are_not_cut_to_the_actions_limit():
    turn = _Turn()
    assert kn.PHOTOS_MAX > kn.ACTIONS_MAX
    kept = capped(LOG, turn, Command(actions=_photos(5)))
    assert len(kept.actions) == 5 and not turn.capped


def test_photos_do_not_eat_the_budget_of_other_actions():
    turn = _Turn()
    others = tuple(ToolCall("delete_waypoint", {"name": f"т{i}"}) for i in range(kn.ACTIONS_MAX + 1))
    kept = capped(LOG, turn, Command(actions=_photos(5) + others))
    assert sum(a.tool == "person_photo" for a in kept.actions) == 5
    assert sum(a.tool == "delete_waypoint" for a in kept.actions) == kn.ACTIONS_MAX and len(turn.capped) == 1


def test_the_model_may_send_as_many_photos_as_the_ceiling():
    assert kn.ACTIONS_ACCEPT_MAX >= kn.PHOTOS_MAX


# --- 3. перенос группы faces ----------------------------------------------------------------------------------


def _names(text, carry=()):
    return {t.name for t in available_tools(Session("telegram").snapshot, None, "telegram", text, carry=carry)}


@pytest.mark.parametrize("text", ["ну пришли их", "а ещё?", "покажи остальные", "давай"])
def test_followup_without_face_words_keeps_faces_tools_after_a_faces_turn(text):
    assert FACES <= _names(text, carry=(kn.FACES_GROUP,))
    assert not FACES & _names(text)


def test_another_topic_beats_the_carry():
    snap = Session("telegram").snapshot
    assert kn.FACES_GROUP not in groups_for(snap, "включи музыку", carry=(kn.FACES_GROUP,), agent="telegram")


def test_voice_agent_never_gets_faces_through_carry():
    names = {
        t.name
        for t in available_tools(
            Session("personality").snapshot, None, "personality", "а ещё?", carry=(kn.FACES_GROUP,)
        )
    }
    assert not FACES & names


def test_carried_groups_come_from_called_tools_only():
    assert carried_groups(["people_list"]) == (kn.FACES_GROUP,)
    assert carried_groups(["get_current_time"]) == ()


def test_engine_carries_faces_to_the_next_turn_of_the_same_source_then_drops_it():
    ports = Ports(
        [
            Answer(Command(actions=(ToolCall("name_person", {"id": "fff371fe", "name": "Кирилл"}),)), "deepseek"),
            Answer(Command(), "deepseek"),
            Answer(Command(), "deepseek"),
        ]
    )
    engine = _engine(ports)
    _run(engine, "назови fff371fe Кириллом")
    _run(engine, "а ещё?")
    assert FACES <= ports.asked[1]
    _run(engine, "а ещё?")  # второй ход тулов faces не звал - переноса нет
    assert not FACES & ports.asked[-1]


# --- 5. say_aloud в чате только по явной просьбе --------------------------------------------------------------


def _aloud():
    return Command(actions=(ToolCall(kn.ALOUD_TOOL, {"text": "привет"}),))


@pytest.mark.parametrize("text", ["скажи вслух привет", "озвучь это", "произнеси привет", "Озвучи, пожалуйста"])
def test_explicit_aloud_request_keeps_say_aloud(text):
    cmd = _aloud()
    assert unasked_aloud(LOG, _Turn(text), "telegram", cmd) is cmd


@pytest.mark.parametrize("text", ["привет, как дела", "покажи лица", "расскажи про погоду"])
def test_say_aloud_without_a_request_is_dropped_with_a_logged_reason(text, caplog):
    with caplog.at_level(logging.INFO):
        out = unasked_aloud(LOG, _Turn(text), "telegram", _aloud())
    assert out.actions == () and "say_aloud отброшен" in caplog.text


def test_other_agents_and_other_actions_are_untouched():
    cmd = _aloud()
    assert unasked_aloud(LOG, _Turn("привет"), "personality", cmd) is cmd
    other = Command(actions=(ToolCall("name_person", {"id": "fff371fe", "name": "К"}), *_aloud().actions))
    out = unasked_aloud(LOG, _Turn("привет"), "telegram", other)
    assert [a.tool for a in out.actions] == ["name_person"]
