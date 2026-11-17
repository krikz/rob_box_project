"""Пульт лиц в Telegram, вторая часть (#4505): потолок снимков за ход, перенос группы ``faces`` на следующий ход,
``say_aloud`` в чате только по явной просьбе озвучить (ADR-0148: решает код, а не правило в промпте)."""

import asyncio
import logging

import pytest

from rob_box_dialog import action_chain
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import SOURCE_TELEGRAM, Addressed
from rob_box_dialog.command import Command, Invalid, ToolCall, parse, unkept_done_claims, validate
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools, groups_for
from rob_box_dialog.turn import Answer, TurnEngine, carried_groups, unasked_aloud
from rob_box_dialog.words import done_claims

FACES = {"people_list", "person_photo", "name_person"}
LOG = logging.getLogger("t4505")


class _Turn:
    def __init__(self, text="", pickup=False):
        self.text, self.pickup, self.turn_id = text, pickup, "t1"
        self.wake_words = None  # таблица вейков хода (``turn._Turn.wake_words``): сессия теста без профиля


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


# --- 4. снимки лиц: чтение, не вереница -----------------------------------------------------------------------


def _photos(n):
    return tuple(ToolCall("person_photo", {"id": f"{i:08x}"}) for i in range(n))


def test_photos_are_reads_and_never_a_chain():
    """Потолка «три за ход» больше нет (ADR-0165 этап 3): снимки - ``query``, команда с ними идёт разом, как раньше."""
    assert not action_chain.runs_as_chain(_photos(5))
    deletes = tuple(ToolCall("delete_waypoint", {"name": f"т{i}"}) for i in range(5))
    assert not action_chain.runs_as_chain(_photos(5) + deletes)


def test_the_model_may_send_ten_photos():
    assert kn.ACTIONS_ACCEPT_MAX == 10


def test_ten_photos_all_reach_the_executor_in_one_turn():
    ports = Ports([
        Answer(Command(actions=_photos(10)), "deepseek"),
        Answer(Command(speech="Снимки выше."), "deepseek"),
    ])

    _run(_engine(ports), "ну пришли их")

    assert ports.executed == [("person_photo", {"id": f"{n:08x}"}) for n in range(10)]


def test_the_eleventh_photo_is_rejected_by_the_existing_action_limit():
    raw = {"actions": [{"tool": "person_photo", "args": {"id": f"{n:08x}"}} for n in range(11)]}
    result = validate(parse(raw), [kn.PHOTO_TOOL], Session("telegram").snapshot)
    assert isinstance(result, Invalid) and result.reason == "too_many_actions"


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


@pytest.mark.parametrize("tool", ["people_list", "person_photo"])
def test_carried_groups_come_from_called_tools_only(tool):
    assert carried_groups([tool]) == (kn.FACES_GROUP,)
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


# --- 6. обещание снимков без person_photo ----------------------------------------------------------------------


PHOTO_SEND_PROMISE = "Сейчас пришлю снимки всех, кого видел сегодня."


@pytest.mark.parametrize("text", [PHOTO_SEND_PROMISE, "Отправлю фото в чат."])
def test_photo_send_promises_are_recognized(text):
    assert done_claims(text) == ("photo_send",)


@pytest.mark.parametrize("text", [
    "Не пришлю фото без разрешения.",
    "Пришлю ссылку в чат.",
    "Китайский зонд прислал новые снимки Луны.",
])
def test_other_send_words_are_not_photo_claims(text):
    assert done_claims(text) == ()


def test_live_photo_promise_with_only_say_aloud_is_a_done_claim_violation():
    raw = {
        "speech": PHOTO_SEND_PROMISE,
        "actions": [{"tool": kn.ALOUD_TOOL, "args": {"text": PHOTO_SEND_PROMISE}}],
    }
    result = validate(parse(raw), [kn.PHOTO_TOOL, kn.ALOUD_TOOL], Session("telegram").snapshot)
    assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_DONE_CLAIM
    assert kn.PHOTO_TOOL in result.all[0].allowed


def test_person_photo_backs_the_send_promise_but_say_aloud_does_not():
    photo = Command(speech=PHOTO_SEND_PROMISE, actions=(ToolCall(kn.PHOTO_TOOL, {"id": "71cda108"}),))
    aloud = Command(speech=PHOTO_SEND_PROMISE, actions=_aloud().actions)
    assert unkept_done_claims(photo) == ()
    assert unkept_done_claims(aloud) == ("photo_send",)
