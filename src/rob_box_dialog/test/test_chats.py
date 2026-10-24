"""Телеграм-агент на общем движке (ADR-0150 §9.2, PR-12a #3706): сессия и история на чат, ответы — текстом в чат."""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import COORDINATOR, Addressed, NotAddressed
from rob_box_dialog.agents import TIER1_GROUPS, agent_spec, allows_tier1
from rob_box_dialog.chats import ChatPorts, ChatTurns, channel_ref, chat_of
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine, outcome_phrase


class Base:
    """Общие порты хоста: ответы LLM по очереди, исходы действий по имени тула."""

    def __init__(self, answers=(), outcomes=None):
        self.answers, self.outcomes = list(answers), outcomes or {}
        self.asked, self.executed, self.intents = [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([a.tool for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                             *self.outcomes.get(a.tool, ("done", None)))
                for n, a in enumerate(actions)]

    def intent(self, turn_id, epoch, kind):
        self.intents.append(kind)


class Host:
    """Хост: движок на чат — как ``dialog_node.build_chats``; ``sent`` — опубликованные ``DialogOutput``."""

    def __init__(self, answers=(), outcomes=None):
        self.base = Base(answers, outcomes)
        self.sent, self.logs = [], []
        self.chats = ChatTurns(self.make, ("робот",), self.logs.append)

    def make(self, chat_id):
        session = Session("telegram", session_id=channel_ref(chat_id))
        ports = ChatPorts(self.base, chat_id, lambda ref, text, sid: self.sent.append((ref, text)), self.logs.append)
        return TurnEngine(session, ports, system_prompt="Ты ассистент в чате.")

    def say(self, chat_id, text):
        decision = self.chats.decide(chat_id, text, 0.0)
        if isinstance(decision, Addressed):
            return asyncio.run(self.chats.run(chat_id, decision))
        return decision


def test_channel_ref_round_trip_and_foreign_channel():
    assert channel_ref(495039871) == "telegram:495039871"
    assert chat_of("telegram:495039871") == 495039871
    assert chat_of("headset:1") is None and chat_of("telegram:x") is None and chat_of(None) is None


def test_session_id_is_the_chat_channel():
    assert Session("telegram", session_id="telegram:7").snapshot.session_id == "telegram:7"
    assert len(Session("personality").snapshot.session_id) == 32


def test_coordinator_call_is_not_addressed_and_runs_nothing():
    host = Host()
    decision = host.say(1, "Клод привет")
    assert decision == NotAddressed(COORDINATOR)
    assert host.base.asked == [] and host.sent == []


def test_chat_text_is_addressed_without_wake_and_wake_is_stripped():
    host = Host()
    assert host.chats.decide(1, "который час", 0.0) == Addressed("который час", "telegram")
    assert host.chats.decide(1, "Робот, который час", 0.0) == Addressed("который час", "telegram")


def test_reply_goes_to_its_chat_as_text_and_into_the_chat_history():
    host = Host([Answer(Command(speech="Привет!"), "deepseek"), Answer(Command(speech="Помню."), "deepseek")])
    host.say(42, "привет")
    host.say(42, "помнишь, что я сказал?")
    assert host.sent == [("telegram:42", "Привет!"), ("telegram:42", "Помню.")]
    second = host.base.asked[1]
    assert {"role": "assistant", "content": "Привет!"} in second  # текст доставлен целиком — сразу в окне


def test_second_chat_has_its_own_session_and_history():
    host = Host([Answer(Command(speech="Это чат 1."), "deepseek"), Answer(Command(speech="Это чат 2."), "deepseek")])
    host.say(1, "секрет первого чата")
    host.say(2, "привет")
    assert host.chats.engine(1) is not host.chats.engine(2)
    assert host.chats.engine(2).session.snapshot.session_id == "telegram:2"
    assert not any("секрет первого чата" in str(m.get("content")) for m in host.base.asked[1])
    assert host.sent[1] == ("telegram:2", "Это чат 2.")


def test_command_outcome_phrase_and_speech_both_go_to_chat_not_speakers():
    tool = "save_waypoint"
    cmd = Command(speech="Записываю.", actions=(ToolCall(tool, {"name": "кухня"}),))
    host = Host([Answer(cmd, "deepseek")])
    host.say(5, "запомни точку: это кухня")
    assert host.base.executed == [[tool]]
    result = ActionResult("x/0", 0, "telegram", tool, kn.TOOL_CLASSES[tool], "done")
    texts = [text for ref, text in host.sent]
    assert all(ref == "telegram:5" for ref, _ in host.sent)
    assert texts == [outcome_phrase([result]), "Записываю."]  # фраза кода по событию, затем текст модели


def test_failed_command_is_reported_honestly_in_chat():
    tool = "save_waypoint"
    host = Host([Answer(Command(actions=(ToolCall(tool, {"name": "кухня"}),)), "deepseek")],
                outcomes={tool: ("rejected", None)})
    host.say(5, "запомни точку кухня")
    failed = ActionResult("x/0", 0, "telegram", tool, kn.TOOL_CLASSES[tool], "rejected")
    assert host.sent == [("telegram:5", outcome_phrase([failed]))]


def test_silence_words_in_chat_are_not_tier1_silence_of_the_chat():
    host = Host([Answer(Command(speech="Хорошо, молчу в голосе не я."), "deepseek")])
    host.say(3, "замолчи")
    assert host.base.asked, "«замолчи» в чате — слова для LLM телеграма, а не кадр Silence (решение О2)"
    assert host.chats.engine(3).session.snapshot.top("Silence") is None


def test_stop_in_chat_is_tier1_without_llm_and_stops_motion():
    host = Host()
    host.say(3, "стоп")
    assert host.base.asked == [] and host.base.intents == ["stop_motion"]


def test_tier1_groups_cover_every_grammar_kind_and_follow_agent_specs():
    assert set(TIER1_GROUPS) == set(kn.TIER1_KINDS)
    assert all(allows_tier1(agent_spec("personality"), kind) for kind in kn.TIER1_KINDS)
    telegram = agent_spec("telegram")
    assert allows_tier1(telegram, "stop_all") and allows_tier1(telegram, "new_session")
    assert not allows_tier1(telegram, "silence") and not allows_tier1(telegram, "volume")


def test_failed_turn_is_logged_loudly_not_swallowed():
    host = Host()

    async def boom(addressed):
        raise RuntimeError("llm down")

    host.chats.engine(9).run = boom
    assert asyncio.run(host.chats.run(9, Addressed("привет", "telegram"))) is None
    assert any("ход упал: RuntimeError: llm down" in line for line in host.logs)


def test_chat_ports_do_not_touch_robot_speech_or_performance():
    base = Base()
    ports = ChatPorts(base, 1, lambda *a: None, lambda line: None)
    assert ports.stop_perform("halt") is False
    assert ports.cancel_speech(0, "x") is None
    ports.intent("t", 0, "stop_motion")
    assert base.intents == ["stop_motion"]
