"""Озвучка из чата Telegram (ADR-0150 §5.1, §9.2.3, PR-12b #3707): ``say_aloud`` с итогом по ``SpeechEvent``.

Эталон: «сочини историю и расскажи» — в чат уходит текст истории, динамики читают её, затем «Озвучил (N с)» по событию
``finished``, а не по публикации запроса. Пользовательская тишина Личности ей не мешает (О2), ``OperatorHold`` — отказ.
"""

import asyncio
from types import SimpleNamespace

from rob_box_core import speech_client
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import Addressed
from rob_box_dialog.agents import agent_spec, tools_for
from rob_box_dialog.chats import ChatPorts, ChatTurns, channel_ref
from rob_box_dialog.command import Command, Invalid, ToolCall, validate
from rob_box_dialog.execute import Executor, OwnerEvent
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import aloud_named, available_tools
from rob_box_dialog.turn import Answer, TurnEngine

STORY = "Жил-был робот. Он любил стрижей."
KIND_STARTED, KIND_FINISHED, KIND_CANCELLED, KIND_REJECTED = 2, 4, 5, 7


class Base:
    """Общие порты хоста: ответ LLM, настоящий ``Executor`` и владелец речи, отвечающий ``pending`` с ``event_ref``."""

    def __init__(self, command, reply=("pending", None)):
        self.command, self.reply = command, reply
        self.requests = []
        self.executor = Executor(self._send, on_late=lambda r: self.on_late(r))
        self.on_late = lambda result: None

    async def ask(self, messages, tools, timeout_s):
        return Answer(self.command, "stub", 1, 10, lambda data: ())

    async def execute(self, actions, snapshot, turn_id):
        return await self.executor.run(actions, snapshot, turn_id)

    async def _send(self, request):
        self.requests.append(request)
        status, reason = self.reply
        speech_id = f"sp-{len(self.requests)}"
        self.executor.on_event(OwnerEvent(request.turn_id, status, reason, {"seconds": 0},
                                          event_ref=speech_id if status == "pending" else None))

    def intent(self, turn_id, epoch, kind):
        return None


class Host:
    """Хост чата, как ``dialog_node.build_chats``: исполнитель усыновлён, поздний итог — в ``ChatPorts.late``."""

    def __init__(self, command, reply=("pending", None), hold=False):
        self.base = Base(command, reply)
        self.sent = []
        self.hold = hold
        self.chats = ChatTurns(self._make, ("робот",))

    def _make(self, chat_id):
        session = Session("telegram", session_id=channel_ref(chat_id))
        ports = ChatPorts(self.base, chat_id, lambda ref, text, sid: self.sent.append((ref, text)),
                          hold=lambda: self.hold)
        self.base.on_late = ports.late
        self.chats.adopt(self.base.executor)
        return TurnEngine(session, ports, system_prompt="Ты ассистент в чате.")

    async def ask(self, text, then=()):
        """Ход чата; ``then`` — события речи, которые приходят после хода в том же цикле (ROS-поток хоста)."""
        decision = self.chats.decide(1, text, 0.0)
        assert isinstance(decision, Addressed)
        await self.chats.run(1, decision)
        results = [self.chats.on_speech(event) for event in then]
        await asyncio.sleep(0)
        return results


def _event(kind, speech_id="sp-1", reason="", seconds=0.0):
    return SimpleNamespace(kind=kind, speech_id=speech_id, reason=reason, seconds=seconds, agent="telegram")


def _story_command():
    return Command(speech=STORY, actions=(ToolCall("say_aloud", {"text": STORY}),))


def _run(coro):
    return asyncio.run(coro)


def test_story_goes_to_chat_and_confirmation_follows_the_finished_event():
    host = Host(_story_command())

    _run(host.ask("сочини историю про робота и расскажи", then=[_event(KIND_FINISHED, seconds=11.6)]))

    assert [r.args["text"] for r in host.base.requests] == [STORY]
    assert host.sent == [("telegram:1", STORY), ("telegram:1", "Озвучил (12 с).")]


def test_no_confirmation_before_the_event_only_the_story():
    host = Host(_story_command())

    _run(host.ask("расскажи историю"))

    assert host.sent == [("telegram:1", STORY)]


def test_rejected_late_event_says_the_owner_reason_not_a_success():
    host = Host(_story_command())

    _run(host.ask("расскажи историю", then=[_event(KIND_REJECTED, reason="unsupported_script")]))

    assert host.sent[-1] == ("telegram:1", "Не озвучил: выбранный голос не читает эти буквы.")


def test_cancelled_late_event_says_interrupted():
    host = Host(_story_command())

    _run(host.ask("расскажи историю", then=[_event(KIND_CANCELLED)]))

    assert host.sent[-1] == ("telegram:1", "Не озвучил: перебили.")


def test_immediate_rejection_of_the_owner_is_reported_by_code():
    host = Host(_story_command(), reply=("rejected", "busy"))

    _run(host.ask("расскажи историю"))

    assert host.sent[0] == ("telegram:1", "Не озвучил: динамики заняты.")
    assert host.sent[1] == ("telegram:1", STORY)


def test_operator_hold_rejects_without_asking_the_speech_owner():
    host = Host(_story_command(), hold=True)

    _run(host.ask("расскажи историю"))

    assert host.base.requests == []
    assert host.sent[0] == ("telegram:1", "Не озвучил: оператор занял робота.")


def test_user_silence_of_personality_does_not_block_say_aloud():
    """О2: кадр ``Silence`` — в сессии Личности; хост чата спрашивает только ``OperatorHold`` (hold=False)."""
    personality = Session("personality")
    personality.push_scene("Silence", "user")
    host = Host(_story_command(), hold=personality.snapshot.top("OperatorHold") is not None)

    _run(host.ask("расскажи историю", then=[_event(KIND_FINISHED, seconds=5)]))

    assert personality.snapshot.top("Silence") is not None
    assert len(host.base.requests) == 1
    assert host.sent[-1] == ("telegram:1", "Озвучил (5 с).")


def test_foreign_and_non_final_speech_events_are_ignored():
    host = Host(_story_command())

    results = _run(host.ask("расскажи историю", then=[_event(KIND_STARTED), _event(KIND_FINISHED, "other-id")]))

    assert results == [False, False]
    assert host.sent == [("telegram:1", STORY)]


def test_say_aloud_is_telegram_only_and_visible_only_on_aloud_words():
    snap = Session("telegram").snapshot
    assert "say_aloud" in tools_for(agent_spec("telegram"))
    assert "say_aloud" not in tools_for(agent_spec("personality"))
    assert "say_aloud" not in tools_for(agent_spec("operator"))
    names = lambda text, agent: {t.name for t in available_tools(snap, agent=agent, text=text)}  # noqa: E731
    assert "say_aloud" in names("сочини короткую историю про робота и расскажи", "telegram")
    assert "say_aloud" not in names("какая погода", "telegram")
    assert "say_aloud" not in names("расскажи вслух", "personality")
    assert aloud_named("прочитай вслух") and not aloud_named("который час")
    for phrase in ("агитируй вслух: поступайте на факультет", "продавай слона громко и агрессивно"):
        assert "say_aloud" in names(phrase, "telegram"), phrase


def test_validator_accepts_say_aloud_from_chat_with_speech_and_rejects_it_for_personality():
    cmd = _story_command()
    tools = {"say_aloud"}

    assert isinstance(validate(cmd, tools, Session("telegram").snapshot), Command)
    assert isinstance(validate(cmd, tools, Session("personality").snapshot), Invalid)


def test_say_aloud_is_an_action_with_a_speech_event_source():
    from rob_box_core.tool_catalog import get_tool

    assert kn.TOOL_CLASSES["say_aloud"] == "action.speak_aloud"
    assert get_tool("say_aloud").event_source == speech_client.SPEECH_EVENT_TOPIC
    assert kn.ACTION_CLASSES["action.speak_aloud"].agents == ("operator", "telegram")


def test_speech_owner_queue_keeps_replies_of_different_agents_in_order():
    """Реплика против реплики — очередь владельца речи (tts_node, FIFO), а не вытеснение: ``say_aloud`` не рвёт чужую."""
    from rob_box_dialog.plan.composition import compose, kind_of

    assert kind_of("say_aloud") == "speech"
    assert ("speech", "speech") in kn.OWNER_QUEUES
    assert compose("speech", ["speech"]).replaces == ()


def test_held_result_is_a_normal_action_result_with_the_catalog_phrase():
    from rob_box_dialog.respond import phrase_from_result

    result = ActionResult("t/0", 1, "telegram", "say_aloud", "action.speak_aloud", "rejected", "operator_hold")

    assert phrase_from_result(result).text == "Не озвучил: оператор занял робота."
