"""#4040: чат Telegram — пульт робота в ROS-хосте ``dialog_node`` (проводка ``RobotRemote``, решение Шифу 28.10).

Живой робот 26.10 (dev-0d29158): «спой песенку про кабана…» из чата → ``perform`` ушёл в чат пятью сообщениями
текстом, вслух ничего — ``ChatPorts.perform`` возвращал ``None``. Теперь выступление агента чата берёт хост выступления
Личности в её эпохе (голос робота), лицо — лицо робота без стадии хода; в ``/dialog/output`` — фраза кода, не сегменты.
"""

from __future__ import annotations

import sys
import types

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
try:  # настоящий rosidl-пакет (CI) или заглушка сервиса DialogControl (Windows)
    import rob_box_dialog_msgs.srv  # noqa: F401
except ImportError:
    class _Request:
        ACTION_HOLD, ACTION_RESUME, ACTION_RESET = 0, 1, 2

        def __init__(self, action=0, set_by="", ttl_s=0.0):
            self.action, self.set_by, self.ttl_s = action, set_by, ttl_s

    _srv = types.ModuleType("rob_box_dialog_msgs.srv")
    _srv.DialogControl = type("DialogControl", (), {"Request": _Request,
                                                    "Response": lambda: types.SimpleNamespace()})
    sys.modules["rob_box_dialog_msgs.srv"] = _srv
    sys.modules["rob_box_dialog_msgs"].srv = _srv

from rob_box_llm.provider import LLMResponse, ToolCall, ToolResult  # noqa: E402
from rob_box_voice import dialog_node as dn  # noqa: E402

TIME = type(M.TurnEvent().stamp)
BOAR = ["Вот идёт кабан по грязи, хрюкает на всю округу", "Роллинг ап зе Хилл — кабан летит с холма"]


class _LLM:
    def __init__(self, responses):
        self.responses, self.calls = list(responses), 0
        self.answered_by = "deepseek"

    async def complete(self, messages, tools=(), settings=None):
        self.calls += 1
        return self.responses.pop(0)


class _Tools:
    def __init__(self):
        self.calls = []

    async def execute(self, call):
        self.calls.append((call.name, dict(call.arguments)))
        return ToolResult(tool_call_id=call.id, content="ok", data={"status": "done"})


class _PerformHost:
    """Хост выступления Личности (``PerformHost``): что и в какой эпохе ему отдали."""

    def __init__(self):
        self.started = []

    def start(self, perf, turn_id, epoch=0):
        self.started.append((perf, turn_id, epoch))
        return {"status": "pending", "block_id": f"perform-{turn_id}"}

    def edit(self, perf):
        return {"status": "done"}


def _node(monkeypatch, command):
    chat_llm = _LLM([LLMResponse(content="", tool_calls=(ToolCall(id="c1", name="command", arguments=command),))])
    tools = _Tools()
    monkeypatch.setattr(dn, "build_llm", lambda node, counter, agent="personality": (
        (chat_llm, "Ты пульт робота.") if agent == "telegram" else (_LLM([]), "Ты робот.")))
    monkeypatch.setattr(dn, "build_tools", lambda node: types.SimpleNamespace(execute=tools.execute))
    monkeypatch.setattr(dn, "build_chat_tools", lambda node, personality_tools: tools)
    made = dn.DialogNode()
    stamp = TIME(sec=1, nanosec=0)
    made.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))
    made._tool_executor.performer = _PerformHost()
    made.test_tools = tools
    return made


@pytest.fixture()
def made(request, monkeypatch):
    node = _node(monkeypatch, request.param)
    yield node
    node.shutdown()


def _chat(made, text, chat_id=42):
    msg = M.Utterance()
    msg.text, msg.source, msg.chat_id, msg.provider = text, M.Utterance.SOURCE_TELEGRAM, chat_id, "telegram"
    made._on_utterance(msg)
    made.last_chat_turn.result(timeout=10)


def _published(made, topic):
    pubs = [p for t, p in made._publishers if t == topic]
    return [c.args[0] for p in pubs for c in p.publish.call_args_list]


SING = {"perform": {"kind": "song", "backing": "beat", "segments": BOAR}}
SHOW = {"actions": [{"tool": "play_animation", "args": {"animation": "rainbow"}}]}


@pytest.mark.parametrize("made", [SING], indirect=True)
def test_sing_from_telegram_is_performed_by_the_robot_in_the_personality_epoch(made):
    made._session.replace_epoch()
    made._session.replace_epoch()  # эпоха Личности 2, у чата своя (0): выступление — в эпохе Личности
    epoch = made._session.snapshot.epoch

    _chat(made, "спой песенку про кабана в стиле Роллинг ап зе Хилл")

    started = made._tool_executor.performer.started
    assert [(list(p.segments), e) for p, _, e in started] == [(BOAR, epoch)]
    out = [(o.agent, o.channel_ref, o.text) for o in _published(made, "/dialog/output")]
    assert out == [("telegram", "telegram:42", "Пою на роботе: «Вот идёт кабан по грязи, хрюкает на всю округу»")]
    assert not any(BOAR[1] in text for _, _, text in out), "текст песни в чат не идёт"
    assert _published(made, "/voice/speech/request") == [], "служебной речи агента чата в динамиках нет"


@pytest.mark.parametrize("made", [SHOW], indirect=True)
def test_show_from_telegram_lands_on_the_robot_face_without_a_turn_event(made):
    _chat(made, "покажи радугу")

    assert made.test_tools.calls == [("play_animation", {"animation": "rainbow"})]
    faces = _published(made, "/voice/expression/request")
    assert [(f.agent, f.emotion, f.earcon) for f in faces] == [("personality", "rainbow", "")]
    assert [o.text for o in _published(made, "/dialog/output")] == ["Показываю на роботе: rainbow."]
    assert _published(made, "/dialog/turn_event") == [], "хода у робота нет: кольцо и метрики хода не трогаются"
    assert _published(made, "/voice/speech/request") == []
