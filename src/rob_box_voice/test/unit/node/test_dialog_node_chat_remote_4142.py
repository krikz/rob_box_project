"""#4142: хвосты пульта робота из чата Telegram в ROS-хосте ``dialog_node`` (после PR #4139).

* «Спой» из чата решает тот же ``SingRouteHost``, что голос Личности (решение Шифу 28.10): «с Suno» — генерация;
* музыка робота — в сессии чата (путь пения и срез судят по ней);
* лицо из чата в тишине Личности не уходит лицу робота: в чат — фраза кода о причине, не «Показываю»;
* ``block_end`` хоста выступления закрывает запись «идёт» и в окне чата.
"""

from __future__ import annotations

import json
import sys
import types
from unittest.mock import MagicMock

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

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog import phrases  # noqa: E402
from rob_box_dialog.command import current_form  # noqa: E402
from rob_box_dialog.plan.plan_events import PlanEvent  # noqa: E402
from rob_box_llm.provider import LLMResponse, ToolCall, ToolResult  # noqa: E402
from rob_box_voice import dialog_node as dn  # noqa: E402
from rob_box_voice.core import plan_host  # noqa: E402

TIME = type(M.TurnEvent().stamp)
BOAR = ["Вот идёт кабан по грязи, хрюкает на всю округу", "Роллинг ап зе Хилл — кабан летит с холма"]


class _LLM:
    """LLM чата: ответы по очереди; что модель видела — форма хода и тулы."""

    def __init__(self, responses):
        self.responses, self.calls, self.seen = list(responses), 0, []
        self.answered_by = "deepseek"

    async def complete(self, messages, tools=(), settings=None):
        self.calls += 1
        self.seen.append((current_form(), [t["function"]["name"] for t in tools]))
        return self.responses.pop(0)


class _Tools:
    def __init__(self):
        self.calls = []

    async def execute(self, call):
        self.calls.append((call.name, dict(call.arguments)))
        return ToolResult(tool_call_id=call.id, content="ok", data={"status": "done"})


class _PerformHost:
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
    made.test_tools, made.test_llm = tools, chat_llm
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
SUNO = {"actions": [{"tool": "generate_music", "args": {"prompt": "песня про кабана"}}]}
SHOW = {"actions": [{"tool": "play_animation", "args": {"animation": "victory"}}]}


@pytest.mark.parametrize("made", [SING], indirect=True)
def test_chat_and_personality_share_one_sing_route_host(made):
    """Одно место решения пути пения (#4079): у чата тот же хост, что у Личности, а не второй."""
    _chat(made, "привет")

    assert made._chats.engine(42)._sing is made._engine._sing is not None


@pytest.mark.parametrize("made", [SUNO], indirect=True)
def test_sing_with_suno_from_telegram_takes_the_generate_route_of_the_voice_path(made):
    """«Спой с Suno …» из чата — путь ``generate`` кода (маркер «suno» общей таблицы), как голосом: форма хода требует
    ``generate_music``, выступление-песня закрыто."""
    _chat(made, "спой с Suno песню про кабана")

    form, tools = made.test_llm.seen[0]
    assert form.sing.startswith("generate:") and form.sing_form is kn.SING_FORMS["generate"]
    assert "generate_music" in tools
    assert made.test_tools.calls[0][0] == "generate_music"


@pytest.mark.parametrize("made", [SING], indirect=True)
def test_robot_music_state_reaches_the_chat_session(made):
    _chat(made, "привет")
    msg = MagicMock()
    msg.data = json.dumps({"state": "playing", "track_id": "t9"})

    made._on_music_state(msg)

    assert made._chats.engine(42).session.snapshot.music_playing, "чат — пульт робота: музыка робота в его снимке"


@pytest.mark.parametrize("made", [SHOW], indirect=True)
def test_face_from_telegram_in_personality_silence_is_refused_by_code_not_claimed(made):
    """Тишина Личности (кадр ``Silence``): face_node ответил бы ``skipped{silenced}``. Лицо лицу робота не уходит,
    в чат — фраза кода о причине, а не «Показываю на роботе: …» (ADR-0148)."""
    made._session.push_scene("Silence", "user")

    _chat(made, "покажи победу")

    assert _published(made, "/voice/expression/request") == []
    out = [o.text for o in _published(made, "/dialog/output")]
    assert out == [phrases.text(f"{kn.REMOTE_SHOW_REJECTED_PHRASE}.{kn.REMOTE_SILENCED}")]


@pytest.mark.parametrize("made", [SHOW], indirect=True)
def test_face_from_telegram_outside_silence_is_named_in_russian(made):
    _chat(made, "покажи победу")

    faces = _published(made, "/voice/expression/request")
    assert [(f.agent, f.emotion) for f in faces] == [("personality", "victory")]
    assert [o.text for o in _published(made, "/dialog/output")] == ["Показываю на роботе: победа."]


@pytest.mark.parametrize("made", [SING], indirect=True)
def test_block_end_of_the_robot_performance_closes_it_in_the_chat_window(made):
    """``block_end`` хоста выступления раньше шёл только в журнал Личности: окно чата держало «выступление идёт»."""
    _chat(made, "спой на синтезаторе песенку про кабана")
    log = made._chats.engine(42).log
    turn_id = made._tool_executor.performer.started[0][1]
    assert log.performing()

    plan_host.note_perform_end(made, PlanEvent("block_end", f"perform-{turn_id}", turn_id=turn_id, reason="done",
                                               segment_no=2))

    assert not log.performing()
