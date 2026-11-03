"""#4243: роль героя и подпись шага хода уходят из ``ChatPorts`` в чат вместе с репликой (ADR-0160 §2.4)."""

from __future__ import annotations

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.chats import PROGRESS_KIND, ChatPorts


class Base:
    def stage(self, *args, **kwargs):
        return None


def ports(sent, progress=None):
    def send(ref, text, speech_id, **extra):
        sent.append((ref, text, extra))

    return ChatPorts(Base(), 7, send, lambda line: None, progress=progress)


def test_hero_line_carries_the_role_to_the_chat_and_narrator_line_does_not():
    sent = []
    p = ports(sent)
    p.say("t1", 1, "Ты делал бэкап?", "Ермил", None)
    p.say("t1", 1, "Привет", None, None)
    assert sent == [("telegram:7", "Ты делал бэкап?", {"role": "Ермил"}), ("telegram:7", "Привет", {})]


def test_progress_label_comes_from_phrases_for_thinking_and_executing_only():
    labels = []
    p = ports([], progress=lambda ref, label: labels.append((ref, label)))
    for stage in ("addressed", "thinking", "executing", "speaking"):
        p.stage("t1", 1, stage, "started")
    p.stage("t1", 1, "thinking", "finished")
    assert labels == [("telegram:7", phrases.text("chat.progress.thinking")),
                      ("telegram:7", phrases.text("chat.progress.executing"))]
    assert kn.CHAT_PROGRESS_STAGES == ("thinking", "executing") and PROGRESS_KIND == "progress"


def test_no_progress_callback_means_no_progress_and_no_failure():
    ports([]).stage("t1", 1, "thinking", "started")


# --- шаги хода: тулы и их исходы человеческими словами (#4243b) ---------------------------------------------------

import asyncio  # noqa: E402
from types import SimpleNamespace  # noqa: E402

from rob_box_dialog.chats import step_label, step_outcome  # noqa: E402
from rob_box_dialog.command import ToolCall  # noqa: E402
from rob_box_dialog.respond import ActionResult  # noqa: E402


class Exec:
    def __init__(self, status):
        self.status = status

    async def execute(self, actions, snapshot, turn_id):
        return [ActionResult(f"{turn_id}/{n}", 1, "telegram", a.tool, kn.TOOL_CLASSES[a.tool], self.status)
                for n, a in enumerate(actions)]


def test_every_tool_class_has_a_step_label_and_own_label_wins():
    for cls in set(kn.TOOL_CLASSES.values()):
        assert f"chat.step.{cls}" in phrases.catalog(), cls
    assert step_label("search_web") == phrases.text("chat.tool.search_web")
    assert step_label("set_volume") == phrases.text("chat.step.action.local")


def test_tool_steps_stream_start_then_outcome_and_never_carry_arguments():
    steps = []
    p = ChatPorts(Exec("done"), 7, lambda *a, **k: None, lambda line: None, progress=lambda ref, label: steps.append(label))
    call = ToolCall("search_web", {"query": "пароль hunter2 sk-secret"})
    asyncio.run(p.execute([call], SimpleNamespace(epoch=1, agent="telegram"), "t1"))
    assert steps == ["Ищу в интернете…", "Ищу в интернете - готово"]
    assert not any("hunter2" in s or "sk-" in s for s in steps)


def test_failed_tool_is_reported_as_failed_and_pending_has_no_outcome_yet():
    assert step_outcome("search_web", "rejected") == "Ищу в интернете - не вышло"
    assert step_outcome("search_web", "timeout") == "Ищу в интернете - не вышло"
    assert step_outcome("search_web", "pending") is None


def test_no_progress_callback_means_execute_still_works():
    p = ChatPorts(Exec("done"), 7, lambda *a, **k: None, lambda line: None)
    out = asyncio.run(p.execute([ToolCall("get_current_time", {})], SimpleNamespace(epoch=1, agent="telegram"), "t1"))
    assert out[0].status == "done"
