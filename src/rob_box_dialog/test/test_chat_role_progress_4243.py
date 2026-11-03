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
