"""#4090 — фраза кода владельца из данных исхода звучит в диалоге v2 (ADR-0148: фразу об исполнении строит код).

Живая проверка 27.10 18:58 UTC: «Робот, сыграй метал на Ренардо.» — синт заиграл брейкбит вместо метала, тул положил в
данные исхода фразу кода «Метал на синте играю ближайшим стилем — брейкбит.» (``source_note``), а вслух прозвучало
только «Включаю музыку.»: фраза исхода брала класс тула и не смотрела в данные. Ход целиком на фейковых портах:
исполнитель отдаёт результат как настоящий (класс ТУЛА, данные = аргументы вызова поверх ответа тула).
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine, say_late

NEAREST = "Метал на синте играю ближайшим стилем — брейкбит."
FALLBACK = "Сгенерировать сейчас не могу: нет ключа сервиса генерации, играю своё."


class Ports:
    """Транспорт хода: ответы LLM по очереди, исходы действий по имени тула ``(статус, причина, данные тула)``."""

    def __init__(self, answers=(), outcomes=None):
        self.answers = list(answers)
        self.outcomes = outcomes or {}
        self.said = []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, a in enumerate(actions):
            status, reason, data = self.outcomes.get(a.tool, ("done", None, {}))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                                    status, reason, {**dict(a.args), **data}))
        return out

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


def _engine(ports):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))


def _music_turn(tool, args, outcome, text="Робот, сыграй метал на Ренардо."):
    ports = Ports([Answer(Command(actions=(ToolCall(tool, args),)), "deepseek")], {tool: outcome})
    asyncio.run(_engine(ports).run(Addressed(text, VOICE)))
    return ports.said


@pytest.mark.parametrize("tool,args", [
    ("request_music", {"intent": "track", "text": "сыграй метал на Ренардо", "genre": "rock", "engine": "synth"}),
    ("generate_music", {"kind": "instrumental", "style": "metal"}),
])
@pytest.mark.parametrize("note", [NEAREST, FALLBACK])
def test_note_of_the_music_owner_is_said_before_the_launch_phrase(tool, args, note):
    said = _music_turn(tool, args, ("done", None, {"ok": True, "style": "breaks", "source_note": note}))
    assert said == [f"{note} {phrases.text('media.done')}"]


def test_without_a_note_the_launch_phrase_is_alone():
    said = _music_turn("request_music", {"intent": "track", "text": "сыграй на Ренардо"}, ("done", None, {"ok": True}))
    assert said == [phrases.text("media.done")]


def test_note_of_a_launch_that_failed_is_not_said():
    """Трек не заиграл — «играю ближайшим стилем» было бы враньём: только фраза провала."""
    said = _music_turn("request_music", {"intent": "track", "text": "сыграй метал на Ренардо"},
                       ("rejected", None, {"ok": False, "source_note": NEAREST}))
    assert said == [phrases.text("media.rejected")]


def test_late_music_outcome_carries_its_note_too():
    """Музыка заиграла после срока хода (``on_late``, #3960) — поздняя фраза кода та же: с фразой владельца."""
    ports = Ports()
    say_late(_engine(ports), ActionResult("t9/0", 0, "personality", "request_music", kn.TOOL_CLASSES["request_music"],
                                          "done", None, {"source_note": NEAREST}))
    assert ports.said == [f"{NEAREST} {phrases.text('media.done')}"]


def test_whole_reply_of_the_owner_replaces_the_launch_phrase_and_beats_the_note():
    """Порядок как у ``media_phrases.launch_text`` v1: реплика целиком (трек кэша) — вместо фразы исхода, фраза
    ``note`` при ней не звучит."""
    cache = "Ставлю песню из сочинённого раньше: «Кот и ёж»."
    said = _music_turn("request_music", {"intent": "track", "text": "спой про кота"},
                       ("done", None, {"ok": True, "source_message": cache, "source_note": NEAREST}))
    assert said == [cache]


BRIDGE_NOTE = "Сгенерировать сейчас не могу: песня ещё пишется, играю своё."


@pytest.mark.parametrize("tool,args,data", [
    ("generate_music", {"kind": "robot_song"},
     {"ok": True, "pending": "song", "track_id": "bridge-1", "source": "synth", "style": "club",
      "source_message": "Пишу песню, пою голосом Russian_ReliableMan — минуты полторы. Пока играет своё.",
      "source_note": BRIDGE_NOTE}),
    ("request_music", {"intent": "track", "text": "сгенерируй метал"},
     {"ok": True, "pending": "instrumental", "track_id": "bridge-2", "style": "breaks",
      "source_message": "Сочиняю новый трек, метал — секунд 50. Пока играет своё — брейкбит.", "source_note": NEAREST}),
    ("generate_music", {"kind": "robot_song"},
     {"ok": True, "pending": "song", "source_message": "Пишу песню.", "source_note": BRIDGE_NOTE}),
])
def test_song_being_written_says_the_wait_phrase_of_the_dialog_table_not_the_owner_text(tool, args, data):
    """Ревью PR #4107: «песня пишется» — фраза таблицы диалога по данным (``knowledge.GEN_WAIT``, #4104), одна. Фраза
    владельца (``source_message`` «Пишу песню …» и ``source_note`` моста) её не подменяет и к ней не добавляется —
    иначе в v2 две реализации одной фразы."""
    said = _music_turn(tool, args, ("done", None, data), "сгенерируй новую песню")
    key = kn.GEN_WAIT.key(data)
    assert key and len(said) == 1 and said[0] in set(phrases.catalog()[key]), said
    assert data["source_message"] not in said[0] and data["source_note"] not in said[0]


def test_owner_text_fields_belong_to_action_tools_of_the_catalog():
    for tool, fields in kn.OUTCOME_OWNER_TEXT.items():
        assert kn.TOOL_CLASSES[tool].startswith("action.") and fields.whole and fields.note, tool
