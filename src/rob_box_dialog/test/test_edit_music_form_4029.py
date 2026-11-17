"""#4029 — правка играющего в диалоге v2: форма хода, одновременность с выступлением и фраза исхода из результата.

Кейс Шифу 27.10: «сделай бит потяжелее, более металлический!» посреди рэпа — правка текущего бита, выступление не
прерывается и держит сетку, генерации нет, отдельной фразы нет. Хост решает путь хода ``edit:<вид>`` до модели
(``rob_box_music.track_edit.edit_route``); здесь — что форма закрывает и требует, как правка ложится на выступление
(``plan.composition``) и что говорит код по результату ``edit_music`` (``respond``).
"""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.agents import agent_spec, tools_for
from rob_box_dialog.command import Command, Form, parse, validate
from rob_box_dialog.plan.composition import compose, kind_of
from rob_box_dialog.respond import ActionResult, audible, phrase_from_result, phrase_key
from rob_box_dialog.respond import quiet_in_performance as heard_quiet
from rob_box_dialog.turn import form_tools

FORM = Form(passed=False, voice=False, sing="edit:heavier", sing_over=True)
TOOLS = (
    "stop_music",
    "get_music_state",
    "request_music",
    "dj_set",
    "music_library",
    "generate_music",
    "speak_text",
    "memory_save",
)


def _result(status, reason=None, **data):
    return ActionResult("turn-1", 0, "personality", "edit_music", kn.TOOL_CLASSES["edit_music"], status, reason, data)


def test_edit_route_is_its_own_family_that_closes_new_music_and_needs_the_edit():
    assert kn.sing_family("edit:heavier") == kn.sing_family("edit:faster", over_music=True) == "edit"
    form = FORM.sing_form
    assert set(form.closed_tools) == {"request_music", "generate_music", "dj_set"}
    assert form.needs_tool == kn.EDIT_TOOL and not form.needs_perform and not form.backings
    assert set(kn.sing_kinds("edit:heavier")) == set(kn.PERFORM_KINDS)  # выступление форма правки не сужает


def test_turn_tools_of_the_edit_form_have_the_edit_and_no_new_order():
    tools = [t.name for t in form_tools([_entry(n) for n in TOOLS], FORM, "personality")]
    assert kn.EDIT_TOOL in tools and not {"request_music", "generate_music", "dj_set"} & set(tools)
    assert kn.EDIT_TOOL in tools_for(agent_spec("personality"))
    assert all(kn.EDIT_TOOL not in g.tools for g in kn.TOOL_GROUPS.values())  # группа music — те же ≤ 6 схем


def _entry(name):
    from rob_box_core.tool_catalog import get_tool  # noqa: PLC0415

    return get_tool(name)


@pytest.mark.parametrize(
    "answer",
    [
        {"actions": [{"tool": "request_music", "args": {"intent": "track", "text": "металл", "genre": "rock"}}]},
        {"actions": [{"tool": "generate_music", "args": {"kind": "instrumental", "style": "metal"}}]},
        {"actions": [{"tool": "dj_set", "args": {"action": "start", "theme": "металл"}}]},
        {"speech": "Сейчас сделаю потяжелее!"},
    ],
)
def test_new_music_or_words_instead_of_the_edit_are_a_violation(answer):
    tools = (*TOOLS, kn.EDIT_TOOL)
    assert validate(parse(answer), tools, form=FORM).reason == kn.VIOLATION_SING_ROUTE


def test_the_edit_passes_the_form():
    tools = (*TOOLS, kn.EDIT_TOOL)
    cmd = validate(parse({"actions": [{"tool": "edit_music", "args": {"change": "heavier"}}]}), tools, form=FORM)
    assert isinstance(cmd, Command) and [a.tool for a in cmd.actions] == ["edit_music"]


def test_the_edit_does_not_replace_the_performance_nor_its_beat():
    """Строка ``music`` вытеснила бы выступление на такте (кейс 27.10: рэп без тактов); правка — рядом со всем."""
    assert kind_of("edit_music") == "query"
    decision = compose(kind_of("edit_music"), ("perform", "backing", "speech"))
    assert decision.policy == "MERGE" and not decision.replaces and not decision.waits_for
    assert compose(kind_of("request_music"), ("perform",)).replaces == ("perform",)  # было так


def test_beat_edit_under_the_performance_is_heard_not_said():
    """Шифу 27.10: «просто входящий запрос на изменение текущего трека» — фразы поверх рэпа нет (тот же механизм, что
    у «Включаю музыку.» посреди выступления: ``knowledge.QUIET_IN_PERFORMANCE_KEYS``, #4075)."""
    heard = _result("done", heard=True, message="Сделал звук тяжелее.")
    assert phrase_key(heard) == "music.edit_heard" and heard_quiet(heard)
    assert audible([heard], speaks=False, performing=True) == []
    # вне выступления (бит оставили играть) — говорится; предел и отказ под выступлением — тоже
    assert phrase_from_result(heard).text == "Сделал звук тяжелее."
    # трек, который человек поставил до рэпа (28.10 01:20): бит выступления не при чём - слышна сама и эта правка
    persons = _result("done", heard=True, backing=False, message="Сделал звук тяжелее.")
    assert heard_quiet(persons) and audible([persons], speaks=False, performing=True) == []
    assert audible([persons], speaks=False, performing=False) == [persons]
    limit = _result("done", heard=False, message="Тяжелее этот бит уже не сделаю.")
    refused = _result("rejected", "edit_backing_tempo", message="Под выступление темп не меняю.")
    assert audible([limit, refused], speaks=False, performing=True) == [limit, refused]


def test_edit_outcome_phrase_is_the_message_of_the_result():
    said = phrase_from_result(_result("done", heard=False, message="Ускорил до 149 BPM."))
    assert said.text == "Ускорил до 149 BPM." and said.emotion != "sad"
    refused = phrase_from_result(
        _result(
            "rejected",
            "edit_file",
            message="Это готовая запись — её темп и звук на " "ходу не изменю. Могу сделать музыку громче или тише.",
        )
    )
    assert refused.text.startswith("Это готовая запись") and refused.emotion == "sad"
    assert phrase_from_result(_result("rejected", "error")).text == "Не получилось поменять музыку — играет как было."
    assert "такта" in phrase_from_result(_result("timeout")).text
    assert "Включаю" not in phrase_from_result(_result("done", message="Добавил баса.")).text
