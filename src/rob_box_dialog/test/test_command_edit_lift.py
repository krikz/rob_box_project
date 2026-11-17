"""Правка выступления: с какого сегмента — решает код, поля правки не на месте не бракуют команду (§5.6.4 X3, #3700).

Корпус — команды DeepSeek из громкого прогона A20 (Sound Pi, 22.10, «Робот, и про енота» на ACTIVE=2), все три
провалили ход молча до этого фикса. Тексты сегментов сокращены, форма полей — как в логе ``🧾 [command]``.
"""

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Busy, Command, Invalid, parse, to_args, validate

ALL = [t.name for t in TOOL_CATALOG]
BUSY = Busy(frozenset({"action.perform"}), active_segment=2)
_SEG = ["А вот второй герой — енот-хулиган.", "Он моет лапы, будто шеф-повар в цеху."]
_RAP = {"kind": "rap", "backing": "beat", "segments": _SEG}

#: (прогон, ответ модели, переложено, отброшено). Вердикт до фикса — в комментарии.
LIVE_A20 = [
    # a20_1 07:26:15 unknown_field 'perform: edit_pending' → повтор: from_segment_without_edit '2'
    ("a20_1", {"perform": {**_RAP, "edit_pending": True, "from_segment": 2}},
     ("perform.edit_pending",), ("perform.from_segment",)),
    # a20_2 07:28:30 unknown_field 'perform: edit_pending' → повтор: from_segment_without_edit '3'
    ("a20_2", {"perform": {**_RAP, "edit_pending": True, "from_segment": 3}},
     ("perform.edit_pending",), ("perform.from_segment",)),
    # a20_3 07:31:53 unknown_field 'command: from_segment' → повтор: edit_started_segment 'from_segment=2, ACTIVE=2'
    ("a20_3", {"edit_pending": True, "from_segment": 2, "perform": _RAP}, (), ("command.from_segment",)),
]


@pytest.mark.parametrize("run, raw, moved, dropped", LIVE_A20, ids=[c[0] for c in LIVE_A20])
def test_live_a20_commands_are_edits_now(run, raw, moved, dropped):
    cmd = parse(raw)
    assert isinstance(cmd, Command), cmd
    assert cmd.edit_pending and cmd.perform.from_segment == 0  # номер выбирает код, не модель
    assert cmd.moved == moved and cmd.dropped == dropped
    assert isinstance(validate(cmd, ALL, busy=BUSY), Command)


def test_top_level_flag_wins_over_the_nested_one():
    cmd = parse({"edit_pending": False, "perform": {**_RAP, "edit_pending": True}})
    assert isinstance(cmd, Command) and not cmd.edit_pending and cmd.moved == ("perform.edit_pending",)


def test_edit_still_needs_a_running_performance():
    res = validate(parse(LIVE_A20[0][1]), ALL, busy=Busy())
    assert isinstance(res, Invalid) and res.reason == "edit_without_active_perform"


def test_schema_and_bridge_have_no_from_segment():
    assert "from_segment" not in kn.command_schema("json")["properties"]["perform"]["properties"]
    assert "from_segment" not in to_args(parse(LIVE_A20[1][1]))["perform"]


def test_prompt_example_of_an_edit_parses_as_a_valid_edit():
    """Пример правки в подсказке (только форма; решает структура) — сам валидная правка."""
    import json
    import re
    text = kn.prompt_fragment()
    example = json.loads(re.search(r"\{\"edit_pending\".*\]\}\}", text).group(0))
    cmd = parse(example)
    assert isinstance(validate(cmd, ALL, busy=BUSY), Command) and cmd.moved == () and cmd.dropped == ()


def test_command_without_edit_fields_is_untouched():
    cmd = parse({"perform": _RAP})
    assert isinstance(cmd, Command) and cmd.moved == () and cmd.dropped == () and not cmd.edit_pending


def test_add_beat_during_a_performance_is_a_valid_command_live_a22():
    """A22 22.10 a22_2: «Робот, добавь бит» во время рэпа без бита — ``perform`` с подложкой без куплетов (0) или с
    одним; раньше ``bad_segments`` дважды и ход провален. Вне выступления — по-прежнему от SEGMENTS_MIN."""
    empty = {"perform": {"kind": "rap", "backing": "beat", "segments": []}, "edit_pending": True}
    one = {"perform": {"kind": "rap", "backing": "beat", "segments": ["Бит пошёл, комар в такт."]}}
    for raw in (empty, one):
        assert isinstance(validate(parse(raw), ALL, busy=BUSY), Command), raw
    none = {"perform": {"kind": "rap", "backing": "none", "segments": []}}
    res = validate(parse(none), ALL, busy=BUSY)
    assert isinstance(res, Invalid) and res.reason == "bad_segments"  # ни куплетов, ни бита — нечего делать
    res = validate(parse(one), ALL, busy=Busy())
    assert isinstance(res, Invalid) and res.reason == "bad_segments"  # новое выступление — от SEGMENTS_MIN
