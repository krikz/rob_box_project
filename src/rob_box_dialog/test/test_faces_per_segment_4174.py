"""#4174 (n810, марафон-2 28.10): рэп отвергнут целиком из-за числа лиц по куплетам.

«Робот, а теперь под этот бит зачитай короткий рэп про енотика» -> ``invalid:bad_segments expression_per_segment не по
числу сегментов`` дважды -> «Не понял, как это сделать». Модель оба раза прислала верный рэп; лиц было не столько,
сколько куплетов. Лицо - косметика (ADR-0148, как #4138): число лиц подгоняет код (лишние отрезает, недостающие
дополняет последним), поправка - в ``Command.dropped`` (лог ``fields_dropped``), выступление исполняется. Роли и
голоса сегментов смысл несут - их длина по-прежнему ``bad_segments``; нет сегментов - ``bad_segments``, как раньше."""

import pytest

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog.command import Command, Invalid, parse, validate

ALL = [t.name for t in TOOL_CATALOG]
RAP = ["Енотик крадёт печенье", "Лапки быстрые как ветер", "Ночью он бежит по крышам", "А наутро спит в корзине"]


def _check(perform):
    return validate(parse({"perform": {"kind": "rap", "backing": "beat", **perform}}), ALL)


def _ok(perform):
    res = _check(perform)
    assert isinstance(res, Command), res
    return res


def _faces(*emotions):
    return [{"emotion": e} for e in emotions]


def test_4174_n810_four_verses_three_faces_is_performed_with_four_faces():
    cmd = _ok({"segments": RAP, "expression_per_segment": _faces("happy", "surprised", "sad")})
    assert cmd.perform.segments == tuple(RAP)
    assert [e.emotion for e in cmd.perform.expression_per_segment] == ["happy", "surprised", "sad", "sad"]
    assert cmd.dropped == ("perform.expression_per_segment[3->4]",)


def test_4174_three_verses_five_faces_is_performed_with_the_first_three():
    cmd = _ok({"segments": RAP[:3], "expression_per_segment": _faces("happy", "surprised", "sad", "angry", "neutral")})
    assert [e.emotion for e in cmd.perform.expression_per_segment] == ["happy", "surprised", "sad"]
    assert cmd.dropped == ("perform.expression_per_segment[5->3]",)


def test_4174_matching_count_is_untouched_and_nothing_is_dropped():
    cmd = _ok({"segments": RAP[:2], "expression_per_segment": _faces("happy", "sad")})
    assert [e.emotion for e in cmd.perform.expression_per_segment] == ["happy", "sad"] and cmd.dropped == ()


def test_4174_no_faces_stay_no_faces():
    cmd = _ok({"segments": RAP[:2]})
    assert cmd.perform.expression_per_segment is None and cmd.dropped == ()


def test_4174_empty_face_list_means_no_faces_not_a_rejection():
    cmd = _ok({"segments": RAP[:2], "expression_per_segment": []})
    assert cmd.perform.expression_per_segment is None
    assert cmd.dropped == ("perform.expression_per_segment[0->0]",)


def test_4174_padding_repeats_the_last_face_and_other_cosmetics_are_still_dropped():
    per = [{"emotion": "happy"}, {"emotion": "sad", "earcon": "none"}]
    cmd = _ok({"segments": RAP[:3], "expression_per_segment": per})
    assert [e.emotion for e in cmd.perform.expression_per_segment] == ["happy", "sad", "sad"]
    assert all(e.earcon is None for e in cmd.perform.expression_per_segment)
    assert set(cmd.dropped) == {"perform.expression_per_segment[2->3]", "perform.expression_per_segment[1].earcon",
                                "perform.expression_per_segment[2].earcon"}


def test_4174_fitted_command_survives_a_second_validation_unchanged():
    first = _ok({"segments": RAP, "expression_per_segment": _faces("happy")})
    again = validate(first, ALL)
    assert isinstance(again, Command) and again.perform == first.perform and again.dropped == first.dropped


@pytest.mark.parametrize("segments", [[], ["один"], ["а"] * 13, ["куплет", ""]])
def test_4174_bad_segments_still_reject_whatever_the_faces(segments):
    res = _check({"segments": segments, "expression_per_segment": _faces("happy", "sad", "angry")})
    assert isinstance(res, Invalid) and res.reason == "bad_segments", res


def test_4174_faces_are_expressions_with_enum_still_checked():
    res = _check({"segments": RAP, "expression_per_segment": _faces("happy", "ecstatic")})
    assert isinstance(res, Invalid) and res.reason == "bad_enum", res
