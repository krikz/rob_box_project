"""Хвост ревью PR #4132 (#3975): ``heard`` правки играющего ловят мутации ``plan.limited`` и ``plan.refused``.

``heard`` — «правка встала на такте целиком, без предела и отказа»: только тогда её исход слышен сам, и фраза молчит
посреди выступления (``knowledge.QUIET_IN_PERFORMANCE_KEYS``). Предел или отказ говорятся всегда. Раньше тесты не
доходили до смешанного исхода («сделано одно, другое упёрлось в предел»): выкинуть ``plan.limited`` из условия было
можно без красного теста — «Ускорил до 180 BPM.» молчало бы, а предел «быстрее некуда» терялся.
"""

import pytest

from rob_box_dialog import knowledge as dkn
from rob_box_dialog.respond import audible, phrase_key, quiet_in_performance
from rob_box_mcp_tools.engine.track_edit import TrackEditor
from rob_box_music import knowledge as kn
from rob_box_music.track_edit import SYNTH, EditPlan, EditTarget, plan_edit

from .test_edit_music_4029 import _as_result

pytestmark = pytest.mark.unit

TARGET = EditTarget(SYNTH, bpm=120.0)


def _data(applied=(), limited=(), kept=(), refused=""):
    plan = EditPlan(("heavier", "faster"), 120.0, "", 0.0, tuple(applied), tuple(limited), tuple(kept), refused)
    return TrackEditor._result(plan, TARGET, "req00001", "")  # noqa: SLF001


@pytest.mark.parametrize(
    "applied,limited,kept,refused,heard",
    [
        (("heavier",), (), (), "", True),  # встала целиком
        (("heavier", "faster"), (), (), "", True),  # обе встали
        (("faster",), ("heavier",), (), "", False),  # одно сделано, другое - предел: предел говорится
        (("faster",), (), (), "failed", False),  # отказ при сделанном
        (("faster",), ("heavier",), (), "failed", False),
        ((), ("heavier",), (), "", False),  # одни пределы
        ((), (), ("develop",), "", False),  # «развивай»: играю дальше, менять нечего, слышать нечего
        ((), (), (), "", False),
    ],
)
def test_heard_only_for_a_whole_edit_without_a_limit_or_a_refusal(applied, limited, kept, refused, heard):
    assert _data(applied, limited, kept, refused)["heard"] is heard


def _from_planner(kinds, **target):
    """Исход настоящего планировщика правки (``plan_edit``) глазами тула - без рук составленного плана."""
    plan = plan_edit(kinds, EditTarget(SYNTH, style=kn.DEFAULT_STYLE, **target))
    return TrackEditor._result(plan, EditTarget(SYNTH, **target), "req00001", "")  # noqa: SLF001


def test_a_limit_next_to_a_done_edit_is_said_even_under_the_performance():
    """Смешанный исход планировщика: темп уже на потолке (предел), а «потяжелее» встало - фраза НЕ молчит посреди рэпа."""
    data = _from_planner(("faster", "heavier"), bpm=float(kn.BPM_RANGE[1]))
    assert data["changes"] == ["heavier"] and data["limited"] == ["faster"] and data["heard"] is False
    result = _as_result(data)
    assert phrase_key(result) == "music.edit_said" and not quiet_in_performance(result)
    assert audible([result], speaks=False, performing=True) == [result]


def test_a_refusal_next_to_a_done_edit_is_said_even_under_the_performance():
    """Бит выступления: темп не меняется (сетка куплетов), «потяжелее» встало - отказ по темпу говорится."""
    data = _from_planner(("faster", "heavier"), bpm=120.0, backing=True)
    assert data["changes"] == ["heavier"] and data["refused"] == "backing_tempo" and data["heard"] is False
    result = _as_result(data)
    assert phrase_key(result) == "music.edit_said" and not quiet_in_performance(result)
    assert audible([result], speaks=False, performing=True) == [result]


def test_a_whole_edit_is_the_one_that_stays_quiet_under_the_performance():
    data = _from_planner(("heavier",), bpm=120.0)
    assert data["heard"] is True
    result = _as_result(data)
    assert phrase_key(result) == "music.edit_heard" and quiet_in_performance(result)
    assert audible([result], speaks=False, performing=True) == []
    assert dkn.QUIET_IN_PERFORMANCE_KEYS[-1] == "music.edit_heard"
