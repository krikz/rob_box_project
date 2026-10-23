"""``set_volume`` в процентах (PR-13b #3708): «громкость 60 %» — ``action=set, level=60``, а не ``normal``.

Шкала одна — ``SetVolumeTool.MIN_DB..MAX_DB``: проценты и шаги ложатся на неё же, ответ называет проценты.
"""

from __future__ import annotations

import pytest

from .._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools.system import SetVolumeTool as T
_ros_stubs = _ros.fixture()


@pytest.mark.parametrize("level, db", [(0, T.MIN_DB), (100, T.MAX_DB), (50, (T.MIN_DB + T.MAX_DB) / 2),
                                       (150, T.MAX_DB), (-5, T.MIN_DB)])
def test_percent_maps_onto_the_db_scale_clamped(level, db):
    target, error = T.target_db("set", level, -3.0)
    assert error is None and target == pytest.approx(db)


def test_sixty_percent_is_not_normal():
    target, _ = T.target_db("set", 60, T.NORMAL_DB)
    assert target != pytest.approx(T.NORMAL_DB) and T.percent(target) == 60


@pytest.mark.parametrize("level", [None, "громко"])
def test_set_without_a_number_is_an_honest_error(level):
    target, error = T.target_db("set", level, -3.0)
    assert target is None and "level" in error


def test_steps_stay_on_the_same_scale():
    assert T.target_db("louder", None, T.MAX_DB - 1.0)[0] == T.MAX_DB
    assert T.target_db("quieter", None, T.MIN_DB + 1.0)[0] == T.MIN_DB
    assert T.target_db("max", None, 0.0)[0] == T.MAX_DB and T.target_db("normal", None, 0.0)[0] == T.NORMAL_DB
    assert T.target_db("louder", None, -3.0) == (-3.0 + T.STEP_DB, None)
    assert T.target_db("wobble", None, 0.0)[0] is None
