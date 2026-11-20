"""#4479 (М14 n604): ``style=rock`` из вызова модели доходит до сета, мусор в реплике его не перебивает."""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.engine.engine_tools import set_style
from rob_box_mcp_tools.engine.gen.gen_session import slot_genres

from .test_adr0158_pr8_gen_set import Kie, _dj, _lib
from .test_engine_request_music import _tools
from .test_engine_session import _rig

pytestmark = pytest.mark.unit

_GARBAGE = "Робот, диджей, уводи в сторону подруг добреет"


@pytest.mark.parametrize("theme", ["рок", "подруг добреет", None])
def test_model_style_rock_survives_garbage_heard_text(theme):
    dj, _req = _tools(_rig())
    data = dj.execute(action="start", style="rock", theme=theme, heard_text=_GARBAGE).data
    assert data["style"] == "rock"
    assert data.get("theme_source") == "pool"


def test_named_style_beats_garbage_before_default_club():
    assert set_style("rock", "подруг добреет", _GARBAGE) == "rock"
    assert slot_genres(_GARBAGE, (), "rock")[0] == ("rock",)
    assert slot_genres(_GARBAGE, ("club",), "rock")[0] == ("rock",)


def test_generated_set_keeps_named_rock_when_heard_text_names_no_style(tmp_path):
    dj = _dj(_lib(tmp_path, cached=0), Kie())
    data = dj.execute(action="start", style="rock", theme="подруг добреет", heard_text=_GARBAGE).data
    assert data["ok"] is True and data["style"] == "rock"
    assert data.get("source") != "pool"


def test_heard_style_word_still_fills_an_empty_argument():
    heard = "Робот, включи джаз сет про космос"
    assert set_style(None, "космос", heard) == "jazz"
    assert set_style("auto", "космос", heard) == "jazz"
    assert slot_genres(heard, (), "auto")[0] == ("jazz",)
    assert slot_genres("включи джаз сет", (), "rock")[0] == ("jazz",)
