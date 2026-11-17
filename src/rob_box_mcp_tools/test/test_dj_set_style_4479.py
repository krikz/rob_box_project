"""#4479 (М14 n604): ``style=rock`` из вызова модели доходит до сета, мусор в реплике его не перебивает."""

from __future__ import annotations

import pytest

from .test_engine_request_music import _tools
from .test_engine_session import _rig

pytestmark = pytest.mark.unit

_GARBAGE = "Робот, диджей, уводи в сторону подруг добреет"


@pytest.mark.parametrize("theme", ["рок", "подруг добреет", None])
def test_model_style_rock_survives_garbage_heard_text(theme):
    dj, _req = _tools(_rig())
    data = dj.execute(action="start", style="rock", theme=theme, heard_text=_GARBAGE).data
    assert data["style"] == "rock"
