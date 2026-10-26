"""ADR-0157 PR-1 (#3558): ``dj_set(start)`` без слова сета в реплике хода — отказ кодом ``not_a_set_request``;
``request_music`` берёт инструмент из реплики хода и играет лид его синтом (или синтом соседа той же семьи).

Прогон 09.10 07:52: «сыграй баха на трамбоне» → LLM ``dj_set(theme='Бах на трамбоне')`` → сет с «Mundian To Bach
Ke». Режим решает код по ``heard_text`` (скрытый аргумент хода), LLM в сет по инерции контекста не уводит.
"""

from unittest.mock import patch

import pytest

from rob_box_mcp_tools.engine import tools_v2
from rob_box_mcp_tools.engine.classic import ClassicPick
from rob_box_mcp_tools.engine.tools_v2 import NOT_A_SET_REQUEST, club_theme, lead_for, set_refusal

from ._ros_stubs import RosStubs
from .test_engine_classic import _picker
from .test_engine_request_music import _tools
from .test_engine_session import _rig


_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.llm_adapter import TURN_CONTEXT_ARGS
_ros_stubs = _ros.fixture()


@pytest.mark.parametrize("heard", ["сыграй баха на трамбоне", "Сыграй на балалайке",
                                   "не еби голову сыграй Баха на балалайке какой нахуй сэт"])
def test_dj_set_refuses_a_piece_request(heard):
    rig = _rig()
    dj, _req = _tools(rig)
    result = dj.execute(action="start", theme="Бах на трамбоне", heard_text=heard)
    assert result.success is False and result.data["reason"] == NOT_A_SET_REQUEST
    assert not dj.running and rig.events == []  # ни звука, ни отказа плеера: сет даже не собирался


@pytest.mark.parametrize("heard", ["включи диджей сет на тему Бах", "включи рейв", "ты диджей Снупдог",
                                   "замути сэт про космос"])
def test_set_words_or_style_let_the_set_start(heard):
    assert set_refusal(heard, running=False) is None


def test_no_turn_text_or_running_set_is_not_refused():
    """Вызов не из хода (нет ``heard_text``) и смена темы посреди сета (#3400) — не заказ одной пьесы."""
    assert set_refusal(None, running=False) is None
    assert set_refusal("сыграй марио", running=True) is None
    assert set_refusal("сыграй марио", running=False)["reason"] == NOT_A_SET_REQUEST


def test_request_music_gets_the_turn_text_hidden_from_the_llm():
    assert "heard_text" in TURN_CONTEXT_ARGS["request_music"]


def test_lead_from_the_turn_text_first_then_from_the_tool_text():
    assert lead_for("сыграй баха на трамбоне", "Бах").synth == "brass"
    assert lead_for(None, "Бах на кселофоне").synth == "marimba"
    assert lead_for("сыграй баха", "баха") is None


def test_request_music_plays_the_neighbour_when_there_is_no_pack():
    """«Баха на балалайке»: синта балалайки нет → лид гитары (первый сосед семьи), в результате — оба ключа."""
    rig = _rig()
    leads = []
    plain = _picker([])

    def picker(query, *, seed, deck="A", lead=None):
        leads.append(lead)
        return plain(query, seed=seed, deck=deck)

    _dj, req = _tools(rig, classic=picker)
    result = req.execute(intent="melody", text="Баха", heard_text="сыграй Баха на балалайке")
    assert result.success, result.error
    assert result.data["instrument"] == "balalaika" and result.data["instrument_played"] == "guitar"
    assert leads[0].synth == "pluck"


def test_request_music_without_instrument_is_unchanged():
    rig = _rig()
    _dj, req = _tools(rig, classic=lambda query, seed: ClassicPick(query=query, found=False, reason="нет в базе"))
    result = req.execute(intent="melody", text="калинку", heard_text="поставь калинку")
    assert result.data["reason"] == "not_found" and "instrument" not in result.data


def test_club_order_on_an_instrument_plays_its_synth():
    """«Сыграй что-нибудь на арфе» — одиночный трек, лид — синт арфы из таблицы, а не тембр семьи темы."""
    rig = _rig()
    _dj, req = _tools(rig)
    built = []
    real = tools_v2.compose

    def spy(*args, **kwargs):
        built.append(real(*args, **kwargs))
        return built[-1]

    with patch.object(tools_v2, "compose", spy):
        result = req.execute(intent="track", text="сыграй на арфе", heard_text="Робот, сыграй что-нибудь на арфе")
    assert result.success and result.data["instrument_played"] == "harp"
    assert built[0].parts["lead"].synth_or_sample == "karp"
    assert result.data["title"].startswith("клубный трек")  # слова заказа и инструмента — не тема трека


@pytest.mark.parametrize("text,theme", [
    ("сыграй что-нибудь на арфе", ""), ("Робот, сыграй на пианино", ""),
    ("поставь клубный трек", ""), ("космос", "космос"),
    # #3960 п.2: глаголы заказа, служебные слова и название движка — не тема (LLM-разбор спрашивал «Renardo»)
    ("поставь фоном ровный ритм на Ренардо только не громко", "фоном ровный ритм громко"),
    ("Робот, поставь фоном ровный ритм наррендарда только не громко", "фоном ровный ритм громко"),
    ("сыграй на индардо", ""), ("сыграй на ре нардо космос", "космос"),
])
def test_club_theme_drops_the_order_and_instrument_words(text, theme):
    assert club_theme(text) == theme
