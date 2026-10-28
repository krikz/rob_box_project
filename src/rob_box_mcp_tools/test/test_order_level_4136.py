"""#4136 п.2 — «поставь фоном ровный ритм на синтезаторе, только негромко» (марафон 28.10, акт 8, n809).

Трек синтеза заиграл, а просьба о громкости пропала: STT дал «не громко», в теме сета «не» выпало, и тема стала
«фоном ровный ритм громко» (противоположное просьбе); громкость музыки не менялась, и об этом никто не сказал. Решает
код (ADR-0148): слова громкости не входят в тему (``theme_grounding.heard_theme``), а уровень из слов заказа
(``words.asked_level``) код применяет шагом мастер-фейдера музыки, как только заказ зазвучал (``engine.order_level``);
итог (применено или нет и почему) — в данных результата, поле ``level``.
"""

import logging

import pytest

from rob_box_dialog.words import asked_level, level_word
from rob_box_mcp_tools.base import MCPToolResult
from rob_box_mcp_tools.engine.order_level import NO_VOLUME_TOOL, UNCHANGED, apply_level, with_level
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.theme_grounding import heard_theme
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, club_theme

from .test_engine_session import _rig
from .test_issue_4013_engine_slot_and_quick_start import RHYTHM, _tools

pytestmark = pytest.mark.unit

#: Живая фраза n809 28.10 07:02:09 (то, что LLM передала в ``text``) и её чистый вариант.
LIVE_TEXT = "поставь фоном ровный ритм на синтезаторе только не громко"
CLEAN_TEXT = "Робот, поставь фоном ровный ритм на синтезаторе, только негромко."


class Fader:
    """Подставной ``set_music_volume``: что просили и что ответил (двигается ли фейдер)."""

    def __init__(self, moves=True, ok=True):
        self.moves, self.ok, self.asked = moves, ok, []

    def __call__(self, action):
        self.asked.append(action)
        if not self.ok:
            return MCPToolResult(success=False, error="runtime не создан")
        old, new = 0.5, (0.35 if self.moves else 0.5)
        return MCPToolResult(
            success=True,
            data={"old_gain": old, "new_gain": new, "percent": round(new * 100)},
            message="Громкость музыки: 50% → 35%",
        )


# ── тема: громкость и «не» не попадают в тему против просьбы ───────────────────────────────────────────────────


@pytest.mark.parametrize("text", [LIVE_TEXT, CLEAN_TEXT, RHYTHM])
def test_loudness_and_its_negation_are_not_the_theme_of_the_order(text):
    """n809: тема была «фоном ровный ритм громко» - противоположное просьбе. Слово громкости и «не» перед ним уходят."""
    theme = club_theme(text)
    words = theme.replace(",", " ").split()
    assert "громко" not in words and "негромко" not in words and "не" not in words, theme
    assert "ритм" in words and "ровный" in words


@pytest.mark.parametrize("word", ["громко", "тихо", "негромко", "потише", "погромче", "тише", "громче", "убавь"])
def test_a_loudness_word_is_never_a_theme_word(word):
    assert level_word(word)
    assert word not in heard_theme(f"сет про космос {word}").split()


def test_negation_stays_with_the_theme_word_it_negates():
    """«Не» связано со словом после него: слово темы остаётся с ним, а не теряет отрицание (было: «не космос» → «космос»)."""
    assert heard_theme("поставь сет не космос") == "не космос"


@pytest.mark.parametrize("title", ["Тихий дон", "Тихая ночь", "громкость поменьше"])
def test_adjectives_and_the_loudness_noun_stay_in_the_theme(title):
    """По основе «тих»/«громк» открылись бы названия пьес («Тихий Дон») и «громкость»: слова громкости - целые наречия."""
    assert not level_word(title.split()[0])
    assert heard_theme(f"поставь {title}").split()[0] == title.split()[0].lower()


# ── уровень из слов заказа ─────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "text,level",
    [
        (LIVE_TEXT, "quieter"),
        (CLEAN_TEXT, "quieter"),
        ("поставь ритм негромко", "quieter"),
        ("поставь ритм тихо", "quieter"),
        ("поставь ритм потише", "quieter"),
        ("только не громко", "quieter"),
        ("поставь ритм громко", "louder"),
        ("поставь ритм погромче", "louder"),
        ("только не тихо", "louder"),
        ("поставь ритм", ""),
        ("громкость поменьше", ""),
        ("поставь тихий дон", ""),
        ("поставь потише, нет, погромче", ""),  # просил обоих: выбирать нечем - код не двигает фейдер
    ],
)
def test_asked_level_reads_the_direction_with_negation(text, level):
    assert asked_level(text) == level


# ── шаг фейдера при старте трека ───────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("text", [LIVE_TEXT, CLEAN_TEXT])
def test_quiet_order_steps_the_music_fader_once_the_track_plays(text):
    rig = _rig()
    _dj, req = _tools(rig)
    fader = req.level = Fader()
    result = req.execute(intent="track", text=text, engine="synth", heard_text=CLEAN_TEXT)
    assert result.success and rig.owner.now_playing()["engine"] == "synth"
    assert fader.asked == ["quieter"], "громкость - шаг фейдера кодом, один раз"
    assert result.data["level"] == {"asked": "quieter", "applied": True, "percent": 35}


def test_order_without_loudness_words_does_not_touch_the_fader():
    rig = _rig()
    _dj, req = _tools(rig)
    fader = req.level = Fader()
    result = req.execute(intent="track", text="поставь ровный ритм", heard_text="Робот, поставь ровный ритм")
    assert result.success and fader.asked == [] and "level" not in result.data


def test_unplayed_order_does_not_move_the_fader(monkeypatch):
    """Заказ не зазвучал (ок=false) - уровень не трогаем: громкость относится к музыке, которой нет."""
    rig = _rig()
    _dj, req = _tools(rig)
    fader = req.level = Fader()
    monkeypatch.setattr(req, "order", lambda *a, **k: MCPToolResult(success=False, data={"ok": False}))
    result = req.execute(intent="track", text=LIVE_TEXT, heard_text=CLEAN_TEXT)
    assert not result.success and fader.asked == []


def test_no_volume_tool_is_said_in_the_result_not_lost(caplog):
    """Звукового сервера нет - заказ играет, а просьба об уровне честно помечена «не применена» (и в логе)."""
    rig = _rig()
    _dj, req = _tools(rig)
    assert req.level is None
    with caplog.at_level(logging.WARNING):
        result = req.execute(intent="track", text=LIVE_TEXT, heard_text=CLEAN_TEXT)
    assert result.success
    assert result.data["level"] == {"asked": "quieter", "applied": False, "reason": NO_VOLUME_TOOL}
    assert any("[level]" in r.getMessage() for r in caplog.records)


def test_fader_at_its_floor_is_reported_unchanged():
    done = apply_level("quieter", Fader(moves=False))
    assert done == {"asked": "quieter", "applied": False, "percent": 50, "reason": UNCHANGED}


@pytest.mark.parametrize("setter", [Fader(ok=False), lambda action: 1 / 0])
def test_a_failing_fader_does_not_fail_the_playing_order(setter):
    result = with_level(MCPToolResult(success=True, data={"ok": True}), "поставь потише", setter)
    assert result.success and result.data["level"]["applied"] is False and result.data["level"]["reason"]


def test_dj_set_order_gets_the_same_level_step():
    rig = _rig()
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 4242)
    fader = dj.level = Fader()
    heard = "Робот, ты диджей, сет про космос, только негромко"
    result = dj.execute(action="start", theme="космос", heard_text=heard)
    assert result.success and fader.asked == ["quieter"] and result.data["level"]["applied"] is True


def test_request_music_tool_has_the_level_hook_default_off():
    rig = _rig()
    _dj, req = _tools(rig)
    assert req.level is None
