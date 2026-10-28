"""#4173, марафон-4 (28.10 22:23:39, акт 6, n607): «помягче, поставь спокойное» под идущим диджей-сетом.

До #4192 модель звала ``request_music`` - новый трек снимал сет (``reason=request_music``, марафон-3 19:50:18); после
#4192 путь правки брал ``edit_music``, а тул на сете честно отказывал (``отказ=set``): настроение сета не менялось.
Теперь правка настроения (``knowledge.SET_VIBES``) под сетом - смена стиля идущего сета кодом: та же тема и персона,
остаток длины, стиль из таблицы (``tools_v2.restyle_set`` - тот же путь, что смена темы посреди сета, #3400); фраза
исхода - из исхода; ни ``request_music``, ни ``generate_music``.

Настоящие ``PlayerOwner`` + ``RenardoAdapter`` + ``compose``/``render`` на симуляторе клока (рига
``test_edit_music_4029``), без LLM и без звука.
"""

from functools import partial

import pytest

from rob_box_mcp_tools.engine.tools_v2 import restyle_set
from rob_box_mcp_tools.engine.track_edit import TrackEditor
from rob_box_music import knowledge as kn
from rob_box_music.track_edit import SET, EditTarget, edit_text, plan_edit

from .test_edit_music_4029 import N607, Deck, Owner, _clock_wait, _events, _live_rig, _no_new_order, _tools

pytestmark = pytest.mark.unit

#: n607 дословно, как услышал робот (STT марафона-3 28.10 19:50:12, va_m3.log).
HEARD = N607


def _set_rig(style="club"):
    rig = _live_rig()
    req, tool = _tools(rig)
    dj = req._dj  # noqa: SLF001 — тот же DjSetTool, что у request_music
    req.editor = TrackEditor(rig.owner, rig.adapter, wait=_clock_wait(rig), restyle=partial(restyle_set, dj))
    out = dj.execute(action="start", theme="ночная дискотека", persona="Робби", style=style, tracks=4)
    assert out.success, out
    rig.clock.run_until(rig.clock.beat + 9)
    req.order = _no_new_order  # ни request_music, ни generate_music: заказа нового трека нет
    return rig, req, tool, dj, out.data


def test_n607_softer_under_a_set_turns_the_set_calm_not_a_refusal_and_not_a_new_track():
    rig, _req, tool, dj, first = _set_rig()
    assert rig.owner.now_playing()["enabled"] is True and first["style"] == "club"
    before = dj._last_dj  # noqa: SLF001 — тема и персона идущего сета

    result = tool.execute(heard_text=HEARD)

    assert result.success, result
    data = result.data
    assert data["changes"] == ["softer"] and data["set_style"] == "lofi"
    assert data["message"] == "Сделал сет спокойнее — дальше лоуфай."
    rig.clock.run_until(rig.clock.beat + 9)  # трек сета в новом стиле встал на деку
    # сет тот же по смыслу: тема и персона прежние, идёт сет (не одиночный трек), стиль - спокойный
    snap = rig.owner.now_playing()
    assert snap["enabled"] is True and snap["style"] == "lofi" and snap["theme"] == "ночная дискотека"
    assert dj.running and dj._last_dj == before and before[0] == "ночная дискотека"  # noqa: SLF001
    # остаток длины прежнего сета, а не новый сет по умолчанию
    assert snap["tracks"] == first["tracks"] - 1
    started = [e["track_id"] for e in _events(rig, "started")]
    assert started == [first["track_id"], data["track_id"]] and snap["track_id"] == data["track_id"]
    assert kn.STYLES["lofi"].bpm[0] <= snap["bpm"] <= kn.STYLES["lofi"].bpm[1]


def test_softer_on_an_already_calm_set_is_the_limit_and_the_set_is_untouched():
    rig, _req, tool, _dj, first = _set_rig(style="lofi")

    result = tool.execute(heard_text=HEARD)

    assert result.success and result.data["limited"] == ["softer"] and result.data["changes"] == []
    assert result.data["message"] == "Сет и так спокойный — играет лоуфай."
    assert rig.owner.now_playing()["track_id"] == first["track_id"]
    assert len(_events(rig, "started")) == 1


def test_faster_under_a_set_stays_an_honest_refusal():
    """Темп и бас треков сета на ходу не меняются: прежний честный отказ ``EDIT_REFUSALS["set"]``."""
    rig, _req, tool, _dj, first = _set_rig()
    result = tool.execute(heard_text="Робот, ускорь")
    assert not result.success and result.data["message"] == kn.EDIT_REFUSALS["set"]
    assert rig.owner.now_playing()["track_id"] == first["track_id"]


def test_failed_restyle_says_so_and_does_not_claim_the_old_set_plays():
    snap = {"playing": True, "track_id": "gen26164:00:A", "source": "generated", "enabled": True, "style": "club"}
    editor = TrackEditor(Owner(**snap), Deck(), restyle=lambda style: {"ok": False, "reason": "music_source"})
    data = editor.edit((), HEARD)
    assert data["ok"] is False and data["message"] == kn.EDIT_REFUSALS["set_vibe_failed"]
    assert "играет как было" not in data["message"]


def test_softer_under_a_generated_set_restyles_it_and_says_what_plays_meanwhile(tmp_path, monkeypatch):
    """Сет из кэша и генерации (марафон-4: ``gen26164``): тот же путь смены стиля - остаток длины в лоуфае, а пока
    лоуфай пишется, фраза кода пути говорит, что играет (мост синтезом), а не только «сделал»."""
    from .test_adr0158_pr8_gen_set import Kie, _dj, _fake_audio_track, _lib  # noqa: PLC0415

    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    dj = _dj(_lib(tmp_path, cached=2), Kie())
    out = dj.execute(action="start", style="rock", tracks=3, heard_text="включи рок-сет на 3 трека")
    assert out.success and dj.running
    snap = {"playing": True, "track_id": out.data["track_id"], "enabled": True, "style": "rock"}
    editor = TrackEditor(Owner(**snap), Deck(), restyle=partial(restyle_set, dj))

    data = editor.edit((), HEARD)

    assert data["ok"] is True and data["set_style"] == "lofi" and dj.running
    note = "Пока я дописываю лоуфай — вот вам синтвейв, секунд 30."
    assert data["message"] == f"Сделал сет спокойнее — дальше лоуфай. {note}" and data["heard"] is False


@pytest.mark.parametrize("kinds,style,expect", [
    (("softer",), "club", ("lofi", ("softer",), (), "")),
    (("heavier",), "club", ("rock", ("heavier",), (), "")),
    (("softer",), "lofi", ("", (), ("softer",), "")),
    (("faster",), "club", ("", (), (), SET)),
    (("more_bass",), "club", ("", (), (), SET)),
    (("faster", "softer"), "club", ("lofi", ("softer",), (), "")),
])
def test_plan_under_a_set_is_decided_by_the_vibe_table(kinds, style, expect):
    plan = plan_edit(kinds, EditTarget(SET, style=style))
    assert (plan.restyle, plan.applied, plan.limited, plan.refused) == expect
    assert not plan.changes_sound  # трек сета не пересобирается
    assert edit_text(plan)


def test_vibe_styles_are_set_styles():
    """Стиль смены настроения - ключ ``STYLES`` (``dj_set`` style enum), вид правки - из ``TRACK_EDITS``."""
    for kind, vibe in kn.SET_VIBES.items():
        assert kind in kn.TRACK_EDITS and vibe.style in kn.STYLES and vibe.done and vibe.limit
