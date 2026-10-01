"""Честный ввод compose_music (ADR-0132, PR-2).

1. ``root``/``scale`` при ``name=`` ПЕРЕГАРМОНИЗИРУЮТ тему: бас и пэд
   строятся в заданной тональности, тема (лид) играется как есть.
   Раньше они молча игнорировались.
2. Тихие подстановки стали ошибками со списком допустимых значений:
   неизвестная форма (было → arc), лад, тоника (было → C), bpm и swing вне
   диапазона (было — кламп в ``render``).
3. Вызовы без root/scale не меняются — это держит golden
   (``test_arranger_golden``), здесь только явный путь.

Вход — замороженные RTTTL-строки golden-фикстуры, а не поиск по
библиотеке: ранжирование поиска меняется и не должно ронять эти тесты.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from unittest.mock import Mock

import pytest

from ._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.core.arranger import (
        BPM_RANGE,
        FORMS,
        SCALE_INTERVALS,
        VALID_ROOTS,
        ArrangementError,
        check_bpm,
        check_form,
        check_root,
        check_scale,
        check_swing,
        render,
        spec_from_flat,
    )
    from rob_box_mcp_tools.core.rtttl_compose import (
        key_fit,
        melody_to_compose_params,
        rtttl_to_melody,
    )
    from rob_box_mcp_tools.core.score_sheet import KEY_FIT_WARN, describe
    from rob_box_mcp_tools.tools.music import ComposeMusicTool
_ros_stubs = _ros.fixture()

_FIXTURE = Path(__file__).parent / "fixtures" / "arranger_golden.json"
_ARR = dict(lead_synth="blip", bass_synth="moogbass", pad_synth="strings")


@pytest.fixture(scope="module")
def themes():
    cases = json.loads(_FIXTURE.read_text(encoding="utf-8"))["cases"]
    out = {}
    for case in cases:
        out.setdefault(case["key"], case["rtttl"])
    return out


def _params(rtttl, root=None, scale=None):
    return melody_to_compose_params(rtttl_to_melody(rtttl), root=root, scale=scale)


def _scale_pcs(root, scale):
    tonic = VALID_ROOTS.index(root)
    return {(tonic + i) % 12 for i in SCALE_INTERVALS[scale]}


def _share_in(notes, pcs):
    """Доля длительности звучащих нот, чей класс высоты лежит в ``pcs``."""
    flat = []
    for note, dur in notes:
        if note is None:
            continue
        for pitch in (note if isinstance(note, tuple) else (note,)):
            flat.append((pitch, dur))
    total = sum(d for _p, d in flat)
    return sum(d for p, d in flat if p % 12 in pcs) / total


# ---------------------------------------------------------------------------
# 1. Перегармонизация явной тональностью (ядро)
# ---------------------------------------------------------------------------


def test_default_call_has_no_explicit_key_record(themes):
    """Без root/scale — прежние решения, без полей явной тональности."""
    params = _params(themes["national_2"])
    assert (params["root"], params["scale"]) == ("C", "major")
    assert "key_source" not in params["decisions"]


def test_explicit_key_transposes_lead_and_reharmonizes_bass_and_pad(themes):
    auto = _params(themes["national_2"])
    forced = _params(themes["national_2"], root="D", scale="minor")

    assert (forced["root"], forced["scale"]) == ("D", "minor")
    assert (forced["harmony"].root, forced["harmony"].scale) == ("D", "minor")
    # Тема — та же самая по ритму и контуру, но перенесена в тональность
    # (issue #3293): интервалы между нотами сохранены.
    assert forced["lead_midi"] != auto["lead_midi"]
    assert forced["lead_dur"] == auto["lead_dur"]
    f_notes = [int(x) for x in forced["lead_midi"].split(", ") if x != "None"]
    a_notes = [int(x) for x in auto["lead_midi"].split(", ") if x != "None"]
    assert len({f - a for f, a in zip(f_notes, a_notes)}) == 1
    assert forced["decisions"]["key_fit"] >= KEY_FIT_WARN
    # Аккомпанемент — другой и следует новой тональности: в ре-миноре
    # есть B♭ (10), которого нет в до-мажоре.
    assert forced["harmony"].bass != auto["harmony"].bass
    assert forced["harmony"].pad != auto["harmony"].pad
    bass_pcs = {n % 12 for n, _d in forced["harmony"].bass if n is not None}
    pad_pcs = {p % 12 for ch, _d in forced["harmony"].pad if ch for p in ch}
    assert 10 in bass_pcs | pad_pcs
    assert 10 not in {n % 12 for n, _d in auto["harmony"].bass if n is not None}


def test_explicit_distant_key_bass_and_pad_follow_new_key(themes):
    """F# мажор на до-мажорной теме: бас и пэд в основном в F# мажоре,
    а не в определённом по нотам до-мажоре."""
    forced = _params(themes["national_2"], root="F#", scale="major")
    new_pcs = _scale_pcs("F#", "major")
    old_pcs = _scale_pcs("C", "major")
    harmony = forced["harmony"]
    assert _share_in(harmony.bass, new_pcs) > _share_in(harmony.bass, old_pcs)
    assert _share_in(harmony.pad, new_pcs) > _share_in(harmony.pad, old_pcs)


def test_only_root_keeps_detected_scale_and_vice_versa(themes):
    only_root = _params(themes["national_2"], root="G")
    assert (only_root["root"], only_root["scale"]) == ("G", "major")
    only_scale = _params(themes["national_2"], scale="minor")
    assert (only_scale["root"], only_scale["scale"]) == ("C", "minor")


def test_explicit_key_decisions_record_detected_and_fit(themes):
    dec = _params(themes["national_2"], root="A", scale="minor")["decisions"]
    assert dec["key_source"] == "explicit"
    assert dec["key_explicit"] == ("A", "minor")
    assert dec["key_detected"] == ("C", "major")
    # Параллельная тональность: тот же звукоряд — тема вся в ладу.
    assert dec["key_fit"] >= 0.9
    # Ранжирование авто-кандидатов записано как и раньше.
    assert dec["key_ranked"][0][:2] == ("C", "major")


def test_key_fit_counts_duration_in_scale():
    notes = [(60, 1.0), (61, 3.0), (None, 2.0)]  # C (в C major), C# (вне)
    assert key_fit(notes, "C", "major") == 0.25
    assert key_fit([(None, 1.0)], "C", "major") == 1.0


# ---------------------------------------------------------------------------
# 1b. Партитура называет явную тональность
# ---------------------------------------------------------------------------


def _sheet(rtttl, root, scale):
    params = _params(rtttl, root=root, scale=scale)
    spec = spec_from_flat(
        harmony=params["harmony"], bpm=float(params["bpm"]),
        root=params["root"], scale=params["scale"],
        lead_midi=params["lead_midi"], lead_dur=params["lead_dur"],
        form="arc", repeat=False, **_ARR,
    )
    code = render(spec)
    sheet = describe(
        spec=spec, code=code, harmony=params["harmony"],
        prep_decisions=params["decisions"], title="t",
    )
    return spec, code, sheet


def test_score_sheet_shows_explicit_key_and_detected(themes):
    _spec, code, sheet = _sheet(themes["national_2"], "A", "minor")
    assert 'Root.default = "A"' in code
    assert 'Scale.default = "minor"' in code
    assert sheet["key"]["source"] == "задана вызовом"
    assert sheet["key"]["detected"] == {"root": "C", "scale": "major"}
    assert sheet["decisions"]["key"] == "explicit→A minor (auto C major)"
    assert "задана вызовом; по нотам C major" in sheet["text"]
    assert not any("спорит" in w for w in sheet["warnings"])
    assert not any("неуверенная" in w for w in sheet["warnings"])


def test_distant_explicit_key_transposes_theme_instead_of_warning(themes):
    """F# major на до-мажорной теме: тема переносится (issue #3293), а не
    играет мимо аккомпанемента — спора с партитурой больше нет."""
    spec, code, sheet = _sheet(themes["national_2"], "F#", "major")
    assert 'Root.default = "F#"' in code
    assert sheet["key"]["fit"] >= KEY_FIT_WARN
    assert not any("спорит с темой" in w for w in sheet["warnings"])


# ---------------------------------------------------------------------------
# 2. Тихие подстановки → ошибки со списком (ядро check_*)
# ---------------------------------------------------------------------------


def test_check_form():
    assert check_form(None) == "arc"
    assert check_form(" Verse_Chorus ") == "verse_chorus"
    with pytest.raises(ArrangementError) as exc:
        check_form("symphony")
    for name in FORMS:
        assert name in str(exc.value)


@pytest.mark.parametrize(
    "given, expected",
    [(None, None), ("", None), ("a", "A"), ("F#", "F#"), ("Bb", "A#"),
     ("eb", "D#"), ("Cb", "B"), ("B#", "C"), ("C♯", "C#")],
)
def test_check_root_normalizes(given, expected):
    assert check_root(given) == expected


@pytest.mark.parametrize("bad", ["H", "Am", "до", "C##", "X"])
def test_check_root_rejects_with_list(bad):
    with pytest.raises(ArrangementError) as exc:
        check_root(bad)
    msg = str(exc.value)
    assert all(root in msg for root in VALID_ROOTS)
    assert "scale" in msg  # подсказка: лад задаётся отдельно


def test_check_scale():
    assert check_scale(None) is None
    assert check_scale("MINOR") == "minor"
    assert check_scale("harmonicminor") == "harmonicMinor"
    with pytest.raises(ArrangementError) as exc:
        check_scale("blues")
    assert all(name in str(exc.value) for name in SCALE_INTERVALS)


def test_check_bpm_and_swing_ranges():
    assert check_bpm(None) is None
    assert check_bpm(60) == 60.0 and check_bpm("180") == 180.0
    for bad in (59, 181, 300, "fast"):
        with pytest.raises(ArrangementError, match="bpm"):
            check_bpm(bad)
    assert check_swing(None) == 0.0
    assert check_swing(0.3) == 0.3
    for bad in (-0.1, 0.31, 1, "much"):
        with pytest.raises(ArrangementError, match="swing"):
            check_swing(bad)


# ---------------------------------------------------------------------------
# 2b. Тул: ошибка до всякой работы, музыка не запускается
# ---------------------------------------------------------------------------


def _tool(mock_node):
    mgr = Mock()
    mgr.execute_code = Mock(return_value={"success": True})
    return ComposeMusicTool(mock_node, mgr), mgr


_COMPOSED = dict(
    lead_notes="0, 2, 4, 7", pad_notes="0, 2, 4", bass_notes="0, 4",
    drums="X...o...", **_ARR,
)


@pytest.mark.parametrize(
    "bad, listed",
    [
        (dict(form="symphony"), sorted(FORMS)),
        (dict(scale="blues"), list(SCALE_INTERVALS)),
        (dict(root="H"), list(VALID_ROOTS)),
        (dict(bpm=240), ["60", "180"]),
        (dict(bpm=30), ["60", "180"]),
        (dict(swing=0.5), ["0", "0.3"]),
        (dict(swing=-0.2), ["0", "0.3"]),
    ],
)
def test_tool_rejects_silent_substitutions(mock_node, bad, listed):
    tool, mgr = _tool(mock_node)
    result = tool.execute(**{**_COMPOSED, **bad})
    assert result.success is False
    for value in listed:
        assert value in result.error
    mgr.execute_code.assert_not_called()


def test_tool_rejects_bad_input_with_name_before_lookup(mock_node):
    """С name= ошибка ввода приходит раньше поиска мелодии."""
    tool, mgr = _tool(mock_node)
    result = tool.execute(name="tetris", root="H", **_ARR)
    assert result.success is False
    assert "Неизвестная тоника" in result.error
    mgr.execute_code.assert_not_called()


def test_tool_valid_edges_still_play(mock_node):
    tool, mgr = _tool(mock_node)
    result = tool.execute(
        **_COMPOSED, form="AMBIENT", root="bb", scale="Dorian",
        bpm=BPM_RANGE[1], swing=0.3,
    )
    assert result.success is True, result.error
    code = mgr.execute_code.call_args[0][0]
    assert 'Root.default = "A#"' in code
    assert 'Scale.default = "dorian"' in code
    assert f"Clock.bpm = {BPM_RANGE[1]:g}" in code
    assert tool.last_score["form"]["name"] == "ambient"  # PR-4: dict — вне ответа модели


def test_tool_name_with_explicit_key_reharmonizes(mock_node, themes):
    """Путь тула целиком: name= + root/scale → код в заданной тональности,
    партитура — explicit, тема та же, что без root/scale."""
    rec = {"name": "national_2", "title": "National 2", "rtttl": themes["national_2"]}
    tool, mgr = _tool(mock_node)
    tool._resolve_melody = lambda _n, _v: rec  # замороженный вход

    assert tool.execute(name="anthem", **_ARR).success is True
    auto_code = mgr.execute_code.call_args[0][0]
    result = tool.execute(name="anthem", root="D", scale="minor", **_ARR)
    assert result.success is True, result.error
    code = mgr.execute_code.call_args[0][0]

    assert 'Root.default = "D"' in code and 'Scale.default = "minor"' in code
    assert code != auto_code
    lead_line = [ln for ln in auto_code.splitlines() if ln.startswith("p2 >>")]
    # Тема перенесена в заданную тональность (issue #3293) — строка лида другая.
    assert lead_line and lead_line[0] not in code.splitlines()
    score = tool.last_score  # PR-4: структурная партитура — вне ответа модели
    assert score["decisions"]["key"] == "explicit→D minor (auto C major)"


# ---------------------------------------------------------------------------
# 4. Issue #3293: тема транспонируется под явную тонику, ответ честен
# ---------------------------------------------------------------------------

_JUMP_AMP = 'Jump&Amp:d=16,o=5,b=90:f#,c#,f#,f#,a#,f#,a#,c#6,b,a#,b,g#,g#,d#,g#,g#,b,g#,b,d#6,c#,b,a#,f#,f#,c#,f#,f#,a#,f#,a#,c#6,b,a#,b,a#,b,g#,c#6,b,a#,f#,f#,f#'  # реальная запись «Jump &Amp Let's Party» (F# major)


def _lead_pcs(params):
    return {int(x) % 12 for x in params["lead_midi"].split(", ") if x != "None"}


def test_issue_3293_theme_follows_explicit_minor_key():
    """F# major тема + root=A, scale=minor: тема в ладу, не какофония."""
    params = _params(_JUMP_AMP, root="A", scale="minor")
    dec = params["decisions"]
    assert dec["key_detected"][0] == "F#"
    assert dec["key_fit"] >= KEY_FIT_WARN
    assert dec["key_transpose"] != 0
    assert _lead_pcs(params) <= _scale_pcs("A", "minor")
    assert (params["root"], params["scale"]) == ("A", "minor")


def test_issue_3293_explicit_root_alone_moves_theme_keeps_scale():
    params = _params(_JUMP_AMP, root="A")
    assert (params["root"], params["scale"]) == ("A", "major")
    assert _lead_pcs(params) <= _scale_pcs("A", "major")


def test_issue_3293_lead_register_stays_in_working_range():
    for root in VALID_ROOTS:
        params = _params(_JUMP_AMP, root=root, scale="minor")
        notes = [int(x) for x in params["lead_midi"].split(", ") if x != "None"]
        assert 55 <= min(notes) and max(notes) <= 88, root


def test_issue_3293_no_explicit_key_leaves_theme_untouched():
    auto = _params(_JUMP_AMP)
    assert "key_transpose" not in auto["decisions"]
    same = _params(_JUMP_AMP, scale=None)
    assert same["lead_midi"] == auto["lead_midi"]


def test_issue_3293_key_honesty_note_in_compose_music_answer(mock_node):
    """В ОТВЕТЕ compose_music (message), не только в партитуре."""
    rec = {"name": "jump_amp", "title": "Jump", "rtttl": _JUMP_AMP}
    tool, _mgr = _tool(mock_node)
    tool._resolve_melody = lambda _n, _v: rec
    result = tool.execute(name="slavonic party", root="A", scale="minor", **_ARR)
    assert result.success is True, result.error
    assert "Тональность" in result.message
    assert "перенесена" in result.message and "относительный" in result.message
    fresh, _m = _tool(mock_node)  # root/scale прошлого вызова наследуются
    fresh._resolve_melody = lambda _n, _v: rec
    plain = fresh.execute(name="slavonic party", **_ARR)
    assert "Тональность" not in plain.message
