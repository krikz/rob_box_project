"""Регрессия issue #2978 на живых кейсах (24.09.2026): гимн, Терминатор, DJ-петля.

Живая жалоба: состав слоёв (d1/d2/p1/p2/p3/d3) на прогонах ``soviethy``
(гимн), ``terminat`` (Терминатор) и DJ-сетов был ОДИНАКОВ от intro до
outro — форма меняла только громкость, не состав. Тест не подгоняет
ничего под эти конкретные записи (правило — общая таблица
:data:`core.arranger.FORMS`, см. ``test_form_role_plan.py``): он лишь
фиксирует, что на РЕАЛЬНОМ прогоне архива состав действительно меняется
по ходу формы, вместо очередной ручной правки под одну тему.
"""

from __future__ import annotations

import gzip
import json
from pathlib import Path

import pytest

from rob_box_mcp_tools.core.arranger import render, spec_from_flat
from rob_box_mcp_tools.core.rtttl_compose import melody_to_compose_params, rtttl_to_melody
from rob_box_mcp_tools.core.score_sheet import describe

_ARCHIVE = (
    Path(__file__).resolve().parents[1]
    / "rob_box_mcp_tools" / "data" / "rtttl_melodies.jsonl.gz"
)

#: Живые кейсы 24.09.2026: soviethy — гимн, terminat — тема Терминатора.
_LIVE_KEYS = ("soviethy", "terminat")


@pytest.fixture(scope="module")
def archive():
    rows = {}
    with gzip.open(_ARCHIVE, "rt", encoding="utf-8") as fh:
        for line in fh:
            row = json.loads(line)
            rows.setdefault(row["name"], row)
    return rows


def _sheet(rtttl: str, **flat):
    params = melody_to_compose_params(rtttl_to_melody(rtttl))
    kw = dict(form="arc", lead_synth="blip", bass_synth="moogbass", pad_synth="strings")
    kw.update(flat)
    spec = spec_from_flat(
        harmony=params["harmony"], bpm=float(params["bpm"]),
        root=str(params["root"]), scale=str(params["scale"]),
        lead_midi=str(params["lead_midi"]), lead_dur=str(params["lead_dur"]),
        **kw,
    )
    code = render(spec)
    return describe(spec=spec, code=code, harmony=params["harmony"],
                     prep_decisions=params["decisions"])


@pytest.mark.parametrize("key", _LIVE_KEYS)
@pytest.mark.parametrize("repeat", [False, True], ids=["track", "dj_loop"])
def test_role_count_varies_across_the_form(archive, key, repeat):
    """Не тот же баг: секции формы играют РАЗНЫМ числом ролей, не одним."""
    sheet = _sheet(archive[key]["rtttl"], repeat=repeat)
    counts = [len(roles) for _name, roles in sheet["form"]["composition"]]
    assert len(set(counts)) > 1, (
        f"{key}: во всех секциях играет одно и то же число ролей {counts} — "
        "тот же баг, что issue #2978"
    )


@pytest.mark.parametrize("key", _LIVE_KEYS)
def test_a_section_has_the_theme_alone_over_minimal_accompaniment(archive, key):
    """Хотя бы одна секция — тема поверх минимального аккомпанемента."""
    sheet = _sheet(archive[key]["rtttl"])
    composition = dict(sheet["form"]["composition"])
    found = any(
        "lead" in roles and len(set(roles) - {"lead"}) <= 2
        for roles in composition.values()
    )
    assert found, f"{key}: ни одна секция не соло темы: {composition}"


@pytest.mark.parametrize("key", _LIVE_KEYS)
def test_drums_are_silent_in_the_opening_section(archive, key):
    """Барабаны не должны звучать с первой же секции — это вход слоя, не
    статичный набор на весь трек (тот же факт, что ``TestForm.
    test_layer_enters_and_leaves_across_the_form`` проверяет на синтетике)."""
    sheet = _sheet(archive[key]["rtttl"])
    opening_roles = sheet["form"]["composition"][0][1]
    assert "drums" not in opening_roles, (
        f"{key}: барабаны уже в открывающей секции {sheet['form']['composition'][0]}"
    )
