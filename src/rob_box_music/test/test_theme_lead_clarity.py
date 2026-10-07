"""Тема читаема (отзыв Шифу 07.10 о «В пещере горного короля»: «какофония, сильное эхо, тема размазана» у клуба,
рейва и synthwave; chiptune на ``blip`` — «ничотак»).

Правило — данные ``knowledge.LEAD_CLARITY`` (замер ``scripts/music/loudness_nrt_v2.py --clarity``) и пороги
``THEME_*``; применяет одно место — ``arrange.mix.role_palette``. Тесты — на поведении: какой синт реально играет хук
трека, какие эффекты стоят на его строке Renardo, длина нот хука против шага.
"""

from __future__ import annotations

import dataclasses
import random
import re

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix
from rob_box_music.arrange.compose import compose
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

THEME = "в пещере горного короля"
#: Эффекты, которых на строке хука быть не должно (эхо/реверб Renardo — ``echo``/``room``/``verb``).
WET_KEYS = ("echo", "room", "verb")


def test_unreadable_synths_from_the_complaint_are_not_theme_leads():
    """Синты первых трёх стилей 07.10 — вне правила; ``blip`` (chiptune, «ничотак») — внутри."""
    for synth in ("epiano", "hoover", "supersawlead", "rave", "strangerarp", "kalimba", "rhpiano"):
        assert not kn.theme_lead_ok(synth), synth
    assert kn.theme_lead_ok("blip")
    assert not kn.theme_lead_ok("неизмеренный")  # без замера — не читаемый


@pytest.mark.parametrize("name", sorted(kn.STYLES))
def test_every_style_family_keeps_readable_leads_and_picks_only_them(name):
    style = kn.STYLES[name]
    for family in style.timbres:
        palette = mix.role_palette(style, family, "lead")
        assert len(palette) >= 2 and set(palette) <= kn.THEME_LEAD_OK, (name, family, palette)
        assert set(style.timbres[family]["lead"]) <= kn.THEME_LEAD_OK, (name, family)  # таблица не врёт
        for seed in range(12):
            assert mix.role_timbre(style, family, "lead", (), random.Random(seed)) in kn.THEME_LEAD_OK


def test_guard_drops_an_unreadable_lead_even_if_a_table_brings_it_back():
    club = kn.STYLES["club"]
    bad = dataclasses.replace(club, timbres={"hard": {**club.timbres["hard"], "lead": ("supersawlead", "pluck")}})
    assert mix.role_palette(bad, "hard", "lead") == ("pluck",)
    only_bad = dataclasses.replace(club, timbres={"hard": {**club.timbres["hard"], "lead": ("hoover", "epiano")}})
    with pytest.raises(ValueError, match="THEME_LEAD_OK"):
        mix.role_palette(only_bad, "hard", "lead")
    assert mix.role_palette(bad, "hard", "pad") == club.timbres["hard"]["pad"]  # правило темы — только у лида


@pytest.mark.parametrize("name", ["club", "rave", "synthwave", "chiptune"])
def test_theme_hook_plays_readable_dry_and_notes_never_outlast_their_step(name):
    """Сеты «на тему горного короля» по стилям: лид трека — читаемый, строка лида без эха/реверба, расстройка
    второго голоса не шире пэда (``PAD_DETUNE``), нота хука не длиннее шага до следующей."""
    for seed in range(3):
        plan = seeded_plan(seeded_profile(THEME, name), seed, set_id=f"lead{seed}")
        track = compose(plan, 0)
        lead = track.parts["lead"]
        assert lead.synth_or_sample in kn.THEME_LEAD_OK, (name, seed, lead.synth_or_sample)
        program = render(track, "A")
        line = next(ln for ln in program.code.splitlines() if ln.startswith(f"{program.slots['lead']} >> "))
        assert f">> {lead.synth_or_sample}(" in line
        assert not any(re.search(rf"\b{k}=", line) for k in WET_KEYS), line
        for shift in re.findall(r"pshift=\(\(0, ([0-9.]+)\),\)", line):
            assert float(shift) <= kn.PAD_DETUNE, line
        onsets = sorted({e.beat for e in lead.pitches})
        step = {a: b - a for a, b in zip(onsets, onsets[1:])}
        assert all(e.dur_beats <= step[e.beat] + 1e-9 for e in lead.pitches if e.beat in step), (name, seed)
