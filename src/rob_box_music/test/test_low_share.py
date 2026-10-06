"""A9′ (низ дропа ≥ 0.5, ADR-0152 §4) в модели микса на роботе: ``mix.low_share`` по ``Mix`` трека (#3401, #3441).

Мощность роли в секции — ``Mix.level_db`` (dB RMS роли, звучащей всю секцию), под сайдчейном — × средняя мощность
огибающей секции, бочка рисунка build (2 удара на такт вместо 4) — пополам; FX — один удар на секцию, не в счёт;
пэд — в шкале ``pumped16`` со сдвигом синта на роботе (``knowledge.PAD_ROBOT_DB``).
Сверка с записью робота 06.10 (ser6, 5 сетов, 46 дропов): модель − запись по низу дропа — медиана −0.004, σ 0.078
(было: «робот = 0.62 модели» #3401 — σ 0.166, A9-модель 0.81–0.87 всем трекам при записи 0.31–0.85).
"""

from __future__ import annotations

import statistics
from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix
from rob_box_music.arrange.compose import compose
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

THEMES = ("славянская вечеринка", "космос", "киберпанк", "детский праздник")
CLUB = kn.STYLES["club"]


def _low_shares():
    shares = {}
    for theme in THEMES:
        plan = seeded_plan(seeded_profile(theme), 11, set_id="s11")
        for n in range(1, 6):
            track = compose(plan, n, deck="A")
            offset = kn.PAD_FIGURES[track.history_key.pad_figure].level_offset_db
            for i, sec in enumerate(track.form.sections):
                share = mix.low_share(CLUB, track.parts, track.form, i, track.mix.duck[i], track.mix.duck_roles,
                                      offset)
                shares.setdefault(sec.name, []).append(share)
    return shares


@pytest.fixture(scope="module")
def low_shares():
    return _low_shares()


@pytest.mark.parametrize("section", ["drop", "drop2"])
def test_drops_reach_a9_on_the_robot(low_shares, section):
    """A9′: медиана низа дропов ≥ 0.5 и каждый дроп не ниже порога A9-модели стиля (запас на σ модели)."""
    assert statistics.median(low_shares[section]) >= 0.5
    assert min(low_shares[section]) >= CLUB.a9_model_low


def test_build_keeps_low_without_bass(low_shares):
    """Build без баса (#3396): низ держит одна бочка. Build модель не сверен с записью (ser6: модель выше записи
    на 0.09, σ 0.13 — LPF-свип и ролл клэпа модель не видит), A9′ его не касается: тест держит, что бочка не
    утонула в модели."""
    assert statistics.median(low_shares["build"]) >= 0.35


def test_a9_model_hears_the_pad_synth_like_the_robot():
    """ser6 (#3441): на одном уровне ``ambi`` на роботе забирал низ дропа (0.49 против 0.75 у ``sinepad``) —
    A9-модель без поправки видит то же, а с поправкой ``ambi`` тише и дроп не ниже порога."""
    track = compose(seeded_plan(seeded_profile(THEMES[0]), 11, set_id="s11"), 2, deck="A")
    untrimmed = replace(CLUB, a9_model_low=0.0)
    model = {}
    for pad in ("sinepad", "ambi"):
        parts = {**track.parts, "pad": replace(track.parts["pad"], synth_or_sample=pad)}
        model[pad] = mix.mix_parts(untrimmed, parts, track.form, "pumped16")[1].a9_model
        trimmed = mix.mix_parts(CLUB, parts, track.form, "pumped16")[1]
        assert trimmed.a9_model >= CLUB.a9_model_low
    assert model["ambi"] < CLUB.a9_model_low <= model["sinepad"], model
