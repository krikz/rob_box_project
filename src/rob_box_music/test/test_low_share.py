"""A9 (низ ≥ 0.5) в модели микса: доля мощности бочки и баса в дропах по ``Mix`` трека (#3401).

Мощность роли в секции — ``Mix.level_db`` (dB RMS роли, звучащей всю секцию), под сайдчейном — × средняя мощность
огибающей секции, бочка рисунка build (2 удара на такт вместо 4) — пополам; FX — один удар на секцию, не в счёт.
Перемер на роботе 05.10 (5 сетов, 25 треков): низ <150 Гц дропа = 0.62 модельной доли бочки+баса (0.42 при 0.68),
второго дропа — 0.61 (0.37 при 0.61). Значит A9 в дропе требует модельной доли ≥ 0.5 / 0.62 ≈ 0.8.
"""

from __future__ import annotations

import statistics

import pytest

from rob_box_music.arrange import mix, samples
from rob_box_music.arrange.compose import compose
from rob_box_music.model import SAMPLE_ROLES
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

THEMES = ("славянская вечеринка", "космос", "киберпанк", "детский праздник")
#: Низ дропа на роботе / модельная доля бочки+баса (перемер 05.10, медианы по 25 и 22 дропам).
ROBOT_LOW_PER_MODEL_SHARE = {"drop": 0.62, "drop2": 0.61}


def _section_power(track, i):
    sec = track.form.sections[i]
    duck = track.mix.duck[i]
    env = mix.duck_envelope(duck.trigger, duck.depth)
    duck_power = sum(g * g for g in env) / len(env)
    out = {}
    for role, db in track.mix.level_db.items():
        if role in SAMPLE_ROLES:
            if role == "fx" or sec.name not in samples.SECTIONS[role]:
                continue
        elif role not in sec.roles:
            continue
        p = 10 ** (db / 10)
        if role in track.mix.duck_roles:
            p *= duck_power
        if role == "kick":
            p *= mix.look(sec.energy).kick.count("X") / 4
        out[role] = p
    return out


def _low_shares():
    shares = {}
    for theme in THEMES:
        plan = seeded_plan(seeded_profile(theme), 11, set_id="s11")
        for n in range(1, 6):
            track = compose(plan, n, deck="A")
            for i, sec in enumerate(track.form.sections):
                pw = _section_power(track, i)
                total = sum(pw.values())
                if total:
                    shares.setdefault(sec.name, []).append((pw.get("kick", 0) + pw.get("bass", 0)) / total)
    return shares


@pytest.fixture(scope="module")
def low_shares():
    return _low_shares()


@pytest.mark.parametrize("section", ["drop", "drop2"])
def test_drops_reach_a9_by_the_robot_calibration(low_shares, section):
    expected = statistics.median(low_shares[section]) * ROBOT_LOW_PER_MODEL_SHARE[section]
    assert expected >= 0.49, f"{section}: ожидаемый низ на роботе {expected:.2f}"


def test_every_drop_is_carried_by_kick_and_bass(low_shares):
    """Ни один дроп не отдаёт середине больше четверти мощности (сэмплы и луп с разными файлами — разный потолок)."""
    assert min(low_shares["drop"]) >= 0.8
    assert min(low_shares["drop2"]) >= 0.72


def test_build_keeps_low_without_bass(low_shares):
    """Build без баса (#3396): низ держит одна бочка — её доля в модели не ниже половины."""
    assert statistics.median(low_shares["build"]) >= 0.5
