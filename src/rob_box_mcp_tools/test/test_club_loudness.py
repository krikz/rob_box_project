"""Issue #3154 — статическая модель громкости club и калибровка уровней.

Числа модели — офлайн-рендер (``scripts/music/club_loudness_nrt.py``), НЕ
замер на роботе; живой замер jack_rec делает координатор. Эти тесты держат
три вещи:

* модель покрывает всю палитру каркаса (новый синт/рисунок без замера не
  пройдёт молча);
* модель сходится с офлайн-рендером треков, которые звучали вживую
  (до калибровки: те же сиды, что в комментариях issue);
* калибровка по модели: основной блок любого каркаса на уровне стиля,
  тихие секции не тише основного больше чем на ~10 dB (остаток — только
  там, где слои фактуры упёрлись в потолок, и это видно).
"""

from __future__ import annotations

import itertools

import pytest

from rob_box_mcp_tools.core import club_loudness as L
from rob_box_mcp_tools.core.club_arranger import (
    CLUB_TEMPLATES,
    HATS_PATTERNS,
    KICK_PATTERNS,
    LAYER_LEVELS,
    MAX_LAYER_AMP,
    ROLE_SYNTHS,
    build_matrix,
    club_kit,
)


def _kits():
    """Покрытие палитры без полного перебора (2700 каркасов ≈ 1 мин).

    Каждый шаблон × каждый пэд × каждый лид (пэд и лид решают тихие
    секции), бас/бочка/хэты по кругу — каждый вариант встречается.
    """
    kicks, hats, basses = sorted(KICK_PATTERNS), sorted(HATS_PATTERNS), ROLE_SYNTHS["bass"]
    for i, (template, pad, lead) in enumerate(
        itertools.product(CLUB_TEMPLATES, ROLE_SYNTHS["pad"], ROLE_SYNTHS["lead"])
    ):
        yield dict(template=template, pad=pad, lead=lead, bass=basses[i % len(basses)],
                   kick=kicks[i % len(kicks)], hats=hats[i % len(hats)])


KITS = list(_kits())


def test_model_covers_the_whole_kit_palette():
    assert set(L.LANE_DB_AT_UNIT["kick"]) == set(KICK_PATTERNS)
    assert set(L.LANE_DB_AT_UNIT["hats"]) == set(HATS_PATTERNS)
    for role in ("lead", "bass", "pad"):
        assert set(L.LANE_DB_AT_UNIT[role]) == set(ROLE_SYNTHS[role]), role
    assert {k for kit in KITS for k in kit.items()} >= {
        (key, value) for key, values in (("kick", KICK_PATTERNS), ("hats", HATS_PATTERNS)) for value in values
    }


def test_style_multiplier_targets_classic_level():
    """Общий множитель стиля: основной блок club = громкая секция classic."""
    assert L.TARGET_MAIN_DB == L.STYLE_MAIN_DB["classic"]


# Офлайн-рендер ДО калибровки (LAYER_LEVELS как есть), dB RMS по блокам,
# 124 BPM — те самые сиды из живых замеров issue #3154.
NRT_BEFORE = {
    # превью 29.09 (живое −52…−55 dB все 62 с)
    (7886811, "D"): [-53.8, -55.1, -54.4, -29.3, -23.7, -23.7, -23.1, -21.5,
                     -22.4, -21.9, -24.3, -53.9, -22.5, -21.9, -21.5, -22.9],
    # трек #2 29.09 (живое −22…−25 dB, «секция на −70 dB» — блок 6)
    (4433602, "D"): [-22.5, -24.3, -22.0, -24.2, -52.2, -54.7, -72.0, -29.4,
                     -22.5, -24.3, -24.4, -55.7, -22.5, -24.3, -22.0, -23.4],
    # трек сета 28.09 (живое −55 dB ~45 с после перехода)
    (3330602, "A"): [-21.1, -20.1, -19.9, -21.0, -54.6, -54.0, -54.2, -26.2,
                     -21.1, -20.0, -22.1, -53.4, -21.1, -20.1, -19.9, -19.9],
}


@pytest.mark.parametrize("seed, root", sorted(NRT_BEFORE))
def test_model_matches_offline_render_before_calibration(seed, root):
    kit = club_kit(seed)
    matrix = build_matrix(kit["template"])
    model = L.block_db(matrix, kit, L.flat_levels(matrix, LAYER_LEVELS))
    rendered = NRT_BEFORE[(seed, root)]
    assert max(abs(a - b) for a, b in zip(model, rendered)) <= 3.5
    main = L.main_blocks(matrix)
    assert sum(model[i] - rendered[i] for i in main) / len(main) == pytest.approx(0, abs=1.5)


def test_before_calibration_quiet_sections_were_30db_down():
    """Причина дефекта в модели: секции без бочки на 30–50 dB тише основного."""
    for seed, _root in NRT_BEFORE:
        kit = club_kit(seed)
        report = L.kit_report(build_matrix(kit["template"]), kit, LAYER_LEVELS, MAX_LAYER_AMP)
        assert report["before"]["worst_drop_db"] >= 30, seed


@pytest.mark.parametrize("kit", KITS, ids=lambda k: "-".join(k[x] for x in ("template", "pad", "lead")))
def test_calibrated_main_block_is_on_target(kit):
    matrix = build_matrix(kit["template"])
    levels = L.calibrate_levels(matrix, kit, LAYER_LEVELS, MAX_LAYER_AMP)
    assert L.main_db(matrix, kit, levels) == pytest.approx(L.TARGET_MAIN_DB, abs=0.5)
    assert all(0 <= v <= MAX_LAYER_AMP + 1e-9 for values in levels.values() for v in values)


@pytest.mark.parametrize("kit", KITS, ids=lambda k: "-".join(k[x] for x in ("template", "pad", "lead")))
def test_quiet_sections_lifted_or_honestly_capped(kit):
    """Тихий блок либо ≥ основной − SECTION_FLOOR_DB, либо все его слои
    фактуры на потолке (синт не дотягивает — остаток виден, не спрятан)."""
    matrix = build_matrix(kit["template"])
    levels = L.calibrate_levels(matrix, kit, LAYER_LEVELS, MAX_LAYER_AMP)
    main = L.main_db(matrix, kit, levels)
    blocks = L.block_db(matrix, kit, levels)
    for block, db in enumerate(blocks):
        if db >= main - L.SECTION_FLOOR_DB - 0.2:
            continue
        lift = [lane for lane in L.LIFT_LANES if matrix.lanes[lane][block]]
        assert lift and all(levels[lane][block] == pytest.approx(MAX_LAYER_AMP) for lane in lift), (block, db)


def test_residual_drop_by_pad_synth():
    """Остаток по модели: warmpad держит 8 dB, space упирается в потолок
    (~8.3), sinepad — самый тихий пэд (HPF 1 кГц), ~10.5 dB. Офлайн-рендер
    sinepad-каркасов показал до 16 dB (уровень sinepad зависит от высоты
    аккорда, модель это усредняет) — см. PR #3154."""
    worst = {}
    for kit in KITS:
        report = L.kit_report(build_matrix(kit["template"]), kit, LAYER_LEVELS, MAX_LAYER_AMP)
        worst[kit["pad"]] = max(worst.get(kit["pad"], 0.0), report["after"]["worst_drop_db"])
    assert worst["warmpad"] <= L.SECTION_FLOOR_DB + 0.2
    assert worst["space"] <= 9.0
    assert worst["sinepad"] <= 11.0


def test_live_neighbours_are_within_3db_after_calibration():
    """Превью (seed 7886811) и трек #2 (seed 4433602) живого сета 29.09:
    до калибровки по модели основной блок −21.8 и −23.0, интро превью −54;
    после — оба на цели, а вход превью с блока бочки (#3166-путь)."""
    mains = []
    for seed in (7886811, 4433602, 3330602, 2126493):
        kit = club_kit(seed)
        matrix = build_matrix(kit["template"])
        mains.append(L.main_db(matrix, kit, L.calibrate_levels(matrix, kit, LAYER_LEVELS, MAX_LAYER_AMP)))
    assert max(mains) - min(mains) <= 3.0
