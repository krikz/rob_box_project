"""Issue #2841 — каталог лупов и белый список pack 1 за флагом.

Живой сет 23.09.2026: ``loop()`` не прозвучал ни разу, пак 1 недоступен
целиком. Эти тесты фиксируют контракт:

* каталог читается из ``data/sample_loops.json`` (25 кандидатов pack 1 +
  ``foxdot`` из пака 0);
* флаг ``ROB_BOX_PACK1_LOOPS`` по умолчанию ВЫКЛЮЧЕН — непрослушанный звук
  не должен доехать до динамика, пока Шифу его не прослушал;
* санитайзер переписывает имя лупа в путь, который Renardo найдёт, и
  честно отказывает во всём остальном (не-литерал, имя вне каталога).
"""

import pytest

from rob_box_mcp_tools.core import sample_loops
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

MAX_AMP = 0.7

#: 25 лупов из ``/opt/rob_box/samples/1_pitchglitch_samples/_loop_`` (ls на
#: Vision Pi, 23.09.2026).
ROBOT_PACK1_LOOPS = {
    "ambient_1", "arabic_1", "beatbox_1", "break_1", "dnb_1", "dnb_2",
    "dnb_3", "electro_1", "electro_2", "funk_1", "future_1", "future_2",
    "glitch_1", "glitch_2", "guitar_1", "hiphop_1", "hiphop_2",
    "industrial_1", "jazzhop_1", "jungle_1", "jungle_2", "perc_1", "perc_2",
    "piano_1", "yiddish_1",
}


# ---------------------------------------------------------------------------
# Каталог и флаг
# ---------------------------------------------------------------------------


def test_catalog_lists_all_robot_pack1_loops_as_candidates():
    catalog = sample_loops.loop_catalog()
    pack1 = {name for name, info in catalog.items() if info.pack == 1}
    assert pack1 == ROBOT_PACK1_LOOPS


def test_catalog_keeps_pack0_foxdot_loop():
    info = sample_loops.loop_catalog()["foxdot"]
    assert info.pack == 0
    assert info.path == "foxdot"


def test_pack1_path_is_relative_to_default_loop_dir():
    """Renardo ищет лупы только в ``0_foxdot_default/_loop_`` — путь пака 1
    обязан выходить оттуда на два уровня вверх."""
    info = sample_loops.loop_catalog()["dnb_1"]
    assert info.path == "../../1_pitchglitch_samples/_loop_/dnb_1"


def test_flag_defaults_to_off():
    assert sample_loops.pack1_loops_enabled({}) is False


@pytest.mark.parametrize("value", ["1", "true", "YES", " on "])
def test_flag_truthy_values_enable(value):
    assert sample_loops.pack1_loops_enabled({sample_loops.PACK1_LOOPS_ENV: value})


@pytest.mark.parametrize("value", ["0", "false", "", "no"])
def test_flag_falsy_values_keep_it_off(value):
    assert not sample_loops.pack1_loops_enabled({sample_loops.PACK1_LOOPS_ENV: value})


def test_find_loop_accepts_name_wav_and_canonical_path():
    by_name = sample_loops.find_loop("dnb_1")
    assert by_name is not None
    assert sample_loops.find_loop("dnb_1.wav") == by_name
    assert sample_loops.find_loop(by_name.path) == by_name
    assert sample_loops.find_loop("no_such_loop") is None


def test_denial_for_pack1_when_flag_off_names_the_flag():
    reason = sample_loops.loop_denial("dnb_1", enabled=False)
    assert reason is not None
    assert sample_loops.PACK1_LOOPS_ENV in reason


def test_no_denial_for_pack0_even_when_flag_off():
    assert sample_loops.loop_denial("foxdot", enabled=False) is None


def test_no_denial_for_pack1_when_flag_on():
    assert sample_loops.loop_denial("jungle_1", enabled=True) is None


# ---------------------------------------------------------------------------
# Длина лупа под темп
# ---------------------------------------------------------------------------


def test_loop_beats_picks_nearest_power_of_two():
    # jungle_1 = 2.0 с; на 120 BPM это ровно 4 бита.
    info = sample_loops.loop_catalog()["jungle_1"]
    assert sample_loops.loop_beats(info, 120) == 4.0
    assert sample_loops.loop_rate(info, 120) == pytest.approx(1.0)


def test_loop_beats_follows_tempo():
    # dnb_1 = 1.396 с: 170 BPM → ~3.96 бита → 4; 80 BPM → ~1.86 → 2.
    info = sample_loops.loop_catalog()["dnb_1"]
    assert sample_loops.loop_beats(info, 170) == 4.0
    assert sample_loops.loop_beats(info, 80) == 2.0


def test_loop_rate_stays_within_sqrt2_of_native():
    """Ближайшая степень двойки по логарифму: скорость не уходит дальше
    чем в √2 раз — иначе выбралась бы соседняя степень."""
    for info in sample_loops.loop_catalog().values():
        for bpm in (60, 90, 120, 150, 180):
            rate = sample_loops.loop_rate(info, bpm)
            if sample_loops.loop_beats(info, bpm) in (1.0, 32.0):
                continue  # край шкалы — там отклонение может быть больше
            assert 2 ** -0.5 - 1e-9 <= rate <= 2 ** 0.5 + 1e-9, (info.name, bpm, rate)


# ---------------------------------------------------------------------------
# Санитайзер
# ---------------------------------------------------------------------------


def test_pack1_loop_is_rejected_by_default():
    result = sanitize_renando('d3 >> loop("dnb_1", dur=4, beat_stretch=1)', MAX_AMP)
    assert result.quality_errors
    assert any("не прослушан" in e for e in result.quality_errors)


def test_pack1_loop_is_rewritten_to_path_when_flag_on():
    result = sanitize_renando(
        'd3 >> loop("dnb_1", dur=4, beat_stretch=1)', MAX_AMP,
        pack1_loops_enabled=True,
    )
    assert result.quality_errors == ()
    assert "loop('../../1_pitchglitch_samples/_loop_/dnb_1', dur=4" in result.code


def test_rewritten_code_passes_sanitizer_again():
    """Уже переписанный путь — тоже валидный аргумент (код, сохранённый
    через save_track, проходит санитайзер повторно)."""
    first = sanitize_renando(
        'd3 >> loop("hiphop_1", dur=4)', MAX_AMP, pack1_loops_enabled=True
    )
    second = sanitize_renando(first.code, MAX_AMP, pack1_loops_enabled=True)
    assert second.quality_errors == ()
    assert second.code == first.code


def test_pack0_loop_passes_without_flag():
    result = sanitize_renando('d3 >> loop("foxdot", dur=8)', MAX_AMP)
    assert result.quality_errors == ()
    assert "loop('foxdot', dur=8" in result.code


def test_unknown_loop_is_hard_error_even_with_flag():
    result = sanitize_renando(
        'd3 >> loop("/etc/passwd", dur=4)', MAX_AMP, pack1_loops_enabled=True
    )
    assert any("нет в каталоге" in e for e in result.quality_errors)


def test_non_literal_loop_name_is_hard_error():
    result = sanitize_renando(
        'name = "dnb_1"\nd3 >> loop(name, dur=4)', MAX_AMP, pack1_loops_enabled=True
    )
    assert any("строковым литералом" in e for e in result.quality_errors)


def test_pattern_loop_method_is_not_mistaken_for_loop_player():
    """``P[:8].loop(2)`` — метод паттерна, а не плеер ``loop``."""
    result = sanitize_renando(
        "p1 >> pluck(P[0, 2, 4].loop(2), dur=0.5)", MAX_AMP
    )
    assert result.quality_errors == ()


def test_spack_still_rejected_with_honest_reason():
    result = sanitize_renando('d1 >> play("x", spack=1)', MAX_AMP, pack1_loops_enabled=True)
    assert any("spack" in e and "игнорируется" in e for e in result.quality_errors)
