"""Issue #2968 — белый список одиночных FX (выстрел/сирена/скрэтч/лазер).

Живой сет 24.09.2026: Шифу попросил «добавь выстрелы пистолетов» на
гангста-вечеринке — модель не вызвала ни одного тула, а даже вызвав,
упёрлась бы в то же самое, что #2841 нашёл для лупов (spack= не работает
в этой сборке Renardo). Эти тесты фиксируют контракт FX-каталога:

* каталог читается из ``data/sample_fx.json`` (по образцу
  ``data/sample_loops.json``, см. test_sample_loops.py);
* использует ТОТ ЖЕ флаг, что лупы пака 1 (``ROB_BOX_PACK1_LOOPS``) — по
  умолчанию ВЫКЛЮЧЕН, ни один сэмпл не прослушан на 16 kHz DAC;
* санитайзер переписывает имя FX в путь через тот же ``loop(...)`` синт,
  что и жанровый луп (нет отдельного FX-синта в Renardo);
* подбор идёт через жанровые ТЕГИ (``fx_by_genre``), а не через зашитое
  сопоставление «жанр → конкретный сэмпл» (наказ Шифу к #2968: системный
  механизм, белый список — только гарантия безопасности до прослушки).
"""

import pytest

from rob_box_mcp_tools.core import sample_fx, sample_loops
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

MAX_AMP = 0.7

#: Имена файлов подтверждены чтением файлового дерева в контейнере
#: voice-assistant (ssh 10.1.1.21, 24.09.2026) — см. data/sample_fx.json.
EXPECTED_FX = {
    "gunshot_1", "gunshot_2", "siren_1",
    "laser_1", "laser_2", "laser_3", "laser_4", "laser_5",
    "scratch_1", "scratch_2", "scratch_3", "scratch_4", "scratch_5",
}


# ---------------------------------------------------------------------------
# Каталог и флаг
# ---------------------------------------------------------------------------


def test_catalog_lists_expected_fx_candidates():
    assert set(sample_fx.fx_catalog()) == EXPECTED_FX


def test_catalog_path_is_relative_to_default_loop_dir():
    """Тот же трюк, что у лупов (#2841): путь выходит из
    ``0_foxdot_default/_loop_`` на два уровня вверх в пак 1 — БЕЗ
    ``_loop_`` внутри пака 1, FX лежат в обычных буквенных папках."""
    info = sample_fx.fx_catalog()["gunshot_1"]
    assert info.path == "../../1_pitchglitch_samples/u/upper/005_Snare_AltGunShot_Kaonaya.wav"


def test_fx_reuses_the_same_flag_as_loops():
    assert sample_loops.PACK1_LOOPS_ENV == "ROB_BOX_PACK1_LOOPS"
    assert sample_fx.fx_enabled({}) is False
    assert sample_fx.fx_enabled({sample_loops.PACK1_LOOPS_ENV: "1"}) is True


def test_find_fx_accepts_name_wav_and_canonical_path():
    by_name = sample_fx.find_fx("gunshot_1")
    assert by_name is not None
    assert sample_fx.find_fx("gunshot_1.wav") == by_name
    assert sample_fx.find_fx(by_name.path) == by_name
    assert sample_fx.find_fx("no_such_fx") is None


def test_denial_when_flag_off_names_the_flag():
    reason = sample_fx.fx_denial("gunshot_1", enabled=False)
    assert reason is not None
    assert sample_loops.PACK1_LOOPS_ENV in reason


def test_no_denial_when_flag_on():
    assert sample_fx.fx_denial("siren_1", enabled=True) is None


def test_unknown_fx_denial_lists_catalog():
    reason = sample_fx.fx_denial("no_such_fx", enabled=True)
    assert reason is not None
    assert "gunshot_1" in reason


# ---------------------------------------------------------------------------
# Теги — «жанр → теги → поиск», не зашитое сопоставление (наказ Шифу)
# ---------------------------------------------------------------------------


def test_every_fx_has_at_least_one_tag():
    for info in sample_fx.fx_catalog().values():
        assert info.tags, info.name


def test_fx_by_genre_is_data_driven_not_hardcoded():
    """Подбор идёт по тегам каталога, а не по имени файла/переключателю
    в коде — смена JSON меняет выдачу без правки логики."""
    gangsta = sample_fx.fx_by_genre("gangsta")
    assert "gunshot_1" in gangsta
    assert "siren_1" in gangsta
    dnb = sample_fx.fx_by_genre("dnb")
    assert "laser_1" in dnb
    assert set(dnb).isdisjoint({"gunshot_1"})  # выстрел не тегирован dnb


def test_fx_by_genre_unknown_tag_is_empty_not_error():
    assert sample_fx.fx_by_genre("nonexistent-genre-xyz") == ()


def test_tags_for_unknown_name_is_empty():
    assert sample_fx.tags_for("no_such_fx") == ()


# ---------------------------------------------------------------------------
# Санитайзер — тот же loop(...) синт, что и жанровые лупы
# ---------------------------------------------------------------------------


def test_fx_is_rejected_by_default():
    result = sanitize_renando('d3 >> loop("gunshot_1", amp=0.3)', MAX_AMP)
    assert result.quality_errors
    assert any("не прослушан" in e for e in result.quality_errors)


def test_fx_is_rewritten_to_path_when_flag_on():
    result = sanitize_renando(
        'd3 >> loop("gunshot_1", amp=0.3)', MAX_AMP, pack1_loops_enabled=True,
    )
    assert result.quality_errors == ()
    assert (
        "loop('../../1_pitchglitch_samples/u/upper/005_Snare_AltGunShot_Kaonaya.wav'"
        in result.code
    )


def test_rewritten_fx_code_passes_sanitizer_again():
    """Уже переписанный путь (со своим ``.wav`` на конце, в отличие от
    лупов) — тоже валидный аргумент при повторном прогоне (save_track)."""
    first = sanitize_renando(
        'd3 >> loop("siren_1", amp=0.3)', MAX_AMP, pack1_loops_enabled=True,
    )
    second = sanitize_renando(first.code, MAX_AMP, pack1_loops_enabled=True)
    assert second.quality_errors == ()
    assert second.code == first.code


def test_fx_name_does_not_collide_with_loop_catalog():
    """Каталоги лупов и FX не пересекаются по имени — иначе резолвинг
    в sanitizer._resolve_loops был бы неоднозначным."""
    assert set(sample_loops.loop_catalog()) & set(sample_fx.fx_catalog()) == set()


def test_unknown_name_mentions_both_catalogs():
    result = sanitize_renando(
        'd3 >> loop("totally_unknown", amp=0.3)', MAX_AMP, pack1_loops_enabled=True,
    )
    assert any("Лупа/FX" in e and "totally_unknown" in e for e in result.quality_errors)


@pytest.mark.parametrize("name", sorted(EXPECTED_FX))
def test_every_fx_candidate_resolves_when_flag_on(name):
    result = sanitize_renando(
        f'd1 >> loop({name!r}, amp=0.3)', MAX_AMP, pack1_loops_enabled=True,
    )
    assert result.quality_errors == (), (name, result.quality_errors)
