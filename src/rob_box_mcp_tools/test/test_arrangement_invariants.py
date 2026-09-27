"""Гейт CI: инварианты аранжировки, не тональность одной песни (ADR-0132 PR-8).

ADR-0132 §7: точная таблица ожидаемой тональности по ~35 темам архива
(бывшая ``_KEY_REFERENCE``, теперь ``scripts/music/reference_report.py``)
перестала быть гейтом CI — подгонка весов ``detect_key`` под конкретные
темы однажды уже сменила тональность у 24% архива (issue #2873), улучшая
ровно те темы, ради которых её правили, и портя случайное подмножество
остальных.

Вместо этого гейт проверяет ИНВАРИАНТЫ выведенной аранжировки — то, что
должно быть верно для ЛЮБОЙ темы, а не для конкретных 35: бас держится
лада/аккорда, пэд не тонет в басе, лид в рабочем регистре, партии не
расходятся по длине, плееров не больше слотов деплоя, код проходит
санитайзер, рендер детерминирован. Выборка — те же ~35 эталонных тем
(инвариант тоже стоит проверить на «трудных» темах) + детерминированная
seed-выборка ~200 тем реального архива (:class:`RtttlLibrary`), чтобы
гейт не зависел от конкретного маленького набора.

ВАЖНО (ADR-0132 §7 / CONTRIBUTING.md): жалоба на звучание конкретной песни
чинится ручкой (:mod:`core.harmonize`/:mod:`core.arranger` ``*Options``)
или пресетом, НЕ правкой авто-констант этого пайплайна. Правка авто-констант
(``_pick_chords``, ``detect_key`` веса, ``_pad_ceiling`` и т.п.) допускается
только с отчётом ``scripts/music/reference_report.py`` по ВСЕЙ выборке,
приложенным к PR. Если инвариант ниже не выполняется на сегодняшнем коде —
здесь фиксируется ИЗМЕРЕННЫЙ baseline с комментарием, а не правится
аранжировщик заодно с тестовой политикой.

Сочетание ручек аранжировки (``DEFAULT_FLAT``) — то же самое, что использует
``scripts/music/reference_report.py`` для сводки по той же выборке; при
правке порогов/синтов держи оба файла в синхроне (комментарий в обоих).
"""

from __future__ import annotations

import gzip
import json
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Tuple

import pytest

from rob_box_mcp_tools.core.arranger import SCALE_INTERVALS, VALID_ROOTS, render, spec_from_flat
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando
from rob_box_mcp_tools.core.rtttl_compose import melody_to_compose_params, rtttl_to_melody
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.core.score_sheet import describe

MAX_AMP = 0.7

#: Детерминированная seed-выборка архива — тот же сид/размер, что в
#: ``scripts/music/reference_report.py``, чтобы цифры двух отчётов были
#: сравнимы. ~30 сек. бюджет CI: замер на этой машине — около 3-5 с на
#: полную выборку (рендер + санитайзер + партитура), см. отчёт PR.
SAMPLE_SIZE = 200
SAMPLE_SEED = 20260924

#: Порог «бас держится лада или аккорда» по ДЛИТЕЛЬНОСТИ (не по количеству
#: нот — короткий проходящий тон не должен весить как половинная нота).
#: ADR-0132 §7 явно просит "по длительности ≥ 90%"; измерено: на всей
#: выборке (эталоны + 200 сид-тем) худшая тема даёт 0.9375 — порог держит
#: реальный запас, а не подогнан впритык.
BASS_FIT_MIN = 0.90

#: Диапазон лида, ADR-0132 §7. Строгий min/max (без учёта длительности)
#: промахивается на ~17% сид-выборки на единичных коротких проходящих
#: нотах-выбросах у самого края регистра — метрика по ДЛИТЕЛЬНОСТИ честнее.
#: Но и она не проходит порог 0.90 на части ЭТАЛОННЫХ тем: измерено —
#: ``mozart40`` 0.756, ``indianaj_4`` 0.890 (обе — темы, на которых раньше
#: точечно правили регистр лида, issue #2840; на 200 сид-темах архива хуже
#: 0.9024). ADR-0132 §7: если инвариант не выполняется на сегодняшнем
#: коде — фиксируем измеренный baseline вместо правки аранжировщика заодно
#: с тестовой политикой (PR-8 нот не трогает). Порог ниже худшего измерения
#: с небольшим запасом — ловит структурную регрессию, не сегодняшнее
#: поведение.
LEAD_FIT_MIN = 0.70
LEAD_RANGE = (55, 88)

#: Измеренный СЕГОДНЯШНИЙ максимум "пэд выше низа корпуса темы"
#: (``score_sheet._checks``: ``pad.hi - лид[10-й перцентиль]``) на
#: объединении эталонов + сид-выборки: худшее значение — 26 (``mozart40``,
#: тема 8 тактов — короткая тема даёт неустойчивый 10-й перцентиль), на
#: 200 сид-темах архива хуже 9. Инвариант "пэд ниже корпуса темы" СЕГОДНЯ
#: не выполняется строго — ADR-0132 §7 разрешает зафиксировать измеренный
#: baseline вместо правки аранжировщика заодно с тестовой политикой (PR-8
#: не трогает ноты). Порог даёт запас над измеренным максимумом, чтобы
#: ловить резкую регрессию, а не сегодняшнее поведение.
PAD_UNDER_BODY_MAX = 30

#: Ограничение деплоя — плееры удержаны в d1-d3/p1-p3 (см. arranger.py).
MAX_PLAYERS = 6

#: Ручки аранжировки "по умолчанию" — держать в синхроне с
#: ``scripts/music/reference_report.py::DEFAULT_FLAT``.
DEFAULT_FLAT = dict(
    form="arc", lead_synth="pluck", bass_synth="bass", pad_synth="strings",
    repeat=False,
)

#: ~35 эталонных тем ADR-0132/#2873 (имена, без ожидаемой тональности — та
#: таблица переехала в ``scripts/music/reference_report.py``). Держим их
#: в выборке инвариантов намеренно: это темы, на которых раньше точечно
#: правили автоматику, — хороший стресс-тест для инвариантов.
REFERENCE_THEME_KEYS: Tuple[str, ...] = (
    "hallofth_2", "mountainking", "hallofth", "hallofth_3", "inthehal",
    "national_2", "furelise", "odetojoy", "odetojoy_2", "5thsymph",
    "imperial", "tetris", "tetris_2", "supermar_4", "jesujoyo", "eineklei",
    "badineri", "mozart40", "4seasons_5", "ladonnae", "indianaj_4",
    "raidersm", "james_bond", "missionimpossi", "pinkpant", "rondoala",
    "stilldre", "nextepis", "terminat", "jinglebe_2", "thefirst",
    "whitechr", "rudolpht_2", "rudolpht_4", "toccata", "jinglebe_5",
)

_ARCHIVE = (
    Path(__file__).resolve().parents[1]
    / "rob_box_mcp_tools" / "data" / "rtttl_melodies.jsonl.gz"
)


def _archive_names() -> List[str]:
    names: List[str] = []
    seen = set()
    with gzip.open(_ARCHIVE, "rt", encoding="utf-8") as fh:
        for line in fh:
            name = json.loads(line)["name"]
            if name not in seen:
                seen.add(name)
                names.append(name)
    return names


def _sampled_theme_keys() -> Tuple[str, ...]:
    names = sorted(_archive_names())
    rnd = random.Random(SAMPLE_SEED)
    return tuple(rnd.sample(names, min(SAMPLE_SIZE, len(names))))


SAMPLE_THEME_KEYS: Tuple[str, ...] = _sampled_theme_keys()
#: Полная выборка гейта: эталоны + seed-сэмпл, без дублей, порядок стабилен.
ALL_THEME_KEYS: Tuple[str, ...] = tuple(
    dict.fromkeys(REFERENCE_THEME_KEYS + SAMPLE_THEME_KEYS)
)


def _scale_pitch_classes(root: str, scale: str) -> set:
    semitone = VALID_ROOTS.index(root) if root in VALID_ROOTS else 0
    intervals = SCALE_INTERVALS.get(scale, SCALE_INTERVALS["minor"])
    return {(semitone + i) % 12 for i in intervals}


def _chord_at(harmony, beat: float):
    for chord in harmony.chords:
        if chord.start <= beat < chord.start + chord.beats:
            return chord
    return harmony.chords[-1]


def _bass_fit_ratio(harmony) -> float:
    """Доля длительности баса, что в ладу темы ИЛИ в тонах текущего аккорда."""
    scale = _scale_pitch_classes(harmony.root, harmony.scale)
    cursor = total = fit = 0.0
    for note, dur in harmony.bass:
        if note is not None:
            total += dur
            chord = _chord_at(harmony, cursor)
            if note % 12 in scale or note % 12 in chord.pitch_classes:
                fit += dur
        cursor += dur
    return fit / total if total else 1.0


def _lead_fit_ratio(harmony, lo: int = LEAD_RANGE[0], hi: int = LEAD_RANGE[1]) -> float:
    total = sum(dur for note, dur in harmony.lead if note is not None)
    inside = sum(dur for note, dur in harmony.lead if note is not None and lo <= note <= hi)
    return inside / total if total else 1.0


def _render_default(rtttl: str):
    """Тот же путь, что ``ComposeMusicTool.execute`` при ``name=``, ручки — auto."""
    params = melody_to_compose_params(rtttl_to_melody(rtttl))
    harmony = params["harmony"]
    spec = spec_from_flat(
        harmony=harmony, bpm=float(params["bpm"]), root=str(params["root"]),
        scale=str(params["scale"]), lead_midi=str(params["lead_midi"]),
        lead_dur=str(params["lead_dur"]), **DEFAULT_FLAT,
    )
    return harmony, spec, render(spec)


@dataclass
class _Theme:
    harmony: Any
    spec: Any
    code: str
    checks: Dict[str, Any]
    sanitizer_ok: bool


@dataclass
class _RenderedThemes:
    themes: Dict[str, _Theme] = field(default_factory=dict)
    skipped: int = 0


@pytest.fixture(scope="module")
def library(tmp_path_factory) -> RtttlLibrary:
    db = tmp_path_factory.mktemp("rtttl_invariants") / "lib.db"
    return RtttlLibrary(db_path=str(db))


def _render_one(library: RtttlLibrary, name: str) -> Any:
    """Отрендерить одну тему; ``None`` — темы нет в архиве или не разобралась."""
    entry = library.get(name)
    if entry is None:
        return None
    try:
        harmony, spec, code = _render_default(entry["rtttl"])
    except ValueError:
        return None
    sheet = describe(spec=spec, code=code, harmony=harmony)
    sanitized = sanitize_renando(code, MAX_AMP)
    sanitizer_ok = not (
        sanitized.security_error or sanitized.quality_errors or sanitized.slot_error
    )
    return _Theme(harmony, spec, code, sheet["checks"], sanitizer_ok)


@pytest.fixture(scope="module")
def rendered_themes(library: RtttlLibrary) -> _RenderedThemes:
    """Гармонизировать + отрендерить всю выборку один раз (все тесты её делят)."""
    out = _RenderedThemes()
    for name in ALL_THEME_KEYS:
        theme = _render_one(library, name)
        if theme is None:
            out.skipped += 1
        else:
            out.themes[name] = theme
    return out


def _theme_or_skip(rendered_themes: _RenderedThemes, key: str) -> _Theme:
    theme = rendered_themes.themes.get(key)
    if theme is None:
        pytest.skip(f"{key}: тема не разобралась или не найдена в архиве")
    return theme


# ---------------------------------------------------------------------------
# Сводные (структурные) проверки выборки
# ---------------------------------------------------------------------------


def test_sample_covers_reference_and_seeded_archive():
    assert set(REFERENCE_THEME_KEYS) <= set(ALL_THEME_KEYS)
    assert len(SAMPLE_THEME_KEYS) == SAMPLE_SIZE
    assert len(ALL_THEME_KEYS) >= SAMPLE_SIZE


def test_parse_failures_are_rare(rendered_themes: _RenderedThemes):
    """Темы, не прошедшие парсинг/гармонизацию, пропускаются, а не роняют гейт.

    Порог — щедрый (5%): цель не «ноль пропусков», а «пропуски не проглочены
    молча» (issue разбора #2896: библиотека честно отдаёт то, что есть).
    """
    total = len(ALL_THEME_KEYS)
    assert rendered_themes.skipped / total < 0.05, (
        rendered_themes.skipped, total, "подозрительно много пропусков"
    )


# ---------------------------------------------------------------------------
# Инварианты на каждой теме выборки
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("key", sorted(ALL_THEME_KEYS))
def test_bass_stays_in_key_or_chord_by_duration(rendered_themes, key):
    theme = _theme_or_skip(rendered_themes, key)
    fit = _bass_fit_ratio(theme.harmony)
    assert fit >= BASS_FIT_MIN, (key, fit)


@pytest.mark.parametrize("key", sorted(ALL_THEME_KEYS))
def test_lead_stays_in_working_range_by_duration(rendered_themes, key):
    theme = _theme_or_skip(rendered_themes, key)
    fit = _lead_fit_ratio(theme.harmony)
    assert fit >= LEAD_FIT_MIN, (key, fit)


@pytest.mark.parametrize("key", sorted(ALL_THEME_KEYS))
def test_pad_sounds_strictly_above_bass(rendered_themes, key):
    theme = _theme_or_skip(rendered_themes, key)
    margin = theme.checks.get("pad_over_bass")
    if margin is None:
        pytest.skip(f"{key}: партии баса или пэда нет")
    assert margin > 0, (key, margin)


@pytest.mark.parametrize("key", sorted(ALL_THEME_KEYS))
def test_pad_does_not_drift_far_above_theme_body(rendered_themes, key):
    """Baseline, не цель: см. ``PAD_UNDER_BODY_MAX`` — инвариант "пэд ниже
    корпуса темы" сегодня не выполняется строго на части архива."""
    theme = _theme_or_skip(rendered_themes, key)
    over = theme.checks.get("pad_under_theme_body")
    if over is None:
        pytest.skip(f"{key}: партии пэда или лида нет")
    assert over <= PAD_UNDER_BODY_MAX, (key, over)


@pytest.mark.parametrize("key", sorted(ALL_THEME_KEYS))
def test_part_durations_match_theme_length(rendered_themes, key):
    theme = _theme_or_skip(rendered_themes, key)
    harmony = theme.harmony
    lead_len = sum(dur for _n, dur in harmony.lead)
    for role, part in (("bass", harmony.bass), ("pad", harmony.pad), ("counter", harmony.counter)):
        part_len = sum(dur for _n, dur in part)
        assert part_len == pytest.approx(lead_len), (key, role, part_len, lead_len)


@pytest.mark.parametrize("key", sorted(ALL_THEME_KEYS))
def test_player_count_within_deploy_slots(rendered_themes, key):
    theme = _theme_or_skip(rendered_themes, key)
    slots = theme.checks.get("slots", [])
    assert len(slots) <= MAX_PLAYERS, (key, slots)


@pytest.mark.parametrize("key", sorted(ALL_THEME_KEYS))
def test_generated_code_passes_sanitizer(rendered_themes, key):
    theme = _theme_or_skip(rendered_themes, key)
    assert theme.sanitizer_ok, (key, theme.code[:200])


# ---------------------------------------------------------------------------
# Детерминизм — отдельная выборка (полное удвоение рендера всех тем не
# нужно для этого инварианта; достаточно репрезентативного подмножества).
# ---------------------------------------------------------------------------

_DETERMINISM_KEYS = tuple(sorted(REFERENCE_THEME_KEYS)[:10]) + tuple(SAMPLE_THEME_KEYS[:10])


@pytest.mark.parametrize("key", _DETERMINISM_KEYS)
def test_render_is_deterministic(library, key):
    entry = library.get(key)
    if entry is None:
        pytest.skip(f"{key}: тема не найдена в архиве")
    try:
        _harmony_a, _spec_a, code_a = _render_default(entry["rtttl"])
        _harmony_b, _spec_b, code_b = _render_default(entry["rtttl"])
    except ValueError:
        pytest.skip(f"{key}: тема не разобралась")
        return
    assert code_a == code_b, key
