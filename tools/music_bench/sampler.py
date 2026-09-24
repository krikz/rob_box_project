"""Выборка тем из RTTTL-архива -> ``CompositionSpec`` x 3 комбо (issue #2977).

Архив — тот же ``data/rtttl_melodies.jsonl.gz`` (10460 мелодий), что
использует ``compose_music(name=...)`` на роботе (``core.rtttl_library.
RtttlLibrary``). Каждая тема прогоняется через ТОТ ЖЕ конвейер, что и на
живом вызове (``rtttl_compose.melody_to_compose_params`` ->
``arranger.spec_from_flat``) — стенд не выдумывает свою аранжировку.

Три комбо на тему (acceptance issue #2977: «2-3 комбо»):

* ``default`` — ручки не тронуты (``ArrangeOptions()`` — всё ``auto``).
* ``counter_on`` — второй голос принудительно включён
  (``ArrangeOptions(counter="on")``) — RC1/#2963 подозревают именно
  ОДНОВРЕМЕННОЕ звучание нескольких ролей в сумме.
* ``max_stack`` — второй голос И удвоение лида октавой одновременно
  включены (``counter="on", theme_octaves="on"``) — худший случай состава
  слоёв, прямой стенд-тест гипотезы issue #2963 «сумма слоёв > 1 перед
  tanh». (Ручка ``levels`` для этого не годится: её потолок — 1.0,
  ADR-0132 issue #2963, то есть множитель 1.0 — это уже дефолт, а не
  «громче».)
"""

from __future__ import annotations

import random
import sqlite3
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, List, Optional

from . import _repo_paths  # noqa: F401

from rob_box_mcp_tools.core.arranger import (  # noqa: E402
    ArrangeOptions,
    CompositionSpec,
    spec_from_flat,
)
from rob_box_mcp_tools.core.rtttl_compose import (  # noqa: E402
    melody_to_compose_params,
    rtttl_to_melody,
)
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary  # noqa: E402

#: Тембры темы — те же имена, что уже проверены golden-тестом аранжировщика
#: (``test_arranger_golden.py``), не подобраны под конкретный трек.
LEAD_SYNTH = "blip"
BASS_SYNTH = "moogbass"
PAD_SYNTH = "warmpad"
COUNTER_SYNTH = "pluck"


@dataclass(frozen=True)
class ArchiveSample:
    name: str
    title: str
    rtttl: str


@dataclass(frozen=True)
class BenchCombo:
    label: str
    options: ArrangeOptions


COMBOS: List[BenchCombo] = [
    BenchCombo("default", ArrangeOptions()),
    BenchCombo("counter_on", ArrangeOptions(counter="on")),
    BenchCombo("max_stack", ArrangeOptions(counter="on", theme_octaves="on")),
]


def _migrated_db_path() -> str:
    """Создать одноразовую SQLite-копию архива (issue #2977: не трогаем боевую
    ``/data/voice_memory.db`` — см. память ``robot-data-is-shared-with-live-people``)."""
    tmp = Path(tempfile.gettempdir()) / "music_bench_rtttl.db"
    return str(tmp)


def open_archive() -> RtttlLibrary:
    return RtttlLibrary(db_path=_migrated_db_path())


def sample_archive(count: int, seed: int = 2977) -> List[ArchiveSample]:
    """``count`` случайных тем архива (детерминированно по ``seed``)."""
    open_archive()  # мигрирует архив в SQLite-файл при первом вызове (idempotent)
    conn = sqlite3.connect(_migrated_db_path())
    try:
        rows = conn.execute(
            "SELECT name, title, rtttl FROM rtttl_melodies WHERE rtttl != ''"
        ).fetchall()
    finally:
        conn.close()
    rng = random.Random(seed)
    rng.shuffle(rows)
    picked = rows[:count]
    return [ArchiveSample(name=r[0], title=r[1] or r[0], rtttl=r[2]) for r in picked]


def build_spec(sample: ArchiveSample, combo: BenchCombo) -> Optional[CompositionSpec]:
    """Тема + комбо -> ``CompositionSpec`` (``None`` — тема не разобралась).

    Ошибки конвейера (``ValueError``/``ArrangementError``) не глушатся
    молча наружу отчёта: :mod:`.run_bench` считает их и печатает отдельно
    (AGENTS.md: честный FAIL лучше красивого PASS) — здесь только
    ``None``, чтобы сборщик выборки мог продолжить со следующей темой.
    """
    try:
        melody = rtttl_to_melody(sample.rtttl)
        params = melody_to_compose_params(melody)
        spec = spec_from_flat(
            harmony=params["harmony"],
            bpm=params["bpm"],
            root=params["root"],
            scale=params["scale"],
            form="arc",
            lead_synth=LEAD_SYNTH,
            bass_synth=BASS_SYNTH,
            pad_synth=PAD_SYNTH,
            counter_synth=COUNTER_SYNTH,
            repeat=False,
            options=combo.options,
        )
        return spec
    except Exception as exc:  # noqa: BLE001 — отчёт печатает exc, не глушит
        raise RuntimeError(f"{sample.name}/{combo.label}: {exc}") from exc


def iter_bench_specs(
    count: int, seed: int = 2977
) -> Iterator[tuple]:
    """(sample, combo, spec-or-exception) для каждой темы x каждого комбо."""
    samples = sample_archive(count, seed=seed)
    for sample in samples:
        for combo in COMBOS:
            try:
                spec = build_spec(sample, combo)
                yield sample, combo, spec, None
            except RuntimeError as exc:
                yield sample, combo, None, exc
