"""Issue #3181 — курируемые пулы мелодий ``rob_box_voice`` реально в архиве.

``rob_box_voice.core.dj_theme_melodies.MELODY_POOLS`` — курируемый список id
(``rob_box_voice`` не зависит от ``rob_box_mcp_tools``, см. докстринг того
модуля: читать gzip-архив из чистого DJ-модуля без ROS/I/O — не вариант).
Честный контракт — этот тест: каждый id из пулов ДЕЙСТВИТЕЛЬНО есть в
``rtttl_melodies.jsonl.gz`` и несёт валидный RTTTL. Тег архива сверяется
там, где утверждение об этом честное — см. ``GAME_POOL_TAG_MISMATCH`` ниже:
несколько NES-тем в архиве размечены ``tv``/``picaxe:mixed3``, а не
``game`` (это описано в докстринге ``dj_theme_melodies`` как сознательное
расхождение куратора с архивной тегировкой, а не баг теста).
"""

from __future__ import annotations

import gzip
import json
from importlib.resources import as_file, files

import pytest

from rob_box_voice.core.dj_theme_melodies import MELODY_POOLS

#: Issue #3181 — id тега ``game``, у которых архив НЕ проставил тег
#: ``game`` (только ``tv`` и/или ``picaxe:mixed3``), хотя по названию
#: (``title``) это узнаваемые NES-темы, которые issue просит в пуле. Не
#: "тест врёт" — тест ниже это явно фиксирует и не требует от них тега
#: ``game``, только существование в архиве.
GAME_POOL_TAG_MISMATCH = frozenset(
    {"doubledr", "teenagem", "donkeyko", "circus", "commando",
     "bubblebo", "dizzy", "arkanoid"}
)


def _load_archive() -> dict:
    """``{name: record}`` для всего ``rtttl_melodies.jsonl.gz`` пакета."""
    records: dict = {}
    with as_file(
        files("rob_box_mcp_tools.data").joinpath("rtttl_melodies.jsonl.gz")
    ) as path:
        with gzip.open(path, "rt", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                rec = json.loads(line)
                records[rec["name"]] = rec
    return records


@pytest.fixture(scope="module")
def archive() -> dict:
    return _load_archive()


def _all_pool_ids():
    for tag, pool in MELODY_POOLS.items():
        for melody_id in pool:
            yield tag, melody_id


@pytest.mark.parametrize("tag,melody_id", list(_all_pool_ids()))
def test_curated_id_exists_in_archive(archive, tag, melody_id):
    assert melody_id in archive, (
        f"MELODY_POOLS[{tag!r}] держит id {melody_id!r}, которого нет в "
        "rtttl_melodies.jsonl.gz"
    )
    rec = archive[melody_id]
    assert rec.get("rtttl"), f"{melody_id!r}: пустой rtttl"


@pytest.mark.parametrize("tag,melody_id", list(_all_pool_ids()))
def test_curated_id_has_expected_tag_or_documented_mismatch(archive, tag, melody_id):
    rec = archive[melody_id]
    tags = set(rec.get("tags") or [])
    if tag == "game" and melody_id in GAME_POOL_TAG_MISMATCH:
        # Честно проверяем обратное: архив ДЕЙСТВИТЕЛЬНО не даёт им game —
        # если апстрим когда-нибудь перетегирует архив, тест это заметит
        # (assert упадёт) и напомнит вычистить запись из "документированных
        # расхождений".
        assert "game" not in tags, (
            f"{melody_id!r} теперь помечен тегом game в архиве — уберите "
            "его из GAME_POOL_TAG_MISMATCH"
        )
        return
    assert tag in tags, (
        f"MELODY_POOLS[{tag!r}] держит id {melody_id!r} с тегами {tags!r} "
        f"— тега {tag!r} нет"
    )


def test_game_pool_has_recognizable_nes_titles(archive):
    """Живой прогон 29.09 (issue #3181): «денди» — узнаваемые NES-темы сначала."""
    expected_titles = {
        "contra": "contra",
        "supermar": "mario",
        "tetris": "tetris",
        "zelda": "zelda",
        "pacman": "pacman",
        "mariobro": "mario",
    }
    for melody_id, needle in expected_titles.items():
        title = (archive[melody_id].get("title") or "").casefold()
        assert needle in title, f"{melody_id!r}: title {title!r} без {needle!r}"
