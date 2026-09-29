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
     "bubblebo", "mortalko", "arkanoid"}
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
    rtttl = rec.get("rtttl") or ""
    assert rtttl, f"{melody_id!r}: пустой rtttl"
    # Issue #3181 (замечание координатора) — ``dizzy`` в архиве имел
    # РОВНО такую форму: непустая строка ``"Dizzy:d=32,o=5,b=300:"`` без
    # единой ноты после последнего двоеточия. ``assert rtttl`` выше эту
    # поломку не ловит (строка непустая) — проверяем ноты отдельно.
    notes = rtttl.rsplit(":", 1)[-1]
    assert notes.strip(), f"{melody_id!r}: rtttl без нот ({rtttl!r})"


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


def test_supermar_4_is_the_recognizable_overworld_riff(archive):
    """Issue #3181 (координатор, PR-1 #3182) — ``supermar`` в архиве это НЕ
    узнаваемый мотив (та же запись, что ``supermar_6``, Super Mario World);
    узнаваемое «ми-ми-ми-до-ми-соль» — ``supermar_4``. Проверяем по НАЧАЛУ
    самих нот RTTTL, а не по имени/title."""
    assert "supermar_4" in MELODY_POOLS["game"]
    assert "supermar" not in MELODY_POOLS["game"]
    notes_4 = archive["supermar_4"]["rtttl"].split(":")[-1]
    # Культовый рифф: E E (пауза) E (пауза) C E (пауза) G — ровно первые
    # ноты supermar_4 в архиве (см. докстринг dj_theme_melodies.py).
    assert notes_4.startswith("e,e,p,e,p,c,e,p,g")
    # supermar/supermar_6 — тот же (не культовый) мотив друг у друга,
    # ДРУГОЙ рисунок, чем supermar_4 — начинается на "a,8f." у обоих, а не
    # на культовую фразу. Честно фиксируем, ПОЧЕМУ они не в пуле, чтобы
    # регрессия (кто-то вернёт "supermar" вместо "supermar_4") сломала
    # этот тест, а не осталась незамеченной.
    notes_plain = archive["supermar"]["rtttl"].split(":")[-1]
    notes_6 = archive["supermar_6"]["rtttl"].split(":")[-1]
    assert notes_plain.startswith("a,8f.,16c,16d")
    assert notes_6.startswith("a,8f.,16c,16d")
    assert not notes_plain.startswith("e,e,p,e,p,c,e,p,g")


def test_game_pool_has_recognizable_nes_titles(archive):
    """Живой прогон 29.09 (issue #3181): «денди» — узнаваемые NES-темы сначала."""
    expected_titles = {
        "contra": "contra",
        "supermar_4": "mario",
        "tetris": "tetris",
        "zelda": "zelda",
        "pacman": "pacman",
        "mariobro": "mario",
    }
    for melody_id, needle in expected_titles.items():
        title = (archive[melody_id].get("title") or "").casefold()
        assert needle in title, f"{melody_id!r}: title {title!r} без {needle!r}"
