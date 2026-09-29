"""dj_theme_melodies.py — тема сета → тег RTTTL-архива → пул мелодий (issue #3181).

Живой лог 29.09: «Ты диджей 8 битный монстр и у нас сегодня клубная
вечеринка любителей денди» — тема не влияла на выбор мелодий, каждый
DJ-переход играл ``compose_music(style="club", ...)`` без ``name=``, и сет
звучал однотипно, хотя в RTTTL-архиве (``rob_box_mcp_tools/data/
rtttl_melodies.jsonl.gz``) лежат узнаваемые игровые темы (тег ``game``,
102 мелодии).

``rob_box_voice`` не зависит от ``rob_box_mcp_tools`` (нет
``<exec_depend>`` в ``package.xml`` — см. ``dialogue_node._build_tool_
provider`` для единственного места, где такой импорт вообще делается, и
там это управляемый ROS-lazy-import с явным probe/RuntimeError на старте
ноды, а не что-то, что годится внутри чистого модуля состояния DJ). Читать
gzip-архив (10461 запись) отсюда, чтобы отсортировать пул по
``core.rtttl_library._melody_quality``, означало бы протащить I/O и
кросс-пакетную зависимость в модуль, который :mod:`.dj_mode` документирует
как «без ROS и без I/O» и который юнит-тесты гоняют без ФС.

Честный путь вместо этого: курируемый список id ЗДЕСЬ (без чтения архива в
рантайме), и отдельный тест в ``rob_box_mcp_tools``
(``test/test_issue_3181_dj_theme_melody_pool.py``) проверяет, что каждый
id из :data:`MELODY_POOLS` РЕАЛЬНО есть в архиве. Честная оговорка:
несколько id из требования issue #3181 (doubledr, teenagem, donkeyko,
circus, commando, bubblebo, dizzy, arkanoid) в архиве размечены тегами
``tv`` / ``picaxe:mixed3``, а НЕ ``game`` — архивная разметка сама по себе
не всегда «игровая» для игровых тем (например, тег ``game`` есть у Sonic /
Final Fantasy, но не у Bubble Bobble). Пул для тега ``game`` собран по
УЗНАВАЕМОСТИ темы (что и просит issue: «в начале пула должны быть
узнаваемые NES-темы»), а не строго по архивному тегу — тест в
``rob_box_mcp_tools`` проверяет существование id, а тег — только там, где
он у записи архива реально ``game``.

Правило «слова темы → тег» — подстроки/токены нормализованной (casefold)
строки темы, БЕЗ регулярных выражений (мораторий #3132 действует и на
``dj_mode``/эту его соседку). Первое совпадение по порядку правил
побеждает.

Уточнение координатора (PR-1, #3182): в архиве имя ``supermar`` — НЕ
узнаваемая overworld-тема, а тот же (не культовый) рисунок, что и
``supermar_6`` (Super Mario World) — оба начинаются ``a,8f.,16c,16d...``,
не с культового «ми-ми-ми-до-ми-соль».
Узнаваемый мотив — ``supermar_4`` (и ``supermar_2``): их RTTTL реально
начинается ``e,e,...,c,e,...,g...`` (E-E-_-E-_-C-E-_-G — тот самый
культовый рифф). Проверено по НАЧАЛУ RTTTL каждого id пула ``game``, а не
по имени; заодно нашлась пустая мелодия ``dizzy`` (``Dizzy:d=32,o=5,
b=300:`` — ноты после двоеточия отсутствуют, архивный брак) — заменена на
``mortalko`` (валидный RTTTL, тоже без тега ``game`` в архиве — см.
:mod:`rob_box_mcp_tools`-тест).
"""

from __future__ import annotations

from typing import Dict, Sequence, Tuple

#: Порядок важен — первое совпадение побеждает. Ключевые слова — уже
#: casefold()-нутые подстроки; проверяются как ``keyword in normalized``.
_THEME_TAG_RULES: Tuple[Tuple[str, Tuple[str, ...]], ...] = (
    (
        "game",
        (
            "денди", "dendy", "8 бит", "8-бит", "8бит", "8 битный",
            "8-битный", "nes", "нинтендо", "nintendo", "сега", "sega",
            "приставк", "игр", "game",
        ),
    ),
    (
        "movie",
        ("кино", "фильм", "movie"),
    ),
    (
        "christmas",
        ("новый год", "новогодн", "рождеств", "christmas"),
    ),
    (
        "classical",
        ("классик", "классич", "classical"),
    ),
)

#: Issue #3181 — курируемые пулы id по тегу архива, отсортированные так,
#: чтобы самые узнаваемые темы звучали первыми. Id проверены отдельным
#: тестом в rob_box_mcp_tools на существование в
#: ``rtttl_melodies.jsonl.gz`` (см. модульный docstring выше).
MELODY_POOLS: Dict[str, Tuple[str, ...]] = {
    # Денди/NES-сет (issue #3181 живой прогон 29.09) — узнаваемые темы
    # сначала: платформеры/аркады, потом менее известные.
    "game": (
        "contra", "supermar_4", "tetris", "zelda", "doubledr", "teenagem",
        "donkeyko", "pacman", "mariobro", "circus", "commando",
        "bubblebo", "mortalko", "arkanoid",
    ),
    "movie": (
        "batman", "titanic", "superman", "terminat", "ghostbus",
        "harrypot",
    ),
    "christmas": (
        "jinglebe", "decktheh", "frostyth", "alliwant", "feliznav",
        "amazingg",
    ),
    "classical": (
        "minuetin", "toccata", "bourree", "brandenb", "aironthe",
        "fugueind",
    ),
}


def theme_to_tag(theme: str) -> str:
    """Тег архива для темы сета или ``""`` — правило не сработало.

    Подстрочное сопоставление нормализованной (``casefold()``) строки —
    без регексов (мораторий #3132). Первое совпавшее правило побеждает.
    """
    if not theme:
        return ""
    normalized = theme.casefold()
    for tag, keywords in _THEME_TAG_RULES:
        if any(keyword in normalized for keyword in keywords):
            return tag
    return ""


def melody_pool_for_theme(theme: str) -> Tuple[str, str]:
    """``(tag, pool)`` для темы сета — ``("", ())`` без совпадения/пула."""
    tag = theme_to_tag(theme)
    pool = MELODY_POOLS.get(tag, ())
    return (tag, pool) if pool else ("", ())


def pick_melody(pool: Sequence[str], track_no: int, played: Sequence[str]) -> str:
    """Следующая мелодия пула для трека ``track_no``, ещё не сыгранная.

    Детерминированно: индекс — ``(track_no - 1) % len(pool)``, дальше
    обход пула по кругу до первого id, которого нет среди ``played``
    (сравнение без регистра — тот же контракт, что у
    ``DJModeController._played_line``). Пул исчерпан — берём id по
    базовому индексу повторно (лучше повтор, чем пустая строка).

    Честная оговорка: ``played`` — заголовки треков из
    ``/voice/music/form`` (``DJState.played_names``), не сами id архива;
    до мержа PR-1 (#3181 план) ``style="club"`` + ``name=`` уходит в
    classic (issue #3113 п.2), и заголовок формы там — архивный ``title``
    (например «Contra»), который у части id совпадает с id без регистра
    (``contra``), а у части — нет (``mariobro`` → «Mario Bros»). Исключение
    повтора поэтому best-effort, не гарантия.
    """
    if not pool:
        return ""
    n = len(pool)
    start = (track_no - 1) % n
    played_cf = {p.casefold() for p in played}
    for offset in range(n):
        candidate = pool[(start + offset) % n]
        if candidate.casefold() not in played_cf:
            return candidate
    return pool[start]


__all__ = [
    "MELODY_POOLS",
    "melody_pool_for_theme",
    "pick_melody",
    "theme_to_tag",
]
