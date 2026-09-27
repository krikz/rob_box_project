#!/usr/bin/env python3
"""voice_facts_marathon_inventory.py — инвентаризация кастовых фактов в
``voice_memory.db`` (issue #2781).

Зачем этот скрипт
------------------
Изоляция БД дикторов (#2750, починена в #2763/#2770) накрывала только
``speakers.db``. Долгосрочная память (таблица ``voice_facts``, пишется
MCP-тулом ``memory_save``) до этой карточки писалась прямо в боевую
``/data/voice_memory.db`` без изоляции: акт «Знакомство» ночного
марафона называет LLM факты о синтезированном касте ("Саша не ест лук",
"Борис болеет за Спартак" — голоса ``anton``/``ermil``). Замер на Vision
Pi 22.09.2026 нашёл 59 из 116 фактов боевой базы, упоминающих этот каст,
вперемешку с фактами живых людей мастерской.

Разовая чистка НАКОПЛЕННЫХ до фикса кастовых фактов — решение владельца
робота, не автоматики (см. issue #2781, раздел "Что сделать"). Этот
скрипт делает ровно то, что нужно для этого решения, и НИЧЕГО сверх:
показывает, что было бы затронуто (``list``, единственная команда без
``--apply``), и по явному запросу — либо помечает подозрительные факты
(``tag``, обратимо: правит только ``category``), либо удаляет их
(``delete``, необратимо). Ни одна мутирующая команда не трогает диск без
``--apply`` И ``--yes`` одновременно (защита от одной опечатки во флаге),
и обе снимают ``.bak-<UTC>Z`` копию файла ПЕРЕД записью — тот же приём,
которым раньше вручную чистили ``speakers.db`` (issue #2750: ``cp
/data/speakers.db /data/speakers.db.bak-<UTC>Z`` перед ``DELETE FROM``).

Эвристика отбора — по умолчанию подстрока имени каста (``--keyword
Саша --keyword Борис``, регистронезависимо) в тексте факта. Значения по
умолчанию взяты из issue #2781 (единственный каст, известный на
22.09.2026); при появлении других синтетических имён в будущих
марафонах — передать свои ``--keyword``. ``list`` печатает КАЖДЫЙ
совпавший факт целиком (id, category, speaker_id, created_at, текст) —
финальное решение "это точно каст или тёзка живого человека" делает
человек, читающий вывод, эвристика только сужает выборку.

Как это запускают на Vision Pi
-------------------------------
::

    # 1. Посмотреть, что было бы затронуто — БЕЗОПАСНО, ничего не пишет:
    docker exec voice-assistant python3 /tmp/voice_facts_marathon_inventory.py \\
        list --db /data/voice_memory.db

    # 2. Пометить (обратимо — category='suspected_marathon_cast'),
    #    после того как оператор проверил вывод list:
    docker exec voice-assistant python3 /tmp/voice_facts_marathon_inventory.py \\
        tag --db /data/voice_memory.db --apply --yes

    # 3. Удалить НАВСЕГДА (только если оператор уверен):
    docker exec voice-assistant python3 /tmp/voice_facts_marathon_inventory.py \\
        delete --db /data/voice_memory.db --apply --yes

Файл не копируется в образ ``voice-assistant`` автоматически (та же
оговорка, что у ``face_store_admin.py`` — ``scripts/`` не входит в
Docker-контекст сборки), поэтому ``docker cp
voice_facts_marathon_inventory.py voice-assistant:/tmp/`` — отдельный
шаг перед запуском.

Честность (AGENTS.md: «честный FAIL лучше красивого PASS»)
-------------------------------------------------------------
Этот скрипт писался и тестировался ТОЛЬКО на синтетическом SQLite-файле
во временном каталоге (``tmp_path``) в рамках issue #2781. На боевой
``/data/voice_memory.db`` Vision Pi он НЕ запускался — ни ``list``, ни
тем более мутирующие команды. Инвентаризация и чистка 59 кастовых
фактов остаются решением владельца робота (см. docstring выше и текст
issue #2781); этот PR только даёт инструмент, не исполняет его.
"""

from __future__ import annotations

import argparse
import shutil
import sqlite3
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Sequence

DEFAULT_DB_PATH = "/data/voice_memory.db"
# Issue #2781 — известный на 22.09.2026 каст ночного марафона (голоса
# anton/ermil). Расширяется через повторяемый --keyword на CLI, не
# правкой этого списка под каждый новый прогон.
DEFAULT_KEYWORDS: List[str] = ["Саша", "Борис"]
TAG_CATEGORY = "suspected_marathon_cast"


@dataclass(frozen=True)
class Fact:
    id: int
    fact: str
    category: str
    speaker_id: Optional[str]
    created_at: float
    updated_at: float


def _connect(db_path: str) -> sqlite3.Connection:
    path = Path(db_path)
    if not path.exists():
        raise FileNotFoundError(
            f"{db_path} не найден — проверь путь (issue #2781: боевой файл "
            "по умолчанию /data/voice_memory.db ВНУТРИ контейнера "
            "voice-assistant, не на хосте Vision Pi)"
        )
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    return conn


def find_matches(conn: sqlite3.Connection, keywords: Sequence[str]) -> List[Fact]:
    """Факты, чей текст содержит хотя бы одно из ``keywords`` (без учёта
    регистра). Пустой ``keywords`` -> пустой результат (явно, не "всё").

    Фильтрация — в Python, не в SQL: SQLite's ``LIKE ... COLLATE NOCASE``
    складывает регистр только для ASCII (встроенная реализация, без
    ICU-расширения) — "Саша"/"саша" им НЕ считаются равными, а каст
    ночного марафона называют именно кириллицей. Таблица ``voice_facts``
    на живом роботе — сотни строк, не миллионы, полная выборка с фильтром
    на стороне Python здесь дешевле, чем тащить ICU только ради LIKE.
    """
    if not keywords:
        return []
    lowered_keywords = [kw.lower() for kw in keywords]
    rows = conn.execute(
        "SELECT id, fact, category, speaker_id, created_at, updated_at "
        "FROM voice_facts ORDER BY id"
    ).fetchall()
    return [
        Fact(
            id=r["id"],
            fact=r["fact"],
            category=r["category"],
            speaker_id=r["speaker_id"],
            created_at=r["created_at"],
            updated_at=r["updated_at"],
        )
        for r in rows
        if any(kw in r["fact"].lower() for kw in lowered_keywords)
    ]


def total_fact_count(conn: sqlite3.Connection) -> int:
    return conn.execute("SELECT COUNT(*) FROM voice_facts").fetchone()[0]


def _print_report(matches: List[Fact], total: int, keywords: Sequence[str]) -> None:
    print(f"Ключевые слова: {', '.join(keywords)}")
    print(f"Всего фактов в voice_facts: {total}")
    print(f"Совпало по ключевым словам: {len(matches)}")
    print("-" * 78)
    for f in matches:
        when = datetime.fromtimestamp(f.created_at, tz=timezone.utc).strftime(
            "%Y-%m-%d %H:%M:%S UTC"
        )
        speaker = f.speaker_id or "NULL"
        print(f"id={f.id:<6} speaker_id={speaker:<10} category={f.category:<12} {when}")
        print(f"    {f.fact}")


def _backup_db_file(db_path: str) -> str:
    """Снять ``.bak-<UTC>Z`` копию ПЕРЕД любой мутацией — issue #2750/#2781:
    ручная чистка без бэкапа уже один раз стёрла профиль живого человека
    в другой БД этого репозитория. Копируются и ``-wal``/``-shm``, если
    существуют (WAL-режим — см. VoiceMemory.__init__)."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup_path = f"{db_path}.bak-{stamp}"
    shutil.copy2(db_path, backup_path)
    for suffix in ("-wal", "-shm"):
        side = f"{db_path}{suffix}"
        if Path(side).exists():
            shutil.copy2(side, f"{backup_path}{suffix}")
    return backup_path


def cmd_list(args: argparse.Namespace) -> int:
    conn = _connect(args.db)
    try:
        matches = find_matches(conn, args.keyword)
        _print_report(matches, total_fact_count(conn), args.keyword)
    finally:
        conn.close()
    return 0


def cmd_tag(args: argparse.Namespace) -> int:
    """Обратимая пометка: ``category`` совпавших фактов -> TAG_CATEGORY.
    Текст факта и ``speaker_id`` не трогаются — решение "удалить
    насовсем" остаётся отдельным шагом (``delete``)."""
    conn = _connect(args.db)
    try:
        matches = find_matches(conn, args.keyword)
        _print_report(matches, total_fact_count(conn), args.keyword)
        if not matches:
            print("Нечего помечать — совпадений нет.")
            return 0
        if not (args.apply and args.yes):
            print(
                f"\nDRY-RUN: {len(matches)} факт(ов) были бы помечены "
                f"category='{TAG_CATEGORY}'. Ничего не записано — передай "
                "--apply --yes, чтобы применить."
            )
            return 0
        backup_path = _backup_db_file(args.db)
        print(f"Бэкап снят: {backup_path}")
        with conn:
            conn.executemany(
                "UPDATE voice_facts SET category = ?, updated_at = strftime('%s','now') "
                "WHERE id = ?",
                [(TAG_CATEGORY, f.id) for f in matches],
            )
        print(f"Помечено {len(matches)} факт(ов) как '{TAG_CATEGORY}'.")
    finally:
        conn.close()
    return 0


def cmd_delete(args: argparse.Namespace) -> int:
    """НЕОБРАТИМОЕ удаление совпавших фактов. Требует --apply --yes."""
    conn = _connect(args.db)
    try:
        matches = find_matches(conn, args.keyword)
        _print_report(matches, total_fact_count(conn), args.keyword)
        if not matches:
            print("Нечего удалять — совпадений нет.")
            return 0
        if not (args.apply and args.yes):
            print(
                f"\nDRY-RUN: {len(matches)} факт(ов) были бы удалены "
                "НАВСЕГДА. Ничего не удалено — передай --apply --yes, "
                "чтобы применить (бэкап будет снят автоматически перед "
                "удалением)."
            )
            return 0
        backup_path = _backup_db_file(args.db)
        print(f"Бэкап снят: {backup_path}")
        with conn:
            conn.executemany(
                "DELETE FROM voice_facts WHERE id = ?",
                [(f.id,) for f in matches],
            )
        print(f"Удалено {len(matches)} факт(ов).")
    finally:
        conn.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Инвентаризация/чистка кастовых фактов ночного марафона в "
            "voice_facts (issue #2781). По умолчанию ничего не пишет — "
            "'list' безопасна всегда, 'tag'/'delete' без --apply --yes "
            "тоже только печатают, что было бы сделано."
        )
    )
    parser.add_argument(
        "--db",
        default=DEFAULT_DB_PATH,
        help=f"Путь к voice_memory.db (по умолчанию {DEFAULT_DB_PATH} — "
        "путь ВНУТРИ контейнера voice-assistant).",
    )
    parser.add_argument(
        "--keyword",
        action="append",
        default=None,
        help="Подстрока для поиска в тексте факта (регистронезависимо, "
        f"повторяемый флаг). По умолчанию: {DEFAULT_KEYWORDS!r}.",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list", help="Показать совпадающие факты (всегда безопасно).")
    p_list.set_defaults(func=cmd_list)

    p_tag = sub.add_parser(
        "tag",
        help=f"Пометить совпадающие факты category='{TAG_CATEGORY}' (обратимо).",
    )
    p_tag.add_argument("--apply", action="store_true", help="Применить изменения (иначе dry-run).")
    p_tag.add_argument("--yes", action="store_true", help="Подтвердить применение (нужен вместе с --apply).")
    p_tag.set_defaults(func=cmd_tag)

    p_delete = sub.add_parser(
        "delete", help="Удалить совпадающие факты НАВСЕГДА."
    )
    p_delete.add_argument("--apply", action="store_true", help="Применить изменения (иначе dry-run).")
    p_delete.add_argument("--yes", action="store_true", help="Подтвердить применение (нужен вместе с --apply).")
    p_delete.set_defaults(func=cmd_delete)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.keyword is None:
        args.keyword = list(DEFAULT_KEYWORDS)
    try:
        return args.func(args)
    except FileNotFoundError as exc:
        print(f"ОШИБКА: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
