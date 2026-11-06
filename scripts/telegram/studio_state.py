"""Состояние студии: единое место для всех, кто его пишет.

Зачем отдельный модуль. Раньше логику покрытия фактов дублировали studio_facts.py
и channel_studio.py, а load_state был написан в трёх местах. Из-за этого:
  * факт, ушедший в идею, не попадал в pending — и предлагался заново каждую сессию
    (повторная обработка одного и того же события);
  * сессия и опрос писали state.json параллельно — одно затирало другое.

Здесь один формат состояния, одна атомарная запись и одна блокировка на файл:
читатель-писатель не может потерять чужое обновление.
"""

from __future__ import annotations

import fcntl
import json
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterator, Optional

# Сколько раз факт можно предлагать, прежде чем он станет отложенным.
MAX_ATTEMPTS = 3


def now_iso() -> str:
    """Текущее время в ISO — метки в состоянии сравниваются как строки."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def default_state() -> dict:
    """Состояние по умолчанию: смещение getUpdates, список идей, словарь постов."""
    return {"offset": 0, "ideas": [], "posts": {}, "messages": []}


def load_state(path: Path) -> dict:
    """Прочитать состояние.

    Битый файл — не «offset: 0»: это значило бы, что студия начнёт заново
    обрабатывать всю ленту апдейтов и пришлёт Шифу старые идеи по второму кругу.
    Поэтому при битом файле берём последнюю резервную копию, и только если её нет —
    честно отвечаем «состояния нет» (пустой offset), но НЕ притворяемся, что
    всё обработано.
    """
    path = Path(path)
    try:
        state = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        backup = path.with_suffix(path.suffix + ".bak")
        try:
            state = json.loads(backup.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return default_state()
        state["_restored_from_backup"] = True
        return state
    if not isinstance(state, dict):
        return default_state()
    state.setdefault("offset", 0)
    return state


def save_state(path: Path, state: dict) -> None:
    """Сохранить состояние атомарно: временный файл + переименование.

    Перед записью копия предыдущего состояния: если файл испортится (сбой диска,
    правка руками), студия поднимется с последнего known-good, а не начнёт
    обрабатывать всю лену заново.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        try:
            backup = path.with_suffix(path.suffix + ".bak")
            backup.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
        except OSError:
            pass
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)


@contextmanager
def state_transaction(path: Path) -> Iterator[dict]:
    """Прочитать → изменить → записать под блокировкой файла.

    Блокировка держится на всём цикле чтения-изменения-записи: иначе два процесса
    (опрос кнопок и сессия анализа) читают одно состояние и затирают друг друга.

    Состояние сохраняется ДАЖЕ если обработка упала. Иначе апдейт, который уже
    забрали у Telegram, терялся бы навсегда: смещение не сдвинулось, а апдейт
    Telegram больше не отдаст. Поэтому смещение двигаем сразу, а не в конце цикла.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = path.with_suffix(path.suffix + ".lock")
    with open(lock_path, "a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        state = load_state(path)
        try:
            yield state
        finally:
            # Смещение обязано уехать даже после ошибки: апдейт уже забран у Telegram
            # и больше не будет отдан. Поэтому состояние пишется в finally.
            save_state(path, state)
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


# --- покрытие фактов ---------------------------------------------------------


def covered_keys(state: dict) -> set:
    """Ключи фактов, которые уже показаны (пост опубликован) или отклонены Шифу."""
    return set(state.get("covered", {})) | set(state.get("rejected", {}))


def pending_facts(state: dict) -> list:
    """Факты, которые уже предлагали, но они не попали в пост.

    Их нельзя просто выбросить: иначе событие потеряется навсегда. Но и предлагать
    бесконечно — тоже нельзя, поэтому у факта есть счётчик попыток.
    """
    return list(state.get("pending", []))


def save_pending(state: dict, facts: list) -> None:
    """Запомнить факты, которые ушли в идеи: их событие ещё не показано."""
    pending = {f.get("key"): f for f in state.get("pending", [])}
    for fact in facts:
        key = fact.get("key")
        if not key:
            continue
        old = pending.get(key) or {}
        fact = dict(fact)
        fact["attempts"] = int(old.get("attempts", 0)) + 1
        pending[key] = fact
    state["pending"] = list(pending.values())


def drop_pending(state: dict, keys: set) -> None:
    """Убрать из ожидающих факты, которые закрыты (опубликованы или отклонены)."""
    state["pending"] = [f for f in state.get("pending", []) if f.get("key") not in keys]


def mark_covered(state: dict, keys: set, post_id: str, when: Optional[str] = None) -> None:
    """Отметить факты как показанные и убрать их из ожидающих."""
    covered = state.setdefault("covered", {})
    for key in keys:
        covered[key] = {"post": post_id, "at": when or now_iso()}
    drop_pending(state, keys)


def mark_rejected(state: dict, keys: set, when: Optional[str] = None) -> None:
    """Отметить факты как отклонённые Шифу: больше не предлагать."""
    rejected = state.setdefault("rejected", {})
    for key in keys:
        rejected[key] = {"at": when or now_iso()}
    drop_pending(state, keys)


def defer_exhausted(state: dict, keys: set) -> None:
    """Отложить факты, которые предлагали слишком много раз."""
    state["deferred"] = sorted(set(state.get("deferred", [])) | set(keys))
