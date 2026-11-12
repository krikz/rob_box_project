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

# Сколько ждать ответа на черновик, прежде чем он протухнет (ADR-0170).
#
# Разбор 11.11: черновик p20261109-810 провисел в proposed с 09.11, товарищ Шифу
# его проигнорировал — и пайплайн встал: факт pr:4555 потерялся между
# «предложили» и «взяли», новый черновик по нему больше не собирался.
# Протухание возвращает факт в pending, и следующая сессия берётся за него снова.
DRAFT_TTL_HOURS = 48

# Статусы черновика, которые ещё можно обсуждать.
LIVE_DRAFT_STATUSES = ("proposed",)


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


def keep_collected(state: dict, facts: list, closed: Optional[set] = None) -> None:
    """Оставить собранные, но НЕ показанные факты в ожидании.

    Разбор 11.11: сессия собирает факты, в черновик попадает один, а остальные
    пропадали — метка `last_session` сдвигалась, и повторный сбор их уже не
    возвращал (фильтр `merged:>=since`). Материал утекал.

    Здесь факты кладутся в `pending` БЕЗ роста `attempts`: показ не было —
    факт ещё никто не видел. `attempts` растёт в момент показа (save_pending
    при регистрации черновика).
    """
    done = set(closed or set()) | covered_keys(state)
    pending = {f.get("key"): f for f in state.get("pending", [])}
    for fact in facts:
        key = fact.get("key")
        if not key or key in done:
            continue
        existing = pending.get(key)
        if existing is None:
            entry = dict(fact)
            entry["attempts"] = int(fact.get("attempts", 0))
            pending[key] = entry
        else:
            # Уже в ожидании: обновляем содержимое, но не трогаем счётчик показов.
            existing.update({k: v for k, v in fact.items() if v not in (None, "")})
    state["pending"] = list(pending.values())


def save_pending(state: dict, facts: list, post_id: str = "") -> None:
    """Запомнить факты, которые ушли в идеи: их событие ещё не показано.

    Счётчик `attempts` растёт на КАЖДЫЙ показ. Раньше по нему же факт уходил
    в `deferred` после трёх показов, то есть игнор товарища Шифа приравнивался
    к отказу и событие терялось (разбор 11.11, факт pr:4555).

    Теперь отказ — только явный: товарищ Шифу нажал «Мимо» (mark_rejected).
    Показы считаем, но в `deferred` по ним не уводим — протухший черновик
    возвращает факт в pending (expire_stale_drafts).
    """
    pending = {f.get("key"): f for f in state.get("pending", [])}
    for fact in facts:
        key = fact.get("key")
        if not key:
            continue
        old = pending.get(key) or {}
        fact = dict(fact)
        # Показ считаем один раз на пост: register_post вызывается дважды
        # (до отправки и после), и без post_id один показ считался бы двумя.
        same_post = bool(post_id) and old.get("post_id") == post_id
        fact["attempts"] = int(old.get("attempts", 0)) + (0 if same_post else 1)
        if post_id:
            fact["post_id"] = post_id
        pending[key] = fact
    state["pending"] = list(pending.values())


def expire_stale_drafts(state: dict, ttl_hours: int = DRAFT_TTL_HOURS, when: Optional[str] = None) -> list:
    """Протухшие черновики → expired, их факты — обратно в pending.

    Товарищ Шифу проигнорировал черновик — это не отказ. Отказ он нажимает
    кнопкой. Поэтому молчание только снимает черновик с обсуждения, а событие
    возвращается в очередь на материал: следующая сессия соберёт по нему новый.

    Возвращает список протухших id — вызывающий пишет их в состояние.
    """
    moment = when or now_iso()
    posts = state.get("posts") or {}
    expired = []
    for post_id, record in posts.items():
        if not isinstance(record, dict):
            continue
        if record.get("status") not in LIVE_DRAFT_STATUSES:
            continue
        # В очереди пост уже не обсуждается: он ждёт своего слота.
        if record.get("queue_file"):
            continue
        created = record.get("created_at") or record.get("proposed_at")
        if not created:
            continue
        try:
            age = datetime.fromisoformat(str(created))
        except ValueError:
            continue
        if age.tzinfo is None:
            age = age.replace(tzinfo=timezone.utc)
        hours = (datetime.fromisoformat(moment.replace("Z", "+00:00")) - age).total_seconds() / 3600.0
        if hours < ttl_hours:
            continue
        record["status"] = "expired"
        record["expired_at"] = moment
        expired.append(post_id)
        # Факты черновика возвращаются в ожидание: событие не потеряно.
        # Берём полный факт из fact_dicts записи поста — по одному ключу
        # факт без claim нечем показать (он и не пригодился бы в посте).
        by_key = {str(f.get("key")): f for f in (record.get("fact_dicts") or []) if f.get("key")}
        keys = set(record.get("fact_keys") or []) | set(by_key)
        if keys:
            pending = {f.get("key"): f for f in state.get("pending", [])}
            for key in keys:
                was_pending = pending.pop(key, None) or {}
                from_post = by_key.get(key) or {}
                # Сливаем, а не выбираем одно: в pending может лежать только
                # ключ, а в записи поста — полный факт с claim и source.
                # Без merge факт терял заголовок и пост было не из чего собрать.
                fact = {**from_post, **{k: v for k, v in was_pending.items() if v not in (None, "")}}
                fact.setdefault("key", key)
                # Показ не равен отказу: счётчик попыток не растёт.
                fact["attempts"] = int(was_pending.get("attempts", from_post.get("attempts", 0)))
                fact.pop("post_id", None)
                pending[key] = fact
            state["pending"] = list(pending.values())
    return expired


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
