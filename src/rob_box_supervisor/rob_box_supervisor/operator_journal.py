"""operator_journal.py — журнал ТАРС (лог изменений оператора, §5.4 целевой архитектуры).

Память ТАРС — **не история чата, а лог изменений**: что сделал, когда, чем
кончилось (ADR-0051 §2.5, target-operator-agent-and-dialogue.md §5.4).

Сжатие двухступенчатое (§5.4):

1. **Схлопывание повторов** — основное. Три записи «перезапустил
   voice-assistant» за час становятся одной: «перезапускал voice-assistant
   3 раза за час, последний раз в 14:32». Почти без потери смысла, потому
   что смысл здесь в счётчике.
2. **LLM-саммари хвоста** — страховая для записей старше суток (вне этого
   модуля; здесь — только схлопывание + bounded-окно).

Модуль чистый (без ROS, без файловой системы в контракте — персист опционален
через ``path``), синхронный и тестируемый без rclpy. Живёт в
``rob_box_supervisor`` (домен оператора), переживает чистку harness-каркаса.

Контракт ``record()``:
- запись идентифицируется ``action`` (то, что сделано);
- если последняя запись с тем же ``action`` в окне ``collapse_window_s`` —
  счётчик ``count += 1``, обновляется ``last_ts_ms`` (и ``outcome``/``detail``
  — как последний исход);
- иначе — новая запись.

``recent(limit)`` / ``render(limit)`` отдают окно для инжекта в
``dynamic_system`` операторского AgentCore (чтобы ТАРС знал, что недавно
делал).
"""

from __future__ import annotations

import json
import os
import time as _time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping, Optional

# Дефолт окна схлопывания: час (DoD-инвариант журнала).
DEFAULT_COLLAPSE_WINDOW_S: float = 3600.0
# Максимум записей в памяти: журнал — bounded-окно, не бесконечный лог.
DEFAULT_MAX_ENTRIES: int = 500


@dataclass(frozen=True)
class JournalEntry:
    """Одна схлопнутая запись журнала оператора.

    ``ts_ms`` — первое вхождение действия, ``last_ts_ms`` — самое свежее,
    ``count`` — сколько раз действие повторилось в окне схлопывания
    (начиная с 1).
    """

    action: str
    ts_ms: int
    last_ts_ms: int
    count: int = 1
    outcome: str = ""
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        """Плоский dict для сериализации (JSONL)."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "JournalEntry":
        """Восстановить запись из dict (JSONL-десериализация)."""
        return cls(
            action=str(data.get("action", "")),
            ts_ms=int(data.get("ts_ms", 0)),
            last_ts_ms=int(data.get("last_ts_ms", data.get("ts_ms", 0))),
            count=int(data.get("count", 1)),
            outcome=str(data.get("outcome", "") or ""),
            detail=str(data.get("detail", "") or ""),
        )


class OperatorJournal:
    """Лог изменений оператора со схлопыванием повторов (§5.4).

    Синхронный и thread-safe-по-минимуму (ROS-callback супервизора —
    одиночный поток; блокировка не нужна, но мутации идут через один
    объект). Персист опционален: при ``path`` запись/загрузка в JSONL.

    Args:
        collapse_window_s: окно схлопывания повторов в секундах (default 1 ч).
        max_entries: потолок числа записей в памяти (вытесняем старейшие).
        path: опциональный путь JSONL-файла для персиста (''/None = memory-only).
        clock: источник времени (для тестов; default ``time.time``).
    """

    name = "operator"

    def __init__(
        self,
        *,
        collapse_window_s: float = DEFAULT_COLLAPSE_WINDOW_S,
        max_entries: int = DEFAULT_MAX_ENTRIES,
        path: Optional[str] = None,
        clock: Any = None,
    ) -> None:
        self._collapse_window_s = float(collapse_window_s)
        self._max_entries = int(max_entries)
        self._path: Optional[str] = path
        # ``clock()`` → float epoch-seconds. Тесты подменяют для
        # детерминизма окна схлопывания.
        self._clock = clock or _time.time
        # Записи в порядке вставки (старые первыми). Поиск окна — по action.
        self._entries: list[JournalEntry] = []
        if path:
            self._load()

    # ── мутации ──────────────────────────────────────────────────────

    def record(
        self,
        action: str,
        *,
        outcome: str = "",
        detail: str = "",
        ts_ms: Optional[int] = None,
    ) -> JournalEntry:
        """Зафиксировать действие оператора.

        Если последняя запись с тем же ``action`` лежит в окне
        схлопывания (``now - last_ts_ms <= collapse_window_s``) — это
        повтор: ``count += 1``, ``last_ts_ms``/``outcome``/``detail``
        обновляются. Иначе — новая запись.

        Returns:
            Обновлённая (или новая) запись.
        """
        action = str(action or "").strip()
        if not action:
            raise ValueError("OperatorJournal.record: action required")
        ts = int(ts_ms) if ts_ms is not None else self._now_ms()

        # Ищем последнюю запись с тем же action с конца.
        for idx in range(len(self._entries) - 1, -1, -1):
            entry = self._entries[idx]
            if entry.action != action:
                continue
            # Повтор в окне → схлопываем.
            if ts - entry.last_ts_ms <= self._collapse_window_s * 1000:
                collapsed = JournalEntry(
                    action=action,
                    ts_ms=entry.ts_ms,
                    last_ts_ms=ts,
                    count=entry.count + 1,
                    outcome=outcome,
                    detail=detail,
                )
                self._entries[idx] = collapsed
                self._save()
                return collapsed
            # Тот же action, но вне окна → новая запись (break: дальше
            # старее — не ищем).
            break

        new_entry = JournalEntry(
            action=action,
            ts_ms=ts,
            last_ts_ms=ts,
            count=1,
            outcome=outcome,
            detail=detail,
        )
        self._entries.append(new_entry)
        # Bounded-окно: вытесняем старейшие.
        if len(self._entries) > self._max_entries:
            self._entries = self._entries[-self._max_entries:]
        self._save()
        return new_entry

    def clear(self) -> None:
        """Очистить журнал (тесты / ручной сброс)."""
        self._entries = []
        self._save()

    # ── чтение ───────────────────────────────────────────────────────

    def recent(self, *, limit: int = 20) -> list[JournalEntry]:
        """Свежайшие записи (по убыванию ``last_ts_ms``)."""
        ordered = sorted(
            self._entries, key=lambda e: e.last_ts_ms, reverse=True
        )
        return ordered[:limit]

    def render(self, *, limit: int = 10, now_ms: Optional[int] = None) -> str:
        """Компактный текст для инжекта в ``dynamic_system`` AgentCore.

        Строки вида: ``• 14:32 перезапустил voice-assistant ×3 (outcome)``.
        Пустой журнал → пустая строка (ничего не вклеиваем).
        """
        entries = self.recent(limit=limit)
        if not entries:
            return ""
        now = int(now_ms) if now_ms is not None else self._now_ms()
        lines: list[str] = []
        for e in entries:
            when = _fmt_clock(e.last_ts_ms, now)
            suffix = f" ×{e.count}" if e.count > 1 else ""
            outcome = f" — {e.outcome}" if e.outcome else ""
            lines.append(f"• {when} {e.action}{suffix}{outcome}")
        return "[журнал оператора]\n" + "\n".join(lines)

    def __len__(self) -> int:
        return len(self._entries)

    # ── персист (JSONL) ──────────────────────────────────────────────

    def _load(self) -> None:
        """Прочитать журнал из JSONL (пропуская битые строки)."""
        if not self._path:
            return
        path = Path(self._path)
        if not path.exists():
            return
        entries: list[JournalEntry] = []
        try:
            with open(path, "r", encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        entries.append(JournalEntry.from_dict(json.loads(line)))
                    except (ValueError, TypeError):
                        continue
        except OSError:
            # Журнал не должен ронять супервизор: не читается → пусто.
            return
        entries.sort(key=lambda e: e.ts_ms)
        self._entries = entries[-self._max_entries:]

    def _save(self) -> None:
        """Записать журнал в JSONL (best-effort, atomic через tmp+rename)."""
        if not self._path:
            return
        try:
            path = Path(self._path)
            if path.parent and not path.parent.exists():
                os.makedirs(path.parent, exist_ok=True)
            tmp = path.with_suffix(path.suffix + ".tmp")
            with open(tmp, "w", encoding="utf-8") as fh:
                for entry in self._entries:
                    fh.write(json.dumps(entry.to_dict(), ensure_ascii=False) + "\n")
            os.replace(tmp, path)
        except OSError:
            # Best-effort: персист не должен валить обработку команды.
            return

    # ── helpers ──────────────────────────────────────────────────────

    def _now_ms(self) -> int:
        return int(self._clock() * 1000)


def _fmt_clock(ts_ms: int, now_ms: int) -> str:
    """Отформатировать ``ts_ms`` как HH:MM, либо «N мин назад» для свежих."""
    delta_min = max(0, int((now_ms - ts_ms) / 60_000))
    if delta_min < 60:
        return f"{delta_min} мин назад" if delta_min else "только что"
    try:
        import datetime

        return datetime.datetime.fromtimestamp(ts_ms / 1000).strftime("%H:%M")
    except (OSError, OverflowError, ValueError):
        return f"{ts_ms}"


__all__ = [
    "DEFAULT_COLLAPSE_WINDOW_S",
    "DEFAULT_MAX_ENTRIES",
    "JournalEntry",
    "OperatorJournal",
]
