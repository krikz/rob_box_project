"""Эмпирический сбор STT-семплов с wake-сегментов шлема — issue #2158, ADR-0076.

Источник: wake-поток шлема (source=_SRC_WAKE в stt_node). STT-вывод
записывается в JSONL **только когда**:

    ROBBOX_STT_COLLECT=1   (kill-switch по умолчанию; без переменной no-op)

Цель — собрать факты о том, как именно STT (Yandex + Vosk fallback)
слышит «ТАРС» на микрофоне шлема, чтобы принимать решение о расширении
wake-списка **по данным**, а не придумывать (целевая §14).

**Что пишется:**

* каждый wake-сегмент, для которого STT вернул НЕпустой текст;
* каждый wake-сегмент, для которого STT вернул пусто (только причина
  и длительность — текста у нас нет).

**Чего модуль НЕ делает:**

* не меняет поведение маршрутизации (`/avatar/stt/result`);
* не читает состояние ROS;
* не зависит от сети / диска / времени;
* никуда не пишет, кроме ``path`` (по умолчанию ``data/stt_tars_samples.jsonl``).
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Optional, Sequence


# Kill-switch: переменная окружения. Если не выставлена — сбор отключён.
# Это сознательно жёсткий рубильник: в проде карточка требует явного
# включения, никаких дефолтных значений «на всякий случай».
def _collect_enabled() -> bool:
    return os.environ.get("ROBBOX_STT_COLLECT") == "1"


# Путь по умолчанию. ``data/`` уже в .gitignore — не утечёт в коммит,
# если кто-то случайно запустит с переменной.
DEFAULT_LOG_PATH = Path("data") / "stt_tars_samples.jsonl"


def append_sample(
    *,
    raw_text: Optional[str],
    has_operator_wake: bool,
    duration_s: float,
    attempts: Sequence[dict],
    operator_wake_words: Sequence[str],
    path: Path | str = DEFAULT_LOG_PATH,
) -> bool:
    """Записать один wake-сегмент в JSONL. Возвращает True, если записали.

    ``attempts`` — список словарей ``{"provider": "yandex|vosk", "ok":
    bool, "reason": "ok|empty|timeout|error|low_confidence"}``. Это снимок
    первичного провайдера и факта успеха — для пост-анализа «а был ли
    fallback или Yandex вернул мусор сам».

    Это **side-channel для отладки**, никаких обещаний формата/совместимости
    вне ADR-0076. Пишется атомарно: сначала полная строка в памяти,
    затем один ``write`` — чтобы при вылете середины сегмента JSONL
    остался валидным для подсчёта частот.
    """
    if not _collect_enabled():
        return False
    try:
        record = {
            "ts_ms": int(time.time() * 1000),
            "source": "wake",
            "duration_s": round(float(duration_s), 3),
            "raw_text": raw_text,
            "has_operator_wake": bool(has_operator_wake),
            "operator_wake_words": list(operator_wake_words),
            "attempts": list(attempts),
        }
        line = json.dumps(record, ensure_ascii=False)
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "a", encoding="utf-8") as fh:
            fh.write(line + "\n")
        return True
    except OSError:
        # Файловая система вне нашей юрисдикции — STT не должен падать
        # из-за сборщика. Это строго side-channel.
        return False


def build_attempts_snapshot(
    attempts,
) -> list[dict]:
    """Безопасный снимок списка ``STTAttempt`` / аналогов → dict-список.

    Не импортируем rob_box_voice.stt_fallback — это тяжёлая зависимость.
    Используем hasattr/duck-typing."""
    out: list[dict] = []
    for a in attempts or []:
        try:
            out.append(
                {
                    "provider": getattr(a, "provider", "unknown"),
                    "ok": bool(getattr(a, "reason", "") == "ok"),
                    "reason": getattr(a, "reason", "unknown"),
                    "latency_ms": int(getattr(a, "latency_ms", 0) or 0),
                }
            )
        except Exception:
            out.append({"provider": "unknown", "ok": False, "reason": "snapshot_failed"})
    return out


__all__ = [
    "DEFAULT_LOG_PATH",
    "append_sample",
    "build_attempts_snapshot",
]
