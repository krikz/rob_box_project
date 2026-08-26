"""Phase 2.2 ROS2 subscriber: /quest/perf → /var/log/quest_perf/$(date).jsonl.

Подписывается на топик ``/quest/perf`` (тип ``std_msgs/String`` — JSON-encoded
``telemetry_perf`` payload, см. docs/architecture/meta-quest-api.md §6.1).
Валидирует payload, дедуплицирует по ``seq``, пишет в jsonl с ротацией по дням.

Зачем отдельная нода
--------------------
На Vision Pi нода ``rob_box_quest`` форвардит WSS payloads в ROS2. Нам нужен
*подписчик*, который:
  1. Сохраняет данные для Phase 3 Grafana (sqlite + jsonl).
  2. Не блокирует WSS server-side цикл (минимум CPU, async logging).
  3. Выдерживает переполнение диска (rotation по дням + trim старых файлов).

Использование (colcon-стиль, ROS2 humble)
-----------------------------------------
  ros2 run rob_box_quest quest_perf_logger

Или как composable node внутри ``rob_box_quest_container``.

Тестирование
------------
``python3 -m pytest test/test_perf_logger.py -v`` — юнит-тесты на JSONL
rotation и seq dedup (без ROS2 зависимостей).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String as StringMsg
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
except ImportError:  # pragma: no cover - rclpy недоступен вне ROS2 env
    rclpy = None
    Node = None  # type: ignore[assignment,misc]
    StringMsg = None  # type: ignore[assignment]
    SingleThreadedExecutor = None  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    QoSReliabilityPolicy = None  # type: ignore[assignment]

LOG = logging.getLogger("quest_perf_logger")

# Default location для jsonl-файлов (см. body §4 карточки):
DEFAULT_LOG_DIR = "/var/log/quest_perf"

# Cap на размер одного файла (ротация внутри дня). 50 MB × 30 дней ≈ 1.5 GB.
DEFAULT_MAX_BYTES_PER_FILE = 50 * 1024 * 1024
DEFAULT_RETENTION_DAYS = 30


class JsonlWriter:
    """Thread-safe JSONL writer с ротацией по дате и cap по размеру.

    Открывает ``{date}.jsonl`` в ``log_dir`` (append-режим). При смене даты
    или превышении ``max_bytes_per_file`` — закрывает и открывает новый файл.
    Также при старте удаляет файлы старше ``retention_days``.
    """

    def __init__(
        self,
        log_dir: str = DEFAULT_LOG_DIR,
        max_bytes_per_file: int = DEFAULT_MAX_BYTES_PER_FILE,
        retention_days: int = DEFAULT_RETENTION_DAYS,
    ) -> None:
        self.log_dir = Path(log_dir)
        self.max_bytes_per_file = max_bytes_per_file
        self.retention_days = retention_days
        self._lock = threading.Lock()
        self._current_date: str = ""
        self._current_bytes: int = 0
        self._fh = None
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._prune_old_files()

    def _filename(self, date_str: str) -> Path:
        return self.log_dir / f"{date_str}.jsonl"

    def _open(self, date_str: str) -> None:
        if self._fh is not None:
            self._fh.close()
        path = self._filename(date_str)
        self._fh = path.open("a", encoding="utf-8", buffering=1)  # line-buffered
        self._current_bytes = path.stat().st_size if path.exists() else 0
        self._current_date = date_str

    def _prune_old_files(self) -> None:
        """Удаляет файлы старше retention_days (best-effort, без raise)."""
        if self.retention_days <= 0:
            return
        cutoff = time.time() - self.retention_days * 86400
        try:
            for p in self.log_dir.glob("*.jsonl"):
                if p.stat().st_mtime < cutoff:
                    p.unlink()
                    LOG.info("pruned old log file: %s", p)
        except OSError as exc:
            LOG.warning("prune failed: %s", exc)

    def write(self, record: dict[str, Any]) -> None:
        """Записать record как JSON line. Thread-safe.

        Ротация: по дате (UTC) и по размеру файла.
        """
        line = json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n"
        encoded = line.encode("utf-8")

        with self._lock:
            today = datetime.utcnow().strftime("%Y-%m-%d")
            if self._fh is None or self._current_date != today:
                self._open(today)
            elif self._current_bytes + len(encoded) > self.max_bytes_per_file:
                # Rotate внутри дня: добавляем .N к имени (N = counter).
                base = self._filename(self._current_date)
                # Найти следующий свободный индекс.
                idx = 1
                while True:
                    candidate = self.log_dir / f"{self._current_date}.{idx}.jsonl"
                    if not candidate.exists():
                        break
                    idx += 1
                self._fh.close()
                # Rename текущего файла в .N
                if base.exists():
                    base.rename(self.log_dir / f"{self._current_date}.{idx}.jsonl")
                self._open(today)

            self._fh.write(line)
            self._current_bytes += len(encoded)

    def close(self) -> None:
        with self._lock:
            if self._fh is not None:
                self._fh.close()
                self._fh = None


class PerfDedup:
    """Track last-seen seq для каждого session_id (защита от duplicate frames).

    On WSS reconnect client может прислать тот же seq ещё раз — игнорируем.
    """

    def __init__(self, max_sessions: int = 64) -> None:
        self._lock = threading.Lock()
        self._last_seq: dict[str, int] = {}
        self._max_sessions = max_sessions

    def is_duplicate(self, session_id: str, seq: int) -> bool:
        with self._lock:
            if self._last_sessions_size() > self._max_sessions:
                # FIFO trim — старейшая запись удаляется.
                oldest = next(iter(self._last_seq))
                self._last_seq.pop(oldest, None)
            last = self._last_seq.get(session_id, -1)
            if seq <= last:
                return True
            self._last_seq[session_id] = seq
            return False

    def _last_sessions_size(self) -> int:
        return len(self._last_seq)


def validate_telemetry_payload(payload: dict[str, Any]) -> Optional[str]:
    """Возвращает строку с ошибкой или None, если payload валиден.

    Минимальные проверки: наличие ``type``, ``ts_ms``, ``seq``, ``source``.
    Остальные поля опциональны (могут отсутствовать в desktop-режиме).
    """
    if payload.get("type") != "telemetry_perf":
        return f"unexpected type: {payload.get('type')!r}"
    ts = payload.get("ts_ms")
    if not isinstance(ts, (int, float)) or ts <= 0:
        return f"invalid ts_ms: {ts!r}"
    seq = payload.get("seq")
    if not isinstance(seq, int) or seq < 0:
        return f"invalid seq: {seq!r}"
    source = payload.get("source")
    if source not in ("desktop", "webxr"):
        return f"invalid source: {source!r}"
    return None


# ---- ROS2 Node wrapper ---------------------------------------------------

if rclpy is not None and Node is not None:
    class QuestPerfLoggerNode(Node):  # type: ignore[misc, valid-type]
        """ROS2 node, подписанная на /quest/perf."""

        def __init__(
            self,
            log_dir: str = DEFAULT_LOG_DIR,
            max_bytes_per_file: int = DEFAULT_MAX_BYTES_PER_FILE,
            retention_days: int = DEFAULT_RETENTION_DAYS,
        ) -> None:
            super().__init__("quest_perf_logger")
            self._writer = JsonlWriter(log_dir, max_bytes_per_file, retention_days)
            self._dedup = PerfDedup()
            qos = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
            self._sub = self.create_subscription(
                StringMsg,
                "/quest/perf",
                self._on_message,
                qos,
            )
            self.get_logger().info(
                f"quest_perf_logger started, logging to {log_dir}"
            )

        def _on_message(self, msg: StringMsg) -> None:
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError as exc:
                self.get_logger().warn(f"invalid JSON: {exc}")
                return

            err = validate_telemetry_payload(payload)
            if err is not None:
                self.get_logger().warn(f"invalid payload: {err}")
                return

            session_id = payload.get("session_id", "unknown")
            seq = int(payload["seq"])
            if self._dedup.is_duplicate(session_id, seq):
                self.get_logger().debug(f"dup seq={seq} session={session_id}")
                return

            self._writer.write(payload)

        def close(self) -> None:
            self._writer.close()


# ---- CLI ------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Phase 2.2 telemetry logger (/quest/perf -> jsonl)",
    )
    p.add_argument(
        "--log-dir",
        default=os.environ.get("QUEST_PERF_LOG_DIR", DEFAULT_LOG_DIR),
        help="куда писать jsonl (default: %(default)s)",
    )
    p.add_argument(
        "--max-bytes",
        type=int,
        default=int(os.environ.get("QUEST_PERF_MAX_BYTES", DEFAULT_MAX_BYTES_PER_FILE)),
        help="ротация внутри дня при превышении этого размера",
    )
    p.add_argument(
        "--retention-days",
        type=int,
        default=int(os.environ.get("QUEST_PERF_RETENTION_DAYS", DEFAULT_RETENTION_DAYS)),
        help="хранить логи N дней (0 = хранить вечно)",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="не запускать ROS2 (для юнит-тестов и dev-проверки)",
    )
    return p.parse_args()


def main() -> int:
    logging.basicConfig(
        level=os.environ.get("LOG_LEVEL", "INFO"),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    args = _parse_args()

    if args.dry_run or rclpy is None:
        LOG.info(
            "dry-run mode (rclpy=%s, dry_run=%s); nothing to do",
            rclpy is not None,
            args.dry_run,
        )
        return 0

    rclpy.init()
    node = QuestPerfLoggerNode(
        log_dir=args.log_dir,
        max_bytes_per_file=args.max_bytes,
        retention_days=args.retention_days,
    )

    def _shutdown(signum, frame):  # noqa: ARG001
        LOG.info("shutdown signal %s received", signum)
        node.close()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())