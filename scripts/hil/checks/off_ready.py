#!/usr/bin/env python3
"""Вердикт возврата сервиса после ``hil.sh off`` (#5170, ADR-0018).

``running`` контейнера — не готовность. Для сервиса с Docker HEALTHCHECK
готовность — свежий ``.State.Health.Status`` этого запуска. Запись прошлой
жизни контейнера (StartedAt раньше момента возврата) готовностью не считается:
Docker переносит Health.Log через recreate, и старый ``healthy`` иначе
прошёл бы мимо SIGILL на configure.

Новых ``ros2 lifecycle get`` / ``lifecycle_probe`` здесь нет: такие пробы
уже гасили nav2 через bond-таймаут (#5150/#5151). Для nav2 критерий один —
штатный healthcheck контейнера (#5060).

Код выхода CLI: 0. Решение — слова stdout, не код процесса:
``ready`` | ``wait`` | ``fail <причина>`` | ``unknown`` | ``none``.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from typing import Any, Optional

# Слово товарища Шифу (бриф #5170): окно старта до 3 минут, нормальный подъём nav2.
START_WINDOW_S = 180
# Дольше этого Health.Log не считаем пробой этого запуска (interval nav2 — 15 с).
STALE_LOG_S = 120
# Статусы Docker, при которых контейнер жив и healthcheck вообще может идти.
_ALIVE = ("running", "restarting")

# Готовность до HIL не записана (старый PRE_STATE без поля health).
# Не PASS: «робот в боевом режиме» при неизвестной готовности — ложь (ADR-0018).
UNKNOWN_BEFORE = "unknown"


def parse_rfc3339(text: str) -> Optional[datetime]:
    """Docker StartedAt / Health.Log.Start → aware datetime. Пусто и мусор — None."""
    raw = (text or "").strip()
    if not raw or raw == "0001-01-01T00:00:00Z":
        return None
    if raw.endswith("Z"):
        raw = raw[:-1] + "+00:00"
    # Docker пишет наносекунды; datetime принимает микросекунды.
    dot = raw.find(".")
    if dot != -1:
        frac, _, tail = raw[dot + 1 :].partition("+")
        tz = ""
        if not tail and "-" in raw[dot + 1 :]:
            frac, _, tail = raw[dot + 1 :].partition("-")
            tz = "-" + tail if tail else ""
        else:
            tz = "+" + tail if tail else ""
        raw = raw[: dot + 1] + frac[:6].ljust(6, "0") + tz
    try:
        parsed = datetime.fromisoformat(raw)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed


def _seconds_between(later: datetime, earlier: datetime) -> float:
    return (later - earlier).total_seconds()


def health_log_output(health: dict) -> str:
    """Текст последней пробы Health.Log. Пусто — честно «пробы нет», не выдумка."""
    log = health.get("Log") or []
    if not log:
        return ""
    last = log[-1] or {}
    return str(last.get("Output") or "").strip()


def _probe_from_previous_life(health: dict, started: Optional[datetime]) -> bool:
    """Health.Log есть, но его Start раньше StartedAt этого запуска."""
    if started is None:
        return False
    log = health.get("Log") or []
    if not log:
        return False
    probed = parse_rfc3339(str((log[-1] or {}).get("Start") or ""))
    return probed is not None and probed < started


def probe_is_fresh(health: dict, started: Optional[datetime], now: datetime) -> bool:
    """Проба этого запуска, не запись прошлой жизни контейнера.

    Сравниваем Start последней записи Health.Log с StartedAt контейнера.
    Лога нет — пробы этого запуска ещё не было (start_period), это не «свежий healthy».
    """
    if started is None:
        return False
    log = health.get("Log") or []
    if not log:
        return False
    last = log[-1] or {}
    probed = parse_rfc3339(str(last.get("Start") or ""))
    if probed is None:
        return False
    if probed < started:
        return False
    return _seconds_between(now, probed) <= STALE_LOG_S


def readiness_of(inspect: dict, now: Optional[datetime] = None) -> str:
    """Слово готовности по одному ``docker inspect``: healthy|unhealthy|starting|none|absent.

    ``none`` — HEALTHCHECK у контейнера нет (готовность не спрашиваем, как раньше).
    ``starting`` — healthcheck есть, но свежей пробы этого запуска ещё нет
    либо статус ``starting``. ``unhealthy`` — свежая проба не ``healthy``.
    """
    now = now or datetime.now(timezone.utc)
    state = inspect.get("State")
    # Пустой объект и объект без State — docker молчит. Это не «контейнера нет»:
    # absent только когда inspect явно сказал Status=absent (#5170).
    if not isinstance(state, dict) or not state:
        return "starting"
    status = str(state.get("Status") or "").strip() or "starting"
    if status == "absent":
        return "absent"
    health = state.get("Health")
    if not isinstance(health, dict) or not health:
        return "none"
    started = parse_rfc3339(str(state.get("StartedAt") or ""))
    if status not in _ALIVE:
        return "unhealthy"
    # Проба прошлой жизни контейнера (Start < StartedAt) — не «ещё стартует».
    # Docker переносит Health.Log через recreate: старый healthy иначе прошёл бы.
    if _probe_from_previous_life(health, started):
        return "unhealthy"
    if not probe_is_fresh(health, started, now):
        return "starting"
    reported = str(health.get("Status") or "").strip().lower()
    if reported == "healthy":
        return "healthy"
    if reported == "starting":
        return "starting"
    return "unhealthy"


def window_open(returned_at: Optional[datetime], now: datetime) -> bool:
    """Окно старта ещё не закрыто. Момент возврата неизвестен — окна нет (не ждём вечно)."""
    if returned_at is None:
        return False
    return _seconds_between(now, returned_at) < START_WINDOW_S


def before_ready(pre_field: str) -> str:
    """Поле готовности из PRE_STATE. Пусто / «-» / мусор — готовность до HIL неизвестна."""
    field = (pre_field or "").strip()
    if field in ("", "-", "?", "unset"):
        return UNKNOWN_BEFORE
    return field


def verdict(before: str, after_status: str, after_ready: str) -> str:
    """ok | warn | fail — то же тройное решение, что раньше у ``svc_verdict``, плюс готовность.

    Сервис без HEALTHCHECK (``none``) судится по ``running``, как раньше.
    До HIL был ``healthy``, после off не ``healthy`` — fail, даже если контейнер ``running``.
    До HIL готовность неизвестна, а сейчас не ``healthy`` — не ok (нельзя сказать «в бою»).
    """
    if after_status == "running" and after_ready in ("healthy", "none"):
        return "ok"
    if after_ready == "starting":
        return "wait"
    before_status, _, before_health = (before or "").partition(":")
    before_status = before_status.strip() or "running"
    prior = before_ready(before_health)
    if after_status != "running" and before_status != "running" and prior != "healthy":
        return "warn"
    return "fail"


def fail_reason(name: str, after_status: str, after_ready: str, probe: str) -> str:
    """Причина FAIL: статус контейнера и текст последней пробы, не общая фраза."""
    text = probe.strip() or "пробы healthcheck нет"
    one_line = " | ".join(line.strip() for line in text.splitlines() if line.strip())
    return f"{name}: после off {after_status}/{after_ready}; healthcheck: {one_line}"


def decide(
    before: str,
    inspect: dict,
    returned_at: Optional[datetime],
    now: Optional[datetime] = None,
) -> tuple[str, str]:
    """Пара (слово, причина). Слово: ready|wait|fail|unknown.

    ``unknown`` — до HIL готовность не записана, а сейчас сервис не healthy
    (старый PRE_STATE). Это не PASS и не «подожди ещё»: печатать «в боевом
    режиме» нельзя, замок не снимать.
    """
    now = now or datetime.now(timezone.utc)
    state = inspect.get("State") or {}
    status = str(state.get("Status") or "absent")
    ready = readiness_of(inspect, now)
    word = verdict(before, status, ready)
    if word == "ok":
        return "ready", ""
    if word == "wait" and window_open(returned_at, now):
        return "wait", ""
    if word == "warn":
        return "warn", ""
    name = str(inspect.get("Name") or "?").lstrip("/")
    probe = health_log_output(
        (state.get("Health") or {}) if isinstance(state.get("Health"), dict) else {}
    )
    _, _, before_health = (before or "").partition(":")
    if before_ready(before_health) == UNKNOWN_BEFORE and ready not in (
        "healthy",
        "none",
    ):
        reason = fail_reason(name, status, ready, probe)
        return "unknown", f"готовность до HIL неизвестна; {reason}"
    return "fail", fail_reason(name, status, ready, probe)


def _load_inspect(path: str) -> dict[str, Any]:
    if path == "-":
        raw = sys.stdin.read()
    else:
        with open(path, encoding="utf-8") as fh:
            raw = fh.read()
    data = json.loads(raw or "{}")
    if isinstance(data, list):
        data = data[0] if data else {}
    if not isinstance(data, dict):
        raise ValueError("docker inspect: ждали объект")
    return data


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="вердикт готовности сервиса после hil.sh off"
    )
    parser.add_argument(
        "--inspect", required=True, help="файл docker inspect (JSON) или -"
    )
    parser.add_argument(
        "--before", default="running:-", help="поле PRE_STATE: status:health"
    )
    parser.add_argument(
        "--returned-at", default="", help="RFC3339 момента compose up этого off"
    )
    parser.add_argument(
        "--now", default="", help="RFC3339 «сейчас»; пусто — системные часы"
    )
    parser.add_argument(
        "--readiness",
        action="store_true",
        help="напечатать только слово готовности (для снимка PRE_STATE на on)",
    )
    args = parser.parse_args(argv)
    inspect = _load_inspect(args.inspect)
    now = parse_rfc3339(args.now) if args.now else datetime.now(timezone.utc)
    if args.readiness:
        print(readiness_of(inspect, now))
        return 0
    word, reason = decide(args.before, inspect, parse_rfc3339(args.returned_at), now)
    print(word)
    if reason:
        print(reason)
    return 0


if __name__ == "__main__":
    sys.exit(main())
