"""Guard #4531: юнит автозапуска Main Pi переживает холодный старт.

Проверяет поля юнита (поведение конфигурации, не тексты комментариев): ждёт сеть,
время и CAN, повторяет запуск без лимита, не тянет образы из сети.
"""

from __future__ import annotations

import configparser
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
UNIT = REPO_ROOT / "host" / "main" / "robbox-main.service"


def _seconds(value: str) -> int:
    m = re.fullmatch(r"(\d+)(s|m)?", str(value).strip())
    assert m, f"не умею разобрать длительность {value!r}"
    return int(m.group(1)) * (60 if m.group(2) == "m" else 1)


def _unit() -> configparser.ConfigParser:
    cp = configparser.ConfigParser(strict=False, interpolation=None)
    cp.optionxform = str
    cp.read(UNIT, encoding="utf-8")
    return cp


def test_unit_retries_after_failed_compose_up() -> None:
    svc = _unit()["Service"]
    assert svc["Restart"] == "on-failure"
    assert 1 <= _seconds(svc["RestartSec"]) <= 60
    assert _unit()["Unit"]["StartLimitIntervalSec"] == "0", "лимит повторов не должен оставить юнит в failed"
    assert _seconds(svc["TimeoutStartSec"]) >= 300


def test_unit_waits_for_network_time_and_can() -> None:
    unit = _unit()["Unit"]
    for target in ("network-online.target", "time-sync.target", "can-setup.service"):
        assert target in unit["After"].split(), f"{target} нет в After="
        assert target in unit["Wants"].split(), f"{target} нет в Wants= (одного After мало)"
    assert "docker.service" in unit["Requires"].split()


def test_unit_starts_without_network_pull() -> None:
    svc = _unit()["Service"]
    assert svc["Type"] == "oneshot"
    assert "up -d" in svc["ExecStart"] and "--pull never" in svc["ExecStart"]
    assert svc["ExecStop"].endswith("compose down")
    assert svc["WorkingDirectory"].endswith("/docker/main")
