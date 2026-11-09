"""Guard #4531: холодный старт Sound Pi переживает неготовый zenoh-роутер.

Проверяет поля compose и юнита (поведение конфигурации, не тексты комментариев):
окно ожидания healthy у роутера не меньше порога, юнит повторяет запуск и ждёт сеть/время.
"""

from __future__ import annotations

import configparser
import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
COMPOSE = REPO_ROOT / "docker" / "sound" / "docker-compose.yaml"
UNIT = REPO_ROOT / "host" / "sound" / "robbox-sound.service"

# Замер 07.11: на холодном старте роутер не успел за 30 + 20 x 5 = 130 с.
MIN_WAIT_SECONDS = 180


def _seconds(value: str) -> int:
    m = re.fullmatch(r"(\d+)(s|m)?", str(value).strip())
    assert m, f"не умею разобрать длительность {value!r}"
    return int(m.group(1)) * (60 if m.group(2) == "m" else 1)


def _unit() -> configparser.ConfigParser:
    cp = configparser.ConfigParser(strict=False, interpolation=None)
    cp.optionxform = str
    cp.read(UNIT, encoding="utf-8")
    return cp


def test_zenoh_router_waits_long_enough_for_cold_start() -> None:
    hc = yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))["services"]["zenoh-router"]["healthcheck"]
    window = _seconds(hc["start_period"]) + hc["retries"] * _seconds(hc["interval"])
    assert window >= MIN_WAIT_SECONDS, f"окно ожидания роутера {window} с < {MIN_WAIT_SECONDS} с"


def test_unit_retries_after_failed_compose_up() -> None:
    svc = _unit()["Service"]
    assert svc["Restart"] == "on-failure"
    assert 1 <= _seconds(svc["RestartSec"]) <= 60
    assert _unit()["Unit"]["StartLimitIntervalSec"] == "0", "лимит повторов не должен оставить юнит в failed"


def test_unit_waits_for_network_and_time_sync() -> None:
    unit = _unit()["Unit"]
    for target in ("network-online.target", "time-sync.target"):
        assert target in unit["After"].split(), f"{target} нет в After="
        assert target in unit["Wants"].split(), f"{target} нет в Wants= (одного After мало)"


def test_unit_starts_without_network_pull() -> None:
    svc = _unit()["Service"]
    assert svc["Type"] == "oneshot"
    assert "up -d" in svc["ExecStart"] and "--pull never" in svc["ExecStart"]
    assert svc["ExecStop"].endswith("compose down")
