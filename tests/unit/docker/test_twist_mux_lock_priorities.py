"""Замок пульта не глушит держателя floor (#4694, ADR-0081 §5.1).

twist_mux глушит источники СТРОГО меньшего приоритета (замер на роботе,
ADR-0081 §5.1). joystick_lock=100 при quest=90 маскировал шлем, пока пульт
вооружён и молчит. Замок должен глушить web/voice/nav и не трогать quest.
cmd_vel_joy остаётся 100, emergency — 255, teleop_lock — 90.
"""

from __future__ import annotations

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
TWIST_MUX = REPO_ROOT / "docker" / "main" / "config" / "twist_mux" / "twist_mux.yaml"

# Источники, которые замок пульта обязан глушить (issue #1344).
BLOCKED = {"web_ui": 50, "voice": 25, "navigation": 10}


def _params() -> dict:
    return yaml.safe_load(TWIST_MUX.read_text(encoding="utf-8"))["twist_mux"]["ros__parameters"]


def test_joystick_lock_blocks_web_voice_nav_and_not_quest():
    params = _params()
    topics = params["topics"]
    locks = params["locks"]
    lock = locks["joystick_lock"]["priority"]
    quest = topics["quest"]["priority"]
    joy = topics["joystick"]["priority"]
    emergency = topics["emergency_stop"]["priority"]

    assert locks["joystick_lock"]["topic"] == "/joystick_lock"
    assert locks["joystick_lock"]["timeout"] == 0.0
    assert locks["teleop_lock"]["priority"] == 90
    assert topics["quest"]["topic"] == "cmd_vel_quest"
    assert quest == 90
    assert joy == 100
    assert emergency == 255

    # strict <: равный приоритет не глушится (ADR-0081 §5.1).
    assert lock < quest, "замок пульта глушит quest — ARM шлема не дойдёт до моторов"
    assert lock < joy, "замок не должен подниматься до аппаратуры"
    assert lock < emergency
    for name, priority in BLOCKED.items():
        assert topics[name]["priority"] == priority
        assert lock > priority, f"{name} не глушится замком пульта"
