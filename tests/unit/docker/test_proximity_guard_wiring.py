"""Зона безопасности корпуса (#4365): имена мигалки и звука живут в каталогах, сервис по умолчанию выключен.

Лежит здесь, а не в ``rob_box_teleop/test``: в сборке пакета (G-Run Tests, ``test_ws``) нет ``sound_pack/``.
"""

import json
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "rob_box_teleop"))

from rob_box_teleop import proximity as px  # noqa: E402

COMPOSE = REPO / "docker" / "main" / "docker-compose.yaml"
NAV2_PARAMS = REPO / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"
GUARD_SCRIPT = REPO / "docker" / "main" / "scripts" / "teleop" / "start_proximity_guard.sh"


def test_alarm_names_exist_in_the_led_scenes_and_the_sound_pack():
    catalog = json.loads((REPO / "sound_pack" / "sound_catalog.json").read_text(encoding="utf-8"))
    triggers = {info["trigger"] for name, info in catalog["sounds"].items() if name.endswith(".mp3")}
    states = (REPO / "src" / "rob_box_animations" / "rob_box_animations" / "face" / "states.py").read_text(
        encoding="utf-8")
    for alarm in px.ALARMS.values():
        assert alarm.sound in triggers, alarm.sound
        assert f"'{alarm.animation}':" in states, alarm.animation
    assert px.ALARMS[px.ZONE_WARN].phrase is None, "«внимание» — без голоса (решение Шифу 04.11)"
    assert px.ALARMS[px.ZONE_STOP].phrase, "«стоп» — голосом"


def _service():
    return yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))["services"]["proximity-guard"]


def test_service_is_off_unless_its_profile_is_enabled():
    assert _service()["profiles"] == ["proximity-guard"]


def test_service_reads_the_nav2_footprint_and_runs_the_guard_script():
    svc = _service()
    assert "./config:/config:ro" in svc["volumes"]
    assert "./scripts/teleop:/scripts:ro" in svc["volumes"]
    assert svc["command"][-1] == "/scripts/start_proximity_guard.sh"
    assert "proximity_guard_node" in GUARD_SCRIPT.read_text(encoding="utf-8")
    body = px.body_from_nav2_params(yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8")))
    assert (body.x_max - body.x_min, body.y_max - body.y_min) == pytest.approx((0.61, 0.605))
