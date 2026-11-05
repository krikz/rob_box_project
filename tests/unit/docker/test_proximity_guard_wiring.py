"""Зона безопасности корпуса (#4365): имена мигалки и звука живут в каталогах, сервис всегда в стеке.

Лежит здесь, а не в ``rob_box_teleop/test``: в сборке пакета (G-Run Tests, ``test_ws``) нет ``sound_pack/``.
"""

import importlib.util
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


def test_service_is_always_in_the_stack_with_a_healthcheck():
    svc = _service()
    assert "profiles" not in svc, "деплой Main Pi — голый `docker compose up -d`: с профилем сервис не поднимется"
    assert svc["healthcheck"]["test"]
    assert svc["restart"] == "unless-stopped"


def test_service_reads_the_nav2_footprint_and_runs_the_guard_script():
    svc = _service()
    assert "./config:/config:ro" in svc["volumes"]
    assert "./scripts/teleop:/scripts:ro" in svc["volumes"]
    assert svc["command"][-1] == "/scripts/start_proximity_guard.sh"
    assert "proximity_guard_node" in GUARD_SCRIPT.read_text(encoding="utf-8")
    body = px.body_from_nav2_params(yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8")))
    assert (body.x_max - body.x_min, body.y_max - body.y_min) == pytest.approx((0.61, 0.605))


# -- бипер заднего хода и маячок «дорожные работы» (решение Шифу 05.11) -----------------------------------------

PACK = REPO / "sound_pack"


def _beep_generator():
    np = pytest.importorskip("numpy")
    spec = importlib.util.spec_from_file_location("_make_backup_beep", PACK / "make_backup_beep.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return np, mod


def test_backup_beeper_sounds_lie_in_the_pack_with_catalog_entries_matching_the_generator():
    _, gen = _beep_generator()
    sounds = json.loads((PACK / "sound_catalog.json").read_text(encoding="utf-8"))["sounds"]
    by_trigger = {info["trigger"]: (name, info) for name, info in sounds.items()}
    for alarm in px.ALARMS.values():
        name, info = by_trigger[alarm.sound]
        assert (PACK / name).is_file() and 0 < (PACK / name).stat().st_size < 20_000, name
        assert info["duration"] == gen.BEEPS[name], name          # длина в каталоге — та, что синтезирована
        assert info["category"] == "robot" and info["source"] == "synthesized", name
        # бип короче периода повтора: между бипами слышна пауза (бип 0.45 / пауза 0.45; стоп 0.2 / 0.2)
        assert info["duration"] == pytest.approx(alarm.sound_every_s / 2), name


def test_backup_beep_is_a_1khz_tone_inside_the_respeaker_band_without_clipping():
    np, gen = _beep_generator()
    for seconds in gen.BEEPS.values():
        x = gen.synth(seconds)
        assert len(x) == round(gen.RATE * seconds)
        assert np.max(np.abs(x)) == pytest.approx(10 ** (gen.PEAK_DBFS / 20)) and gen.PEAK_DBFS <= -6.0
        assert abs(x[0]) < 1e-3 and abs(x[-1]) < 0.05 * np.max(np.abs(x))   # плавные края — без щелчка
        spec = np.abs(np.fft.rfft(x)) ** 2
        freqs = np.fft.rfftfreq(len(x), 1 / gen.RATE)
        assert abs(freqs[np.argmax(spec)] - 1000.0) < 10.0                 # основной тон ~1 кГц
        assert spec[freqs > 7500.0].sum() < 1e-4 * spec.sum()              # всё в полосе ReSpeaker (до 8 кГц)


def test_roadworks_beacon_flashes_amber_left_then_right_never_both():
    pytest.importorskip("numpy")
    sys.path.insert(0, str(REPO / "src" / "rob_box_animations"))
    from rob_box_animations.face import states, vehicles  # noqa: E402

    assert states.SCENES["roadworks_beacon"].fn is vehicles.roadworks_beacon
    assert states.resolve("roadworks_beacon") == "roadworks_beacon"
    steps = [vehicles.roadworks_pattern(i / 100) for i in range(120)]
    assert not any(left and right for left, right in steps)
    assert sum(left for left, _ in steps) and sum(right for _, right in steps)
    for color in (vehicles.AMBER, vehicles.YELLOW):                       # янтарь: красный > зелёный > синего нет
        assert color[0] > color[1] > color[2] == 0
