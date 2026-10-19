"""Guard: docker/sound — голосовой стек переехал с Vision Pi (ADR-0159 PR-2b).

Проверяет поведение compose-файлов и дерева каталогов, а не тексты комментариев:
сервисы живут ровно в одном месте (одна реализация), пути состояния и конфигов
указывают на существующее, sensor-bridge не стартует по умолчанию.
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
SOUND = REPO_ROOT / "docker" / "sound"
VISION = REPO_ROOT / "docker" / "vision"

# supervisor (ТАРС) — решение Шифу 19.10 (ADR-0159 В8): пишет в тот же harness_voice.db.
MOVED_SERVICES = ("voice-assistant", "supercollider", "voice-action-server", "supervisor")


def _compose(pi_dir: Path) -> dict:
    return yaml.safe_load((pi_dir / "docker-compose.yaml").read_text(encoding="utf-8"))


def _default(value: str) -> str:
    """``${VAR:-default}`` → ``default`` — как compose при пустом окружении."""
    return re.sub(r"\$\{[A-Za-z_]+:-([^}]*)\}", r"\1", value)


def _env(service: dict) -> dict[str, str]:
    out = {}
    for item in service.get("environment") or []:
        key, _, val = item.partition("=")
        out[key] = _default(val)
    return out


def test_voice_stack_lives_on_sound_pi_only() -> None:
    sound, vision = _compose(SOUND)["services"], _compose(VISION)["services"]
    for name in MOVED_SERVICES:
        assert name in sound, f"{name} должен быть в docker/sound"
        assert name not in vision, f"{name} остался в docker/vision — две реализации"
    # Vision не зависит от переехавших сервисов: межузловой depends_on compose не выражает
    for name, svc in vision.items():
        deps = svc.get("depends_on") or {}
        assert not set(deps) & set(MOVED_SERVICES), f"vision/{name} ждёт сервис другого Pi"
    assert "sc_synthdefs" in (_compose(SOUND).get("volumes") or {})
    assert "sc_synthdefs" not in (_compose(VISION).get("volumes") or {})


def test_moved_files_left_vision() -> None:
    for rel in ("config/voice_assistant", "config/audio", "config/wake_words.yaml",
                "scripts/voice_assistant", "scripts/supercollider",
                "scripts/supervisor/start_supervisor.sh"):
        assert not (VISION / rel).exists(), f"docker/vision/{rel} вернулся"
        assert (SOUND / rel).exists(), f"docker/sound/{rel} нет"
    # арбитр остался на Vision и берёт только свой скрипт
    assert (VISION / "scripts/supervisor/start_arbiter.sh").exists()
    assert not (SOUND / "scripts/supervisor/start_arbiter.sh").exists()


def test_supervisor_shares_state_dir_with_voice_assistant_on_one_node() -> None:
    """Смысл переезда ТАРС (В8): оба пишут в один /data/harness_voice.db — каталог один и узел один."""
    services = _compose(SOUND)["services"]
    sup, va = services["supervisor"], services["voice-assistant"]
    assert "./data/voice:/data" in sup["volumes"]
    assert "./data/voice:/data" in va["volumes"]
    # лицевую базу ТАРС читает только на чтение; пишет её vision-face на Vision Pi
    assert "./data/faces:/data/faces:ro" in sup["volumes"]
    assert sup["container_name"] == "avatar-supervisor"
    # арбитр остался на Vision и не ходит за файлами голосового стека
    vision = _compose(VISION)["services"]
    assert "avatar-arbiter" in vision and "supervisor" not in vision
    for vol in vision["avatar-arbiter"]["volumes"]:
        assert "data/voice" not in vol and "voice_assistant" not in vol


def test_supervisor_dockerfile_copies_script_from_its_new_place() -> None:
    # Каталог сборки supervisor переехал в docker/sound (ADR-0159 PR-3).
    dockerfile = (SOUND / "supervisor" / "Dockerfile").read_text(encoding="utf-8")
    assert "docker/sound/scripts/supervisor/start_supervisor.sh" in dockerfile
    assert "docker/vision/scripts/supervisor/start_supervisor.sh" not in dockerfile


def test_zenoh_router_of_sound_pi() -> None:
    router = _compose(SOUND)["services"]["zenoh-router"]
    assert router["container_name"] == "zenoh-router-sound"
    assert router["image"] == "eclipse/zenoh:1.6.2"
    assert router["mem_limit"] == "128m"
    env = _env(router)
    assert env["ZENOH_LISTEN_IP"] == "10.1.1.12"
    # вариант Б (ADR-0159 В3): прямой линк на Vision и линк на Main
    assert env["ZENOH_CONNECT_ENDPOINTS"].split(",") == [
        "tcp/10.1.1.10:7447#iface=eth0",
        "tcp/10.1.1.11:7447#iface=eth0",
    ]


def test_ollama_is_addressed_over_the_network() -> None:
    env = _env(_compose(SOUND)["services"]["voice-assistant"])
    assert env["OLLAMA_BASE_URL"] == "http://10.1.1.11:11434"
    assert "localhost" not in env["OLLAMA_BASE_URL"]


def test_sensor_bridge_is_opt_in_and_runs_only_the_bridge_node() -> None:
    svc = _compose(SOUND)["services"]["sensor-bridge"]
    assert svc["profiles"] == ["sensors"], "sensor-bridge не должен стартовать по умолчанию"
    assert svc["mem_limit"] == "128m"
    assert "/dev/sensor_hub:/dev/sensor_hub" in svc["devices"]
    env = _env(svc)
    assert env["SENSOR_UART_PORT"] == "/dev/sensor_hub"
    assert env["SENSOR_UART_BAUD"] == "115200"
    assert "perception-" in svc["image"] and "PERCEPTION_TAG" in svc["image"]
    cmd = svc["command"]
    assert cmd[0] == "/ros_scripts/ros_with_namespace.sh"
    assert "ros2 run rob_box_perception perception_bridge" in cmd[-1]
    # не весь internal_dialogue_docker.launch.py: context_aggregator остаётся на Main Pi
    assert "launch" not in cmd[-1]


def test_perception_image_has_pyserial() -> None:
    reqs = (REPO_ROOT / "docker" / "main" / "perception" / "requirements.txt").read_text(
        encoding="utf-8"
    )
    assert re.search(r"^pyserial\b", reqs, re.M), "без pyserial perception_bridge — в stub-режиме"


def test_every_sound_service_pins_pull_policy() -> None:
    for name, svc in _compose(SOUND)["services"].items():
        assert svc.get("pull_policy") == "if_not_present", name


def test_relative_bind_mount_sources_exist() -> None:
    """Каталоги конфигов/скриптов, которые compose монтирует, обязаны лежать в docker/sound
    (иначе docker молча создаст пустой каталог root'ом и сервис стартует без конфигов)."""
    stateful = ("./data/", "./cache/")  # состояние: в git не лежит, создаёт docker/cutover
    for name, svc in _compose(SOUND)["services"].items():
        for vol in svc.get("volumes") or []:
            src = vol.split(":")[0]
            if not src.startswith("."):
                continue
            if src.startswith(stateful):
                continue
            assert (SOUND / src).resolve().exists(), f"{name}: {src} не существует"


def test_promtail_labels_sound_pi_logs_separately() -> None:
    cfg = (SOUND / "config" / "monitoring" / "promtail-config.yaml").read_text(encoding="utf-8")
    assert "host: sound-pi" in cfg
    assert "vision-pi" not in cfg
    names = {s["container_name"] for s in _compose(SOUND)["services"].values()}
    assert {"cadvisor-sound", "promtail-sound"} <= names


def test_image_versions_files_carry_tags_of_sound_services() -> None:
    for env in ("dev", "test", "latest"):
        text = (SOUND / f".image-versions.{env}").read_text(encoding="utf-8")
        assert re.search(r"^VOICE_ASSISTANT_TAG=\S+$", text, re.M), env
        assert re.search(r"^PERCEPTION_TAG=\S+$", text, re.M), env
        vision = (VISION / f".image-versions.{env}").read_text(encoding="utf-8")
        assert "VOICE_ASSISTANT_TAG" not in vision, f"vision/.image-versions.{env}: дубль тега"
