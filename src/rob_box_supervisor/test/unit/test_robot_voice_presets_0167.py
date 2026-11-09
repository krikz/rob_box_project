"""Каталог пресетов стиля робота - в его профиле, хосты получают путь из compose (ADR-0167 Ш1а-2).

У ядра (``rob_box_core.bridge_protocol``) своего каталога нет: без env ``ROB_BOX_VOICE_PRESETS_YAML`` белый список
пуст. Робот кладёт ``voice_presets.yaml`` и промпты ``voice_presets/*.txt`` рядом с ``assistant.yaml``
(``docker/sound/config`` → ``/config``), а compose даёт путь тем, кто его читает: ``quest`` (``ws_server``) и
``supervisor`` (белый список + ``grip_pipeline``). Тест держит то, что раньше держали проверки «запасного списка» ядра.
"""

from __future__ import annotations

import os
from pathlib import Path

import yaml

from rob_box_core.bridge_protocol import VOICE_PIPELINE_DEFAULT_LANGUAGE, VOICE_PRESETS_ENV, load_voice_lists
from rob_box_supervisor import grip_pipeline
from rob_box_supervisor.supervisor_node import VOICE_LANGUAGES, VOICE_PRESET_IDS

REPO = Path(os.environ.get("ROB_BOX_REPO_ROOT") or Path(__file__).resolve().parents[4])
PROFILE_DIR = REPO / "docker" / "sound" / "config"
CATALOG = PROFILE_DIR / "voice_presets.yaml"
COMPOSE = REPO / "docker" / "sound" / "docker-compose.yaml"
IN_CONTAINER = "/config/voice_presets.yaml"
#: Сервисы, которые читают каталог: мост шлема и супервизор (белый список, пайплайн грипа).
READERS = ("quest", "supervisor")


def _catalog() -> dict:
    return yaml.safe_load(CATALOG.read_text(encoding="utf-8"))


def test_catalog_lives_next_to_the_robot_profile():
    assert (PROFILE_DIR / "assistant.yaml").is_file()
    assert CATALOG.is_file()
    assert not (REPO / "src" / "rob_box_voice" / "config" / "voice_presets.yaml").exists()
    assert not (REPO / "src" / "rob_box_voice" / "config" / "presets").exists()


def test_robot_whitelist_is_the_catalog():
    assert (VOICE_PRESET_IDS, VOICE_LANGUAGES) == load_voice_lists(str(CATALOG))


def test_robot_catalog_content():
    assert len(VOICE_PRESET_IDS) >= 6 and len(VOICE_PRESET_IDS) == len(set(VOICE_PRESET_IDS))
    assert "translate" in VOICE_PRESET_IDS  # нейтральный пресет грипа: только перевод
    assert len(VOICE_LANGUAGES) >= 4
    assert VOICE_PIPELINE_DEFAULT_LANGUAGE in VOICE_LANGUAGES
    data = _catalog()
    assert data["default_preset"] in VOICE_PRESET_IDS
    assert data["default_language"] in VOICE_LANGUAGES


def test_every_preset_prompt_is_next_to_the_catalog():
    for key, cfg in _catalog()["presets"].items():
        prompt = CATALOG.parent / cfg["prompt_file"]
        assert prompt.is_file(), f"{key}: нет {prompt}"


def test_grip_pipeline_reads_the_same_catalog_with_prompts():
    loaded = grip_pipeline.load_voice_presets()
    assert Path(loaded["_path"]).resolve() == CATALOG.resolve()
    assert tuple(loaded["presets"]) == VOICE_PRESET_IDS
    assert all(p["prompt_text"].strip() for p in loaded["presets"].values())


def test_compose_gives_the_catalog_path_to_its_readers():
    services = yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))["services"]
    for name in READERS:
        service = services[name]
        assert f"{VOICE_PRESETS_ENV}={IN_CONTAINER}" in service["environment"], name
        assert "./config:/config:ro" in service["volumes"], name
