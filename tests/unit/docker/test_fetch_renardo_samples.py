"""Тесты хук-фетчера Renardo-сэмплов.

Раньше файл лежал рядом с модулем (``docker/vision/voice_assistant/tests/``)
и туда не заглядывал ни один guard-прогон. Модуль переехал в Ресурсный пак
(``docker/vision/scripts/resource_pack/fetch_renardo_samples.py``), тест —
сюда, под ``tests/unit/docker``, который guard-команда запускает.
"""

import importlib.util
import pathlib

import pytest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
MODULE_PATH = (
    REPO_ROOT / "docker" / "vision" / "scripts" / "resource_pack" / "fetch_renardo_samples.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("fetch_renardo_samples", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_module_lives_in_resource_pack() -> None:
    """Фетчер — часть Ресурсного пака, а не образа voice-assistant.

    Если файл уедет обратно в docker/vision/voice_assistant, запись
    ``renardo-samples`` в manifest.yaml и apply_resource_pack.sh
    перестанут его находить — и деплой молча уйдёт в synth-only.
    """
    assert MODULE_PATH.exists(), f"{MODULE_PATH} не найден"


def test_resolve_samples_dir_prefers_cli_target(tmp_path, monkeypatch) -> None:
    module = load_module()
    monkeypatch.setenv(module.SAMPLES_DIR_ENV, str(tmp_path / "from-env"))

    resolved = module.resolve_samples_dir(str(tmp_path / "from-cli"))

    assert resolved == tmp_path / "from-cli"


def test_resolve_samples_dir_falls_back_to_env(tmp_path, monkeypatch) -> None:
    module = load_module()
    monkeypatch.setenv(module.SAMPLES_DIR_ENV, str(tmp_path / "from-env"))

    assert module.resolve_samples_dir(None) == tmp_path / "from-env"


@pytest.mark.parametrize("env_value", ["", "   "])
def test_resolve_samples_dir_ignores_blank_env(monkeypatch, env_value) -> None:
    """Пустой RENARDO_SAMPLES_DIR = «не задано», а не «текущий каталог»."""
    module = load_module()
    monkeypatch.setenv(module.SAMPLES_DIR_ENV, env_value)

    assert module.resolve_samples_dir(None) == pathlib.Path(module.SAMPLES_DIR_PATH)


def test_resolve_samples_dir_default_is_previous_behaviour(monkeypatch) -> None:
    module = load_module()
    monkeypatch.delenv(module.SAMPLES_DIR_ENV, raising=False)

    assert module.resolve_samples_dir(None) == pathlib.Path(module.SAMPLES_DIR_PATH)


def test_marker_makes_default_pack_idempotent(tmp_path) -> None:
    """Маркер downloaded_at.txt в ЦЕЛЕВОМ каталоге, а не в ~/.config."""
    module = load_module()

    assert module.is_default_spack_initialized(tmp_path) is False

    module.write_default_pack_marker(module.Logger(), tmp_path)

    marker = tmp_path / module.DEFAULT_SAMPLES_PACK_NAME / module.MARKER_FILENAME
    assert marker.exists()
    assert module.is_default_spack_initialized(tmp_path) is True


def test_iter_collection_files_preserves_absolute_urls() -> None:
    module = load_module()

    tree = {
        "name": "1_pitchglitch_samples",
        "path": "./samples/1_pitchglitch_samples",
        "children": [
            {
                "name": "g",
                "path": "./samples/1_pitchglitch_samples/g",
                "children": [
                    {
                        "name": "upper",
                        "path": "./samples/1_pitchglitch_samples/g/upper",
                        "children": [
                            {
                                "name": "023_Various_Guitar_PetitBarcelona.wav",
                                "path": "./samples/1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav",
                                "url": "https://collections.renardo.org/samples/1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav",
                            }
                        ],
                    }
                ],
            }
        ],
    }

    entries = list(module.iter_collection_files(tree))

    assert entries == [
        (
            "https://collections.renardo.org/samples/1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav",
            pathlib.Path("1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav"),
        )
    ]


def test_download_collection_retries_failures_sequentially(tmp_path) -> None:
    module = load_module()

    tree = {
        "name": "1_pitchglitch_samples",
        "path": "./samples/1_pitchglitch_samples",
        "children": [
            {
                "name": "g",
                "path": "./samples/1_pitchglitch_samples/g",
                "children": [
                    {
                        "name": "upper",
                        "path": "./samples/1_pitchglitch_samples/g/upper",
                        "children": [
                            {
                                "name": "023_Various_Guitar_PetitBarcelona.wav",
                                "path": "./samples/1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav",
                                "url": "https://collections.renardo.org/samples/1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav",
                            }
                        ],
                    }
                ],
            }
        ],
    }

    attempts = {"count": 0}

    def fake_load_index(_json_url, _logger):
        return tree

    def fake_download_file(*, url, destination, logger, timeout_seconds, retries):
        attempts["count"] += 1
        if attempts["count"] == 1:
            return False
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(b"wav")
        return True

    module.load_json_index = fake_load_index
    module.download_file = fake_download_file

    failures = module.download_collection(
        json_url="https://collections.renardo.org/samples/1_pitchglitch_samples/collection_index.json",
        download_dir=tmp_path,
        logger=module.Logger(),
        max_workers=4,
    )

    assert failures == []
    assert attempts["count"] == 2
    assert (tmp_path / "1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav").read_bytes() == b"wav"
