from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[2] / "docker/vision/scripts/supercollider/start_supercollider.sh"


def test_supercollider_cleans_host_jack_registry_and_db_dirs() -> None:
    script = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "jack-shm-registry" in script
    assert "jack_db-" in script