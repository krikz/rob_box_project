"""Юнит-тесты для scripts/ci/gen_build_matrix.py (план §4.3).

scripts/ci/gen_build_matrix.py — читатель docker/build-manifest.yaml,
Phase 1 плана docs/plans/2026-09-15-service-manifest.md. Эти тесты
проверяют только библиотечные функции (load_manifest, independent_services,
chained_services, topo_layers, image_versions_map) на СИНТЕТИЧЕСКИХ
мини-манифестах — сверка с РЕАЛЬНЫМИ workflow-файлами делается отдельным
guard-тестом (scripts/ci/tests/test_service_manifest_sync.py).

scripts/ci/ — не пакет (нет __init__.py, в отличие от scripts/lint/),
поэтому модуль импортируется напрямую по пути, как это уже делается для
других standalone-скриптов репозитория (см. tests/unit/scripts/test_tars_stats.py,
scripts/testing/test_patch_rtabmap_launch.py).
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "ci" / "gen_build_matrix.py"
REAL_MANIFEST = REPO_ROOT / "docker" / "build-manifest.yaml"

_SPEC = importlib.util.spec_from_file_location(
    "gen_build_matrix_under_test", SCRIPT_PATH
)
assert _SPEC is not None and _SPEC.loader is not None, f"{SCRIPT_PATH}: spec failed"
gen_build_matrix = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gen_build_matrix)  # type: ignore[union-attr]


# --- helpers -------------------------------------------------------------


def _write_manifest(tmp_path: Path, services_yaml: str, pi: str = "test") -> Path:
    """Оборачивает фрагмент `services:` в валидный по форме манифест."""
    text = f"version: 1\npis:\n  {pi}:\n    services:\n{services_yaml}"
    p = tmp_path / "build-manifest.yaml"
    p.write_text(text, encoding="utf-8")
    return p


# --- 1. один независимый сервис ------------------------------------------


def test_single_independent_service(tmp_path):
    manifest_path = _write_manifest(
        tmp_path,
        """\
      alone:
        dockerfile: docker/vision/alone/Dockerfile
        build_context: "."
        base: null
        tag:
          ros_distro: true
        image_versions: false
""",
    )
    manifest = gen_build_matrix.load_manifest(manifest_path)
    services = gen_build_matrix.services_for_pi(manifest, "test")

    independent = gen_build_matrix.independent_services(services)
    chained = gen_build_matrix.chained_services(services)

    assert [s["name"] for s in independent] == ["alone"]
    assert chained == []


# --- 2. двухуровневая цепочка voice-base → voice-assistant → supervisor --


def test_chained_services_topological_order(tmp_path):
    manifest_path = _write_manifest(
        tmp_path,
        """\
      voice-base:
        dockerfile: docker/vision/voice_base/Dockerfile
        build_context: docker/vision/voice_base
        base:
          family: rtabmap
        tag:
          ros_distro: true
        image_versions: false

      voice-assistant:
        dockerfile: docker/vision/voice_assistant/Dockerfile
        build_context: "."
        base:
          service: voice-base
        depends_on: [voice-base]
        tag:
          ros_distro: true
        image_versions: VOICE_ASSISTANT_TAG

      supervisor:
        dockerfile: docker/vision/supervisor/Dockerfile
        build_context: "."
        base:
          service: voice-assistant
        depends_on: [voice-assistant]
        tag:
          ros_distro: true
        image_versions: false
""",
    )
    manifest = gen_build_matrix.load_manifest(manifest_path)
    services = gen_build_matrix.services_for_pi(manifest, "test")

    independent = gen_build_matrix.independent_services(services)
    chained = gen_build_matrix.chained_services(services)

    assert [s["name"] for s in independent] == ["voice-base"]
    assert [s["name"] for s in chained] == ["voice-assistant", "supervisor"]
    # chained_services обязан сохранить depends_on в каждой записи — это то,
    # что потребитель (workflow-шаблон) использует для needs:.
    assert chained[0]["depends_on"] == ["voice-base"]
    assert chained[1]["depends_on"] == ["voice-assistant"]


# --- 3. цикл в depends_on → понятная ошибка, не RecursionError -----------


def test_dependency_cycle_raises_manifest_error(tmp_path):
    manifest_path = _write_manifest(
        tmp_path,
        """\
      a:
        dockerfile: docker/vision/a/Dockerfile
        build_context: "."
        base: null
        depends_on: [b]
        tag:
          ros_distro: true
        image_versions: false

      b:
        dockerfile: docker/vision/b/Dockerfile
        build_context: "."
        base: null
        depends_on: [a]
        tag:
          ros_distro: true
        image_versions: false
""",
    )
    with pytest.raises(gen_build_matrix.ManifestError, match="cycle"):
        gen_build_matrix.load_manifest(manifest_path)


def test_topo_layers_raises_on_cycle_directly():
    services = {
        "a": {"depends_on": ["b"]},
        "b": {"depends_on": ["a"]},
    }
    with pytest.raises(gen_build_matrix.ManifestError, match="cycle"):
        gen_build_matrix.topo_layers(services)


# --- 4. base.service без соответствующего depends_on ----------------------


def test_base_service_without_matching_depends_on_raises(tmp_path):
    manifest_path = _write_manifest(
        tmp_path,
        """\
      voice-base:
        dockerfile: docker/vision/voice_base/Dockerfile
        build_context: docker/vision/voice_base
        base:
          family: rtabmap
        tag:
          ros_distro: true
        image_versions: false

      voice-assistant:
        dockerfile: docker/vision/voice_assistant/Dockerfile
        build_context: "."
        base:
          service: voice-base
        depends_on: []
        tag:
          ros_distro: true
        image_versions: false
""",
    )
    with pytest.raises(gen_build_matrix.ManifestError) as exc_info:
        gen_build_matrix.load_manifest(manifest_path)
    message = str(exc_info.value)
    # Ошибка обязана называть оба поля и оба значения (план §4.3) — иначе
    # читателю придётся угадывать, что именно разошлось.
    assert "voice-assistant" in message
    assert "base.service" in message
    assert "voice-base" in message
    assert "depends_on" in message


# --- 5. невалидный идентификатор image_versions ---------------------------


def test_invalid_image_versions_identifier_raises(tmp_path):
    manifest_path = _write_manifest(
        tmp_path,
        """\
      broken:
        dockerfile: docker/vision/broken/Dockerfile
        build_context: "."
        base: null
        tag:
          ros_distro: true
        image_versions: "OAK D TAG"
""",
    )
    with pytest.raises(gen_build_matrix.ManifestError, match="image_versions"):
        gen_build_matrix.load_manifest(manifest_path)


# --- image_versions_map ----------------------------------------------------


def test_image_versions_map_skips_false():
    services = {
        "oak-d": {"image_versions": "OAK_D_TAG"},
        "supercollider": {"image_versions": False},
    }
    assert gen_build_matrix.image_versions_map(services) == {"OAK_D_TAG": "oak-d"}


# --- 6. smoke-тест на реальном манифесте -----------------------------------


@pytest.mark.skipif(
    not REAL_MANIFEST.exists(), reason="docker/build-manifest.yaml ещё не создан"
)
def test_real_manifest_parses_and_has_19_services():
    manifest = gen_build_matrix.load_manifest(REAL_MANIFEST)
    vision = gen_build_matrix.services_for_pi(manifest, "vision")
    main = gen_build_matrix.services_for_pi(manifest, "main")

    assert len(vision) == 11, f"expected 11 Vision Pi services, got {sorted(vision)}"
    assert len(main) == 8, f"expected 8 Main Pi services, got {sorted(main)}"

    # Двухуровневая цепочка должна разрешиться без ошибок и в правильном
    # порядке — это же покрытие интеграционно, не только синтетически.
    chained = gen_build_matrix.chained_services(vision)
    assert [s["name"] for s in chained] == ["voice-assistant", "supervisor"]


# --- CLI smoke --------------------------------------------------------------


def test_cli_mode_tags_prints_json(tmp_path, capsys):
    manifest_path = _write_manifest(
        tmp_path,
        """\
      oak-d:
        dockerfile: docker/vision/oak-d/Dockerfile
        build_context: docker/vision
        base:
          family: depthai
        tag:
          ros_distro: true
        image_versions: OAK_D_TAG
""",
        pi="vision",
    )
    # --pi ограничен choices=["vision", "main"] (план §4.2 — CLI работает
    # только над реальными Pi), поэтому синтетический манифест здесь тоже
    # использует ключ "vision", а не произвольное имя "test".
    rc = gen_build_matrix.main(
        ["--manifest", str(manifest_path), "--pi", "vision", "--mode", "tags"]
    )
    assert rc == 0
    out = json.loads(capsys.readouterr().out)
    assert out == {"OAK_D_TAG": "oak-d"}


def test_cli_unknown_manifest_error_exits_nonzero(tmp_path, capsys):
    manifest_path = _write_manifest(
        tmp_path,
        """\
      broken:
        dockerfile: docker/vision/broken/Dockerfile
        build_context: "."
        base: null
        tag:
          ros_distro: true
        image_versions: "not a tag"
""",
        pi="vision",
    )
    rc = gen_build_matrix.main(
        ["--manifest", str(manifest_path), "--pi", "vision", "--mode", "matrix"]
    )
    assert rc == 1
    assert "image_versions" in capsys.readouterr().err
