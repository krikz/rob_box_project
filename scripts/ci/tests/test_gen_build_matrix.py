"""Юнит-тесты для scripts/ci/gen_build_matrix.py (план §4.3).

scripts/ci/gen_build_matrix.py — читатель docker/build-manifest.yaml,
Phase 1 плана docs/plans/2026-09-15-service-manifest.md. Эти тесты
проверяют только библиотечные функции (load_manifest, topo_layers,
wave_entries, image_versions_map) на СИНТЕТИЧЕСКИХ
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

    assert gen_build_matrix.topo_layers(services) == [["alone"]]


# --- 2. цепочка voice-base → voice-assistant → supervisor = три волны ------


def test_chain_becomes_one_wave_per_level(tmp_path):
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

    # Волна k ждёт волну k-1 через needs: — supervisor не соберётся раньше,
    # чем voice-assistant допушит свежий образ (гонка 09.09.2026).
    assert gen_build_matrix.topo_layers(services) == [
        ["voice-base"],
        ["voice-assistant"],
        ["supervisor"],
    ]


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
def test_real_manifest_parses_and_splits_into_waves():
    manifest = gen_build_matrix.load_manifest(REAL_MANIFEST)
    vision = gen_build_matrix.services_for_pi(manifest, "vision")
    sound = gen_build_matrix.services_for_pi(manifest, "sound")
    main = gen_build_matrix.services_for_pi(manifest, "main")

    # ADR-0159: голосовой стек и supervisor (наследует voice-assistant)
    # собираются для Sound Pi — вся цепочка FROM живёт в одном Pi.
    assert sorted(sound) == ["supercollider", "supervisor", "voice-assistant", "voice-base"]
    assert len(vision) == 6, f"expected 6 Vision Pi services, got {sorted(vision)}"
    assert len(main) == 8, f"expected 8 Main Pi services, got {sorted(main)}"

    assert gen_build_matrix.topo_layers(sound) == [
        ["supercollider", "voice-base"],
        ["voice-assistant"],
        ["supervisor"],
    ]
    # У Vision и Main графы плоские — одна волна.
    assert len(gen_build_matrix.topo_layers(vision)) == 1
    assert len(gen_build_matrix.topo_layers(main)) == 1
    assert gen_build_matrix.max_wave_count(manifest) == 3


# --- зависимость между Pi запрещена (ADR-0159 PR-3) ------------------------
#
# "L: Build All Services" собирает Pi параллельно: ребро в чужой Pi означало
# бы сборку FROM старого образа. load_manifest отказывает сразу.


_CROSS_PI_MANIFEST = """\
version: 1
pis:
  vision:
    services:
      child:
        dockerfile: docker/vision/child/Dockerfile
        build_context: "."
        base:
          service: parent
        depends_on: [parent]
        tag:
          ros_distro: true
        image_versions: false
  sound:
    services:
      parent:
        dockerfile: docker/sound/parent/Dockerfile
        build_context: "."
        base:
          family: ros2-zenoh
        tag:
          ros_distro: true
        image_versions: false
"""


def test_dependency_on_service_of_another_pi_is_rejected(tmp_path):
    p = tmp_path / "m.yaml"
    p.write_text(_CROSS_PI_MANIFEST, encoding="utf-8")
    with pytest.raises(gen_build_matrix.ManifestError, match="pis.vision"):
        gen_build_matrix.load_manifest(p)


def test_same_service_name_in_two_pis_is_rejected(tmp_path):
    p = tmp_path / "m.yaml"
    p.write_text(
        _CROSS_PI_MANIFEST.replace("[parent]", "[]")
        .replace("service: parent", "family: ros2-zenoh")
        .replace("      parent:", "      child:"),
        encoding="utf-8",
    )
    with pytest.raises(gen_build_matrix.ManifestError, match="уже занято"):
        gen_build_matrix.load_manifest(p)


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
    # --pi ограничен choices=["vision", "sound", "main"] (план §4.2 — CLI работает
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
        ["--manifest", str(manifest_path), "--pi", "vision", "--mode", "waves"]
    )
    assert rc == 1
    assert "image_versions" in capsys.readouterr().err


# =========================================================================
# Phase 2/3 (план §5): разрешение манифеста в параметры сборки.
#
# Эти тесты закрывают то, что до Phase 2 проверялось сверкой с текстом
# workflow: формулу тега, состав build-args, спецификацию source-hash.
# После Phase 2 в workflow этих строк больше нет — они рождаются здесь,
# значит и проверять их надо здесь.
# =========================================================================


def _real(pi: str, docker_tag: str = "dev"):
    manifest = gen_build_matrix.load_manifest(REAL_MANIFEST)
    services = gen_build_matrix.services_for_pi(manifest, pi)
    ctx = gen_build_matrix.build_context(manifest, docker_tag=docker_tag)
    return services, ctx


# --- волны реального манифеста -------------------------------------------


def test_waves_resolve_build_params_in_topological_order():
    """wave_entries — те же элементы matrix_entry, разложенные по волнам."""
    services, ctx = _real("sound")
    waves = gen_build_matrix.wave_entries(services, ctx)
    assert [[e["name"] for e in w] for w in waves] == gen_build_matrix.topo_layers(services)
    assert waves[1][0] == gen_build_matrix.matrix_entry("voice-assistant", services, ctx)


# --- формула тега ----------------------------------------------------------


def test_tag_formula_includes_ros_distro_by_default():
    services, ctx = _real("vision")
    entry = gen_build_matrix.matrix_entry("oak-d", services, ctx)
    assert entry["tags"].splitlines() == [
        "ghcr.io/krikz/rob_box:oak-d-humble-dev",
        "localhost:5000/krikz/rob_box:oak-d-humble-dev",
    ]


def test_tag_formula_omits_ros_distro_for_supercollider():
    """Единственный сервис с tag.ros_distro: false ("tag naming is
    inconsistent"). Манифест фиксирует факт, генератор его воспроизводит.
    """
    services, ctx = _real("sound")
    entry = gen_build_matrix.matrix_entry("supercollider", services, ctx)
    assert entry["tags"].splitlines() == [
        "ghcr.io/krikz/rob_box:supercollider-dev",
        "localhost:5000/krikz/rob_box:supercollider-dev",
    ]
    assert "humble" not in entry["tags"]


# --- build-args ------------------------------------------------------------


def test_base_family_resolves_to_base_registry():
    services, ctx = _real("main")
    entry = gen_build_matrix.matrix_entry("lslidar", services, ctx)
    assert (
        "BASE_IMAGE=localhost:5000/krikz/rob_box_base:pcl-humble"
        in entry["build_args"].splitlines()
    )


def test_base_service_resolves_to_local_prefix_tag_of_that_service():
    """supervisor наследует ОБРАЗ voice-assistant из локального реестра —
    именно поэтому у него есть и base.service, и depends_on (план §2.3).
    """
    services, ctx = _real("sound")
    entry = gen_build_matrix.matrix_entry("supervisor", services, ctx)
    assert (
        "BASE_IMAGE=localhost:5000/krikz/rob_box:voice-assistant-humble-dev"
        in entry["build_args"].splitlines()
    )
    assert entry["depends_on"] == "voice-assistant"


def test_service_without_base_gets_no_base_image_arg():
    services, ctx = _real("sound")
    # voice-resources тут был вторым примером сервиса без base; узел удалён
    # вместе с образом (сэмплы едут Ресурсным паком на хост).
    for name in ("supercollider",):
        entry = gen_build_matrix.matrix_entry(name, services, ctx)
        assert entry["build_args"] == "APT_PROXY=http://host.docker.internal:3142", (
            f"{name}: ожидался только APT_PROXY, получено {entry['build_args']!r}"
        )


def test_extra_build_args_are_appended():
    services, ctx = _real("vision")
    entry = gen_build_matrix.matrix_entry("vision-hailo", services, ctx)
    assert "HAILO_INSTALL_BINDING=whl" in entry["build_args"].splitlines()
    assert entry["pre_build"] == "fetch_hailort_wheel"


def test_submodule_sha_is_passed_through():
    services, ctx = _real("main")
    assert (
        gen_build_matrix.matrix_entry("ros2-control", services, ctx)["submodule_sha"]
        == "src/vesc_nexus"
    )
    services, ctx = _real("vision")
    assert (
        gen_build_matrix.matrix_entry("led-matrix", services, ctx)["submodule_sha"]
        == "src/ros2leds"
    )


# --- source_hash spec ------------------------------------------------------


def test_source_hash_spec_converts_extensions_to_find_patterns():
    """Расширение ".py" превращается в шаблон "*.py", а имена файлов
    ("package.xml", "CMakeLists.txt") остаются литералами — ровно так, как
    было написано руками в find-выражениях прежних job'ов.
    """
    services, ctx = _real("main")
    entry = gen_build_matrix.matrix_entry("robot-state-publisher", services, ctx)
    assert entry["source_hash_arg"] == "URDF_FILES_HASH"  # НЕ SOURCE_HASH, см. §2.4
    assert entry["source_hash_spec"] == (
        "src/rob_box_description\t*.xacro *.urdf package.xml CMakeLists.txt"
    )


def test_source_hash_spec_keeps_group_order_for_multi_group_services():
    services, ctx = _real("vision")
    entry = gen_build_matrix.matrix_entry("quest", services, ctx)
    assert entry["source_hash_spec"].splitlines() == [
        "src/rob_box_quest/rob_box_quest\t*.py",
        "src/rob_box_quest/webxr_client\t*.ts *.json *.html",
        "src/rob_box_core\t*.py",
    ]


def test_services_without_source_hash_get_empty_spec():
    services, ctx = _real("vision")
    entry = gen_build_matrix.matrix_entry("oak-d", services, ctx)
    assert entry["source_hash_arg"] == ""
    assert entry["source_hash_spec"] == ""


def test_every_matrix_entry_has_the_same_keys():
    """Шаблон job'а один на все сервисы, поэтому набор ключей обязан
    совпадать — иначе `matrix.<key>` у части сервисов молча даст пустоту.
    """
    for pi in ("vision", "sound", "main"):
        services, ctx = _real(pi)
        entries = [e for w in gen_build_matrix.wave_entries(services, ctx) for e in w]
        keys = {frozenset(e) for e in entries}
        assert len(keys) == 1, f"{pi}: элементы matrix с разным набором ключей: {keys}"


# --- CLI Phase 2/3 ---------------------------------------------------------


def test_cli_waves_requires_docker_tag(capsys):
    rc = gen_build_matrix.main(
        ["--manifest", str(REAL_MANIFEST), "--pi", "vision", "--mode", "waves"]
    )
    assert rc == 1
    assert "--docker-tag" in capsys.readouterr().err


def test_cli_waves_emits_single_line_json(capsys):
    rc = gen_build_matrix.main(
        [
            "--manifest", str(REAL_MANIFEST),
            "--pi", "sound",
            "--mode", "waves",
            "--docker-tag", "test",
        ]
    )
    assert rc == 0
    out = capsys.readouterr().out.strip()
    # В $GITHUB_OUTPUT однострочная форма `key=value` не переживёт перевод
    # строки — JSON обязан быть в одну строку.
    assert "\n" not in out
    waves = json.loads(out)
    manifest = gen_build_matrix.load_manifest(REAL_MANIFEST)
    services = gen_build_matrix.services_for_pi(manifest, "sound")
    assert [[e["name"] for e in w] for w in waves] == gen_build_matrix.topo_layers(services)


def test_cli_old_matrix_and_chained_modes_are_gone(capsys):
    """Режимы named-job'ов удалены вместе с per-Pi workflow (ADR-0159 PR-3)."""
    for mode in ("matrix", "chained"):
        with pytest.raises(SystemExit):
            gen_build_matrix.main(
                ["--manifest", str(REAL_MANIFEST), "--pi", "sound", "--mode", mode,
                 "--docker-tag", "dev"]
            )
    capsys.readouterr()


def test_cli_tags_shell_format_prints_var_and_tag(capsys):
    rc = gen_build_matrix.main(
        [
            "--manifest", str(REAL_MANIFEST),
            "--pi", "main",
            "--mode", "tags",
            "--format", "shell",
            "--docker-tag", "dev",
        ]
    )
    assert rc == 0
    lines = capsys.readouterr().out.strip().splitlines()
    assert "NAV2_TAG nav2-humble-dev" in lines
    # 8 сервисов Main Pi минус teleop (image_versions: false).
    assert len(lines) == 7
    # phantom-поле MICRO_ROS_AGENT_TAG не принадлежит ни одному сервису —
    # генератор его не печатает, значит Phase 3 перестаёт его обновлять.
    assert not any(line.startswith("MICRO_ROS_AGENT_TAG") for line in lines)


def test_cli_services_shell_format(capsys):
    rc = gen_build_matrix.main(
        [
            "--manifest", str(REAL_MANIFEST),
            "--pi", "vision",
            "--mode", "services",
            "--format", "shell",
        ]
    )
    assert rc == 0
    manifest = gen_build_matrix.load_manifest(REAL_MANIFEST)
    assert capsys.readouterr().out.split() == sorted(
        gen_build_matrix.services_for_pi(manifest, "vision")
    )
