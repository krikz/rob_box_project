"""Fail-closed проверка пинов .image-versions.<tag> до SSH (issue #5110, #4653).

Деплой с registry_source=local берёт образы по пинам из локального registry.
Отсутствующий или плавающий пин раньше проявлялся только после остановки стека
на Pi (compose pull падает) либо молча — старым образом (тихая деградация).
Замер 23.11.2026: prod-пины `latest-b2e7144` разрешались для 14 сервисов из 16 —
`teleop-humble-latest-b2e7144` и `quest-humble-latest-b2e7144` в registry нет.

Тесты не ходят в настоящий registry: поднимают локальный HTTP-сервер на
127.0.0.1 со своим набором манифестов и передают его адрес через
ROBBOX_REGISTRY_HOST. Живой реестр — только источник фактов, не зависимость.
"""

import http.server
import os
import shutil
import subprocess
import threading
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "ci" / "verify_prod_local_tags.sh"
WORKFLOW = ROOT / ".github" / "workflows" / "L-Deploy and Verify.yml"

# Реальные пины staging 23.11.2026 (Main Pi, чекаут 369a33f05): все 16 = dev-04556b7.
STAGING_PINS = {
    "main": {
        "TWIST_MUX_TAG": "dev-04556b7",
        "ROBOT_STATE_PUBLISHER_TAG": "dev-04556b7",
        "RTABMAP_TAG": "dev-04556b7",
        "ROS2_CONTROL_TAG": "dev-04556b7",
        "LSLIDAR_TAG": "dev-04556b7",
        "PERCEPTION_TAG": "dev-04556b7",
        "NAV2_TAG": "dev-04556b7",
        "TELEOP_TAG": "dev-04556b7",
    },
    "vision": {
        "OAK_D_TAG": "dev-04556b7",
        "LED_MATRIX_TAG": "dev-04556b7",
        "CEILING_CAMERA_TAG": "dev-04556b7",
        "VISION_HAILO_TAG": "dev-04556b7",
    },
    "sound": {
        "VOICE_ASSISTANT_TAG": "dev-04556b7",
        "TELEGRAM_BOT_TAG": "dev-04556b7",
        "SUPERVISOR_TAG": "dev-04556b7",
        "QUEST_TAG": "dev-04556b7",
        "PERCEPTION_TAG": "dev-04556b7",
    },
}

PROD_PINS = {
    component: dict.fromkeys(pins, "latest-b2e7144")
    for component, pins in STAGING_PINS.items()
}
# В registry есть 14 из 16 prod-тегов; у teleop и quest — только плавающие.
PROD_MISSING = {"teleop-humble-latest-b2e7144", "quest-humble-latest-b2e7144"}


def _image_name(pin_var: str, ros_distro: str = "humble") -> str:
    return f"{pin_var[: -len('_TAG')].lower().replace('_', '-')}-{ros_distro}"


def _all_images(pins: dict, tag: str) -> set:
    return {f"{_image_name(var)}-{tag}" for group in pins.values() for var in group}


class _RegistryHandler(http.server.BaseHTTPRequestHandler):
    manifests: set = set()

    def do_GET(self):  # noqa: N802 — API http.server
        if self.path == "/v2/":
            code = 200
        elif self.path.startswith("/v2/krikz/rob_box/manifests/"):
            code = 200 if self.path.rsplit("/", 1)[-1] in self.manifests else 404
        else:
            code = 404
        self.send_response(code)
        self.send_header("Content-Length", "0")
        self.end_headers()

    def log_message(self, *args):  # тишина в pytest output
        pass


@pytest.fixture
def registry(monkeypatch):
    """Локальный fake registry: registry({tag1, tag2, ...}) → адрес host:port."""
    servers = []

    def start(manifests):
        handler = type("Handler", (_RegistryHandler,), {"manifests": set(manifests)})
        srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        servers.append(srv)
        return f"127.0.0.1:{srv.server_address[1]}"

    yield start
    for srv in servers:
        srv.shutdown()


def _repo_copy(tmp_path: Path, pins: dict, tag: str) -> Path:
    """Копия нужных частей репо в tmp: скрипт + docker/<group>/.image-versions.<tag>."""
    root = tmp_path / "repo"
    (root / "scripts" / "ci").mkdir(parents=True)
    shutil.copy(SCRIPT, root / "scripts" / "ci" / "verify_prod_local_tags.sh")
    shutil.copy(
        ROOT / "scripts" / "ci" / "verify-image-in-registry.sh", root / "scripts" / "ci"
    )
    for group, group_pins in pins.items():
        (root / "docker" / group).mkdir(parents=True)
        (root / "docker" / group / f".image-versions.{tag}").write_text(
            "# тестовые пины\n" + "".join(f"{k}={v}\n" for k, v in group_pins.items()),
            encoding="utf-8",
        )
    return root


def _run(root: Path, tag: str, registry_addr: str):
    env = dict(os.environ)
    env["ROBBOX_REGISTRY_HOST"] = registry_addr
    return subprocess.run(
        ["bash", str(root / "scripts" / "ci" / "verify_prod_local_tags.sh"), tag],
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
    )


def test_all_pins_present_passes(tmp_path, registry):
    """Staging 23.11.2026: 16/16 пинов dev-04556b7 есть в registry → rc 0."""
    root = _repo_copy(tmp_path, STAGING_PINS, "dev")
    addr = registry(_all_images(STAGING_PINS, "dev-04556b7"))

    proc = _run(root, "dev", addr)

    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "все 16 пинов" in proc.stdout
    assert "::error" not in proc.stdout


def test_missing_pin_fails_closed_before_any_ssh(tmp_path, registry):
    """Prod 23.11.2026: teleop/quest latest-b2e7144 в registry нет →
    ::error с именем образа и ненулевой rc (деплой не доходит до SSH)."""
    root = _repo_copy(tmp_path, PROD_PINS, "latest")
    present = _all_images(PROD_PINS, "latest-b2e7144") - PROD_MISSING
    addr = registry(present)

    proc = _run(root, "latest", addr)

    assert proc.returncode == 1, proc.stdout + proc.stderr
    for image in sorted(PROD_MISSING):
        assert "::error" in proc.stdout and image in proc.stdout
    assert "отсутствующих в registry 2" in proc.stdout
    # Остальные 14 прошли — падение не валит всё подряд.
    assert "✅ teleop" not in proc.stdout


def test_floating_pin_is_rejected(tmp_path, registry):
    """Пин `latest` вместо `latest-<sha>` — плавающий тег: ::error, rc 1,
    даже если такой образ в registry есть."""
    pins = {"main": {"TELEOP_TAG": "latest"}}
    root = _repo_copy(tmp_path, pins, "latest")
    addr = registry({"teleop-humble-latest"})

    proc = _run(root, "latest", addr)

    assert proc.returncode == 1, proc.stdout + proc.stderr
    assert "плавающий" in proc.stdout
    assert "TELEOP_TAG=latest" in proc.stdout


def test_unreachable_registry_fails_closed(tmp_path):
    """Registry недоступен → честный FAIL, а не «пины ок»."""
    root = _repo_copy(tmp_path, STAGING_PINS, "dev")

    proc = _run(root, "dev", "127.0.0.1:1")

    assert proc.returncode == 1, proc.stdout + proc.stderr
    assert "registry http://127.0.0.1:1 недоступен" in proc.stdout


def test_missing_pin_file_for_tag_is_an_error(tmp_path, registry):
    """Нет docker/*/.image-versions.<tag> — проверять нечего, rc 2."""
    root = _repo_copy(tmp_path, STAGING_PINS, "dev")
    addr = registry(_all_images(STAGING_PINS, "dev-04556b7"))

    proc = _run(root, "latest", addr)

    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert ".image-versions.latest" in proc.stdout


def test_image_name_rule_matches_compose():
    """Правило `<VAR>_TAG → <var-with-dashes>-humble` — не догадка: каждый пин
    из настоящих .image-versions.* обязан встречаться в compose своего компонента
    именно как `image: …:<имя>-${ROS_DISTRO:-humble}-${VAR:-…}`."""
    checked = 0
    for pin_file in sorted((ROOT / "docker").glob("*/.image-versions.*")):
        compose = pin_file.parent / "docker-compose.yaml"
        if not compose.exists():
            continue
        compose_text = compose.read_text(encoding="utf-8")
        for line in pin_file.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            var = line.split("=", 1)[0]
            if not var.endswith("_TAG"):
                continue
            base = var[: -len("_TAG")].lower().replace("_", "-")
            needle = "}:" + base + "-${ROS_DISTRO:-humble}-${" + var + ":-"
            assert needle in compose_text, (
                f"{pin_file}: {var} → {base}, но в {compose} нет строки образа "
                f"с этой парой (правило имени разошлось с compose)"
            )
            checked += 1
    assert checked >= 16, f"проверено только {checked} пинов — файлы пинов не найдены?"


def test_workflow_runs_preflight_before_ssh_and_only_for_local():
    """Шаг preflight стоит до первого SSH-шага и запускается только при
    registry_source=local (github/skip пины локального registry не касаются)."""
    data = yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))
    steps = [s for job in data["jobs"].values() for s in job.get("steps", [])]
    names = [s.get("name", "") for s in steps]
    idx = [
        i
        for i, name in enumerate(names)
        if "verify_prod_local_tags.sh" in str(steps[i].get("run", ""))
    ]
    assert idx, "в workflow нет шага с scripts/ci/verify_prod_local_tags.sh"
    preflight = steps[idx[0]]
    assert "#5110" in preflight["name"]
    assert "local" in str(preflight["if"])
    assert "dry_run" in str(preflight["if"])
    first_ssh = next(
        i for i, name in enumerate(names) if name == "Verify SSH Tools and Connectivity"
    )
    assert idx[0] < first_ssh, "preflight обязан идти ДО первого SSH-шага"


def test_script_is_executable():
    assert os.access(SCRIPT, os.X_OK), "скрипт должен быть исполняемым (chmod +x)"
