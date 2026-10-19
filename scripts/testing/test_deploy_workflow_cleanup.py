import re
from pathlib import Path


WORKFLOW_PATH = Path(__file__).resolve().parents[2] / ".github/workflows/L-Deploy and Verify.yml"

# Раньше очистка была двумя шагами «[Vision Pi] / [Main Pi] Cleanup Docker
# Artifacts»; разбор тайминга run 35645507502 слил их в один параллельный
# пост-деплойный шаг. ADR-0159: к ним добавился Sound Pi.
CLEANUP_STEP = "[Vision Pi + Sound Pi + Main Pi] Cleanup Docker Artifacts (parallel)"
NODES = (("Vision Pi", "vision"), ("Sound Pi", "sound"), ("Main Pi", "main"))


def _steps(workflow):
    """Имена шагов и их тексты по порядку (шаг начинается с `      - name:`)."""
    parts = re.split(r"(?m)^      - name: ", workflow)[1:]
    return [(p.splitlines()[0].strip().strip('"'), p) for p in parts]


def test_deploy_workflow_prunes_unused_docker_artifacts_on_all_pis():
    steps = dict(_steps(WORKFLOW_PATH.read_text(encoding="utf-8")))

    assert CLEANUP_STEP in steps, f"шаг очистки не найден: {CLEANUP_STEP}"
    body = steps[CLEANUP_STEP]
    for host in ("env.vision_pi_ip", "env.sound_pi_ip", "env.main_pi_ip"):
        assert host in body, f"очистка не ходит на {host}"
    assert body.count("docker image prune -af") >= 3, "image prune должен идти на всех трёх Pi"
    assert body.count("docker builder prune -af") >= 3, "builder prune должен идти на всех трёх Pi"


def test_deploy_workflow_never_prunes_while_stack_is_stopped():
    """`docker image prune -af` удаляет все образы без контейнеров. Если он
    выполнится между Stop Containers (`compose down`) и Start Containers,
    он снесёт образы стека, которые только что скачал/проверил Pull-шаг
    (#2930), и `up --pull never` упадёт. Исходно очистка стояла ДО Stop,
    сейчас — после Start; инвариант один: не внутри окна Stop..Start."""
    names = [name for name, _ in _steps(WORKFLOW_PATH.read_text(encoding="utf-8"))]
    cleanup = names.index(CLEANUP_STEP)

    for pi, _ in NODES:
        stop = names.index(f"[{pi}] Stop Containers")
        start = names.index(f"[{pi}] Start Containers")
        assert stop < start
        assert not stop < cleanup < start, (
            f"{CLEANUP_STEP} стоит между [{pi}] Stop и Start — prune снесёт образы остановленного стека"
        )


def test_environment_local_is_aliased_to_dev():
    """Ретро 18.08 (issue #1379, t_544e4aa5): environment=local раньше давал
    IMAGE_TAG=local, которого нет ни в local, ни в github registry → deploy fail.
    Теперь должен быть алиас на IMAGE_TAG=dev с явным WARNING-логом."""
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")

    # Внутри case "local)" должна быть строка IMAGE_TAG=dev (а не IMAGE_TAG=local)
    local_block_idx = workflow.index('local)')
    # следующая IMAGE_TAG после этого индекса
    local_block = workflow[local_block_idx:local_block_idx + 400]
    assert "IMAGE_TAG=dev" in local_block, (
        "Ретро #1379: environment=local должен алиаситься на IMAGE_TAG=dev, "
        "а не оставаться IMAGE_TAG=local (голый тег 'local' в registry не "
        "публикуется — compose pull падает)."
    )
    # Должен быть явный warning с упоминанием ретро
    assert "WARNING: environment=local" in workflow
    assert "issue #1379" in workflow


def test_deploy_pull_ignores_compose_pull_policy():
    """Issue #2609 (16.09): docker/vision/docker-compose.yaml ставит
    pull_policy: if_not_present (#2634). С ним голый `docker compose pull`
    пишет «Skipped Image is already present locally» и не обновляет :dev —
    деплой проходил зелёным, а робот оставался на старых образах."""
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    # Issue #2930: сам pull переехал в preflight-скрипт, который оба
    # Pull-шага (Vision Pi и Main Pi) вызывают перед Stop Containers.
    preflight = (Path(__file__).resolve().parents[2] / "scripts/deploy/preflight_pull_images.sh").read_text(encoding="utf-8")
    assert workflow.count("scripts/deploy/preflight_pull_images.sh") >= 3, "ожидались pull-шаги для Vision, Sound и Main Pi"

    pulls = [
        line
        for line in (workflow + "\n" + preflight).splitlines()
        if "docker compose pull" in line and not line.strip().startswith("#") and "echo" not in line
    ]
    assert pulls, "docker compose pull не найден ни в workflow, ни в preflight"
    for line in pulls:
        assert "--policy always" in line, f"pull без --policy always: {line.strip()}"


def test_vision_autostart_pull_ignores_compose_pull_policy():
    setup = (Path(__file__).resolve().parents[2] / "scripts/setup/setup_vision_pi.sh").read_text(encoding="utf-8")
    exec_pre = [line for line in setup.splitlines() if line.startswith("ExecStartPre=") and "compose pull" in line]
    assert exec_pre and all("--policy always" in line for line in exec_pre)


# ── ADR-0159: Sound Pi как третий узел деплоя ───────────────────────────────────

def _compose_dirs():
    root = Path(__file__).resolve().parents[2] / "docker"
    return sorted(d.name for d in root.iterdir() if (d / "docker-compose.yaml").is_file() and d.name in {n for _, n in NODES})


def test_every_node_compose_dir_has_a_deploy_pipeline():
    """Для каждого каталога docker/{main,vision,sound} с compose в workflow есть
    полный набор шагов Update/Pull/Stop/Start/Check Container Status/Logs, и они
    ходят именно в этот каталог (а не копируют чужой по ошибке)."""
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    steps = dict(_steps(workflow))
    assert _compose_dirs() == ["main", "sound", "vision"], "каталог compose узла появился/исчез — обнови NODES"

    for pi, short in NODES:
        for action in ("Update Code", "Pull Docker Images", "Stop Containers", "Start Containers",
                       "Sync System Time", "Check Container Status", "Check Container Logs"):
            assert f"[{pi}] {action}" in steps, f"нет шага [{pi}] {action}"
        for action in ("Pull Docker Images", "Stop Containers", "Start Containers"):
            assert f"docker/{short}" in steps[f"[{pi}] {action}"], f"[{pi}] {action} не ходит в docker/{short}"
        assert f"docker/{short}" in steps[f"[{pi}] Check Container Status"]
        assert f"SCOPE={short}" in steps[f"[{pi}] Check Container Status"]
        assert f"SCOPE={short}" in steps[f"[{pi}] Check Container Logs"]
        assert "sound_pi_ip=10.1.1.12" in workflow


def test_node_order_is_vision_then_sound_then_main():
    """ADR-0159 §3.4: Vision → Sound → Main."""
    names = [name for name, _ in _steps(WORKFLOW_PATH.read_text(encoding="utf-8"))]
    starts = [names.index(f"[{pi}] Start Containers") for pi, _ in NODES]
    assert starts == sorted(starts)


def test_sound_pi_ip_is_set_in_every_environment():
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    assert workflow.count('echo "sound_pi_ip=10.1.1.12" >> $GITHUB_ENV') == workflow.count(
        'echo "vision_pi_ip=10.1.1.11" >> $GITHUB_ENV'
    ) == 4


def test_voice_resource_pack_goes_to_sound_and_hef_stays_on_vision():
    """Модели STT/TTS, сэмплы и партитуры — для voice-assistant/supercollider (Sound Pi);
    Hailo HEF — PCIe-карта на Vision Pi (ADR-0159 §2.5)."""
    steps = dict(_steps(WORKFLOW_PATH.read_text(encoding="utf-8")))
    stt = steps["[Sound Pi] Ensure STT/TTS models (Ресурсный пак, Этап 3)"]
    samples = steps["[Sound Pi] Ensure Renardo samples (Ресурсный пак, хук)"]
    for body in (stt, samples):
        assert "env.sound_pi_ip" in body and "vision_pi_ip" not in body
    assert "--only vosk-ru-small,silero-tts-v5-ru" in stt
    assert "score-library" in samples
    assert not [n for n in steps if n.startswith("[Vision Pi]") and ("STT/TTS" in n or "Renardo" in n)]
    hef = next(body for name, body in steps.items() if name.startswith("[Vision Pi] Ensure HEF"))
    assert "env.vision_pi_ip" in hef and "yolov8n-hef,retinaface-hef,arcface-hef" in hef


def test_sound_pi_checks_host_secrets_before_stopping_the_stack():
    names = [name for name, _ in _steps(WORKFLOW_PATH.read_text(encoding="utf-8"))]
    secrets = next(i for i, n in enumerate(names) if n.startswith("[Sound Pi] Check .env.secrets"))
    assert secrets < names.index("[Sound Pi] Stop Containers")
    assert names.index("[Sound Pi] Update Code") < secrets
    body = dict(_steps(WORKFLOW_PATH.read_text(encoding="utf-8")))[names[secrets]]
    assert "exit 1" in body and "docker/sound/.env.secrets" in body


def test_sound_expected_containers_match_compose_services_without_profile():
    """Список EXPECTED_CONTAINERS в шаге статуса не должен разойтись с compose."""
    import yaml

    compose = yaml.safe_load((Path(__file__).resolve().parents[2] / "docker/sound/docker-compose.yaml").read_text(encoding="utf-8"))
    always_on = sorted(s["container_name"] for s in compose["services"].values() if not s.get("profiles"))
    body = dict(_steps(WORKFLOW_PATH.read_text(encoding="utf-8")))["[Sound Pi] Check Container Status"]
    m = re.search(r'EXPECTED_CONTAINERS="([^"]*)"', body)
    assert m, "EXPECTED_CONTAINERS не задан для Sound Pi"
    assert sorted(m.group(1).split()) == always_on


def test_no_systemd_unit_reinstall_for_sound():
    """ADR-0159 §2.5: «Ensure systemd unit» (reinstall_vision_pi_systemd.sh) на Sound не копируем."""
    names = [name for name, _ in _steps(WORKFLOW_PATH.read_text(encoding="utf-8"))]
    assert not [n for n in names if n.startswith("[Sound Pi]") and "Systemd" in n]


def test_vision_pull_still_covers_supervisor_image_used_by_avatar_arbiter():
    """avatar-arbiter остаётся на Vision Pi, но его образ — supervisor, который
    после ADR-0159 собирает Sound-сборка. Vision-pull берёт образы из
    `docker compose config --images` (preflight), поэтому образ подтянется, пока
    arbiter описан в compose Vision — этот тест ловит удаление/переезд arbiter'а."""
    import yaml

    root = Path(__file__).resolve().parents[2]
    compose = yaml.safe_load((root / "docker/vision/docker-compose.yaml").read_text(encoding="utf-8"))
    arbiter = next(s for s in compose["services"].values() if s.get("container_name") == "avatar-arbiter")
    assert ":supervisor-" in arbiter["image"]
    pull = dict(_steps(WORKFLOW_PATH.read_text(encoding="utf-8")))["[Vision Pi] Pull Docker Images"]
    assert "cd ~/rob_box_project/docker/vision" in pull and "preflight_pull_images.sh" in pull
    assert "docker compose config --images" in (root / "scripts/deploy/preflight_pull_images.sh").read_text(encoding="utf-8")


def test_every_referenced_step_id_is_defined_once():
    """Опечатка в `steps.sound_status.outputs...` проявилась бы только на реальном
    деплое (пустая строка -> "unknown"/ложный success) - ловим статически."""
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    ids = re.findall(r"(?m)^        id: (\w+)\s*$", workflow)
    assert len(ids) == len(set(ids)), f"повторяющиеся id шагов: {sorted(i for i in ids if ids.count(i) > 1)}"
    referenced = set(re.findall(r"steps\.(\w+)\.outputs", workflow))
    assert referenced <= set(ids), f"ссылки на несуществующие шаги: {sorted(referenced - set(ids))}"
    for node in ("vision", "sound", "main"):
        assert {f"{node}_status", f"{node}_logs", f"{node}_topics"} <= set(ids)
