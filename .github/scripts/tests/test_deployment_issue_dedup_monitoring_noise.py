"""Issue #3664 п.1: гейт деплоя не должен заводить «🚨 Deploy issues» из-за шума мониторинга.

Каждый деплой 19.10 (#3637, #3653, #3663) получал critical от трёх cadvisor
(Vision/Sound/Main) из-за INFO-строки klog

    I1019 20:55:15.630362       1 gce.go:45] Error while reading product_name: ...

— слово «Error» в тексте INFO-строки не делает её ошибкой, а на Raspberry Pi DMI
нет никогда. Плюс две известные проблемы мониторинга, у которых уже есть свои
issue (#3664 п.2 — cadvisor слеп на Docker 29; #3659 — promtail со старым
клиентом Docker API): они понижены с critical до warning (видны в отчёте, но
deploy-issue не заводят).

Строки ниже — настоящие (из Deploy and Verify run 39322722203 и issue #3659);
синтетические только 64-символьные id контейнеров, которые в issue обрезаны.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

SCRIPT_PATH = Path(__file__).resolve().parents[1] / "deployment_issue_dedup.py"
SPEC = importlib.util.spec_from_file_location("deployment_issue_dedup", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

SCOPES = ("vision", "sound", "main")

CONTAINER_ID = "782c4f0e9a1b3d5c6e7f8091a2b3c4d5e6f708192a3b4c5d6e7f8091a2b3c4d5"

GCE_INFO_LINES = {
    "vision": "I1019 20:55:15.630362       1 gce.go:45] Error while reading product_name: "
    "open /sys/class/dmi/id/product_name: no such file or directory",
    "sound": "I1019 20:58:21.490134       1 gce.go:45] Error while reading product_name: "
    "open /sys/class/dmi/id/product_name: no such file or directory",
    "main": "I1019 20:59:58.693093       1 gce.go:45] Error while reading product_name: "
    "open /sys/class/dmi/id/product_name: no such file or directory",
}

LAYER_ID_LINE = (
    "E1019 20:59:21.591612       1 manager.go:1116] Failed to create existing container: "
    f"/system.slice/docker-{CONTAINER_ID}.scope: failed to identify the read-write layer ID "
    f'for container "{CONTAINER_ID}". - open '
    f"/rootfs/var/lib/docker/image/overlayfs/layerdb/mounts/{CONTAINER_ID}/mount-id: "
    "no such file or directory"
)

PROMTAIL_OLD_CLIENT_LINE = (
    "level=error caller=refresh.go:80 component=docker_discovery "
    'msg="Unable to refresh target groups" err="error while listing containers: '
    "Error response from daemon: client version 1.42 is too old. "
    'Minimum supported API version is 1.44"'
)


def critical(log_text: str, scope: str) -> str | None:
    return MODULE.extract_relevant_log_line(log_text, scope=scope, severity="critical")


def warning(log_text: str, scope: str) -> str | None:
    return MODULE.extract_relevant_log_line(log_text, scope=scope, severity="warning")


# --- п.1: klog уровня I/W не critical -------------------------------------------------


@pytest.mark.parametrize("scope", SCOPES)
def test_klog_info_gce_product_name_is_not_critical(scope: str) -> None:
    assert critical(GCE_INFO_LINES[scope], scope) is None


@pytest.mark.parametrize("scope", SCOPES)
def test_klog_info_gce_product_name_is_not_a_warning_either(scope: str) -> None:
    # INFO остаётся INFO: ни в critical, ни в warning отчёт он не попадает.
    assert warning(GCE_INFO_LINES[scope], scope) is None


def test_klog_info_is_not_critical_whatever_error_word_it_carries() -> None:
    # Правило по уровню klog, а не по конкретному gce.go: любая I-строка с «error»,
    # «failed to», «exception» в тексте — не ошибка.
    for text in (
        "I1019 20:55:15.630362       1 factory.go:123] Failed to get something optional, continuing",
        "I1019 20:55:15.630362       1 handler.go:7] exception handler registered",
        "I1019 20:55:15.630362       1 x.go:9] retrying after error: temporary",
    ):
        assert critical(text, "vision") is None, text


def test_klog_warning_level_is_demoted_to_warning_and_stays_visible() -> None:
    # W — уровень, который эмиттер сам назвал предупреждением: не critical, но в отчёте
    # (слова «warn» в тексте нет, поэтому без явной ветки строка пропала бы совсем).
    line = "W1019 20:55:16.100000       1 container.go:420] Failed to get RW layer size for container: unavailable"

    assert critical(line, "vision") is None
    assert warning(line, "vision") == line


def test_klog_warning_level_without_error_words_is_still_ignored() -> None:
    # Демоция касается только строк, которые иначе были бы critical; прочие W-строки
    # (без слов ошибки и без «warn») по-прежнему никуда не попадают.
    line = "W1019 20:55:16.100000       1 machine.go:65] Cannot read vendor id correctly, set empty"

    assert critical(line, "vision") is None
    assert warning(line, "vision") is None


@pytest.mark.parametrize("level", ["E", "F"])
def test_klog_error_and_fatal_levels_stay_critical(level: str) -> None:
    line = f"{level}1019 20:55:15.630362       1 manager.go:1116] Failed to start container manager: boom"

    assert critical(line, "vision") == line
    assert warning(line, "vision") is None


def test_info_noise_does_not_mask_real_error_later_in_the_same_log() -> None:
    real = "E1019 21:00:01.000000       1 manager.go:77] Failed to start container manager: cannot mount cgroup"
    logs = "\n".join([GCE_INFO_LINES["vision"], "I1019 20:55:16.000000       1 ok.go:1] started", real])

    assert critical(logs, "vision") == real


def test_klog_regexp_requires_the_full_klog_header() -> None:
    # «I» в начале обычной строки — не klog: такая строка с «error» остаётся critical.
    line = "Initialization error: cannot open /dev/video0"

    assert critical(line, "vision") == line


# --- п.2/#3659: известные проблемы мониторинга → warning ------------------------------


@pytest.mark.parametrize("scope", ["vision", "sound"])
def test_cadvisor_layer_id_error_is_demoted_to_warning(scope: str) -> None:
    assert critical(LAYER_ID_LINE, scope) is None
    assert warning(LAYER_ID_LINE, scope) == LAYER_ID_LINE


@pytest.mark.parametrize("scope", SCOPES)
def test_promtail_old_docker_api_client_is_demoted_to_warning(scope: str) -> None:
    assert critical(PROMTAIL_OLD_CLIENT_LINE, scope) is None
    assert warning(PROMTAIL_OLD_CLIENT_LINE, scope) == PROMTAIL_OLD_CLIENT_LINE


def test_other_cadvisor_container_error_stays_critical() -> None:
    # Та же E-строка «Failed to create existing container», но по другой причине.
    line = (
        "E1019 20:59:21.591612       1 manager.go:1116] Failed to create existing container: "
        f"/system.slice/docker-{CONTAINER_ID}.scope: permission denied"
    )

    assert critical(line, "vision") == line


def test_other_promtail_error_stays_critical() -> None:
    line = (
        "level=error caller=refresh.go:80 component=docker_discovery "
        'msg="Unable to refresh target groups" err="error while listing containers: '
        'Cannot connect to the Docker daemon at unix:///var/run/docker.sock"'
    )

    assert critical(line, "vision") == line


def test_old_docker_client_error_outside_promtail_discovery_stays_critical() -> None:
    # Понижение привязано к docker_discovery promtail: приложение, которое ходит в Docker
    # SDK и получает то же сообщение, — реальный сбой, а не шум мониторинга.
    line = (
        "docker.errors.APIError: 400 Client Error: client version 1.41 is too old. "
        "Minimum supported API version is 1.44"
    )

    assert critical(line, "sound") == line


def test_known_monitoring_issues_do_not_mask_real_error_in_the_same_log() -> None:
    real = "[ERROR] [1729371000.123] [vision_hailo_node]: HEF loader failed: HailoRTStatusException(63)"
    logs = "\n".join([LAYER_ID_LINE, PROMTAIL_OLD_CLIENT_LINE, GCE_INFO_LINES["vision"], real])

    assert critical(logs, "vision") == real


# --- критичное остаётся критичным ----------------------------------------------------


@pytest.mark.parametrize(
    "line",
    [
        "2026-10-19 21:00:01 CRITICAL dialogue_node: unhandled exception in callback: boom",
        "[ERROR] [1729371000.123] [vision_hailo_node]: HEF loader failed: HailoRTStatusException(63). Кадр пропущен.",
        "[tts_node-4] ERROR: Out of memory: Killed process 4242 (python3)",
        "Segmentation fault (core dumped)",
        "[FATAL] [1729371000.123] [nav2]: controller server crashed",
        "F1019 20:55:15.630362       1 main.go:1] Fatal: failed to initialise",
    ],
)
def test_real_critical_lines_stay_critical(line: str) -> None:
    assert critical(line, "vision") == line
