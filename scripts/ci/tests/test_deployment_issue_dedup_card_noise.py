"""Карточки #4437, #4484, #4521, #4540: шум, который тянул «🚨 Deploy issues» каждый день.

Строки ниже — настоящие, из тел карточек и логов Deploy and Verify (run_id в комментариях).
Каждый тест пары «шум не critical» сопровождается парой «похожая настоящая беда остаётся critical».
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


def critical(log_text: str, scope: str) -> str | None:
    return MODULE.extract_relevant_log_line(log_text, scope=scope, severity="critical")


def warning(log_text: str, scope: str) -> str | None:
    return MODULE.extract_relevant_log_line(log_text, scope=scope, severity="warning")


# --- proximity_guard: стартовая гонка (run 42031160255, 05.11) -------------------------

GUARD_ZONE = (
    "[INFO] [1793883554.955000113] [proximity_guard]: 🛡️ зона корпуса: x[-0.304, 0.306] y[-0.302, 0.302]; "
    "внимание < 0.40 м, стоп < 0.20 м, только в движении (|v| > 0.03, |w| > 0.1)"
)
GUARD_BLIND = (
    "[ERROR] [1793883554.958885491] [proximity_guard]: 🛡️ нет свежих скана или одометрии (> 0.5 с) — "
    "зона НЕ контролируется"
)
GUARD_LIDAR = (
    "[INFO] [1793883554.974008483] [proximity_guard]: лидар lslidar_n10 в base_footprint: " "x=-0.171 y=-0.000 yaw=0.0°"
)
GUARD_OK = "[INFO] [1793883559.193597327] [proximity_guard]: 🛡️ скан и одометрия идут — зона под контролем"


def test_proximity_guard_startup_blind_window_that_recovered_is_not_critical() -> None:
    logs = "\n".join([GUARD_ZONE, GUARD_BLIND, GUARD_LIDAR, GUARD_OK])

    assert critical(logs, "main") is None


def test_recovered_startup_blind_window_stays_visible_as_warning() -> None:
    logs = "\n".join([GUARD_ZONE, GUARD_BLIND, GUARD_LIDAR, GUARD_OK])

    assert warning(logs, "main") == GUARD_BLIND


def test_proximity_guard_blind_without_recovery_stays_critical() -> None:
    # Лидар или одометрия не пришли совсем (HIL-стоп lslidar, упавший ros2-control): возврата нет.
    logs = "\n".join([GUARD_ZONE, GUARD_BLIND, GUARD_LIDAR])

    assert critical(logs, "main") == GUARD_BLIND


def test_proximity_guard_going_blind_again_after_recovery_stays_critical() -> None:
    # Последнее слово за порядком строк: ушла в слепую зону уже после возврата — настоящая потеря.
    blind_again = GUARD_BLIND.replace("1793883554", "1793883900")
    logs = "\n".join([GUARD_BLIND, GUARD_OK, blind_again])

    assert critical(logs, "main") == GUARD_BLIND


def _guard_echo(age_s: int = 4) -> str:
    # Формат print_report: msg обрезан до 60 символов (health_monitor.py:137).
    return (
        f"[health_monitor-4]   [ERROR] proximity_guard ({age_s}s ago): 🛡️ нет свежих скана или одометрии (> 0.5 с) — "
        "зона НЕ контро"
    )


def test_health_monitor_echo_of_guard_error_is_not_demoted_by_the_window() -> None:
    """Эхо + строка возврата, но без собственной строки слепоты — critical.

    Эхо health_monitor обрезано до 60 символов (`health_monitor.py:137`), возврата
    из него не прочитать. Строка «зона под контролем» двигает окно только для
    собственной строки ноды `[proximity_guard]: ...`. Самой строки слепоты в дампе
    нет — понижать нечего, эхо остаётся единственным critical (run 42108080199).
    Эхо само по себе окно возврата не двигает.
    """
    echo = _guard_echo()
    logs = "\n".join([echo, GUARD_OK])

    assert critical(logs, "main") == echo


def test_health_monitor_echo_is_not_a_separate_critical_when_own_blind_line_is_present() -> None:
    # Дамп как в #4979: эхо + собственная строка слепоты + возврат. Пункт один —
    # собственная строка, и окно возврата понижает её. Эхо — дубль, не второй critical.
    echo = _guard_echo(42)
    logs = "\n".join([GUARD_BLIND, echo, GUARD_OK])

    assert critical(logs, "main") is None
    assert warning(logs, "main") == GUARD_BLIND


def test_health_monitor_echo_alone_stays_critical() -> None:
    # Только эхо: собственной строки слепоты нет, понижать нечем.
    echo = _guard_echo(42)

    assert critical(echo, "main") == echo


def test_other_error_next_to_recovered_guard_still_surfaces() -> None:
    real = "[ERROR] [1793883560.1] [nav2]: controller server crashed"
    logs = "\n".join([GUARD_BLIND, GUARD_OK, real])

    assert critical(logs, "main") == real


# --- стартовый fail-fast, контейнер потом жив (issue #4989, run 44817771673) -------------

# Форма строки — как в карточке #4989 (vision-hailo, gaze.py). Числа и топик не важны:
# окно решает порядок «умер → стартовал и больше не падает», а не текст ошибки.
FAILFAST = (
    "[ERROR] [1795209099.065226069] [vision_hailo]: vision_hailo: Gaze source 'oak_d' не отдал "
    "кадр за 10.0s (capability-honest, ADR-0018).. fail-fast (real-mode обязателен, gaze_source='oak_d')."
)
DIED = "[ERROR] [vision_hailo-1]: process has died [pid 57, exit code 1, cmd 'vision_hailo']."
STARTED_AGAIN = "[INFO] [vision_hailo-1]: process started with pid [203]"
XLINK = "[ERROR] [oak_d]: X_LINK_ERROR: camera link lost, no frames"


def test_startup_failfast_then_healthy_does_not_create_candidate() -> None:
    # (а) fail-fast, затем контейнер поднялся и больше не падает — кандидата нет.
    logs = "\n".join([FAILFAST, DIED, STARTED_AGAIN, "[INFO] [vision_hailo]: subscribed to image_raw"])

    assert critical(logs, "vision") is None


def test_startup_failfast_without_recovery_creates_candidate() -> None:
    # (б) fail-fast без подъёма: смерть есть, повторного старта нет.
    logs = "\n".join([FAILFAST, DIED])

    assert critical(logs, "vision") == FAILFAST


def test_failfast_that_came_back_and_died_again_creates_candidate() -> None:
    # (в) поднялся, но снова падает: restarts растут, «жив после» нет.
    logs = "\n".join([FAILFAST, DIED, STARTED_AGAIN, DIED.replace("pid 57", "pid 203")])

    assert critical(logs, "vision") == FAILFAST


def test_familiar_failfast_line_without_alive_after_stays_critical() -> None:
    # «Строка знакомая» недостаточна: нет свидетельства «жив после» — кандидат есть.
    assert critical(FAILFAST, "vision") == FAILFAST


def test_recovered_failfast_does_not_mask_a_real_xlink_error() -> None:
    # Настоящий X_LINK рядом с оправившимся fail-fast карточку получает.
    logs = "\n".join([FAILFAST, DIED, STARTED_AGAIN, XLINK])

    assert critical(logs, "vision") == XLINK


def test_crash_loop_without_failfast_marker_stays_critical() -> None:
    # «process has died» само по себе не шум: без маркера fail-fast цикл
    # (смерть, старт, снова смерть) остаётся critical.
    logs = "\n".join([DIED, STARTED_AGAIN, DIED.replace("pid 57", "pid 203")])

    assert critical(logs, "vision") == DIED


# --- promtail → Loki 429 (runs 42288712543 … 42335902952, 06–07.11) --------------------

PROMTAIL_429 = (
    "level=warn ts=2026-11-06T20:32:23.903583867Z caller=client.go:419 component=client host=10.1.1.249:3100 "
    'msg="error sending batch, will retry" status=429 tenant= error="server returned HTTP status 429 Too Many '
    "Requests (429): Ingestion rate limit exceeded for user fake (limit: 4194304 bytes/sec) while attempting to "
    "ingest '8985' lines totaling '1048497' bytes, reduce log volume or contact your Loki administrator to see if "
    'the limit can be increased"'
)


def test_promtail_loki_ingestion_rate_limit_is_not_critical() -> None:
    assert critical(PROMTAIL_429, "sound") is None


def test_promtail_loki_server_error_stays_critical() -> None:
    line = PROMTAIL_429.replace("status=429", "status=500").replace("Ingestion rate limit exceeded", "internal error")
    line = line.replace("Loki administrator", "operator")  # без слова «loki» общий шум-фильтр не сработает

    assert critical(line, "sound") == line


# --- stt: INFO-метрика с timeout (run 42249624372, 06.11) ------------------------------


def test_stt_attempt_metric_timeout_is_not_critical() -> None:
    line = (
        "[stt_node-5] [INFO] [1793981633.436280660] [stt_node]: [stt_attempt_metric] provider=yandex "
        "reason=timeout latency_ms=5026 attempt=0 text=- error='timeout>5.0s'"
    )

    assert critical(line, "sound") is None


def test_stt_real_error_stays_critical() -> None:
    line = "[stt_node-5] [ERROR] [1793981633.4] [stt_node]: Failed to open audio device: boom"

    assert critical(line, "sound") == line


# --- tts: однократная смерть провайдера на 30 с (run 42475127085, 08.11) ---------------

TTS_DEAD_30 = (
    "[tts_node-4] [WARN] [1794137057.417456363] [tts_node]: 💀 minimax помечен мёртвым на 30s "
    "(Exception: MiniMax stream: нет звука (не уложился в 7.9 с))"
)


def test_tts_provider_transient_death_is_demoted_to_warning() -> None:
    assert critical(TTS_DEAD_30, "sound") is None
    assert warning(TTS_DEAD_30, "sound") == TTS_DEAD_30


def test_tts_provider_permanent_death_stays_critical() -> None:
    # 300 s — постоянный отказ (ключ, квота): «voice stack degrades silently», это уже беда.
    line = TTS_DEAD_30.replace("на 30s", "на 300s").replace("MiniMax stream: нет звука", "HTTP 401 invalid api key")

    assert critical(line, "sound") == line


# --- sound_node: UI-звук пропущен (все четыре карточки) --------------------------------


def test_sound_node_ui_sound_skipped_busy_is_not_a_warning() -> None:
    line = (
        '[sound_node-6] [WARN] [1793894035.554782811] [sound_node]: ⚠️ Звук "ui_confirm" пропущен: busy '
        "(играет: button_click)"
    )

    assert warning(line, "sound") is None
    assert critical(line, "sound") is None


def test_sound_node_other_warning_stays_visible() -> None:
    line = '[sound_node-6] [WARN] [1793894035.5] [sound_node]: ⚠️ Звук "ui_confirm" не найден в каталоге'

    assert warning(line, "sound") == line


# --- stt: эхо дедлайна облака в perception (run 42249624372, 06.11) --------------------

STT_DEADLINE_ECHO = (
    "[health_monitor-4]   [WARN] stt_node (2s ago): ⚠️ [issue 1477] stream error: StatusCode.DEADLINE_EXCEEDED D"
)


def test_stt_cloud_deadline_echo_is_not_critical() -> None:
    assert critical(STT_DEADLINE_ECHO, "main") is None


def test_stt_other_grpc_status_echo_stays_critical() -> None:
    line = STT_DEADLINE_ECHO.replace("DEADLINE_EXCEEDED", "UNAUTHENTICATED")

    assert critical(line, "main") == line


# Sound Pi, staging run 45414643521. The next successful Yandex attempt preceded the deploy check.
STT_DEADLINE = (
    "[stt_node-5] [WARN] [1795530779.006027370] [stt_node]: ⚠️ [issue 1477] stream error: "
    "StatusCode.DEADLINE_EXCEEDED Deadline Exceeded [#2924 phase=REAL_TIME responses=0 partials=0 "
    "eou=0 segments=0 stream=[-]]"
)


def test_stt_source_deadline_is_reported_as_warning() -> None:
    recovery = "[stt_node-5] [INFO] [1795530810.728058853] [stt_node]: [stt_attempt_metric] provider=yandex reason=ok"
    logs = f"{STT_DEADLINE}\n{recovery}"
    assert critical(logs, "sound") is None
    assert warning(logs, "sound") == STT_DEADLINE


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ("[WARN]", "[ERROR]"),
        ("DEADLINE_EXCEEDED", "UNAUTHENTICATED"),
        ("DEADLINE_EXCEEDED", "PERMISSION_DENIED"),
        ("[stt_node]:", "[dialog_node]:"),
    ],
)
def test_stt_other_source_errors_stay_critical(old: str, new: str) -> None:
    line = STT_DEADLINE.replace(old, new)
    assert critical(line, "sound") == line


def test_stt_deadline_does_not_hide_a_traceback() -> None:
    detail = "[stt_node-5] RuntimeError: failed to capture audio"
    logs = f"{STT_DEADLINE}\n[stt_node-5] Traceback (most recent call last):\n{detail}"
    assert critical(logs, "sound") == detail


def test_stt_deadline_does_not_hide_another_node_error() -> None:
    error = "[ERROR] [audio_node]: capture failed"
    assert critical(f"{STT_DEADLINE}\n{error}", "sound") == error


@pytest.mark.parametrize(
    "line",
    [
        STT_DEADLINE.replace("Deadline Exceeded", "[ERROR] Authorization failed"),
        STT_DEADLINE + " [ERROR] audio capture failed",
        STT_DEADLINE.replace("stream=[-]", "stream=[[ERROR] audio capture failed]"),
    ],
)
def test_stt_deadline_with_unexpected_error_text_stays_critical(line: str) -> None:
    assert critical(line, "sound") == line
