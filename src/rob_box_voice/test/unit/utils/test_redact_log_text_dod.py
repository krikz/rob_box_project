"""DoD acceptance test for ``redact_log_text`` integration with ``read_logs``.

Карточка ``[operator-agent 08-DoD] тесты на slice/redact/ConfirmationPolicy``
issue #1998 формулирует DoD-2 так:

    «``read_logs`` возвращает ``***REDACTED***`` вместо ``DEEPSEEK_API_KEY=...``»

Здесь — узкая приёмочная проверка этого факта на уровне текста,
который ``read_logs`` через ``operator_admin._sanitize_text`` отдаёт
в LLM. Тест НЕ поднимает rclpy / Loki / HealthMonitor — он строит
синтетический «stub-лог», имитирующий ``docker logs`` операторского
контейнера, и проверяет, что после прохождения через
``redact_log_text`` наружу не утекает ни один из обязательных ключей.

Этот файл сознательно узкий:

* Покрывает ровно тот сценарий, что указан в DoD-2 (DEEPSEEK/MINIMAX_API_KEY,
  «очевидный не-секрет» остаётся нетронутым, идемпотентность).
* Широкая матрица других форм (``--token=``, bare JWT, vendor-префиксы)
  живёт в ``test_redact.py`` — там же, где и юнит-тесты на helper.
* Подгрузка ``redact.py`` напрямую — тот же приём, что в
  ``test_redact.py`` (нужно, чтобы тест был герметичен от
  ``utils/__init__``, который тянет pyaudio / ReSpeaker).
"""

from __future__ import annotations

import importlib.util as _ilu
import sys as _sys
from pathlib import Path

import pytest


# ---------------------------------------------------------------------------
# Подгрузка ``rob_box_voice.utils.redact`` напрямую — обходим
# ``utils/__init__``, чтобы не зависеть от pyaudio / ReSpeaker,
# которых нет в dev-контейнере.
# ---------------------------------------------------------------------------

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[3]  # .../src/rob_box_voice
_REDACT_PATH = _PKG_ROOT / "rob_box_voice" / "utils" / "redact.py"


def _load_redact_module():
    spec = _ilu.spec_from_file_location(
        "rob_box_voice_redact_under_test", _REDACT_PATH
    )
    assert spec is not None and spec.loader is not None  # type guard для pyright
    mod = _ilu.module_from_spec(spec)
    _sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


_redact = _load_redact_module()
redact_log_text = _redact.redact_log_text


# ---------------------------------------------------------------------------
# Stub-логи — имитируют ``docker logs`` операторского контейнера.
# ---------------------------------------------------------------------------
#
# Значения ключей — явно-плейсхолдерные, чтобы secret-scanner (и ручной
# ревьюер) мог отличить их от настоящих секретов. Это сделано
# осознанно: тесты КОММИТЯТСЯ в публичный репозиторий, и в них не
# должно быть ничего похожего на настоящий API-ключ.

_STUB_DOCKER_LOGS = """\
2026-09-07T10:11:12 voice-assistant 1 - DEBUG boot: DEEPSEEK_API_KEY=PLACEHOLDER_NOT_A_SECRET ready
2026-09-07T10:11:13 voice-assistant 1 - DEBUG env: MINIMAX_API_KEY=PLACEHOLDER_NOT_A_SECRET ok
2026-09-07T10:11:14 voice-assistant 1 - INFO  start provider=yandex_fallback reason=cold_start
2026-09-07T10:11:15 voice-assistant 1 - WARN  retry attempt=2 status_code=503
2026-09-07T10:11:16 voice-assistant 1 - ERROR upstream deepseek: hint="check DEEPSEEK_API_KEY env"
"""


def _run_stub_through_redactor(log_blob: str) -> str:
    """Имитирует ``read_logs`` → ``_sanitize_text`` → LLM-payload.

    На каждый line лога накладываем ``redact_log_text`` — именно это
    делает ``operator_admin._sanitize_text`` (см. файл
    ``tools/operator_admin.py``: функция прогоняет обе — upstream
    и log — версии, но для DoD-2 достаточно log-варианта).
    """
    return "\n".join(redact_log_text(line) for line in log_blob.splitlines())


# ---------------------------------------------------------------------------
# DoD-2 acceptance tests
# ---------------------------------------------------------------------------


def test_dod2_read_logs_redacts_deepseek_api_key_in_stubbed_log() -> None:
    """DoD-2: ``DEEPSEEK_API_KEY=...`` в stub-логе превращается в ``***``.

    Карточка требует именно этого поведения: что бы операторский
    агент ни увидел через ``read_logs``, значение ``DEEPSEEK_API_KEY``
    не должно туда утечь. Имя ключа при этом сохраняется — для
    контекста при диагностике.
    """
    sanitized = _run_stub_through_redactor(_STUB_DOCKER_LOGS)

    # 1. Сырое placeholder-значение не должно выжить.
    assert "PLACEHOLDER_NOT_A_SECRET" not in sanitized, (
        "DoD-2 нарушен: значение API-ключа осталось в выводе read_logs.\n"
        f"--- sanitized ---\n{sanitized}\n--- end ---"
    )

    # 2. Имя ключа должно быть видно (оператору нужен контекст, что
    #    именно «утонуло» в ***).
    assert "DEEPSEEK_API_KEY" in sanitized, (
        "Имя DEEPSEEK_API_KEY должно быть видно оператору для контекста."
    )

    # 3. Маска — это *** (helper оставляет KEY=***).
    assert "DEEPSEEK_API_KEY=***" in sanitized, (
        "Ожидаем форму 'DEEPSEEK_API_KEY=***', "
        f"получили вывод:\n{sanitized}"
    )


def test_dod2_read_logs_redacts_minimax_api_key_in_stubbed_log() -> None:
    """DoD-2 контраст: ``MINIMAX_API_KEY=...`` тоже маскируется.

    ``MINIMAX_API_KEY`` — второй обязательный секрет операторского
    контейнера. Если бы регресс оставил только DEEPSEEK_API_KEY
    (например, в regex-паттерне добавили бы только одно имя), этот
    тест бы покраснел.
    """
    sanitized = _run_stub_through_redactor(_STUB_DOCKER_LOGS)

    assert "PLACEHOLDER_NOT_A_SECRET" not in sanitized
    assert "MINIMAX_API_KEY=***" in sanitized


def test_dod2_non_secret_context_preserved_in_stub_log() -> None:
    """DoD-2 негатив: «обычные» лог-строки не должны пострадать.

    DoD-2 — это про защиту секретов, а не про «всё замаскировать».
    Поэтому ``start provider=yandex_fallback`` и ``retry attempt=2
    status_code=503`` должны остаться в выводе — иначе оператор не
    сможет диагностировать проблему, ради которой он позвал ``read_logs``.
    """
    sanitized = _run_stub_through_redactor(_STUB_DOCKER_LOGS)

    assert "start provider=yandex_fallback" in sanitized, (
        "Не-секретный контекст не должен маскироваться:\n" + sanitized
    )
    assert "retry attempt=2 status_code=503" in sanitized, (
        "Не-секретный контекст не должен маскироваться:\n" + sanitized
    )
    # Контейнер / сервис-имя тоже должно сохраниться — без него
    # оператор не понимает, ЧЬИ это логи.
    assert "voice-assistant" in sanitized


def test_dod2_redactor_is_idempotent_on_stub_log() -> None:
    """DoD-2: повторный прогон через redactor не должен ломать вывод.

    Это — инвариант идемпотентности ``redact_log_text``. Если бы
    regex был неидемпотентным (например, добавлял маску поверх уже
    замаскированной строки), вторая итерация могла бы выдать
    ``DEEPSEEK_API_KEY=**********``, и ``read_logs`` дважды был бы
    неконсистентным.
    """
    once = _run_stub_through_redactor(_STUB_DOCKER_LOGS)
    twice = _run_stub_through_redactor(once)

    assert once == twice, (
        "redact_log_text обязан быть идемпотентным — иначе read_logs\n"
        "двух вызовов подряд даёт разные строки.\n"
        f"--- once ---\n{once}\n--- twice ---\n{twice}"
    )


@pytest.mark.parametrize(
    "secret_line",
    [
        "boot: DEEPSEEK_API_KEY=PLACEHOLDER_NOT_A_SECRET",
        "env: MINIMAX_API_KEY=PLACEHOLDER_NOT_A_SECRET",
        "boot: MIMO_API_KEY=PLACEHOLDER_NOT_A_SECRET",
        "boot: YANDEX_API_KEY=PLACEHOLDER_NOT_A_SECRET",
    ],
)
def test_dod2_each_required_secret_masked_independently(secret_line: str) -> None:
    """Параметризованная матрица DoD-2: каждый из 4 обязательных секретов.

    Карточка перечисляет набор ключей, которые ОБЯЗАНЫ быть
    замаскированы. Если кто-то добавит новый секрет в ``.env.secrets``
    (например, ``ROBOFLOW_TOKEN``) и забудет расширить паттерн —
    эта матрица должна расширяться новым кейсом.
    """
    sanitized = redact_log_text(secret_line)
    assert "PLACEHOLDER_NOT_A_SECRET" not in sanitized
    assert "***" in sanitized
