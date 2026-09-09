"""AvatarSupervisor — ROS 2 нода-координатор аватара: голос + супервизор-агент.

Разделение ролей (ADR-0051 §2.2, issue #1987):
- Арбитраж floor/FSM + публикация ``/avatar/state`` вынесены в отдельную
  ноду ``avatar_arbiter`` (``rob_box_supervisor/arbiter_node.py``) — там
  живут ``LockManager``/``ModeManager``/агрегатор и сервисы
  ``/avatar_arbiter/{acquire_floor,release_floor,set_avatar_mode}``.
- Здесь (``AvatarSupervisor``) остаются:
  * voice-управление ``dialogue_node``/``tts_node``: ``/avatar/set_voice_mode``,
    ``/avatar/set_voice_preset``, ``/avatar/set_voice_language``,
    ``/avatar/set_voice``, ``/avatar/preview_voice*`` (ADR-0028 S5, AV-27/28);
  * супервизор-агент оператора (ТАРС, issue #1988): ``/avatar/command`` и
    ``/avatar/stt/result`` → ``AgentCore`` (промпт оператора) →
    ``/avatar/command_result`` + voice-mode swap;
  * пайплайн грипа (issue #1989, шаг 4б): ``/avatar/ptt/result`` +
    ``/avatar/voice_pipeline`` → transform (0|1 LLM, pure-логика в
    ``grip_pipeline.py``) → ``/voice/tts/request``. Прямоточный путь, НЕ
    агентский цикл: без AgentCore / ToolProvider / памяти (§7.5, инвариант 6c).

Параметр ``mode`` (default ``"monitor"``) остаётся гейтом применения
voice-параметров: в ``monitor`` супервизор не трогает чужие параметры
(ADR-0028 §4.5/S12), в ``active`` — применяет.

Источники истины:
- ADR-0051 §2.2 + docs/architecture/target-operator-agent-and-dialogue.md §4
- ADR-0028 §4.3 / §4.5 / S5 (voice-параметры, monitor-режим)
- docs/architecture/avatar-supervisor-agent.md (агент оператора, AV-21)
- docs/architecture/meta-quest-api.md §3, §5.1 (wire-контракт клиентов)

Zenoh: ``ZENOH_SESSION_CONFIG_URI`` env-переменная подхватывается
rmw_zenoh_cpp автоматически — нам читать её вручную не нужно, только
залогировать на старте для диагностики (см. :py:meth:`__init__`).
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import os
import time
import types
import uuid
from typing import Any, Iterator, Mapping, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString

# AV-27 / issue #1919 — импорт SoT голосов. Pure-Python, без rclpy —
# безопасен в любом окружении. Используется в _apply_set_voice и
# _on_preview_voice для валидации voice_id.
from rob_box_voice.tts_voice_registry import voices_for as _voices_for

# Issue #2081 — единый источник истины для топиков avatar/command*.
# До этого PR константы дублировались локально (см. ниже старые
# AVATAR_COMMAND_TOPIC/AVATAR_COMMAND_RESULT_TOPIC) — это и было причиной
# регрессии: rob_box_core.avatar_command содержит актуальные значения
# контракта, а supervisor импортирует старые. Теперь импортируем SoT.
from rob_box_core.avatar_command import (
    AVATAR_COMMAND_RESULT_TOPIC,
    AVATAR_COMMAND_TOPIC,
)
# voice-vr 12 (issue #2197, ADR-0080 §1.3 / §2.3): единый сборщик SSML.
# Issue #2233 — try/except fallback: round-деплои (z-{e2e}/test-round-N)
# наследуют voice-assistant-humble-*-test из registry, а тот обновляется
# develop-билдами БЕЗ SHA-tag pinning (issue #1826 anti-loop). Если
# PR #2218 (utterance) merged в develop ПОСЛЕ последнего push
# voice-assistant-humble-test → supervisor стартует в base-image без
# rob_box_core.utterance → ModuleNotFoundError → restart-loop.
# Fallback держит supervisor живым: monitor-режим публикует /avatar/state,
# avatar-TTS запросы уходят с минимальной локальной реализацией Sink/Utterance.
# Удалить этот блок когда round-ветки начнут использовать SHA-pinned теги
# или develop-build будет триггерить voice-assistant rebuild (TODO issue).
try:
    from rob_box_core.utterance import Sink, Utterance  # type: ignore[import-not-found]
except ImportError:
    from rob_box_supervisor._utterance_fallback import Sink, Utterance  # noqa: F401
# Issue #2240 — ws_server и supervisor больше НЕ держат свои копии
# whitelist'а AV-28 §P7: single source of truth в rob_box_core.bridge_protocol.
# Раньше копия тут расходилась с ws_server / YAML (молчаливый отказ на
# пресет `translate` + языки fr/de/zh/hi — см. комментарий ниже). Теперь
# импортируем канон, conformance проверяется в test_supervisor_node.py.
from rob_box_core.bridge_protocol import (  # noqa: E402,F401
    VOICE_LANGUAGES,  # re-export для обратной совместимости
    VOICE_PRESET_IDS,  # re-export для обратной совместимости
)
# ADR-0083 §2.3 — supervisor собирает AgentCore через build_agent(AgentSpec).
# До этого PR у supervisor был свой ``_build_operator_llm`` (без persist_path),
# свой ``_load_operator_system_prompt``, своя ``_load_operator_skill_prompts``
# (лезла в voice/skills — протечка), и не было ``on_prompt``. Сейчас все
# эти 4 хелпера удалены, остался только ``_build_operator_memory``
# (нода владеет asyncio-loop) и ``_build_operator_tools`` (нужен self
# для ``ROSMCPToolProvider(LLMToolCallAdapter(self))``).
from rob_box_harness.core.assembly import (  # noqa: E402  — lazy harness import
    AgentSpec,
    build_agent,
)
# ADR-0083 §G — PromptStats для supervisor идут в ту же гистограмму
# ``voice_llm_prompt_tokens``, что и dialogue_node (issue #2111).
from rob_box_voice.observability import (  # noqa: E402
    record_llm_prompt_tokens,
)

# voice-vr 21 — единый белый список voice-пресетов и языков (AV-28 §P7).
# Раньше жил в ``supervisor_node`` и ``ws_server`` двумя параллельными
# копиями (третья — приватный ``_AV28_*`` на классе), что и дало
# «UI сказал применилось, supervisor сказал applied=False» (пресет
# ``translate`` и языки fr/de/zh/hi тихо выпали из ротации). Теперь
# единственный источник — ``rob_box_core.bridge_protocol`` (зеркало
# ``voice_presets.yaml`` + TS-генерация).
from rob_box_core.bridge_protocol import (  # noqa: E402,F401 — re-export SoT
    VOICE_LANGUAGES,
    VOICE_PRESET_IDS,
)

# ADR-0080 §2.7 / voice-vr 21 — единый топик-контракт смены голоса.
# ``/voice/tts/set_voice`` живёт на tts_node (``_on_set_voice``) и
# единственный, кто принимает voice_id от супервизора. Раньше здесь
# был ленивый параметр-клиент на tts_node (знание внутренней схемы
# имён ``yandex_voice``/``minimax_voice``/``silero_speaker``) — это
# явный шов ADR-0080 §2.7, теперь закрыт.
SET_TTS_VOICE_TOPIC: str = "/voice/tts/set_voice"


# AV-14 (issue #1906) — ``/avatar/state`` wire format lives in
# :mod:`rob_box_supervisor.core.state`. The supervisor here ONLY calls
# :func:`encode_for_ros_string`; no local msgpack, no JSON fallback.
# See ``docs/plans/2026-09-02-avatar-epic-state-audit.md`` §1.2 G3 for the
# silent-fail bug this prevents (msgpack publisher + JSON consumer = Telegram
# never saw any state). The codec helper also handles the
# ``forward-compat /msgpack absent`` defensive case via
# :func:`is_ros_string_safe`, so this module does not need its own try/except
# + JSON fallback — the bug we are closing lived precisely in that branch.


# Default monitor-mode reason, который нода возвращает клиентам в Phase 1.
# Зафиксирован строкой, чтобы логи и e2e-тесты могли матчить без магических
# литералов по всему коду (ADR-0028 §4.5).
MONITOR_MODE_REASON = "supervisor_in_monitor_mode"

# Reason-коды для типизированных ответов. Экспортируются константами, чтобы
# клиенты/тесты могли матчить без магических литералов (S14 ADR-0028).
REASON_OK = "ok"
REASON_GRANTED = "granted"
REASON_RELEASED = "released"
REASON_HELD_BY_OTHER = "held_by_other"
REASON_BAD_REQUEST = "bad_request"
REASON_CONFLICT = "conflict"
REASON_PERMISSION_DENIED = "permission_denied"
REASON_INVALID_EVENT = "invalid_event"
REASON_INVALID_REQUEST = "invalid_request"
REASON_APPLIED = "applied"
# По ADR-0028 §4.5 — в monitor-режиме сервис НЕ вмешивается, но и НЕ
# отказывает клиенту. Стандартное behaviour: applied=false, reason=MONITOR_MODE_REASON.
REASON_MONITOR = MONITOR_MODE_REASON

# ADR-0083 §2.3 / issue #2111 — supervisor собирает AgentCore через
# :func:`rob_box_harness.core.assembly.build_agent` с
# :class:`AgentSpec`. Это закрывает три бага:
#   * §1.3 #1 — supervisor ронял HealthCache() без persist_path (после
#     рестарта все «больные» провайдеры снова «здоровы»). ``build_agent``
#     использует общий ``~/.rob_box/llm_health.json`` — supervisor и
#     dialogue делят один кеш.
#   * §1.3 #3 — supervisor лез в ``rob_box_voice/prompts/skills`` по
#     относительному пути (протечка в чужой пакет). Теперь спек явно
#     задаёт ``skill_slice=("operator.speech", "operator.control")``
#     и больше не открывает ``voice/skills``.
#   * §2.3 #G — supervisor не публиковал PromptStats. ``AgentSpec.on_prompt``
#     зовёт ``_record_supervisor_prompt_stats``, и ``record_llm_prompt_tokens``
#     пишет в ту же гистограмму, что и dialogue.
# ADR-0083 §E — ``/data/operator_memory.db`` упразднён. Оба агента
# (personality + operator) пишут в ``/data/harness_voice.db`` через
# ``SQLiteVoiceMemory(agent=...)``. ``agent`` — колонка ``facts`` (миграция
# 011_agent_namespace.sql).
#
# ADR-0066 — после merge §6 dialogue_node больше НЕ принимает параметр
# ``voice_input_mode``. Управление личностью идёт через топик
# ``/dialogue/control`` (String JSON, action: pause|resume). Внешний
# контракт ``/avatar/set_voice_mode`` остаётся для обратной совместимости
# с клиентами (UI Quest, web-admin), но режимы ``respeaker`` / ``off``
# маппятся на ``resume`` / ``pause`` соответственно; остальные значения
# (quest_ttts, quest_stt и т.п.) — отвергаются с
# ``reason="voice_mode_deprecated"`` (ADR-0018: честный FAIL, не молчание).
SET_VOICE_MODE_TOPIC: str = "/avatar/set_voice_mode"

# ADR-0066 §2.1 — единый канал «оператор → личность» (pause/resume).
# Publisher — супервизор (AvatarSupervisor), subscriber — dialogue_node.
# QoS — RELIABLE, KEEP_LAST depth=10 (синхронизировано с dialogue_node,
# см. dialogue_node.py:200). Payload: JSON
# ``{"action": "pause"|"resume", "reason": str, "ts_s": float}``.
DIALOGUE_CONTROL_TOPIC: str = "/dialogue/control"
DIALOGUE_CONTROL_ACK_TOPIC: str = "/dialogue/control_ack"
# Допустимые ``action``-значения (ADR-0066 §2.1, _on_dialogue_control
# в dialogue_node.py:2189). Используются как whitelist в
# ``_publish_dialogue_control`` для защиты от опечаток в caller-коде.
DIALOGUE_CONTROL_ACTIONS: frozenset[str] = frozenset({"pause", "resume"})
# Строковые литералы — для прямого использования в payload и в legacy-маппинге
# (вместо ``DIALOGUE_CONTROL_ACTIONS``-итерации — читаемость).
DIALOGUE_CONTROL_PAUSE: str = "pause"
DIALOGUE_CONTROL_RESUME: str = "resume"

# AV-27 / issue #1919 — TTS picker топики (симметрично /avatar/set_voice_mode).
# Payload — JSON в std_msgs/String, как принято в supervisor_node. См.
# docs/architecture/tts-picker-ros-path.md §128-150.
SET_VOICE_TOPIC: str = "/avatar/set_voice"
PREVIEW_VOICE_TOPIC: str = "/avatar/preview_voice"
PREVIEW_VOICE_RESULT_TOPIC: str = "/avatar/preview_voice/result"
PREVIEW_VOICE_AUDIO_TOPIC: str = "/avatar/preview_voice/audio"
PREVIEW_VOICE_ERROR_TOPIC: str = "/avatar/preview_voice/error"

# ── Bridge.execute(Command) — ADR-0051 §2.1, §2.2, issue #2002 ───────
# kind-enum зеркалит rob_box_supervisor_msgs/msg/Command.msg (uint8-значения
# ОБЯЗАНЫ совпадать с .msg — фиксируем строкой-константой, чтобы и
# unit-тесты, и реальные сервисные ответы матчились без магических
# литералов). Допустимые значения: 0 (UNKNOWN — для отказа/невалидного
# запроса) и 1..6 для нормальных команд.
KIND_UNKNOWN: int = 0
KIND_ACQUIRE_FLOOR: int = 1
KIND_RELEASE_FLOOR: int = 2
KIND_SET_AVATAR_MODE: int = 3
KIND_SET_VOICE_MODE: int = 4
KIND_EMERGENCY_STOP: int = 5
KIND_HEARTBEAT: int = 6

# Имя единого сервиса для всего шва «клиент ↔ supervisor». Клиенты
# (Quest, Telegram, будущий web-admin) после миграции (Phase 2) будут
# дёргать только его; в Phase 1 legacy-сервисы остаются и продолжают
# работать — никаких breaking changes (ADR-0013 incremental delivery).
EXECUTE_COMMAND_SERVICE: str = "/supervisor/execute"

# Reason-коды для Response.msg (должны совпадать со списком в
# rob_box_supervisor_msgs/msg/Response.msg — фиксируем явно, чтобы
# регрессия «новый код, но reason другой» ловилась grep'ом).
EXEC_REASON_UNKNOWN_KIND: str = "unknown_kind"
EXEC_REASON_MONITOR_MODE: str = "monitor_mode"
EXEC_REASON_HELD_BY_OTHER: str = "held_by_other"
EXEC_REASON_MODE_CONFLICT: str = "mode_conflict"
EXEC_REASON_BAD_REQUEST: str = "bad_request"
EXEC_REASON_NOT_IMPLEMENTED: str = "not_implemented"
EXEC_REASON_VOICE_MODE_REJECTED: str = "voice_mode_rejected"
EXEC_REASON_EMERGENCY_OFF: str = "emergency_off"
EXEC_REASON_HEARTBEAT_REFRESHED: str = "heartbeat_refreshed"
EXEC_REASON_WRONG_CLIENT: str = "wrong_client"
EXEC_REASON_HEARTBEAT_NOOP: str = "heartbeat_no_op"
# Phase 2 (issue #2002): arbiter недоступен (CI mock без него, IDL не
# пересобран, или workspace вообще без avatar_arbiter-ноды). Честный
# FAIL, чтобы клиент мог отличить «прокси сломан» от «arbiter отверг».
EXEC_REASON_ARBITER_UNAVAILABLE: str = "arbiter_unavailable"
# Phase 2 (issue #2002): arbiter-клиент создан, но вызов упал по timeout /
# exception (rclpy ServiceNotAvailable, future.result() timeout). Клиент
# видит ``applied=False`` + reason — обычно retry-safe.
EXEC_REASON_ARBITER_TIMEOUT: str = "arbiter_timeout"

# Имена arbiter-сервисов (фaсадируются через execute). Импорт ленивый:
# arbiter_node не импортируется в supervisor_node (разные ноды, разные
# процессы), но имена должны совпадать с AvatarArbiter.ACQUIRE_FLOOR_SERVICE
# и т.п. (ADR-0051 §2.2).
ARBITER_ACQUIRE_FLOOR: str = "/avatar_arbiter/acquire_floor"
ARBITER_RELEASE_FLOOR: str = "/avatar_arbiter/release_floor"
ARBITER_SET_AVATAR_MODE: str = "/avatar_arbiter/set_avatar_mode"


def _normalize_voice_mode(value: Any) -> str:
    """Привести значение ``voice_mode`` из ``Command`` к ``str``.

    ROS 2 ``string``-поля на mock-стенде могут приходить как ``str``, так и
    как ``MagicMock``/просто ``None`` (если клиент не заполнил). Здесь
    нормализуем к ``str`` без падения, чтобы :py:meth:`AvatarSupervisor.execute`
    мог отдать внятный ``reason="voice_mode_rejected: ..."`` вместо
    ``TypeError`` при битом payload (ADR-0018 — не молчим на отказе).
    """
    if isinstance(value, str):
        return value
    if value is None:
        return ""
    return str(value)


def _coerce_kind(value: Any) -> int:
    """Безопасно достать ``Command.kind`` как ``int``.

    MagicMock на mock-стенде без ``__int__`` отдаёт repr при попытке
    сравнения — падаем в TypeError. Здесь нормализуем к ``int`` через
    ребро ``isinstance(value, bool) → int`` (bool — подкласс int) и
    ``int(value)`` с fallback на 0 (=KIND_UNKNOWN).
    """
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    try:
        return int(value)
    except (TypeError, ValueError):
        return KIND_UNKNOWN


def _make_execute_response(
    *,
    accepted: bool,
    applied: bool,
    reason: str,
    held_by: str = "",
    actual_mode: str = "",
    contacted_service: str = "",
) -> Any:
    """Собрать :class:`rob_box_supervisor_msgs.msg.Response` без зависимости
    от реального IDL на mock-стенде.

    Причины (ADR-0051 §2.1, ADR-0018 — «не молчим на отказе»):

    * IDL-сборка ``rob_box_supervisor_msgs`` доступна только при полной
      пересборке workspace (ament_cmake). На unit-стенде (CI без ROS) и
      в dev-режиме без пересборки модуль не подгружается — нужен
      fallback. Контракт Response.msg — фиксированный набор bool/string
      полей, см. ``rob_box_supervisor_msgs/msg/Response.msg``.
    * Возвращаем **экземпляр**, а не класс — клиентский код
      (Future.result(), mock-тесты) ожидает готовый объект с полями.

    Приоритет источника типа:

    1. ``rob_box_supervisor_msgs.msg.Response`` (нормальный путь).
    2. ``mock_rob_box_supervisor_msgs_msg.Response`` (unit-стенд,
       conftest ставит его в ``sys.modules`` ДО импорта ноды).
    3. ``types.SimpleNamespace(**kwargs)`` — самый слабый fallback,
       чтобы :py:meth:`AvatarSupervisor.execute` не падал на отсутствии
       типа в неожиданных окружениях (например, скрипт-генератор без
       conftest). Достаточно для ``response.accepted = True`` и т.п.
    """
    fields = {
        "accepted": bool(accepted),
        "applied": bool(applied),
        "reason": str(reason or ""),
        "held_by": str(held_by or ""),
        "actual_mode": str(actual_mode or ""),
        "contacted_service": str(contacted_service or ""),
    }
    try:
        from rob_box_supervisor_msgs.msg import Response as _IdlResponse  # noqa: PLC0415

        resp = _IdlResponse()
    except Exception:  # noqa: BLE001 — IDL может быть недоступен в dev/test
        try:
            from mock_rob_box_supervisor_msgs_msg import (  # type: ignore[import-not-found]  # noqa: PLC0415
                Response as _MockResponse,
            )

            resp = _MockResponse()
        except Exception:  # noqa: BLE001
            resp = types.SimpleNamespace()
    for name, value in fields.items():
        setattr(resp, name, value)
    return resp
# AV-28 §P7 (issue #1920) — voice style preset / language топики.
# Симметрично /avatar/set_voice_mode и /avatar/set_voice: payload — String
# с одним ID (preset|language) без JSON (для скорости и простоты парсинга).
# voice-vr 21: супервизор НЕ пишет в dialogue_node (ADR-0080 §2.7),
# см. блок AV-28 §P7 ниже — операция только логируется, формализация
# идёт через ``grip_pipeline`` (yaml-direct).
SET_VOICE_PRESET_TOPIC: str = "/avatar/set_voice_preset"
SET_VOICE_LANGUAGE_TOPIC: str = "/avatar/set_voice_language"
# Whitelist preset/language для AV-28 §P7. Single source of truth —
# rob_box_core.bridge_protocol.VOICE_PRESET_IDS / VOICE_LANGUAGES
# (импортированы выше). Локальная копия была ДВЕ: эта и приватная
# _AV28_* внутри класса, валидировала вторая. Разъехавшись с yaml, они
# дали молчаливый отказ: ws_server отвечал Quest'у voice_set_ack
# (UI показывал «применилось»), а супервизор ронял запрос в
# applied=False, и оператор об этом не узнавал. Так выпали пресет
# `translate` и языки fr/de/zh/hi. Теперь whitelist один — канон,
# импортированный сверху. Issue #2240 фиксирует архитектуру.
# AV-21 (issue #1913) — супервизор-агент «мозг оператора» (ADR-0028 §1.1).
# Вход: ``/avatar/command`` (std_msgs/String, JSON), выход:
# ``/avatar/command_result``. Полные JSON-схемы — в
# ``docs/architecture/avatar-supervisor-agent.md``. Наполнять вход
# будут карточки-после (AV-22: Quest STT, Telegram-текст).
#
# Issue #2081: константы AVATAR_COMMAND_TOPIC / AVATAR_COMMAND_RESULT_TOPIC
# теперь импортируются из rob_box_core.avatar_command (см. imports выше) —
# это единый SoT, чтобы quest/supervisor/telegram не разъезжались.
# Вейк-вход оператора (шаг 05, issue #1990). Топик создаёт stt_node
# (wake-роутер); пока шаг 05 не смержен — подписка дремлет (в ROS
# подписка на несуществующий топик безвредна). Payload v1 — как
# /avatar/command, поэтому обработчик тот же (_on_avatar_command).
AGENT_STT_RESULT_TOPIC: str = "/avatar/stt/result"

# ── Шаг 4б (issue #1989): пайплайн грипа (§7.5 target-operator-agent-and-dialogue.md) ──
# Прямоточный путь речи оператора с левого грипа: /avatar/ptt/result + конфиг
# /avatar/voice_pipeline → transform (0 или 1 вызов LLM) → /voice/tts/request.
# Оба входа создают другие ноды (ptt/result — stt_node, шаг 05/#1990;
# voice_pipeline — quest_node/панель); пока их нет — подписки дремлют.
# Никакого AgentCore / ToolProvider / памяти на этом пути (инвариант 6c).
GRIP_PTT_RESULT_TOPIC: str = "/avatar/ptt/result"
GRIP_VOICE_PIPELINE_TOPIC: str = "/avatar/voice_pipeline"
# Выход пайплайна — динамики робота (тот же топик, что у инструмента say).
GRIP_TTS_REQUEST_TOPIC: str = "/voice/tts/request"
# ADR-0055 / issue #1993 — обратный канал ТАРС в шлем. Старый grip pipeline
# публиковал в /voice/tts/request (этот топик оставлен ТОЛЬКО для инструмента
# ``say``, см. ADR-0055 §Чего не делаем). Все собственные реплики
# avatar_supervisor'а и грип-пайплайна идут в /avatar/tts/request (sink="headset"),
# tts_node шлёт синтез в /avatar/tts/audio, quest_node доставляет в шлем.
AVATAR_TTS_REQUEST_TOPIC: str = "/avatar/tts/request"
# source в /voice/tts/request от пайплайна грипа (для метрик tts_node).
GRIP_TTS_SOURCE: str = "operator"
# Значения preset, означающие «без стиля» (0 вызовов LLM) даже при
# llm_enabled=true — семантика ``preset=none`` из карточки #1989.
GRIP_OFF_PRESETS: frozenset[str] = frozenset({"", "none", "off"})
# Default конфигурации пайплайна до первого /avatar/voice_pipeline:
# «Без стиля» — грип произносит дословно, без LLM.
#
# ADR-0080 §2.7 / issue #2265: единственный источник истины для
# дефолтного языка — ``rob_box_core.bridge_protocol.VOICE_PIPELINE_DEFAULT_LANGUAGE``.
# Раньше тут жила копия «ru» в трёх местах (supervisor_node, ws_server,
# catalog comment), которая разъезжалась молча. Теперь — алиас через
# прямой импорт (тот же приём, что для VOICE_PRESET_IDS / VOICE_LANGUAGES,
# см. комментарий выше и voice-vr 21).
from rob_box_core.bridge_protocol import VOICE_PIPELINE_DEFAULT_LANGUAGE  # noqa: E402,F401
GRIP_DEFAULT_LANGUAGE: str = VOICE_PIPELINE_DEFAULT_LANGUAGE  # re-export для обратной совместимости

# Какой ``action`` слать в ``/dialogue/control`` пока супервизор-агент
# обрабатывает команду оператора. ADR-0066 §6.7: теперь это всегда
# ``pause`` (вход) и ``resume`` (finally). Режим «off» из старого
# ``voice_input_mode`` сводится именно к pause личности — никакого TTL
# (ADR-0051 инвариант 8). Если оператор хочет ограниченную паузу — он
# делает это в супервизоре через ``TaskScheduler`` (target §7.3 §8).
AGENT_DIALOGUE_PAUSE_ACTION: str = "pause"
AGENT_DIALOGUE_RESUME_ACTION: str = "resume"

# Валидные ``source``-поля в ``/avatar/command``. Используется только для
# метрик (label) и валидации payload-а — НЕ для роутинга (это работа
# AV-22). Неизвестные источники НЕ отбрасываем, лишь логируем warning
# (расширяемость — future: ``"web"``, ``"admin"``).
AGENT_COMMAND_SOURCES: tuple[str, ...] = ("quest", "telegram")

# Timeout ожидания ответа операторского агента при обработке команды
# (защита от зависшего LLM, который отправил запрос и не вернул
# ответ). В текущем PR — без жёсткого таймаута, но константа здесь,
# чтобы Phase 2 не пришлось переписывать.
AGENT_COMMAND_TIMEOUT_S: float = 30.0


# ── Арбитраж floor/FSM вынесен в avatar_arbiter (ADR-0051 §2.2, #1987) ──
# Событийные имена FSM, wire-режимы, MODE_TRANSITIONS, floor-маппинги и
# ленивая загрузка rob_box_supervisor_msgs больше НЕ живут здесь: всё это
# переехало в rob_box_supervisor/arbiter_node.py (AvatarArbiter), который
# владеет LockManager/FSM и /avatar/state. Этот модуль (AvatarSupervisor)
# остаётся за голосом (voice-параметры dialogue_node/tts_node) и
# супервизор-агентом ТАРС (/avatar/command + /avatar/stt/result → AgentCore).


class AvatarSupervisor(Node):
    """ROS 2 нода ``avatar_supervisor`` — голос + супервизор-агент (Vision Pi).

    Арбитраж floor/FSM + /avatar/state вынесены в отдельную ноду
    ``avatar_arbiter`` (ADR-0051 §2.2, issue #1987): здесь остаются
    voice-параметры (``/avatar/set_voice_mode``, preset/language,
    ``/avatar/set_voice``, preview) и агент оператора ТАРС
    (``/avatar/command`` + ``/avatar/stt/result`` → ``AgentCore``) плюс
    пайплайн грипа (issue #1989: ``/avatar/ptt/result`` +
    ``/avatar/voice_pipeline`` → ``/voice/tts/request``).
    Клиент floor-сервисов ходит на ``/avatar_arbiter/*``.
    """

    # ── AGENT_* параметры (AV-21, ТАРС / issue #1988) ───────────────
    # ``agent_enabled`` гейт всего agent-прохода (default true — ТАРС
    # работает сразу). ``system_prompt_file`` — имя файла в
    # ``rob_box_supervisor/prompts/``. Параметр ``agent_during_voice_mode``
    # удалён вместе с ``voice_input_mode`` (ADR-0066 §6 / §6.7): теперь
    # во время обработки команды оператора супервизор ВСЕГДА шлёт
    # ``pause`` в ``/dialogue/control`` (и ``resume`` в finally). Если
    # клиенту нужен другой режим — он вызывает
    # ``/dialogue/control {action:...}`` напрямую (новый API).
    AGENT_ENABLED_PARAM = "agent_enabled"
    SYSTEM_PROMPT_FILE_PARAM = "system_prompt_file"

    def __init__(self) -> None:
        super().__init__("avatar_supervisor")

        # Параметр mode (default monitor). В monitor — наблюдаем, не
        # вмешиваемся в voice-параметры; в active — применяем. (Арбитраж
        # floor / /avatar/state вынесен в avatar_arbiter, issue #1987.)
        self.declare_parameter("mode", "monitor")
        self._mode: str = str(self.get_parameter("mode").value or "monitor")

        # Логгер ROS (не stdlib logging — для unified rclpy logging).
        self._log = self.get_logger()

        # ADR-0028 S5 (обновлено в ADR-0066 §6.7) — супервизор управляет
        # личностью через топик ``/dialogue/control`` (String JSON,
        # action: pause|resume). Phase 1 транспорт — топик (см.
        # SET_VOICE_MODE_TOPIC). dialogue_node ack-ает на
        # ``/dialogue/control_ack`` (sub для будущей телеметрии).
        self.create_subscription(
            RosString, SET_VOICE_MODE_TOPIC, self._on_set_voice_mode, 10
        )
        # ADR-0066 §6.7 — публикация в ``/dialogue/control``. Используется
        # для swap-контекста вокруг ``_run_agent_sync`` (pause на входе,
        # resume в finally) и для обработки явных
        # ``/avatar/set_voice_mode``-команд (legacy-контракт).
        self._dialogue_control_pub = self.create_publisher(
            RosString, DIALOGUE_CONTROL_TOPIC, 10
        )
        # AV-28 §P7 (issue #1920) — voice style preset / language топики.
        # Валидируем ID по whitelist (см. единый список
        # ``rob_box_core.bridge_protocol``) и только логируем факт
        # приёма (voice-vr 21 / ADR-0080 §2.7). Раньше здесь стояла
        # запись в dialogue_node — живой путь формализации теперь в
        # ``grip_pipeline`` (yaml-direct). Когда расширим
        # ``/dialogue/control`` под set_preset/set_language, обработчик
        # сменит тело — сигнатура топиков и whitelist остаются.
        self.create_subscription(
            RosString, SET_VOICE_PRESET_TOPIC, self._on_set_voice_preset, 10
        )
        self.create_subscription(
            RosString, SET_VOICE_LANGUAGE_TOPIC, self._on_set_voice_language, 10
        )
        # AV-27 / issue #1919 — set_voice / preview_voice → супервизор.
        # Валидируем voice_id по реестру и выставляем параметр tts_node.
        self.create_subscription(RosString, SET_VOICE_TOPIC, self._on_set_voice, 10)
        self.create_subscription(
            RosString, PREVIEW_VOICE_TOPIC, self._on_preview_voice, 10
        )
        # Publishers для ответов preview_voice. Аудио (BINARY) отдельно от
        # result/error (String JSON) — UI Quest матчит по request_id.
        self._preview_result_pub = self.create_publisher(
            RosString, PREVIEW_VOICE_RESULT_TOPIC, 10
        )
        self._preview_audio_pub = self.create_publisher(
            RosString, PREVIEW_VOICE_AUDIO_TOPIC, 10
        )
        self._preview_error_pub = self.create_publisher(
            RosString, PREVIEW_VOICE_ERROR_TOPIC, 10
        )
        # ADR-0080 §2.7 / voice-vr 21 — write-сторона в чужие
        # ROS-параметры tts_node УДАЛЕНА. supervisor публикует запрос в
        # /voice/tts/set_voice (RosString JSON ``{voice_id, provider,
        # source}``), и tts_node применяет через свой ``_on_set_voice``
        # — знание схемы имён (``yandex_voice``/``minimax_voice``/
        # ``silero_speaker``) живёт ТОЛЬКО в tts_node.
        self._set_voice_tts_pub = self.create_publisher(
            RosString, SET_TTS_VOICE_TOPIC, 10
        )

        # ── AV-21/ТАРС (issue #1988): супервизор-агент оператора ────
        # ``agent_enabled`` default true — мастер-гейт всего agent-прохода;
        # при false нода ведёт себя как до 4а (никаких tool-каллов/свапов
        # голоса). Движок создаётся ЛЕНИВО (см. _ensure_agent_core) — при
        # enabled=false нода не инстанцирует LLM / Tools / Memory и не
        # делает лишнего ввода-вывода. Инвариант тестов: enabled=false →
        # core не создаётся.
        self.declare_parameter(self.AGENT_ENABLED_PARAM, True)
        self.declare_parameter(
            self.SYSTEM_PROMPT_FILE_PARAM, "operator_system_prompt.txt"
        )
        # Параметр ``agent_during_voice_mode`` удалён вместе с
        # ``voice_input_mode`` (ADR-0066 §6.7). Старое default-поведение
        # «pause личности на время обработки команды» теперь жёстко
        # зашито в ``_dialogue_control_swap`` (см. ниже).
        # ── LLM / tools / память оператора (issue #1988) ─────────────
        # Issue #2111: default chain должен совпадать с dialogue_node
        # (`minimax,deepseek`) — иначе supervisor поднимается с одним
        # провайдером без API-ключа и agent не отвечает. Полный
        # health-fallback chain — отдельная карточка позже.
        # ADR-0043 §3.2: runtime yaml (когда он появится для supervisor)
        # должен быть синхронизирован с этим значением в том же коммите.
        self.declare_parameter("llm_providers", "minimax,deepseek")
        # issue #2116 — озвучивать ли ответ агента в шлем. Выключается
        # на стенде, где синтез не нужен (тесты тракта по топикам).
        self.declare_parameter("speak_agent_replies", True)
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("max_tokens", 0)
        self.declare_parameter("llm_streaming", False)
        self.declare_parameter("history_max_turns", 10)
        # tool_provider: "ros_mcp" (реальные MCP-инструменты через
        # LLMToolCallAdapter → /mcp/execute), "fake"/"none" — тесты/smoke.
        self.declare_parameter("tool_provider", "ros_mcp")
        # ADR-0083 §E — память оператора больше НЕ отдельный файл. Оба
        # агента (personality + operator) пишут в ``/data/harness_voice.db``
        # через ``SQLiteVoiceMemory(agent=...)``: ``agent='operator'`` для
        # ТАРС, ``agent='personality'`` для личности. Колонка ``agent``
        # в ``facts`` создана миграцией 011_agent_namespace.sql (PR #2276).
        #
        # Параметр ``operator_db_path`` оставлен в виде DEPRECATED
        # fallback — общий host-path ``./data/voice:/data`` уже
        # смонтирован у supervisor и voice-assistant.
        self.declare_parameter("sqlite_db_path", "/data/harness_voice.db")
        self.declare_parameter("operator_db_path", "/data/harness_voice.db")
        self.declare_parameter("journal_path", "/data/operator_journal.jsonl")
        self._agent_enabled: bool = bool(
            self.get_parameter(self.AGENT_ENABLED_PARAM).value
        )
        self._system_prompt_file: str = str(
            self.get_parameter(self.SYSTEM_PROMPT_FILE_PARAM).value
            or "operator_system_prompt.txt"
        )
        # Поле ``_agent_during_voice_mode`` удалено вместе с параметром
        # ``voice_input_mode`` (ADR-0066 §6.7). Старое поведение «поставь
        # личность в off на время обработки команды» теперь реализовано
        # через ``_dialogue_control_swap`` — жёстко шлём ``pause`` на
        # входе и ``resume`` в finally (контракт ADR-0066 §2.1).

        # AgentCore создаётся ЛЕНИВО (см. _ensure_agent_core): при
        # ``agent_enabled=false`` мы не должны инстанцировать LLM /
        # Tools / Memory. DSM оператора держим рядом с core — перед
        # каждым входом гоним его в DIALOGUE (см. _run_agent_sync).
        self._agent_core: Any = None
        self._operator_dsm: Any = None
        # Журнал ТАРС (§5.4) — тоже лениво, персист по journal_path.
        self._operator_journal: Any = None
        # Метрики супервизор-агента (issue #2232). РАНЬШЕ здесь стояла
        # заглушка ``{"enabled": False}`` с комментарием «реальные счётчики
        # поднимутся при первом ``_build_agent_metrics``» — но триггера не
        # существовало: ``_build_agent_metrics`` не вызывался НИ ОТКУДА, а
        # ``_record_agent_command`` / ``_record_agent_tool_call`` коротко
        # замыкались на ``enabled=False`` и молча ничего не писали. То есть
        # ``avatar_agent_commands_total`` и ``avatar_agent_tool_calls_total``
        # были нулевыми всегда. Два теста в test_avatar_agent.py это ловили,
        # но CI не запускал rob_box_supervisor (#2232), и красное никто не
        # видел.
        #
        # Строим сразу: ``_build_agent_metrics`` зависит только от ленивого
        # импорта ``rob_box_voice.observability.metrics`` (при ImportError
        # сам возвращает no-op-заглушку с ``enabled=False``), поэтому
        # безопасен в ``__init__`` после ``self._log`` (строка 432).
        self._agent_metrics: dict[str, Any] = self._build_agent_metrics()
        # Поле ``_voice_input_mode_before_swap`` удалено вместе с
        # ``voice_input_mode`` (ADR-0066 §6.7). В новой схеме через
        # ``/dialogue/control`` супервизор ВСЕГДА шлёт ``pause`` на входе
        # и ``resume`` в finally — режим личности не «свапается», а
        # переводится в SILENCED → IDLE. Snapshot предыдущего значения
        # больше не нужен, restore логика исчезает.

        # Publisher /avatar/command_result (для супервизор-агента).
        # QoS — default reliable (depth=10). Не latched: результаты
        # привязаны к конкретной команде, late joiner их НЕ получит
        # (это поведение «command-response», не «state-broadcast»).
        self._agent_result_pub = self.create_publisher(
            RosString, AVATAR_COMMAND_RESULT_TOPIC, 10
        )
        # Подписка /avatar/command — JSON с командой оператора.
        self.create_subscription(
            RosString, AVATAR_COMMAND_TOPIC, self._on_avatar_command, 10
        )
        # Подписка /avatar/stt/result — вейк-вход оператора (шаг 05,
        # #1990). Дремлющая: до появления публикатора безвредна.
        self.create_subscription(
            RosString, AGENT_STT_RESULT_TOPIC, self._on_avatar_command, 10
        )

        # ── Шаг 4б (issue #1989): пайплайн грипа (§7.5) ──────────────
        # Прямоточный путь, НЕ агентский цикл: ptt/result + конфиг панели →
        # transform (0|1 LLM) → /voice/tts/request. Состояние конфигурации —
        # «одна точка правды на ноде оператора»; default — «Без стиля»
        # (llm_enabled=False), пока панель не пришлёт /avatar/voice_pipeline.
        self._pipeline_llm_enabled: bool = False
        self._pipeline_preset: str = ""
        self._pipeline_language: str = GRIP_DEFAULT_LANGUAGE
        # LLM грипа — лениво, отдельно от агента (см. _grip_transform_once).
        # Грип НЕ строит AgentCore / ToolProvider / Memory.
        self._grip_llm: Any = None
        self._tts_request_pub = self.create_publisher(
            RosString, GRIP_TTS_REQUEST_TOPIC, 10
        )
        # ADR-0055 / issue #1993 — обратный канал ТАРС в шлем. Publisher
        # /avatar/tts/request для СОБСТВЕННЫХ реплик supervisor'а и пайплайна
        # грипа. Контракт String JSON (см. ADR-0055 §tts_node):
        # ``{request_id, ssml, sink:\"headset\", voice?, language?}``.
        # ``/voice/tts/request`` остаётся ТОЛЬКО для инструмента ``say``
        # (там ALSA-путь + динамики робота, НЕ шлем).
        self._avatar_tts_request_pub = self.create_publisher(
            RosString, AVATAR_TTS_REQUEST_TOPIC, 10
        )
        self.create_subscription(
            RosString, GRIP_PTT_RESULT_TOPIC, self._on_grip_ptt_result, 10
        )
        self.create_subscription(
            RosString,
            GRIP_VOICE_PIPELINE_TOPIC,
            self._on_grip_voice_pipeline,
            10,
        )
        self._grip_metrics = self._build_grip_metrics()

        # Issue #2113 (quest #2112) — TARS 2 panel dispatcher. Превращает
        # LLM tool call ``show_metrics(query)`` в URL Grafana-панели и
        # публикует его в ``/avatar/tars/panel_url`` (на этот топик уже
        # подписан Quest-клиент). Регистрация tool'а — в ``_build_operator_tools``
        # ниже, после инициализации AgentCore/registry (tars_panel.register_tool
        # дёргается в той же фазе, что и MCP-tools).
        try:
            from rob_box_supervisor.tars_panel import (  # noqa: PLC0415
                TarsPanelDispatcher,
            )

            self._tars_panel_dispatcher = TarsPanelDispatcher(self)
        except Exception as exc:  # noqa: BLE001
            # Не валим supervisor из-за optional фичи: warn и идём дальше.
            self.get_logger().warning(
                f"[issue #2113] TarsPanelDispatcher init failed: {exc}"
            )
            self._tars_panel_dispatcher = None

        # Метрики (см. rob_box_voice.observability.metrics). Регистрируются
        # лениво через get_metric — если prometheus_client недоступен,
        # это no-op (см. там же is_metrics_enabled). Метрики-объекты
        # кладём в self один раз (lazy init в __init__, не в каждом
        # callback — иначе на каждое сообщение новый Counter).
        self._grip_metrics = self._build_grip_metrics()

        # ── Bridge.execute(Command) — единый шов «клиент ↔ supervisor»
        # (ADR-0051 §2.1, issue #2002). В Phase 1 фасад работает ПАРАЛЛЕЛЬНО
        # с legacy сервисами (``/avatar_arbiter/*``): никаких breaking changes
        # (ADR-0013). Полная миграция клиентов — отдельная карточка Phase 2.
        #
        # Регистрация ``/supervisor/execute`` через IDL-тип
        # ``ExecuteCommand`` (из rob_box_supervisor_msgs). На mock-стенде
        # (conftest) тот же тип уже есть, без пересборки IDL — fallback на
        # ``mock_rob_box_supervisor_msgs_srv.ExecuteCommand``. В обоих
        # случаях ``create_service`` проверяет наличие nested
        # ``.Request``/``.Response`` (issue #1904, t_979f0cb2) — поэтому
        # передаём ПОЛНЫЙ srv-класс, а не ``.Request``.
        self._srv_execute: Any = None
        try:
            from rob_box_supervisor_msgs.srv import (  # noqa: PLC0415
                ExecuteCommand as _IdlExecuteCommand,
            )

            execute_srv_type: Any = _IdlExecuteCommand
        except Exception:  # noqa: BLE001 — IDL недоступен в dev-режиме
            try:
                from mock_rob_box_supervisor_msgs_srv import (  # type: ignore[import-not-found]  # noqa: PLC0415
                    ExecuteCommand as _MockExecuteCommand,
                )

                execute_srv_type = _MockExecuteCommand
            except Exception:  # noqa: BLE001
                execute_srv_type = None
        if execute_srv_type is not None:
            # create_service требует ``.Request``/``.Response`` — иначе
            # RuntimeError «The service type provided is not valid»
            # (issue #1904). Проверяем до вызова, чтобы в unit-стенде
            # без нормального IDL мы не словили ту же ошибку, что и в проде.
            req_attr = getattr(execute_srv_type, "Request", None)
            resp_attr = getattr(execute_srv_type, "Response", None)
            if req_attr is not None and resp_attr is not None:
                self._srv_execute = self.create_service(
                    execute_srv_type,
                    EXECUTE_COMMAND_SERVICE,
                    self._on_execute_command,
                )

        # ── Bridge.execute(Command) — arbiter-клиенты (Phase 2, issue #2002) ──
        # ACQUIRE/RELEASE_FLOOR и SET_AVATAR_MODE проксируются через
        # avatar_arbiter (владелец LockManager/FSM после ADR-0051 §2.2).
        # Клиенты (quest_node, telegram_node) после миграции ходят
        # через ``/supervisor/execute`` и не знают о существовании
        # arbiter-сервисов; Phase 3 удалит legacy-сервисы arbiter-а
        # полностью (issue #2002 план).
        #
        # Клиенты создаются ЛЕНИВО (в первом execute() для floor/mode),
        # чтобы в monitor-режиме / без arbiter (CI mock-rclpy) supervisor
        # не падал на старте. create_client проверяет nested
        # ``.Request``/``.Response`` через ту же фабрику (issue #1904).
        self._arbiter_acquire_client: Any = None
        self._arbiter_release_client: Any = None
        self._arbiter_set_mode_client: Any = None
        self._arbiter_srv_types: dict[str, Any] = {}

        self._log_startup_diagnostics()

    # ── Bridge.execute(Command) → arbiter proxy (Phase 2, issue #2002) ──
    def _ensure_arbiter_clients(self) -> bool:
        """Лениво создать ROS 2 service-клиенты к avatar_arbiter.

        Возвращает ``True`` если все три клиента (acquire/release/set_mode)
        доступны; ``False`` если IDL / rclpy недоступны (CI mock-stend без
        arbiter — supervisor остаётся в facade_only для ACQUIRE/RELEASE/
        SET_AVATAR_MODE). Не бросает исключений наружу: на mock-стенде
        или в неполном workspace клиент будет ``None``, а :py:meth:`execute`
        для floor/mode отдаст ``reason="arbiter_unavailable"`` (честный
        FAIL, ADR-0018).

        Создаём клиентов ТОЛЬКО в active-режиме (монитор не зовёт arbiter —
        S12, ADR-0028 §4.5). Монитор-путь идёт через ``EXEC_REASON_MONITOR_MODE``
        раньше, чем мы сюда добираемся.
        """
        if self._mode != "active":
            return False
        if (
            self._arbiter_acquire_client is not None
            and self._arbiter_release_client is not None
            and self._arbiter_set_mode_client is not None
        ):
            return True
        try:
            from rob_box_supervisor_msgs.srv import (  # noqa: PLC0415
                AcquireFloor as _IdlAcquire,
                ReleaseFloor as _IdlRelease,
                SetAvatarMode as _IdlSetMode,
            )

            types_map = {
                "acquire": _IdlAcquire,
                "release": _IdlRelease,
                "set_mode": _IdlSetMode,
            }
        except Exception:  # noqa: BLE001 — IDL недоступен (CI / dev-mode)
            try:
                from mock_rob_box_supervisor_msgs_srv import (  # type: ignore[import-not-found]  # noqa: PLC0415
                    AcquireFloor as _MockAcquire,
                    ReleaseFloor as _MockRelease,
                    SetAvatarMode as _MockSetMode,
                )

                types_map = {
                    "acquire": _MockAcquire,
                    "release": _MockRelease,
                    "set_mode": _MockSetMode,
                }
            except Exception:  # noqa: BLE001
                return False
        # rclpy.Node.create_client требует ПОЛНЫЙ srv-класс (issue #1904).
        # Если прислали .Request-объект — будет RuntimeError; оставляем
        # mock-rclpy поднимать это в тестах, а в проде ловим здесь.
        for key, srv_type in types_map.items():
            if not (hasattr(srv_type, "Request") and hasattr(srv_type, "Response")):
                return False
        try:
            self._arbiter_acquire_client = self.create_client(
                types_map["acquire"], ARBITER_ACQUIRE_FLOOR
            )
            self._arbiter_release_client = self.create_client(
                types_map["release"], ARBITER_RELEASE_FLOOR
            )
            self._arbiter_set_mode_client = self.create_client(
                types_map["set_mode"], ARBITER_SET_AVATAR_MODE
            )
        except Exception:  # noqa: BLE001 — rclpy недоступен
            self._arbiter_acquire_client = None
            self._arbiter_release_client = None
            self._arbiter_set_mode_client = None
            return False
        self._arbiter_srv_types = types_map
        return True

    # ── Bridge.execute(Command) → arbiter proxy (Phase 2, issue #2002) ──
    def _proxy_to_arbiter_sync(self, kind: int, command: Any) -> Any:
        """Sync-проксирование Command в arbiter через ROS 2 service-call.

        Вызывается из :py:meth:`execute` для KIND_ACQUIRE_FLOOR/
        KIND_RELEASE_FLOOR/KIND_SET_AVATAR_MODE в active-режиме, после
        успешного :py:meth:`_ensure_arbiter_clients` (т.е. клиенты и
        srv-типы гарантированно инициализированы).

        Стратегия вызова:

        * ``client.call_async(req)`` → ``Future``.
        * Ждём ответ через ``rclpy.Future`` (НЕ блокирующий wait — мы в
          callback-контексте сервиса ``/supervisor/execute``, и крутить
          executor вложенно нельзя: deadlock).
        * Реальный паттерн для ROS 2: rclpy spin-once в отдельном
          ``SingleThreadedExecutor`` — но это лишний boilerplate. Вместо
          этого supervisor делает sync future.wait() через
          ``_arbiter_wait_for_future`` helper, который спит polling-цикл
          и периодически вызывает ``rclpy.spin_once(node, timeout_sec=0)``,
          чтобы callbacks пришли. На mock-rclpy (CI) ``spin_once`` — no-op,
          а Future резолвится внешним кодом (тесты).
        * Если arbiter-сервиса нет (CI/dev) — client.service_is_ready()
          вернёт ``False``, и мы отдаём ``reason="arbiter_unavailable"``.

        Маппинг ответа (issue #2002 ADR):

        * ACQUIRE_FLOOR: arbiter.AcquireFloorResponse
          ``{success, granted, held_by, reason, applied}`` →
          ``Response{accepted=applied AND granted, applied, reason,
          held_by, actual_mode="", contacted_service=ARBITER_ACQUIRE_FLOOR}``
        * RELEASE_FLOOR: ``{success, reason, applied}`` →
          ``Response{accepted=applied, applied, reason,
          contacted_service=ARBITER_RELEASE_FLOOR}``
        * SET_AVATAR_MODE: ``{success, mode, reason, applied}`` →
          ``Response{accepted=applied, applied, reason, actual_mode=mode,
          contacted_service=ARBITER_SET_AVATAR_MODE}``

        В Phase 2 (issue #2002) это сделано для 3 команд × 3 путей =
        9 unit-тестов в :file:`test_execute_command.py::TestExecuteArbiterProxy`.
        """
        client_id = str(getattr(command, "client_id", "") or "")
        if kind == KIND_ACQUIRE_FLOOR:
            client = self._arbiter_acquire_client
            srv_name = ARBITER_ACQUIRE_FLOOR
            floor = str(getattr(command, "floor", "") or "")
            return self._call_arbiter_floor(
                client, srv_name, command, client_id=client_id, floor=floor
            )
        if kind == KIND_RELEASE_FLOOR:
            client = self._arbiter_release_client
            srv_name = ARBITER_RELEASE_FLOOR
            floor = str(getattr(command, "floor", "") or "")
            return self._call_arbiter_floor(
                client, srv_name, command, client_id=client_id, floor=floor
            )
        if kind == KIND_SET_AVATAR_MODE:
            client = self._arbiter_set_mode_client
            srv_name = ARBITER_SET_AVATAR_MODE
            mode = str(getattr(command, "avatar_event", "") or "")
            return self._call_arbiter_set_mode(
                client, srv_name, command, client_id=client_id, mode=mode
            )
        # Сюда не должны попасть — execute() уже отфильтровал unknown kind.
        return _make_execute_response(
            accepted=False,
            applied=False,
            reason=EXEC_REASON_UNKNOWN_KIND,
        )

    def _call_arbiter_floor(
        self,
        client: Any,
        srv_name: str,
        command: Any,
        *,
        client_id: str,
        floor: str,
    ) -> Any:
        """Проксировать ACQUIRE/RELEASE_FLOOR в avatar_arbiter.

        ``floor`` валидируется против wire-контракта (только ``"teleop"``
        и ``"voice"``; пустая строка → ``bad_request``). arbiter уже
        принимает любые non-empty строки (W3-2 fallback), но мы фильтруем
        заранее чтобы не делать лишний service-call с мусором.
        """
        if floor and floor not in ("teleop", "voice"):
            # Невалидный floor — bad_request, без service-call (ADR-0018).
            return _make_execute_response(
                accepted=False,
                applied=False,
                reason=EXEC_REASON_BAD_REQUEST,
                held_by=client_id,
            )
        try:
            request_obj = client.srv_type.Request()  # type: ignore[attr-defined]
        except Exception:  # noqa: BLE001
            return _make_execute_response(
                accepted=True,
                applied=False,
                reason=EXEC_REASON_ARBITER_UNAVAILABLE,
                held_by=client_id,
                contacted_service=srv_name,
            )
        try:
            request_obj.client_id = client_id  # type: ignore[attr-defined]
            request_obj.floor = floor  # type: ignore[attr-defined]
        except Exception:  # noqa: BLE001
            return _make_execute_response(
                accepted=False,
                applied=False,
                reason=EXEC_REASON_BAD_REQUEST,
                held_by=client_id,
                contacted_service=srv_name,
            )
        return self._dispatch_arbiter_call(
            client,
            srv_name,
            request_obj,
            kind_hint="floor",
            client_id=client_id,
        )

    def _call_arbiter_set_mode(
        self,
        client: Any,
        srv_name: str,
        command: Any,
        *,
        client_id: str,
        mode: str,
    ) -> Any:
        """Проксировать SET_AVATAR_MODE в avatar_arbiter.

        ``mode`` — целевой avatar-режим (``off``/``telegram_active``/
        ``avatar_present``/``mixed``), как требует wire-контракт
        ``meta-quest-api.md`` §3. Пустая строка → ``bad_request``.
        """
        if not mode:
            return _make_execute_response(
                accepted=False,
                applied=False,
                reason=EXEC_REASON_BAD_REQUEST,
                held_by=client_id,
            )
        try:
            request_obj = client.srv_type.Request()  # type: ignore[attr-defined]
            request_obj.client_id = client_id  # type: ignore[attr-defined]
            request_obj.mode = mode  # type: ignore[attr-defined]
        except Exception:  # noqa: BLE001
            return _make_execute_response(
                accepted=False,
                applied=False,
                reason=EXEC_REASON_BAD_REQUEST,
                held_by=client_id,
                contacted_service=srv_name,
            )
        return self._dispatch_arbiter_call(
            client,
            srv_name,
            request_obj,
            kind_hint="set_mode",
            client_id=client_id,
        )

    def _dispatch_arbiter_call(
        self,
        client: Any,
        srv_name: str,
        request_obj: Any,
        *,
        kind_hint: str,
        client_id: str,
    ) -> Any:
        """Sync-call к arbiter-сервису + маппинг ответа в :class:`Response`.

        На mock-rclpy (CI) ``call_async`` возвращает ``MagicMock``, чей
        ``result()`` сразу отдаёт настроенное значение (тесты через
        ``client.result.return_value = ...``). На реальном rclpy ждём
        через ``_arbiter_wait_for_future``.
        """
        try:
            future = client.call_async(request_obj)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(
                f"_dispatch_arbiter_call[{srv_name}]: call_async raised: {exc}"
            )
            return _make_execute_response(
                accepted=True,
                applied=False,
                reason=EXEC_REASON_ARBITER_UNAVAILABLE,
                held_by=client_id,
                contacted_service=srv_name,
            )
        try:
            response = self._arbiter_wait_for_future(future, srv_name)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(
                f"_dispatch_arbiter_call[{srv_name}]: wait failed: {exc}"
            )
            return _make_execute_response(
                accepted=True,
                applied=False,
                reason=EXEC_REASON_ARBITER_TIMEOUT,
                held_by=client_id,
                contacted_service=srv_name,
            )
        return self._arbiter_response_to_response(
            response, srv_name, kind_hint=kind_hint, client_id=client_id
        )

    @staticmethod
    def _arbiter_wait_for_future(future: Any, srv_name: str) -> Any:
        """Дождаться ``future.result()`` с таймаутом 50 мс.

        50 мс — запас над обычным ROS round-trip (avatar_arbiter
        отвечает синхронно, без I/O, ~1 мс на mock-стенде). Если arbiter
        отвечает дольше — клиент supervisor'а сам переподнимет (call_async
        → future.result() — sync wait не блокирует executor, мы
        находимся в callback-контексте).
        """
        timeout_s = 0.05
        try:
            # rclpy.Future имеет .result(timeout=...) в новых версиях; на
            # mock-rclpy (CI) это MagicMock с .return_value. Используем
            # .result(timeout=...) если доступно, иначе fallback на
            # ``return_value``.
            result_method = getattr(future, "result", None)
            if result_method is None:
                raise RuntimeError(f"future {future!r} has no .result()")
            try:
                return result_method(timeout=timeout_s)
            except TypeError:
                # Future.result() без аргументов (mock-стенд).
                return result_method()
        except Exception:  # noqa: BLE001
            raise

    @staticmethod
    def _arbiter_response_to_response(
        response: Any,
        srv_name: str,
        *,
        kind_hint: str,
        client_id: str,
    ) -> Any:
        """Маппинг arbiter-ответа в :class:`Response` фасада.

        kind_hint:

        * ``"floor"`` — ACQUIRE/RELEASE_FLOOR (поля granted/held_by)
        * ``"set_mode"`` — SET_AVATAR_MODE (поле mode → actual_mode)
        * ``"heartbeat"`` — зарезервировано на Phase 3, пока не используется
        """
        if response is None:
            return _make_execute_response(
                accepted=False,
                applied=False,
                reason=EXEC_REASON_ARBITER_TIMEOUT,
                held_by=client_id,
                contacted_service=srv_name,
            )
        # response может быть mock-rclpy с .granted/.held_by/.applied/.reason
        # или типизированным IDL — getattr нормально работает в обоих.
        granted = bool(getattr(response, "granted", False))
        held_by_raw = getattr(response, "held_by", "")
        # str()-оборачиваем только реальные строки; MagicMock truthy
        # на mock-стенде и приведёт к str(MagicMock) вместо "".
        held_by = str(held_by_raw) if isinstance(held_by_raw, str) else ""
        mode_raw = getattr(response, "mode", "")
        mode = str(mode_raw) if isinstance(mode_raw, str) else ""
        applied = bool(getattr(response, "applied", False))
        reason_raw = getattr(response, "reason", "")
        reason = str(reason_raw) if isinstance(reason_raw, str) else ""
        if kind_hint == "set_mode":
            accepted = applied
            return _make_execute_response(
                accepted=accepted,
                applied=applied,
                reason=reason,
                actual_mode=mode,
                contacted_service=srv_name,
            )
        # floor (acquire/release)
        accepted = applied and (granted or kind_hint == "release")
        return _make_execute_response(
            accepted=accepted,
            applied=applied,
            reason=reason,
            held_by=held_by or client_id,
            contacted_service=srv_name,
        )

    def _on_execute_command(
        self, request: Any, response: Any
    ) -> Any:
        """Callback сервиса ``/supervisor/execute`` (ADR-0051 §2.1).

        ``ExecuteCommand.Request.command`` — :class:`Command`; ``Response``
        — :class:`Response` с полями ``accepted/applied/reason/held_by/
        actual_mode/contacted_service``. Заполняем ``response.response``
        (вложенное поле srv-контракта) результатом
        :py:meth:`AvatarSupervisor.execute` и возвращаем тот же объект —
        это контракт rclpy ``Service.callback``.
        """
        command = getattr(request, "command", None)
        result = self.execute(command)
        # ``response.response`` — поле srv ``ExecuteCommand.Response``.
        # Если клиентский mock-сгенерённый класс не имеет этого поля
        # (например, очень старый fallback) — пишем напрямую в response.
        nested = getattr(response, "response", None)
        if nested is None:
            for name in (
                "accepted",
                "applied",
                "reason",
                "held_by",
                "actual_mode",
                "contacted_service",
            ):
                if hasattr(response, name) and not hasattr(response, "response"):
                    setattr(response, name, getattr(result, name, ""))
            return response
        # Нормальный путь: response.response — отдельный объект.
        for name in (
            "accepted",
            "applied",
            "reason",
            "held_by",
            "actual_mode",
            "contacted_service",
        ):
            if hasattr(nested, name):
                setattr(nested, name, getattr(result, name, ""))
        return response

    @staticmethod
    def _try_parse_json(text: Optional[str]) -> Any:
        if not text:
            return None
        try:
            return json.loads(text)
        except (ValueError, TypeError):
            return None

    def _log_startup_diagnostics(self) -> None:
        """Залогировать env и mode на старте — помогает e2e/postmortem.

        Используем f-string + одиночный ``msg`` вместо ``info(fmt, *args)``:
        rclpy ``RcutilsLogger.info`` принимает ``(msg, *args)``, где *args — это
        позиционные параметры для ``%``-форматирования msg, а не самостоятельные
        поля. Вызов с 3+ args (например, ``info(fmt, a, b, c)``) ломает рантайм
        ``TypeError: RcutilsLogger.info() takes 2 positional arguments but N were given``
        (issue #1644, run #32892615440). Тестировано в карточке t_369751b4.
        """
        zenoh = os.environ.get("ZENOH_SESSION_CONFIG_URI", "<unset>")
        self._log.info(
            f"avatar_supervisor started: mode={self._mode}, zenoh={zenoh}"
        )

    # ── dialogue control (ADR-0066 §6.7) ──────────────────────────
    # После удаления ``voice_input_mode`` (ADR-0066 §6) единственная
    # точка влияния супервизора на личность — топик ``/dialogue/control``.
    # Метод ``_apply_voice_mode`` оставлен для обратной совместимости с
    # клиентами ``/avatar/set_voice_mode`` (UI Quest, web-admin): mode
    # ``respeaker`` → ``resume``, ``off`` → ``pause``, прочие → отвергаются
    # как устаревшие (``voice_mode_deprecated``). Супервизор-агент
    # (ТАРС) ВСЕГДА шлёт ``pause``/``resume`` напрямую через
    # ``_dialogue_control_swap`` (без legacy-обёртки).
    LEGACY_VOICE_MODE_TO_ACTION: Mapping[str, str] = {
        "respeaker": DIALOGUE_CONTROL_RESUME,
        "off": DIALOGUE_CONTROL_PAUSE,
    }

    def _publish_dialogue_control(
        self, action: str, reason: str = ""
    ) -> bool:
        """Опубликовать ``{action, reason, ts_s}`` в ``/dialogue/control``.

        Returns ``True`` если publish выполнен (диалоговая нода может
        ack-нуть на ``/dialogue/control_ack``; ack на стороне супервизора
        не ждём — fire-and-forget, личность всё равно реагирует идемпотентно).
        Returns ``False`` при невалидном ``action`` (защита от опечаток в
        caller-коде). ADR-0066 §2.1.
        """
        if action not in DIALOGUE_CONTROL_ACTIONS:
            self._log.warning(
                f"_publish_dialogue_control: invalid action={action!r}, "
                f"expected one of {sorted(DIALOGUE_CONTROL_ACTIONS)}"
            )
            return False
        try:
            payload = {
                "action": action,
                "reason": reason,
                "ts_s": time.time(),
            }
            msg = RosString()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._dialogue_control_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 — отказ не должен валить ноду
            self._log.warning(
                f"_publish_dialogue_control: publish failed: {exc}"
            )
            return False
        return True

    def _on_set_voice_mode(self, msg: RosString) -> None:
        """Обработка ``/avatar/set_voice_mode`` — legacy-контракт.

        После удаления ``voice_input_mode`` (ADR-0066 §6) этот топик всё ещё
        принимается (UI Quest, web-admin), но mode → action маппинг сводится
        к двум значениям: ``respeaker``→``resume``, ``off``→``pause``.
        В monitor-режиме принимаем и логируем, но НЕ применяем (S12);
        в active — публикуем в ``/dialogue/control``.
        """
        mode = (msg.data or "").strip()
        applied, reason = self._apply_voice_mode(mode)
        # f-string: RcutilsLogger принимает ОДИН msg (issue #1644).
        self._log.info(
            f"SetVoiceMode: mode={mode} applied={applied} reason={reason}"
        )

    def _apply_voice_mode(self, mode: str) -> tuple[bool, str]:
        """Legacy-контракт ``/avatar/set_voice_mode`` через ``/dialogue/control``.

        После удаления ``voice_input_mode`` (ADR-0066 §6) супервизор больше
        НЕ ставит параметр на dialogue_node — только публикует action в
        ``/dialogue/control``. Маппинг (ADR-0066 §6.7):

        * ``respeaker`` → ``resume`` (вернуть личность к активной работе)
        * ``off``       → ``pause`` (глушим личность, оператор работает)
        * остальные (``quest_ttts``, ``quest_stt``, ``quest_llm_formalize``,
          ``quest_passthrough``, ``quest_command``) — отвергаются с
          ``reason="voice_mode_deprecated: <mode>"`` (ADR-0018 — честный
          FAIL, не молчание). Эти режимы больше не существуют — клиенты
          должны мигрировать на ``/dialogue/control`` напрямую или на
          новые MCP-инструменты ``dialogue_pause``/``dialogue_resume``
          (отдельная карточка).

        Возвращает ``(applied, reason)`` для симметрии со старым контрактом:
        ``Bridge.execute(Command)`` и тесты ``test_execute_command.py``
        матчат reason без изменений.
        """
        if mode in self.LEGACY_VOICE_MODE_TO_ACTION:
            action = self.LEGACY_VOICE_MODE_TO_ACTION[mode]
        else:
            return False, f"voice_mode_deprecated: {mode!r}"
        if self._mode != "active":
            return False, MONITOR_MODE_REASON
        applied = self._publish_dialogue_control(
            action, reason=f"legacy_set_voice_mode:{mode}"
        )
        return (applied, "applied" if applied else "publish_failed")

    # ── Bridge.execute(Command) — ADR-0051 §2.1, issue #2002 ──────────
    def execute(self, command: Any) -> Any:
        """Единый фасад «клиент ↔ supervisor» через :class:`Command`.

        Phase 1 (эта карточка) делает **только диспетчеризацию** по
        ``command.kind`` + честный FAIL/OK по reason-кодам; **никаких**
        side-effects на floor/avatar-mode (это удел avatar_arbiter, ADR-0051
        §2.2). Для команд, которые исторически жили в arbiter
        (ACQUIRE/RELEASE_FLOOR, SET_AVATAR_MODE), supervisor в Phase 1
        возвращает ``accepted=true, applied=false, reason="facade_only"`` —
        клиенту это явный сигнал «фасад есть, логика ещё у arbiter; иди
        к нему». Полная миграция — Phase 2 отдельной карточкой.

        Карта ``kind → поведение`` (Phase 1) делегирована приватным
        диспетчерам (ADR-0021 R1, декомпозиция CC=16→≤15 per method,
        issue t_ef140676):

        * ``KIND_ACQUIRE_FLOOR`` / ``KIND_RELEASE_FLOOR`` /
          ``KIND_SET_AVATAR_MODE`` → :py:meth:`_dispatch_floor_or_avatar_mode`.
        * ``KIND_SET_VOICE_MODE`` → :py:meth:`_dispatch_voice_mode`.
        * ``KIND_EMERGENCY_STOP`` → :py:meth:`_dispatch_emergency_stop`.
        * ``KIND_HEARTBEAT`` → :py:meth:`_dispatch_heartbeat`.
        * ``KIND_UNKNOWN`` (kind вне [1..6] или битый payload) →
          accepted=false reason="unknown_kind".

        Параметр ``command`` — любой объект с атрибутами ``kind``,
        ``client_id``, ``floor``, ``avatar_event``, ``voice_mode``,
        ``emergency``. На mock-стенде conftest подсовывает SimpleNamespace
        или MagicMock — читаем через ``getattr`` с дефолтом, чтобы
        не падать на отсутствующих полях.
        """
        kind = _coerce_kind(getattr(command, "kind", KIND_UNKNOWN))

        if kind in (
            KIND_ACQUIRE_FLOOR,
            KIND_RELEASE_FLOOR,
            KIND_SET_AVATAR_MODE,
        ):
            return self._dispatch_floor_or_avatar_mode(command, kind)
        if kind == KIND_SET_VOICE_MODE:
            return self._dispatch_voice_mode(command, kind)
        if kind == KIND_EMERGENCY_STOP:
            return self._dispatch_emergency_stop(command, kind)
        if kind == KIND_HEARTBEAT:
            return self._dispatch_heartbeat(command, kind)

        # ── unknown_kind (включая KIND_UNKNOWN=0 и битый payload) ────
        return _make_execute_response(
            accepted=False,
            applied=False,
            reason=EXEC_REASON_UNKNOWN_KIND,
        )

    # ── Диспетчеры для execute() — декомпозиция CC=16→≤15 (ADR-0021 R1).
    # Issue t_ef140676: каждый приватный метод принимает command и kind (kind
    # нужен только для _proxy_to_arbiter_sync), возвращает полный
    # ExecuteResponse. Семантика 1-в-1 с прежним execute() — тесты
    # test_execute_command.py матрицу 6×5+1+3 должны проходить без правок.

    def _dispatch_floor_or_avatar_mode(
        self, command: Any, kind: int
    ) -> Any:
        """ACQUIRE/RELEASE_FLOOR + SET_AVATAR_MODE → arbiter proxy.

        Phase 2 (issue #2002): arbiter — реальный владелец LockManager/FSM
        после ADR-0051 §2.2 (issue #1987). Клиенты ходят через
        ``/supervisor/execute`` и не должны знать о существовании arbiter.
        """
        client_id = getattr(command, "client_id", "") or ""
        if not client_id:
            return _make_execute_response(
                accepted=False,
                applied=False,
                reason=EXEC_REASON_BAD_REQUEST,
            )
        # Монитор-режим: supervisor принимает, но не делает (S12). Это
        # поведение сохранено из Phase 1; в Phase 2 активный путь идёт
        # через arbiter-клиент.
        if self._mode != "active":
            return _make_execute_response(
                accepted=True,
                applied=False,
                reason=EXEC_REASON_MONITOR_MODE,
            )
        if not self._ensure_arbiter_clients():
            # Arbiter недоступен (CI mock без него, или workspace без
            # пересборки IDL). Честный FAIL (ADR-0018): не притворяемся
            # что проксировали, а явно отдаём reason=arbiter_unavailable
            # (НЕ facade_only — это уже сделано, Phase 2 закрывает gap).
            return _make_execute_response(
                accepted=True,
                applied=False,
                reason=EXEC_REASON_ARBITER_UNAVAILABLE,
                held_by=str(client_id),
            )
        # Делегируем arbiter-у. ACQUIRE → granted/applied, RELEASE →
        # applied, SET_AVATAR_MODE → applied/actual_mode. Маппинг полей
        # см. ниже в _proxy_to_arbiter_sync.
        return self._proxy_to_arbiter_sync(kind, command)

    def _dispatch_voice_mode(self, command: Any, kind: int) -> Any:
        """SET_VOICE_MODE → локально через :py:meth:`_apply_voice_mode`."""
        voice_mode = _normalize_voice_mode(
            getattr(command, "voice_mode", "")
        )
        applied, sub_reason = self._apply_voice_mode(voice_mode)
        if applied:
            return _make_execute_response(
                accepted=True,
                applied=True,
                reason="applied",
                actual_mode=voice_mode,
            )
        # _apply_voice_mode уже отдаёт внятные reason:
        # voice_mode_deprecated / monitor_mode / publish_failed.
        # Маппим на уровень фасада:
        if sub_reason == MONITOR_MODE_REASON:
            facade_reason = EXEC_REASON_MONITOR_MODE
        elif sub_reason.startswith("voice_mode_deprecated"):
            facade_reason = EXEC_REASON_VOICE_MODE_REJECTED
        elif sub_reason == "publish_failed":
            # Не смогли опубликовать в /dialogue/control (например,
            # rclpy error / нет подписчика) — facade трактует как
            # voice_mode_rejected с детальным reason в логах
            # (ADR-0018: честный FAIL, не молчание).
            facade_reason = EXEC_REASON_VOICE_MODE_REJECTED
        else:
            facade_reason = EXEC_REASON_BAD_REQUEST
        return _make_execute_response(
            accepted=False,
            applied=False,
            reason=facade_reason,
            actual_mode=voice_mode,
        )

    def _dispatch_emergency_stop(self, command: Any, kind: int) -> Any:
        """EMERGENCY_STOP → not_implemented (Phase 1, ADR-0018).

        В Phase 2 добавим публикацию на ``/avatar/emergency_stop`` +
        лок-стейт LockManager. Пока честный FAIL: приняли — но не сделали;
        emergency=true → emergency=false различимы по ``accepted/applied``.
        """
        emergency = bool(getattr(command, "emergency", False))
        # emergency=false («снять стоп») — accepted=true, applied=false
        # (нет стопа, который нужно снимать) → reason=emergency_off.
        # Это НЕ ошибка: клиент мог честно думать, что стоп активен.
        # emergency=true — accepted=true, applied=false,
        # reason=not_implemented (Phase 2).
        if not emergency:
            return _make_execute_response(
                accepted=True,
                applied=False,
                reason=EXEC_REASON_EMERGENCY_OFF,
            )
        return _make_execute_response(
            accepted=True,
            applied=False,
            reason=EXEC_REASON_NOT_IMPLEMENTED,
        )

    def _dispatch_heartbeat(self, command: Any, kind: int) -> Any:
        """HEARTBEAT → noop в Phase 1.

        arbiter уже публикует FloorState через Legacy, supervisor heartbeat
        в Phase 1 просто подтверждает приём. Phase 2 заменит на полноценный
        floor-refresh.
        """
        client_id = getattr(command, "client_id", "") or ""
        if not client_id:
            return _make_execute_response(
                accepted=False,
                applied=False,
                reason=EXEC_REASON_BAD_REQUEST,
            )
        return _make_execute_response(
            accepted=True,
            applied=True,
            reason=EXEC_REASON_HEARTBEAT_NOOP,
            held_by=str(client_id),
        )

    # ── AV-28 §P7 (issue #1920) — voice style preset + language ─────────
    # ADR-0080 §2.7 / voice-vr 21: супервизор больше НЕ пишет в чужие
    # ROS-параметры. ``voice_preset`` / ``voice_output_language`` УДАЛЕНЫ
    # из ``dialogue_node`` целиком (declare_parameter + обработка в
    # parameters_callback — см. dialogue_node.py, эта же карточка): там
    # больше нет параметра, писать в который. Топики
    # ``/avatar/set_voice_preset`` / ``/avatar/set_voice_language``
    # остаются легаси-приёмниками для обратной совместимости с
    # UI/quest_node: whitelist-валидация по единому списку
    # (``rob_box_core.bridge_protocol``) + ack/nack оператору через
    # ``voice_set_ack`` (UI откатывает optimistic update, если nack), но
    # применённое значение НИКУДА не пишется и не влияет на звучание —
    # это самостоятельно не подключённый путь (ADR-0018), оставленный как
    # заглушка до расширения ``/dialogue/control`` под set_preset/
    # set_language (тема отдельной карточки).
    #
    # Живой путь стиля/языка речи — ``/avatar/voice_pipeline`` →
    # ``_on_grip_voice_pipeline`` → ``self._pipeline_preset`` /
    # ``self._pipeline_language`` (грип-пайплайн, issue #1989); он читает
    # ``grip_pipeline.load_voice_presets()`` из yaml сам, независимо от
    # этого блока. НЕ путать два канала.

    # Валидируем по модульным VOICE_PRESET_IDS / VOICE_LANGUAGES — второй
    # копии списка здесь больше нет (см. комментарий у импорта).
    _AV28_PRESET_IDS: frozenset[str] = frozenset(VOICE_PRESET_IDS)
    _AV28_LANGUAGES: frozenset[str] = frozenset(VOICE_LANGUAGES)

    def _on_set_voice_preset(self, msg: RosString) -> None:
        """Обработка ``/avatar/set_voice_preset`` — легаси-приём стиля речи.

        voice-vr 21: ``applied=true`` означает только «прошёл whitelist +
        режим active», НЕ «где-то что-то поменялось». Ничего не
        публикуется и не сохраняется — см. комментарий блока AV-28 §P7
        выше про два разных канала.
        """
        preset = (msg.data or "").strip()
        applied, reason = self._apply_voice_preset(preset)
        self._log.info(
            f"SetVoicePreset: preset={preset} applied={applied} reason={reason}"
        )

    def _apply_voice_preset(self, preset: str) -> tuple[bool, str]:
        """Чистая логика применения ``voice_preset`` (тестируется без rclpy).

        voice-vr 21: супервизор не пишет в чужие ROS-параметры (ADR-0080 §2.7),
        а dialogue_node больше не имеет параметра ``voice_preset`` вообще
        (удалён). Этот метод только валидирует и логирует — он НЕ вызывает
        ``grip_pipeline`` и НЕ трогает ``self._pipeline_preset``; тот
        живёт своей жизнью через ``/avatar/voice_pipeline`` (см. блочный
        комментарий выше). Легаси-заглушка до explicit-контракта.
        """
        if not preset:
            return False, "empty_voice_preset"
        if preset not in self._AV28_PRESET_IDS:
            return False, f"invalid_voice_preset: {preset!r}"
        if self._mode != "active":
            return False, MONITOR_MODE_REASON
        # Грядущая карточка расширит ``/dialogue/control`` под
        # set_preset/set_language — до тех пор здесь только whitelist +
        # лог, без побочных эффектов (см. docstring метода).
        self._log.info(
            f"[voice-vr 21] voice_preset accepted={preset!r} "
            "(legacy no-op: no foreign param writes, no pipeline state change)"
        )
        return True, "applied"

    def _on_set_voice_language(self, msg: RosString) -> None:
        """Обработка ``/avatar/set_voice_language`` — легаси-приём языка вывода."""
        language = (msg.data or "").strip()
        applied, reason = self._apply_voice_language(language)
        self._log.info(
            f"SetVoiceLanguage: language={language} applied={applied} reason={reason}"
        )

    def _apply_voice_language(self, language: str) -> tuple[bool, str]:
        """Чистая логика применения ``voice_output_language`` (тестируется без rclpy).

        Аналогично :py:meth:`_apply_voice_preset` — супервизор больше НЕ
        пишет в чужие ROS-параметры (ADR-0080 §2.7), и это НЕ то же самое,
        что смена языка грип-пайплайна (``self._pipeline_language``, через
        ``/avatar/voice_pipeline``). Только валидация + лог.
        """
        if not language:
            return False, "empty_voice_language"
        if language not in self._AV28_LANGUAGES:
            return False, f"invalid_voice_language: {language!r}"
        if self._mode != "active":
            return False, MONITOR_MODE_REASON
        self._log.info(
            f"[voice-vr 21] voice_output_language accepted={language!r} "
            "(legacy no-op: no foreign param writes, no pipeline state change)"
        )
        return True, "applied"

    # ── AV-27 TTS picker (issue #1919) ──────────────────────────────
    def _on_set_voice(self, msg: RosString) -> None:
        """Обработка ``/avatar/set_voice`` — сменить голос TTS.

        ADR-0080 §2.7 / voice-vr 21: супервизор НЕ пишет в чужие
        ROS-параметры (раньше — параметр-клиент на tts_node — это
        знание внутренней схемы имён ``yandex_voice``/``minimax_voice``/
        ``silero_speaker``, явный шов ADR-0080 §2.7). Теперь публикует
        JSON в ``/voice/tts/set_voice`` (``SET_TTS_VOICE_TOPIC``), и
        tts_node применяет через ``_on_set_voice``. В monitor-режиме
        НЕ публикуем (S12) — только логируем и выходим.

        Quest-сервер уже выполнил свою валидацию по текущему активному
        провайдеру (по /voice/tts/provider_state); мы дублируем её по SoT
        (tts_voice_registry.voices_for) — это страховка от race, когда
        провайдер переключился между cmd'ом и обработкой на supervisor.
        """
        raw = (msg.data or "").strip()
        if not raw:
            self._log.warning("SetVoice: empty payload")
            return
        try:
            data = json.loads(raw)
        except (ValueError, TypeError) as exc:
            self._log.warning(f"SetVoice: bad json: {exc}")
            return
        voice_id = data.get("voice_id") if isinstance(data, dict) else None
        if not isinstance(voice_id, str) or not voice_id:
            self._log.warning(f"SetVoice: missing voice_id (raw={raw!r})")
            return
        provider_hint = data.get("provider") if isinstance(data, dict) else None
        applied, reason = self._apply_set_voice(voice_id, provider_hint=provider_hint)
        # f-string: RcutilsLogger принимает ОДИН один (issue #1644).
        self._log.info(
            f"SetVoice: voice_id={voice_id} provider={provider_hint} applied={applied} reason={reason}"
        )

    def _apply_set_voice(
        self, voice_id: str, provider_hint: str | None = None
    ) -> tuple[bool, str]:
        """Чистая логика применения set_voice (тестируется без rclpy).

        Возвращает ``(applied, reason)``. В monitor — ``applied=False`` без
        записи (S12). В active — публикация JSON в
        ``/voice/tts/set_voice`` (явный контракт ADR-0080 §2.7),
        tts_node применяет через ``_on_set_voice`` (знание схемы имён
        параметров — внутри tts_node, НЕ в supervisor).

        Обратная совместимость: контракт на запись в чужие ROS-параметры
        tts_node УДАЛЁН. Знание имён (``yandex_voice``/``minimax_voice``/
        ``silero_speaker``) — это шов ADR-0080 §2.7, теперь закрыт.
        На тестовом стенде оставлены прямые ``ros2 param set`` /
        MCP-SetVoice — это путь ``parameters_callback`` внутри tts_node.
        """
        if self._mode != "active":
            return False, MONITOR_MODE_REASON
        # Резолвим целевой provider. Предпочитаем hint из payload (quest_node
        # знает активного из /voice/tts/provider_state), fallback — первое
        # вхождение voice_id в любом провайдере (минимальный fallback для
        # тестовых сред; в проде hint всегда есть).
        provider = provider_hint
        if not provider:
            for p in ("yandex", "minimax", "silero"):
                if voice_id in _voices_for(p):
                    provider = p
                    break
        if not provider:
            return False, f"voice_not_in_any_provider: {voice_id!r}"
        if voice_id not in _voices_for(provider):
            return False, f"voice_unavailable:{provider}:{voice_id}"
        # Публикуем JSON в /voice/tts/set_voice — единственный живой
        # write-side для голоса (ADR-0080 §2.7 / voice-vr 21). tts_node
        # принимает, валидирует, обновляет атрибут и логирует. Никаких
        # write-side в чужие ROS-параметры в supervisor_node больше нет
        # (DoD-критерий: «supervisor не пишет в чужие ROS-параметры»).
        payload = json.dumps(
            {"voice_id": voice_id, "provider": provider, "source": "set_voice"}
        )
        self._set_voice_tts_pub.publish(RosString(data=payload))
        self._log.info(
            f"SetVoice: published voice_id={voice_id} provider={provider} "
            f"to {SET_TTS_VOICE_TOPIC} (applied=True, reason=applied:{provider})"
        )
        return True, f"applied:{provider}"

    def _on_preview_voice(self, msg: RosString) -> None:
        # ADR-0077 / issue #2138.A.3 — picker'у голосов нужен «прослушиваемый
        # образец». Канал: ``/avatar/preview_voice`` (JSON, ws_server → здесь)
        # → ``/avatar/tts/request`` с ``sink="preview"`` → ``tts_node`` делает
        # pure-synth (``synthesize_preview``) → ``/avatar/preview_voice/audio``
        # (JSON+base64) → ws_server → клиент (``preview_audio_sink.ts``).
        # Валидация/делегация вынесены в helpers чтобы не раздувать CC
        # (ADR-0021).
        data = self._preview_parse_payload(msg)
        if data is None:
            return  # ошибка уже залогирована
        request_id, voice_id, text, provider = self._preview_extract_fields(data)
        if request_id is None:
            self._log.warning("PreviewVoice: missing request_id")
            return
        if voice_id is None:
            self._publish_preview_error(request_id, "voice_id_required")
            return
        if text is None:
            self._publish_preview_error(request_id, "text_required")
            return
        # Валидация по реестру — делаем ДО публикации, чтобы picker сразу
        # получил honest error (а не silent-hang). Дубликат логики в
        # tts_node — это сознательно: supervisor шлёт честный preview_error,
        # даже если tts_node прислал бы тот же reason с задержкой на сеть.
        resolved = self._preview_resolve_provider(request_id, voice_id, provider)
        if resolved is None:
            return  # preview_error уже опубликован
        provider = resolved
        # Делегируем в tts_node через существующий канал /avatar/tts/request
        # с sink="preview". request_id протаскиваем до tts_node — он его
        # проставит в preview_voice_audio/result/error, чтобы ws_server
        # коррелировал с picker'ом.
        sent_rid = self._publish_avatar_tts(
            text=text,
            voice=voice_id,
            sink="preview",
            request_id=request_id,
        )
        if not sent_rid:
            # На случай если _publish_avatar_tts вернул пустую строку
            # (text drop). request_id сохранён, ошибка уже отлогирована
            # внутри. Дополнительно шлём preview_error с той же причиной
            # для ws_server, чтобы picker не завис на «слушаю…».
            self._publish_preview_error(request_id, "empty_text_dropped")

    @staticmethod
    def _preview_parse_payload(msg: RosString):
        """Парсит msg.data → dict или None (с WARN-логом)."""
        raw = (msg.data or "").strip()
        if not raw:
            AvatarSupervisor._preview_log_static("empty payload")
            return None
        try:
            data = json.loads(raw)
        except (ValueError, TypeError) as exc:
            AvatarSupervisor._preview_log_static(f"bad json: {exc}")
            return None
        if not isinstance(data, dict):
            AvatarSupervisor._preview_log_static("payload not dict")
            return None
        return data

    @staticmethod
    def _preview_extract_fields(data: dict):
        """Возвращает (request_id, voice_id, text, provider) — None если
        неправильный тип или пусто (но без логирования — caller сам
        решает что делать)."""
        request_id = data.get("request_id")
        voice_id = data.get("voice_id")
        text = data.get("text")
        provider = data.get("provider")

        def _str_or_none(v):
            return v if isinstance(v, str) and v else None

        return (
            _str_or_none(request_id),
            _str_or_none(voice_id),
            _str_or_none(text),
            _str_or_none(provider),
        )

    def _preview_resolve_provider(
        self, request_id: str, voice_id: str, provider: Optional[str]
    ) -> Optional[str]:
        """Валидирует voice_id по реестру. Возвращает финальный provider
        (str) или None (если preview_error уже опубликован и caller должен
        return)."""
        if provider is not None:
            if voice_id not in _voices_for(provider):
                self._publish_preview_error(
                    request_id, f"voice_unavailable:{provider}:{voice_id}"
                )
                return None
            return provider
        # Без hint — ищем где знают.
        known_in = [
            p for p in ("yandex", "minimax", "silero") if voice_id in _voices_for(p)
        ]
        if not known_in:
            self._publish_preview_error(request_id, "voice_unknown")
            return None
        # Берём первого провайдера, который знает голос (для tts_node это
        # hint — какой голос у какого провайдера искать). Если голос
        # доступен у нескольких, берём minimax (приоритет для preview).
        return "minimax" if "minimax" in known_in else known_in[0]

    @staticmethod
    def _preview_log_static(msg: str) -> None:
        # Stand-alone логгер — _preview_parse_payload static, без self.
        import logging as _logging

        _logging.getLogger("rob_box_supervisor.preview").warning(msg)

    def _publish_preview_error(self, request_id: str, reason: str) -> None:
        """Опубликовать preview_voice_error (JSON) для ws_server.

        ws_server слушает /avatar/preview_voice/error и шлёт клиенту
        ``JSON_EVENT{type:"preview_voice_error", request_id, reason, ts_ms}``.
        """
        payload = {
            "request_id": request_id,
            "reason": reason,
            "ts_ms": int(time.time() * 1000),
        }
        try:
            msg = RosString()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._preview_error_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"PreviewVoice: error publish failed: {exc}")

    # ── AV-21 (issue #1913): супервизор-агент «мозг оператора» ─────
    # Скелет: топики /avatar/command → /avatar/command_result, гейт
    # ``agent_enabled``, voice-mode swap с try/finally, метрики через
    # rob_box_voice.observability.metrics. Дизайн / acceptance —
    # docs/plans/2026-09-02-avatar-supervisor-agent-design.md §5.

    # ── helpers: metrics ─────────────────────────────────────────────────────────────────

    def _build_agent_metrics(self) -> dict[str, Any]:
        """Зарегистрировать / достать метрики супервизор-агента.

        Используем ``get_metric`` из rob_box_voice.observability.metrics —
        он no-op'ит если prometheus_client нет (CI / unit-тесты), и
        идемпотентно регистрирует счётчик/гистограмму (повторный вызов
        с тем же именем = тот же объект). Метрики, как и в voice-слое,
        живут в process-global prometheus REGISTRY (cross-node в проде
        не пересекаются, т.к. каждая ROS-нода = отдельный процесс).
        """
        try:
            from rob_box_voice.observability.metrics import get_metric  # noqa: PLC0415
        except ImportError:
            # Если rob_box_voice недоступен (минимальный CI-env без
            # voice-пакета) — все методы record_* станут no-op через
            # ``MetricsDisabled``. Это идома проекта, см. metrics.py.
            get_metric = None  # type: ignore[assignment]
        if get_metric is None:
            # Подменяем на заглушки, чтобы ``self._agent_metrics`` всегда
            # был dict-ом и все .inc/.observe/.labels были no-op.
            return {
                "commands": _NoopLabelCounter(),
                "tool_calls": _NoopLabelCounter(),
                "latency": _NoopHistogram(),
                "enabled": False,
            }
        return {
            "commands": get_metric(
                "counter",
                "avatar_agent_commands_total",
                "Supervisor-agent command outcomes, labelled by source and result.",
                labelnames=("source", "result"),
            ),
            "tool_calls": get_metric(
                "counter",
                "avatar_agent_tool_calls_total",
                "Supervisor-agent tool invocations, labelled by tool name.",
                labelnames=("tool",),
            ),
            "latency": get_metric(
                "histogram",
                "avatar_agent_latency_seconds",
                "Supervisor-agent end-to-end command latency (seconds).",
            ),
            "enabled": True,
        }

    def _record_agent_command(
        self, source: str, result: str, latency_s: Optional[float] = None
    ) -> None:
        """Инкрементить ``avatar_agent_commands_total`` и засечь гистограмму."""
        metrics = self._agent_metrics
        if not metrics.get("enabled", False):
            return
        try:
            metrics["commands"].labels(source=source or "unknown", result=result).inc()
            if latency_s is not None:
                metrics["latency"].observe(latency_s)
        except Exception as exc:  # noqa: BLE001 — метрики best-effort
            self._log.warning(f"agent_metrics: command record failed: {exc}")

    def _record_agent_tool_call(self, tool_name: str) -> None:
        """Инкрементить ``avatar_agent_tool_calls_total``."""
        metrics = self._agent_metrics
        if not metrics.get("enabled", False):
            return
        try:
            metrics["tool_calls"].labels(tool=tool_name).inc()
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"agent_metrics: tool_call record failed: {exc}")

    # ── helpers: dialogue control swap (ADR-0066 §6.7) ──────────────────
    # После удаления ``voice_input_mode`` swap вокруг ``_run_agent_sync``
    # работает по новой схеме: на входе публикуем ``pause`` в
    # ``/dialogue/control``, в ``finally`` (включая путь с исключением)
    # публикуем ``resume``. Snapshot предыдущего состояния личности НЕ
    # нужен: dialogue_node уже идемпотентно хранит своё состояние и
    # ack-ает на ``/dialogue/control_ack``. Если супервизор стартует, а
    # личность уже в SILENCED — повторный pause no-op'ит в dialogue_node
    # (см. ``_apply_operator_pause``, dialogue_node.py:2200).
    #
    # try/finally сохранён для AC #8: даже если LLM бросит исключение
    # внутри swap, личность гарантированно вернётся в IDLE. Без finally
    # оператор «вырубит» личность одной командой, а resume так и не
    # прилетит — это и есть регрессия ADR-0028 S5.

    @contextlib.contextmanager
    def _dialogue_control_swap(self) -> Iterator[None]:
        """Контекст-менеджер «пока оператор работает, личность молчит».

        ADR-0066 §6.7: на входе публикуем ``pause`` в ``/dialogue/control``,
        в ``finally`` (включая путь с исключением) — ``resume``. Поле
        ``prev_mode``/``_capture_current_voice_mode`` больше не нужны:
        dialogue_node сам знает свой FSM-стейт и ack-ает, супервизор лишь
        дёргает переходы pause↔resume.

        В monitor-режиме ``_publish_dialogue_control`` всё равно публикует
        (мы не знаем, опубликовал ли кто-то до нас) — dialogue_node всё
        равно идемпотентен, но в логе будет видна попытка apply даже в
        monitor. Тест-инвариант: в monitor поведение **не проверяется** на
        идемпотентность dialogue_node (мы лишь тестируем, что swap шлёт
        две команды).
        """
        # Входим в режим «оператор работает».
        applied_in = self._publish_dialogue_control(
            DIALOGUE_CONTROL_PAUSE,
            reason="agent_during_command",
        )
        if not applied_in:
            # Если publish упал (нет подписчика / rclpy error) — не валим
            # обработку команды, но логируем. Семантика: «попытались
            # заглушить, не вышло — команду всё равно обработаем».
            self._log.debug(
                "dialogue_control_swap.enter: pause publish failed "
                "(no subscriber or rclpy issue)"
            )
        try:
            yield
        finally:
            # Восстанавливаем личность — ВСЕГДА, включая путь с исключением
            # (AC #8). resume из не-paused состояния в dialogue_node —
            # no-op + ack с текущим состоянием (§2.5 ADR-0066), так что
            # безопасно даже если pause не дошёл.
            applied_out = self._publish_dialogue_control(
                DIALOGUE_CONTROL_RESUME,
                reason="agent_command_done",
            )
            if not applied_out:
                self._log.warning(
                    "dialogue_control_swap.exit: resume publish failed — "
                    "dialogue_node may remain SILENCED. Operator can "
                    "manually send /dialogue/control {action:resume}."
                )

    # ── helpers: AgentCore (issue #1988, шаг 4а) ───────────────────────
    # OperatorHarness заменён на AgentCore: промпт оператора, реальный
    # ROSMCPToolProvider, память namespace operator (отдельная БД) и
    # журнал ТАРС (§5.4, operator_journal.py). Сборка ленивая — только
    # при agent_enabled=true и первом входе.

    def _param_str(self, name: str, default: str = "") -> str:
        """Прочитать string-параметр с защитой от необъявленного (rclpy)."""
        try:
            return str(self.get_parameter(name).value or default)
        except Exception:  # noqa: BLE001 — необъявленный/битый параметр
            return default

    def _param_int(self, name: str, default: int = 0) -> int:
        try:
            return int(self.get_parameter(name).value or default)
        except Exception:  # noqa: BLE001
            return default

    def _param_bool(self, name: str, default: bool = False) -> bool:
        try:
            return bool(self.get_parameter(name).value or default)
        except Exception:  # noqa: BLE001
            return default

    def _ensure_agent_core(self) -> Any:
        """Ленивая сборка операторского AgentCore (один раз на ноду).

        Тест-инвариант: ``agent_enabled=false`` → core НЕ создаётся
        (в _on_avatar_command _ensure_agent_core не вызывается). Кеш в
        ``self._agent_core`` — чтобы на каждый /avatar/command не
        пересобирать AgentCore (там LLM/Tools/Memory init дорого).
        """
        if self._agent_core is None:
            core, dsm = self._build_agent_core_sync()
            if core is not None:
                self._agent_core = core
                self._operator_dsm = dsm
        return self._agent_core

    def _build_agent_core_sync(self) -> tuple[Any, Any]:
        """Собрать ``(AgentCore, DialogueStateMachine)`` оператора.

        ADR-0083 §2.3 — supervisor использует :func:`build_agent` из
        :mod:`rob_box_harness.core.assembly`. Все расхождения между
        личностью и ТАРС раскладываются по полям :class:`AgentSpec`:

        * ``name='operator'``, ``memory_namespace='operator'`` — обе
          ноды пишут в ``/data/harness_voice.db``, фильтрация по
          колонке ``agent`` (миграция 011_agent_namespace.sql).
        * ``narrow_tools_to_skill=False`` — ТАРС видит все инструменты
          (ADR-0083 §2.2 #J).
        * ``use_scheduler=False`` — W7b планировщик живёт только на
          стороне личности (ADR-0083 §2.2 #F).
        * ``health_cache_persist_path`` — общий ``~/.rob_box/llm_health.json``
          (ADR-0083 §1.3 #1). Если dialogue_node уже туда пишет —
          supervisor и личность делят один HealthCache.
        * ``per_provider_settings={}`` — supervisor держит пусто,
          ``temperature``/``max_tokens`` приходят из YAML
          dialogue-параметров (ADR-0083 §1.2 #B).
        * ``health_balance_checkers={}`` — supervisor держит пусто
          (ADR-0083 §1.2 #A): MiniMax без balance API, deepseek
          авто-детектится в ``build_agent``.
        * ``on_prompt=self._record_supervisor_prompt_stats`` —
          закрывает §G (supervisor не публиковал PromptStats).

        ``tools`` и ``memory`` нода собирает сама — ``tools``
        требует ``self`` для ``ROSMCPToolProvider``, ``memory``
        требует asyncio-loop, на котором зовётся
        ``SQLiteVoiceMemory.init()``. Возвращает ``(None, None)``
        на любой сбой (ADR-0018: честный FAIL, не падение ноды).
        """
        try:
            from rob_box_harness.core.dialogue_state_machine import (  # noqa: PLC0415
                DialogueStateMachine,
            )
        except ImportError as exc:
            self._log.warning(f"_build_agent_core_sync: import failed: {exc}")
            return (None, None)

        tools = self._build_operator_tools()
        if tools is None:
            return (None, None)
        memory = self._build_operator_memory()

        # ── Operator AgentSpec (ADR-0083 §2.3 / §1.2 #A-#B / §2.2 #F/#J) ──
        provider_chain = self._parse_provider_chain(
            self._param_str("llm_providers", "minimax,deepseek")
        )
        settings = self._build_operator_llm_settings()
        try:
            from pathlib import Path  # noqa: PLC0415

            spec = AgentSpec(
                name="operator",
                prompt_dir=self._resolve_prompts_dir() or Path(""),
                system_prompt_file=self._system_prompt_file,
                skill_slice=("operator.speech", "operator.control"),
                use_scheduler=False,
                memory_namespace="operator",
                on_prompt=self._record_supervisor_prompt_stats,
                provider_chain=provider_chain,
                settings=settings,
                per_provider_settings={},
                health_cache_persist_path=Path(
                    "~/.rob_box/llm_health.json"
                ).expanduser(),
                health_balance_checkers={},
                history_trim_limit=self._param_int("history_max_turns", 10),
                narrow_tools_to_skill=False,  # ADR-0083 §2.2 #J
                dsm=DialogueStateMachine(),
                user_id="operator",
            )
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"_build_agent_core_sync: AgentSpec build failed: {exc}")
            return (None, None)

        try:
            core = build_agent(spec, tools=tools, memory=memory)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"_build_agent_core_sync: build_agent failed: {exc}")
            return (None, None)

        # Журнал ТАРС создаём вместе с core (персист — best-effort).
        self._operator_journal = self._build_operator_journal()
        return (core, spec.dsm)

    @staticmethod
    def _parse_provider_chain(raw: str) -> tuple[str, ...]:
        """Распарсить CSV-параметр ``llm_providers`` → tuple.

        Зеркалит ``dialogue_node._parse_provider_chain`` (issue #2111,
        ADR-0043 §3.2). Default повторяет ``dialogue_node`` —
        ``("minimax", "deepseek")``, иначе supervisor поднимается
        с одним провайдером без API-ключа, agent не отвечает.
        """
        names = tuple(
            n.strip().lower() for n in raw.split(",") if n.strip()
        )
        return names or ("deepseek",)

    def _build_operator_llm_settings(self) -> Any:
        """``LLMSettings`` для AgentSpec оператора (temperature/max_tokens).

        ADR-0083 §1.2 #B — supervisor НЕ передаёт per-provider settings
        (передаёт ``{}``), но глобальные ``temperature``/``max_tokens``
        из параметров supervisor берёт. Используется как ``AgentSpec.settings``
        (одно значение для primary провайдера).
        """
        try:
            from rob_box_llm.provider import LLMSettings  # noqa: PLC0415
        except ImportError:
            return None
        try:
            temperature = float(self.get_parameter("temperature").value or 0.0)
        except Exception:  # noqa: BLE001
            temperature = 0.0
        try:
            max_tokens = int(self.get_parameter("max_tokens").value or 0)
        except Exception:  # noqa: BLE001
            max_tokens = 0
        return LLMSettings(
            temperature=(temperature if temperature > 0 else None),
            max_tokens=(max_tokens if max_tokens > 0 else None),
        )

    def _record_supervisor_prompt_stats(self, stats: Any) -> None:
        """Опубликовать размер промпта supervisor'а (ADR-0083 §G).

        Зеркало :meth:`dialogue_node._on_prompt_stats` — та же
        гистограмма ``voice_llm_prompt_tokens``, различается через
        метку ``skill`` (``"none"`` пока скиллы оператора не
        активированы). Зовётся из :class:`AgentCore` на КАЖДОЕ
        обращение к LLM, включая каждую итерацию тул-цикла.

        Любое исключение гасится: телеметрия не имеет права ронять
        живой ход. AgentCore тоже глушит исключения наблюдателя —
        это второй слой.
        """
        try:
            record_llm_prompt_tokens(
                stats.provider,
                tokens=stats.prompt_tokens,
                skill=stats.skill,
                estimated=stats.estimated,
            )
        except Exception as exc:  # noqa: BLE001
            self._log.debug(f"[operator prompt] record failed: {exc}")

    def _build_operator_tools(self) -> Any:
        """Реальный ToolProvider оператора (ROSMCPToolProvider поверх /mcp).

        ``tool_provider=ros_mcp`` (default) — LLMToolCallAdapter → /mcp/execute
        + манифесты ToolRegistry → адаптер legacy-контракта AgentCore.
        ``fake``/``none`` — тесты/smoke (0 инструментов, chat-only).
        """
        backend = self._param_str("tool_provider", "ros_mcp").strip().lower()
        if backend in ("fake", "none"):
            try:
                from rob_box_harness.tools import FakeToolProvider  # noqa: PLC0415
            except ImportError:
                return None
            self._log.info(
                f"operator tool_provider={backend}: FakeToolProvider (chat-only)"
            )
            return FakeToolProvider()
        if backend != "ros_mcp":
            self._log.warning(
                f"unknown operator tool_provider {backend!r}; falling back to fake"
            )
            try:
                from rob_box_harness.tools import FakeToolProvider  # noqa: PLC0415

                return FakeToolProvider()
            except ImportError:
                return None
        try:
            from rob_box_harness.core.tool_registry import ToolRegistry  # noqa: PLC0415
            from rob_box_harness.executors import (  # noqa: PLC0415
                ROSMCPToolProvider,
                adapt_tool_provider,
            )
            from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter  # noqa: PLC0415
        except ImportError as exc:
            self._log.warning(f"_build_operator_tools: MCP import failed: {exc}")
            return None
        try:
            bridge = LLMToolCallAdapter(self)
            provider = ROSMCPToolProvider(bridge)
            registry = ToolRegistry()
            # Issue #2113 (quest #2112) — TARS 2 tool: ``show_metrics``.
            # Регистрируется ДО ``provider.update_tools``, чтобы у LLM
            # уже была актуальная спецификация. Dispatcher уже создан в
            # ``__init__``; если он упал там (например, тест без ROS) —
            # пропускаем регистрацию без exception.
            if getattr(self, "_tars_panel_dispatcher", None) is not None:
                try:
                    self._tars_panel_dispatcher.register_tool(registry)
                except Exception as exc:  # noqa: BLE001
                    self._log.warning(
                        f"[issue #2113] show_metrics tool registration "
                        f"failed: {exc}"
                    )
            provider.update_tools(
                [
                    {
                        "type": "function",
                        "function": {
                            "name": spec.name,
                            "description": spec.description,
                            "parameters": dict(spec.parameters),
                        },
                    }
                    for spec in registry.list_tools()
                ]
            )
            catalogue = provider.list_tools()
            if not catalogue:
                self._log.warning("_build_operator_tools: empty MCP catalogue")
                return None
            self._log.info(
                f"operator tools: {len(catalogue)} MCP tools via ROSMCPToolProvider"
            )
            return adapt_tool_provider(provider)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"_build_operator_tools: build failed: {exc}")
            return None

    def _build_operator_memory(self) -> Any:
        """Память оператора — общая БД с личностью, фильтр по ``agent`` (ADR-0083 §E).

        До ADR-0083 §E supervisor писал в отдельный файл
        ``/data/operator_memory.db`` и терял общий контекст с личностью.
        Теперь обе ноды пишут в ``/data/harness_voice.db`` через
        ``SQLiteVoiceMemory(agent=...)``: ``agent='operator'`` для ТАРС,
        ``agent='personality'`` для личности. Колонка ``agent`` в
        ``facts`` создана миграцией 011_agent_namespace.sql.

        Параметр ``sqlite_db_path`` — единый с dialogue_node
        (тот же default ``/data/harness_voice.db``). ``operator_db_path``
        оставлен как DEPRECATED fallback для round-веток до полного
        ребилда. При сбое — InMemoryStore (нода живёт).
        """
        # Приоритет: ``sqlite_db_path`` (новый, ADR-0083 §E) →
        # ``operator_db_path`` (legacy, тот же файл сейчас). Оба ведут
        # на ``/data/harness_voice.db`` в текущем supervisor.yaml.
        db = (
            self._param_str("sqlite_db_path", "")
            or self._param_str("operator_db_path", "")
            or "/data/harness_voice.db"
        )
        try:
            from rob_box_harness.memory import SQLiteVoiceMemory  # noqa: PLC0415

            store = SQLiteVoiceMemory(db_path=db, agent="operator")
            asyncio.run(store.init())
            return store
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"operator memory init failed ({exc}); InMemoryStore")
            try:
                from rob_box_harness.memory import InMemoryStore  # noqa: PLC0415

                store = InMemoryStore()
                try:
                    asyncio.run(store.init())
                except Exception:  # noqa: BLE001
                    pass
                return store
            except ImportError:
                return None

    @staticmethod
    def _resolve_prompts_dir() -> Any:
        """Каталог ``rob_box_supervisor/prompts`` (источник истины).

        Порядок: ament share (установленный пакет) → source-tree
        (colcon symlink / unit-тесты). Возвращает ``Path`` или ``None``.
        Используется в :meth:`_build_agent_core_sync` для построения
        :class:`AgentSpec.prompt_dir` — ``build_agent`` берёт
        ``system_prompt_file`` и скиллы ``skill_slice`` отсюда.
        Раньше supervisor ещё читал ``rob_box_voice/prompts/skills``
        best-effort (протечка ADR-0083 §1.3 #3) — закрыто, теперь
        ``skill_slice`` строго ``("operator.speech", "operator.control")``.
        """
        from pathlib import Path  # noqa: PLC0415

        try:
            from ament_index_python.packages import (  # noqa: PLC0415
                get_package_share_directory,
            )

            share = Path(get_package_share_directory("rob_box_supervisor")) / "prompts"
            if share.is_dir():
                return share
        except Exception:  # noqa: BLE001 — нет ament (unit-тесты)
            pass
        # Source-tree: <repo>/src/rob_box_supervisor/prompts
        source = Path(__file__).resolve().parents[1] / "prompts"
        return source if source.is_dir() else None

    def _build_operator_journal(self) -> Any:
        """Журнал ТАРС (§5.4): лог изменений со схлопыванием повторов."""
        from rob_box_supervisor.operator_journal import OperatorJournal  # noqa: PLC0415

        return OperatorJournal(
            path=self._param_str("journal_path", "/data/operator_journal.jsonl")
        )

    def _render_journal_context(self) -> str:
        """Свежие записи журнала для ``dynamic_system`` (AgentCore)."""
        journal = getattr(self, "_operator_journal", None)
        if journal is None:
            return ""
        try:
            return journal.render(limit=8)
        except Exception:  # noqa: BLE001 — журнал не роняет ход
            return ""

    def _record_operator_journal(
        self, source: str, summary: str, tool_names: list[str]
    ) -> None:
        """Записать исход команды в журнал ТАРС (best-effort)."""
        journal = getattr(self, "_operator_journal", None)
        if journal is None:
            return
        try:
            if tool_names:
                action = "выполнил: " + ", ".join(tool_names)
            else:
                action = "ответил оператору"
            outcome = (summary or "")[:80]
            journal.record(action, outcome=outcome)
        except Exception as exc:  # noqa: BLE001
            self._log.debug(f"_record_operator_journal: {exc}")

    def _run_agent_sync(self, core: Any, payload: Mapping[str, Any]) -> dict[str, Any]:
        """Один turn оператора: ``AgentCore.process_input`` → mapping.

        DSM оператора гоним в DIALOGUE (зеркало dialogue_node._on_stt,
        фикс #1217) и зовём ``process_input`` с
        ``preclassified_event=STT_RESULT`` — прямой команде вейк-слово не
        нужно, повторная классификация навредит (wake-слово внутри текста).
        Журнал инжектим через ``dynamic_system``.

        Mapping DialogResult → ``{ok, summary, tool_calls, error, source,
        client_id}``: ``ok`` — агент ответил (текстом или инструментом);
        ``summary`` — оператор-видимый текст ответа; ``tool_calls`` — имена
        реально исполненных инструментов.
        """
        text = str(payload.get("text", "") or "").strip()
        source = str(payload.get("source", "") or "")
        client_id = str(payload.get("client_id", "") or "")
        if not text:
            return {
                "ok": False,
                "summary": "empty_input",
                "tool_calls": [],
                "error": "empty text",
                "source": source,
                "client_id": client_id,
            }
        try:
            from rob_box_harness.core.dialogue_state_machine import (  # noqa: PLC0415
                DialogueEvent,
                DialogueStateKind,
            )
        except ImportError as exc:
            return {
                "ok": False,
                "summary": f"agent_unavailable: {type(exc).__name__}",
                "tool_calls": [],
                "source": source,
                "client_id": client_id,
            }

        # DSM-пре-драйв в DIALOGUE.
        dsm = self._operator_dsm
        if dsm is not None:
            state = getattr(dsm, "current_state", None)
            if state != DialogueStateKind.DIALOGUE:
                if state == DialogueStateKind.IDLE:
                    try:
                        dsm.on_event(DialogueEvent.WAKE_WORD)
                    except Exception:  # noqa: BLE001
                        pass
                try:
                    dsm.on_event(DialogueEvent.STT_RESULT)
                except Exception:  # noqa: BLE001
                    pass

        journal_text = self._render_journal_context()
        try:
            result = asyncio.run(
                core.process_input(
                    text,
                    dynamic_system=journal_text or None,
                    preclassified_event=DialogueEvent.STT_RESULT,
                )
            )
        except Exception as exc:  # noqa: BLE001 — AgentCore не должен, но страхуемся
            return {
                "ok": False,
                "summary": f"llm_error: {type(exc).__name__}",
                "tool_calls": [],
                "error": str(exc),
                "source": source,
                "client_id": client_id,
            }

        if getattr(result, "error", None) is not None:
            return {
                "ok": False,
                "summary": f"llm_error: {type(result.error).__name__}",
                "tool_calls": [],
                "error": str(result.error),
                "source": source,
                "client_id": client_id,
            }

        spoken = (getattr(result, "spoken_text", "") or "").strip()
        tool_names = list(getattr(result, "tools_called", None) or [])
        if spoken:
            ok, summary = True, spoken
        elif tool_names:
            ok, summary = True, "ok"
        else:
            ok, summary = False, "no_tool"
        return {
            "ok": ok,
            "summary": summary,
            "tool_calls": [{"name": name} for name in tool_names],
            "source": source,
            "client_id": client_id,
        }

    # ── /avatar/command processing ─────────────────────────────────────────────────

    def _parse_command_payload(self, raw: str) -> dict[str, Any]:
        """Распарсить JSON из ``/avatar/command``.

        Возвращает ``{ok, payload, error}`` — pure-функция для
        тестируемости. ``ok=False`` если JSON битый ИЛИ
        отсутствуют обязательные поля ``source``/``client_id``/``text``.
        Допускаем ``ts_ms`` опциональным (генерируем client-side).
        """
        if not raw:
            return {"ok": False, "error": "empty payload"}
        data = self._try_parse_json(raw)
        if not isinstance(data, dict):
            return {"ok": False, "error": "malformed_input: not a JSON object"}
        text = data.get("text")
        if not isinstance(text, str) or not text.strip():
            return {"ok": False, "error": "malformed_input: missing 'text' field"}
        source = data.get("source")
        if not isinstance(source, str) or not source.strip():
            return {"ok": False, "error": "malformed_input: missing 'source' field"}
        client_id = data.get("client_id", "")
        if not isinstance(client_id, str):
            client_id = str(client_id) if client_id is not None else ""
        ts_ms = data.get("ts_ms")
        return {
            "ok": True,
            "payload": {
                "source": source,
                "client_id": client_id,
                "text": text.strip(),
                "ts_ms": ts_ms,
            },
        }

    def _generate_request_id(self, payload: Mapping[str, Any]) -> str:
        """Сгенерировать ``request_id`` для /avatar/command_result.

        Детерминированно из ``client_id + ts_ms`` если есть (повторная
        обработка той же команды = тот же request_id — удобно для
        дедупликации на клиенте). Иначе — UUID4.
        """
        cid = str(payload.get("client_id", "") or "")
        ts = payload.get("ts_ms")
        if cid and ts is not None:
            return f"{cid}:{ts}"
        return uuid.uuid4().hex

    def _publish_command_result(self, request_id: str, body: Mapping[str, Any]) -> None:
        """Опубликовать результат в /avatar/command_result (std_msgs/String JSON).

        Схема — docs/architecture/avatar-supervisor-agent.md §3.
        ``body`` содержит ``ok``/``summary``/``tool_calls``/опц.
        ``latency_ms``. Невалидный JSON невозможен (мы сами собираем),
        но защищаемся от битых типов: всё приводится к примитивам.
        """
        result = {
            "request_id": request_id,
            "ok": bool(body.get("ok", False)),
            "summary": str(body.get("summary", "")),
            "tool_calls": list(body.get("tool_calls", []) or []),
        }
        if "latency_ms" in body:
            result["latency_ms"] = int(body["latency_ms"])
        try:
            payload = json.dumps(result, ensure_ascii=False)
        except (TypeError, ValueError) as exc:
            self._log.warning(f"_publish_command_result: json.dumps failed: {exc}")
            payload = json.dumps(
                {"request_id": request_id, "ok": False, "summary": "publish_error"}
            )
        msg = RosString()
        msg.data = payload
        self._agent_result_pub.publish(msg)

    # Функция by-design ветвится на: malformed_input / agent_disabled /
    # agent_unavailable / ok / no_tool / llm_error / outer_error.
    # Каждая ветка — короткий блок с побочкой (publish + metric).
    # Извлечение в helper-ы увеличит indirection без выигрыша по
    # читаемости (флоу плоский, не цикл).
    def _on_avatar_command(self, msg: RosString) -> None:  # noqa: C901
        """ROS-callback ``/avatar/command`` (и ``/avatar/stt/result``, шаг 05).

        Главная точка входа супервизор-агента (ТАРС). Поток:
          1. Парсинг JSON. Битый → публикуем ``malformed_input``, выходим.
          2. Гейт ``agent_enabled``. False → публикуем ``agent_disabled``.
          3. Ленивая инициализация AgentCore (один раз).
          4. ``_dialogue_control_swap()`` (try/finally) — личность молчит
             пока мы работаем (ADR-0066 §6.7: публикуем ``pause`` в
             ``/dialogue/control`` на входе, ``resume`` в finally).
          5. ``AgentCore.process_input(payload)`` → результат.
          6. Публикация результата в ``/avatar/command_result``.
          7. Метрики + запись в журнал ТАРС.
        """
        started_ns = time.monotonic_ns()
        raw = msg.data or ""
        parsed = self._parse_command_payload(raw)
        if not parsed["ok"]:
            self._log.warning(f"_on_avatar_command: {parsed['error']}")
            self._publish_command_result(
                request_id=uuid.uuid4().hex,
                body={"ok": False, "summary": "malformed_input", "tool_calls": []},
            )
            self._record_agent_command(source="unknown", result="malformed_input")
            return

        payload = parsed["payload"]
        source = payload["source"]
        request_id = self._generate_request_id(payload)

        if not self._agent_enabled:
            self._publish_command_result(
                request_id=request_id,
                body={"ok": False, "summary": "agent_disabled", "tool_calls": []},
            )
            self._record_agent_command(source=source, result="agent_disabled")
            return

        if source not in AGENT_COMMAND_SOURCES:
            # Не блокируем (расширяемость — AV-22 добавит "web"/"admin"),
            # но логируем — чтобы оператор видел «незнакомый источник».
            self._log.warning(
                f"_on_avatar_command: unknown source={source!r} (expected one of "
                f"{AGENT_COMMAND_SOURCES}); processing anyway"
            )

        core = self._ensure_agent_core()
        if core is None:
            self._publish_command_result(
                request_id=request_id,
                body={"ok": False, "summary": "agent_unavailable", "tool_calls": []},
            )
            self._record_agent_command(source=source, result="agent_unavailable")
            return

        # Snapshot «текущего» voice_input_mode ДО swap.apply — нужен
        # для finally-восстановления. В Phase 1 это всегда "unknown"
        # (см. _capture_current_voice_mode), в active-режиме Phase 2
        # заменит на настоящий GetParameters.
        #
        # ADR-0066 §6.7 — snapshot больше не нужен: dialogue_node сам
        # знает свой FSM-стейт. Swap публикует ``pause`` на входе и
        # ``resume`` в finally без предварительного capture.

        try:
            with self._dialogue_control_swap():
                result = self._run_agent_sync(core, payload)
        except Exception as exc:  # noqa: BLE001 — НЕ ДОЛЖНО сбежать из swap
            # ``_dialogue_control_swap`` имеет try/finally, но защищаемся от
            # ошибок ВНЕ swap (publish, метрики). Сам swap уже
            # отправил resume.
            self._log.warning(f"_on_avatar_command: outer exception: {exc}")
            result = {
                "ok": False,
                "summary": f"outer_error: {type(exc).__name__}",
                "tool_calls": [],
            }

        # Нормализуем tool_calls (_run_agent_sync возвращает dict-ы вида
        # {"name": ...}; записываем метрики на каждый tool).
        tool_calls = result.get("tool_calls", []) or []
        tool_names: list[str] = []
        for tc in tool_calls:
            name = tc.get("name") if isinstance(tc, dict) else None
            if isinstance(name, str) and name:
                tool_names.append(name)
                self._record_agent_tool_call(name)

        latency_ms = int((time.monotonic_ns() - started_ns) / 1_000_000)
        self._publish_command_result(
            request_id=request_id,
            body={
                "ok": bool(result.get("ok", False)),
                "summary": str(result.get("summary", "")),
                "tool_calls": tool_calls,
                "latency_ms": latency_ms,
            },
        )
        # issue #2116 — ТАРС обязан ОТВЕЧАТЬ ГОЛОСОМ В ШЛЕМ, а не только
        # класть текст в /avatar/command_result. Это решение владельца №12
        # хендоффа и ADR-0055: «все supervisor-ответы строго в
        # /avatar/tts/request с sink=headset».
        #
        # ``_publish_avatar_tts`` существовал, был покрыт целым файлом тестов
        # (test_supervisor_avatar_tts.py) — и НЕ ИМЕЛ НИ ОДНОГО вызова в
        # проде. Замер на роботе 2026-09-08: агент отвечал
        # `content='ТАРС, агент оператора.'`, а /avatar/tts/request оставался
        # пуст и /avatar/tts/audio отдавал 0 байт. Ответ упирался в текстовый
        # топик и умирал там.
        #
        # Озвучиваем только голосовой вход: на текстовую команду из Telegram
        # оператор ждёт текст, а не речь в наушниках.
        self._maybe_speak_agent_reply(source, str(result.get("summary", "")))

        # Журнал ТАРС (§5.4): что сделал, когда, чем кончилось.
        self._record_operator_journal(
            source, str(result.get("summary", "")), tool_names
        )

        result_label = "ok" if result.get("ok", False) else "error"
        if not result.get("ok", False):
            # Различаем «no_tool» от «error» — иначе в метрике сольются
            # два разных operational-сигнала.
            summary = str(result.get("summary", ""))
            if summary.startswith("no_tool"):
                result_label = "no_tool"
            elif summary.startswith("llm_error"):
                result_label = "llm_error"
        self._record_agent_command(
            source=source,
            result=result_label,
            latency_s=(time.monotonic_ns() - started_ns) / 1_000_000_000.0,
        )

    # ── Шаг 4б (issue #1989): пайплайн грипа ─────────────────────────
    # Прямоточный путь (§7.5): /avatar/ptt/result + /avatar/voice_pipeline →
    # transform(text, preset, language) → /voice/tts/request. Без AgentCore,
    # ToolProvider, памяти и истории (инвариант 6c). Pure-логика
    # (классификация, загрузка пресетов, сборка сообщений) — в
    # :mod:`rob_box_supervisor.grip_pipeline`.

    def _build_grip_metrics(self) -> dict[str, Any]:
        """Зарегистрировать / достать метрики пайплайна грипа.

        Паттерн — как ``_build_agent_metrics``: no-op, если rob_box_voice /
        prometheus_client недоступны.
        """
        try:
            from rob_box_voice.observability.metrics import get_metric  # noqa: PLC0415
        except ImportError:
            get_metric = None  # type: ignore[assignment]
        if get_metric is None:
            return {
                "utterances": _NoopLabelCounter(),
                "llm_calls": _NoopLabelCounter(),
                "enabled": False,
            }
        return {
            "utterances": get_metric(
                "counter",
                "avatar_grip_utterances_total",
                "Grip pipeline utterances, labelled by transform mode.",
                labelnames=("mode",),
            ),
            "llm_calls": get_metric(
                "counter",
                "avatar_grip_llm_calls_total",
                "Grip pipeline single-shot LLM calls, labelled by mode.",
                labelnames=("mode",),
            ),
            "enabled": True,
        }

    def _record_grip_utterance(self, mode: str) -> None:
        """Инкрементить ``avatar_grip_utterances_total{mode}``."""
        metrics = self._grip_metrics
        if not metrics.get("enabled", False):
            return
        try:
            metrics["utterances"].labels(mode=mode).inc()
        except Exception as exc:  # noqa: BLE001 — метрики best-effort
            self._log.warning(f"grip_metrics: utterance record failed: {exc}")

    def _record_grip_llm_call(self, mode: str) -> None:
        """Инкрементить ``avatar_grip_llm_calls_total{mode}`` (0|1 на фразу)."""
        metrics = self._grip_metrics
        if not metrics.get("enabled", False):
            return
        try:
            metrics["llm_calls"].labels(mode=mode).inc()
        except Exception as exc:  # noqa: BLE001 — метрики best-effort
            self._log.warning(f"grip_metrics: llm_call record failed: {exc}")

    def _on_grip_voice_pipeline(self, msg: RosString) -> None:
        """ROS-callback ``/avatar/voice_pipeline`` — конфиг из панели шлема.

        Payload — JSON ``{llm_enabled: bool, preset: str, language: str}``.
        Валидируем и кладём в состояние пайплайна (одна точка правды на
        ноде оператора). Битый/неизвестный ввод не валит ноду и не меняет
        предыдущую конфигурацию — только warning.
        """
        raw = (msg.data or "").strip()
        if not raw:
            self._log.warning("GripPipeline: empty voice_pipeline config")
            return
        data = self._try_parse_json(raw)
        if not isinstance(data, dict):
            self._log.warning("GripPipeline: voice_pipeline config not a JSON object")
            return
        llm_enabled = bool(data.get("llm_enabled", False))
        preset = str(data.get("preset", "") or "").strip().lower()
        language = str(data.get("language", "") or "").strip().lower()
        if language and language not in VOICE_LANGUAGES:
            self._log.warning(
                f"GripPipeline: unknown language {language!r} — default {GRIP_DEFAULT_LANGUAGE!r}"
            )
            language = GRIP_DEFAULT_LANGUAGE
        if preset and preset not in VOICE_PRESET_IDS and preset not in GRIP_OFF_PRESETS:
            self._log.warning(
                f"GripPipeline: unknown preset {preset!r} — treated as no-style"
            )
            preset = ""
        self._pipeline_llm_enabled = llm_enabled
        self._pipeline_preset = preset
        self._pipeline_language = language or GRIP_DEFAULT_LANGUAGE
        self._log.info(
            f"GripPipeline: config llm_enabled={llm_enabled} preset={preset!r} "
            f"language={self._pipeline_language!r}"
        )

    @staticmethod
    def _extract_grip_ptt_text(raw: str) -> str:
        """Извлечь STT-текст из ``/avatar/ptt/result`` (pure).

        Канонический payload — голый String с распознанным текстом. Допускаем
        и JSON ``{"text": ...}`` (симметрия с /avatar/stt/result) — на случай,
        если шаг 05 (stt-роутер) решит слать обёртку. Битый JSON → сам raw.
        """
        if not raw:
            return ""
        stripped = raw.strip()
        data = AvatarSupervisor._try_parse_json(stripped)
        if isinstance(data, dict):
            text = data.get("text")
            if isinstance(text, str) and text.strip():
                return text.strip()
            return ""
        return stripped

    def _on_grip_ptt_result(self, msg: RosString) -> None:
        """ROS-callback ``/avatar/ptt/result`` — фраза с левого грипа.

        Запускает прямоточную трансформацию по текущей конфигурации панели
        и публикует результат в ``/voice/tts/request`` (динамики робота,
        §7.5). Целевая аудитория — люди рядом с роботом; оператор слышит
        свою реплику акустически (динамики → воздух → уши) и через
        PTT-гейт клиента, side-tone в шлем не нужен (инвариант 6b §7.4:
        два выхода звука не смешиваются).
        """
        text = self._extract_grip_ptt_text(msg.data or "")
        if not text:
            self._log.debug("GripPipeline: empty ptt result — ignored")
            return
        self._log.info(f"GripPipeline: ptt text={text[:80]!r}")
        self._run_grip_pipeline(text)

    @staticmethod
    def _classify_grip_preset(preset: str, llm_enabled: bool) -> str:
        """Классифицировать выбор панели в режим трансформации (pure).

        ``direct`` → 0 вызовов LLM (дословно); ``translate`` / ``style`` →
        1 вызов. Неизвестный пресет при включённом LLM не стилизуем молча —
        дословный TTS честнее (ADR-0018).
        """
        from rob_box_supervisor.grip_pipeline import classify_preset  # noqa: PLC0415

        return classify_preset(preset, llm_enabled, VOICE_PRESET_IDS)

    def _run_grip_pipeline(self, text: str) -> None:
        """Прямоточная трансформация фразы грипа.

        Решает по конфигурации: дословно (0 вызовов) или ровно один LLM-вызов
        (перевод/стиль). При любом сбое LLM — честный fallback на дословный
        TTS (0 доп. вызовов).
        """
        mode = self._classify_grip_preset(
            self._pipeline_preset, self._pipeline_llm_enabled
        )
        if mode == "direct":
            self._publish_grip_tts(text)
            self._record_grip_utterance(mode)
            return
        rewritten = self._grip_transform_once(
            text, self._pipeline_preset, self._pipeline_language
        )
        if rewritten is None:
            self._log.info("GripPipeline: LLM transform unavailable — direct TTS")
            self._publish_grip_tts(text)
            self._record_grip_utterance(f"{mode}_fallback")
            return
        # Текст переписан LLM — язык override, чтобы tts_node говорил на
        # выбранном языке (AV-28). Дословные ветки выше язык НЕ трогают.
        self._publish_grip_tts(rewritten, language=self._pipeline_language)
        self._record_grip_utterance(mode)

    def _grip_transform_once(
        self, text: str, preset_key: str, language: str
    ) -> Optional[str]:
        """Ровно один LLM-вызов трансформации текста пресетом.

        Возвращает переписанный текст или ``None`` (нет провайдера / промпта,
        ошибка, пустой или байт-в-байт идентичный ответ) — вызывающий код
        тогда уходит в дословный TTS. ``tool_calls`` ответа игнорируются: у
        этого пути нет ToolProvider (инвариант 6c), и ``complete`` вызывается
        с ``tools=()``.
        """
        from rob_box_supervisor.grip_pipeline import (  # noqa: PLC0415
            build_messages,
            language_label,
            language_prompt_section,
            load_voice_presets,
            select_prompt_section,
        )

        data = load_voice_presets()
        preset_cfg = (data.get("presets") or {}).get(preset_key) or {}
        prompt_text = preset_cfg.get("prompt_text") or ""
        if not prompt_text:
            self._log.warning(
                f"GripPipeline: preset {preset_key!r} has no prompt_text — direct TTS"
            )
            return None
        languages = data.get("languages") or {}
        section = language_prompt_section(languages, language)
        system_prompt = select_prompt_section(prompt_text, section)
        messages = build_messages(
            system_prompt=system_prompt,
            preset_name=str(preset_cfg.get("name") or preset_key),
            target_language_label=language_label(languages, language),
            user_text=text,
        )
        llm = self._grip_llm
        if llm is None:
            llm = self._build_operator_llm()
            if llm is None:
                self._log.warning("GripPipeline: no LLM provider — direct TTS")
                return None
            self._grip_llm = llm
        mode = "translate" if preset_key == "translate" else "style"
        self._record_grip_llm_call(mode)
        try:
            response = asyncio.run(self._grip_complete_once(llm, messages))
        except Exception as exc:  # noqa: BLE001 — LLM не должен валить ноду
            self._log.warning(f"GripPipeline: LLM call failed ({exc!r}) — direct TTS")
            return None
        rewritten = (response.content or "").strip()
        if not rewritten or rewritten == text:
            self._log.info("GripPipeline: empty/identical LLM result — direct TTS")
            return None
        return rewritten

    async def _grip_complete_once(self, llm: Any, messages: list[Any]) -> Any:
        """Один вызов LLM грипа: ``tools=()`` (нет ToolProvider) + таймаут.

        Отдельная async-обёртка, чтобы ``_grip_transform_once`` был простым и
        таймаут висел сверху над сетью, не блокируя ROS-callback навсегда.
        """
        timeout_s = 30.0
        return await asyncio.wait_for(
            llm.complete(messages, tools=()), timeout=timeout_s
        )

    def _publish_grip_tts(
        self, text: str, language: Optional[str] = None
    ) -> None:
        """Опубликовать текст в ``/voice/tts/request`` (динамики робота, §7.5).

        Прямоточный пайплайн грипа (issue #1989, §7.5):
        ``/avatar/ptt/result`` → ``/avatar/voice_pipeline`` →
        ``transform`` → ``/voice/tts/request``. Целевая аудитория —
        люди рядом с роботом, поэтому грип говорит голосом робота из
        динамиков (тот же канал, что и инструмент ``say``).

        Payload — JSON ``{ssml, language?, priority: "operator"}``:
        ``ssml`` обязателен (tts_node читает ssml в dialogue_callback),
        ``priority="operator"`` (ADR-0056 §3.5, _TTS_PRIORITY_PREEMPTS)
        чтобы voice-планировщик вставил реплику сразу за текущей
        озвучкой и положил её поверх очереди динамика — иначе грип
        «глохнет» в очереди перед текущей фразой личности.
        ``language`` передаём ТОЛЬКО когда текст переписан LLM и должен
        звучать на выбранном языке (AV-28); дословный текст остаётся на
        языке оператора без override.

        Шлем (sink="headset", ``/avatar/tts/request``) сюда НЕ идёт —
        инвариант 6b §7.4: «Два выхода звука не смешиваются».
        Собственные реплики ТАРС остаются в шлеме через ``_publish_avatar_tts``.

        Issue #2096 — drop при пустом text (раньше публиковали
        ``<speak></speak>``, tts_node получал пустой SSML и райзил
        MiniMax "text is empty" → CRITICAL в deploy-логе).

        Issue #2137 — регресс против §7.5: ``_publish_grip_tts`` ошибочно
        слал в ``/avatar/tts/request`` (sink=headset), и оператор слышал
        «сам себя» в шлеме, а люди рядом — ничего. Фикс: обратно на
        ``/voice/tts/request`` с ``priority="operator"``.
        """
        # Issue #2096 — пустой text → DROP (warning). Раньше уходило в
        # /avatar/tts/request, tts_node ловил MiniMax bad-request, лог
        # CRITICAL в deploy-issue (см. PR #NNNN).
        if not text or not text.strip():
            self._log.warning(
                f"GripPipeline: voice_tts_request skipped — empty text "
                f"(language={language!r})"
            )
            return

        # voice-vr 12 (issue #2197): единый сборщик SSML — ``Utterance``.
        # XML-экранирование &, <, > делает сам сборщик. Для ``/voice/tts/request``
        # ``sink`` НЕ включаем: канал по контракту — динамики робота, шлем
        # (sink=headset) — это отдельный ``_publish_avatar_tts`` (инвариант 6b).
        utterance = Utterance(
            text=text,
            sink=Sink.SPEAKERS,
            priority=GRIP_TTS_SOURCE,  # "operator" — REPLACE-priority
            language=language,
        )
        payload = utterance.to_request()
        payload.pop("sink", None)
        try:
            msg = RosString()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._tts_request_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"GripPipeline: voice_tts_request publish failed: {exc}")

    def _maybe_speak_agent_reply(self, source: str, summary: str) -> None:
        """Озвучить ответ агента в шлем, если вход был голосовым (#2116).

        Вынесено из ``_on_avatar_command`` отдельным методом ради
        CC-бюджета (ADR-0021): встроенная ветка подняла сложность
        обработчика с 17 до 19 и завалила ``scripts/lint/cc_budget.py``.
        Поднимать baseline вместо выноса нельзя — именно так выигрыш от
        шага 1 плана миграции и откатывается.

        Озвучиваем только голосовой вход: на текстовую команду из Telegram
        оператор ждёт текст, а не речь в наушниках. Пустой ``summary``
        отсекает сам ``_publish_avatar_tts`` (#2096).
        """
        if source != "quest":
            return
        if not self._param_bool("speak_agent_replies", True):
            return
        self._publish_avatar_tts(summary)

    def _publish_avatar_tts(
        self,
        text: str,
        language: Optional[str] = None,
        voice: Optional[str] = None,
        sink: str = "headset",
        request_id: Optional[str] = None,
    ) -> str:
        """ADR-0055 / issue #1993 — публикация собственной реплики ТАРС в шлем.

        Используется для всех supervisor-ответов вида «принял», «не умею»,
        «камера повёрнута», «не понял» и т.п. — раньше они ходили в
        ``/voice/tts/request`` (если вообще ходили), теперь строго в
        ``/avatar/tts/request`` с ``sink="headset"``, чтобы попасть в шлем
        оператора через новый обратный канал (ADR-0055).

        ADR-0077 / issue #2138.A.3 — preview-канал для picker'а голосов.
        ``sink="preview"`` уходит на тот же ``/avatar/tts/request``, но
        tts_node его ловит в ``_on_avatar_tts_request`` отдельной веткой и
        делает pure-synth БЕЗ _synthesize_and_play (НЕ идёт в FIFO/ALSA,
        НЕ публикует /avatar/tts/audio, НЕ публикует /voice/audio/speech).
        Результат — bytes в mp3/wav контейнере — публикуется в
        ``/avatar/preview_voice/audio`` (см. preview_audio_sink.ts §1).

        request_id: для sink="preview" caller ЗНАЕТ request_id (он пришёл
        от quest_node через /avatar/preview_voice), и его надо протащить
        до tts_node для корреляции preview_voice_audio/done/error с ws_server.
        Для sink="headset" — генерируем свой uuid4().hex[:8] (старое поведение).

        Returns:
            request_id (uuid hex8), чтобы caller мог логировать/коррелировать
            с /avatar/tts/error и /voice/tts/finished от tts_node.

        Issue #2096 — drop при пустом text. Возвращает пустую строку
        request_id, чтобы caller не пытался коррелировать несуществующий
        запрос.
        """
        import uuid as _uuid

        # Issue #2096 — пустой text → DROP (warning) + возврат пустого
        # request_id. Зеркальная защита к _on_avatar_tts_request в tts_node:
        # upstream не должен слать в /avatar/tts/request нечего синтезировать.
        if not text or not text.strip():
            self._log.warning(
                f"avatar_supervisor: avatar_tts_request skipped — empty text "
                f"(language={language!r}, voice={voice!r}, sink={sink!r})"
            )
            return ""

        if request_id is not None:
            # Caller-provided (preview). Не регенерим.
            rid = request_id
        else:
            rid = _uuid.uuid4().hex[:8]
        # voice-vr 12 (issue #2197): единый сборщик SSML — ``Utterance``.
        # XML-экранирование &, <, > делает сам сборщик; sink приходит как
        # строка от вызывающего (headset|preview) — нормализуем через Sink.
        # speech_id НЕ добавляем: tts_node генерит свой через
        # ``chunk_data.get("speech_id", str(_uuid.uuid4()))``, а в payload'е
        # request_id уже служит уникальным ключом.
        utterance = Utterance(
            text=text,
            sink=sink,
            voice=voice,
            language=language,
        )
        payload = {
            "request_id": rid,
            **utterance.to_request(),
        }
        try:
            msg = RosString()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._avatar_tts_request_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(
                f"avatar_supervisor: avatar_tts_request publish failed: {exc}"
            )
        return rid


class _NoopLabelCounter:
    """Заглушка для метрик при отсутствии ``rob_box_voice`` (минимальный CI-env).

    Имеет те же ``labels(...).inc()``, что и prometheus Counter, но
    ничего не считает. Это позволяет ``self._agent_metrics`` быть
    всегда dict-ом, без ``if rob_box_voice is not None`` в каждом
    методе record_*.

    ``labels`` — обычный атрибут (не слот), чтобы unit-тесты могли
    подменить его на spy и перехватить вызовы ``.inc()``.
    """

    def labels(self, *args: Any, **kwargs: Any) -> "_NoopLabelCounter":
        return self

    def inc(self, amount: float = 1.0) -> None:
        return None


class _NoopHistogram:
    """Заглушка для гистограммы latency при отсутствии ``rob_box_voice``.

    Аналогично ``_NoopLabelCounter`` — ``labels`` не слот, чтобы тесты
    могли его подменить.
    """

    def observe(self, amount: float) -> None:
        return None

    def labels(self, *args: Any, **kwargs: Any) -> "_NoopHistogram":
        return self


def main(args: Optional[list] = None) -> None:
    """Console-script entry point: ``ros2 run rob_box_supervisor supervisor_node``.

    Issue #2131: ``rclpy.spin(node)`` (= ``SingleThreadedExecutor``) приводил к
    deadlock-голоду subscriber'а ``/mcp/result``: пока основной поток блокирован
    в ``LLMToolCallAdapter.execute_tool_call_sync.result_event.wait()``,
    callback ``on_result`` не мог быть доставлен → стабильный таймаут 10 с.
    Решение — ``MultiThreadedExecutor`` (как в ``dialogue_node.main``): тогда
    ``ReentrantCallbackGroup`` адаптера реально диспетчеризует callback в
    фоновом потоке. См. ADR-0072.
    """
    if not rclpy.ok():
        rclpy.init(args=args)
    node = AvatarSupervisor()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
