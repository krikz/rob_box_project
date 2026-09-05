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
    ``/avatar/command_result`` + voice-mode swap.

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
import uuid
from typing import Any, Iterator, Mapping, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString

# AV-27 / issue #1919 — импорт SoT голосов. Pure-Python, без rclpy —
# безопасен в любом окружении. Используется в _apply_set_voice и
# _on_preview_voice для валидации voice_id.
from rob_box_voice.tts_voice_registry import voices_for as _voices_for


def _voice_param_key_for(provider: str) -> str:
    """Целевой параметр tts_node для голоса активного провайдера.

    Соответствие задано в src/rob_box_voice/config/tts_node.yaml:
      yandex  → yandex_voice   (tts_node.py:677)
      minimax → minimax_voice  (tts_node.py:716)
      silero  → silero_speaker (tts_node.py:691)

    Это единственное место, где живёт маппинг provider → param-key. Если
    завтра появится новый провайдер — добавить ветку здесь + соответствующее
    объявление параметра в tts_node.yaml + запись в PROVIDER_VOICES.
    """
    if provider == "yandex":
        return "yandex_voice"
    if provider == "minimax":
        return "minimax_voice"
    if provider == "silero":
        return "silero_speaker"
    # Provider без поддержки смены голоса — вызывающий код ловит
    # ``voice_unavailable:provider:voice_id`` через validation.
    return ""


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

# ADR-0027 §3.4 — валидные значения ``voice_input_mode`` на dialogue_node.
# Супервизор — единственная точка, которая имеет право их менять (ADR-0028 S5).
# "off" (W3-1, §3.5 docs/design/dialogue-mode-spec-2026-08-28.md) —
# «диалог off»: блокирует диалоговую ноду ТОЛЬКО для обычных людей у
# ReSpeaker-микрофона; вход оператора (Telegram/Quest) продолжает работать.
VOICE_INPUT_MODES: tuple[str, ...] = (
    "respeaker",
    "quest_passthrough",
    "quest_ttts",
    "quest_stt",
    "quest_llm_formalize",
    "off",
)

# Phase 1 транспорт запроса смены режима голоса. Phase 2 заменит на
# ``SetVoiceMode``-сервис с кастомным IDL (ADR-0028 §4.3) — здесь топик
# достаточен, чтобы не плодить rosidl-интерфейсы ради ради-фазы.
SET_VOICE_MODE_TOPIC: str = "/avatar/set_voice_mode"

# AV-27 / issue #1919 — TTS picker топики (симметрично /avatar/set_voice_mode).
# Payload — JSON в std_msgs/String, как принято в supervisor_node. См.
# docs/architecture/tts-picker-ros-path.md §128-150.
SET_VOICE_TOPIC: str = "/avatar/set_voice"
PREVIEW_VOICE_TOPIC: str = "/avatar/preview_voice"
PREVIEW_VOICE_RESULT_TOPIC: str = "/avatar/preview_voice/result"
PREVIEW_VOICE_AUDIO_TOPIC: str = "/avatar/preview_voice/audio"
PREVIEW_VOICE_ERROR_TOPIC: str = "/avatar/preview_voice/error"
# AV-28 §P7 (issue #1920) — voice style preset / language топики.
# Симметрично /avatar/set_voice_mode и /avatar/set_voice: payload — String
# с одним ID (preset|language) без JSON (для скорости и простоты парсинга).
# Супервизор делает SetParameters на dialogue_node (см. ADR-0028 §S5).
SET_VOICE_PRESET_TOPIC: str = "/avatar/set_voice_preset"
SET_VOICE_LANGUAGE_TOPIC: str = "/avatar/set_voice_language"
# Whitelist preset/language для AV-28 §P7. Должен совпадать с ws_server.
# (Мы не импортируем ws_server — цикл. Источник правды — voice_presets.yaml;
# здесь — копия для runtime-валидации, её сверяет тест
# test_whitelists_match_ws_server_and_yaml.)
#
# Копия была ДВЕ: эта и приватная _AV28_* внутри класса, валидировала
# вторая. Разъехавшись с yaml, они дали молчаливый отказ: ws_server
# отвечал Quest'у voice_set_ack (UI показывал «применилось»), а
# супервизор ронял запрос в applied=False, и оператор об этом не узнавал.
# Так выпали пресет `translate` и языки fr/de/zh/hi. Теперь копия одна.
VOICE_PRESET_IDS: tuple[str, ...] = (
    "technical",
    "street",
    "caveman",
    "business",
    "philosopher",
    "lenin",
    "translate",
)
VOICE_LANGUAGES: tuple[str, ...] = ("ru", "en", "fr", "de", "zh", "hi")
# AV-21 (issue #1913) — супервизор-агент «мозг оператора» (ADR-0028 §1.1).
# Вход: ``/avatar/command`` (std_msgs/String, JSON), выход:
# ``/avatar/command_result``. Полные JSON-схемы — в
# ``docs/architecture/avatar-supervisor-agent.md``. Наполнять вход
# будут карточки-после (AV-22: Quest STT, Telegram-текст).
AVATAR_COMMAND_TOPIC: str = "/avatar/command"
AVATAR_COMMAND_RESULT_TOPIC: str = "/avatar/command_result"
# Вейк-вход оператора (шаг 05, issue #1990). Топик создаёт stt_node
# (wake-роутер); пока шаг 05 не смержен — подписка дремлет (в ROS
# подписка на несуществующий топик безвредна). Payload v1 — как
# /avatar/command, поэтому обработчик тот же (_on_avatar_command).
AGENT_STT_RESULT_TOPIC: str = "/avatar/stt/result"

# Какой ``voice_input_mode`` выставлять на ``dialogue_node`` пока супервизор
# обрабатывает команду оператора. ``"off"`` — «диалог off» (W3-1, полное
# управление оператора; см. dialogue-mode-spec-2026-08-28.md §3.5).
# Переопределяется параметром ``agent_during_voice_mode``.
AGENT_DURING_VOICE_MODE_DEFAULT: str = "off"

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
    (``/avatar/command`` + ``/avatar/stt/result`` → ``AgentCore``).
    Клиент floor-сервисов ходит на ``/avatar_arbiter/*``.
    """

    # ── AGENT_* параметры (AV-21, ТАРС / issue #1988) ───────────────
    # ``agent_enabled`` гейт всего agent-прохода (default true — ТАРС
    # работает сразу). ``system_prompt_file`` — имя файла в
    # ``rob_box_supervisor/prompts/``. ``agent_during_voice_mode`` — какой
    # voice_input_mode выставлять на dialogue_node на время обработки
    # команды (default "off" — "диалог off", полное управление оператора).
    AGENT_ENABLED_PARAM = "agent_enabled"
    SYSTEM_PROMPT_FILE_PARAM = "system_prompt_file"
    AGENT_DURING_VOICE_MODE_PARAM = "agent_during_voice_mode"

    def __init__(self) -> None:
        super().__init__("avatar_supervisor")

        # Параметр mode (default monitor). В monitor — наблюдаем, не
        # вмешиваемся в voice-параметры; в active — применяем. (Арбитраж
        # floor / /avatar/state вынесен в avatar_arbiter, issue #1987.)
        self.declare_parameter("mode", "monitor")
        self._mode: str = str(self.get_parameter("mode").value or "monitor")

        # Логгер ROS (не stdlib logging — для unified rclpy logging).
        self._log = self.get_logger()

        # ADR-0028 S5 — супервизор единственный, кто меняет voice_input_mode
        # на dialogue_node. Phase 1 транспорт — топик (см. SET_VOICE_MODE_TOPIC).
        self.create_subscription(
            RosString, SET_VOICE_MODE_TOPIC, self._on_set_voice_mode, 10
        )
        # AV-28 §P7 (issue #1920) — voice style preset / language топики.
        # Валидируем ID по whitelist (тот же, что в ws_server.py) и выставляем
        # SetParameters на dialogue_node (voice_preset / voice_output_language).
        # Без рестарта dialogue_node — параметр подхватывается на следующей фразе.
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
        # Параметр-клиент к dialogue_node создаётся лениво в active-режиме
        # (в monitor супервизор НЕ трогает чужие параметры — S12).
        self._dialogue_param_client = None
        # AV-27 — параметр-клиент к tts_node. Создаётся лениво в _set_tts_voice_param
        # (минимальный контакт с tts_node, в monitor — не создаётся).
        self._tts_param_client = None

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
        self.declare_parameter(
            self.AGENT_DURING_VOICE_MODE_PARAM, AGENT_DURING_VOICE_MODE_DEFAULT
        )
        # ── LLM / tools / память оператора (issue #1988) ─────────────
        # Дефолты повторяют dialogue_node (один провайдер — deepseek из
        # env). Полный health-fallback chain — отдельная карточка позже.
        self.declare_parameter("llm_providers", "deepseek")
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("max_tokens", 0)
        self.declare_parameter("llm_streaming", False)
        self.declare_parameter("history_max_turns", 10)
        # tool_provider: "ros_mcp" (реальные MCP-инструменты через
        # LLMToolCallAdapter → /mcp/execute), "fake"/"none" — тесты/smoke.
        self.declare_parameter("tool_provider", "ros_mcp")
        # Память оператора — ОТДЕЛЬНАЯ база (не пересекается с личностью;
        # namespace-ы — шаг 10, #2000). Журнал ТАРС — JSONL со
        # схлопыванием повторов (§5.4).
        self.declare_parameter("operator_db_path", "/data/operator_memory.db")
        self.declare_parameter("journal_path", "/data/operator_journal.jsonl")
        self._agent_enabled: bool = bool(
            self.get_parameter(self.AGENT_ENABLED_PARAM).value
        )
        self._system_prompt_file: str = str(
            self.get_parameter(self.SYSTEM_PROMPT_FILE_PARAM).value
            or "operator_system_prompt.txt"
        )
        self._agent_during_voice_mode: str = str(
            self.get_parameter(self.AGENT_DURING_VOICE_MODE_PARAM).value
            or AGENT_DURING_VOICE_MODE_DEFAULT
        )

        # AgentCore создаётся ЛЕНИВО (см. _ensure_agent_core): при
        # ``agent_enabled=false`` мы не должны инстанцировать LLM /
        # Tools / Memory. DSM оператора держим рядом с core — перед
        # каждым входом гоним его в DIALOGUE (см. _run_agent_sync).
        self._agent_core: Any = None
        self._operator_dsm: Any = None
        # Журнал ТАРС (§5.4) — тоже лениво, персист по journal_path.
        self._operator_journal: Any = None
        # Текущий ``voice_input_mode`` dialogue_node — нужен для
        # _voice_mode_swap (восстановить прежнее значение в finally).
        # ``None`` = мы не знаем (первый swap после старта) → в finally
        # НЕ делаем restore, только логируем warning, чтобы не сбросить
        # режим в дефолт по своей инициативе.
        self._voice_input_mode_before_swap: Optional[str] = None

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

        # Метрики (см. rob_box_voice.observability.metrics). Регистрируются
        # лениво через get_metric — если prometheus_client недоступен,
        # это no-op (см. там же is_metrics_enabled). Метрики-объекты
        # кладём в self один раз (lazy init в __init__, не в каждом
        # callback — иначе на каждое сообщение новый Counter).
        self._agent_metrics = self._build_agent_metrics()

        self._log_startup_diagnostics()

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

    # ── voice mode (ADR-0028 S5) ─────────────────────────────────────
    def _on_set_voice_mode(self, msg: RosString) -> None:
        """Обработка ``/avatar/set_voice_mode`` — запрос сменить режим голоса.

        Единственная точка, которая имеет право менять ``voice_input_mode``
        на ``dialogue_node`` (ADR-0028 S5). В monitor-режиме принимаем и
        логируем, но НЕ применяем (S12); в active — выставляем параметр.
        """
        mode = (msg.data or "").strip()
        applied, reason = self._apply_voice_mode(mode)
        # f-string: RcutilsLogger принимает ОДИН msg (issue #1644).
        self._log.info(f"SetVoiceMode: mode={mode} applied={applied} reason={reason}")

    def _apply_voice_mode(self, mode: str) -> tuple[bool, str]:
        """Чистая логика применения ``voice_input_mode`` (тестируется без rclpy).

        Возвращает ``(applied, reason)``. Не валит ноду на битом входе.
        """
        if mode not in VOICE_INPUT_MODES:
            return False, f"invalid_voice_mode: {mode!r}"
        if self._mode != "active":
            return False, MONITOR_MODE_REASON
        try:
            self._set_dialogue_param("voice_input_mode", mode)
        except Exception as exc:  # noqa: BLE001 — отказ не должен валить ноду
            self._log.warning(f"SetVoiceMode: failed to set dialogue param: {exc}")
            return False, f"param_set_failed: {exc}"
        return True, "applied"

    # ── AV-28 §P7 (issue #1920) — voice style preset + language ─────────
    # Симметрично ``_on_set_voice_mode``: супервизор единственный, кто
    # выставляет ``voice_preset``/``voice_output_language`` на ``dialogue_node``
    # (ADR-0028 §S5). Whitelist тот же, что в ws_server.VOICE_PRESET_IDS/
    # VOICE_LANGUAGES — но supervisor не импортирует ws_server (цикл),
    # поэтому держим локальный whitelist и доверяем ws_server'у первый
    # уровень валидации. В monitor-режиме (S12) принимаем и логируем, но
    # НЕ применяем.

    # Валидируем по модульным VOICE_PRESET_IDS / VOICE_LANGUAGES — второй
    # копии списка здесь больше нет (см. комментарий у констант).
    _AV28_PRESET_IDS: frozenset[str] = frozenset(VOICE_PRESET_IDS)
    _AV28_LANGUAGES: frozenset[str] = frozenset(VOICE_LANGUAGES)

    def _on_set_voice_preset(self, msg: RosString) -> None:
        """Обработка ``/avatar/set_voice_preset`` — запрос сменить стиль речи."""
        preset = (msg.data or "").strip()
        applied, reason = self._apply_voice_preset(preset)
        self._log.info(
            f"SetVoicePreset: preset={preset} applied={applied} reason={reason}"
        )

    def _apply_voice_preset(self, preset: str) -> tuple[bool, str]:
        """Чистая логика применения ``voice_preset`` (тестируется без rclpy)."""
        if not preset:
            return False, "empty_voice_preset"
        if preset not in self._AV28_PRESET_IDS:
            return False, f"invalid_voice_preset: {preset!r}"
        if self._mode != "active":
            return False, MONITOR_MODE_REASON
        try:
            self._set_dialogue_param("voice_preset", preset)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"SetVoicePreset: failed to set dialogue param: {exc}")
            return False, f"param_set_failed: {exc}"
        return True, "applied"

    def _on_set_voice_language(self, msg: RosString) -> None:
        """Обработка ``/avatar/set_voice_language`` — запрос сменить язык вывода."""
        language = (msg.data or "").strip()
        applied, reason = self._apply_voice_language(language)
        self._log.info(
            f"SetVoiceLanguage: language={language} applied={applied} reason={reason}"
        )

    def _apply_voice_language(self, language: str) -> tuple[bool, str]:
        """Чистая логика применения ``voice_output_language`` (тестируется без rclpy)."""
        if not language:
            return False, "empty_voice_language"
        if language not in self._AV28_LANGUAGES:
            return False, f"invalid_voice_language: {language!r}"
        if self._mode != "active":
            return False, MONITOR_MODE_REASON
        try:
            self._set_dialogue_param("voice_output_language", language)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"SetVoiceLanguage: failed to set dialogue param: {exc}")
            return False, f"param_set_failed: {exc}"
        return True, "applied"

    def _set_dialogue_param(self, name: str, value: str) -> None:
        """Выставить string-параметр на ``dialogue_node`` через SetParameters.

        Клиент создаётся лениво (первый вызов в active-режиме). Вызов
        асинхронный (rclpy client), результат логируем в done-callback —
        в monitor-режиме метод не вызывается вовсе (S12).
        """
        if self._dialogue_param_client is None:
            from rcl_interfaces.srv import SetParameters  # noqa: PLC0415

            self._dialogue_param_client = self.create_client(
                SetParameters, "/dialogue_node/set_parameters"
            )
        from rcl_interfaces.msg import (  # noqa: PLC0415
            Parameter,
            ParameterType,
            ParameterValue,
        )
        from rcl_interfaces.srv import SetParameters  # noqa: PLC0415

        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = value
        req.parameters = [param]

        future = self._dialogue_param_client.call_async(req)

        def _done(fut) -> None:
            try:
                res = fut.result()
                ok = bool(res and res.results and res.results[0].successful)
                if not ok:
                    self._log.warning(
                        "SetVoiceMode: dialogue_node rejected parameter set"
                    )
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f"SetVoiceMode: parameter set failed: {exc}")

        future.add_done_callback(_done)

    # ── AV-27 TTS picker (issue #1919) ──────────────────────────────
    def _on_set_voice(self, msg: RosString) -> None:
        """Обработка ``/avatar/set_voice`` — сменить голос TTS.

        Дизайн (docs/architecture/tts-picker-ros-path.md §128-150): валидируем
        voice_id по ``tts_voice_registry``, выставляем соответствующий
        строковый параметр на ``tts_node`` через SetParameters
        (lazy-клиент /tts_node/set_parameters). В monitor-режиме НЕ трогаем
        чужие параметры (S12) — только логируем и выходим.

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
        записи (S12). В active — SetParameters на tts_node с параметр-ключом,
        зависящим от активного провайдера (см. ``_voice_param_key_for``).
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
        param_key = _voice_param_key_for(provider)
        if not param_key:
            return False, f"no_param_key_for_provider:{provider}"
        try:
            self._set_tts_voice_param(param_key, voice_id)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"SetVoice: tts_node param-set failed: {exc}")
            return False, f"param_set_failed:{exc}"
        return True, f"applied:{provider}:{param_key}"

    def _set_tts_voice_param(self, name: str, value: str) -> None:
        """Выставить string-параметр голоса на ``tts_node``.

        Клиент создаётся лениво (первый вызов в active-режиме). Аналогично
        :py:meth:`_set_dialogue_param` — разные клиенты потому что SetParameters
        скоуплен на конкретный нод (см. design t_5b9d5d0c §23-27).
        """
        # Ленивый импорт — как в _set_dialogue_param (ADR-0021):
        # supervisor_node обязан импортироваться без ROS-стека.
        from rcl_interfaces.msg import (  # noqa: PLC0415
            Parameter,
            ParameterType,
            ParameterValue,
        )
        from rcl_interfaces.srv import SetParameters  # noqa: PLC0415

        if self._tts_param_client is None:
            self._tts_param_client = self.create_client(
                SetParameters, "/tts_node/set_parameters"
            )
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = value
        req.parameters = [param]

        future = self._tts_param_client.call_async(req)

        def _done(fut) -> None:
            try:
                res = fut.result()
                ok = bool(res and res.results and res.results[0].successful)
                if not ok:
                    self._log.warning("SetVoice: tts_node rejected parameter set")
                else:
                    self._log.info(
                        f"SetVoice: tts_node accepted param {name}={value!r}"
                    )
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f"SetVoice: parameter set future failed: {exc}")

        future.add_done_callback(_done)

    def _on_preview_voice(self, msg: RosString) -> None:
        """Обработка ``/avatar/preview_voice`` — синтезировать preview-фразу.

        Текущий MVP: full preview-synthesis в tts_node — отдельная карточка
        (рефакторинг _synthesize_and_play на pure-synth + playback). Здесь
        supervisor делает валидацию и публикует honest error в
        ``/avatar/preview_voice/error``. Контракт ws_server ↔ клиент
        сохранён полностью — UI увидит причину и отрисует «preview пока
        недоступен, попробуйте позже».
        """
        raw = (msg.data or "").strip()
        if not raw:
            self._log.warning("PreviewVoice: empty payload")
            return
        try:
            data = json.loads(raw)
        except (ValueError, TypeError) as exc:
            self._log.warning(f"PreviewVoice: bad json: {exc}")
            return
        if not isinstance(data, dict):
            self._log.warning("PreviewVoice: payload not dict")
            return
        request_id = data.get("request_id")
        voice_id = data.get("voice_id")
        text = data.get("text")
        provider = data.get("provider")
        if not isinstance(request_id, str) or not request_id:
            self._log.warning("PreviewVoice: missing request_id")
            return
        if not isinstance(voice_id, str) or not voice_id:
            self._publish_preview_error(request_id, "voice_id_required")
            return
        if not isinstance(text, str) or not text:
            self._publish_preview_error(request_id, "text_required")
            return
        # Валидация по реестру.
        if provider and isinstance(provider, str):
            if voice_id not in _voices_for(provider):
                self._publish_preview_error(
                    request_id, f"voice_unavailable:{provider}:{voice_id}"
                )
                return
        else:
            # Без hint — ищем где знают.
            known_in = [
                p for p in ("yandex", "minimax", "silero") if voice_id in _voices_for(p)
            ]
            if not known_in:
                self._publish_preview_error(request_id, "voice_unknown")
                return
        # MVP: честная ошибка.
        self._publish_preview_error(
            request_id, "preview_synthesis_not_implemented_in_mvp"
        )

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

    # ── helpers: voice-mode swap (ADR-0028 S5) ─────────────────────────────────────

    @contextlib.contextmanager
    def _voice_mode_swap(self) -> Iterator[None]:
        """Контекст-менеджер «пока оператор работает, личность молчит».

        ADR-0028 S5: единственная точка, которая имеет право менять
        ``voice_input_mode`` на ``dialogue_node`` — супервизор (через
        ``_apply_voice_mode`` / ``_set_dialogue_param``). При входе
        ставим ``agent_during_voice_mode`` (default "off"), при выходе
        (в ``finally``, включая путь с исключением) — восстанавливаем
        предыдущее значение. Если предыдущее неизвестно (``None`` —
        см. :py:meth:`_capture_current_voice_mode`), в finally НЕ делаем
        restore и только логируем — иначе свапнём режим в дефолт по
        своей инициативе.

        Контекст-менеджер намеренно вызывает ``_apply_voice_mode`` (а
        не пишет в dialogue_node напрямую) — это и есть «через
        супервизор», а не side-door (карточка §4). В monitor-режиме
        ``_apply_voice_mode`` возвращает ``(False, MONITOR_MODE_REASON)``,
        и свап фактически не применяется — но try/finally дисциплина
        сохраняется (тест-инвариант AC #8: «voice_input_mode
        восстановлен даже если LLM упал»).
        """
        prev_mode = self._voice_input_mode_before_swap
        # Входим в режим «оператор работает».
        applied_in, reason_in = self._apply_voice_mode(self._agent_during_voice_mode)
        if not applied_in:
            # В monitor — это ожидаемо; в active — повод для warn (но
            # НЕ ошибка, чтобы не валить обработку команды).
            self._log.debug(
                f"voice_mode_swap.enter: mode={self._agent_during_voice_mode} "
                f"applied={applied_in} reason={reason_in}"
            )
        try:
            yield
        finally:
            # Восстанавливаем ТОЛЬКО если знаем предыдущее значение И
            # оно отличается от текущего. ``None`` = «не знаем»
            # (см. _capture_current_voice_mode) — НЕ делаем restore,
            # иначе свапнём dialogue_node в дефолт по своей инициативе.
            if prev_mode is not None and prev_mode != self._agent_during_voice_mode:
                applied_out, reason_out = self._apply_voice_mode(prev_mode)
                if not applied_out:
                    self._log.warning(
                        f"voice_mode_swap.exit: failed to restore mode={prev_mode} "
                        f"reason={reason_out}"
                    )
            elif prev_mode is None:
                self._log.debug(
                    "voice_mode_swap.exit: previous mode unknown, no restore"
                )
            # Сбрасываем snapshot: следующий swap начнёт с чистого
            # состояния (``prev_mode`` будет снова захвачен в
            # _on_avatar_command ДО входа в swap).
            self._voice_input_mode_before_swap = None

    def _capture_current_voice_mode(self) -> Optional[str]:
        """Захватить текущий ``voice_input_mode`` dialogue_node для swap.

        Phase 1 (монитор): у нас нет гарантированного способа узнать
        текущее значение (нет GetParameters клиента к dialogue_node).
        Возвращаем ``None`` — swap использует его как «не пытаться
        restore в finally». Phase 2 (active-режим) заменит это на
        настоящий GetParameters.ack.

        Тест-инвариант (AC #8): если LLM бросит исключение после
        capture, finally-ветка _voice_mode_swap НЕ пытается
        восстанавливать ``None — иначе свапнём dialogue_node в дефолт
        по своей инициативе.
        """
        return None

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

        Импорты ленивые: ``rob_box_harness`` — opt dep для
        ``rob_box_supervisor``. Любой сбой сборки логируем и возвращаем
        ``(None, None)`` — нода публикует ``agent_unavailable`` и
        продолжает жить (ADR-0018: честный FAIL, а не падение ноды).
        """
        try:
            from rob_box_harness.core.agent_core import AgentCore  # noqa: PLC0415
            from rob_box_harness.core.dialogue_state_machine import (  # noqa: PLC0415
                DialogueStateMachine,
            )
        except ImportError as exc:
            self._log.warning(f"_build_agent_core_sync: import failed: {exc}")
            return (None, None)

        llm = self._build_operator_llm()
        if llm is None:
            return (None, None)
        tools = self._build_operator_tools()
        if tools is None:
            return (None, None)
        memory = self._build_operator_memory()
        system_prompt = self._load_operator_system_prompt()
        if not system_prompt:
            self._log.warning("_build_agent_core_sync: operator system prompt empty")
            return (None, None)
        skill_prompts = self._load_operator_skill_prompts()
        dsm = DialogueStateMachine()
        try:
            core = AgentCore(
                llm=llm,
                tools=tools,
                memory=memory,
                dsm=dsm,
                system_prompt=system_prompt,
                skill_prompts=skill_prompts,
                narrow_tools_to_skill=False,
                use_streaming=self._param_bool("llm_streaming", False),
                history_trim_limit=self._param_int("history_max_turns", 10),
                llm_settings=self._build_operator_llm_settings(),
            )
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"_build_agent_core_sync: AgentCore build failed: {exc}")
            return (None, None)
        try:
            core.set_active_skill("operator.speech")
        except Exception:  # noqa: BLE001 — срез опционален
            pass
        # Журнал ТАРС создаём вместе с core (персист — best-effort).
        self._operator_journal = self._build_operator_journal()
        return (core, dsm)

    def _build_operator_llm(self) -> Any:
        """Построить LLM-провайдер оператора (дефолт — deepseek из env).

        Single-provider достаточно для 4а (решение Шифу). Полный
        health-fallback chain как у dialogue_node — отдельная карточка.
        """
        try:
            from rob_box_harness.providers import (  # noqa: PLC0415
                DEEPSEEK_DEFAULT_BASE_URL,
                DEEPSEEK_DEFAULT_MODEL,
                LLM_PROVIDER_REGISTRY,
                build_deepseek_provider,
            )
        except ImportError as exc:
            self._log.warning(f"_build_operator_llm: providers import failed: {exc}")
            return None
        chain_raw = self._param_str("llm_providers", "deepseek")
        names = [p.strip().lower() for p in chain_raw.split(",") if p.strip()] or [
            "deepseek"
        ]
        built: list[Any] = []
        for name in names:
            entry = LLM_PROVIDER_REGISTRY.get(name)
            if entry is None:
                self._log.warning(
                    f"_build_operator_llm: unknown provider {name!r} skipped"
                )
                continue
            api_key = os.environ.get(str(entry.get("env_key_var", "") or "")) or None
            base_url = str(
                entry.get("default_base_url", "") or DEEPSEEK_DEFAULT_BASE_URL
            )
            model = str(entry.get("default_model", "") or DEEPSEEK_DEFAULT_MODEL)
            try:
                if name == "minimax":
                    from rob_box_harness.config import LLMConfig  # noqa: PLC0415
                    from rob_box_harness.providers import (  # noqa: PLC0415
                        build_minimax_provider,
                    )

                    provider = build_minimax_provider(
                        LLMConfig(
                            provider="minimax",
                            model=model,
                            api_key=api_key,
                            timeout_s=90.0,
                        )
                    )
                else:
                    provider = build_deepseek_provider(
                        api_key=api_key, base_url=base_url, model=model
                    )
                built.append(provider)
            except Exception as exc:  # noqa: BLE001 — один провайдер не валит цепочку
                self._log.warning(f"_build_operator_llm: {name} build failed: {exc}")
        if not built:
            self._log.warning(
                f"_build_operator_llm: no LLM provider built (chain={names!r})"
            )
            return None
        if len(built) == 1:
            return built[0]
        try:
            from rob_box_harness.health import (  # noqa: PLC0415
                HealthAwareFallbackLLM,
                HealthCache,
            )

            return HealthAwareFallbackLLM(built, cache=HealthCache(), logger=self._log)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"_build_operator_llm: fallback wrap failed: {exc}")
            return built[0]

    def _build_operator_llm_settings(self) -> Any:
        """``LLMSettings`` для AgentCore оператора (temperature/max_tokens)."""
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
        """Память оператора — отдельная SQLite-база (не пересекается с
        личностью). При сбое — InMemoryStore (нода живёт)."""
        db = self._param_str("operator_db_path", "") or "~/.rob_box/operator_memory.db"
        try:
            from rob_box_harness.memory import SQLiteVoiceMemory  # noqa: PLC0415

            store = SQLiteVoiceMemory(db_path=db)
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

    def _load_operator_system_prompt(self) -> str:
        """Прочитать ``operator_system_prompt.txt`` (rob_box_supervisor/prompts).

        Пустой/нет файла → ``""`` (сборка core честно откажется).
        """
        prompts_dir = self._resolve_prompts_dir()
        if prompts_dir is None:
            self._log.warning("operator prompts dir not found (no ament, no source)")
            return ""
        path = prompts_dir / (self._system_prompt_file or "operator_system_prompt.txt")
        try:
            return path.read_text(encoding="utf-8").strip()
        except OSError as exc:
            self._log.warning(f"operator system prompt unreadable: {path} ({exc})")
            return ""

    def _load_operator_skill_prompts(self) -> dict[str, str]:
        """Фрагменты срезов оператора + полный каталог личности (best-effort).

        Срезы ``operator.speech``/``operator.control`` — из пакета
        supervisor (prompts/skills). Фрагменты полного каталога личности —
        из ``rob_box_voice`` (prompts/skills), best-effort: нет пакета /
        файла → просто нет фрагмента, инструменты из каталога остаются.
        """
        from pathlib import Path  # noqa: PLC0415

        loaded: dict[str, str] = {}
        prompts_dir = self._resolve_prompts_dir()
        if prompts_dir is not None:
            # (1) Собственные срезы оператора.
            for name in ("operator.speech", "operator.control"):
                path = prompts_dir / "skills" / f"{name}.txt"
                try:
                    text = path.read_text(encoding="utf-8").strip()
                except OSError:
                    continue
                if text:
                    loaded[name] = text
        # (2) Полный каталог личности — best-effort.
        try:
            from rob_box_core.tool_catalog import skill_names  # noqa: PLC0415

            try:
                from ament_index_python.packages import (  # noqa: PLC0415
                    get_package_share_directory,
                )

                voice_skills = (
                    Path(get_package_share_directory("rob_box_voice"))
                    / "prompts"
                    / "skills"
                )
            except Exception:  # noqa: BLE001 — source-tree fallback (unit-тесты)
                voice_skills = (
                    Path(__file__).resolve().parents[2]
                    / "rob_box_voice"
                    / "prompts"
                    / "skills"
                )
            for skill in skill_names():
                path = voice_skills / f"{skill}.txt"
                try:
                    text = path.read_text(encoding="utf-8").strip()
                except OSError:
                    continue
                if text:
                    loaded[skill] = text
        except Exception as exc:  # noqa: BLE001
            self._log.debug(
                f"_load_operator_skill_prompts: personality fragments skipped: {exc}"
            )
        if loaded:
            self._log.info(f"operator skill fragments: {sorted(loaded)}")
        return loaded

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
          4. ``_voice_mode_swap()`` (try/finally) — личность молчит
             пока мы работаем.
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

        # Снимок «текущего» voice_input_mode ДО swap.apply — нужно
        # для finally-восстановления. В Phase 1 это всегда "unknown"
        # (см. _capture_current_voice_mode), в active-режиме Phase 2
        # заменит на настоящий GetParameters.
        self._voice_input_mode_before_swap = self._capture_current_voice_mode()

        try:
            with self._voice_mode_swap():
                result = self._run_agent_sync(core, payload)
        except Exception as exc:  # noqa: BLE001 — НЕ ДОЛЖНО сбежать из swap
            # ``_voice_mode_swap`` имеет try/finally, но защищаемся от
            # ошибок ВНЕ swap (publish, метрики). Сам swap уже
            # восстановил voice_input_mode.
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
    """Console-script entry point: ``ros2 run rob_box_supervisor supervisor_node``."""
    if not rclpy.ok():
        rclpy.init(args=args)
    node = AvatarSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
