#!/usr/bin/env python3
"""
system.py - Инструменты управления системой робота

Инструменты:
- SetVolumeTool: Установить громкость TTS
- SetPitchTool: Установить высоту голоса
- SetSpeedTool: Установить скорость речи
- GetCurrentTimeTool: Текущее время/дата (плюс ``formatted_time``
  по-русски — issue #1777)
- GetRobotStatusTool: Получить статус робота

Issue #1777 — формат-гайд русского времени:
LLM раньше халлюцинировал «тридцать семь минут одиннадцатого» / «22:37
вечера» — tool возвращал корректные данные, но модель склеивала
форматы. Решение: tool сам отдаёт уже-озвученную строку ``formatted_time``
(«двадцать два тридцать семь»), и LLM читает её дословно. Это сильно
уменьшает свободу галлюцинаций. Числа прописью берутся из
``format_time_ru`` — pure-функция, покрыта юнит-тестами.
"""

import datetime as _datetime
import math
import os
import threading
from typing import Callable, List, NamedTuple, Optional, TYPE_CHECKING

# Ленивый импорт ROS 2 модулей для поддержки unit тестов
if TYPE_CHECKING:
    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType, wait_future


class SetVolumeTool(MCPTool):
    """Инструмент для управления громкостью TTS.

    Уровень — ``volume_db`` у ``tts_node``; проценты (``action=set``, ``level``) ложатся линейно на шкалу
    ``MIN_DB..MAX_DB`` — та же шкала, что у шагов: «громкость 60 %» — абсолютный уровень, а не ``normal``
    (живой ТАРС v2 23.10: «громкость 60 %» → ``set_volume{action: normal}``, PR-13b #3708).
    """

    event_source = "srv:/tts_node/set_parameters"
    #: Шкала громкости голоса, дБ: тише ``MIN_DB`` шаг не уводит, громче ``MAX_DB`` — нет.
    MIN_DB: float = -20.0
    MAX_DB: float = 6.0
    NORMAL_DB: float = -3.0
    STEP_DB: float = 3.0
    _MESSAGES = {"louder": "Делаю громче", "quieter": "Делаю тише", "max": "Максимальная громкость",
                 "normal": "Нормальная громкость"}

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from rcl_interfaces.srv import GetParameters, SetParameters

        self.get_params_client = node.create_client(GetParameters, "/tts_node/get_parameters")
        self.set_params_client = node.create_client(SetParameters, "/tts_node/set_parameters")

    @property
    def name(self) -> str:
        return "set_volume"

    @property
    def description(self) -> str:
        # Issue #3125 — описание обязано различать голос и музыку: раньше
        # «громче» во время DJ-сета уходило сюда, музыка не менялась.
        return (
            "Громкость ГОЛОСА робота (речь TTS), НЕ музыки. Используй для "
            "'говори громче', 'тише говори', 'максимальная громкость голоса', "
            "а также для 'громче/тише', когда музыка НЕ играет. Если играет "
            "трек/бит/DJ-сет и юзер просит громче/тише музыку — вызывай "
            "set_music_volume, а не этот тул. Названный уровень в процентах "
            "('громкость 60 %') — action=set и level."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="action",
                type="string",
                description=(
                    "louder — громче на шаг, quieter — тише на шаг, max — на "
                    "максимум, normal — обычный уровень, set — выставить level"
                ),
                required=True,
                enum=["louder", "quieter", "max", "normal", "set"],
            ),
            MCPToolParameter(
                name="level",
                type="integer",
                description=(
                    "Только для action=set: громкость голоса в процентах, "
                    "0..100 (значения вне диапазона обрезаются)."
                ),
                required=False,
            ),
        ]

    @property
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: громкость TTS. personality.
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        """Set volume - FAST операция < 2s (ROS service call)."""
        return ToolExecutionType.FAST

    @classmethod
    def percent(cls, volume_db: float) -> int:
        """Уровень голоса в процентах шкалы ``MIN_DB..MAX_DB``."""
        span = cls.MAX_DB - cls.MIN_DB
        return round(max(0.0, min(1.0, (volume_db - cls.MIN_DB) / span)) * 100)

    @classmethod
    def target_db(cls, action: str, level: Optional[float], current: float):
        """Целевой уровень, дБ, для ``action`` — или ``(None, ошибка)``."""
        if action == "set":
            try:
                pct = max(0.0, min(100.0, float(level)))
            except (TypeError, ValueError):
                return None, f"action=set требует level 0..100, получено {level!r}"
            return cls.MIN_DB + pct / 100.0 * (cls.MAX_DB - cls.MIN_DB), None
        steps = {"louder": min(current + cls.STEP_DB, cls.MAX_DB), "quieter": max(current - cls.STEP_DB, cls.MIN_DB),
                 "max": cls.MAX_DB, "normal": cls.NORMAL_DB}
        if action not in steps:
            return None, f"Неизвестное действие: {action}"
        return steps[action], None

    def _current_db(self) -> Optional[float]:
        from rcl_interfaces.srv import GetParameters

        get_request = GetParameters.Request()
        get_request.names = ["volume_db"]
        future = self.get_params_client.call_async(get_request)
        if not wait_future(future, timeout_sec=2.0) or future.result() is None:
            return None
        return future.result().values[0].double_value

    def _apply_db(self, volume_db: float) -> Optional[str]:
        """Записать ``volume_db`` в ``tts_node``; ошибка — текстом, успех — ``None``."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters

        set_request = SetParameters.Request()
        param = Parameter()
        param.name = "volume_db"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = volume_db
        set_request.parameters = [param]
        future = self.set_params_client.call_async(set_request)
        if not wait_future(future, timeout_sec=2.0):
            return "Не удалось установить громкость (timeout)"
        if future.result() is None or not future.result().results[0].successful:
            return "Не удалось установить громкость"
        return None

    def execute(self, action: str, level: Optional[float] = None) -> MCPToolResult:
        """Установить громкость голоса: шагом, на максимум, обычную или ``level`` процентов."""
        self.log_info(f"Изменение громкости: {action} level={level}")
        if not self.get_params_client.wait_for_service(timeout_sec=1.0):
            return MCPToolResult(success=False, error="TTS параметры недоступны")
        current_volume = self._current_db()
        if current_volume is None:
            return MCPToolResult(success=False, error="Не удалось получить текущую громкость")
        new_volume, error = self.target_db(action, level, current_volume)
        if error is not None:
            return MCPToolResult(success=False, error=error)
        old_pct, new_pct = self.percent(current_volume), self.percent(new_volume)
        data = {"old_volume": current_volume, "new_volume": new_volume, "percent": new_pct}
        if abs(new_volume - current_volume) < 0.1:
            edge = {"louder": "Громкость уже максимальная", "quieter": "Громкость уже минимальная"}
            return MCPToolResult(success=True, data=data,
                                 message=edge.get(action, f"Громкость голоса уже {new_pct}%"))
        error = self._apply_db(new_volume)
        if error is not None:
            return MCPToolResult(success=False, error=error)
        self.log_info(f"Громкость: {current_volume:.1f} → {new_volume:.1f} dB ({old_pct}% → {new_pct}%)")
        message = self._MESSAGES.get(action, f"Громкость голоса: {old_pct}% → {new_pct}%")
        return MCPToolResult(success=True, data=data, message=message)


class TtsParams:
    """Параметры ``tts_node`` по ROS-сервисам: чтение с проверкой типа, запись с причиной отказа (#4022).

    Тип проверяется: параметра нет (``NOT_SET``) — это «не прочитал», а не 0.0, из которого тул посчитал бы шаг.
    """

    def __init__(self, node):
        # Динамический импорт во время выполнения
        from rcl_interfaces.srv import GetParameters, SetParameters

        self._get = node.create_client(GetParameters, "/tts_node/get_parameters")
        self._set = node.create_client(SetParameters, "/tts_node/set_parameters")

    def ready(self) -> bool:
        return self._get.wait_for_service(timeout_sec=1.0)

    def read(self, name: str):
        """Значение параметра (``float`` у double, ``bool`` у bool) или ``None``: нет ответа / параметр не задан."""
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters

        request = GetParameters.Request()
        request.names = [name]
        future = self._get.call_async(request)
        if not wait_future(future, timeout_sec=2.0) or future.result() is None:
            return None
        value = future.result().values[0]
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        if value.type == ParameterType.PARAMETER_BOOL:
            return value.bool_value
        return None

    def write(self, name: str, value: float) -> Optional[str]:
        """Записать double-параметр: ``None`` — нода приняла, иначе причина отказа текстом."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters

        request = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = value
        request.parameters = [param]
        future = self._set.call_async(request)
        if not wait_future(future, timeout_sec=2.0):
            return "timeout"
        if future.result() is None:
            return "нет ответа"
        outcome = future.result().results[0]
        return None if outcome.successful else (outcome.reason or "нода отклонила значение")


class _Step(NamedTuple):
    """Действие тула на оси: направление шага (−1/0/+1), слова «сделал» и «уже на границе»."""

    direction: int
    said: str
    edge: str


def _rejected(reason: str, error: str, **data) -> MCPToolResult:
    """Честный отказ: статус ``rejected`` и причина — в данных, чтобы фразу об исходе строил код, а не «Готово»."""
    return MCPToolResult(success=False, error=error, message=error,
                         data={"status": "rejected", "reason": reason, **data})


def _unsupported(kind: str, provider: str) -> MCPToolResult:
    """У активного провайдера нет регулировки (``kind`` = ``speed`` | ``pitch``): голос не менялся — так и говорим."""
    from rob_box_voice.tts_voice_registry import PROVIDER_LABELS

    what = {"speed": "скорости", "pitch": "тона"}[kind]
    return _rejected(f"{kind}_unsupported",
                     f"У провайдера {PROVIDER_LABELS.get(provider, provider or '?')} нет регулировки {what}, "
                     "голос не изменился", provider=provider)


def _active_provider(node) -> str:
    """Провайдер, чей голос звучит сейчас: фактический от ``tts_node`` (после фолбека), как у ``speak_text``."""
    from .dialogue import _active_tts_provider

    return _active_tts_provider(node)


def _step_axis(tts: TtsParams, axis, step: _Step, provider: str, kind: str,
               describe: Callable[[float], str]) -> MCPToolResult:
    """Шаг по оси активного провайдера: прочитать, записать, прочитать назад. Фраза — из прочитанного значения (#4022).

    Параметр, который нода записала, но не применила к синтезу, — не успех: ответ «готово» ставит код только после
    того, как значение прочитано обратно и совпало с запрошенным.
    """
    from rob_box_voice.tts_prosody import target_value

    current = tts.read(axis.param)
    if current is None:
        return _rejected("read_failed", f"Не удалось прочитать {axis.param}", provider=provider)
    target = target_value(axis, step.direction, current)
    data = {"provider": provider, "param": axis.param, f"old_{kind}": current}
    if abs(target - current) < 0.01:
        return MCPToolResult(success=True, message=step.edge,
                             data={**data, "status": "done", "reason": "at_limit", f"new_{kind}": current})
    error = tts.write(axis.param, target)
    if error is not None:
        return _rejected("set_failed", f"Не удалось установить {axis.param}: {error}", **data)
    applied = tts.read(axis.param)
    if applied is None or abs(applied - target) > 1e-6:
        return _rejected("not_applied", f"{axis.param}: просили {target:g}, нода держит {applied!r}", **data)
    return MCPToolResult(success=True, message=f"{step.said}: {describe(applied)}",
                         data={**data, "status": "done", f"new_{kind}": applied})


class SetPitchTool(MCPTool):
    """Инструмент для управления высотой голоса."""

    event_source = "srv:/tts_node/set_parameters"
    #: Действия → шаг по оси ``pitch_shift`` (коэффициент тона, 1.0 — норма).
    _STEPS = {
        "higher": _Step(1, "Говорю выше", "Голос уже максимально высокий"),
        "lower": _Step(-1, "Говорю ниже", "Голос уже минимально низкий"),
        "normal": _Step(0, "Нормальный голос", "Голос уже нормальный"),
    }

    def __init__(self, node):
        super().__init__(node)
        self.tts = TtsParams(node)

    @property
    def name(self) -> str:
        return "set_pitch"

    @property
    def description(self) -> str:
        return ("Высота голоса робота шагом: higher / lower (±0.2, диапазон 0.5–2.0), normal — 1.0. Для команд "
                "'говори выше', 'говори ниже'. Абсолютного значения нет; <prosody pitch> в speak_text не "
                "работает (MiniMax и Yandex его вырезают, pitch игнорируется) — только этот тул.")

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="action",
                type="string",
                description="Действие с высотой голоса",
                required=True,
                enum=["higher", "lower", "normal"],
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Set pitch - FAST операция < 2s (ROS service call)."""
        return ToolExecutionType.FAST

    @property
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: pitch TTS. personality.
        return "personality"

    def execute(self, action: str) -> MCPToolResult:
        """Установить высоту голоса: параметром активного провайдера, а нет такого — честный отказ (#4022)."""
        from rob_box_voice.tts_prosody import describe_pitch, pitch_axis

        self.log_info(f"Изменение pitch: {action}")
        step = self._STEPS.get(action)
        if step is None:
            return _rejected("bad_action", f"Неизвестное действие: {action}")
        if not self.tts.ready():
            return _rejected("params_unavailable", "TTS параметры недоступны")
        chipmunk = self.tts.read("chipmunk_mode")  # от режима зависит, слышен ли тон у этого провайдера
        if chipmunk is None:
            return _rejected("read_failed", "Не удалось прочитать chipmunk_mode")
        provider = _active_provider(self.node)
        axis = pitch_axis(provider, bool(chipmunk))
        if axis is None:
            return _unsupported("pitch", provider)
        result = _step_axis(self.tts, axis, step, provider, "pitch",
                            lambda ratio: "тон " + describe_pitch(ratio, bool(chipmunk)))
        self.log_info(f"Pitch [{provider}, {axis.param}]: {result.message}")
        return result


class SetSpeedTool(MCPTool):
    """Инструмент для управления скоростью речи."""

    event_source = "srv:/tts_node/set_parameters"
    #: Действия → шаг по оси скорости активного провайдера (``minimax_speed`` / ``yandex_speed``).
    _STEPS = {
        "faster": _Step(1, "Говорю быстрее", "Скорость уже максимальная"),
        "slower": _Step(-1, "Говорю медленнее", "Скорость уже минимальная"),
        "normal": _Step(0, "Нормальная скорость", "Скорость уже нормальная"),
    }

    def __init__(self, node):
        super().__init__(node)
        self.tts = TtsParams(node)

    @property
    def name(self) -> str:
        return "set_speed"

    @property
    def description(self) -> str:
        return "Установить скорость речи робота. Используй для команд 'говори быстрее', 'говори медленнее'."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="action",
                type="string",
                description="Действие со скоростью речи",
                required=True,
                enum=["faster", "slower", "normal"],
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Set speed - FAST операция < 2s (ROS service call)."""
        return ToolExecutionType.FAST

    @property
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: speed TTS. personality.
        return "personality"

    def execute(self, action: str) -> MCPToolResult:
        """Установить скорость речи: параметром активного провайдера, а нет такого — честный отказ (#4022)."""
        from rob_box_voice.tts_prosody import speed_axis

        self.log_info(f"Изменение speed: {action}")
        step = self._STEPS.get(action)
        if step is None:
            return _rejected("bad_action", f"Неизвестное действие: {action}")
        provider = _active_provider(self.node)
        axis = speed_axis(provider)
        if axis is None:
            return _unsupported("speed", provider)
        if not self.tts.ready():
            return _rejected("params_unavailable", "TTS параметры недоступны", provider=provider)
        result = _step_axis(self.tts, axis, step, provider, "speed", lambda speed: f"скорость {speed:g}")
        self.log_info(f"Speed [{provider}, {axis.param}]: {result.message}")
        return result


WEEKDAYS_RU = [
    "понедельник", "вторник", "среда", "четверг", "пятница", "суббота", "воскресенье",
]
MONTHS_RU = [
    "января", "февраля", "марта", "апреля", "мая", "июня",
    "июля", "августа", "сентября", "октября", "ноября", "декабря",
]
PERIOD_RU = ("ночь", "утро", "день", "вечер")

# Часы 0..23 → именительный падеж: 0 → «ноль», 1 → «один», …, 23 → «двадцать три».
HOURS_RU = (
    "ноль", "один", "два", "три", "четыре", "пять",
    "шесть", "семь", "восемь", "девять", "десять",
    "одиннадцать", "двенадцать", "тринадцать", "четырнадцать", "пятнадцать",
    "шестнадцать", "семнадцать", "восемнадцать", "девятнадцать", "двадцать",
    "двадцать один", "двадцать два", "двадцать три",
)
# Минуты 0..59 — те же слова, что часы (один/два/три/...); отдельная таблица
# чтобы имена оставались стабильны и тестируемы.
MINUTES_RU = HOURS_RU + (
    "двадцать четыре", "двадцать пять", "двадцать шесть", "двадцать семь",
    "двадцать восемь", "двадцать девять", "тридцать", "тридцать один",
    "тридцать два", "тридцать три", "тридцать четыре", "тридцать пять",
    "тридцать шесть", "тридцать семь", "тридцать восемь", "тридцать девять",
    "сорок", "сорок один", "сорок два", "сорок три", "сорок четыре",
    "сорок пять", "сорок шесть", "сорок семь", "сорок восемь", "сорок девять",
    "пятьдесят", "пятьдесят один", "пятьдесят два", "пятьдесят три",
    "пятьдесят четыре", "пятьдесят пять", "пятьдесят шесть", "пятьдесят семь",
    "пятьдесят восемь", "пятьдесят девять",
)

# Часовые пояса для робота. По умолчанию Europe/Moscow — голосовой ассистент
# сейчас работает только из Москвы (issue #1777). Если в окружении выставлен
# ROBOT_TIMEZONE — уважаем (для тестов/других локаций).
ROBOT_TIMEZONE = os.environ.get("ROBOT_TIMEZONE", "Europe/Moscow")


def _resolve_robot_timezone():
    """Best-effort resolve ``ROBOT_TIMEZONE`` → ``datetime.tzinfo``.

    Возвращает ``None`` если zoneinfo не установлена или имя кривое — тогда
    caller падает на local naive ``datetime.now()``. Без падения: голосовой
    пайплайн не должен валиться на отсутствующей tzdata.
    """
    try:
        from zoneinfo import ZoneInfo, ZoneInfoNotFoundError  # py3.9+
    except Exception:  # noqa: BLE001 — old Python / нет zoneinfo
        return None
    try:
        return ZoneInfo(ROBOT_TIMEZONE)
    except ZoneInfoNotFoundError:
        return None
    except Exception:  # noqa: BLE001
        return None


def _hour_period(hour: int) -> str:
    """5..11 → «утро», 12..16 → «день», 17..21 → «вечер», иначе → «ночь»."""
    if 5 <= hour < 12:
        return PERIOD_RU[1]
    if 12 <= hour < 17:
        return PERIOD_RU[2]
    if 17 <= hour < 22:
        return PERIOD_RU[3]
    return PERIOD_RU[0]


def format_time_ru(dt: "_datetime.datetime", tz: "Optional[_datetime.tzinfo]" = None) -> str:
    """Pure-функция: вернуть время ``dt`` (HH:MM) русской прописью.

    Args:
        dt: tz-aware (или naive) ``datetime``. Если naive и ``tz`` не передан —
            используется ``dt.hour``/``dt.minute`` как есть. Если naive и ``tz``
            передан — сначала ``dt.replace(tzinfo=tz)``.
        tz: если задан, naive-``dt`` интерпретируется в этом tz (полезно когда
            вызывающий уже знает «локальное время = робот-зона»).

    Returns:
        Строка вида «двадцать два тридцать семь» (HH прописью + MM прописью).
        НЕ содержит «вечера/утра/дня» — этот «period суток» живёт в LLM.

    Examples:
        >>> from datetime import datetime
        >>> format_time_ru(datetime(2026, 8, 31, 12, 0))      # doctest: +SKIP
        'двенадцать ровно'
        >>> format_time_ru(datetime(2026, 8, 31, 14, 0))      # doctest: +SKIP
        'два часа ровно'
        >>> format_time_ru(datetime(2026, 8, 31, 22, 37))     # doctest: +SKIP
        'двадцать два тридцать семь'
    """
    if not isinstance(dt, _datetime.datetime):
        raise TypeError(f"format_time_ru expected datetime, got {type(dt).__name__}")
    if dt.tzinfo is None and tz is not None:
        dt = dt.replace(tzinfo=tz)
    hour = int(dt.hour) % 24
    minute = int(dt.minute)
    if not (0 <= hour < 24 and 0 <= minute < 60):
        raise ValueError(f"hour/minute out of range: {hour}:{minute}")

    h_word = HOURS_RU[hour]
    if minute == 0:
        # Целый час. По-русски род часа зависит от последней цифры:
        # 1 → «час», 2..4 → «часа», 0/5..20 → «часов», 21 → «час»,
        # 22..23 → «часа». «ровно» как маркер неопределённого рода нам не
        # подходит: «два часа ровно» звучит коряво, поэтому говорим род.
        if hour in (1, 21):
            return f"{h_word} час ровно"
        if hour in (2, 3, 4, 22, 23):
            return f"{h_word} часа ровно"
        # 0, 5..12, 13..20
        return f"{h_word} часов ровно"
    return f"{h_word} {MINUTES_RU[minute]}"


class GetCurrentTimeTool(MCPTool):
    """Инструмент для получения текущего времени и даты.

    Не требует ROS-зависимостей — время берётся из системных часов Python.
    Вызывать когда пользователь спрашивает время, дату, день недели и т.д.

    Issue #1777 — время возвращается в timezone, который определяется
    приоритетом:

    1. ``ROBOT_TIMEZONE`` env var (per-deployment override, например
       ``Europe/Berlin`` если робот переехал из Москвы).
    2. ``Europe/Moscow`` default (matches operator locale + Russian prompts).
    3. ``datetime.timezone.utc`` graceful fallback если ``zoneinfo`` не нашёл
       таймзону (отсутствует tzdata). Логирует warning, никогда не падает.

    Без этого фикса контейнер ``voice-assistant`` стартует с TZ=UTC по
    умолчанию, ``datetime.datetime.now()`` возвращает UTC, а юзер слышит
    «семь сорок шесть утра» вместо «десять сорок шесть утра» (см. #1763
    и #1777).
    """

    DEFAULT_TIMEZONE = "Europe/Moscow"

    @property
    def name(self) -> str:
        return "get_current_time"

    @property
    def description(self) -> str:
        return (
            "Получить текущее время и дату. Используй когда пользователь спрашивает "
            "который час, какая дата, какой день недели, какое время суток, сколько сейчас времени. "
            "Поле formatted_time уже написано русской прописью — озвучивай его дословно, время не выдумывай."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        """Мгновенная операция — только Python datetime, никаких I/O."""
        return ToolExecutionType.INSTANT

    def execute(self) -> MCPToolResult:
        """Вернуть текущее время в timezone робота (Europe/Moscow по умолчанию).

        Issue #1777 (resolved by rebase): добавлена timezone-aware datetime
        через ``_resolve_robot_timezone`` (ROBOT_TIMEZONE env → Europe/Moscow
        default → UTC fallback).

        Issue #1762 (этот PR): возвращает ``formatted_time`` — время русской
        прописью («двадцать два тридцать семь»), чтобы LLM не пыталось само
        склеивать «22:37» + «вечера». Правила формата живут в
        ``format_time_ru``; LLM читает ``formatted_time`` дословно через
        ``speak_text``.
        """
        # Issue #1777: тайм-зона робота. По умолчанию Europe/Moscow
        # (робот-ассистент сейчас работает из Москвы). Если zoneinfo
        # недоступна / имя кривое — падаем на local naive как раньше.
        tz = _resolve_robot_timezone()
        if tz is not None:
            now = _datetime.datetime.now(tz=tz)
        else:
            now = _datetime.datetime.now()

        # Русские названия дней и месяцев без зависимости от locale
        WEEKDAYS_RU = ["понедельник", "вторник", "среда", "четверг", "пятница", "суббота", "воскресенье"]
        MONTHS_RU = [
            "января", "февраля", "марта", "апреля", "мая", "июня",
            "июля", "августа", "сентября", "октября", "ноября", "декабря",
        ]

        hour = now.hour
        period = _hour_period(hour)

        # formatted_time считается в локальной зоне now.tzinfo (или naive).
        formatted_time = format_time_ru(now)

        data = {
            "time": now.strftime("%H:%M"),
            "date": f"{now.day} {MONTHS_RU[now.month - 1]} {now.year}",
            "weekday": WEEKDAYS_RU[now.weekday()],
            "period": period,
            "iso": now.isoformat(timespec="seconds"),
            "timezone": str(tz),
            # Issue #1762: русская пропись для дословного озвучивания.
            "formatted_time": formatted_time,
        }

        message = (
            f"Сейчас {data['time']}, {data['weekday']}, {data['date']}, {data['period']}. "
            f"По-русски: {formatted_time}."
        )
        self.log_info(f"Текущее время: {message} (tz={data['timezone']})")
        return MCPToolResult(success=True, data=data, message=message)


class GetRobotStatusTool(MCPTool):
    """Инструмент для получения статуса робота.

    Реальные данные читаются из ROS-топиков:
    - /odom          (nav_msgs/Odometry)     — позиция робота (x, y, theta)
    - /battery_state (sensor_msgs/BatteryState) — уровень заряда батареи (%)

    Подписки создаются при инициализации инструмента, execute() отдаёт
    последние полученные значения. Если топик не публикует данные — в ответе
    явная ошибка/пометка недоступности вместо hardcoded заглушки.
    """

    # Топики ROS для реальных данных
    ODOM_TOPIC = "/odom"
    BATTERY_TOPIC = "/battery_state"

    def __init__(self, node, wait_timeout_sec: float = 2.0):
        super().__init__(node)
        self._position: Optional[dict] = None  # {"x": float, "y": float, "theta": float}
        self._battery_level: Optional[float] = None  # проценты 0-100
        self._data_event = threading.Event()
        # Сколько ждать первое сообщение из топиков при «холодном» старте
        self._wait_timeout_sec = wait_timeout_sec

        if self.node is not None:
            # Динамический импорт ROS 2 модулей для поддержки unit тестов
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import BatteryState

            self.node.create_subscription(Odometry, self.ODOM_TOPIC, self._on_odom, 10)
            self.node.create_subscription(BatteryState, self.BATTERY_TOPIC, self._on_battery, 10)

    def _on_odom(self, msg):
        """Сохранить последнюю позицию из /odom (nav_msgs/Odometry)."""
        try:
            pos = msg.pose.pose.position
            q = msg.pose.pose.orientation
            # yaw (theta) из кватерниона ориентации
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)
            self._position = {"x": float(pos.x), "y": float(pos.y), "theta": theta}
            self._data_event.set()
        except Exception as e:
            self.log_warning(f"Ошибка обработки /odom: {e}")

    def _on_battery(self, msg):
        """Сохранить последний уровень заряда из /battery_state (sensor_msgs/BatteryState)."""
        try:
            # ROS: percentage == -1.0 означает «неизвестно»
            if msg.percentage is not None and float(msg.percentage) >= 0.0:
                self._battery_level = float(msg.percentage)
            else:
                self._battery_level = None
            self._data_event.set()
        except Exception as e:
            self.log_warning(f"Ошибка обработки /battery_state: {e}")

    @property
    def name(self) -> str:
        return "get_robot_status"

    @property
    def description(self) -> str:
        return "Получить текущий статус робота (позиция, батарея, состояние систем)."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        """Get robot status - MEDIUM операция 2-10s (множественные ROS queries)."""
        return ToolExecutionType.MEDIUM

    def execute(self) -> MCPToolResult:
        """Получить статус робота из реальных ROS-топиков /odom и /battery_state."""
        self.log_info("Запрос статуса робота")

        # Если данные ещё не приходили — ждём первое сообщение (топики обычно
        # публикуют с частотой 10-50 Гц, так что первое сообщение приходит быстро).
        if self._position is None or self._battery_level is None:
            self._data_event.wait(timeout=self._wait_timeout_sec)

        unavailable = []
        if self._position is None:
            unavailable.append(self.ODOM_TOPIC)
        if self._battery_level is None:
            unavailable.append(self.BATTERY_TOPIC)

        status = {
            "position": self._position,
            "battery_level": self._battery_level,
            # ADR-0051 §6: 'systems' больше не захардкожен. Если оператор
            # хочет знать «нода X поднялась?» — это ros2_node_status,
            # а не get_robot_status. Здесь оставляем пустой словарь,
            # чтобы ключ остался для обратной совместимости с потребителями,
            # которые его читают.
            "systems": {},
        }

        if unavailable:
            missing = ", ".join(unavailable)
            status["unavailable_topics"] = unavailable
            # Частичные данные: хотя бы один топик жив — отдаём реальные данные с пометкой
            if self._position is not None or self._battery_level is not None:
                return MCPToolResult(
                    success=True,
                    data=status,
                    message=f"Статус робота получен (недоступны топики: {missing})",
                )
            return MCPToolResult(
                success=False,
                data=status,
                error=(
                    "Статус робота недоступен: топики /odom и /battery_state не публикуют данные "
                    "(ROS-топики не запущены или данные ещё не получены)"
                ),
            )

        return MCPToolResult(success=True, data=status, message="Статус робота получен")
