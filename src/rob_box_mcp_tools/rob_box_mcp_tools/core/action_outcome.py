"""Честный разбор исхода Nav2-цели и вызова сервиса (карточка #3789, ADR-0148).

Раньше ``_send_nav_goal`` считал успехом любой ответ Nav2, в том числе
``ABORTED`` и ``CANCELED``, а ``finish_mapping`` / ``load_map`` отвечали
«готово», не дождавшись ответа сервиса. Робот говорил «Приехал», не доехав.

Здесь решение «успех или нет» и фраза о нём строятся кодом из результата
исполнения — LLM только озвучивает готовое. Модуль ROS-free: статусы
``action_msgs/msg/GoalStatus`` продублированы числами (они часть опубликованного
интерфейса ROS 2 и не меняются), чтобы тесты шли без ``rclpy``.
"""

from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional

from rob_box_dialog import knowledge as kn

# action_msgs/msg/GoalStatus
GOAL_STATUS_UNKNOWN = 0
GOAL_STATUS_ACCEPTED = 1
GOAL_STATUS_EXECUTING = 2
GOAL_STATUS_CANCELING = 3
GOAL_STATUS_SUCCEEDED = 4
GOAL_STATUS_CANCELED = 5
GOAL_STATUS_ABORTED = 6

# Виды исхода навигации.
NAV_SUCCEEDED = "succeeded"
NAV_ABORTED = "aborted"
NAV_CANCELED = "canceled"
NAV_TIMEOUT = "timeout"
NAV_REJECTED = "rejected"
NAV_NO_RESPONSE = "no_response"
NAV_SERVER_UNAVAILABLE = "server_unavailable"
NAV_UNKNOWN_STATUS = "unknown_status"
#: Nav2 ответил успехом, но поза на карте за поездку потеряла опору (#4495, ``core.localization_check``).
NAV_LOCALIZATION_LOST = "localization_lost"

# Виды исхода вызова сервиса.
SVC_OK = "ok"
SVC_ERROR = "service_error"
SVC_UNAVAILABLE = "unavailable"
SVC_TIMEOUT = "timeout"
SVC_NO_RESPONSE = "no_response"

# action_msgs/srv/CancelGoal: коды ответа (часть опубликованного интерфейса ROS 2).
CANCEL_ERROR_NONE = 0
CANCEL_ERROR_REJECTED = 1
CANCEL_ERROR_UNKNOWN_GOAL_ID = 2
CANCEL_ERROR_GOAL_TERMINATING = 3

# Виды исхода отмены цели (дополняют SVC_* — те же значения для сбоя вызова). Успехи — контракт с фразой кода диалога
# (``kn.STOP_TOOL_PHRASES``, #3789): значения оттуда.
CANCEL_CONFIRMED = kn.STOP_CONFIRMED
CANCEL_NO_ACTIVE_GOAL = kn.STOP_IDLE
CANCEL_REFUSED = "cancel_refused"


@dataclass(frozen=True)
class Outcome:
    """Исход операции: ``kind`` — машинный признак, ``error``/``message`` — текст."""

    kind: str
    success: bool
    error: Optional[str] = None
    message: Optional[str] = None


_NAV_FAILURES = {
    NAV_ABORTED: ("Nav2 прервал навигацию (ABORTED)", "Не доехал: навигация прервана"),
    NAV_CANCELED: ("Навигация отменена (CANCELED)", "Движение отменено, до точки не доехал"),
    NAV_TIMEOUT: ("Навигация превысила таймаут", "Не доехал до точки за отведённое время"),
    NAV_REJECTED: ("Nav2 отклонил цель", "Цель недостижима"),
    NAV_NO_RESPONSE: ("Nav2 не ответил на цель", "Навигация недоступна"),
    NAV_SERVER_UNAVAILABLE: ("Nav2 action server недоступен", "Навигация недоступна"),
    NAV_LOCALIZATION_LOST: (
        "Nav2 ответил SUCCEEDED, но локализация за поездку потеряна (#4495)",
        "Не уверен, что доехал: потерял, где нахожусь на карте",
    ),
}


def nav_outcome(kind: str, status: Optional[int] = None) -> Outcome:
    """Исход навигации по виду; успех — только ``NAV_SUCCEEDED``."""
    if kind == NAV_SUCCEEDED:
        return Outcome(NAV_SUCCEEDED, True)
    if kind in _NAV_FAILURES:
        error, message = _NAV_FAILURES[kind]
        return Outcome(kind, False, error, message)
    return Outcome(
        NAV_UNKNOWN_STATUS,
        False,
        f"Nav2 вернул неожиданный статус цели: {status!r}",
        "Не уверен, что доехал: навигация не подтвердила прибытие",
    )


#: Допуск цели по положению: ``xy_goal_tolerance`` goal checker'а Nav2 (``docker/main/config/nav2/nav2_params.yaml``).
NAV_XY_TOLERANCE_M = 0.25
#: Цель достигнута быстрее - робот уже стоял на ней: Nav2 не успел бы проехать (живой 04.11: Goal succeeded за 60 мс,
#: #4344).
NAV_INSTANT_S = 1.0


def arrival_kind(elapsed_s: float, start_distance_m: Optional[float] = None) -> str:
    """Как достигнута цель (#4344): робот уже стоял на ней (``kn.NAV_ALREADY_THERE``) или доехал (``kn.NAV_ARRIVED``).

    Уже стоял - Nav2 ответил успехом быстрее :data:`NAV_INSTANT_S` или путь до цели по обратной связи Nav2
    (наибольший ``distance_remaining``; ``None`` - обратной связи не было) не длиннее допуска цели.
    """
    if elapsed_s < NAV_INSTANT_S:
        return kn.NAV_ALREADY_THERE
    if start_distance_m is not None and start_distance_m <= NAV_XY_TOLERANCE_M:
        return kn.NAV_ALREADY_THERE
    return kn.NAV_ARRIVED


def start_distance(distances: Iterable[float]) -> Optional[float]:
    """Путь до цели в начале поездки по обратной связи Nav2: наибольший ``distance_remaining`` (первая обратная связь
    до расчёта пути бывает нулём), ``None`` - обратной связи не было."""
    seen = [float(d) for d in distances]
    return max(seen) if seen else None


def nav_data(kind: str, elapsed_s: Optional[float] = None, start_distance_m: Optional[float] = None) -> Dict[str, Any]:
    """Данные исхода навигации для фразы кода (``kn.NAV_TOOL_PHRASES``): вид исхода, а у успеха - как достигнута цель.

    Неуспех кладёт вид исхода и в ``reason``: по нему диалог берёт фразу отказа (``motion.rejected.<вид>``).
    """
    data: Dict[str, Any] = {kn.NAV_OUTCOME_FIELD: kind}
    if kind != NAV_SUCCEEDED:
        data["reason"] = kind
        return data
    # время не измерено - не «мгновенно»: «уже на точке» только по доказательству
    elapsed = float("inf") if elapsed_s is None else float(elapsed_s)
    data["elapsed_s"] = None if elapsed_s is None else round(elapsed, 3)
    data["start_distance_m"] = None if start_distance_m is None else round(start_distance_m, 3)
    data[kn.NAV_ARRIVAL_FIELD] = arrival_kind(elapsed, start_distance_m)
    return data


def classify_nav_status(status: Optional[int]) -> Outcome:
    """Статус ``GoalStatus`` из результата Nav2 → исход.

    Успех — строго ``STATUS_SUCCEEDED``. Всё остальное (в том числе
    ``None``, ``UNKNOWN``, ``EXECUTING``) — неуспех: «не доказано» значит «не доехал».
    """
    if status == GOAL_STATUS_SUCCEEDED:
        return nav_outcome(NAV_SUCCEEDED)
    if status == GOAL_STATUS_ABORTED:
        return nav_outcome(NAV_ABORTED)
    if status == GOAL_STATUS_CANCELED:
        return nav_outcome(NAV_CANCELED)
    return nav_outcome(NAV_UNKNOWN_STATUS, status)


def classify_service_call(
    *, ready: bool, completed: bool, response: Any, label: str, error: Optional[str] = None
) -> Outcome:
    """Итог вызова ROS-сервиса: нет сервиса / таймаут / пустой ответ / отказ / ok.

    ``response`` — то, что вернул ``future.result()``. Сервисы без поля
    ``success`` (``std_srvs/Empty``) считаются успешными, если ответ пришёл.
    Если у ответа есть ``success`` и он ложный — это отказ сервиса, причина
    берётся из ``message``. ``error`` — исключение будущего (если было).
    """
    if not ready:
        return Outcome(SVC_UNAVAILABLE, False, f"{label}: сервис недоступен")
    if not completed:
        return Outcome(SVC_TIMEOUT, False, f"{label}: таймаут ожидания ответа")
    if error:
        return Outcome(SVC_NO_RESPONSE, False, f"{label}: вызов завершился ошибкой ({error})")
    if response is None:
        return Outcome(SVC_NO_RESPONSE, False, f"{label}: пустой ответ сервиса")
    if getattr(response, "success", True) is False:
        reason = getattr(response, "message", "") or "без причины"
        return Outcome(SVC_ERROR, False, f"{label}: сервис вернул отказ ({reason})")
    return Outcome(SVC_OK, True)


def classify_cancel_response(response: Any) -> Outcome:
    """Ответ ``CancelGoal`` (отмена всех целей) → исход; успех только по ответу сервера.

    * ``ERROR_NONE`` и есть отменяемые цели — отмена принята сервером;
    * ``ERROR_NONE`` без целей, ``UNKNOWN_GOAL_ID``, ``GOAL_TERMINATING`` — отменять
      было нечего (цель уже завершилась или её не было): это успех, но другой —
      фраза «и так стою», а не «останавливаюсь»;
    * ``REJECTED`` и любой незнакомый код — сервер не принял отмену, это неуспех.

    Сбой самого вызова (нет сервиса, таймаут, пустой ответ) разбирает
    ``classify_service_call`` до этой функции.
    """
    code = getattr(response, "return_code", None)
    canceling = getattr(response, "goals_canceling", None)
    if code == CANCEL_ERROR_NONE and canceling:
        return Outcome(CANCEL_CONFIRMED, True, None, "Останавливаюсь")
    if code in (CANCEL_ERROR_NONE, CANCEL_ERROR_UNKNOWN_GOAL_ID, CANCEL_ERROR_GOAL_TERMINATING):
        return Outcome(CANCEL_NO_ACTIVE_GOAL, True, None, "Я и так стою: активной цели нет")
    return Outcome(
        CANCEL_REFUSED,
        False,
        f"Nav2 не принял отмену цели (код {code!r})",
        "Не уверен, что остановился: навигация не подтвердила отмену",
    )
