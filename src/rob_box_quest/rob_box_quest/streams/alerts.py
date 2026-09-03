"""robot_alert evaluation: чистая логика без ROS / WS / Zenoh.

Источник истины: ``docs/architecture/meta-quest-api.md`` §6 (robot_alert
codes BATTERY_LOW / WIFI_WEAK / ROBOT_STUCK), ``docs/adr/0027-meta-quest-ar-control.md``
§2 R7 (battery-warn, live-индикация), §2 R8.

Дизайн:
  - один вызов ``evaluate_alerts(status, prev_alerts, now_ms, thresholds)``
    возвращает *текущее* множество активных алёртов;
  - ``prev_alerts`` нужен для гистерезиса (порог снятия != порог
    включения) и для того, чтобы выдержка ≥10 с применялась только к
    ПЕРВОМУ пересечению порога, а не к каждому тику;
  - отсутствие данных (``battery_pct < 0``, ``wifi_rssi == 0``, ``odom``
    не приходил) → алёрт НЕ выдумываем (см. ``streams/battery.py``: тот
    же принцип, чтобы HUD показывал «—» а не «0%»);
  - пороги заданы как ROS-параметры (``ThresholdDict``) и
    дублируются дефолтами из ``status_hud.ts`` (BATTERY_LOW_PCT=20,
    WIFI_WEAK_DBM=-75). В PR будут обе стороны рядом для сверки.

Тестируется без rclpy, как и остальные ``streams/*`` модули.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping, Optional


# Wire-уровень (meta-quest-api.md §6): type="robot_alert", level ∈ {warn, error}.
LEVEL_WARN = "warn"
LEVEL_ERROR = "error"


# Коды алёртов — открытый enum, клиент показывает ``code`` для неизвестных
# значений (alertText fallback), см. webxr_client/src/scene/alert_toast.ts.
CODE_BATTERY_LOW = "BATTERY_LOW"
CODE_WIFI_WEAK = "WIFI_WEAK"
CODE_ROBOT_STUCK = "ROBOT_STUCK"


@dataclass(frozen=True)
class Alert:
    """Активный алёрт, который надо показать оператору в шлеме.

    ``args`` — словарь значений для подстановки в текст (например,
    ``{"pct": 12}`` для BATTERY_LOW); формат контракта зафиксирован в
    meta-quest-api.md §6 + StatusAggregator.wifi_rssi sentinel rules.
    """

    code: str
    level: str
    args: Mapping[str, object] = field(default_factory=dict)
    # Момент первого пересечения порога (ms от epoch / monotonic — решает
    # caller); используется для гистерезиса + логов.
    since_ms: int = 0


@dataclass(frozen=True)
class AlertThresholds:
    """Пороги для ``evaluate_alerts``.

    Значения подобраны так, чтобы дефолт совпадал с ``status_hud.ts``
    (Wave 3.A / R8) — клиент и сервер не разъехались на «свечке» с
    пустыми параметрами ROS-ноды.
    """

    # Батарея: включаем при pct ≤ battery_low_pct, выключаем при pct ≥
    # battery_low_pct + battery_hysteresis_pct.
    battery_low_pct: int = 20
    battery_hysteresis_pct: int = 5
    # Wi-Fi: включаем при rssi ≤ wifi_weak_dbm, выключаем при
    # rssi ≥ wifi_weak_dbm + wifi_hysteresis_dbm.
    wifi_weak_dbm: int = -75
    wifi_hysteresis_dbm: int = 5
    # ROBOT_STUCK: команда есть (линейная или угловая компонента
    # значимая), а odom не меняется дольше stuck_timeout_s секунд.
    stuck_timeout_s: float = 3.0
    # Минимальная |velocity| от teleop, которая считается «командой
    # ехать». Ниже этого значения считаем, что оператор не пытается
    # ехать → алёрт ROBOT_STUCK не поднимаем.
    stuck_cmd_eps: float = 0.05
    # Выдержка для всех алёртов (BATTERY_LOW / WIFI_WEAK): алёрт
    # поднимается, только если условие выполняется непрерывно в
    # течение ``hold_ms``. Гистерезис отдельно (пороги выключения).
    hold_ms: int = 10_000


# Status snapshot от QuestNode (агрегатор из StatusAggregator.payload +
    # значение odom). Чтобы не зависеть от msgpack и от ROS-структур —
    # принимаем любой объект с нужными атрибутами. ``battery_pct < 0``
    # трактуется как «нет источника» (sentinel, см. ``status.py``).
# (разделили комментарий, чтобы линтер не жаловался на dataclass выше.)


def _is_battery_source_present(battery_pct: int) -> bool:
    """Sentinel -1 = «нет источника», алёрт не выдумываем."""
    return battery_pct is not None and battery_pct >= 0


def _is_wifi_source_present(wifi_rssi: int) -> bool:
    """Sentinel 0 = «нет источника» (см. status.py + status_hud.ts)."""
    return wifi_rssi is not None and wifi_rssi != 0


def _battery_alert_active(
    pct: int,
    thresholds: AlertThresholds,
    prev: Optional[Alert],
) -> bool:
    """Гистерезис: при активном алёрте требуем pct ≥ low + hysteresis,
    иначе пересечения вниз через low уже недостаточно (выдержка
    должна была сработать при включении)."""
    if not _is_battery_source_present(pct):
        return False
    if prev is not None:
        # Уже поднят — выключаем только когда восстановились с запасом.
        return pct < thresholds.battery_low_pct + thresholds.battery_hysteresis_pct
    return pct <= thresholds.battery_low_pct


def _wifi_alert_active(
    rssi: int,
    thresholds: AlertThresholds,
    prev: Optional[Alert],
) -> bool:
    if not _is_wifi_source_present(rssi):
        return False
    if prev is not None:
        return rssi < thresholds.wifi_weak_dbm + thresholds.wifi_hysteresis_dbm
    return rssi <= thresholds.wifi_weak_dbm


def _stuck_alert_active(
    *,
    cmd_vel_linear: Optional[float],
    cmd_vel_angular: Optional[float],
    odom_motion_s: Optional[float],
    thresholds: AlertThresholds,
) -> bool:
    """``cmd_vel_*`` — последний non-zero twist от teleop, ``None`` если
    ничего не приходило (см. ``TeleopController.last_twist``). ``odom_motion_s``
    — секунды с момента последнего видимого изменения позиции; ``None``
    если odom ещё не приходил."""
    if cmd_vel_linear is None or cmd_vel_angular is None:
        return False
    if odom_motion_s is None:
        # Odom вообще не приходил → считать «не едет» нельзя, иначе
        # стартовый safe-stop завалит оператора алёртами. Возвращаем False.
        return False
    cmd_mag = (cmd_vel_linear ** 2 + cmd_vel_angular ** 2) ** 0.5
    if cmd_mag < thresholds.stuck_cmd_eps:
        return False
    return odom_motion_s >= thresholds.stuck_timeout_s


def _passes_hold(
    code: str,
    prev: Optional[Alert],
    now_ms: int,
    hold_ms: int,
) -> bool:
    """True если алёрт «выдержан» (условие держится дольше hold_ms).

    ROBOT_STUCK выдержки не требует — stuck_timeout_s уже играет её роль.
    """
    if hold_ms <= 0:
        return True
    if prev is None:
        # Caller ещё не выставил since_ms — пропускаем. На практике
        # caller выставляет при первом запуске условия, так что эта
        # ветка фактически означает «алёрт только-только появился».
        return False
    return (now_ms - prev.since_ms) >= hold_ms


def evaluate_alerts(
    *,
    now_ms: int,
    thresholds: AlertThresholds = AlertThresholds(),
    battery_pct: Optional[int] = None,
    wifi_rssi: Optional[int] = None,
    cmd_vel_linear: Optional[float] = None,
    cmd_vel_angular: Optional[float] = None,
    odom_motion_s: Optional[float] = None,
    prev_alerts: Iterable[Alert] = (),
) -> list[Alert]:
    """Вернуть список АКТИВНЫХ алёртов на момент ``now_ms``.

    Параметры с ``None`` — источник недоступен (не приходил / отсутствует
    ROS-топик). Чистая функция: один вход → один выход, без побочных
    эффектов. Состояние гистерезиса передаётся через ``prev_alerts``
    (что было активно на предыдущем тике).
    """
    prev_by_code: dict[str, Alert] = {a.code: a for a in prev_alerts}

    battery_prev = prev_by_code.get(CODE_BATTERY_LOW)
    wifi_prev = prev_by_code.get(CODE_WIFI_WEAK)

    battery_active = (
        battery_pct is not None
        and _battery_alert_active(battery_pct, thresholds, battery_prev)
    )
    wifi_active = (
        wifi_rssi is not None
        and _wifi_alert_active(wifi_rssi, thresholds, wifi_prev)
    )
    stuck_active = _stuck_alert_active(
        cmd_vel_linear=cmd_vel_linear,
        cmd_vel_angular=cmd_vel_angular,
        odom_motion_s=odom_motion_s,
        thresholds=thresholds,
    )

    out: list[Alert] = []

    if battery_active and _passes_hold(
        CODE_BATTERY_LOW, battery_prev, now_ms, thresholds.hold_ms
    ):
        since = battery_prev.since_ms if battery_prev else now_ms
        out.append(
            Alert(
                code=CODE_BATTERY_LOW,
                level=LEVEL_WARN,
                args={"pct": int(battery_pct) if battery_pct is not None else None},
                since_ms=since,
            )
        )

    if wifi_active and _passes_hold(
        CODE_WIFI_WEAK, wifi_prev, now_ms, thresholds.hold_ms
    ):
        since = wifi_prev.since_ms if wifi_prev else now_ms
        out.append(
            Alert(
                code=CODE_WIFI_WEAK,
                level=LEVEL_WARN,
                args={"rssi_dbm": int(wifi_rssi) if wifi_rssi is not None else None},
                since_ms=since,
            )
        )

    if stuck_active:
        since = prev_by_code[CODE_ROBOT_STUCK].since_ms if CODE_ROBOT_STUCK in prev_by_code else now_ms
        out.append(
            Alert(
                code=CODE_ROBOT_STUCK,
                level=LEVEL_ERROR,
                args={
                    "cmd_linear": float(cmd_vel_linear) if cmd_vel_linear is not None else 0.0,
                    "cmd_angular": float(cmd_vel_angular) if cmd_vel_angular is not None else 0.0,
                    "odom_motion_s": float(odom_motion_s) if odom_motion_s is not None else 0.0,
                },
                since_ms=since,
            )
        )

    return out


__all__ = [
    "Alert",
    "AlertThresholds",
    "LEVEL_WARN",
    "LEVEL_ERROR",
    "CODE_BATTERY_LOW",
    "CODE_WIFI_WEAK",
    "CODE_ROBOT_STUCK",
    "evaluate_alerts",
]
