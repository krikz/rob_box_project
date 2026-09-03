"""Unit-тесты ``streams.alerts`` (robot_alert evaluation, чистая логика)."""

from __future__ import annotations

import pytest

from rob_box_quest.streams.alerts import (
    Alert,
    AlertThresholds,
    CODE_BATTERY_LOW,
    CODE_ROBOT_STUCK,
    CODE_WIFI_WEAK,
    LEVEL_ERROR,
    LEVEL_WARN,
    evaluate_alerts,
)


# Короткие выдержки для тестов — иначе каждый тест с hold ждал бы 10 с.
QUICK = AlertThresholds(
    battery_low_pct=20,
    battery_hysteresis_pct=5,
    wifi_weak_dbm=-75,
    wifi_hysteresis_dbm=5,
    stuck_timeout_s=3.0,
    stuck_cmd_eps=0.05,
    hold_ms=0,  # для большинства тестов: выдержка не нужна
)


# --- BATTERY_LOW ------------------------------------------------------------


def test_battery_low_raises_at_threshold():
    alerts = evaluate_alerts(
        now_ms=1000, thresholds=QUICK, battery_pct=20
    )
    codes = [a.code for a in alerts]
    assert CODE_BATTERY_LOW in codes
    assert alerts[0].level == LEVEL_WARN


def test_battery_low_above_threshold_is_clean():
    alerts = evaluate_alerts(
        now_ms=1000, thresholds=QUICK, battery_pct=21
    )
    assert alerts == []


def test_battery_low_hysteresis_clears_at_low_plus_hyst():
    """При pct=20 активен; на pct=25 (low + 5) уже снимается."""
    # 1) поднимаем алёрт.
    raised = evaluate_alerts(
        now_ms=1000, thresholds=QUICK, battery_pct=20
    )
    battery_alert = next(a for a in raised if a.code == CODE_BATTERY_LOW)

    # 2) сразу после подъёма pct=24 — ещё активен (ниже low+hyst=25).
    still_active = evaluate_alerts(
        now_ms=2000,
        thresholds=QUICK,
        battery_pct=24,
        prev_alerts=[battery_alert],
    )
    assert any(a.code == CODE_BATTERY_LOW for a in still_active)

    # 3) pct=25 — снят.
    cleared = evaluate_alerts(
        now_ms=3000,
        thresholds=QUICK,
        battery_pct=25,
        prev_alerts=[battery_alert],
    )
    assert cleared == []


def test_battery_low_no_data_no_alert():
    """battery_pct=-1 (sentinel «нет источника») → алёрта нет."""
    alerts = evaluate_alerts(now_ms=1000, thresholds=QUICK, battery_pct=-1)
    assert alerts == []


def test_battery_low_hold_blocks_below_threshold():
    """Выдержка 10 с: алёрт НЕ поднимается сразу, только после hold.

    QuestNode хранит prev_alerts между тиками и передаёт их; первый
    тик, на котором условие появилось, ещё не имеет prev (since_ms
    неизвестен), но в нашем API мы зашиваем since_ms=now_ms на
    первом появлении. Тест имитирует «первое появление» через явный
    prev_alert с since_ms=now_0.
    """
    with_hold = AlertThresholds(hold_ms=10_000)
    seed = Alert(code=CODE_BATTERY_LOW, level=LEVEL_WARN, since_ms=0)
    # Тик +5 с: ещё не прошёл.
    mid = evaluate_alerts(
        now_ms=5_000,
        thresholds=with_hold,
        battery_pct=10,
        prev_alerts=[seed],
    )
    assert mid == []
    # Тик +11 с: поднялся.
    held = evaluate_alerts(
        now_ms=11_000,
        thresholds=with_hold,
        battery_pct=10,
        prev_alerts=[seed],
    )
    assert any(a.code == CODE_BATTERY_LOW for a in held)


def test_battery_low_args_pct_passed_through():
    alerts = evaluate_alerts(now_ms=1000, thresholds=QUICK, battery_pct=12)
    battery = next(a for a in alerts if a.code == CODE_BATTERY_LOW)
    assert battery.args["pct"] == 12


# --- WIFI_WEAK --------------------------------------------------------------


def test_wifi_weak_raises_at_threshold():
    alerts = evaluate_alerts(
        now_ms=1000, thresholds=QUICK, wifi_rssi=-75
    )
    assert any(a.code == CODE_WIFI_WEAK for a in alerts)


def test_wifi_weak_above_threshold_is_clean():
    alerts = evaluate_alerts(
        now_ms=1000, thresholds=QUICK, wifi_rssi=-74
    )
    assert alerts == []


def test_wifi_weak_hysteresis_clears_at_weak_plus_hyst():
    """При rssi=-75 активен; на rssi=-70 (-75 + 5) снимается."""
    raised = evaluate_alerts(
        now_ms=1000, thresholds=QUICK, wifi_rssi=-75
    )
    wifi_alert = next(a for a in raised if a.code == CODE_WIFI_WEAK)

    cleared = evaluate_alerts(
        now_ms=2000,
        thresholds=QUICK,
        wifi_rssi=-70,
        prev_alerts=[wifi_alert],
    )
    assert cleared == []


def test_wifi_weak_no_data_no_alert():
    """wifi_rssi=0 (sentinel «нет источника») → алёрта нет."""
    alerts = evaluate_alerts(now_ms=1000, thresholds=QUICK, wifi_rssi=0)
    assert alerts == []


# --- ROBOT_STUCK ------------------------------------------------------------


def test_robot_stuck_raises_when_cmd_present_and_odom_static():
    """cmd есть, odom_motion_s >= timeout → алёрт."""
    alerts = evaluate_alerts(
        now_ms=1000,
        thresholds=QUICK,
        cmd_vel_linear=0.5,
        cmd_vel_angular=0.0,
        odom_motion_s=3.0,
    )
    stuck = next(a for a in alerts if a.code == CODE_ROBOT_STUCK)
    assert stuck.level == LEVEL_ERROR


def test_robot_stuck_clears_when_odom_moves():
    """Алёрт был поднят; odom_motion_s=0 (только что двигался) → снят."""
    stuck = Alert(
        code=CODE_ROBOT_STUCK, level=LEVEL_ERROR, since_ms=0,
    )
    cleared = evaluate_alerts(
        now_ms=2000,
        thresholds=QUICK,
        cmd_vel_linear=0.5,
        cmd_vel_angular=0.0,
        odom_motion_s=0.0,
        prev_alerts=[stuck],
    )
    assert cleared == []


def test_robot_stuck_no_command_no_alert():
    """Команды нет → алёрта быть не может, даже если odom стоит."""
    alerts = evaluate_alerts(
        now_ms=1000,
        thresholds=QUICK,
        cmd_vel_linear=0.0,
        cmd_vel_angular=0.0,
        odom_motion_s=99.0,
    )
    assert alerts == []


def test_robot_stuck_command_below_eps_no_alert():
    """Мизерная команда (< stuck_cmd_eps) → не считается «поехали»."""
    alerts = evaluate_alerts(
        now_ms=1000,
        thresholds=QUICK,
        cmd_vel_linear=0.01,  # ниже дефолтного eps=0.05
        cmd_vel_angular=0.0,
        odom_motion_s=10.0,
    )
    assert alerts == []


def test_robot_stuck_no_odom_no_alert():
    """Odom не приходил → не выдумываем «не едет»."""
    alerts = evaluate_alerts(
        now_ms=1000,
        thresholds=QUICK,
        cmd_vel_linear=0.5,
        cmd_vel_angular=0.0,
        odom_motion_s=None,
    )
    assert alerts == []


def test_robot_stuck_below_timeout_no_alert():
    """Odom стоит 1 с (< timeout=3) → ещё не stuck."""
    alerts = evaluate_alerts(
        now_ms=1000,
        thresholds=QUICK,
        cmd_vel_linear=0.5,
        cmd_vel_angular=0.0,
        odom_motion_s=1.0,
    )
    assert alerts == []


def test_robot_stuck_does_not_require_hold():
    """ROBOT_STUCK выдержкой не ограничен: stuck_timeout_s уже играет роль."""
    with_hold = AlertThresholds(hold_ms=10_000)
    alerts = evaluate_alerts(
        now_ms=1_000,  # всего 1 с с начала → hold НЕ прошёл
        thresholds=with_hold,
        cmd_vel_linear=0.5,
        cmd_vel_angular=0.0,
        odom_motion_s=4.0,  # но stuck_timeout_s уже превышен
    )
    assert any(a.code == CODE_ROBOT_STUCK for a in alerts)


# --- Мульти-алёрт + стабильность since_ms -----------------------------------


def test_since_ms_preserved_while_alert_remains_active():
    """При повторных вызовах с активным prev since_ms не сбрасывается."""
    raised = evaluate_alerts(now_ms=1000, thresholds=QUICK, battery_pct=10)
    first = next(a for a in raised if a.code == CODE_BATTERY_LOW)

    second = evaluate_alerts(
        now_ms=5000,
        thresholds=QUICK,
        battery_pct=10,
        prev_alerts=[first],
    )
    again = next(a for a in second if a.code == CODE_BATTERY_LOW)
    assert again.since_ms == first.since_ms == 1000


def test_multiple_alerts_can_coexist():
    alerts = evaluate_alerts(
        now_ms=1000,
        thresholds=QUICK,
        battery_pct=10,
        wifi_rssi=-80,
        cmd_vel_linear=0.5,
        cmd_vel_angular=0.0,
        odom_motion_s=5.0,
    )
    codes = {a.code for a in alerts}
    assert CODE_BATTERY_LOW in codes
    assert CODE_WIFI_WEAK in codes
    assert CODE_ROBOT_STUCK in codes
    assert len(alerts) == 3


def test_unknown_code_does_not_break_diff():
    """prev_alerts может содержать Alert'ы с неизвестными кодами
    (от старой версии сервера / клиента) — функция их просто игнорирует."""
    strange = Alert(code="UNKNOWN_FUTURE", level=LEVEL_WARN, since_ms=0)
    alerts = evaluate_alerts(
        now_ms=1000,
        thresholds=QUICK,
        battery_pct=10,
        prev_alerts=[strange],
    )
    assert any(a.code == CODE_BATTERY_LOW for a in alerts)


# --- Defaults дублируют status_hud.ts ---------------------------------------


def test_default_thresholds_match_status_hud_constants():
    """Дефолты должны совпадать с ``BATTERY_LOW_PCT=20`` и
    ``WIFI_WEAK_DBM=-75`` из ``webxr_client/src/scene/status_hud.ts``
    (R8, в PR обе стороны цитатой)."""
    defaults = AlertThresholds()
    assert defaults.battery_low_pct == 20
    assert defaults.wifi_weak_dbm == -75
    assert defaults.hold_ms == 10_000  # из задания
    assert defaults.stuck_timeout_s == 3.0  # из задания


@pytest.mark.parametrize("pct", [-1, -100, None])
def test_battery_sentinel_or_none_yields_no_alert(pct):
    """Любой «нет данных» → никакой ложной тревоги."""
    alerts = evaluate_alerts(now_ms=1000, thresholds=QUICK, battery_pct=pct)
    assert alerts == []
