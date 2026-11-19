"""Вердикты HIL-проверок группы А (scripts/hil/checks/verdicts.py): PASS/FAIL строит код по raw, без ROS и стенда.

Строки логов — в формате живых прогонов (#3789 04.11, #4365 05.11, #4458): ``docker logs --timestamps`` и проба.
"""

import importlib.util
import py_compile
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
CHECKS = ROOT / "scripts" / "hil" / "checks"
_spec = importlib.util.spec_from_file_location("hil_checks_verdicts", CHECKS / "verdicts.py")
vd = importlib.util.module_from_spec(_spec)
sys.modules["hil_checks_verdicts"] = vd
_spec.loader.exec_module(vd)

PHRASES = vd.load_phrases()
PX = vd.load_proximity()
T0 = 1793828400.0  # 2026-11-04T21:40:00Z


def dk(sec, text):
    """Строка docker logs --timestamps через ``sec`` с после T0."""
    import datetime as dt

    stamp = dt.datetime.fromtimestamp(T0 + sec, dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")
    return f"{stamp}123Z {text}"


def pr(sec, kind, **kv):
    """Строка пробы: эпоха, KIND, sim = sec (RTF 1)."""
    rest = "".join(f" {k}={v}" for k, v in kv.items())
    return f"{T0 + sec:.3f} {kind} sim={sec:.3f}{rest}"


def tts(sec, text):
    return dk(
        sec,
        f"[INFO] [1.0] [tts_node]: 🔊 TTS: speech_id=d1, dialogue_id=None, batch=None 0/1, voice=default, "
        f"lang=default, text={text!r}",
    )


# ── разбор raw ──────────────────────────────────────────────────────────────────────────────


def test_parse_probe_skips_noise_and_reads_fields():
    ev = vd.parse_probe(["zenoh: hello", pr(1.0, "NAV_STATUS", goal="ab12", status="ABORTED"), "", "garbage line"])
    assert [(e.kind, e.fields["status"], e.num("sim")) for e in ev] == [("NAV_STATUS", "ABORTED", 1.0)]


def test_parse_docker_utc_timestamp():
    (e,) = vd.parse_docker([dk(2.5, "[mcp_server]: hi")])
    assert e.t == pytest.approx(T0 + 2.5, abs=1e-3) and e.text == "[mcp_server]: hi"


def test_phrase_matcher_template_and_exact():
    rx = vd.phrase_matcher(PHRASES, "motion.done", "motion.arrived")
    assert rx.match("Приехал.") and rx.match("Приехал на точку хилтест.")
    assert not rx.match("Не доехал: навигация прервалась.")


# ── предусловия ──────────────────────────────────────────────────────────────────────────────

STATUS_ON = (
    "── замок (ros2@10.1.1.10:/tmp/hil.lock)\nHIL on since 2026-11-09T16:34:42Z by krikz@X sandbox=10.1.1.30 "
    "maps=/home/ros2/rob_box_hil/maps_x\n── /controller_manager в графе (сторож в gazebo): 1 (норма HIL — 1)\n"
)
NEED = ["nav2", "followpath-watchdog", "voice-assistant"]


def _inspect(watchdog="running healthy 0"):
    started = "2026-11-04T19:00:00.1Z"
    return vd.parse_inspect(
        [
            f"/nav2 running healthy 0 {started}",
            f"/followpath-watchdog {watchdog} {started}",
            f"/voice-assistant running healthy 0 {started}",
        ],
        T0,
    )


def test_preflight_pass():
    v = vd.v_preflight(
        "hil-checks #4365 2026-11-09T19:00:00Z", "hil-checks", STATUS_ON, _inspect(), NEED, {"ros2@10.1.1.10": 0.3}
    )
    assert v.status == vd.PASS, v.line()


def test_preflight_refuses_restarting_watchdog():
    """Робот 09.11: ``followpath-watchdog|Restarting (126)`` (нет бита исполнения, #4567) — честный отказ."""
    v = vd.v_preflight("hil-checks x", "hil-checks", STATUS_ON, _inspect("restarting none 7"), NEED, {})
    assert v.status == vd.FAIL and "followpath-watchdog: restarting" in v.line()


@pytest.mark.parametrize(
    "lock,owner,status,why",
    [
        ("coord-coverage #4560 2026-11-09T16:31:01Z", "hil-checks", STATUS_ON, "stand.lock: coord-coverage"),
        ("hil-checks x", "hil-checks", "── замок\nнет\n── gazebo: absent -\n", "HIL выключен"),
        ("", "", STATUS_ON, "HIL_CHECKS_OWNER не задан"),
    ],
    ids=["чужой-замок", "hil-выключен", "нет-владельца"],
)
def test_preflight_refuses(lock, owner, status, why):
    v = vd.v_preflight(lock, owner, status, _inspect(), NEED, {})
    assert v.status == vd.FAIL and why in v.line()


def test_preflight_missing_container_and_clock_skew():
    insp = vd.parse_inspect(["Error: No such object: followpath-watchdog"], T0)
    v = vd.v_preflight("a b", "a", STATUS_ON, insp, ["followpath-watchdog"], {"ros2@10.1.1.22": 9.0})
    assert v.status == vd.FAIL and "absent" in v.line() and "+9.0 с" in v.line()


# ── остановка после провала цели (#4435, #4456, #4458) ───────────────────────────────────────


def _fail_then(after):
    return vd.parse_probe(
        [
            pr(0, "NAV_STATUS", goal="n1", status="EXECUTING"),
            pr(0.5, "FP_STATUS", goal="f1", status="EXECUTING"),
            pr(0.6, "MUX", v="+0.150", w="-0.062"),
            pr(10.0, "NAV_STATUS", goal="n1", status="ABORTED"),
        ]
        + after
    )


def test_tool_halt_pass_when_follow_gone_and_mux_zero():
    probe = _fail_then(
        [
            pr(10.1, "FP_STATUS", goal="f1", status="CANCELED"),
            pr(10.2, "MUX", v="+0.000", w="+0.000"),
            pr(13.0, "MUX", v="+0.000", w="+0.000"),
        ]
    )
    assert vd.v_tool_halt(probe, 2.0).status == vd.PASS


def test_tool_halt_fail_on_orphan_driving():
    """3b 05.11: «Goal failed», а через 29 с mux=(+0.150,-0.062) — FollowPath жив."""
    probe = _fail_then([pr(39.0, "MUX", v="+0.150", w="-0.062")])
    v = vd.v_tool_halt(probe, 2.0)
    assert v.status == vd.FAIL and "FollowPath ещё активен: f1" in v.line() and "выход twist_mux ≠ 0" in v.line()


def test_tool_halt_mux_before_grace_is_allowed():
    probe = _fail_then([pr(10.5, "MUX", v="+0.100", w="+0.000"), pr(11.0, "FP_STATUS", goal="f1", status="CANCELED")])
    assert vd.v_tool_halt(probe, 2.0).status == vd.PASS


def test_tool_halt_next_goal_motion_is_not_violation():
    probe = _fail_then(
        [
            pr(10.3, "FP_STATUS", goal="f1", status="CANCELED"),
            pr(20.0, "NAV_STATUS", goal="n2", status="EXECUTING"),
            pr(21.0, "MUX", v="+0.2", w="+0.0"),
        ]
    )
    assert vd.v_tool_halt(probe, 2.0).status == vd.PASS


def test_tool_halt_not_triggered_without_failure():
    probe = vd.parse_probe([pr(0, "NAV_STATUS", goal="n1", status="SUCCEEDED")])
    assert vd.v_tool_halt(probe, 2.0).status == vd.NOT_TRIGGERED


@pytest.mark.parametrize("verdict", [vd.v_tool_halt, lambda p, g: vd.v_orphan_natural(p, [], g)])
def test_halt_ignores_replayed_terminal_before_current_goal(verdict):
    """Ночь 09.11: transient-local CANCELED старой цели ошибочно обвинил новый FollowPath (#4435)."""
    probe = vd.parse_probe(
        [
            pr(858.722, "NAV_STATUS", goal="3442e84f", status="CANCELED"),
            pr(858.792, "NAV_STATUS", goal="89360a90", status="EXECUTING"),
            pr(858.812, "FP_STATUS", goal="4070df4b", status="EXECUTING"),
            pr(859.862, "FP_STATUS", goal="d15a7809", status="EXECUTING"),
            pr(859.862, "FP_STATUS", goal="4070df4b", status="ABORTED"),
            pr(860.8, "MUX", v="+0.15", w="+0.0"),
            pr(918.272, "FP_STATUS", goal="d15a7809", status="SUCCEEDED"),
            pr(918.302, "NAV_STATUS", goal="89360a90", status="SUCCEEDED"),
            pr(918.302, "NAV_RESULT", status="SUCCEEDED"),
        ]
    )
    assert verdict(probe, 2.0).status == vd.NOT_TRIGGERED


def test_tool_halt_replayed_terminal_alone_is_not_a_checked_failure():
    probe = vd.parse_probe([pr(0, "NAV_STATUS", goal="old", status="CANCELED")])
    assert vd.v_tool_halt(probe, 2.0).status == vd.NOT_TRIGGERED


def test_tool_halt_next_goal_before_grace_has_no_stop_window():
    probe = _fail_then(
        [
            pr(10.1, "FP_STATUS", goal="f1", status="CANCELED"),
            pr(10.2, "NAV_STATUS", goal="n2", status="EXECUTING"),
            pr(10.3, "FP_STATUS", goal="f2", status="EXECUTING"),
            pr(12.1, "MUX", v="+0.2", w="+0.0"),
        ]
    )
    assert vd.v_tool_halt(probe, 2.0).status == vd.NOT_TRIGGERED


def test_tool_halt_observed_failure_after_replay_still_fails_on_orphan():
    probe = vd.parse_probe(
        [
            pr(0, "NAV_STATUS", goal="old", status="CANCELED"),
            pr(0.1, "NAV_STATUS", goal="current", status="EXECUTING"),
            pr(0.2, "FP_STATUS", goal="f1", status="EXECUTING"),
            pr(1, "NAV_STATUS", goal="current", status="ABORTED"),
            pr(4, "MUX", v="+0.15", w="+0.0"),
        ]
    )
    assert vd.v_tool_halt(probe, 2.0).status == vd.FAIL
    assert vd.v_orphan_natural(probe, [], 2.0).status == vd.FAIL


def test_halt_counts_both_failures_when_the_same_goal_runs_twice():
    """Проба шлёт переход статуса один раз (probe.py on_status). Та же цель наблюдалась активной,
    ушла в ABORTED, снова стала EXECUTING и снова ABORTED. Оба провала — в этой пробе: первый,
    после которого робот ехал, не должен потеряться из-за повторной активации."""
    probe = vd.parse_probe(
        [
            pr(0, "NAV_STATUS", goal="n1", status="EXECUTING"),
            pr(0.2, "FP_STATUS", goal="f1", status="EXECUTING"),
            pr(5, "NAV_STATUS", goal="n1", status="ABORTED"),
            pr(5.2, "FP_STATUS", goal="f1", status="CANCELED"),
            pr(8, "MUX", v="+0.15", w="+0.0"),
            pr(9, "NAV_STATUS", goal="n1", status="EXECUTING"),
            pr(9.2, "FP_STATUS", goal="f2", status="EXECUTING"),
            pr(12, "NAV_STATUS", goal="n1", status="ABORTED"),
            pr(12.2, "FP_STATUS", goal="f2", status="CANCELED"),
            pr(15, "MUX", v="+0.000", w="+0.000"),
        ]
    )
    bad, checked = vd.halt_violations(probe, 2.0)
    assert [round(vd._sim(e), 1) for e in checked] == [5.0, 12.0]
    assert any("sim=5.00" in line and "twist_mux" in line for line in bad)
    assert not any("sim=12.00" in line for line in bad)


def test_halt_counts_a_goal_seen_only_as_accepted():
    """ACTIVE — это ACCEPTED и EXECUTING (verdicts.py). Цель, которую проба видела только принятой,
    а не исполняемой, всё равно наблюдалась: её ABORTED считается, а не отбрасывается как повтор чужой цели."""
    probe = vd.parse_probe(
        [
            pr(0, "NAV_STATUS", goal="n1", status="ACCEPTED"),
            pr(0.5, "FP_STATUS", goal="f1", status="ACCEPTED"),
            pr(3, "NAV_STATUS", goal="n1", status="ABORTED"),
            pr(6, "MUX", v="+0.0", w="+0.2"),
        ]
    )
    bad, checked = vd.halt_violations(probe, 2.0)
    assert [e.fields.get("goal") for e in checked] == ["n1"]
    assert any("FollowPath ещё активен: f1" in line for line in bad)
    assert any("twist_mux" in line for line in bad)


GOAL = "0123456789abcdef0123456789abcdef"


def _orphan_probe(result="CANCELED", dur=1.7, extra=()):
    return vd.parse_probe(
        [
            pr(0, "NAV_STATUS", goal="n1", status="CANCELED"),
            pr(0.1, "NAV_RESULT", status="CANCELED"),
            pr(1.0, "FP_ACCEPTED", goal=GOAL),
            pr(1.2, "MUX", v="+0.120", w="+0.0"),
            pr(1.0 + dur, "FP_RESULT", goal=GOAL, status=result),
            pr(5.0, "MUX", v="+0.000", w="+0.000"),
        ]
        + list(extra)
    )


WD = [
    dk(
        2.6,
        f"[WARN] [1.0] [followpath_watchdog]: #4458 сирота FollowPath goal_id={GOAL} t=2.6: NavigateToPose n1 "
        "в CANCELED дольше 1.5 с — отмена follow_path и cmd_vel=0",
    )
]


def test_orphan_synthetic_pass():
    v = vd.v_orphan_synthetic(_orphan_probe(), vd.parse_docker(WD), 2.0)
    assert v.status == vd.PASS, v.line()
    assert "сирота ехала: 1" in v.line()


@pytest.mark.parametrize(
    "probe,wd",
    [
        (_orphan_probe(dur=3.0), WD),  # погасил поздно
        (_orphan_probe(result="SUCCEEDED"), WD),  # доехала сама — сторож не погасил
        (_orphan_probe(), []),  # в журнале сторожа строки нет
        (_orphan_probe(extra=[pr(4.0, "MUX", v="+0.1", w="+0")]), WD),  # после отмены ещё едет
    ],
)
def test_orphan_synthetic_fail(probe, wd):
    assert vd.v_orphan_synthetic(probe, vd.parse_docker(wd), 2.0).status == vd.FAIL


def test_orphan_synthetic_not_triggered_without_room():
    probe = vd.parse_probe([pr(0, "ORPHAN_SKIP", why="мало_места", free="0.30")])
    assert vd.v_orphan_synthetic(probe, [], 2.0).status == vd.NOT_TRIGGERED
    assert vd.v_orphan_synthetic([], [], 2.0).status == vd.NO_DATA


def test_orphan_natural_not_triggered_when_bt_halts_follow_itself():
    probe = vd.parse_probe(
        [
            pr(0, "NAV_STATUS", goal="n1", status="EXECUTING"),
            pr(0.5, "FP_STATUS", goal="f1", status="EXECUTING"),
            pr(9.9, "FP_STATUS", goal="f1", status="CANCELED"),
            pr(10.0, "NAV_STATUS", goal="n1", status="ABORTED"),
            pr(10.1, "NAV_RESULT", status="ABORTED"),
        ]
    )
    v = vd.v_orphan_natural(probe, [], 2.0)
    assert v.status == vd.NOT_TRIGGERED and "сироты не было" in v.line()


def test_orphan_natural_pass_when_watchdog_cleans_up():
    probe = _fail_then([pr(11.6, "FP_STATUS", goal="f1", status="CANCELED"), pr(12.5, "MUX", v="+0.0", w="+0.0")])
    v = vd.v_orphan_natural(probe, vd.parse_docker(WD), 2.0)
    assert v.status == vd.PASS and "сирот (FollowPath активен после провала цели): 1" in v.line()


# ── abort контроллера: FollowPath уже принят, progress checker абортит (#4770) ──────────────


def _abort_probe(nav_end="ABORTED", fp_end="ABORTED", arrived=False, extra=()):
    """Цель принята, FollowPath ехал, корпус удержан, контроллер абортил."""
    rows = [
        pr(0.0, "HOLD_START", ahead="1.50"),
        pr(1.0, "NAV_STATUS", goal="n1", status="EXECUTING"),
        pr(1.2, "FP_STATUS", goal="f1", status="EXECUTING"),
        pr(1.5, "MUX", v="+0.080", w="+0.000"),
        pr(2.0, "FP_HELD", goal="f1"),
        pr(2.0, "HOLD", v="+0.00", w="+0.00"),
        pr(32.0, "NAV_PROGRESS", n="1"),
        pr(32.0, "FP_STATUS", goal="f1", status=fp_end),
        pr(32.1, "NAV_STATUS", goal="n1", status=nav_end),
        pr(32.2, "NAV_RESULT", status=nav_end),
        pr(33.0, "MUX", v="+0.000", w="+0.000"),
        pr(34.0, "HOLD_END"),
    ]
    if arrived:
        rows.append(pr(32.3, "NAV_RESULT", status="SUCCEEDED"))
    rows += list(extra)
    return vd.parse_probe(rows)


ABORT_NAV2 = [
    dk(32.0, "[ERROR] [32.0] [controller_server]: Failed to make progress"),
    dk(32.0, "[WARN] [32.0] [follow_path]: Aborting handle"),
]


def test_controller_abort_pass_when_accepted_follow_aborts_and_stops():
    v = vd.v_controller_abort(_abort_probe(), vd.parse_docker(ABORT_NAV2), 2.0, 30.0)
    assert v.status == vd.PASS, v.line()
    assert "наблюдавшихся целей FollowPath: 1" in v.line()
    assert "проверок abort: 1" in v.line()


def test_controller_abort_not_triggered_without_accepted_follow():
    """План не построен — это nav_aborted, не abort контроллера. Не PASS."""
    probe = vd.parse_probe(
        [
            pr(0.0, "HOLD_START", ahead="1.50"),
            pr(1.0, "NAV_STATUS", goal="n1", status="EXECUTING"),
            pr(40.0, "NAV_STATUS", goal="n1", status="ABORTED"),
            pr(40.1, "NAV_RESULT", status="ABORTED"),
        ]
    )
    v = vd.v_controller_abort(probe, [], 2.0, 30.0)
    assert v.status == vd.NOT_TRIGGERED, v.line()
    assert "FollowPath не был принят" in v.line()


def test_controller_abort_fail_when_still_driving_after_grace():
    extra = [pr(35.0, "MUX", v="+0.080", w="+0.000"), pr(35.0, "FP_STATUS", goal="f2", status="EXECUTING")]
    v = vd.v_controller_abort(_abort_probe(extra=extra), vd.parse_docker(ABORT_NAV2), 2.0, 30.0)
    assert v.status == vd.FAIL, v.line()


def test_controller_abort_fail_on_false_arrived():
    v = vd.v_controller_abort(_abort_probe(arrived=True), vd.parse_docker(ABORT_NAV2), 2.0, 30.0)
    assert v.status == vd.FAIL and "ложный" in v.line()


def test_controller_abort_fail_when_allowance_restarted():
    """Без terminate_all progress checker снова даёт 30 с: второй Aborting handle — FAIL."""
    nav2 = ABORT_NAV2 + [dk(62.0, "[WARN] [62.0] [follow_path]: Aborting handle")]
    v = vd.v_controller_abort(_abort_probe(), vd.parse_docker(nav2), 2.0, 30.0)
    assert v.status == vd.FAIL and "Aborting handle: 2" in v.line()


def test_controller_abort_counts_only_observed_follow_goals():
    """#4647: терминал без ACTIVE в этой пробе в счётчик не входит."""
    rows = [
        pr(0.0, "FP_STATUS", goal="replayed", status="ABORTED"),
        pr(1.0, "NAV_STATUS", goal="n1", status="EXECUTING"),
        pr(1.2, "FP_STATUS", goal="f1", status="EXECUTING"),
        pr(2.0, "FP_HELD", goal="f1"),
        pr(2.0, "HOLD", v="+0.00", w="+0.00"),
        pr(32.0, "NAV_PROGRESS", n="1"),
        pr(32.0, "FP_STATUS", goal="f1", status="ABORTED"),
        pr(32.1, "NAV_STATUS", goal="n1", status="ABORTED"),
        pr(32.2, "NAV_RESULT", status="ABORTED"),
        pr(33.0, "MUX", v="+0.000", w="+0.000"),
    ]
    v = vd.v_controller_abort(vd.parse_probe(rows), vd.parse_docker(ABORT_NAV2), 2.0, 30.0)
    assert v.status == vd.PASS, v.line()
    assert "наблюдавшихся целей FollowPath: 1" in v.line()
    assert "replayed" not in v.line()


def test_controller_abort_not_triggered_when_goal_succeeds_despite_hold():
    v = vd.v_controller_abort(_abort_probe(nav_end="SUCCEEDED", fp_end="SUCCEEDED"), [], 2.0, 30.0)
    assert v.status == vd.NOT_TRIGGERED


def test_controller_abort_no_data_without_probe():
    assert vd.v_controller_abort([], [], 2.0, 30.0).status == vd.NO_DATA


# ── kill клиента и отмена с хоста (#4435, матрица) ──────────────────────────────────────────


def _drop_rows(killed=True, canceled=True, fp_end="CANCELED", mux_late=False, arrived=False):
    rows = [
        pr(1.0, "NAV_STATUS", goal="n1", status="EXECUTING"),
        pr(1.2, "FP_STATUS", goal="f1", status="EXECUTING"),
        pr(1.5, "CLIENT_PID", pid="4242"),
        pr(2.0, "MUX", v="+0.080", w="+0.000"),
    ]
    if killed:
        rows.append(pr(3.0, "CLIENT_KILLED", pid="4242", signal="TERM"))
    if canceled:
        rows.append(pr(3.0, "HOST_CANCEL", action="navigate_to_pose"))
    rows += [
        pr(3.4, "FP_STATUS", goal="f1", status=fp_end),
        pr(3.5, "NAV_STATUS", goal="n1", status="CANCELED" if canceled or killed else "EXECUTING"),
        pr(4.0, "MUX", v="+0.000", w="+0.000"),
    ]
    if mux_late:
        rows.append(pr(6.5, "MUX", v="+0.050", w="+0.000"))
    if arrived:
        rows.append(pr(3.6, "NAV_RESULT", status="SUCCEEDED"))
    return rows


def test_client_kill_pass_when_follow_stops_without_cancel_goal():
    rows = _drop_rows(canceled=False)
    v = vd.v_client_kill(vd.parse_probe(rows), 2.0)
    assert v.status == vd.PASS, v.line()
    assert "наблюдавшихся целей FollowPath: 1" in v.line()


def test_client_kill_not_triggered_without_kill_or_follow():
    assert vd.v_client_kill(vd.parse_probe(_drop_rows(killed=False, canceled=False)), 2.0).status == vd.NOT_TRIGGERED
    bare = [pr(1.0, "CLIENT_KILLED", pid="1", signal="TERM"), pr(2.0, "NAV_STATUS", goal="n1", status="CANCELED")]
    assert vd.v_client_kill(vd.parse_probe(bare), 2.0).status == vd.NOT_TRIGGERED


def test_client_kill_fail_when_still_driving_or_arrived():
    assert vd.v_client_kill(vd.parse_probe(_drop_rows(canceled=False, mux_late=True)), 2.0).status == vd.FAIL
    assert vd.v_client_kill(vd.parse_probe(_drop_rows(canceled=False, arrived=True)), 2.0).status == vd.FAIL


def test_host_cancel_pass_and_not_triggered_without_cancel():
    v = vd.v_host_cancel(vd.parse_probe(_drop_rows(killed=False)), 2.0)
    assert v.status == vd.PASS, v.line()
    assert vd.v_host_cancel(vd.parse_probe(_drop_rows(killed=False, canceled=False)), 2.0).status == vd.NOT_TRIGGERED


def test_host_cancel_counts_only_observed_follow():
    rows = [pr(0.0, "FP_STATUS", goal="replayed", status="ABORTED")] + _drop_rows(killed=False)
    v = vd.v_host_cancel(vd.parse_probe(rows), 2.0)
    assert v.status == vd.PASS and "replayed" not in v.line()
    assert "наблюдавшихся целей FollowPath: 1" in v.line()


def test_drop_no_data():
    assert vd.v_client_kill([], 2.0).status == vd.NO_DATA
    assert vd.v_host_cancel([], 2.0).status == vd.NO_DATA


# ── фразы тулов движения (#3789) ─────────────────────────────────────────────────────────────

ABORT_RAW = [
    dk(0.0, "[INFO] [1.0] [dialog_node]: 📤 Отправлен запрос 519d9082: navigate_to_coordinates"),
    dk(
        35.0,
        "[WARN] [1.0] [mcp_server]: ❌ Инструмент navigate_to_coordinates завершился с ошибкой: "
        "Nav2 прервал навигацию (ABORTED)",
    ),
]


def test_nav_aborted_pass():
    v = vd.v_nav_aborted(vd.parse_docker(ABORT_RAW + [tts(35.4, "Не доехал: навигация прервалась.")]), PHRASES)
    assert v.status == vd.PASS, v.line()


def test_nav_aborted_fail_when_only_transport_timeout_spoke():
    """HIL 04.11 21:41: «Не могу поехать.» по таймауту транспорта 10 с, фраза по ABORTED не прозвучала."""
    raw = [
        ABORT_RAW[0],
        dk(
            10.0,
            "[ERROR] [1.0] [dialog_node]: ⏱️ Timeout ожидания результата для "
            "navigate_to_coordinates (request_id: 519d9082)",
        ),
        tts(10.01, "Не могу поехать."),
        ABORT_RAW[1],
    ]
    v = vd.v_nav_aborted(vd.parse_docker(raw), PHRASES)
    assert v.status == vd.FAIL and "транспорт не дождался тула" in v.line()


def test_nav_aborted_false_success_is_fail():
    v = vd.v_nav_aborted(vd.parse_docker(ABORT_RAW + [tts(36.0, "Приехал.")]), PHRASES)
    assert v.status == vd.FAIL and "ложный успех" in v.line()


def test_nav_aborted_not_triggered_on_timeout_outcome():
    raw = [
        ABORT_RAW[0],
        dk(
            120.0,
            "[WARN] [1.0] [mcp_server]: ❌ Инструмент navigate_to_coordinates завершился с "
            "ошибкой: Навигация превысила таймаут",
        ),
    ]
    v = vd.v_nav_aborted(vd.parse_docker(raw), PHRASES)
    assert v.status == vd.NOT_TRIGGERED and "исхода ABORTED не было" in v.line()


STOP_RAW = [
    dk(0.0, "[INFO] [1.0] [dialog_node]: 🧾 [command] verdict=command actions=['stop_navigation']"),
    dk(0.01, "[INFO] [1.0] [mcp_server]: [stop_navigation] Остановка навигации"),
    dk(0.02, "[INFO] [1.0] [mcp_server]: [stop_navigation] Отмена подтверждена: no_active_goal"),
]


def test_stop_idle_lie_from_hil_2151_is_fail():
    """HIL 04.11 21:51:18: no_active_goal → «Приехал.» — ложь об успехе."""
    v = vd.v_stop(vd.parse_docker(STOP_RAW + [tts(0.05, "Приехал.")]), PHRASES, "motion.stop.idle", "stop_idle")
    assert v.status == vd.FAIL and "Приехал" in v.line()


def test_stop_idle_pass():
    v = vd.v_stop(vd.parse_docker(STOP_RAW + [tts(0.05, "Я и так стою.")]), PHRASES, "motion.stop.idle", "stop_idle")
    assert v.status == vd.PASS and "Отмена подтверждена: no_active_goal" in v.line()


def test_stop_not_triggered_by_tier1():
    """HIL 04.11 23:05: «Робот, стой» — Tier-1 stop_all, тул не зовётся."""
    raw = [dk(0.0, "[INFO] [1.0] [dialog_node]: 🧭 [turn-v2] tier1 kind=stop_all args={} turn=t1")]
    v = vd.v_stop(vd.parse_docker(raw), PHRASES, "motion.stop.idle", "stop_idle")
    assert v.status == vd.NOT_TRIGGERED and "tier1: stop_all" in v.line()


def test_stop_active_needs_goal_canceled_by_tool():
    voice = vd.parse_docker(STOP_RAW + [tts(0.05, "Останавливаюсь.")])
    done = vd.parse_probe([pr(0, "NAV_RESULT", status="SUCCEEDED")])
    assert vd.v_stop(voice, PHRASES, "motion.stop.confirmed", "stop_active", done).status == vd.NOT_TRIGGERED
    canceled = vd.parse_probe([pr(0, "NAV_RESULT", status="CANCELED")])
    assert vd.v_stop(voice, PHRASES, "motion.stop.confirmed", "stop_active", canceled).status == vd.PASS


# ── зона безопасности (#4365) и звук защиты в HIL (#4470) ──────────────────────────────────────


def _wall_probe(drift=0.006):
    rows = [pr(0, "PROBE_START", cmd="wall"), pr(1, "CMD", v="+0.10", w="+0.00"), pr(9.0, "EM_ON", gt="(2.578,-1.598)")]
    rows += [
        pr(9.5 + 0.2 * i, "TRACK", x=f"{2.570 + drift * i / 17:.3f}", y="-1.598", yaw="178", v="0", w="0")
        for i in range(18)
    ]
    rows += [pr(13.1, "PUSH_END", secs="12.1", em="yes"), pr(15.1, "SCAN_END", front="0.667"), pr(15.2, "PROBE_DONE")]
    return vd.parse_probe(rows)


GUARD = [
    dk(6.0, "[INFO] [1.0] [proximity_guard]: 🛡️ зона clear → warn (до корпуса 0.39 м) в точке (+0.70, +0.10)"),
    dk(
        8.95,
        "[WARN] [1.0] [proximity_guard]: 🛡️ зона warn → stop (до корпуса 0.19 м) в точке (+0.49, +0.30); "
        "v=+0.10 м/с w=+0.00 рад/с",
    ),
]


def _voice(beep=True, say=True):
    rows = [dk(6.1, "[INFO] [1.0] [sound_node]: 🔔 Триггер: robot_backup_beep (request )")]
    if beep:
        rows.append(dk(9.2, "[INFO] [1.0] [sound_node]: 🔔 Триггер: robot_backup_beep_fast (request )"))
    if say:
        rows.append(tts(10.5, "Осторожно!"))
    return vd.parse_docker(rows)


def _run(probe=None, guard=GUARD, voice=None, lights=3):
    return vd.ZoneRun(probe or _wall_probe(), vd.parse_docker(guard), voice or _voice(), lights)


def test_zone_pass_stop_beep_phrase_lights_together():
    v = vd.v_zone(_run(), PX)
    assert v.status == vd.PASS, v.line()
    assert "до корпуса 0.19 м" in v.line() and "«Осторожно!»: 1" in v.line()


@pytest.mark.parametrize(
    "run,why",
    [
        (lambda: _run(voice=_voice(beep=False)), "писк robot_backup_beep_fast: 0"),
        (lambda: _run(voice=_voice(say=False)), "«Осторожно!»: 0"),
        (lambda: _run(lights=0), "маячок roadworks_beacon: 0"),
        (lambda: _run(probe=_wall_probe(drift=0.2)), "сдвиг за удержание стопа: 0.2"),
    ],
)
def test_zone_fail(run, why):
    v = vd.v_zone(run(), PX)
    assert v.status == vd.FAIL and why in v.line()


def test_zone_fail_without_stop_and_not_triggered_without_wall():
    assert vd.v_zone(_run(guard=GUARD[:1]), PX).status == vd.FAIL
    nowall = vd.parse_probe([pr(0, "WALL_NOT_FOUND", nearest="9.00", max="3.5")])
    assert vd.v_zone(_run(probe=nowall), PX).status == vd.NOT_TRIGGERED


MUTED = dk(
    8.96,
    "[INFO] [1.0] [proximity_guard]: 🛡️ HIL: зона stop, на настоящий робот не уходит (HIL_GUARD_SOUND=off): "
    "sound:'robot_backup_beep_fast', speech:'Осторожно!'",
)


def test_guard_sound_on_pass():
    v = vd.v_guard_sound(_run(), PX, "on", "HIL_GUARD_SOUND=on\nUSE_SIM_TIME=true")
    assert v.status == vd.PASS, v.line()


def test_guard_sound_off_pass_and_fail():
    quiet = vd.parse_docker([dk(1.0, "[INFO] [1.0] [dialog_node]: ничего")])
    ok = vd.v_guard_sound(_run(guard=GUARD + [MUTED], voice=quiet), PX, "off", "HIL_GUARD_SOUND=off\nUSE_SIM_TIME=true")
    assert ok.status == vd.PASS, ok.line()
    loud = vd.v_guard_sound(_run(guard=GUARD + [MUTED]), PX, "off", "HIL_GUARD_SOUND=off\nUSE_SIM_TIME=true")
    assert loud.status == vd.FAIL and "писков на Sound Pi: 2" in loud.line()


def test_guard_sound_env_mismatch_is_fail():
    v = vd.v_guard_sound(_run(), PX, "off", "HIL_GUARD_SOUND=on\nUSE_SIM_TIME=true")
    assert v.status == vd.FAIL and "не в режиме off" in v.line()


def _corner_guard(gap, y):
    """Строка стопа защиты: зазор и точка, как их печатает нода (proximity_guard_node._log_transition)."""
    return [
        dk(
            8.95,
            f"[WARN] [1.0] [proximity_guard]: 🛡️ зона warn → stop (до корпуса {gap:.2f} м) "
            f"в точке (+0.47, {y:+.2f}); v=+0.10 м/с w=+0.00 рад/с",
        )
    ]


def test_zone_stop_inside_the_corridor_below_stop_m_is_fail():
    """#4365: стоп должен случиться до 0.20 м. Зазор 0.16 м при точке внутри коридора хода
    (|y| ≤ 0.3525) значит, защита пропустила порог — это FAIL, а не PASS."""
    v = vd.v_zone(_run(guard=_corner_guard(0.16, 0.30)), PX)
    assert v.status == vd.FAIL and "0.16" in v.line()


def test_corner_stop_at_019_inside_the_corridor_passes():
    """Сценарий C (#4631): стенка перекрывает левый угол и лежит в коридоре хода. Стоп с зазором
    0.19 м — защита сработала не позже порога 0.20 (допуск на такт)."""
    v = vd.v_corner_stop(vd.parse_docker(_corner_guard(0.19, 0.30)))
    assert v.status == vd.PASS, v.line()


def test_corner_stop_at_016_inside_the_corridor_fails():
    """Сценарий C: зазор 0.16 м при точке внутри коридора (|y| ≤ 0.3525) — защита пропустила порог."""
    v = vd.v_corner_stop(vd.parse_docker(_corner_guard(0.16, 0.30)))
    assert v.status == vd.FAIL and "0.16" in v.line()


def test_corner_point_outside_the_corridor_is_not_a_stop():
    """Решение Шифу 10.11 (#4438): точка вне коридора хода с зазором 0.16–0.19 м — торец сбоку,
    не стоп. Стоп по такой точке — FAIL: защита затормозила там, где Шифу велел проехать."""
    v = vd.v_corner_stop(vd.parse_docker(_corner_guard(0.17, 0.40)))
    assert v.status == vd.FAIL and "вне коридора" in v.line()


def _door_probe(result="SUCCEEDED", xs=(0.5, 1.5, 2.5, 3.0)):
    """Подъезд по оси к проёму x=2 и исход цели."""
    rows = [pr(i, "TRACK", x=f"{x:.3f}", y="0.000", yaw="0.0", v="+0.150", w="+0.000") for i, x in enumerate(xs)]
    rows.append(pr(len(xs), "NAV_RESULT", status=result))
    return vd.parse_probe(rows)


def _door_guard(when, x):
    return dk(
        when,
        "[WARN] [1.0] [proximity_guard]: 🛡️ зона warn → stop (до корпуса 0.19 м) в точке (+0.50, +0.10); "
        "v=+0.15 м/с w=+0.00 рад/с",
    )


def test_doorway_pass_when_goal_succeeded_and_no_stop_in_the_opening():
    """Регресс проёма 0.90 м (#4365): цель доехала, а в проёме x∈[1.2, 2.8] защита не входила в stop."""
    guard = vd.parse_docker([_door_guard(0.2, 0.5)])
    v = vd.v_doorway(_door_probe(), guard, (1.2, 2.8))
    assert v.status == vd.PASS, v.line()


def test_doorway_fail_when_guard_stops_inside_the_opening():
    """Стоп, пока корпус в проёме 0.90 м, — FAIL: косяк вне коридора хода не должен тормозить."""
    guard = vd.parse_docker([_door_guard(1.0, 1.5)])
    v = vd.v_doorway(_door_probe(), guard, (1.2, 2.8))
    assert v.status == vd.FAIL and "stop" in v.line()


def test_doorway_fail_when_the_goal_did_not_succeed():
    v = vd.v_doorway(_door_probe(result="ABORTED"), vd.parse_docker([]), (1.2, 2.8))
    assert v.status == vd.FAIL and "ABORTED" in v.line()


# ── CLI и обвязка ───────────────────────────────────────────────────────────────────────────


def test_cli_writes_verdicts_txt_and_exit_code(tmp_path):
    d = tmp_path / "stop_idle"
    d.mkdir()
    (d / "voice.log").write_text("\n".join(STOP_RAW + [tts(0.05, "Я и так стою.")]), encoding="utf-8")
    assert vd.main(["stop_idle", str(d)]) == 0
    line = (tmp_path / "verdicts.txt").read_text(encoding="utf-8")
    assert line.startswith("VERDICT #3789 stop_idle PASS:")
    assert vd.main(["nav_aborted", str(tmp_path / "empty")]) == 2


def test_probe_compiles(tmp_path):
    py_compile.compile(str(CHECKS / "probe.py"), cfile=str(tmp_path / "p.pyc"), doraise=True)


def _bash_works() -> bool:
    if shutil.which("bash") is None:
        return False
    try:
        return subprocess.run(["bash", "-c", "true"], capture_output=True, timeout=20).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


@pytest.mark.skipif(not _bash_works(), reason="нет рабочего bash")
def test_runner_syntax():
    subprocess.run(["bash", "-n", "run_checks.sh"], check=True, cwd=CHECKS)


def test_runbook_linked_from_docs_readme():
    assert (ROOT / "docs" / "runbooks" / "hil-checks.md").is_file()
    assert "runbooks/hil-checks.md" in (ROOT / "docs" / "README.md").read_text(encoding="utf-8")
