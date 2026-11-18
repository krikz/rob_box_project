"""Офлайн-счётчик пропусков такта Nav2 (#4682 / #4694).

Сырой факт depot4694_20261117T1235Z (Loki nav2, comment 17.11):
  Control loop missed … 6915; Behavior Tree tick rate exceeded 1904; Planner loop missed 77.
Период controller/BT не меняем; порог вердикта не вводим — только расклад уже существующих строк.
"""

from __future__ import annotations

import importlib.util
import py_compile
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MOD_PATH = ROOT / "scripts" / "hil" / "nav2_loop_skips.py"

_spec = importlib.util.spec_from_file_location("nav2_loop_skips", MOD_PATH)
mod = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
sys.modules["nav2_loop_skips"] = mod
_spec.loader.exec_module(mod)

# Точные подстроки из Loki #4694 (depot4694_20261117T1235Z) и docker logs HIL.
CTRL = (
    "[WARN] [1794919000.1] [controller_server]: "
    "Control loop missed its desired rate of 20.0000Hz. The loop actually took 0.072 seconds"
)
BT = (
    "[WARN] [1794919000.2] [BehaviorTreeEngine]: "
    "Behavior Tree tick rate 100.00 was exceeded!"
)
PLAN = (
    "[WARN] [1794919000.3] [planner_server]: "
    "Planner loop missed its desired rate of 20.0000 Hz. The loop actually took 0.081 seconds"
)
# Старый текст из docs/fixes (без «its») — тоже rate miss.
CTRL_LEGACY = "[WARN] [controller_server]: Control loop missed desired rate 20.0000Hz"
OTHER = "[INFO] [bt_navigator]: Begin navigating from current location (0.00, 0.00) to (-6.00, 4.00)"


def test_module_compiles():
    py_compile.compile(str(MOD_PATH), doraise=True)


def test_empty_log_is_zero():
    assert mod.count_loop_skips("") == {
        "controller": 0,
        "bt": 0,
        "planner": 0,
        "rate_missed": 0,
    }


def test_counts_three_classes_separately():
    text = "\n".join([CTRL, BT, PLAN, OTHER, CTRL, BT])
    got = mod.count_loop_skips(text)
    assert got == {"controller": 2, "bt": 2, "planner": 1, "rate_missed": 3}


def test_depot4694_raw_counts_from_issue_comment():
    """Повтор счётчиков Loki #4694: 6915 / 1904 / 77 — без порога, только count."""
    text = "\n".join([CTRL] * 6915 + [BT] * 1904 + [PLAN] * 77)
    got = mod.count_loop_skips(text)
    assert got["controller"] == 6915
    assert got["bt"] == 1904
    assert got["planner"] == 77
    assert got["rate_missed"] == 6915 + 77


def test_format_loop_skips_one_line():
    line = mod.format_loop_skips(
        {"controller": 6915, "bt": 1904, "planner": 77, "rate_missed": 6992}
    )
    assert line == "LOOP_SKIPS controller=6915 bt=1904 planner=77 rate_missed=6992"


def test_rate_missed_compat_matches_old_shell_grep_plus_legacy():
    text = "\n".join([CTRL, PLAN, CTRL_LEGACY, BT, OTHER])
    # shell grep -cE 'missed its desired rate' → CTRL+PLAN = 2; legacy без its — наш бонус.
    assert mod.count_rate_missed_compat(text) == 3
    assert (
        mod.count_loop_skips(text)["rate_missed"] == 3
    )  # Control+Planner, legacy Control


def test_bt_line_without_exceeded_is_not_counted():
    noise = "[INFO] Behavior Tree tick rate configured to 100.00 Hz"
    assert mod.count_loop_skips(noise)["bt"] == 0


def test_cli_prints_loop_skips(tmp_path, capsys):
    log = tmp_path / "nav2.log"
    log.write_text("\n".join([CTRL, BT, PLAN]) + "\n", encoding="utf-8")
    assert mod.main([str(log)]) == 0
    out = capsys.readouterr().out.strip()
    assert out == "LOOP_SKIPS controller=1 bt=1 planner=1 rate_missed=2"


def test_run_scene_uses_counter_not_only_shell_grep():
    """speed HIL после прогона пишет расклад LOOP_SKIPS, period не трогает."""
    script = (ROOT / "scripts" / "hil" / "speed" / "run_scene.sh").read_text(
        encoding="utf-8"
    )
    assert "nav2_loop_skips.py" in script
    assert "nav2_loop_skips.txt" in script
    assert "controller_frequency" not in script
    assert "bt_loop_duration" not in script
