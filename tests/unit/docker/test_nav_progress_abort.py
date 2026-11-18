"""Abort контроллера не крутит ту же цель до таймаута клиента.

HIL Depot 17.11 (#4694, depot4694_20261117T1235Z): 8× «Failed to make progress»
+ ``[follow_path] Aborting handle`` каждые ~30 с (movement_time_allowance),
recovery в логе 0, ``bt_navigator`` не писал Goal failed, route.py снял цель
по TIMEOUT 200 с.

Humble ``SimpleActionServer::work()`` после возврата ``computeControl`` сам
берёт pending FollowPath и снова вызывает callback, не отдавая результат
клиенту BT. ``FollowPath`` BT при смене ``{path}`` шлёт новую цель, пока
старая ещё EXECUTING — она ложится в pending. ``terminate_current()`` абортит
только текущую; ``work()`` принимает pending, ``progress_checker_->reset()``
даёт ещё 30 с. Дерево recovery не видит. Патч: на abort ``terminate_all()``,
pending тоже абортится, callback возвращается, BT получает FAILURE.
"""

from __future__ import annotations

import subprocess
import urllib.request
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
NAV2_DIR = REPO / "docker" / "main" / "nav2"
PATCH = NAV2_DIR / "controller_progress_abort.patch"
DOCKERFILE = NAV2_DIR / "Dockerfile"
PINNED_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
UPSTREAM_URL = (
    f"https://raw.githubusercontent.com/ros-navigation/navigation2/"
    f"{PINNED_COMMIT}/nav2_controller/src/controller_server.cpp"
)

ABORTED, CANCELED, EXECUTING = "ABORTED", "CANCELED", "EXECUTING"


class _Handle:
    def __init__(self, name: str, status: str = EXECUTING):
        self.name = name
        self.status = status

    def is_active(self) -> bool:
        return self.status == EXECUTING

    def abort(self) -> None:
        self.status = ABORTED

    def canceled(self) -> None:
        self.status = CANCELED


class _Server:
    """Humble SimpleActionServer: work() после callback берёт pending."""

    def __init__(self):
        self.current = None
        self.pending = None
        self.preempt = False

    def terminate(self, which: str) -> None:
        handle = self.current if which == "current" else self.pending
        if handle is not None and handle.is_active():
            if False:
                handle.canceled()
            else:
                handle.abort()
        if which == "current":
            self.current = None
        else:
            self.pending = None

    def terminate_current(self) -> None:
        self.terminate("current")

    def terminate_all(self) -> None:
        self.terminate("current")
        self.terminate("pending")
        self.preempt = False

    def is_preempt_requested(self) -> bool:
        return self.preempt

    def accept_pending(self):
        if (
            self.current is not None
            and self.current.is_active()
            and self.current is not self.pending
        ):
            self.current.abort()
        self.current = self.pending
        self.pending = None
        self.preempt = False
        return self.current


def _work(server: _Server, on_abort: str, goals: list[str]) -> list[str]:
    """Один поток work(): callback абортит так, как скажет on_abort."""
    seen: list[str] = []

    def callback() -> None:
        seen.append(server.current.name)
        if on_abort == "current":
            server.terminate_current()
        else:
            server.terminate_all()

    while server.current is not None and server.current.is_active():
        callback()
        if server.pending is not None and server.pending.is_active():
            server.accept_pending()
        else:
            break
    return seen


def test_upstream_pending_goal_restarts_without_returning_failure():
    """Без патча abort текущей цели не останавливает pending: ещё один цикл 30 с."""
    server = _Server()
    first = _Handle("follow-1")
    pending = _Handle("follow-2")
    server.current = first
    server.pending = pending
    server.preempt = True

    seen = _work(server, "current", [])

    assert seen == ["follow-1", "follow-2"]
    assert first.status == ABORTED
    assert pending.status == ABORTED
    assert server.current is None


def test_terminate_all_on_abort_does_not_restart_pending():
    """Патч: pending абортится вместе с текущей, callback один, BT видит FAILURE."""
    server = _Server()
    first = _Handle("follow-1")
    pending = _Handle("follow-2")
    server.current = first
    server.pending = pending
    server.preempt = True

    seen = _work(server, "all", [])

    assert seen == ["follow-1"]
    assert first.status == ABORTED
    assert pending.status == ABORTED
    assert server.pending is None
    assert server.preempt is False


def test_patch_uses_terminate_all_and_does_not_touch_success_path():
    text = PATCH.read_text(encoding="utf-8")
    added = "\n".join(
        line[1:]
        for line in text.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )
    removed = "\n".join(
        line[1:]
        for line in text.splitlines()
        if line.startswith("-") and not line.startswith("---")
    )
    assert "terminate_all()" in added
    assert "terminate_all(result)" in added
    assert removed.count("terminate_current()") == 1
    assert "terminate_current(result)" in removed
    assert "succeeded_current" not in text
    assert "xy_goal_tolerance" not in text
    assert "PathAlign" not in text
    assert "movement_time_allowance" not in text


def test_dockerfile_applies_progress_abort_patch_on_pinned_controller():
    df = DOCKERFILE.read_text(encoding="utf-8")
    assert "controller_progress_abort.patch" in df
    assert PINNED_COMMIT in df
    assert df.index("controller_goal_tf_fail.patch") < df.index(
        "controller_progress_abort.patch"
    )


def test_patch_applies_after_goal_tf_patch(tmp_path: Path):
    """Оба патча ложатся на pinned controller_server.cpp (нужна сеть)."""
    src_root = tmp_path / "navigation2"
    target = src_root / "nav2_controller" / "src" / "controller_server.cpp"
    target.parent.mkdir(parents=True)
    try:
        with urllib.request.urlopen(UPSTREAM_URL, timeout=30) as resp:
            body = resp.read()
    except Exception as exc:  # noqa: BLE001 — honest fail if upstream unreachable
        pytest.fail(f"could not fetch pinned Nav2 source: {exc}")
    target.write_bytes(body)
    before = target.read_text(encoding="utf-8")
    assert before.count("action_server_->terminate_current();") == 5
    assert "catch (nav2_core::PlannerException & e)" in before

    tf_patch = (NAV2_DIR / "controller_goal_tf_fail.patch").read_text(encoding="utf-8")
    for patch_text in (tf_patch, PATCH.read_text(encoding="utf-8")):
        result = subprocess.run(
            ["patch", "-p1", "--forward", "--batch"],
            cwd=src_root,
            input=patch_text,
            text=True,
            capture_output=True,
            check=False,
        )
        assert result.returncode == 0, result.stdout + result.stderr

    after = target.read_text(encoding="utf-8")
    assert "action_server_->terminate_all();" in after
    assert "action_server_->terminate_all(result);" in after
    assert (
        after.count("action_server_->terminate_current();")
        == before.count("action_server_->terminate_current();") - 1
    )
    assert "if (!nav_2d_utils::transformPose(" in after
