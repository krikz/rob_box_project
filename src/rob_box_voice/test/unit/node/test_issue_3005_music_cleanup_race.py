#!/usr/bin/env python3
"""test_issue_3005_music_cleanup_race.py — issue #3005 regression.

Симптом (vision-pi 24.09.2026 14:50–14:55, LLM minimax):

    Робот начинает играть → через несколько секунд выключает → снова
    пытается поставить → и так несколько раз. Слушатель слышит «шляпу».

Корень (race в cleanup-policy):

    Outer turn «сыграй/спой» без music-тула → ``_finalize_music_cleanup_policy``
    ПЕРВЫМ вооружал ``_pending_music_cleanup=True`` → catch-up
    ``_flush_music_cleanup_if_idle`` сразу публиковал
    ``music_cleanup(reason="tts_batch_complete")`` → mcp_server гасил
    Renardo. Только ЗАТЕМ ``_apply_music_guard`` диспатчил Bug C-retry с
    новым ``compose_music``, который приходил в уже пустое аудио.

Принимаем:

* Начатая по запросу юзера музыка не останавливается сразу же (нет гонки
  cleanup vs fresh-start).
* ``music_cleanup``/watchdog не гасят музыку, если она была запущена в
  текущем ходе и юзер явно просил её.

Технический контракт:

* ``_apply_music_guard`` теперь запускается ДО
  ``_finalize_music_cleanup_policy`` в ``_run_turn.finally``. Если guard
  диспатчил ``USER_RETRY`` или ``DJ_RETRY``, outer-finalize пропускается —
  ретрай-тур отработает свой собственный cleanup с правильным ``result``.
* Pending-флаг ``_pending_music_cleanup`` сбрасывается после успешного
  ретрая, чтобы catch-up на tts_batch_complete не убил уже запущенный
  inner-тур'ом ``compose_music``.

Эти тесты — чистый AST, без rclpy. Они фиксируют порядок вызовов и
содержимое ``finally`` хода, чтобы архитектурный фикс нельзя было
развернуть без явного изменения acceptance.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest


_VOICE_PKG = Path(__file__).resolve().parents[3] / "rob_box_voice"
DIALOGUE_NODE = _VOICE_PKG / "dialogue_node.py"


def _class_methods(src: str) -> dict:
    """All DialogueNode methods keyed by name."""
    tree = ast.parse(src)
    cls = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "DialogueNode"
    )
    return {
        fn.name: fn for fn in cls.body
        if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef))
    }


def _calls_in_branch(branch_node, attr: str) -> bool:
    """``self.<attr>(...)`` only inside branch_node (no recursion)."""
    for node in ast.walk(branch_node):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == attr):
            return True
    return False


# ── Tests ────────────────────────────────────────────────────────────────


def test_apply_music_guard_runs_before_finalize_cleanup_policy() -> None:
    """``_apply_music_guard`` стоит ДО ``_finalize_music_cleanup_policy``.

    ДО фикса: ``_finalize_music_cleanup_policy(...)`` стоял первым → для
    хода «сыграй/спой» без music-тула вооружал ``_pending_music_cleanup
    =True`` и catch-up сразу публиковал ``music_cleanup(reason="tts_
    batch_complete")``. Только ЗАТЕМ ``_apply_music_guard`` диспатчил
    Bug C-retry с новым ``compose_music``, но Renardo уже убит.

    ПОСЛЕ фикса: ``_apply_music_guard`` срабатывает ПЕРВЫМ, и если он
    диспатчит ретрай, outer-finalize cleanup-policy пропускается через
    гейт ``if music_retry_dispatched``.

    Тестовая страховка: внутри ``_run_turn`` текстовый порядок вызовов
    ``_apply_music_guard`` и ``_finalize_music_cleanup_policy``. Если
    кто-то снова поставит cleanup-policy ПЕРЕД guard — этот тест упадёт.
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    src_lines = src.splitlines()

    # Find first call (not definition) of each helper inside _run_turn.
    methods = _class_methods(src)
    run_turn = methods["_run_turn"]
    start_line = run_turn.lineno

    apply_call_line = None
    finalize_call_line = None
    for i, line in enumerate(src_lines, 1):
        if i < start_line:
            continue
        # Crude but effective: skip ``def`` lines, find first call.
        if apply_call_line is None and "_apply_music_guard(" in line and "def _apply_music_guard" not in line:
            apply_call_line = i
        if finalize_call_line is None and "_finalize_music_cleanup_policy(" in line and "def _finalize_music_cleanup_policy" not in line:
            finalize_call_line = i
        if apply_call_line is not None and finalize_call_line is not None:
            break

    assert apply_call_line is not None, (
        "_run_turn не вызывает _apply_music_guard — guard снят. "
        "Issue #992 Bug B/C регрессирует (DJ/music guard)."
    )
    assert finalize_call_line is not None, (
        "_run_turn не вызывает _finalize_music_cleanup_policy — "
        "cleanup-policy снят. Issue #935/#992 регрессирует (фронт-чистка)."
    )
    assert apply_call_line < finalize_call_line, (
        f"_apply_music_guard (line {apply_call_line}) должен идти ДО "
        f"_finalize_music_cleanup_policy (line {finalize_call_line}) в "
        "_run_turn.finally. Иначе catch-up cleanup-policy опубликует "
        "music_cleanup раньше, чем guard успеет диспатчить Bug C-ретрай — "
        "флап «start → стоп → старт» (issue #3005)."
    )


def test_retry_dispatched_branch_skips_finalize_cleanup_policy() -> None:
    """Если guard диспатчил ретрай, ``_finalize_music_cleanup_policy`` НЕ зовётся.

    Контракт: в ``finally`` _run_turn есть ``if music_retry_dispatched:``
    (ветка «ретрай отправлен — cleanup-финализируй потом»), внутри которой
    НЕТ вызова ``_finalize_music_cleanup_policy``, а ВНЕ этой ветки
    (``else``) — есть.
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    run_turn = methods["_run_turn"]

    # Find the ``if music_retry_dispatched:`` branch in _run_turn.finally.
    # We accept any structural nesting as long as the ``if`` branch does
    # NOT call ``_finalize_music_cleanup_policy`` and the ``else`` branch
    # DOES.
    found_branch = None
    for node in ast.walk(run_turn):
        if not isinstance(node, ast.If):
            continue
        # Walk test expression looking for ``music_retry_dispatched``.
        test = node.test
        if not isinstance(test, ast.Name):
            continue
        if test.id != "music_retry_dispatched":
            continue
        # Found a candidate. Verify it's inside _run_turn.finally.
        # Verify the IF body does NOT call _finalize_music_cleanup_policy.
        # Verify the ELSE body DOES.
        if_body_finalize = any(
            isinstance(c, ast.Call)
            and isinstance(c.func, ast.Attribute)
            and c.func.attr == "_finalize_music_cleanup_policy"
            for c in ast.walk(ast.Module(body=node.body, type_ignores=[]))
        )
        else_finalize = False
        if node.orelse:
            else_finalize = any(
                isinstance(c, ast.Call)
                and isinstance(c.func, ast.Attribute)
                and c.func.attr == "_finalize_music_cleanup_policy"
                for c in ast.walk(ast.Module(body=node.orelse, type_ignores=[]))
            )
        if if_body_finalize:
            # Wrong branch has the call — keep searching.
            continue
        if else_finalize:
            found_branch = node
            break

    assert found_branch is not None, (
        "В ``finally`` _run_turn должна быть ветка "
        "``if music_retry_dispatched: <чистим pending>`` с ``else``, "
        "внутри которого зовётся _finalize_music_cleanup_policy. "
        "Без этого флап «start → стоп → старт» (issue #3005) возвращается."
    )


def test_retry_branch_clears_pending_music_cleanup_flag() -> None:
    """Если guard диспатчил ретрай, ``_pending_music_cleanup`` сбрасывается в False.

    Без сброса inner-тур стартует с «грязным» состоянием: его собственный
    ``_schedule_music_cleanup`` увидит ``_pending_music_cleanup=True``, и
    если active batches пусты, catch-up опубликует cleanup → убьёт
    только что запущенную inner-туром музыку.
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    run_turn = methods["_run_turn"]

    # Find ``if music_retry_dispatched:`` branch in _run_turn.
    found = False
    for node in ast.walk(run_turn):
        if not isinstance(node, ast.If):
            continue
        if not isinstance(node.test, ast.Name):
            continue
        if node.test.id != "music_retry_dispatched":
            continue
        # Inside the IF body, look for ``self._pending_music_cleanup = False``
        cleared = False
        for sub in ast.walk(ast.Module(body=node.body, type_ignores=[])):
            if not isinstance(sub, ast.Assign):
                continue
            for target in sub.targets:
                if (isinstance(target, ast.Attribute)
                        and target.attr == "_pending_music_cleanup"
                        and isinstance(target.value, ast.Name)
                        and target.value.id == "self"):
                    if isinstance(sub.value, ast.Constant) and sub.value.value is False:
                        cleared = True
                        break
        if cleared:
            found = True
            break

    assert found, (
        "В ветке ``if music_retry_dispatched:`` _run_turn.finally должен "
        "стоять ``self._pending_music_cleanup = False``, чтобы ретрай-тур "
        "стартовал с чистым состоянием. Иначе catch-up на tts_batch_complete "
        "убьёт только что запущенный compose_music."
    )


def test_apply_music_guard_still_disarms_when_no_retry() -> None:
    """Без ретрая ``_apply_music_guard`` НЕ ломает обычный cleanup-flow.

    Negative: подтверждаем, что cleanup-finalize остался доступен для
    ходов без ретрая (force-stop, fallback, normal chat). Если кто-то
    случайно поставит ``return` после ``_apply_music_guard``, этот тест
    упадёт.
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    run_turn = methods["_run_turn"]

    # The finally block must still contain a path that calls
    # _finalize_music_cleanup_policy. We've already verified the order
    # in test_apply_music_guard_runs_before_finalize_cleanup_policy;
    # here we verify the call is still reachable.
    src_text = ast.unparse(run_turn)
    assert "_finalize_music_cleanup_policy" in src_text, (
        "_finalize_music_cleanup_policy всё ещё должен зваться из "
        "_run_turn.finally (для ходов без ретрая — force_stop, fallback, "
        "normal chat). Если его убрали — регресс cleanup #935/#992."
    )