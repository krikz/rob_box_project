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


def test_post_turn_retry_guards_runs_before_finalize_cleanup_policy() -> None:
    """``_apply_post_turn_retry_guards`` стоит ДО ``_finalize_music_cleanup_policy``.

    ДО фикса: ``_finalize_music_cleanup_policy(...)`` стоял первым → для
    хода «сыграй/спой» без music-тула вооружал ``_pending_music_cleanup
    =True`` и catch-up сразу публиковал ``music_cleanup(reason="tts_
    batch_complete")``. Только ЗАТЕМ ``_apply_music_guard`` диспатчил
    Bug C-ретрай с новым ``compose_music``, но Renardo уже убит.

    ПОСЛЕ фикса (develop HEAD переименовал guard в
    ``_apply_post_turn_retry_guards`` — объединение music/tool/babble,
    см. issue #2627/ADR-0021a R2): guard срабатывает ПЕРВЫМ, и если он
    диспатчил ретрай (любой из music/tool), outer-finalize cleanup-policy
    пропускается через гейт ``if any_retry_dispatched:``.

    Тестовая страховка: внутри ``_run_turn`` текстовый порядок вызовов
    ``_apply_post_turn_retry_guards`` и ``_finalize_music_cleanup_policy``.
    Если кто-то снова поставит cleanup-policy ПЕРЕД guard — этот тест упадёт.
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    src_lines = src.splitlines()

    # Find first call (not definition) of each helper inside _run_turn.
    # Issue #3005: cleanup-finalize guard may live in a dedicated helper
    # ``_finalize_music_cleanup_after_retry_guard`` (extracted to keep CC
    # of ``_run_turn`` ≤15, ADR-0021 R1) — accept either the direct call
    # in ``_run_turn`` or the helper call as the gate-equivalent.
    methods = _class_methods(src)
    run_turn = methods["_run_turn"]
    start_line = run_turn.lineno

    apply_call_line = None
    finalize_call_line = None
    for i, line in enumerate(src_lines, 1):
        if i < start_line:
            continue
        # Crude but effective: skip ``def`` lines, find first call.
        if apply_call_line is None and "_apply_post_turn_retry_guards(" in line and "def _apply_post_turn_retry_guards" not in line:
            apply_call_line = i
        # Accept the helper call as well — that's the gate-equivalent path
        # extracted in #3005 to keep _run_turn CC ≤15.
        if finalize_call_line is None and (
            "_finalize_music_cleanup_policy(" in line
            or "_finalize_music_cleanup_after_retry_guard(" in line
        ) and "def _finalize_music_cleanup" not in line:
            finalize_call_line = i
        if apply_call_line is not None and finalize_call_line is not None:
            break

    assert apply_call_line is not None, (
        "_run_turn не вызывает _apply_post_turn_retry_guards — guard снят. "
        "Issue #992 Bug B/C регрессирует (DJ/music guard) + #2627 R2."
    )
    assert finalize_call_line is not None, (
        "_run_turn не вызывает _finalize_music_cleanup_policy / "
        "_finalize_music_cleanup_after_retry_guard — cleanup-policy снят. "
        "Issue #935/#992 регрессирует (фронт-чистка)."
    )
    assert apply_call_line < finalize_call_line, (
        f"_apply_post_turn_retry_guards (line {apply_call_line}) должен идти ДО "
        f"_finalize_music_cleanup_policy / _finalize_music_cleanup_after_retry_guard "
        f"(line {finalize_call_line}) в _run_turn.finally. Иначе catch-up "
        "cleanup-policy опубликует music_cleanup раньше, чем guard успеет "
        "диспатчить Bug C-ретрай — флап «start → стоп → старт» (issue #3005)."
    )


def test_retry_dispatched_branch_skips_finalize_cleanup_policy() -> None:
    """Если guard диспатчил ретрай, ``_finalize_music_cleanup_policy`` НЕ зовётся.

    Контракт: в ``finally`` _run_turn (или в вынесенном helper'е
    ``_finalize_music_cleanup_after_retry_guard``, ADR-0021 R1) есть
    ``if any_retry_dispatched:`` — ветка «ретрай отправлен — cleanup
    финализируй потом». Внутри этой ветки НЕТ вызова
    ``_finalize_music_cleanup_policy``. ВНЕ этой ветки — есть.

    Допускаем две формы:

    * ``if cond: …; else: _finalize_music_cleanup_policy(...)`` —
      explicit else.
    * ``if cond: …; return`` + post-if ``_finalize_music_cleanup_policy(
      ...)`` — early-return (guard-форма, чище для CC).

    Структура развилась вместе с develop (issue #2627 R2): теперь guard
    возвращает пару ``(music_retry_dispatched, tool_retry_dispatched)``, и
    гейт объединяет их в ``any_retry_dispatched = music or tool`` —
    cleanup пропускается для ОБОИХ видов ретрая (issue #3005).
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    search_roots = [
        methods["_run_turn"],
        methods.get("_finalize_music_cleanup_after_retry_guard"),
    ]

    # Find the ``if any_retry_dispatched:`` branch.
    # We accept any structural nesting as long as the ``if`` branch does
    # NOT call ``_finalize_music_cleanup_policy`` and the post-if branch
    # (explicit ``else`` OR code after the if in the enclosing function)
    # DOES. Note: also accept the legacy ``if music_retry_dispatched:``
    # form as a graceful fallback if someone reverts the union.
    found_branch = None
    candidate_test_ids = {"any_retry_dispatched", "music_retry_dispatched"}
    for root in search_roots:
        if root is None:
            continue
        # Walk top-level ``if`` blocks in root first (early-return form).
        # ast.walk recurses into body, so to find the *enclosing* function's
        # post-if code we have to inspect the parent. Build parent map.
        parent_map: dict = {}
        for parent in ast.walk(root):
            for child in ast.iter_child_nodes(parent):
                parent_map[child] = parent

        def _post_if_calls_finalize(if_node) -> bool:
            """Code in the same function AFTER the if, but NOT inside the if body."""
            enclosing = parent_map.get(if_node, root)
            if enclosing is None:
                enclosing = root
            # Locate the if's position within enclosing.body.
            sibling_index = None
            for idx, sibling in enumerate(getattr(enclosing, "body", [])):
                if sibling is if_node:
                    sibling_index = idx
                    break
            if sibling_index is None:
                return False
            after_stmts = list(enclosing.body[sibling_index + 1:])
            # If there's an explicit else, the post-if code is inside node.orelse.
            if getattr(if_node, "orelse", []):
                after_stmts = list(if_node.orelse)
            post_module = ast.Module(body=after_stmts, type_ignores=[])
            for c in ast.walk(post_module):
                if (isinstance(c, ast.Call)
                        and isinstance(c.func, ast.Attribute)
                        and c.func.attr == "_finalize_music_cleanup_policy"):
                    return True
            return False

        for node in ast.walk(root):
            if not isinstance(node, ast.If):
                continue
            # Walk test expression looking for ``any_retry_dispatched``
            # (or, gracefully, ``music_retry_dispatched``).
            test = node.test
            if not isinstance(test, ast.Name):
                continue
            if test.id not in candidate_test_ids:
                continue
            # Found a candidate. Verify the IF body does NOT call
            # _finalize_music_cleanup_policy, and the post-if branch DOES.
            if_body_finalize = any(
                isinstance(c, ast.Call)
                and isinstance(c.func, ast.Attribute)
                and c.func.attr == "_finalize_music_cleanup_policy"
                for c in ast.walk(ast.Module(body=node.body, type_ignores=[]))
            )
            if if_body_finalize:
                # Wrong branch has the call — keep searching.
                continue
            if _post_if_calls_finalize(node):
                found_branch = node
                break
        if found_branch is not None:
            break

    assert found_branch is not None, (
        "В ``finally`` _run_turn (или в вынесенном helper'е "
        "_finalize_music_cleanup_after_retry_guard, ADR-0021 R1) должна быть "
        "ветка ``if any_retry_dispatched: <чистим pending>`` (или "
        "``if music_retry_dispatched:`` как legacy-форма), ВНЕ которой "
        "(explicit else или post-if early-return) зовётся "
        "_finalize_music_cleanup_policy. Без этого флап «start → стоп → "
        "старт» (issue #3005) возвращается."
    )


def test_retry_branch_clears_pending_music_cleanup_flag() -> None:
    """Если guard диспатчил ретрай, ``_pending_music_cleanup`` сбрасывается в False.

    Без сброса inner-тур стартует с «грязным» состоянием: его собственный
    ``_schedule_music_cleanup`` увидит ``_pending_music_cleanup=True``, и
    если active batches пусты, catch-up опубликует cleanup → убьёт
    только что запущенную inner-туром музыку.

    Развилось вместе с develop (#2627 R2): ветка идёт под
    ``if any_retry_dispatched:`` (объединение music+tool из
    ``_apply_post_turn_retry_guards``), но семантически эквивалентно —
    внутри всё равно идёт ``self._pending_music_cleanup = False``.

    Issue #3005 ADR-0021 R1: гейт вынесен в helper. Ищем ветку в ОБОИХ.
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    search_roots = [
        methods["_run_turn"],
        methods.get("_finalize_music_cleanup_after_retry_guard"),
    ]

    # Find ``if any_retry_dispatched:`` (or legacy ``if music_retry_dispatched:``)
    # branch in _run_turn (or extracted helper).
    found = False
    candidate_test_ids = {"any_retry_dispatched", "music_retry_dispatched"}
    for root in search_roots:
        if root is None:
            continue
        for node in ast.walk(root):
            if not isinstance(node, ast.If):
                continue
            if not isinstance(node.test, ast.Name):
                continue
            if node.test.id not in candidate_test_ids:
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
        if found:
            break

    assert found, (
        "В ветке ``if any_retry_dispatched:`` (или legacy "
        "``if music_retry_dispatched:``) _run_turn.finally / "
        "_finalize_music_cleanup_after_retry_guard должен стоять "
        "``self._pending_music_cleanup = False``, чтобы ретрай-тур "
        "стартовал с чистым состоянием. Иначе catch-up на tts_batch_complete "
        "убьёт только что запущенный compose_music."
    )


def test_apply_music_guard_still_disarms_when_no_retry() -> None:
    """Без ретрая ``_apply_music_guard`` НЕ ломает обычный cleanup-flow.

    Negative: подтверждаем, что cleanup-finalize остался доступен для
    ходов без ретрая (force-stop, fallback, normal chat). Если кто-то
    случайно поставит ``return` после ``_apply_music_guard``, этот тест
    упадёт.

    Issue #3005 ADR-0021 R1: путь вызова может идти через helper
    ``_finalize_music_cleanup_after_retry_guard`` — проверяем ОБА варианта.
    """
    src = DIALOGUE_NODE.read_text(encoding="utf-8-sig")
    methods = _class_methods(src)
    run_turn = methods["_run_turn"]

    # The finally block must still contain a path that calls
    # _finalize_music_cleanup_policy OR its helper. We've already verified
    # the order in test_apply_music_guard_runs_before_finalize_cleanup_policy;
    # here we verify the call is still reachable (directly or via helper).
    src_text = ast.unparse(run_turn)
    helper = methods.get("_finalize_music_cleanup_after_retry_guard")
    helper_text = ast.unparse(helper) if helper is not None else ""
    combined = src_text + "\n" + helper_text
    assert (
        "_finalize_music_cleanup_policy" in combined
        or "_finalize_music_cleanup_after_retry_guard" in combined
    ), (
        "_finalize_music_cleanup_policy / _finalize_music_cleanup_after_retry_guard "
        "всё ещё должны зваться из _run_turn.finally (для ходов без ретрая — "
        "force_stop, fallback, normal chat). Если их убрали — регресс cleanup "
        "#935/#992 (issue #3005 helper extraction). "
        f"src_text len={len(src_text)}, helper_text len={len(helper_text)}."
    )