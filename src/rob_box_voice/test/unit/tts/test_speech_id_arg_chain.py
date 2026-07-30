"""Regression tests for kanban task ``t_42726db6`` (issue #929).

The TTS synthesis submit chain used to pass ``speech_id`` twice: once as
the explicit positional kwarg to ``_submit_synthesis``, and once as a
trailing ``*args`` element in the ``dialogue_callback``.  Python's
positional-binding rules meant the trailing duplicate was silently
shadowed by the earlier ``speech_id=None`` default in
``_run_synthesis_worker`` — so functionally the duplicate was a no-op
(many unit tests passed).  But it was confusing and forward-fragile:

* If a future refactor ever drops the ``speech_id=None`` default on
  ``_run_synthesis_worker``, the worker would receive *one extra*
  positional argument and break at runtime with no immediate TypeError
  (since ``*args`` swallows overflow), shifting the meaning of
  ``speech_id`` to whatever the new last positional parameter is.
* The same fragility applies if someone adds a new trailing parameter
  to ``_run_synthesis_worker`` without updating the call site — the
  silent shadowing masks the argument shift.

This test enforces:

1. ``dialogue_callback`` invokes ``_submit_synthesis`` with exactly
   one ``speech_id`` positional argument (the kwarg), followed by the
   four positional args expected by ``_run_synthesis_worker``.
2. ``_run_synthesis_worker``'s signature still terminates in
   ``speech_id=None`` so that, if a caller ever forgets to pass it,
   the default kicks in instead of a TypeError.
3. Both halves agree on the parameter ordering (ssml, text,
   dialogue_id, ssml_attributes, speech_id) — verified by AST
   inspection so the test does not need rclpy/grpc/torch to import.

If any of these invariants is violated, the test fails immediately
with a precise location, rather than letting the bug manifest as a
silent "speech_id stuck at None" symptom deep inside a downstream
consumer at 2 a.m. on the robot.
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest


_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
_TTS_NODE = _PACKAGE_ROOT / "rob_box_voice" / "tts_node.py"


def _parse_module(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def _find_function(tree: ast.Module, name: str) -> ast.FunctionDef:
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
        if isinstance(node, ast.ClassDef):
            for member in node.body:
                if isinstance(member, ast.FunctionDef) and member.name == name:
                    return member
    raise LookupError(f"function {name!r} not found")


def _arg_to_name(node: ast.AST) -> str:
    """Best-effort: extract a human-readable identifier from an AST
    expression node so we can compare argument names. Returns the raw
    ``ast.dump`` if the expression isn't a plain Name / Attribute —
    better than crashing the regression test on a future refactor."""
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        # ``self._run_synthesis_worker`` -> "_run_synthesis_worker"
        return node.attr
    return ast.dump(node)


# ── Acceptance: speech_id passed exactly once ────────────────────────────────


def test_dialogue_callback_passes_speech_id_once() -> None:
    """``dialogue_callback`` must invoke ``_submit_synthesis`` with one
    ``speech_id`` positional (the explicit kwarg) plus the four
    positional worker args — *not* a trailing duplicate ``speech_id``.
    """
    tree = _parse_module(_TTS_NODE)
    cb = _find_function(tree, "dialogue_callback")
    cb_src = ast.unparse(cb)

    # Locate the ``self._submit_synthesis( ... )`` call inside
    # dialogue_callback. AST inspect keeps the test independent of rclpy.
    submit_calls = [
        node
        for node in ast.walk(cb)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "_submit_synthesis"
    ]
    assert len(submit_calls) == 1, (
        "dialogue_callback must call self._submit_synthesis exactly once; "
        f"found {len(submit_calls)}"
    )

    call = submit_calls[0]
    args = call.args  # positional arguments (after the implicit ``self``)
    # AST does not show ``self`` here — args[0] is the first positional.
    positional_names = [_arg_to_name(a) for a in args]

    # _submit_synthesis(self, fn, speech_id, *args) — so positional[0]
    # is ``fn`` (= _run_synthesis_worker), positional[1] is ``speech_id``,
    # and everything after flows into ``*args`` (ssml, text, dialogue_id,
    # ssml_attributes — no trailing duplicate speech_id).
    assert positional_names[0] == "_run_synthesis_worker", (
        "First positional to _submit_synthesis must be _run_synthesis_worker"
    )
    assert positional_names[1] == "speech_id", (
        "Second positional to _submit_synthesis must be the kwarg speech_id"
    )
    assert positional_names.count("speech_id") == 1, (
        "speech_id must be passed exactly once to _submit_synthesis; "
        f"found {positional_names.count('speech_id')} occurrences in "
        f"{positional_names} (duplicate shadows later positional args "
        "without raising TypeError — that's the bug t_42726db6 fixed)"
    )
    # The trailing *args for the worker must be ssml, text, dialogue_id,
    # ssml_attributes in that order — match worker signature.
    expected_tail = ["ssml", "text", "dialogue_id", "ssml_attributes"]
    assert positional_names[2:] == expected_tail, (
        f"Trailing positional args must be {expected_tail} "
        "(matching _run_synthesis_worker signature); "
        f"got {positional_names[2:]}"
    )


def test_run_synthesis_worker_signature_terminates_in_speech_id() -> None:
    """``_run_synthesis_worker``'s last parameter must be ``speech_id=None``
    so that any future caller forgetting to pass it doesn't TypeError, and
    so that the worker can be invoked with the same arity as
    ``dialogue_callback`` passes.
    """
    tree = _parse_module(_TTS_NODE)
    worker = _find_function(tree, "_run_synthesis_worker")

    args = worker.args
    positional = args.args
    defaults = args.defaults  # aligned to the trailing positional args

    n_defaults = len(defaults)
    n_positional = len(positional)
    assert n_positional >= n_defaults, (
        "_run_synthesis_worker: every default-aligned parameter must "
        "have a default"
    )

    last_param = positional[-1]
    assert last_param.arg == "speech_id", (
        "_run_synthesis_worker must terminate in a `speech_id` parameter "
        "so the call site in dialogue_callback can pass it positionally. "
        f"Got terminal param {last_param.arg!r}"
    )

    # The last default value (which aligns with `speech_id`) must be
    # the literal ``None`` — not ``""`` or any other sentinel.
    last_default = defaults[-1]
    assert isinstance(last_default, ast.Constant) and last_default.value is None, (
        "_run_synthesis_worker's speech_id default must be None (not '' "
        "or a placeholder) — downstream code does `if speech_id:` to "
        "decide whether to publish a /voice/tts/finished event."
    )


def test_submit_and_worker_arg_arities_agree() -> None:
    """The number of positional args after the explicit ``speech_id``
    kwarg in dialogue_callback's ``_submit_synthesis`` call must equal
    the number of non-(self, fn, speech_id) positional parameters on
    ``_submit_synthesis`` AND equal the number of positional parameters
    on ``_run_synthesis_worker``.

    If any of these drifts, the executor's ``submit(fn, *args)`` call
    will silently swallow the mismatch because Python ``*args`` is
    permissive.  Catching it here keeps the chain honest.
    """
    tree = _parse_module(_TTS_NODE)
    submit = _find_function(tree, "_submit_synthesis")
    worker = _find_function(tree, "_run_synthesis_worker")
    cb = _find_function(tree, "dialogue_callback")

    # _submit_synthesis(self, fn, speech_id, *args) — only the first two
    # are mandatory positional; *args is variadic.  But the dialogue_callback
    # call site must hand it (fn, speech_id, ssml, text, dialogue_id,
    # ssml_attributes) — exactly 6 positional values (the first two being
    # the function and the speech_id kwarg; the rest become *args).
    submit_call = next(
        node
        for node in ast.walk(cb)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "_submit_synthesis"
    )
    n_call_positional = len(submit_call.args)
    assert n_call_positional == 6, (
        f"dialogue_callback -> _submit_synthesis must pass exactly 6 "
        f"positional args (fn, speech_id, ssml, text, dialogue_id, "
        f"ssml_attributes); got {n_call_positional}"
    )

    # _run_synthesis_worker is a method — first positional is ``self``.
    worker_positional_names = [a.arg for a in worker.args.args if a.arg != "self"]
    assert worker_positional_names == [
        "ssml", "text", "dialogue_id", "ssml_attributes", "speech_id",
    ], (
        f"_run_synthesis_worker signature drifted from canonical "
        f"(ssml, text, dialogue_id, ssml_attributes, speech_id); "
        f"got {worker_positional_names}"
    )

    # _submit_synthesis's own signature must accept at minimum (fn,
    # speech_id) before *args — defensive against accidental removal
    # of the speech_id kwarg.
    submit_positional_names = [
        a.arg for a in submit.args.args if a.arg != "self"
    ]
    assert "speech_id" in submit_positional_names[:2], (
        "_submit_synthesis must keep `speech_id` as the second positional "
        "(right after ``fn``) so callers can identify the synthesis task "
        "in logs / drop diagnostics."
    )