"""test_music_stack_health.py — pytest unit tests for ``core.music_stack_health.MusicStackHealth``.

These tests target the class extracted in Phase 2 of the
``MusicManager`` decomposition (ADR-0134 §5, issue #3014). The
class has no state of its own — it reads and writes five attributes
on a duck-typed ``manager`` (the host ``MusicManager``), so each
test stands up a hand-rolled ``types.SimpleNamespace`` (or a
``MagicMock``) that exposes the same surface.

What this file covers
=====================

* **Health gate on a degraded manager** — confirm that
  ``_evaluate_music_stack_health`` returns a ``MusicStackStatus``
  with ``is_healthy=False`` when the sclang log is missing or
  contains fatal errors, that ``is_music_stack_healthy()`` mirrors
  that, and that ``music_stack_unavailable_error()`` returns a
  non-empty error payload.
* **Health gate on a healthy manager** — same trio of methods, but
  with a healthy log on disk; the manager must report healthy and
  the error payload must not be requested (we don't test it on the
  healthy path, because it's the "degraded" contract).
* **sclang bridge mock** — instead of touching disk at all, both
  ``load_sclang_health`` and ``load_confirmed_synths`` are
  monkey-patched in the module under test to deterministic
  callables. This is the fast path for CI.
* **``_log_synth_truth_discrepancy``** — feed a manager whose
  ``_synthdefs_added`` does not match ``_server_confirmed_synths``,
  assert that ``manager._log_warning`` is called exactly once with
  the issue-#2838 prefix and the correct counts.
* **``known_synth_names``** — empty input returns ``None``;
  non-empty input returns a ``frozenset``; the intersection with
  ``_server_confirmed_synths`` filters out synths sclang never
  acknowledged. The returned value is read-only (frozen).

All assertions use the public-ish surface of the class. No ROS,
no rclpy, no live sockets. Everything runs in <0.5s.
"""

from __future__ import annotations

import os
from types import SimpleNamespace
from typing import Any, Dict, FrozenSet, List, Optional
from unittest.mock import MagicMock

import pytest

from rob_box_mcp_tools.core import music_stack_health as msh
from rob_box_mcp_tools.core.music_stack_health import (
    MusicStackHealth,
    _WRAPPED_FALLBACK,
)
from rob_box_voice.core.music_stack_validation import MusicStackStatus


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


_HEALTHY_LOG = (
    "FoxDot OSCdef registered, ready.\n"
    "SynthDef in scsynth: lead\n"
    "SynthDef in scsynth: bass\n"
    "SynthDef in scsynth: pad\n"
    "SynthDef preload finished\n"
)

_DEGRADED_LOG_FATAL = (
    "ERROR: syntax error in startup file\n"
    "SynthDef preload finished\n"
)

_DEGRADED_LOG_MISSING_SYNTHS = (
    "FoxDot OSCdef registered, ready.\n"
    "SynthDef in scsynth: lead\n"
    "SynthDef preload finished\n"
)


def _make_manager(**overrides: Any) -> Any:
    """Build a duck-typed manager object that ``MusicStackHealth`` can read.

    Mirrors the attribute set documented on ``MusicStackHealth``: the five
    mutated fields plus a ``_log_warning`` sink. Tests that need a degraded
    manager should overwrite ``_music_stack_status`` (or call
    ``_evaluate_music_stack_health`` with a degraded log path).
    """
    base: Dict[str, Any] = {
        "_music_stack_status": MusicStackStatus(
            is_healthy=True,
            oscdef_registered=True,
            missing_synths=(),
            fatal_errors=(),
        ),
        "_server_confirmed_synths": None,
        "_synthdefs_added": set(),
        "_renardo_available": True,
        "_renardo_last_error": None,
        "_critical_synths": ("lead", "bass", "pad"),
        "_require_healthy": True,
        "_log_warning": MagicMock(),
    }
    base.update(overrides)
    return SimpleNamespace(**base)


def _stub_sclang(monkeypatch: pytest.MonkeyPatch, *, status: MusicStackStatus,
                 confirmed: Optional[FrozenSet[str]]) -> None:
    """Monkey-patch the sclang-bridge calls inside ``music_stack_health``.

    ``load_sclang_health`` and ``load_confirmed_synths`` are imported
    into ``music_stack_health`` at module load; to override them we have
    to rebind the names *in the module under test*, not in the source
    module. Both callables accept the same ``sclang_log_path`` argument
    that ``_evaluate_music_stack_health`` forwards.
    """
    monkeypatch.setattr(msh, "load_sclang_health", lambda *a, **kw: status)
    monkeypatch.setattr(msh, "load_confirmed_synths", lambda *a, **kw: confirmed)


# ---------------------------------------------------------------------------
# 1. Health gate on a degraded manager (sclang stubbed)
# ---------------------------------------------------------------------------


def test_evaluate_on_degraded_manager_returns_unhealthy_status(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Degraded sclang log → ``is_healthy=False`` and the gate fires."""
    status = MusicStackStatus(
        is_healthy=False,
        oscdef_registered=True,
        missing_synths=("bass", "pad"),
        fatal_errors=("ERROR: syntax error in startup file",),
    )
    _stub_sclang(monkeypatch, status=status, confirmed=None)
    manager = _make_manager(_renardo_available=True)
    health = MusicStackHealth(manager)

    returned = health._evaluate_music_stack_health(sclang_log_path="/tmp/sclang.log")

    # The exact status object the health gate produced is also stored on
    # the manager so downstream tools (``execute_code`` etc.) see the
    # same view.
    assert returned is status
    assert manager._music_stack_status is status
    assert returned.is_healthy is False
    # Degraded + ``_require_healthy`` (the default) → renardo unavailable.
    assert manager._renardo_available is False


def test_is_music_stack_healthy_returns_false_when_status_degraded(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``is_music_stack_healthy()`` is the boolean view of the latest status."""
    status = MusicStackStatus(
        is_healthy=False,
        oscdef_registered=False,
        missing_synths=("lead",),
        fatal_errors=("sclang boot failed",),
    )
    _stub_sclang(monkeypatch, status=status, confirmed=None)
    manager = _make_manager()
    health = MusicStackHealth(manager)

    health._evaluate_music_stack_health(sclang_log_path="/tmp/sclang.log")

    assert health.is_music_stack_healthy() is False


def test_music_stack_unavailable_error_returns_non_empty_payload_on_degraded(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``music_stack_unavailable_error()`` is the stable error reply."""
    status = MusicStackStatus(
        is_healthy=False,
        oscdef_registered=False,
        missing_synths=("lead", "bass"),
        fatal_errors=("ERROR: syntax error in startup file",),
    )
    _stub_sclang(monkeypatch, status=status, confirmed=None)
    manager = _make_manager()
    health = MusicStackHealth(manager)

    health._evaluate_music_stack_health(sclang_log_path="/tmp/sclang.log")
    payload = health.music_stack_unavailable_error()

    assert payload["success"] is False
    assert payload["error"], "error string must not be empty"
    assert "Музыка недоступна" in payload["error"]
    # Both the fatal error and the missing-synths hint must surface.
    assert "syntax error" in payload["error"]
    assert "missing SynthDefs" in payload["error"]
    assert "lead" in payload["error"] and "bass" in payload["error"]
    # The log path is mentioned so the operator can find the artifact.
    assert "/tmp/sclang.log" in payload["error"]


def test_evaluate_does_not_flip_renardo_when_require_healthy_false(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``_require_healthy=False`` means we tolerate the degraded state."""
    status = MusicStackStatus(
        is_healthy=False,
        oscdef_registered=True,
        missing_synths=("bass",),
        fatal_errors=(),
    )
    _stub_sclang(monkeypatch, status=status, confirmed=None)
    manager = _make_manager(_renardo_available=True, _require_healthy=False)
    health = MusicStackHealth(manager)

    health._evaluate_music_stack_health(sclang_log_path="/tmp/sclang.log")

    # The status is still recorded, but we DO NOT clobber renardo
    # availability — the operator opted into degraded-tolerant mode.
    assert manager._renardo_available is True
    assert manager._music_stack_status.is_healthy is False


# ---------------------------------------------------------------------------
# 2. Health gate on a healthy manager (sclang stubbed)
# ---------------------------------------------------------------------------


def test_evaluate_on_healthy_manager_returns_healthy_status(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Healthy sclang log → ``is_healthy=True`` and renardo stays available."""
    status = MusicStackStatus(
        is_healthy=True,
        oscdef_registered=True,
        missing_synths=(),
        fatal_errors=(),
    )
    confirmed = frozenset({"lead", "bass", "pad"})
    _stub_sclang(monkeypatch, status=status, confirmed=confirmed)
    manager = _make_manager()
    health = MusicStackHealth(manager)

    returned = health._evaluate_music_stack_health(sclang_log_path="/tmp/sclang.log")

    assert returned.is_healthy is True
    assert health.is_music_stack_healthy() is True
    # Healthy stack MUST NOT clobber renardo availability.
    assert manager._renardo_available is True
    # The confirmed-synths set is published on the manager for
    # ``known_synth_names`` to read in the next boot.
    assert manager._server_confirmed_synths is confirmed


# ---------------------------------------------------------------------------
# 3. Real sclang log on disk — both branches reachable end-to-end
# ---------------------------------------------------------------------------


def test_evaluate_with_real_log_file_healthy(tmp_path: Any) -> None:
    """A real ``/tmp/sclang.log``-shaped file is classified healthy."""
    log = tmp_path / "sclang.log"
    log.write_text(_HEALTHY_LOG, encoding="utf-8")

    manager = _make_manager()
    health = MusicStackHealth(manager)

    returned = health._evaluate_music_stack_health(sclang_log_path=str(log))

    assert returned.is_healthy is True
    assert health.is_music_stack_healthy() is True


def test_evaluate_with_real_log_file_degraded(tmp_path: Any) -> None:
    """A real sclang log containing a fatal error → degraded status."""
    log = tmp_path / "sclang.log"
    log.write_text(_DEGRADED_LOG_FATAL, encoding="utf-8")

    manager = _make_manager()
    health = MusicStackHealth(manager)

    returned = health._evaluate_music_stack_health(sclang_log_path=str(log))

    assert returned.is_healthy is False
    assert health.is_music_stack_healthy() is False
    assert manager._renardo_available is False
    # The fatal error from the log must surface in the unavailable
    # payload so the operator can see *why* music is dead.
    payload = health.music_stack_unavailable_error()
    assert "syntax error" in payload["error"]


def test_evaluate_with_missing_log_file_is_degraded(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Any,
) -> None:
    """Missing sclang log → ``is_healthy=False`` + a fatal error hint.

    The manager's env does not need a real ``SCLANG_LOG_PATH``; we
    pin ``sclang_log_path`` to a path that does not exist.
    """
    missing = tmp_path / "no-such.log"
    assert not missing.exists()

    # ``load_confirmed_synths`` opens the same file; with the file
    # missing it should still produce a deterministic empty result
    # rather than blowing up.
    manager = _make_manager()
    health = MusicStackHealth(manager)

    returned = health._evaluate_music_stack_health(sclang_log_path=str(missing))

    assert returned.is_healthy is False
    assert health.is_music_stack_healthy() is False
    assert manager._renardo_available is False
    # The unavailable error mentions the missing file.
    payload = health.music_stack_unavailable_error()
    assert "Музыка недоступна" in payload["error"]
    assert str(missing) in payload["error"]


def test_evaluate_log_path_override_takes_precedence_over_env(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Any,
) -> None:
    """Explicit ``sclang_log_path`` argument beats ``SCLANG_LOG_PATH`` env var."""
    env_log = tmp_path / "env.log"
    env_log.write_text(_DEGRADED_LOG_FATAL, encoding="utf-8")
    arg_log = tmp_path / "arg.log"
    arg_log.write_text(_HEALTHY_LOG, encoding="utf-8")

    monkeypatch.setenv("SCLANG_LOG_PATH", str(env_log))
    manager = _make_manager()
    health = MusicStackHealth(manager)

    returned = health._evaluate_music_stack_health(sclang_log_path=str(arg_log))

    # The arg won — the healthy log was classified healthy, not the
    # degraded one from the environment.
    assert returned.is_healthy is True
    assert health.is_music_stack_healthy() is True


# ---------------------------------------------------------------------------
# 4. sclang / SuperCollider bridge mock — both branches deterministic
# ---------------------------------------------------------------------------


def test_sclang_bridge_mock_healthy_branch(monkeypatch: pytest.MonkeyPatch) -> None:
    """When ``load_sclang_health`` returns healthy, the manager agrees."""
    healthy_status = MusicStackStatus(
        is_healthy=True,
        oscdef_registered=True,
        missing_synths=(),
        fatal_errors=(),
    )
    confirmed = frozenset({"lead", "bass"})
    _stub_sclang(monkeypatch, status=healthy_status, confirmed=confirmed)
    manager = _make_manager()
    health = MusicStackHealth(manager)

    out = health._evaluate_music_stack_health(sclang_log_path="/dev/null")

    assert out is healthy_status
    assert health.is_music_stack_healthy() is True
    assert manager._renardo_available is True
    assert manager._server_confirmed_synths is confirmed


def test_sclang_bridge_mock_unhealthy_branch(monkeypatch: pytest.MonkeyPatch) -> None:
    """When ``load_sclang_health`` returns degraded, the manager flips off."""
    unhealthy_status = MusicStackStatus(
        is_healthy=False,
        oscdef_registered=False,
        missing_synths=("lead",),
        fatal_errors=("sclang bridge died",),
    )
    _stub_sclang(monkeypatch, status=unhealthy_status, confirmed=None)
    manager = _make_manager(_renardo_available=True)
    health = MusicStackHealth(manager)

    out = health._evaluate_music_stack_health(sclang_log_path="/dev/null")

    assert out is unhealthy_status
    assert health.is_music_stack_healthy() is False
    assert manager._renardo_available is False
    payload = health.music_stack_unavailable_error()
    assert "sclang bridge died" in payload["error"]
    assert "missing SynthDefs" in payload["error"]


def test_sclang_bridge_mock_missing_log_path_via_env(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Any,
) -> None:
    """When ``sclang_log_path`` is ``None``, ``SCLANG_LOG_PATH`` is consulted.

    We don't run the real ``load_sclang_health`` (which talks to the
    filesystem); instead we use the same stub used elsewhere and just
    verify the *forwarding* contract: ``None`` is passed through and
    our stub still produces a deterministic answer.
    """
    captured: Dict[str, Any] = {}
    sentinel_status = MusicStackStatus(
        is_healthy=False,
        oscdef_registered=False,
        missing_synths=(),
        fatal_errors=("env-driven check",),
    )

    def _fake_load(path: Optional[str], **kwargs: Any) -> MusicStackStatus:
        captured["path"] = path
        return sentinel_status

    monkeypatch.setattr(msh, "load_sclang_health", _fake_load)
    monkeypatch.setattr(msh, "load_confirmed_synths", lambda *a, **kw: None)
    monkeypatch.setenv("SCLANG_LOG_PATH", str(tmp_path / "from-env.log"))

    manager = _make_manager()
    health = MusicStackHealth(manager)

    health._evaluate_music_stack_health()  # no path → loader receives ``None``

    # The argument we hand off is ``None`` — the loader is responsible
    # for resolving the env var, not us.
    assert captured["path"] is None
    assert health.is_music_stack_healthy() is False


# ---------------------------------------------------------------------------
# 5. ``_log_synth_truth_discrepancy`` — issue #2838
# ---------------------------------------------------------------------------


def test_log_synth_truth_discrepancy_without_confirmation(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """When sclang never reported back, log the "no confirmation" warning."""
    manager = _make_manager(
        _synthdefs_added={"lead", "bass", "pad"},
        _server_confirmed_synths=None,
    )
    health = MusicStackHealth(manager)

    with caplog.at_level("WARNING"):
        health._log_synth_truth_discrepancy()

    # Exactly one warning was emitted through the manager's log sink.
    manager._log_warning.assert_called_once()
    (msg, ), _ = manager._log_warning.call_args
    assert "[music #2838]" in msg
    assert "нет подтверждения" in msg
    assert "прелоада SynthDef" in msg


def test_log_synth_truth_discrepancy_with_discrepancy(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """When sent vs confirmed diverge, the unconfirmed set is named in the log.

    Setup: Python side "sent" ``{lead, bass, pad, dropme}`` via
    ``sdef.add()``; sclang's "SynthDef in scsynth" log only confirmed
    ``{lead, bass}``. The validator must reject ``dropme`` because
    the server never saw it.
    """
    manager = _make_manager(
        _synthdefs_added={"lead", "bass", "pad", "dropme"},
        _server_confirmed_synths=frozenset({"lead", "bass"}),
    )
    health = MusicStackHealth(manager)

    with caplog.at_level("WARNING"):
        health._log_synth_truth_discrepancy()

    manager._log_warning.assert_called_once()
    (msg, ), _ = manager._log_warning.call_args
    assert "[music #2838]" in msg
    assert "подтверждено в scsynth 2" in msg
    assert "отправлено renardo 4" in msg
    # ``dropme`` is the entry the server never saw — it must show up in
    # the "отклоняются" list.
    assert "dropme" in msg
    # ``pad`` is also unconfirmed.
    assert "pad" in msg


def test_log_synth_truth_discrepancy_includes_unwrapped_server_only_synths(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Synths sclang confirmed that have no Python wrapper are also listed.

    Scenario: server reports ``{lead, bass, masterlimiter}`` but
    Python only sent ``{lead, bass}``. ``masterlimiter`` exists on
    the server but ``MusicManager`` has no Python wrapper for it (it
    is a utility bus, not a tone for ``lead_synth``/``bass_synth``).
    The log line surfaces this as "на сервере без Python-обёртки".
    """
    manager = _make_manager(
        _synthdefs_added={"lead", "bass"},
        _server_confirmed_synths=frozenset({"lead", "bass", "masterlimiter"}),
    )
    health = MusicStackHealth(manager)

    with caplog.at_level("WARNING"):
        health._log_synth_truth_discrepancy()

    (msg, ), _ = manager._log_warning.call_args
    assert "masterlimiter" in msg
    assert "на сервере без Python-обёртки" in msg


def test_log_synth_truth_discrepancy_uses_manager_log_warning_not_stdlib(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """The log line goes through ``manager._log_warning`` — not Python logging.

    This keeps the existing ``_logger``/stderr fallback in
    ``MusicManager`` in charge. ``caplog`` should NOT capture the
    message because the manager's own sink swallows it.
    """
    manager = _make_manager(
        _synthdefs_added={"lead"},
        _server_confirmed_synths=None,
    )
    health = MusicStackHealth(manager)

    with caplog.at_level("WARNING"):
        health._log_synth_truth_discrepancy()

    manager._log_warning.assert_called_once()
    # The stdlib ``logging`` machinery was NOT used.
    assert all("нет подтверждения" not in rec.getMessage() for rec in caplog.records)


# ---------------------------------------------------------------------------
# 6. ``known_synth_names`` — read-only view of the truth table
# ---------------------------------------------------------------------------


def test_known_synth_names_returns_none_when_no_synths_added() -> None:
    """Empty ``_synthdefs_added`` → ``None`` (treat as "set unknown", not empty)."""
    manager = _make_manager(_synthdefs_added=set())
    health = MusicStackHealth(manager)

    assert health.known_synth_names() is None


def test_known_synth_names_returns_frozenset_when_no_server_confirmation() -> None:
    """Without server confirmation, fall back to ``sent ∪ CUSTOM_SC_ONLY``."""
    manager = _make_manager(
        _synthdefs_added={"lead", "bass"},
        _server_confirmed_synths=None,
    )
    health = MusicStackHealth(manager)

    out = health.known_synth_names()

    assert isinstance(out, frozenset)
    # Both the sent and the SC-only fallback are present.
    assert "lead" in out and "bass" in out
    assert "warmpad" in out  # from CUSTOM_SC_ONLY_SYNTH_NAMES
    assert out == frozenset({"lead", "bass"}) | _WRAPPED_FALLBACK


def test_known_synth_names_intersects_with_server_confirmation() -> None:
    """When sclang confirmed a subset, only that subset is allowed."""
    manager = _make_manager(
        _synthdefs_added={"lead", "bass", "dropme"},
        _server_confirmed_synths=frozenset({"lead", "bass"}),
    )
    health = MusicStackHealth(manager)

    out = health.known_synth_names()

    assert isinstance(out, frozenset)
    # ``dropme`` was sent but the server never saw it — must be dropped.
    assert "dropme" not in out
    assert "lead" in out and "bass" in out


def test_known_synth_names_returned_value_is_read_only() -> None:
    """The returned container is a ``frozenset`` — callers can't mutate it.

    This is the contract: callers (the SynthDef validator) must not be
    able to alter the truth set; if they try, the mutation must raise
    ``AttributeError``.
    """
    manager = _make_manager(
        _synthdefs_added={"lead"},
        _server_confirmed_synths=None,
    )
    health = MusicStackHealth(manager)

    out = health.known_synth_names()

    assert isinstance(out, frozenset)
    # The exposed set is frozen — ``.add``/``.discard`` raise.
    with pytest.raises(AttributeError):
        out.add("mutated")  # type: ignore[attr-defined]
    with pytest.raises(AttributeError):
        out.discard("lead")  # type: ignore[attr-defined]


def test_known_synth_names_does_not_touch_critical_synths() -> None:
    """``_critical_synths`` is a SEPARATE concept (booted-time gate) and
    must not leak into the known-synth set."""
    manager = _make_manager(
        _synthdefs_added={"lead"},
        _server_confirmed_synths=frozenset({"lead"}),
        _critical_synths=("lead", "bass", "pad", "somespecial"),
    )
    health = MusicStackHealth(manager)

    out = health.known_synth_names()

    # ``somespecial`` is a *critical* synth, not a *known* one — the
    # known-synth set is built from ``_synthdefs_added`` (sent) ∩
    # ``_server_confirmed_synths`` (server-confirmed), nothing else.
    assert "somespecial" not in out


# ---------------------------------------------------------------------------
# 7. ``music_stack_unavailable_error`` — payload shape and env-var override
# ---------------------------------------------------------------------------


def test_music_stack_unavailable_error_uses_sclang_log_path_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """When ``SCLANG_LOG_PATH`` is set, it shows up in the error payload."""
    manager = _make_manager(
        _music_stack_status=MusicStackStatus(
            is_healthy=False,
            oscdef_registered=False,
            missing_synths=(),
            fatal_errors=("sclang syntax error",),
        ),
    )
    health = MusicStackHealth(manager)
    monkeypatch.setenv("SCLANG_LOG_PATH", "/var/log/sclang/custom.log")

    payload = health.music_stack_unavailable_error()

    assert payload["success"] is False
    assert "/var/log/sclang/custom.log" in payload["error"]


def test_music_stack_unavailable_error_falls_back_to_default_log_path() -> None:
    """No ``SCLANG_LOG_PATH`` env → ``/tmp/sclang.log`` in the message."""
    os.environ.pop("SCLANG_LOG_PATH", None)
    manager = _make_manager(
        _music_stack_status=MusicStackStatus(
            is_healthy=False,
            oscdef_registered=False,
            missing_synths=(),
            fatal_errors=("sclang syntax error",),
        ),
    )
    health = MusicStackHealth(manager)

    payload = health.music_stack_unavailable_error()

    assert "/tmp/sclang.log" in payload["error"]


def test_music_stack_unavailable_error_truncates_fatal_errors_to_three() -> None:
    """The error payload caps the fatal-error list at three for readability."""
    manager = _make_manager(
        _music_stack_status=MusicStackStatus(
            is_healthy=False,
            oscdef_registered=False,
            missing_synths=(),
            fatal_errors=("e1", "e2", "e3", "e4", "e5"),
        ),
    )
    health = MusicStackHealth(manager)

    payload = health.music_stack_unavailable_error()

    # First three are present…
    assert "e1" in payload["error"] and "e2" in payload["error"]
    assert "e3" in payload["error"]
    # …the rest are dropped.
    assert "e4" not in payload["error"]
    assert "e5" not in payload["error"]


# ---------------------------------------------------------------------------
# 8. Misc invariants — class shape and __slots__
# ---------------------------------------------------------------------------


def test_music_stack_health_uses_slots_for_just_manager() -> None:
    """``__slots__`` is exactly ``("_manager",)`` — no per-instance dict."""
    manager = _make_manager()
    health = MusicStackHealth(manager)

    # ``__dict__`` is missing on a slotted class.
    assert not hasattr(health, "__dict__")
    # Setting an unknown attribute must raise ``AttributeError``.
    with pytest.raises(AttributeError):
        health.bogus_attribute = 1  # type: ignore[attr-defined]


def test_music_stack_health_holds_reference_to_manager() -> None:
    """The constructor stashes the manager in the single private slot."""
    manager = _make_manager()
    health = MusicStackHealth(manager)

    assert health._manager is manager
