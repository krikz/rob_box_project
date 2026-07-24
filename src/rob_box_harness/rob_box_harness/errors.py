"""Typed errors raised across the Harness Framework.

All errors here are part of the public API. Callers (AgentSession,
real harnesses, the runner entry point) MUST catch these categories
instead of bare ``Exception`` so the lifecycle can map them to
``on_error`` hooks, retry policies, and structured logs.

The hierarchy is intentionally narrow (ADR-0001 §2.4.8: no premature
categorisation): ``HarnessError`` is the umbrella, and we add a small
handful of subclasses worth branching on. Anything else surfaces as
bare ``HarnessError`` with a descriptive message.
"""

from __future__ import annotations


class HarnessError(Exception):
    """Base class for every error raised inside the Harness Framework.

    ``harness`` is the canonical harness name (e.g. ``"dialog"``,
    ``"persistent"``, ``"telegram"``) when the error originates from a
    concrete harness. Subclasses exist for the small handful of
    categories worth branching on; anything else surfaces as bare
    ``HarnessError``.
    """

    def __init__(self, message: str, *, harness: str | None = None) -> None:
        super().__init__(message)
        self.harness = harness


class ConfigError(HarnessError):
    """Configuration is malformed, missing, or semantically inconsistent.

    Examples: unknown harness ``kind``, missing required section, ENV
    placeholder ``${MINIMAX_API_KEY}`` left unresolved, incompatible
    combination (``kind=persistent`` together with an LLM-only model).
    """

    def __init__(
        self,
        message: str,
        *,
        harness: str | None = None,
        section: str | None = None,
    ) -> None:
        super().__init__(message, harness=harness)
        self.section = section


class HarnessNotFoundError(HarnessError):
    """The requested harness name is not registered.

    Raised by :func:`rob_box_harness.runner.run_harness` and
    :class:`rob_box_harness.registry.HarnessRegistry.resolve` when the
    caller asks for a harness that hasn't been registered. Tests use
    this to detect typos in service factories.
    """


class HarnessStateError(HarnessError):
    """The harness is in the wrong state for the requested operation.

    Examples: calling ``init()`` twice without ``teardown()`` in between,
    calling ``run()`` before ``init()``, calling ``teardown()`` while
    ``run()`` is still iterating. These are programming errors, not
    transient failures — no retry.
    """


class ProviderNotFoundError(HarnessError):
    """A port (LLM / Tools / Memory / Effects / Transport) was not wired.

    Raised by the lifecycle when the ``HarnessConfig`` references a
    provider name that no registry has under it. Distinct from
    :class:`HarnessNotFoundError` so callers can tell the two apart
    in dashboards.
    """

    def __init__(
        self,
        message: str,
        *,
        harness: str | None = None,
        port: str | None = None,
    ) -> None:
        super().__init__(message, harness=harness)
        self.port = port


class HookError(HarnessError):
    """A lifecycle hook raised an unexpected exception.

    Hooks are user-supplied callables; if one misbehaves we wrap the
    exception so the harness can finish cleaning up instead of leaking
    resources. The original exception is chained (``raise ... from``)
    for full traceback fidelity.
    """

    def __init__(
        self,
        message: str,
        *,
        harness: str | None = None,
        hook: str | None = None,
    ) -> None:
        super().__init__(message, harness=harness)
        self.hook = hook


__all__ = [
    "HarnessError",
    "ConfigError",
    "HarnessNotFoundError",
    "HarnessStateError",
    "ProviderNotFoundError",
    "HookError",
]
