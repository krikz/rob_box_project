"""Lifecycle hooks — the slimmest possible extension surface for a harness.

Hooks are optional callables that the harness invokes at well-defined
points during its lifecycle. They are NOT lifecycle methods themselves:
the harness remains in charge of ordering, teardown idempotency, and
exception translation. Hooks are just observers / augmenters.

Semantics (ADR-0001 §2.2):

  on_start          — before the first turn; warm caches, log markers.
  on_turn_begin     — before each user input / message handling.
  on_tool_call      — before ToolProvider.execute(); can raise to veto.
  on_tool_result    — after ToolProvider.execute(); can sanitise.
  on_response_chunk — on each streaming chunk (cumulative counts, etc).
  on_error          — translated from any exception inside the lifecycle.
  on_stop           — before teardown; flush, snapshot, final log.

Hook errors are wrapped in :class:`HookError` so the framework can
finish cleanup instead of leaking resources.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable, Mapping

from rob_box_harness.errors import HookError

logger = logging.getLogger(__name__)


# Type aliases — concrete types are intentionally Any so the hooks
# stay decoupled from the value objects defined in rob_box_llm /
# rob_box_harness.tools. The harness passes dictionaries around the
# hook layer; rich typing happens in the inner methods.

Hook = Callable[..., Awaitable[None] | None]
"""A hook callable. Synchronous and async coroutine functions are both
allowed; the framework awaits the result if it's awaitable."""


@dataclass
class LifecycleHooks:
    """User-supplied observer hooks for a harness.

    Each field is optional (``None`` = not registered). Hooks are
    invoked in the order listed in :meth:`iter_hooks`; the harness
    decides which subset to call for a given event.

    A hook that raises is wrapped in :class:`HookError` so the
    framework can continue. To opt-out of this safety net (e.g. inside
    ``on_tool_call`` where vetoing is the whole point), raise the
    framework-specific exception directly.
    """

    on_start: Hook | None = None
    on_turn_begin: Hook | None = None
    on_tool_call: Hook | None = None
    on_tool_result: Hook | None = None
    on_response_chunk: Hook | None = None
    on_error: Hook | None = None
    on_stop: Hook | None = None

    # ------------------------------------------------------------------
    # Helper
    # ------------------------------------------------------------------

    def iter_hooks(self) -> Mapping[str, Hook]:
        """Return a name → callable mapping of the registered hooks.

        Stable across Python's dict ordering guarantees: ``on_start``
        first, ``on_stop`` last. Useful for assertions in tests.
        """
        return {
            name: hook
            for name, hook in (
                ("on_start", self.on_start),
                ("on_turn_begin", self.on_turn_begin),
                ("on_tool_call", self.on_tool_call),
                ("on_tool_result", self.on_tool_result),
                ("on_response_chunk", self.on_response_chunk),
                ("on_error", self.on_error),
                ("on_stop", self.on_stop),
            )
            if hook is not None
        }

    async def invoke(
        self,
        name: str,
        harness: str | None,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        """Call the named hook if registered, wrapping errors.

        Hooks are called with the supplied positional/keyword args
        exactly as passed. If the hook raises, the exception is wrapped
        in :class:`HookError` and re-raised — the harness decides
        whether to swallow it.
        """
        hook = getattr(self, name, None)
        if hook is None:
            return
        try:
            result = hook(*args, **kwargs)
            if hasattr(result, "__await__"):
                await result
        except HookError:
            raise
        except Exception as exc:  # noqa: BLE001 — we wrap a generic hook error
            logger.warning(
                "hook %s raised on harness %s: %s",
                name,
                harness or "<unnamed>",
                exc,
            )
            raise HookError(
                f"hook {name!r} raised {type(exc).__name__}: {exc}",
                harness=harness,
                hook=name,
            ) from exc


__all__ = ["Hook", "LifecycleHooks"]
