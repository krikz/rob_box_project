"""The single public entry point for running a harness.

Mirrors the canonical ``run_harness(name, input, config)`` signature
specified in the task body. The function:

  1. Loads (or accepts) a :class:`HarnessConfig`.
  2. Resolves the harness name through a :class:`HarnessRegistry`.
  3. Builds the harness via :class:`HarnessFactory`.
  4. Drives ``init / run / teardown`` under ``async with``.
  5. Returns a :class:`HarnessRunResult`.

The function is intentionally synchronous-looking (no ``async``)
and returns a coroutine: callers can either ``await`` it or
schedule it via ``asyncio.run``.

The factory caches instances so a long-running process that calls
``run_harness("echo", ...)`` repeatedly reuses the same harness
object — the framework's ``teardown`` is therefore a no-op unless
the caller explicitly resets the cache.
"""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

from rob_box_harness.config import HarnessConfig, load_config
from rob_box_harness.harness import Harness, HarnessRunResult
from rob_box_harness.registry import (
    HarnessFactory,
    HarnessRegistry,
    register_builtin_harnesses,
)


# Default registry: built-ins are registered the first time
# ``run_harness`` is called. We keep the registry at module scope
# so caller code can import it and inspect / extend it.
_default_registry: HarnessRegistry | None = None


def get_default_registry() -> HarnessRegistry:
    """Return the module-level default registry (built-ins registered)."""
    global _default_registry  # noqa: PLW0603 — module-level singleton
    if _default_registry is None:
        _default_registry = register_builtin_harnesses()
    return _default_registry


def reset_default_registry() -> None:
    """Reset the default registry and the factory cache. Test-only helper."""
    global _default_registry  # noqa: PLW0603 — module-level singleton
    _default_registry = None
    HarnessFactory.reset_cache()


async def run_harness(
    name: str,
    input: Any = None,
    config: HarnessConfig | None = None,
    *,
    config_path: str | Path | None = None,
    registry: HarnessRegistry | None = None,
) -> HarnessRunResult:
    """Build, run, and tear down a harness by name.

    Args:
        name: Harness name (e.g. ``"echo"``, ``"upper"``).
        input: Anything the harness's ``step`` method accepts.
        config: A pre-built :class:`HarnessConfig`. If ``None``,
            :func:`load_config` is called with ``config_path`` (or
            the env-default fallback path).
        config_path: Path to a YAML config file. Ignored if
            ``config`` is provided.
        registry: A pre-built :class:`HarnessRegistry`. The default
            registry (registered with built-ins) is used otherwise.

    Returns:
        A :class:`HarnessRunResult` containing the final state and
        the harness's ``step`` output.

    Raises:
        :class:`HarnessNotFoundError` if ``name`` is unregistered.
        :class:`ConfigError` if ``config_path`` is malformed.
        Anything raised by the harness's ``step`` method.
    """
    if config is None:
        config = load_config(config_path)

    reg = registry if registry is not None else get_default_registry()
    harness = HarnessFactory.create(name, config, reg)

    # ``async with`` guarantees teardown on exception. The
    # framework's aclose is idempotent so accidental cache reuse
    # across calls does not leak.
    async with harness as h:
        return await h.run(input)


def run_harness_sync(
    name: str,
    input: Any = None,
    config: HarnessConfig | None = None,
    **kwargs: Any,
) -> HarnessRunResult:
    """Synchronous wrapper around :func:`run_harness`.

    Convenient for scripts / REPL:

        >>> from rob_box_harness import run_harness_sync
        >>> result = run_harness_sync("echo", "hello")
        >>> result.output
        'echo: hello'
    """
    return asyncio.run(run_harness(name, input, config, **kwargs))


__all__ = [
    "run_harness",
    "run_harness_sync",
    "get_default_registry",
    "reset_default_registry",
]
