"""conftest.py — isolated httpx for unit/stt tests.

The CI ``unit/node/conftest.py`` (``tests/unit/node/conftest.py``)
monkey-patches ``sys.modules['httpx']`` to a ``types.SimpleNamespace``
that has only ``Timeout`` — every other httpx symbol is missing. That
shim is fine for tests that import nothing from httpx, but our
``MiniMaxSTTProvider.transcribe`` uses ``except httpx.TimeoutException``
and ``except httpx.HTTPError``. Under the shim those ``except`` clauses
crash with::

    AttributeError: 'types.SimpleNamespace' object has no attribute 'TimeoutException'

Our :mod:`test_minimax_provider` test already had to dodge the same
problem at the test-stub layer (``_FakeReadTimeout`` /
`` ``_FakeConnectError``). To keep the test suite and the provider
running in a single ``pytest test/`` invocation, we restore the real
``httpx`` module before importing the production code. The shim is
``setdefault``-installed, so any module that legitimately needs the
fake httpx will get it back on demand.

Activated automatically by pytest for tests under ``test/unit/stt/``.
"""

from __future__ import annotations

import importlib
import sys


def _restore_real_httpx() -> None:
    """Replace ``sys.modules['httpx']`` with the real package if available.

    No-op when httpx is genuinely missing (no network on CI base image).
    """
    try:
        import httpx as _real_httpx  # noqa: F401
    except ImportError:
        return
    # If the current entry is a SimpleNamespace shim, drop it so the real
    # module takes over on next ``import httpx``. The shim is installed
    # via ``setdefault`` in ``unit/node/conftest.py``; once we pop it the
    # subsequent import re-binds ``sys.modules['httpx']`` to the genuine
    # package.
    current = sys.modules.get("httpx")
    is_shim = current is not None and not hasattr(current, "__file__")
    if is_shim:
        sys.modules.pop("httpx", None)
        importlib.import_module("httpx")


_restore_real_httpx()