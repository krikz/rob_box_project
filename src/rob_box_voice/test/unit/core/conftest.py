"""Local conftest for unit/core tests.

No global hooks needed.  Async test methods in
test_minimax_music_client.py wrap their coroutines with the local
``_run()`` helper so the suite doesn't depend on a global
``asyncio_mode = auto`` setting.  Sync tests work without any
modification.
"""

# Intentionally empty — kept as a marker so future test authors know
# this directory is opt-in for unit/core helpers.
