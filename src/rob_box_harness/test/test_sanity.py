"""Sanity-check: pytest.ini + conftest.py из wt/t_b13cf256 корректно работают.

Этот файл нужен только для верификации самого pytest.ini в worktree
t_b13cf256, где ещё нет src/rob_box_harness/rob_box_harness/*.py.
После merge harness framework (wt/t_35cfe938) этот файл станет
избыточным и будет удалён.
"""

import pytest


def test_always_passes() -> None:
    """Plain unit-тест, без маркеров. Должен попасть в ``-m 'not network'``."""
    assert 1 + 1 == 2


@pytest.mark.skip(reason="network marker not yet configured in this worktree; test file is temporary")
def test_network_marker_recognised() -> None:
    """Должен быть skip, потому что conftest не видит MINIMAX_API_KEY."""
    pytest.fail("network test was not skipped — conftest.py is broken")
