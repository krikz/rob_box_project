"""Regression test: aiohttp.TCPSite must be called with reuse_port=True.

Контекст:
- Issue #1650 (test-round-232): на Vision Pi aiohttp.TCPSite без
  reuse_port=True падал на bind 0.0.0.0:8765 с OSError [Errno 98]
  EADDRINUSE, потому что предыдущий процесс удерживал сокет в TIME_WAIT
  (host-network + не-чистый redeploy).
- Фикс: в _runner() внутри QuestNode._start_aiohttp() ставим reuse_port=True
  в конструктор TCPSite.

Этот тест парсит исходник quest_node.py и проверяет, что вызов TCPSite
содержит reuse_port=True. Дешёвый guard против случайного удаления
параметра в будущих рефакторингах.
"""

from __future__ import annotations

import re
from pathlib import Path


def _quest_node_source() -> str:
    """Return quest_node.py source as str."""
    return (
        Path(__file__).resolve().parents[2]
        / "rob_box_quest"
        / "quest_node.py"
    ).read_text(encoding="utf-8")


def test_tcpsite_call_uses_reuse_port() -> None:
    """TCPSite(...) должен передавать reuse_port=True для устойчивости bind."""
    src = _quest_node_source()

    # Грубый, но устойчивый к переносам строк и вложенным скобкам поиск:
    # находим любой вызов TCPSite и проверяем, что в окне 400 символов
    # после него встречается reuse_port=True.
    pattern = re.compile(r"TCPSite\s*\(", re.DOTALL)
    matches = list(pattern.finditer(src))
    assert matches, "TCPSite(...) не найден в quest_node.py"

    reuse_port_kwarg = re.compile(r"reuse_port\s*=\s*True")
    for m in matches:
        window = src[m.end() : m.end() + 400]
        if reuse_port_kwarg.search(window):
            return
    raise AssertionError(
        "TCPSite(...) в _start_aiohttp должен передавать reuse_port=True. "
        "Без него при не-чистом redeploy на host-network рискуем повторить "
        "OSError [Errno 98] EADDRINUSE из test-round-232 (issue #1650)."
    )


def test_reuse_port_reason_in_comment() -> None:
    """Документация причины (issue #1650) должна быть рядом с reuse_port=True."""
    src = _quest_node_source()

    # Ищем именно kwarg `reuse_port=True` в вызове TCPSite (а не в комментарии).
    # Паттерн: `reuse_port=True` после открывающей скобки TCPSite.
    call_pattern = re.compile(r"TCPSite\s*\(", re.DOTALL)
    kwarg_pattern = re.compile(r"reuse_port\s*=\s*True")
    tcpsite_calls = list(call_pattern.finditer(src))
    assert tcpsite_calls, "TCPSite(...) не найден в quest_node.py"

    matched_call = None
    for c in tcpsite_calls:
        window = src[c.end() : c.end() + 800]
        if kwarg_pattern.search(window):
            matched_call = c
            break
    assert matched_call, "reuse_port=True не найден внутри TCPSite(...)"

    # Контекст: 600 символов ДО найденного TCPSite-вызова — там должен быть
    # блок комментария с обоснованием (issue #1650 / EADDRINUSE).
    nearby = src[max(0, matched_call.start() - 800) : matched_call.end() + 200]
    assert "1650" in nearby or "EADDRINUSE" in nearby, (
        "Рядом с reuse_port=True должна быть ссылка на issue #1650 "
        "или строка 'EADDRINUSE' для будущих мейнтейнеров."
    )
