"""scsynth must create group 1 itself, or Renardo is silent with healthy logs.

Live 31.08. Renardo hangs a group off group 1 for every player
(``ServerManager.get_bundle``: ``/g_new [id, 1, 1]``) and puts every note
inside it. Group 1 is created by the client once, at init.

A freshly started scsynth has only the RootNode. If it restarts after Renardo
has initialised — container restart, crash, or container ordering during a
deploy — group 1 is gone and nobody recreates it:

    FAILURE IN SERVER /g_new Group 1 not found
    FAILURE IN SERVER /s_new Group 6469 not found

The player group fails, then every single note is rejected. From outside
everything looks correct: containers healthy, sclang reports its preload, the
MCP tool returns success, the generated composition sits in the log — and the
robot is silent. It is only visible by asking the server itself
(``/status`` showed numSynths=1 while a full arrangement was "playing").

So the server creates the group at startup, where it cannot depend on which
client connected first or how many times the client has restarted.
"""
from __future__ import annotations

import re
from pathlib import Path


def _repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "src").is_dir() and (parent / "docker").is_dir():
            return parent
    raise RuntimeError("repo root not found")


REPO_ROOT = _repo_root(Path(__file__).resolve())
START_SCRIPT = (
    REPO_ROOT / "docker" / "vision" / "scripts" / "supercollider"
    / "start_supercollider.sh"
)


def _script() -> str:
    return START_SCRIPT.read_text(encoding="utf-8")


def test_startup_script_exists() -> None:
    assert START_SCRIPT.is_file(), f"not found: {START_SCRIPT}"


def test_group_one_is_created_at_startup() -> None:
    text = _script()
    assert "/g_new" in text, (
        "startup не создаёт группу 1 — Renardo будет молчать после любого "
        "рестарта scsynth, и логи при этом останутся чистыми"
    )


def test_group_is_created_with_the_right_arguments() -> None:
    """id=1, addToHead (0), target RootNode (0) — иначе группа не та."""
    text = _script()
    assert re.search(r'pack\(\s*">iii"\s*,\s*1\s*,\s*0\s*,\s*0\s*\)', text), (
        "ожидается /g_new с аргументами (1, 0, 0)"
    )


def test_group_is_created_after_scsynth_is_up() -> None:
    """Порядок важен: до готовности порта сообщение уйдёт в пустоту."""
    text = _script()
    ports_wait = text.index("Waiting for scsynth JACK ports")
    # Не ``index``: строка ``/g_new`` встречается ещё и в комментарии выше,
    # где процитирована ошибка сервера. Нужен сам вызов.
    g_new = text.index('osc_string("/g_new")')
    assert ports_wait < g_new, (
        "/g_new отправляется раньше, чем scsynth поднялся"
    )


def test_osc_string_pads_strings_whose_length_is_a_multiple_of_four() -> None:
    """``/g_new`` и ``,iii`` — ровно те строки, что ломались без терминатора.

    Наивное ``(4 - len % 4) % 4`` даёт НОЛЬ добивки для строк длиной 4, 8, …
    Сообщение уходит без терминатора, сервер разбирает его со сдвигом. На
    этом уже обжигались: ``/n_set`` с тегом ``,isf`` приезжал как
    ``Node 1065353216 not found`` — это float 1.0, прочитанный как int32.
    """
    text = _script()
    assert "while len(b) % 4" in text, (
        "osc_string должен добивать нулями в цикле после терминатора"
    )
    assert "s.encode() + ZERO" in text, (
        "osc_string должен добавлять терминатор БЕЗУСЛОВНО"
    )
