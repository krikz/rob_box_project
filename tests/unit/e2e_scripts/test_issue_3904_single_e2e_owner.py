"""test_issue_3904_single_e2e_owner.py — статический гард «один владелец режима e2e».

Issue #3904 (аудит эпика #3683 §3.5): за эпик e2e-данные трижды попадали в боевую ``/data`` робота, и каждый раз
гард добавлялся в одну ноду отдельно — свой ``e2e_mode`` у ``speaker_id_node``, у ``mcp_server``, у ``dialog_node``.
Дубль флага и есть баг: акт 3 марафона пошёл на боевую ``speakers.db``, потому что флаги жили каждый своей жизнью.

Гард держит по коду:
  1. переключатель ``e2e_mode`` объявляет только нода-владелец (``rob_box_core.e2e_mode.OWNER_NODE``);
  2. каждая нода-последователь (``E2EModeFollower``) сверяется харнессом (``E2E_MODE_FOLLOWERS``) — новый
     последователь без проверки в харнессе писал бы в боевую БД, если бы не применил режим, и никто бы не узнал;
  3. харнесс ставит режим только владельцу, и только через ``_set_e2e_mode``.

Run:
  python -m pytest tests/unit/e2e_scripts/test_issue_3904_single_e2e_owner.py -v --no-cov
"""

from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SRC = REPO_ROOT / "src"
E2E_SCRIPT = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"
OWNER_MODULE = SRC / "rob_box_core" / "rob_box_core" / "e2e_mode.py"

_DECLARE_MODE = re.compile(r"""declare_parameter\(\s*["']e2e_mode["']""")
_FOLLOWER = re.compile(r"\bE2EModeFollower\(")


def _prod_py():
    for path in SRC.rglob("*.py"):
        parts = set(path.parts)
        if "test" in parts or "tests" in parts or path.name.startswith("test_"):
            continue
        yield path, path.read_text(encoding="utf-8", errors="replace")


def _owner_constant(name: str) -> str:
    m = re.search(rf'^{name} = "([^"]+)"', OWNER_MODULE.read_text(encoding="utf-8"), re.M)
    assert m, f"{name} не найден в {OWNER_MODULE}"
    return m.group(1)


def _node_name(source: str) -> str:
    m = re.search(r"""super\(\).__init__\(\s*["']([A-Za-z0-9_]+)["']""", source)
    assert m, "у ноды-последователя не нашлось имени в super().__init__(...)"
    return "/" + m.group(1)


def test_only_the_owner_declares_e2e_mode():
    owner = _owner_constant("OWNER_NODE")
    declaring = sorted(
        str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p, src in _prod_py() if _DECLARE_MODE.search(src))
    assert declaring == ["src/rob_box_voice/rob_box_voice/speaker_id_node.py"], (
        f"e2e_mode объявлен не только у владельца {owner}: {declaring}. Второй переключатель — это и есть баг #3904: "
        "нода должна стать E2EModeFollower (rob_box_core.e2e_mode), а не заводить свой режим."
    )


def test_every_follower_is_checked_by_the_harness():
    followers = sorted(_node_name(src) for _p, src in _prod_py() if _FOLLOWER.search(src) and "class E2EModeFollower"
                       not in src)
    m = re.search(r'^E2E_MODE_FOLLOWERS="([^"]*)"', E2E_SCRIPT.read_text(encoding="utf-8"), re.M)
    assert m, "E2E_MODE_FOLLOWERS не найден в харнессе"
    assert followers, "ни одного последователя — mcp_server (память фактов) обязан им быть"
    assert followers == sorted(m.group(1).split()), (
        f"последователи в коде {followers} ≠ проверяемые харнессом {m.group(1).split()}: последователь, не "
        "сверенный по e2e_mode_applied, мог не применить режим и писать в боевую /data молча"
    )


def test_harness_sets_mode_only_on_the_owner():
    src = E2E_SCRIPT.read_text(encoding="utf-8")
    owner = _owner_constant("OWNER_NODE")
    sets = re.findall(r"ros2 param set (\S+) e2e_mode\b", src)
    assert sets and set(sets) == {owner}, sets
    body = re.search(r"^_set_e2e_mode\(\) \{.*?^\}", src, re.S | re.M).group(0)
    assert f"ros2 param set {owner} e2e_mode" in body
    calls = [line for line in src.splitlines()
             if line.lstrip().startswith("robot_ros") and f"ros2 param set {owner} e2e_mode " in line]
    assert len(calls) == 1, f"режим e2e ставится в харнессе в нескольких местах: {calls}"


def test_followers_read_mode_from_owner_topic():
    topic = _owner_constant("TOPIC")
    assert topic == "/e2e/mode"
    for path, src in _prod_py():
        if _FOLLOWER.search(src) and path != OWNER_MODULE:
            assert "rob_box_core.e2e_mode import" in src, path
