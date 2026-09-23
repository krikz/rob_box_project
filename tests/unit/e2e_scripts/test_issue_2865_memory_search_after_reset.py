"""
test_issue_2865_memory_search_after_reset.py — проверка #2793 в акте 2 не тавтология

Шаг n208_memory_search_tea акта 2 ночного марафона должен был доказывать
#2793 («memory_search находит факт, сохранённый memory_save в voice_facts»),
но спрашивал про чай, пока факт n203 ещё лежал в окне чата:

    [mcp_server]    memory_save {'fact': 'Пьёт только зелёный чай без …'}   <- n203
    … assistant: 'Записал, Саша. Зелёный чай без сахара, лук — табу.'
    [dialogue_node] spoken='Ты сказал, что пьёшь зелёный чай без сахара, Саша.' tools=[]

(E2E акт 2, run 35886659057.) LLM отвечала из истории и тул не звала; а
ключевое слово «чай» есть в самом вопросе, так что даже ответ «ничего про
чай не нашёл» проходил бы keyword-проверку.

Контракт, который держит этот файл (по СОДЕРЖИМОМУ сценария акта 2):
  1. каждый шаг, ожидающий memory_search, стоит ПОСЛЕ шага сброса сессии,
     а сброс — ПОСЛЕ последнего memory_save (факт живёт только в БД);
  2. фраза сброса реально распознаётся dialogue_node как «новая сессия»
     (фразы читаются из исходника, а не копируются сюда);
  3. сброс шага проверяется паттерном (не случился сброс → шаг красный);
  4. ключевые слова шага поиска не встречаются в тексте вопроса;
  5. паттерн «поиск вернул хиты» матчит НАСТОЯЩУЮ строку лога
     MemorySearchTool при N>=1 и НЕ матчит при N=0.

Run:
  python -m pytest tests/unit/e2e_scripts/test_issue_2865_memory_search_after_reset.py -v --no-cov
"""

from __future__ import annotations

import ast
import importlib.util
import json
import re
import sys
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
ACT2 = (
    REPO_ROOT
    / ".github"
    / "e2e"
    / "scenarios"
    / "night"
    / "night_marathon_act2_acquaintance_v1.json"
)
DIALOGUE_NODE = (
    REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "dialogue_node.py"
)
MCP_PKG = REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools"

#: Строка, которую dialogue_node.reset_session пишет в лог при сбросе.
RESET_LOG_LINE = "🧹 [new-session] session reset: text='новая сессия' tg=False"


def _steps() -> list[dict]:
    return json.loads(ACT2.read_text(encoding="utf-8"))["steps"]


def _tools(step: dict) -> list[str]:
    return (step.get("acceptance") or {}).get("expected_tool_calls") or []


def _new_session_phrases() -> tuple[str, ...]:
    """_DEFAULT_NEW_SESSION_PHRASES из исходника dialogue_node (без импорта:
    нода тянет rclpy)."""
    tree = ast.parse(DIALOGUE_NODE.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        target = None
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            target = node.target.id
        elif (
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
        ):
            target = node.targets[0].id
        if target == "_DEFAULT_NEW_SESSION_PHRASES":
            return tuple(ast.literal_eval(node.value))
    raise AssertionError("_DEFAULT_NEW_SESSION_PHRASES не найден в dialogue_node.py")


def _is_reset_step(step: dict) -> bool:
    text = (step.get("text") or "").lower()
    return any(p in text for p in _new_session_phrases())


def _search_steps() -> list[tuple[int, dict]]:
    return [(i, s) for i, s in enumerate(_steps()) if "memory_search" in _tools(s)]


def test_act2_has_memory_search_step() -> None:
    assert _search_steps(), (
        "акт 2 обязан проверять #2793 (memory_search по voice_facts) — "
        "шаг с expected_tool_calls=['memory_search'] пропал"
    )


def test_memory_search_asked_only_after_reset_that_follows_memory_save() -> None:
    steps = _steps()
    last_save = max(i for i, s in enumerate(steps) if "memory_save" in _tools(s))
    for idx, step in _search_steps():
        resets = [i for i in range(last_save + 1, idx) if _is_reset_step(steps[i])]
        assert resets, (
            f"{step['label']}: memory_search спрашивается, пока факт ещё в окне "
            f"чата — между последним memory_save ({steps[last_save]['label']}) "
            "и этим шагом нет сброса сессии. LLM ответит из истории и тул не "
            "позовёт (issue #2865, run 35886659057)."
        )


def test_reset_step_is_verified_by_pattern() -> None:
    steps = _steps()
    for idx, step in _search_steps():
        reset = next(
            (steps[i] for i in range(idx - 1, -1, -1) if _is_reset_step(steps[i])),
            None,
        )
        assert reset is not None, f"{step['label']}: перед шагом нет сброса сессии"
        pats = reset.get("patterns") or []
        assert pats, f"{reset['label']}: сброс без паттерна — не доказан"
        assert all(re.search(p, RESET_LOG_LINE) for p in pats), (
            f"{reset['label']}: паттерны {pats} не матчат строку сброса "
            f"dialogue_node {RESET_LOG_LINE!r}"
        )


def test_search_keywords_are_not_in_the_question() -> None:
    for _idx, step in _search_steps():
        kws = (step.get("acceptance") or {}).get("expected_keywords") or []
        assert kws, f"{step['label']}: без expected_keywords факт в ответе не проверен"
        question = step["text"].lower()
        for kw in kws:
            for alt in (a.strip() for a in kw.lower().split("|")):
                assert alt and alt not in question, (
                    f"{step['label']}: ключ {alt!r} есть в самом вопросе — "
                    "проверка проходит на ответе «ничего не нашёл»"
                )


# ── паттерн «поиск вернул хиты» против настоящей строки лога тула ──────────


def _load_memory_tools() -> types.ModuleType:
    """Грузим base.py + tools/memory.py без tools/__init__ (тот тянет ROS)."""
    pkg_name = "_issue2865_mcp"
    if f"{pkg_name}.tools.memory" in sys.modules:
        return sys.modules[f"{pkg_name}.tools.memory"]
    pkg = types.ModuleType(pkg_name)
    pkg.__path__ = [str(MCP_PKG)]
    sub = types.ModuleType(f"{pkg_name}.tools")
    sub.__path__ = [str(MCP_PKG / "tools")]
    sys.modules[pkg_name] = pkg
    sys.modules[f"{pkg_name}.tools"] = sub
    for name, path in (
        (f"{pkg_name}.base", MCP_PKG / "base.py"),
        (f"{pkg_name}.tools.memory", MCP_PKG / "tools" / "memory.py"),
    ):
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
    return sys.modules[f"{pkg_name}.tools.memory"]


class _Logger:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def info(self, msg: str) -> None:
        self.lines.append(msg)

    warning = warn = error = debug = info


class _Embedder:
    @staticmethod
    def is_available() -> bool:
        return False


class _Memory:
    embedder = _Embedder()

    def __init__(self, hits: list[dict]) -> None:
        self._hits = hits

    def search(self, query, limit=5, speaker_id=None):  # noqa: ARG002
        return list(self._hits)


class _Node:
    def __init__(self, memory: _Memory) -> None:
        self.voice_memory = memory
        self._logger = _Logger()

    def get_logger(self) -> _Logger:
        return self._logger


def _search_log(hits: list[dict]) -> str:
    memory_mod = _load_memory_tools()
    node = _Node(_Memory(hits))
    tool = memory_mod.MemorySearchTool(node)
    result = tool.execute(query="чай")
    assert result.success, result.message
    return "\n".join(
        f"[INFO] [1790180960.1] [mcp_server]: {line}" for line in node._logger.lines
    )


_FACT = {
    "kind": "fact",
    "role": "fact",
    "content": "Пьёт только зелёный чай без сахара",
    "score": 1.0,
    "source": "fact",
    "category": "preference",
}


def _search_patterns() -> list[str]:
    pats: list[str] = []
    for _idx, step in _search_steps():
        pats.extend(p for p in step.get("patterns") or [] if "memory_search" in p)
    return pats


def test_search_step_requires_nonempty_result_pattern() -> None:
    assert _search_patterns(), (
        "шаг memory_search без паттерна на строку лога тула: «тул вызван» "
        "не значит «поиск что-то вернул» (#2793 — поиск был пустым)"
    )


@pytest.mark.parametrize("pattern", _search_patterns() or ["<нет паттерна>"])
def test_result_pattern_matches_real_tool_log_only_with_hits(pattern: str) -> None:
    hit_log = _search_log([_FACT])
    empty_log = _search_log([])
    assert re.search(pattern, hit_log), (
        f"паттерн {pattern!r} не матчит настоящую строку MemorySearchTool "
        f"с 1 хитом:\n{hit_log}"
    )
    assert not re.search(pattern, empty_log), (
        f"паттерн {pattern!r} матчит ПУСТОЙ поиск — не отличает #2793 от "
        f"починенного:\n{empty_log}"
    )
