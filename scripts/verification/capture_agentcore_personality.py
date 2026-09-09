"""Byte-for-byte personality capture for the AgentCore -> AgentCore rename (issue #1986).

Runs a fixed set of deterministic "personality" scenarios against the core class and
dumps the exact LLM message lists + DialogResult fields to JSON. The same script is run
BEFORE the refactor (--module rob_box_harness.core.agent_core) and AFTER
(--module rob_box_harness.core.agent_core); the two JSON files must be identical
("личность байт-в-байт", target-arch §5.1/§5.2, DoD issue #1986).

Usage:
    python scripts/verification/capture_agentcore_personality.py \
        --module rob_box_harness.core.agent_core \
        --out evidence/issue-1986-agentcore/personality_pre.json

The system_prompt is the REAL personality file (master_prompt_compact.txt) and
skill_prompts are REAL fragments from src/rob_box_voice/prompts/skills/, so the
capture covers the actual production "личность" surface, not a stub.
"""

from __future__ import annotations

import argparse
import asyncio
import importlib
import json
import sys
from pathlib import Path
from typing import Any

# repo root = <root>/scripts/verification/<file> -> parents[2]
REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_PROMPTS = REPO_ROOT / "src" / "rob_box_voice" / "prompts"
MASTER_PROMPT = VOICE_PROMPTS / "master_prompt_compact.txt"
SKILLS_DIR = VOICE_PROMPTS / "skills"

_COMPOSER_TEXT = "СКИЛЛ КОМПОЗИТОРА: вызывай compose_music, не описывай словами."
_PLAYER_TEXT = "СКИЛЛ ПЛЕЕРА: gen_list_library потом gen_play_from_library."
# Real fragments when present, else the two literals above (keeps script runnable
# even when rob_box_voice sources are not installed).
_SKILL_FRAGMENTS: dict[str, str] = {
    "composer": _COMPOSER_TEXT,
    "player": _PLAYER_TEXT,
}


def _load_prompt_text(path: Path) -> str:
    if path.is_file():
        return path.read_text(encoding="utf-8")
    return ""


def _load_skill_fragments() -> dict[str, str]:
    """Load a handful of REAL skill fragments from rob_box_voice if available."""
    frags: dict[str, str] = {}
    for name in ("composer", "player", "dj", "memory", "navigation"):
        f = SKILLS_DIR / f"{name}.txt"
        text = _load_prompt_text(f)
        if text:
            frags[name] = text
    if not frags:
        frags = dict(_SKILL_FRAGMENTS)
    return frags


class _CapturingProvider:
    """Fake LLM that records every complete() message list and replies canned text."""

    name = "capture"

    def __init__(self, reply: str = "ок") -> None:
        self.calls: list[list[Any]] = []
        self._reply = reply

    async def complete(self, messages: Any, *, tools: Any = (), **_: Any) -> Any:
        from rob_box_llm.provider import LLMResponse

        self.calls.append(list(messages))
        return LLMResponse(content=self._reply, finish_reason="stop")

    async def aclose(self) -> None:
        return None


def _msg_list(provider: _CapturingProvider) -> list[dict[str, str]]:
    """Normalised [{role, content}] of every LLM call — the byte-for-byte signal."""
    out: list[dict[str, str]] = []
    for call in provider.calls:
        for m in call:
            out.append({"role": m.role, "content": str(m.content)})
    return out


def _scalar(value: Any) -> Any:
    """Coerce enums/non-JSON values to a stable string for the diff."""
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if hasattr(value, "value"):
        return value.value
    return str(value)


def _result_dict(result: Any) -> dict[str, Any]:
    """Normalise a DialogResult — keep only deterministic public fields."""
    return {
        "spoken_text": _scalar(getattr(result, "spoken_text", None)),
        "tools_called": [_scalar(t) for t in (getattr(result, "tools_called", None) or [])],
        "finish_reason": _scalar(getattr(result, "finish_reason", None)),
        "new_state": _scalar(getattr(result, "new_state", None)),
        "error": (
            _scalar(getattr(result, "error", None))
            if getattr(result, "error", None) is not None
            else None
        ),
    }


def _primed_dsm(module: Any) -> Any:
    dsm = module.DialogueStateMachine()
    dsm.on_event(module.DialogueEvent.WAKE_WORD)
    dsm.on_event(module.DialogueEvent.STT_RESULT)
    return dsm


def _scenario_plain(module: Any) -> dict[str, Any]:
    """S1 — pre-skill personality baseline: system_prompt + dynamic + speaker ctx."""
    from rob_box_harness.memory import InMemoryStore
    from rob_box_harness.tools import FakeToolProvider

    provider = _CapturingProvider(reply="Сыграю ритм на бочке и малом.")
    core = module.AgentCore(
        llm=provider,
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        dsm=_primed_dsm(module),
        system_prompt=_load_prompt_text(MASTER_PROMPT),
        use_streaming=False,
        history_trim_limit=20,
    )
    result = asyncio.run(
        core.process_input(
            "сыграй жёсткий барабанный бит",
            preclassified_event=module.DialogueEvent.STT_RESULT,
            dynamic_system="<system_context>speaker=гость, tts_voice=female, session_locked=false</system_context>",
            speaker_context="Ты говоришь с гостем по имени Сергей.",
        )
    )
    return {
        "scenario": "S1-plain-personality",
        "messages": _msg_list(provider),
        "result": _result_dict(result),
        "state": {
            "active_skill": core.active_skill,
            "known_skills": list(core.known_skills()),
        },
    }


def _scenario_skill_active(module: Any) -> dict[str, Any]:
    """S2 — Move A on: real fragments loaded, active skill set, skill text last."""
    from rob_box_harness.memory import InMemoryStore
    from rob_box_harness.tools import FakeToolProvider

    provider = _CapturingProvider(reply="Сейчас подберу из библиотеки.")
    core = module.AgentCore(
        llm=provider,
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        dsm=_primed_dsm(module),
        system_prompt=_load_prompt_text(MASTER_PROMPT),
        use_streaming=False,
        skill_prompts=_load_skill_fragments(),
        history_trim_limit=20,
    )
    core.set_active_skill("player")
    result = asyncio.run(
        core.process_input(
            "поставь что-нибудь из фонотеки",
            preclassified_event=module.DialogueEvent.STT_RESULT,
        )
    )
    return {
        "scenario": "S2-skill-active-player",
        "messages": _msg_list(provider),
        "result": _result_dict(result),
        "state": {
            "active_skill": core.active_skill,
            "skill_load_counters": list(core.skill_load_counters),
            "known_skills": list(core.known_skills()),
        },
    }


def _scenario_deep_history(module: Any) -> dict[str, Any]:
    """S3 — deep session: 20 pairs in the window, skill fragment still last."""
    from rob_box_harness.memory import InMemoryStore, Turn
    from rob_box_harness.tools import FakeToolProvider

    provider = _CapturingProvider(reply="Понял.")
    core = module.AgentCore(
        llm=provider,
        tools=FakeToolProvider(),
        memory=InMemoryStore(),
        dsm=_primed_dsm(module),
        system_prompt=_load_prompt_text(MASTER_PROMPT),
        use_streaming=False,
        skill_prompts=_load_skill_fragments(),
        history_trim_limit=20,
    )
    for i in range(20):
        core._turn_window.append(Turn(role="user", content=f"вопрос {i}"))
        core._turn_window.append(Turn(role="assistant", content=f"ответ {i}"))
    core.set_active_skill("composer")
    result = asyncio.run(
        core.process_input(
            "сделай бит под мой голос",
            preclassified_event=module.DialogueEvent.STT_RESULT,
        )
    )
    return {
        "scenario": "S3-deep-history-20",
        "messages": _msg_list(provider),
        "result": _result_dict(result),
        "state": {
            "active_skill": core.active_skill,
            "known_skills": list(core.known_skills()),
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--module", required=True, help="e.g. rob_box_harness.core.agent_core")
    ap.add_argument("--out", required=True, help="output JSON path")
    args = ap.parse_args()

    module = importlib.import_module(args.module)
    # Both names may exist (alias) — resolve the class by probing AgentCore first.
    cls_name = "AgentCore" if hasattr(module, "AgentCore") else "AgentCore"
    setattr(module, "AgentCore", getattr(module, cls_name))

    scenarios = [
        _scenario_plain(module),
        _scenario_skill_active(module),
        _scenario_deep_history(module),
    ]
    payload = {"module": args.module, "class": cls_name, "scenarios": scenarios}
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, default=_scalar),
        encoding="utf-8",
    )
    print(f"wrote {out} ({out.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
