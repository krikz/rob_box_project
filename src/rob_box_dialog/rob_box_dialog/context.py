"""Контекст вызова LLM структурой (ADR-0150 §7.3, §14 A5).

Снимок хода — разделы-данные :data:`knowledge.CONTEXT_SECTIONS`; текст для модели строит код
(:func:`build_context`), а не склейка строк с правилами. Размер вызова — по блокам :data:`knowledge.CONTEXT_BLOCKS`
из структуры запроса (роль и место сообщения, :func:`measure`), бюджет блоков — :data:`knowledge.CONTEXT_BUDGET_TOKENS`.
Строка лога :data:`PROMPT_TOKENS_MARK` на каждый вызов — источник A5 для ``scripts/dialog/turn_metrics.py``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Iterable, Mapping, Optional, Sequence, Tuple

from rob_box_core.token_estimate import estimate_message_tokens, estimate_tokens, estimate_tool_tokens
from rob_box_dialog import knowledge as kn

#: Обёртка снимка хода (её имя знают схемы тулов: ``<system_context>/<speaker_id>``).
SNAPSHOT_TAG = "system_context"
#: Маркер строки лога о размере вызова LLM (читает ``scripts/dialog/logparse.py``).
PROMPT_TOKENS_MARK = "🧮 [prompt] tokens="


def build_context(sections: Mapping[str, Any]) -> str:
    """Снимок хода ``<system_context>``: разделы в порядке ``CONTEXT_SECTIONS``, пустые пропущены.

    Значение — данные: словарь/список/число идут JSON, строка — как есть (многострочная — блоком). Раздел вне
    ``CONTEXT_SECTIONS`` — ``ValueError``: новое знание в снимок попадает через таблицу, а не строкой по месту.
    """
    unknown = sorted(set(sections) - set(kn.CONTEXT_SECTIONS))
    if unknown:
        raise ValueError(f"build_context: unknown sections {unknown}; known: {kn.CONTEXT_SECTIONS}")
    lines = [f"<{SNAPSHOT_TAG}>"]
    for key in kn.CONTEXT_SECTIONS:
        value = sections.get(key)
        if value is None or (not isinstance(value, (int, float)) and not value):
            continue
        text = value if isinstance(value, str) else json.dumps(value, ensure_ascii=False, separators=(",", ":"))
        lines.append(f"  <{key}>\n{text}\n  </{key}>" if "\n" in text else f"  <{key}>{text}</{key}>")
    lines.append(f"</{SNAPSHOT_TAG}>")
    return "\n".join(lines)


@dataclass(frozen=True)
class ContextBlocks:
    """Токены одного вызова LLM по блокам ``CONTEXT_BLOCKS`` (оценка; ``total`` — их сумма)."""

    tokens: Mapping[str, int]

    @property
    def total(self) -> int:
        return sum(self.tokens.values())

    def over_budget(self) -> Tuple[str, ...]:
        """Блоки сверх ``CONTEXT_BUDGET_TOKENS`` в порядке ``CONTEXT_BLOCKS``."""
        budget = kn.CONTEXT_BUDGET_TOKENS
        return tuple(b for b in kn.CONTEXT_BLOCKS if b in budget and self.tokens.get(b, 0) > budget[b])

    def log_line(self, *, prompt_tokens: int, estimated: bool, provider: str, view: str) -> str:
        """Строка лога вызова: ``tokens`` — от провайдера (``estimated=False``) или оценка, далее блоки и перерасход."""
        blocks = " ".join(f"{b}={self.tokens.get(b, 0)}" for b in kn.CONTEXT_BLOCKS)
        over = ",".join(self.over_budget()) or "-"
        return (f"{PROMPT_TOKENS_MARK}{prompt_tokens} estimated={estimated} provider={provider} view={view} "
                f"{blocks} over={over}")


def measure(messages: Sequence[Any], tools: Iterable[Mapping[str, Any]] = (),
            snapshot: Optional[str] = None) -> ContextBlocks:
    """Разложить запрос на блоки по структуре: первое system — персона, прочие system — факты; до последней реплики
    человека — окно; в ней ``snapshot`` (снимок хода, если он там) отдельно от текста; после неё — тул-цикл хода."""
    tokens = dict.fromkeys(kn.CONTEXT_BLOCKS, 0)
    current = _last_user(messages)
    for i, message in enumerate(messages):
        cost = estimate_message_tokens(message)
        if _role(message) == "system":
            tokens["system" if i == 0 else "facts"] += cost
        elif current is None or i < current:
            tokens["window"] += cost
        else:
            snap = estimate_tokens(snapshot) if i == current and snapshot and snapshot in _text(message) else 0
            tokens["snapshot"] += snap
            tokens["turn"] += max(0, cost - snap)
    tokens["tools"] = sum(estimate_tool_tokens(t) for t in tools)
    return ContextBlocks(MappingProxyType(tokens))


def _last_user(messages: Sequence[Any]) -> Optional[int]:
    return next((i for i in range(len(messages) - 1, -1, -1) if _role(messages[i]) == "user"), None)


def _role(message: Any) -> str:
    return str(message.get("role") if isinstance(message, Mapping) else getattr(message, "role", ""))


def _text(message: Any) -> str:
    content = message.get("content") if isinstance(message, Mapping) else getattr(message, "content", "")
    return content if isinstance(content, str) else ""


__all__ = ["ContextBlocks", "PROMPT_TOKENS_MARK", "SNAPSHOT_TAG", "build_context", "measure"]
