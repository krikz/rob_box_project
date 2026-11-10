"""Системный промпт ТАРС: правила оператора + раздел инструментов, который строит код (ADR-0150 §9.3, #3298;
PR-13b #3708). Правила - основа ядра ``rob_box_dialog/prompts/operator.txt`` с персоной оператора из профиля робота
(``docker/sound/config/operator_persona.txt``, ADR-0167 Ш1б-4; ``rob_box_dialog.prompts.operator_rules``).

Правила держат только поведение (кто ТАРС, честность, краткость) и ни одного имени тула: имена, «что
только у оператора», чем говорить вслух и словарь метрик — из реестра оператора
(``rob_box_core.tool_catalog.operator_visible_tools``: срез ``operator.*`` + видимые Личности), из ``knowledge``
(тулы речи) и ``metrics_source.METRIC_DICTIONARY``. Второй копии списка тулов нет — промпт не может назвать то,
чего нет в реестре, и не может промолчать о том, что есть (#3298: промпт звал ``set_voice_preset``, которого не было).

Об итоге действия говорит код по событию владельца (§5.3), модель «выполнено» не пишет (:data:`CONFIRM_RULE`).
"""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

from rob_box_core.tool_catalog import ToolCatalogEntry, operator_visible_tools
from rob_box_dialog import knowledge as kn
from rob_box_dialog.owners import owned_tools
from rob_box_supervisor.metrics_source import metric_dictionary_text

#: Метрики панели TARS 2 называет тул ``show_metrics`` (его словарь — ``metrics_source``).
METRICS_TOOL = "show_metrics"
#: Заголовок сгенерированного раздела (по нему тест находит раздел).
TOOLS_HEADER = "Инструменты (список строит код из реестра оператора):"
CONFIRM_RULE = "Об итоге действия скажет код по результату инструмента — сам «выполнено»/«не вышло» не пиши."


def _first_sentence(text: str) -> str:
    head = (text or "").strip().split(". ")[0].rstrip(".")
    return head[:160]


def _common_line(common: Sequence[str]) -> List[str]:
    """Строка общих тулов; у продукта без них (нейтральное ядро, ADR-0167 Ш1в) строки нет, а не «…робота: .»."""
    return ["Общие инструменты робота: " + ", ".join(common) + "."] if common else []


def tools_section(entries: Iterable[ToolCatalogEntry]) -> str:
    """Раздел промпта об инструментах оператора из записей реестра: только те, что движок исполняет командой
    (``knowledge.TOOL_CLASSES``), без тулов «только кодом» (``TOOLS_CODE_ONLY``)."""
    entries = tuple(e for e in entries if e.name in kn.TOOL_CLASSES and e.name not in kn.TOOLS_CODE_ONLY)
    names = {e.name for e in entries}
    own = [e for e in entries if not e.llm_visible]
    aloud = [t for t in kn.ALOUD_TOOLS if t in names]
    common = sorted(n for n in names - {e.name for e in own} if n not in kn.SPEECH_CHANNEL_TOOLS)
    lines = [TOOLS_HEADER]
    lines += [f"- {e.name} — {_first_sentence(e.description)}." for e in sorted(own, key=lambda e: e.name)]
    lines += _common_line(common)
    if aloud:
        lines.append(f"Сказать вслух людям рядом — {aloud[0]} (голос робота; личность на это время молчит); "
                     f"{', '.join(kn.SPEECH_CHANNEL_TOOLS)} — инструмент личности, его не вызывай.")
    if METRICS_TOOL in names:
        lines.append(f"{METRICS_TOOL}: в запрос — имя из словаря, PromQL сам не пиши: {metric_dictionary_text()}.")
    lines.append(CONFIRM_RULE)
    return "\n".join(lines)


def operator_prompt(rules: str, entries: Tuple[ToolCatalogEntry, ...] = (),
                    owners: Optional[Sequence[str]] = None) -> str:
    """Промпт ТАРС: правила + раздел инструментов реестра (``entries`` пусто — реестр оператора каталога, только тулы
    владельцев продукта ``owners``: ``owners.owned_tools``, ADR-0167 Ш1в; ``None`` — умолчание ядра, без тулов).
    Пустые правила — пустой промпт (нет правил — нет агента)."""
    if not rules.strip():
        return ""
    own = owned_tools(owners)
    registry = entries or tuple(e for e in operator_visible_tools() if e.name in own)
    return f"{rules.strip()}\n\n{tools_section(registry)}"


__all__ = ["CONFIRM_RULE", "METRICS_TOOL", "TOOLS_HEADER", "operator_prompt", "tools_section"]
