"""Имя ответившего провайдера в метрике промпта (ADR-0150 §14 A5; #3696 ч.5; перенесено из ``agent_core``, #4354 A3b).

Строка ``🧮 [prompt]`` писала ``provider=health-aware-fallback`` — имя обёртки цепочки, одно на DeepSeek и MiniMax.
Цепочка помнит, кто ответил (``HealthAwareFallbackLLM.answered_by``), транспорт хода v2 берёт имя оттуда.
"""

from __future__ import annotations

from rob_box_dialog_host.dialog_v2_ports import answering_provider


def test_answering_provider_prefers_chain_answer_then_name() -> None:
    class _Plain:
        name = "deepseek"

    class _Chain:
        name = "health-aware-fallback"
        answered_by = "minimax"

    class _ChainBeforeCall:
        name = "health-aware-fallback"
        answered_by = None

    assert answering_provider(_Plain()) == "deepseek"
    assert answering_provider(_Chain()) == "minimax"
    assert answering_provider(_ChainBeforeCall()) == "health-aware-fallback"
    assert answering_provider(object()) == "unknown"
