"""Issue #2755 — «Записала» без ``memory_save``.

Прогон 35699257202 (акт 2 night-marathon, 22.09.2026) — два шага подряд
с одинаковым диагнозом::

    >>> ACCEPTANCE[n203_sasha_memory_tea]: ❌ expected tool calls not invoked: ['memory_save']
    >>> ACCEPTANCE[n206_boris_memory]:     ❌ expected tool calls not invoked: ['memory_save']

Робот вслух подтверждал, что запомнил, и не звал тул. Факты потом
пересказывались из контекста диалога — до следующей сессии они не
доживают.

Здесь закреплено и узкое правило (:data:`ACTION_CLAIM_RULES`,
категория ``fact_memory_save``), и широкий fallback #2549, в чьём
словаре глаголов не было ни «записал», ни «запомнил».
"""

from __future__ import annotations

from typing import Optional, Tuple

import pytest

from rob_box_voice.core.dialogue_guards import (
    detect_unbacked_action_claim,
    detect_universal_action_claim,
)


# Реплики ниже — дословно из лога прогона 35699257202.
N203_USER = (
    "Робот, запомни: я пью только зелёный чай без сахара и совсем не ем лук"
)
N203_SPOKEN = (
    "Записала, Саша: зелёный чай без сахара и никакого лука, даже жареного."
)
N206_USER = (
    "Робот, запомни про меня: Борис болеет за Спартак и всегда приносит пиццу"
)
N206_SPOKEN = (
    "Запомнила, Борис: болеешь за Спартак и всегда приносишь пиццу."
)


class TestFactMemorySaveRule:
    """Узкое правило Bug E: «запомни факт» + «записала» + tools=[]."""

    @pytest.mark.parametrize(
        "user_input,spoken",
        [
            pytest.param(N203_USER, N203_SPOKEN, id="n203_tea"),
            pytest.param(N206_USER, N206_SPOKEN, id="n206_spartak"),
        ],
    )
    def test_live_regression_fires(self, user_input: str, spoken: str) -> None:
        rule = detect_unbacked_action_claim(
            user_input=user_input, spoken=spoken, tools_called=()
        )
        assert rule is not None, "шаг из прогона обязан ловиться guard'ом"
        assert rule.category == "fact_memory_save"
        assert "memory_save" in rule.tools

    @pytest.mark.parametrize(
        "tools",
        [
            pytest.param(("memory_save",), id="memory_save"),
            # «запомни, меня зовут Саша» закрывается профилем диктора —
            # это тот же факт, а не пропущенный вызов.
            pytest.param(("register_speaker",), id="register_speaker"),
            pytest.param(("memory_save", "speak_text"), id="save_plus_speak"),
        ],
    )
    def test_tool_call_closes_the_claim(self, tools: Tuple[str, ...]) -> None:
        assert (
            detect_unbacked_action_claim(
                user_input=N203_USER, spoken=N203_SPOKEN, tools_called=tools
            )
            is None
        )

    def test_waypoint_request_keeps_its_own_rule(self) -> None:
        """«запомни эту точку» остаётся за ``waypoint_save``.

        Иначе робот получил бы ретрай с требованием ``memory_save`` там,
        где нужен ``save_waypoint``.
        """
        rule = detect_unbacked_action_claim(
            user_input="Робот, запомни эту точку, тут зарядка",
            spoken="Запомнила эту точку.",
            tools_called=(),
        )
        assert rule is not None
        assert rule.category == "waypoint_save"
        assert rule.tools == frozenset({"save_waypoint"})

    def test_track_request_not_stolen(self) -> None:
        """«запиши трек» — не факт памяти; правило молчит."""
        rule = detect_unbacked_action_claim(
            user_input="Робот, запиши трек в библиотеку",
            spoken="Записала трек.",
            tools_called=(),
        )
        assert rule is None or rule.category != "fact_memory_save"

    @pytest.mark.parametrize(
        "user_input,spoken",
        [
            # Просьбы запомнить не было — узкое правило не лезет.
            pytest.param(
                "Робот, как дела?",
                "Нормально, скучаю по паяльнику.",
                id="no_request_no_claim",
            ),
            # Просьба есть, отчёта о сохранении нет — робот честно
            # переспрашивает, ретрай не нужен.
            pytest.param(
                N203_USER,
                "А сахар совсем не кладём или чуть-чуть можно?",
                id="request_without_claim",
            ),
        ],
    )
    def test_no_false_positive(
        self, user_input: str, spoken: Optional[str]
    ) -> None:
        rule = detect_unbacked_action_claim(
            user_input=user_input, spoken=spoken, tools_called=()
        )
        assert rule is None or rule.category != "fact_memory_save"


class TestUniversalGuardKnowsMemoryVerbs:
    """Широкий fallback #2549 — на случай, когда «запомни» юзер не сказал."""

    @pytest.mark.parametrize(
        "spoken,verb",
        [
            pytest.param("Записала, Саша: зелёный чай.", "Записала", id="past_zapisala"),
            pytest.param("Запомнила про лук.", "Запомнила", id="past_zapomnila"),
            pytest.param("Зафиксировала: Спартак.", "Зафиксировала", id="past_fix"),
            pytest.param("Запишу это в память.", "Запишу", id="future_zapishu"),
            pytest.param("Запомню, что ты не ешь лук.", "Запомню", id="future_zapomnyu"),
        ],
    )
    def test_memory_claim_without_tools_is_a_hit(
        self, spoken: str, verb: str
    ) -> None:
        hit = detect_universal_action_claim(spoken=spoken, tools_called=())
        assert hit is not None, "заявление о памяти без тула — hallucination"
        assert hit.verb.lower() == verb.lower()

    def test_memory_save_justifies_the_claim(self) -> None:
        assert (
            detect_universal_action_claim(
                spoken=N203_SPOKEN, tools_called=("memory_save",)
            )
            is None
        )

    def test_recall_via_memory_search_is_not_a_hit(self) -> None:
        """«Я записала это вчера» после ``memory_search`` — законный ответ."""
        assert (
            detect_universal_action_claim(
                spoken="Записала это ещё вчера: зелёный чай без сахара.",
                tools_called=("memory_search",),
            )
            is None
        )
