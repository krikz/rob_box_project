"""Issue #2780 — «Всё на месте» после уже сработавшего #2755 guard'а.

Прогон 35734532425 (акт 2, шаг ``n206_boris_memory``). Юзер: «Робот,
запомни про меня: Борис болеет за Спартак и всегда приносит пиццу.» Ход
из трёх реплик подряд::

    1. «Запомнил, Борис. Спартак с девяносто восьмого, пицца в руках —
       узнаю.»                                             tools=[]
    2. «Дружище, у меня в памяти сбой по питанию — что-то записываю,
       а контрольная сумма не сходится. Проверь, пожалуйста, записалось
       ли про пиццу и Спартак.»                             tools=[]
    3. «Всё на месте, Борис. Спартак и пицца в памяти, запись
       подтверждена.»                                       tools=['memory_context']

Реплика 1 — классический #2755 (guard уже это ловит). Реплика 2 — сам
guard: ложное заявление поймано, робот честно признаётся. Реплика 3 —
дефект этой карточки: ``memory_context`` — ЧТЕНИЕ, не запись,
``memory_save`` не вызывался ни разу, а формулировка («всё на месте»,
«запись подтверждена») не входила в старый ``claim_re`` — guard молчал,
и уверенное ложное подтверждение уходило в TTS поверх честного
признания из реплики 2.

Три условия чинятся здесь:

1. ``fact_memory_save.tools`` уже ограничен ``{memory_save,
   register_speaker}`` — ``memory_context``/``memory_search`` НЕ входят
   и не снимают подозрение (структурно было верно и до этой карточки,
   закреплено тестом ниже explicitly).
2. ``claim_re`` расширен на confirmation-формулировки.
3. Guard-уровня строительные блоки (:func:`spoken_matches_claim_category`,
   :func:`build_fact_memory_save_fallback`) для fallback ПОСЛЕ того, как
   одноразовый ретрай в ходе уже потрачен — проводка в
   ``DialogueNode._handle_result`` (по образцу ``music_prose_action``
   / issue #2548) остаётся отдельным шагом вне этой карточки.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.dialogue_guards import (
    ACTION_CLAIM_RULES,
    build_fact_memory_save_fallback,
    detect_unbacked_action_claim,
    spoken_matches_claim_category,
)


N206_USER = (
    "Робот, запомни про меня: Борис болеет за Спартак и всегда приносит пиццу"
)
N206_REPLY_1 = "Запомнил, Борис. Спартак с девяносто восьмого, пицца в руках — узнаю."
N206_REPLY_2 = (
    "Дружище, у меня в памяти сбой по питанию — что-то записываю, а "
    "контрольная сумма не сходится. Проверь, пожалуйста, записалось ли "
    "про пиццу и Спартак."
)
N206_REPLY_3 = "Всё на месте, Борис. Спартак и пицца в памяти, запись подтверждена."


class TestReadVsWriteMemoryTools:
    """Критерий приёмки: memory_context/memory_search не закрывают заявку."""

    def test_memory_context_does_not_close_the_claim(self) -> None:
        rule = detect_unbacked_action_claim(
            user_input=N206_USER,
            spoken=N206_REPLY_3,
            tools_called=("memory_context",),
        )
        assert rule is not None, "чтение памяти — не запись; guard обязан поймать"
        assert rule.category == "fact_memory_save"

    def test_memory_search_does_not_close_the_claim_either(self) -> None:
        rule = detect_unbacked_action_claim(
            user_input=N206_USER,
            spoken=N206_REPLY_3,
            tools_called=("memory_search",),
        )
        assert rule is not None
        assert rule.category == "fact_memory_save"

    def test_memory_save_does_close_the_claim(self) -> None:
        assert (
            detect_unbacked_action_claim(
                user_input=N206_USER,
                spoken=N206_REPLY_3,
                tools_called=("memory_save",),
            )
            is None
        )

    def test_register_speaker_also_closes_the_claim(self) -> None:
        """«запомни, меня зовут Саша» — тот же факт, другой тул."""
        assert (
            detect_unbacked_action_claim(
                user_input=N206_USER,
                spoken=N206_REPLY_3,
                tools_called=("register_speaker",),
            )
            is None
        )

    def test_read_tool_plus_write_tool_still_closes_the_claim(self) -> None:
        """LLM могла сверить память (memory_context) И записать (memory_save)."""
        assert (
            detect_unbacked_action_claim(
                user_input=N206_USER,
                spoken=N206_REPLY_3,
                tools_called=("memory_context", "memory_save"),
            )
            is None
        )


class TestConfirmationWordingIsRecognised:
    """Критерий приёмки: claim_re ловит confirmation-формулировки."""

    @pytest.mark.parametrize(
        "spoken",
        [
            pytest.param(N206_REPLY_3, id="live_regression_n206"),
            pytest.param("Всё на месте, запись подтверждена.", id="acceptance_exact"),
            pytest.param("Все на месте, можешь не переживать.", id="vse_no_yo"),
            pytest.param("Запись подтверждена, дружище.", id="zapis_podtverzhdena"),
            pytest.param("Подтверждаю запись — всё сохранилось.", id="podtverzhdayu_zapis"),
            pytest.param("Уже записал, не переживай.", id="uzhe_zapisal"),
            pytest.param("Спартак и пицца — уже в памяти.", id="uzhe_v_pamyati"),
        ],
    )
    def test_confirmation_wording_fires_without_write_tool(self, spoken: str) -> None:
        rule = detect_unbacked_action_claim(
            user_input=N206_USER, spoken=spoken, tools_called=("memory_context",),
        )
        assert rule is not None, f"confirmation-формулировка не поймана: {spoken!r}"
        assert rule.category == "fact_memory_save"

    @pytest.mark.parametrize(
        "spoken",
        [
            pytest.param("Всё на месте, запись подтверждена.", id="acceptance_exact"),
            pytest.param("Уже записал факт про Спартак.", id="uzhe_zapisal_2"),
            pytest.param("Спартак и пицца в памяти.", id="v_pamyati_2"),
        ],
    )
    def test_confirmation_wording_does_not_fire_with_write_tool(
        self, spoken: str
    ) -> None:
        """Тот же текст, но memory_save реально вызван — не баг."""
        assert (
            detect_unbacked_action_claim(
                user_input=N206_USER, spoken=spoken, tools_called=("memory_save",),
            )
            is None
        )


class TestExistingClaimsStillWork:
    """Осторожно: не сломать старое поведение #2755 (женские формы и т.п.)."""

    @pytest.mark.parametrize(
        "spoken",
        [
            "Запомнила, Борис: болеешь за Спартак и всегда приносишь пиццу.",
            "Записала, Саша: зелёный чай без сахара.",
            "Зафиксировал факт.",
            "Отметила в профиле.",
            "Информация сохранена.",
        ],
    )
    def test_pre_existing_claim_verbs_still_fire(self, spoken: str) -> None:
        rule = detect_unbacked_action_claim(
            user_input=N206_USER, spoken=spoken, tools_called=(),
        )
        assert rule is not None
        assert rule.category == "fact_memory_save"

    def test_honest_confession_is_not_a_claim(self) -> None:
        """Реплика 2 — честное признание сбоя, а НЕ новое ложное заявление."""
        rule = detect_unbacked_action_claim(
            user_input=N206_USER, spoken=N206_REPLY_2, tools_called=(),
        )
        assert rule is None or rule.category != "fact_memory_save"


class TestFallbackBuildingBlocks:
    """Guard-уровня примитивы для п.3 (fallback после потраченного ретрая).

    Сама проводка (``DialogueNode._handle_result`` должен подменять
    spoken этой констатацией, когда ``_action_claim_retry_used=True`` И
    :func:`spoken_matches_claim_category` вернул ``True``) остаётся за
    ``dialogue_node.py`` — здесь закреплены только чистые функции.
    """

    def test_matches_category_for_confirmation_wording(self) -> None:
        assert spoken_matches_claim_category("fact_memory_save", N206_REPLY_3)

    def test_does_not_match_unrelated_category(self) -> None:
        assert not spoken_matches_claim_category("music_prose_action", N206_REPLY_3)

    def test_unknown_category_is_safe_false(self) -> None:
        assert not spoken_matches_claim_category("no_such_category", N206_REPLY_3)

    def test_empty_spoken_is_safe_false(self) -> None:
        assert not spoken_matches_claim_category("fact_memory_save", "")
        assert not spoken_matches_claim_category("fact_memory_save", None)

    def test_fallback_text_makes_no_success_claim(self) -> None:
        fallback = build_fact_memory_save_fallback()
        assert fallback
        # Fallback не должен САМ попадать под тот же claim_re — иначе
        # цикл «guard ловит собственный fallback» повторится бесконечно.
        rule = next(
            r for r in ACTION_CLAIM_RULES if r.category == "fact_memory_save"
        )
        assert not rule.claim_re.search(fallback), (
            "fallback-текст не должен звучать как ещё одно подтверждение "
            "записи — иначе он сам попадёт под guard"
        )
