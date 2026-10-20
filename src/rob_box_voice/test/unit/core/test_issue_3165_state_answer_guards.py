"""Issue #3165 — гуарды не наказывают ответ, согласный со снимком плеера.

Живой прогон 29.09.2026 00:10–00:13 UTC (деплой develop ``1652cbf7d``)::

    «Робот что сейчас играет» (снимок плеера: idle)
    spoken='Сейчас тишина — ничего не играет.' tools=[]
    Bug E: заявлено действие без тула (category=music_state)   ← ложно
    retry → 'Клубный трек, сто двадцать четыре…'               ← выдумка
    retry → 'Сейчас тишина, последний трек я остановил.'
    TTS: «Не получилось выполнить — попробуй, п…»              ← ложно

ADR-0149 PR-13a: правило Bug E ``music_state``, ``is_phantom_music_action`` и
``MusicGuard`` удалены — ответ о состоянии музыки модель берёт из
``<music_state>`` (снимок плеера), сверять его регексом больше не с чем.
Здесь — что живой ответ больше не ловится, и фраза fallback'а #2949, когда в
ходе ничего не исполнялось.
"""

from __future__ import annotations

from rob_box_voice.core.dialogue_guards import (
    ACTION_CLAIM_NOTHING_DONE_TEXT,
    ACTION_CLAIM_RULES,
    UniversalActionClaimHit,
    build_action_claim_failure_fallback,
    detect_unbacked_action_claim,
)

LIVE_QUESTION = "[Speaker:unknown] что сейчас играет"
LIVE_ANSWER = "Сейчас тишина — ничего не играет."


def test_music_rules_are_gone_from_bug_e() -> None:
    categories = {rule.category for rule in ACTION_CLAIM_RULES}
    assert not categories & {"music_state", "track_load", "music_prose_action"}


def test_live_state_answer_is_not_a_claim() -> None:
    assert detect_unbacked_action_claim(
        user_input=LIVE_QUESTION, spoken=LIVE_ANSWER, tools_called=()
    ) is None


class TestFallbackPhrase:
    """#2949 fallback: «Не получилось выполнить» — только после попытки."""

    HIT = UniversalActionClaimHit(verb="остановил", tense="past", excerpt="…")

    def test_nothing_attempted_is_not_a_failure(self) -> None:
        text = build_action_claim_failure_fallback(
            self.HIT, nothing_attempted=True
        )
        assert text == ACTION_CLAIM_NOTHING_DONE_TEXT
        assert "Не получилось" not in text

    def test_failed_attempt_keeps_honest_failure(self) -> None:
        text = build_action_claim_failure_fallback(self.HIT)
        assert text.startswith("Не получилось выполнить")
