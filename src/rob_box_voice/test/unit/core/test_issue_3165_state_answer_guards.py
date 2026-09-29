"""Issue #3165 — гуарды не наказывают ответ, согласный со снимком плеера.

Живой прогон 29.09.2026 00:10–00:13 UTC (деплой develop ``1652cbf7d``)::

    «Робот что сейчас играет» (снимок плеера: idle)
    spoken='Сейчас тишина — ничего не играет.' tools=[]
    Bug E: заявлено действие без тула (category=music_state)   ← ложно
    retry → 'Клубный трек, сто двадцать четыре…'               ← выдумка
    retry → 'Сейчас тишина, последний трек я остановил.'
    TTS: «Не получилось выполнить — попробуй, п…»              ← ложно

Здесь — чистые функции: сверка ``music_state`` со снимком
(:func:`detect_unbacked_action_claim`, :func:`is_phantom_music_action`,
``MusicGuard``) и фраза fallback'а #2949, когда в ходе ничего не
исполнялось. Узловой прогон того же сценария —
``test/test_issue_3165_live_state_answer.py``.
"""

from __future__ import annotations

import re

import pytest

from rob_box_voice.core.dialogue_guards import (
    ACTION_CLAIM_NOTHING_DONE_TEXT,
    ACTION_CLAIM_RULES,
    UniversalActionClaimHit,
    build_action_claim_failure_fallback,
    detect_unbacked_action_claim,
    is_phantom_music_action,
)
from rob_box_voice.core.music_guard import MusicGuard, MusicGuardVerdictKind

LIVE_QUESTION = "[Speaker:unknown] что сейчас играет"
LIVE_ANSWER = "Сейчас тишина — ничего не играет."


def _music_state_rule():
    return next(r for r in ACTION_CLAIM_RULES if r.category == "music_state")


class TestMusicStateAgreesWithSnapshot:
    """Bug E ``music_state``: снимок плеера подкрепляет согласный ответ."""

    def test_live_idle_answer_is_not_a_claim(self) -> None:
        assert detect_unbacked_action_claim(
            user_input=LIVE_QUESTION,
            spoken=LIVE_ANSWER,
            tools_called=(),
            music_playing=False,
        ) is None

    @pytest.mark.parametrize("spoken", [
        "Сейчас тишина, последний трек я остановил.",
        "Ничего не играет.",
        "Музыка не играет, минуту назад играл клубный трек.",
    ])
    def test_idle_answers_agree_with_idle_player(self, spoken) -> None:
        assert detect_unbacked_action_claim(
            user_input=LIVE_QUESTION, spoken=spoken, tools_called=(),
            music_playing=False,
        ) is None

    @pytest.mark.parametrize("spoken", [
        "Играет клубный трек.",
        "Сейчас звучит «Still Dre».",
    ])
    def test_playing_answers_agree_with_playing_player(self, spoken) -> None:
        assert detect_unbacked_action_claim(
            user_input=LIVE_QUESTION, spoken=spoken, tools_called=(),
            music_playing=True,
        ) is None

    def test_playing_claim_on_idle_player_still_fires(self) -> None:
        """Ложь против снимка: плеер молчит, модель говорит «играет»."""
        rule = detect_unbacked_action_claim(
            user_input=LIVE_QUESTION,
            spoken="Сейчас играет клубный трек.",
            tools_called=(),
            music_playing=False,
        )
        assert rule is not None and rule.category == "music_state"

    def test_silence_claim_on_playing_player_still_fires(self) -> None:
        rule = detect_unbacked_action_claim(
            user_input=LIVE_QUESTION,
            spoken=LIVE_ANSWER,
            tools_called=(),
            music_playing=True,
        )
        assert rule is not None and rule.category == "music_state"

    def test_unknown_snapshot_keeps_old_behaviour(self) -> None:
        """Снимка нет (``playing="unknown"``) — ответу не на что опереться."""
        rule = detect_unbacked_action_claim(
            user_input=LIVE_QUESTION, spoken=LIVE_ANSWER, tools_called=(),
        )
        assert rule is not None and rule.category == "music_state"

    def test_other_categories_ignore_snapshot(self) -> None:
        """Снимок плеера не подкрепляет «Точка сохранена» (не про музыку)."""
        rule = detect_unbacked_action_claim(
            user_input="запомни эту точку как кухня",
            spoken="Точка сохранена.",
            tools_called=(),
            music_playing=False,
        )
        assert rule is not None and rule.category == "waypoint_save"

    @pytest.mark.parametrize("spoken", [
        LIVE_ANSWER, "Ничего не играет.", "Не играет.", "Играет бит.",
        "Звучит бит.", "Музыка включена.", "Сейчас молчу.",
    ])
    def test_claim_re_matches_exactly_as_before(self, spoken) -> None:
        """Мораторий #3132: группы — те же альтернативы в том же порядке."""
        before = re.compile(
            r"тишин|ничего\s+не\s+игра|не\s+игра|игра\w*|звучит|включен",
            re.IGNORECASE,
        )
        now = _music_state_rule().claim_re.search(spoken)
        old = before.search(spoken)
        assert (now and now.span()) == (old and old.span())


class TestPhantomMusicAction:
    def test_agreeing_answer_is_not_phantom(self) -> None:
        assert is_phantom_music_action(
            user_input=LIVE_QUESTION, spoken=LIVE_ANSWER, tools_called=(),
            music_playing=False,
        ) is None

    def test_default_keeps_old_behaviour(self) -> None:
        assert is_phantom_music_action(
            user_input=LIVE_QUESTION, spoken=LIVE_ANSWER, tools_called=(),
        ) is not None


class TestMusicGuardUsesSnapshot:
    """``_state_query_answered_in_words`` (#3161) больше не режет «что играет»."""

    def test_state_question_with_agreeing_answer_skips(self) -> None:
        verdict = MusicGuard().evaluate(
            was_dj_auto=False,
            user_input="какая сейчас музыка играет",
            tools_called=(),
            spoken="Сейчас тишина, ничего не играет.",
            music_playing=False,
        )
        assert verdict.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
        assert verdict.reason == "state_query_answered_in_words"

    def test_state_question_with_lie_still_retries(self) -> None:
        verdict = MusicGuard().evaluate(
            was_dj_auto=False,
            user_input="какая сейчас музыка играет",
            tools_called=(),
            spoken="Сейчас играет клубный трек.",
            music_playing=False,
        )
        assert verdict.kind is MusicGuardVerdictKind.USER_RETRY


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
