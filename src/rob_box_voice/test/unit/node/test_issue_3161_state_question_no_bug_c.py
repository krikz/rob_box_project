"""Issue #3161 — вопрос «какую музыку ты включал?» после стопа.

Живой прогон 28.09.2026 22:30–22:33 UTC (деплой develop 8888d6f02)::

    22:32:07  «Робот, выключи музыку» → роутер stop_music ✅
    22:32:20  «Робот, а какую музыку ты сейчас включал?»
    +4.4s  spoken='Сейчас играет клубный трек … он в фоне с прошлой просьбы.'
           Bug D babble → retry
    +8.4s  spoken='Минуту назад играл клубный трек …'
           Bug C «user asked for music but LLM skipped execute_music_code»
           → retry 1/3, 2/3 → budget
    +11.1s TTS «Я тут растерялся — бит не запустился, попробуй ещё раз.»

Три провала: (1) ``<music_state>`` из эвристики говорил «играет»;
(2) Bug C ретраил ВОПРОС как заказ музыки; (3) «бит не запустился» при
том, что в ходе не вызывалось ни одного музыкального тула.

STT: реальный Yandex работает с ``TEXT_NORMALIZATION_DISABLED`` — знака
«?» в реплике нет, поэтому здесь она без него (с ним — тоже проходит).
"""

from __future__ import annotations

import json
import logging
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.music_guard import MusicGuard, MusicGuardVerdictKind
from rob_box_voice.core.music_player_state import build_music_state_payload
from rob_box_voice.dialogue_node import DialogueNode

LIVE_USER_INPUT = "[Speaker:unknown] а какую музыку ты сейчас включал"
LIVE_SPOKEN_WRONG = "Сейчас играет клубный трек … он в фоне с прошлой просьбы."
LIVE_SPOKEN_RIGHT = "Минуту назад играл клубный трек …"

T_START = 1_790_635_800.0  # трек запущен
T_STOP = 1_790_635_927.0   # 22:32:07 — stop_music
T_ASK = 1_790_635_940.0    # 22:32:20 — вопрос


def _make_node(*, budget_left: bool):
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._music_guard = MusicGuard()
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._retry_dispatched_in_turn = False
    n._track_mode_music_active = False
    n._music_player_state = None
    n._consume_synthetic_retry = MagicMock(return_value=budget_left)
    n._discard_last_music_reply = MagicMock()
    n._speak_direct = MagicMock()
    n._mark_retry_dispatched = MagicMock()
    n._reopen_dialogue_for_retry = MagicMock()
    n._dispatch_dj_turn = MagicMock()
    n._dispatch_turn = MagicMock()
    n._build_music_retry_prompt = MagicMock(return_value="[CRITICAL] ...")
    n._build_dj_retry_prompt = MagicMock(return_value="[CRITICAL] ...")
    n._publish_music_cleanup = MagicMock()
    n._classify_music_user_input_kind = MagicMock(return_value="other")
    n._should_force_dj_off_for_stop_command = MagicMock(return_value=False)
    n._generated_music_state = None
    return n


def _stop_after_club_track(n) -> None:
    """Плеер: играл «клубный трек», в 22:32:07 остановлен (idle)."""
    n._on_music_state(SimpleNamespace(data=build_music_state_payload(
        playing=True, track_id="club-1", ts=T_START,
    )))
    n._on_music_form(SimpleNamespace(data=json.dumps({
        "form_ends_at": None, "playing": True, "stops_at": None,
        "track": "клубный трек",
    })))
    n._on_music_state(SimpleNamespace(data=build_music_state_payload(
        playing=False, ts=T_STOP,
    )))
    n._on_music_form(SimpleNamespace(data=json.dumps({
        "form_ends_at": None, "playing": False, "stops_at": None,
        "track": None,
    })))


def _spoken(n) -> list:
    return [c.args[0] for c in n._speak_direct.call_args_list]


class TestLiveScenario:
    def test_music_state_tag_says_stopped_and_what(self) -> None:
        n = _make_node(budget_left=True)
        _stop_after_club_track(n)
        tag = n._music_state_mem().render(now=T_ASK)
        assert 'playing="no"' in tag
        assert 'last_track="клубный трек"' in tag
        assert 'last_ended="stopped"' in tag
        assert 'last_ended_ago_s="13"' in tag
        assert n._music_playing_now() is False

    @pytest.mark.parametrize("spoken", [LIVE_SPOKEN_RIGHT, LIVE_SPOKEN_WRONG])
    @pytest.mark.parametrize(
        "user_input", [LIVE_USER_INPUT, LIVE_USER_INPUT + "?"]
    )
    def test_question_gets_no_bug_c_retry(self, spoken, user_input) -> None:
        n = _make_node(budget_left=True)
        _stop_after_club_track(n)
        dispatched = n._apply_music_guard(
            was_dj_auto=False,
            user_input=user_input,
            tools_called=(),
            spoken=spoken,
        )
        assert dispatched is False
        n._dispatch_turn.assert_not_called()
        n._discard_last_music_reply.assert_not_called()
        assert _spoken(n) == []

    def test_no_false_phrase_even_with_exhausted_budget(self) -> None:
        """Живьём бюджет выгорел — и прозвучало «бит не запустился»."""
        n = _make_node(budget_left=False)
        _stop_after_club_track(n)
        n._apply_music_guard(
            was_dj_auto=False,
            user_input=LIVE_USER_INPUT,
            tools_called=(),
            spoken=LIVE_SPOKEN_RIGHT,
        )
        said = _spoken(n)
        assert not any("бит не запустился" in t for t in said)
        assert not any("растерялся" in t for t in said)


# --- MusicGuard: границы исключения ----------------------------------------


def _guard() -> MusicGuard:
    return MusicGuard(logger=logging.getLogger("test_3161"))


class TestGuardBoundaries:
    def test_state_question_answered_in_words_is_skipped(self) -> None:
        verdict = _guard().evaluate(
            was_dj_auto=False,
            user_input=LIVE_USER_INPUT,
            tools_called=(),
            spoken=LIVE_SPOKEN_RIGHT,
        )
        assert verdict.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
        assert verdict.reason == "state_query_answered_in_words"

    def test_unknown_reply_keeps_old_behaviour(self) -> None:
        """``spoken=None`` — ответа не видно: как до #3161, ретрай."""
        verdict = _guard().evaluate(
            was_dj_auto=False,
            user_input=LIVE_USER_INPUT,
            tools_called=(),
            spoken=None,
        )
        assert verdict.kind is MusicGuardVerdictKind.USER_RETRY

    def test_real_music_request_still_retries(self) -> None:
        """Живой 1790620914: «сыграй клубный трек», tools=[] — Bug C."""
        verdict = _guard().evaluate(
            was_dj_auto=False,
            user_input="[Speaker:unknown] а теперь сыграй клубный трек",
            tools_called=(),
            spoken="Клубный трек заряжаю, поехали!",
        )
        assert verdict.kind is MusicGuardVerdictKind.USER_RETRY

    def test_question_with_launch_imperative_still_retries(self) -> None:
        """«включи музыку, какая сейчас модная» — команда, не вопрос.

        Форма вопроса («какая … музык») есть, но императив запуска
        (``MUSIC_STATE_QUERY_OVERRIDES``) снимает её — Bug C остаётся.
        """
        verdict = _guard().evaluate(
            was_dj_auto=False,
            user_input="включи музыку, какая сейчас модная",
            tools_called=(),
            spoken="Хорошо.",
        )
        assert verdict.kind is MusicGuardVerdictKind.USER_RETRY

    def test_phantom_music_claim_on_question_still_retries(self) -> None:
        """Ответ — заявление без тула по таблице Bug E (``music_state``).

        Такой ход в живом узле раньше перехватывает Bug E (ретрай с
        требованием ``get_music_state``), и музыкальный гуард молчит; сам
        гуард в этом случае ведёт себя как до #3161 — не пропускает.
        """
        verdict = _guard().evaluate(
            was_dj_auto=False,
            user_input="какая сейчас музыка играет",
            tools_called=(),
            spoken="Сейчас играет клубный трек.",
        )
        assert verdict.kind is MusicGuardVerdictKind.USER_RETRY
