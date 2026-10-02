"""Issue #2874 — ход, за которым следует синхронный ретрай, молчит.

Живой прогон 23.09.2026 17:30 (DJ «Снупдог»): music-гуард Bug C трижды
отправлял ретрай, а каждый ход до ретрая уходил в TTS — три разные фразы
подряд; на исчерпании retry-budget (#1881) сырой ответ ушёл целиком
(11 TTS-чанков, ~40 с монолога).

Прогоняем настоящий ``_run_turn`` + tool-skipped гуард (музыкальный Bug C
удалён в ADR-0149 PR-13a) и считаем, что реально ушло в TTS.
``_handle_result`` заменён хвостом, который зовёт ``_voice_turn_text`` —
ровно так заканчивается настоящий ``_handle_result``.

DialogueNode собирается через ``object.__new__`` (как в
test_new_session_resets_dj.py), rclpy замокан в conftest.
"""

from __future__ import annotations

import asyncio
import threading
from collections import deque
from unittest.mock import AsyncMock, MagicMock, patch

from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.dialogue_node import DialogueNode

_USER = (
    "Ты диджей Снупдог, у нас гангста-вечеринка на 10 минут. "
    "Играй по очереди: Still Dre, Next Episode, Drop It Like It's Hot"
)
_MONOLOGUE = "Поехали, зал — Still Dre открывает вечер. " + " ".join(
    f"Но чё, gang, я ж DJ на этом железе, фраза {i}, у меня нет "
    "mp3-библиотеки хип-хопа и лицензированных битов Dr. Dre."
    for i in range(12)
)


class _Result:
    def __init__(self, spoken, tools=()):
        self.spoken_text = spoken
        self.tools_called = tuple(tools)
        self.error = None


def _make_node(replies):
    n = object.__new__(DialogueNode)
    n._task_lock = threading.Lock()
    n._run_cancelled = False
    n._pending_music_cleanup = False
    n._track_mode_music_active = False
    n._speaker_id_enabled = False
    n._handle_speaker_turn = MagicMock()
    n._apply_speaker_identity = MagicMock()
    n._build_dynamic_system_context = MagicMock(return_value="<ctx/>")
    n._llm = MagicMock()
    n._active_batches = {}
    n._dsm = MagicMock()
    n._dsm.current_state = DialogueStateKind.DIALOGUE
    n._publish_state = MagicMock()
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._maybe_record_session_end = MagicMock()
    n.get_logger = lambda: MagicMock()
    n._core = MagicMock()
    n._core.process_input = AsyncMock(side_effect=list(replies))
    n._dispatch_turn = MagicMock()
    n._reopen_dialogue_for_retry = MagicMock()
    n._pending_user_messages = deque()
    n._loop = MagicMock()
    # Всё, что звучит: авто-TTS ответа хода + прямые фразы гуардов.
    n._tts = MagicMock()
    n._publish_response = n._tts.response
    n._publish_response_batch = n._tts.batch
    n._publish_response_batch.side_effect = lambda chunks, *a, **k: len(chunks)
    n._speak_direct = n._tts.direct
    n._publish_music_cleanup = n._tts.cleanup

    def _tail(result, **kw):
        n._voice_turn_text(
            result.spoken_text,
            user_input=kw.get("raw_user_command") or kw.get("user_input"),
        )

    n._handle_result = MagicMock(side_effect=_tail)
    return n


def _voiced(n):
    return [
        c for c in n._tts.mock_calls
        if c[0] in ("response", "batch", "direct")
    ]


def _run(n, text, **kw):
    with patch(
        "rob_box_voice.dialogue_node.asyncio.run_coroutine_threadsafe",
        return_value=MagicMock(),
    ):
        asyncio.run(n._run_turn(text, session_epoch=0, **kw))


def _run_chain(n, text=_USER):
    """Юзерский ход + ретраи, которые ход задиспатчил (как в проде)."""
    _run(n, text)
    while n._dispatch_turn.call_count:
        call = n._dispatch_turn.call_args
        n._dispatch_turn.reset_mock()
        _run(
            n, call.args[0],
            is_synthetic=True,
            raw_user_command=call.kwargs.get("raw_user_command"),
        )


class TestRetryChainIsSilent:
    """Ход с синхронным ретраем молчит, звучит только итог цепочки.

    ADR-0149 PR-13a: музыкальный Bug C (``MusicGuard``), на котором был снят
    живой лог, удалён; механизм «придержать ответ до решения post-turn
    гуардов» тот же для tool-skipped гуарда (#1777).
    """

    def _real_tool_guard(self, n):
        del n._apply_tool_skipped_guard  # настоящий гуард
        n._tool_retry_used = False

    def test_turn_with_retry_is_silent_and_retry_answer_is_voiced(self):
        n = _make_node([
            _Result("Сейчас около трёх часов."),  # время из головы, без тула
            _Result("Сейчас пятнадцать ноль семь.", ["get_current_time"]),
        ])
        self._real_tool_guard(n)

        _run_chain(n, "который час")

        assert n._core.process_input.await_count == 2
        n._publish_response.assert_called_once_with("Сейчас пятнадцать ноль семь.")
        assert "около трёх" not in str(_voiced(n))

    def test_exhausted_chain_does_not_speak_raw_monologue(self):
        n = _make_node([_Result(_MONOLOGUE)])
        self._real_tool_guard(n)
        n._tool_retry_used = True  # ретрай уже потрачен — гуард молчит
        n._retry_budget_exhausted_in_turn = True

        _run(n, "который час")

        n._publish_response_batch.assert_not_called()


class TestNormalTurnUnchanged:
    def test_plain_chat_turn_voiced_once(self):
        n = _make_node([_Result("Всё отлично, спасибо!")])

        _run(n, "как дела")

        n._publish_response.assert_called_once_with("Всё отлично, спасибо!")
        n._dispatch_turn.assert_not_called()
        assert n._turn_speech_hold is None
        assert n._active_batches == {}

    def test_direct_handle_result_path_publishes_immediately(self):
        """Вне ``_run_turn`` (нет открытого hold) — как раньше, сразу."""
        n = _make_node([])
        n._turn_speech_hold = None

        n._voice_turn_text("Привет!", user_input="привет")

        n._publish_response.assert_called_once_with("Привет!")

    def test_stop_music_cleanup_fires_after_held_reply(self):
        """Придержанный ответ не даёт погасить музыку раньше, чем он прозвучит."""
        n = _make_node([_Result("Выключаю.", ["stop_music"])])

        _run(n, "выключи музыку")

        names = [c[0] for c in n._tts.mock_calls]
        assert names.index("response") < names.index("cleanup")
        assert n._active_batches == {}


class TestBudgetExhaustedAndMusicTrim:
    def test_budget_exhausted_speaks_first_sentence(self):
        n = _make_node([])
        n._turn_speech_hold = None
        n._retry_budget_exhausted_in_turn = True

        n._voice_turn_text(_MONOLOGUE, user_input="расскажи что-нибудь")

        n._publish_response.assert_called_once_with(
            "Поехали, зал — Still Dre открывает вечер."
        )
        n._publish_response_batch.assert_not_called()

    def test_long_monologue_in_music_context_trimmed(self):
        n = _make_node([])
        n._turn_speech_hold = None
        # Музыкальный контекст — по снимку плеера (идёт DJ-сет движка v2).
        n._music_player_state = MusicPlayerState(state="playing", dj=True)

        n._voice_turn_text(_MONOLOGUE, user_input=_USER)

        n._publish_response.assert_called_once_with(
            "Поехали, зал — Still Dre открывает вечер."
        )

    def test_long_rap_on_request_stays_full(self):
        n = _make_node([])
        n._turn_speech_hold = None

        n._voice_turn_text(_MONOLOGUE, user_input="зачитай рэп про диджея")

        n._publish_response_batch.assert_called_once()
