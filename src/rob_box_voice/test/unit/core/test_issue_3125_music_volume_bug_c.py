"""Issue #3125 (+ остаток #3004) — «играй громче» при играющей музыке.

Живой сет 28.09.2026, ``docker logs voice-assistant`` (develop после #3121).
Все ``user_input`` / ``tools`` ниже — дословно из лога:

(a) ``1790611272`` — «играй громче», ``tools=['set_volume']``, трек играет
    (``[track-mode] TRACK играет с прошлого хода``) → Bug C retry 1/3.
(b) ``1790611314..319`` — phantom-claim «Подкручиваю музыку на максимум» при
    ``tools=[]`` → Bug C 1/3, 2/3 → retry-budget → «Я тут растерялся — бит не
    запустился», хотя бит играл.
(c) ``1790611417..429`` — ``compose_music`` с ``levels: lead=1.3`` отвергнут,
    повтор в том же ходе успешен, но правило #2966 (``tool_error_occurred``)
    сочло ход провалом → Bug C.
"""

from __future__ import annotations

import logging
import pytest

from rob_box_voice.core.music_guard import MusicGuard, MusicGuardVerdictKind
from rob_box_voice.core.music_volume_request import (
    extract_user_utterance,
    is_music_volume_request,
)

# --- дословные строки из живого лога ---------------------------------------

_DJ_WRAPPER = (
    '[TG] [🎧 Музыкальный режим активен — фоновая музыка играет, тема: '
    '"клубная вечеринка", диджей: диджей Дайв. Это ОБЫЧНАЯ команда юзера, '
    'не DJ-переход. Ответь на неё нормально. Не вызывай set_dj_mode и не '
    'меняй музыку, если юзер об этом не просит.] '
)

#: 1790611272 handle_result (обрезано на хвосте backlog'а, как в логе).
LIVE_A_USER_INPUT = (
    _DJ_WRAPPER + "играй громче\n[URGENT_BACKLOG] [ФОНОВЫЙ ЗАПРОС] До этого "
    "обращения (без wake-слова) прозвучало:\n- незнакомец: «алло»\nВАЖНО: "
    "backlog ИМЕЕТ ПРИОРИТЕТ над историей диалога."
)
LIVE_A_TOOLS = ("set_volume",)

#: 1790611314 — вторая попытка юзера и её phantom-ретрай (#2559).
LIVE_B_USER_INPUT = _DJ_WRAPPER + "играй трей кромче а не говори громче"
LIVE_B_RETRY_USER_INPUT = (
    "[Speaker:unknown] играй трей кромче а не говори громче\n\n[CRITICAL] "
    "Твой предыдущий ответ содержал ОБЕЩАНИЕ ДЕЙСТВИЯ (слова «сделал / "
    "запустил / перезапущу / установлю / остановлю / проверю / подложу / "
    "переключу / подкручу / обновлю / поменяю / изменю / доработаю» и т.п.), "
    "но ты НЕ вызвал НИ ОДНОГО инструмента"
)
LIVE_B_SPOKEN = (
    "Йо, понял — трек громче, не голос! Подкручиваю музыку на максимум, "
    "чтоб стены тряслись!"
)

#: 1790611429 handle_result.
LIVE_C_USER_INPUT = _DJ_WRAPPER + "сыграй в пещере гороного короля погромче"
LIVE_C_TOOLS = ("lookup_melody", "compose_music")


def _guard() -> MusicGuard:
    return MusicGuard(logger=logging.getLogger("test_3125"))


# --- детектор просьбы о громкости ------------------------------------------


class TestVolumeRequestDetector:
    @pytest.mark.parametrize(
        "text",
        [
            LIVE_A_USER_INPUT,
            LIVE_B_USER_INPUT,
            LIVE_B_RETRY_USER_INPUT,
            "[Speaker:unknown] играй громче",
            "сделай музыку потише",
            "погромче",
            "тише музыку",
            "прибавь звук",
        ],
    )
    def test_pure_volume_requests(self, text: str) -> None:
        assert is_music_volume_request(text)

    @pytest.mark.parametrize(
        "text",
        [
            LIVE_C_USER_INPUT,  # трек назван → это заказ, не громкость
            "сыграй бетховена",
            _DJ_WRAPPER + "сыграй бетховена",
            "играй",
            "",
        ],
    )
    def test_not_volume_requests(self, text: str) -> None:
        assert not is_music_volume_request(text)

    def test_extract_strips_tags_and_trailing_blocks(self) -> None:
        assert extract_user_utterance(LIVE_A_USER_INPUT) == "играй громче"
        assert (
            extract_user_utterance(LIVE_B_RETRY_USER_INPUT)
            == "играй трей кромче а не говори громче"
        )


# --- (a) «играй громче» + set_volume при играющем треке ---------------------


class TestLiveA_PlayLouderWithSetVolume:
    def test_no_bug_c_retry_when_track_playing(self) -> None:
        v = _guard().evaluate(
            was_dj_auto=False,
            user_input=LIVE_A_USER_INPUT,
            tools_called=LIVE_A_TOOLS,
            spoken="Громче, громче! Танцпол не слышит!",
            music_playing=True,
        )
        assert v.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
        assert v.reason == "volume_adjust_while_playing"

    def test_set_music_volume_satisfies_turn_even_without_playing_flag(self) -> None:
        v = _guard().evaluate(
            was_dj_auto=False,
            user_input=LIVE_A_USER_INPUT,
            tools_called=("set_music_volume",),
            music_playing=False,
        )
        assert v.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
        assert v.reason == "music_volume_set"

    def test_legacy_retry_kept_when_nothing_plays(self) -> None:
        # Музыки нет и тула громкости музыки нет — Bug C как раньше.
        v = _guard().evaluate(
            was_dj_auto=False,
            user_input=LIVE_A_USER_INPUT,
            tools_called=LIVE_A_TOOLS,
            music_playing=False,
        )
        assert v.kind is MusicGuardVerdictKind.USER_RETRY


# --- (b) phantom-claim + tools=[] при играющем треке ------------------------


class TestLiveB_PhantomClaimNoTools:
    @pytest.mark.parametrize("user_input", [LIVE_B_USER_INPUT, LIVE_B_RETRY_USER_INPUT])
    def test_no_bug_c_retry_chain(self, user_input: str) -> None:
        g = _guard()
        for _ in range(3):  # живьём было 1/3, 2/3, затем budget → nudge
            v = g.evaluate(
                was_dj_auto=False,
                user_input=user_input,
                tools_called=(),
                spoken=LIVE_B_SPOKEN,
                music_playing=True,
            )
            assert v.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
        assert g.user_retry_count == 0

    def test_named_track_request_still_retried(self) -> None:
        # «сыграй X погромче» без тулов — это заказ трека, Bug C обязан ретраить.
        v = _guard().evaluate(
            was_dj_auto=False,
            user_input=LIVE_C_USER_INPUT,
            tools_called=(),
            music_playing=True,
        )
        assert v.kind is MusicGuardVerdictKind.USER_RETRY


# --- (c) #3004: ошибка валидации, затем успех в том же ходе -----------------


class TestLiveC_ErrorThenSuccessSameTurn:
    def test_success_after_validation_error_is_success(self) -> None:
        v = _guard().evaluate(
            was_dj_auto=False,
            user_input=LIVE_C_USER_INPUT,
            tools_called=LIVE_C_TOOLS,
            tool_error_occurred=True,
            succeeded_tools=("lookup_melody", "compose_music"),
            music_playing=True,
        )
        assert v.kind is MusicGuardVerdictKind.SKIP
        assert v.reason == "executed"

    def test_only_failed_music_call_still_not_success(self) -> None:
        # #2966 сохранён: музыкальный тул упал, успешен только lookup_melody.
        v = _guard().evaluate(
            was_dj_auto=False,
            user_input=LIVE_C_USER_INPUT,
            tools_called=LIVE_C_TOOLS,
            tool_error_occurred=True,
            succeeded_tools=("lookup_melody",),
        )
        assert v.kind is not MusicGuardVerdictKind.SKIP

    def test_unknown_per_call_info_keeps_legacy_2966(self) -> None:
        v = _guard().evaluate(
            was_dj_auto=False,
            user_input=LIVE_C_USER_INPUT,
            tools_called=LIVE_C_TOOLS,
            tool_error_occurred=True,
        )
        assert v.kind is MusicGuardVerdictKind.USER_RETRY


# --- честный claim после set_music_volume не ловится как phantom -----------


class TestHonestVolumeClaimIsBacked:
    def test_claim_backed_by_set_music_volume(self) -> None:
        from rob_box_voice.core.dialogue_guards import detect_universal_action_claim

        spoken = "Сделал трек громче, качаем дальше!"
        assert detect_universal_action_claim(spoken=spoken, tools_called=()) is not None
        assert (
            detect_universal_action_claim(spoken=spoken, tools_called=("set_music_volume",))
            is None
        )
