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

Issue #3134: (a) и (b) больше не доходят до LLM и до ``MusicGuard``.
ADR-0150 PR-6 (#3696): громкость и её канал по снимку решает Tier-1
``rob_box_dialog.grammar`` (раньше — медиа-грамматика роутера). Здесь — что
эти живые строки действительно закрываются Tier-1 на канал музыки. (c) #3004 жил в
``MusicGuard`` и удалён вместе с ним (ADR-0149 PR-13a).
"""

from __future__ import annotations

import pytest

from rob_box_dialog.grammar import match
from rob_box_dialog.session import MusicStateRef, SessionSnapshot

from rob_box_voice.core.media_command_grammar import extract_user_utterance

# --- дословные строки из живого лога ---------------------------------------

_DJ_WRAPPER = (
    '[TG] [🎧 Музыкальный режим активен — фоновая музыка играет, тема: '
    '"клубная вечеринка", диджей: диджей Дайв. Это ОБЫЧНАЯ команда юзера, '
    'не DJ-переход. Ответь на неё нормально. Не вызывай set_dj_mode и не '
    'меняй музыку, если юзер об этом не просит.] '
)

#: 1790611272 handle_result (обрезано на хвосте backlog'а, как в логе).
LIVE_A_USER_INPUT = (
    _DJ_WRAPPER + "играй громче\n[ФОНОВЫЙ ЗАПРОС] До этого "
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


# --- детектор просьбы о громкости (теперь Tier-1 rob_box_dialog.grammar) ----

_PLAYING = SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0,
                           music=MusicStateRef("playing", title="Still Dre"))


def _volume(text: str):
    command = match(extract_user_utterance(text), _PLAYING)
    return command if command is not None and command.kind == "volume" else None


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
        cmd = _volume(text)
        assert cmd is not None and cmd.args["action"] in ("louder", "quieter")
        assert cmd.args["channel"] == "music"

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
        # Issue #3176: «сыграй бетховена» теперь заказ по имени
        # (PLAY_NAMED, решает база мелодий), но по-прежнему НЕ громкость.
        assert _volume(text) is None

    def test_extract_strips_tags_and_trailing_blocks(self) -> None:
        assert extract_user_utterance(LIVE_A_USER_INPUT) == "играй громче"
        assert (
            extract_user_utterance(LIVE_B_RETRY_USER_INPUT)
            == "играй трей кромче а не говори громче"
        )


class TestLiveAB_ClosedByTier1:
    """(a)/(b) — до LLM не доходят: set_music_volume вызывает код (Tier-1, канал музыки)."""

    @pytest.mark.parametrize(
        "text", ["[TG] играй громче", "[TG] играй трей кромче а не говори громче"]
    )
    def test_tier1_picks_music_channel(self, text: str) -> None:
        cmd = _volume(text)
        assert cmd is not None and dict(cmd.args) == {"action": "louder", "channel": "music"}

    def test_named_track_order_still_goes_to_llm(self) -> None:
        # «сыграй X погромче» — заказ трека, не громкость: Tier-1 его не берёт.
        assert _volume("сыграй в пещере гороного короля погромче") is None


# --- (c) #3004: ошибка валидации, затем успех в том же ходе -----------------


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
