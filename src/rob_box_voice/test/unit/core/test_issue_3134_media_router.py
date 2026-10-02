"""Issue #3134 — роутер медиакоманд до LLM: грамматика, план, DecisionProvider.

Чистые части без ROS: :mod:`rob_box_voice.core.media_command_grammar`
(закрытая грамматика, перенесённая из гуардов #3125/#2999/#2834) и
:mod:`rob_box_voice.core.media_router` (интент за ``DecisionProvider``,
план тулов и фраз по состоянию плеера). Адаптер ноды — в
``test/unit/node/test_issue_3134_media_router_node.py``.
"""

from __future__ import annotations

import asyncio

import pytest

from rob_box_harness.decision import (
    ChoiceAnswer,
    DecisionProvider,
    DeterministicProvider,
    FakeDecisionProvider,
)

from rob_box_voice.core.media_command_grammar import (
    MediaIntent,
    is_music_stop_command,
    parse_media_command,
)
from rob_box_voice.core.media_router import (
    NOTHING_PLAYING_TEXT,
    MediaRouter,
    MediaState,
    default_media_provider,
    media_intent_rules,
)

PLAYING = MediaState(music_playing=True, track_name="В пещере горного короля")
QUIET = MediaState()
DJ_SET = MediaState(music_playing=True, dj_enabled=True, track_name="Still Dre")

STATES = {"playing": PLAYING, "quiet": QUIET, "dj": DJ_SET}


def _calls(plan):
    return [(c.name, c.arguments) for c in plan.tool_calls]


# ── 1. Грамматика ──────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("text", "intent"),
    [
        ("играй громче", MediaIntent.VOLUME_UP),
        ("погромче", MediaIntent.VOLUME_UP),
        ("[TG] сделай музыку громче", MediaIntent.VOLUME_UP),
        ("прибавь звук", MediaIntent.VOLUME_UP),
        ("ты диджей, сделай громче", MediaIntent.VOLUME_UP),
        ("потише", MediaIntent.VOLUME_DOWN),
        ("сделай музыку потише пожалуйста", MediaIntent.VOLUME_DOWN),
        ("убавь музыку", MediaIntent.VOLUME_DOWN),
        ("на максимум", MediaIntent.VOLUME_MAX),
        ("громкость на максимум", MediaIntent.VOLUME_MAX),
        ("выкрути музыку на полную", MediaIntent.VOLUME_MAX),
        ("выключи музыку", MediaIntent.STOP),
        ("стоп диджей", MediaIntent.STOP),
        ("хватит диджеить", MediaIntent.STOP),
        ("выключи режим диджея", MediaIntent.STOP),
        ("останови трек пожалуйста", MediaIntent.STOP),
        ("ты диджей Снупдог, давай сет", MediaIntent.DJ),
        ("запусти диджей-сет", MediaIntent.DJ),
    ],
)
def test_grammar_closed_commands(text: str, intent: MediaIntent) -> None:
    cmd = parse_media_command(text)
    assert cmd.intent is intent
    assert cmd.closed is True


@pytest.mark.parametrize(
    "text",
    [
        "",
        "сыграй в пещере горного короля погромче",  # заказ трека
        "расскажи анекдот погромче",  # не про музыку
        "говори громче",  # голос робота — set_volume, решает LLM
        "говори потише",
        "слишком громко",  # направление неясно
        "не так громко",
        "стоп",  # движение (command-гейт), не музыка
        "замолчи",
        "выключи музыку и включи Oakenfold",  # не закрытый стоп
        "стоп диджей блядь",
        "ты диджей?",
        "кто такой диджей",
        "у тебя сейчас играет музыка?",
    ],
)
def test_grammar_not_closed_command(text: str) -> None:
    assert parse_media_command(text).intent is MediaIntent.NONE


@pytest.mark.parametrize("text", ["сыграй трек", "включи музыку"])
def test_generic_music_request_goes_to_request_music(text: str) -> None:
    """ADR-0149 PR-6: родовой заказ — ``REQUEST_MUSIC`` → ``request_music`` движка v2."""
    assert parse_media_command(text).intent is MediaIntent.REQUEST_MUSIC
    assert [c.name for c in MediaRouter().route(text, QUIET).tool_calls] == ["request_music"]


def test_track_name_tail_needs_current_track() -> None:
    text = "горный король погромче"
    assert parse_media_command(text).intent is MediaIntent.NONE
    assert (
        parse_media_command(text, track_name="Still Dre").intent is MediaIntent.NONE
    )
    cmd = parse_media_command(text, track_name="В пещере горного короля")
    assert cmd.intent is MediaIntent.VOLUME_UP and cmd.closed


def test_live_3125_voice_negation_is_music_volume() -> None:
    # «играй трей кромче а не говори громче» — отказ от голоса, трек громче.
    cmd = parse_media_command("играй трей кромче а не говори громче")
    assert cmd.intent is MediaIntent.VOLUME_UP


@pytest.mark.parametrize(
    ("text", "persona", "theme", "closed"),
    [
        ("ты диджей Снупдог, давай сет", "диджей Снупдог", "", True),
        # STT без запятых: «давай сет» — не часть имени.
        ("ты диджей Снупдог давай сет", "диджей Снупдог", "", True),
        (
            "[TG] Ты диджей Снупдог и у нас сегодня вечеринка ганкста в чорном квартале",
            "диджей Снупдог",
            "вечеринка ганкста в чорном квартале",
            True,
        ),
        ("стань диджеем", "", "", True),
        (
            "Ты диджей PAUL OAKENFOLD, сыграй Still Dre и Next Episode на 10 минут",
            "диджей PAUL OAKENFOLD",
            "",
            False,
        ),
    ],
)
def test_dj_grammar(text: str, persona: str, theme: str, closed: bool) -> None:
    cmd = parse_media_command(text)
    assert cmd.intent is MediaIntent.DJ
    assert (cmd.persona, cmd.theme, cmd.closed) == (persona, theme, closed)


def test_wide_stop_detector_still_catches_mixed_phrases() -> None:
    # Силенс-гейт, command-гейт и Bug F пользуются широким детектором.
    assert is_music_stop_command("выключи музыку и включи Oakenfold")
    assert is_music_stop_command("стоп диджей блядь")
    assert not is_music_stop_command("для робота-диджея")


# ── 2. План: каждая команда × (играет / тишина / DJ) ────────────────────


@pytest.mark.parametrize(
    ("text", "action"),
    [
        ("играй громче", "louder"),
        ("потише", "quieter"),
        ("на максимум", "max"),
    ],
)
@pytest.mark.parametrize("state", ["playing", "quiet", "dj"])
def test_volume_plan(text: str, action: str, state: str) -> None:
    plan = MediaRouter().route(text, STATES[state])
    assert plan is not None
    if state == "quiet":
        assert plan.tool_calls == ()
        assert plan.say_ok == NOTHING_PLAYING_TEXT
    else:
        assert _calls(plan) == [("set_music_volume", {"action": action})]
        assert plan.say_ok and plan.say_fail
    assert not plan.cancel_inflight  # громкость не рвёт идущий ответ


def test_track_name_volume_plan_only_while_that_track_plays() -> None:
    router = MediaRouter()
    plan = router.route("горный король погромче", PLAYING)
    assert _calls(plan) == [("set_music_volume", {"action": "louder"})]
    assert router.route("горный король погромче", QUIET) is None
    assert router.route("горный король погромче", DJ_SET) is None


@pytest.mark.parametrize("state", ["playing", "quiet", "dj"])
def test_stop_plan(state: str) -> None:
    plan = MediaRouter().route("выключи музыку", STATES[state])
    assert plan is not None
    assert _calls(plan) == [("dj_set", {"action": "stop"}), ("stop_music", {})]
    assert plan.cancel_inflight
    assert ("ничего не играет" in plan.say_ok) is (state == "quiet")


@pytest.mark.parametrize("state", ["playing", "quiet", "dj"])
def test_dj_plan_starts_engine_set(state: str) -> None:
    # PR-13a: ни превью compose_music, ни set_dj_mode — сет ведёт движок v2.
    plan = MediaRouter().route("ты диджей Снупдог, давай сет", STATES[state])
    assert _calls(plan) == [("dj_set", {"action": "start", "persona": "диджей Снупдог"})]
    assert plan.confirm_started and "Снупдог" in plan.say_ok


def test_open_dj_request_goes_to_llm() -> None:
    plan = MediaRouter().route(
        "Ты диджей PAUL OAKENFOLD, сыграй Still Dre и Next Episode на 10 минут", QUIET
    )
    assert plan is None


@pytest.mark.parametrize("state", ["playing", "quiet", "dj"])
def test_play_request_is_play_named(state: str) -> None:
    # Issue #3176: заказ по имени забирает роутер; есть ли мелодия, решает
    # база (нода зовёт lookup_melody), промах возвращает реплику в LLM —
    # см. test_issue_3176_play_named*.py.
    plan = MediaRouter().route("сыграй в пещере горного короля", STATES[state])
    assert plan is not None
    assert plan.play_name == "в пещере горного короля"
    assert plan.tool_calls == ()


# ── 3. DecisionProvider ────────────────────────────────────────────────


def test_default_provider_is_deterministic_decision_provider() -> None:
    provider = default_media_provider()
    assert isinstance(provider, DeterministicProvider)
    assert isinstance(provider, DecisionProvider)
    assert MediaRouter().provider_name == "deterministic"


def test_rules_answer_closed_choice() -> None:
    answers = media_intent_rules({"utterance": "потише"}, {"media_intent": object()})
    answer = answers["media_intent"]
    assert isinstance(answer, ChoiceAnswer)
    assert answer.answer == MediaIntent.VOLUME_DOWN.value
    assert answer.confidence == 1.0


def test_provider_decides_intent() -> None:
    # Провайдер говорит «не медиакоманда» — роутер уважает решение.
    none = ChoiceAnswer(answer="none", probabilities={"none": 1.0}, confidence=1.0)
    router = MediaRouter(FakeDecisionProvider("fake", answers={"media_intent": none}))
    assert router.route("играй громче", PLAYING) is None


def test_provider_found_volume_grammar_did_not() -> None:
    up = ChoiceAnswer(
        answer="volume_up", probabilities={"volume_up": 1.0}, confidence=1.0
    )
    router = MediaRouter(FakeDecisionProvider("fake", answers={"media_intent": up}))
    plan = router.route("чуть бодрее звук", PLAYING)
    assert _calls(plan) == [("set_music_volume", {"action": "louder"})]


def test_awaiting_provider_falls_back_to_grammar() -> None:
    # Сетевой провайдер в синхронном колбэке STT не ждём.
    slow = FakeDecisionProvider("jev", answers={}, delay_s=0.5)
    plan = MediaRouter(slow).route("потише", PLAYING)
    assert _calls(plan) == [("set_music_volume", {"action": "quieter"})]
    assert slow.calls == 1


def test_deterministic_provider_async_contract() -> None:
    decision = asyncio.run(
        default_media_provider().decide(
            {"utterance": "выключи музыку"}, {"media_intent": object()}
        )
    )
    assert decision.provider == "deterministic"
    assert decision.answers["media_intent"].answer == "stop"
