"""Issue #3176 — заказ мелодии по имени исполняет роутер медиакоманд.

Живой прогон 29.09.2026 05:54: «Робот, поставь к Элизе» → модель сказала
«Ставлю «К Элизе», погнали» с ``tools=[]`` — заказ потерян. ``tool_choice``
у MiniMax не работает (ADR-0143), поэтому заказ по имени разбирает код:

1. грамматика (:mod:`media_command_grammar`) — по словам, без регексов
   (мораторий #3132): глагол заказа + название; родовые слова — не заказ;
2. план роутера (:mod:`media_router`) — ``play_name``;
3. поток (:mod:`named_play`) — ``lookup_melody`` → полное совпадение →
   ``request_music`` движка v2 (PR-11; ``compose_music`` удалён в PR-13a); промах — реплика в LLM;
4. приём (:class:`SttAdmission.resume_after`) — промах возвращается в
   шаги после ``MediaCommandStep``.
"""

from __future__ import annotations

import asyncio
from typing import Any, Dict, List, Tuple
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState
from rob_box_voice.core.named_play import (
    NamedPlayStatus,
    melody_hit,
    play_ok_text,
    run_named_play,
    tool_data,
)
from rob_box_voice.core.stt_admission import (
    PASS,
    SttAdmission,
    SttContext,
    SttOutcomeKind,
    handled,
)

# ── 1. Грамматика ──────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("text", "name"),
    [
        ("поставь к элизе", "к элизе"),
        ("Поставь к Элизе", "к элизе"),
        ("сыграй в пещере горного короля", "в пещере горного короля"),
        ("включи тетрис", "тетрис"),
        ("врубай имперский марш", "имперский марш"),
        ("заведи марио", "марио"),
        ("давай к элизе", "к элизе"),
        ("ну давай сыграй тетрис", "тетрис"),
        ("можешь поставить имперский марш", "имперский марш"),
        ("поставь-ка марио пожалуйста", "марио"),
        ("поставь песню к элизе", "к элизе"),  # родовое слово срезано
        ("поставь мне jingle bells", "jingle bells"),
        ("[TG] поставь тетрис", "тетрис"),
    ],
)
def test_play_named_grammar(text: str, name: str) -> None:
    cmd = parse_media_command(text)
    assert cmd.intent is MediaIntent.PLAY_NAMED
    assert cmd.name == name
    assert cmd.closed


@pytest.mark.parametrize(
    "text",
    [
        # общие слова — LLM или превью, как раньше
        "сыграй что-нибудь весёлое",
        "поставь музыку",
        "включи музыку",
        "сыграй трек",
        "включи клубный трек",
        "поставь бит",
        "сыграй песню",
        "поставь весёлую песню",
        "включи рок",
        # условие / контекст — решает LLM
        "сыграй песню про зайчиков",
        "включи его снова",
        "поставь что-то в стиле dr dre",
        "давай поговорим",
        "поставь на паузу",
        "поставь таймер",
        "не ставь к элизе",
        # громкость в хвосте — заказ с модификатором, решает LLM (#3125)
        "сыграй в пещере горного короля погромче",
        # не заказ вовсе
        "расскажи анекдот",
        "что сейчас играет",
    ],
)
def test_not_play_named(text: str) -> None:
    assert parse_media_command(text).intent is not MediaIntent.PLAY_NAMED


@pytest.mark.parametrize(
    ("text", "intent"),
    [
        ("выключи музыку", MediaIntent.STOP),
        ("играй громче", MediaIntent.VOLUME_UP),
        ("включи погромче", MediaIntent.VOLUME_UP),
        ("ты диджей Снупдог", MediaIntent.DJ),
        ("включи режим диджея", MediaIntent.DJ),
    ],
)
def test_older_commands_win_over_play_named(text: str, intent: MediaIntent) -> None:
    assert parse_media_command(text).intent is intent


# ── 2. План роутера ────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "media",
    [
        MediaState(),
        MediaState(music_playing=True, track_name="Still Dre"),
        MediaState(music_playing=True, dj_enabled=True, track_name="Still Dre"),
    ],
)
def test_router_plan_is_play_named_in_any_state(media: MediaState) -> None:
    plan = MediaRouter().route("поставь к элизе", media)
    assert plan is not None
    assert plan.play_name == "к элизе"
    assert plan.tool_calls == ()
    assert not plan.cancel_inflight  # отмена — только если мелодия нашлась
    assert plan.say_ok == ""  # фраза — от потока, по найденной записи


def test_generic_request_is_not_routed() -> None:
    assert MediaRouter().route("сыграй что-нибудь весёлое", MediaState()) is None


# ── 3. Поток lookup → compose ──────────────────────────────────────────

_FUR_ELISE = {
    "name": "furelise",
    "title": "Fur Elise",
    "display_title": "Fur Elise",
    "rtttl": "x:d=4:c",
    "match": {"matched": ["fur", "elise"], "unmatched": [], "coverage": 1.0, "ignored": []},
}


def _content(data: Dict[str, Any], message: str = "Нашёл «Fur Elise».") -> str:
    """Как ``core_adapter._result_content``: message + repr(data)."""
    return f"{message}\n{data!r}"


class _Tools:
    """Скриптованный ``execute(name, args) -> (ok, content)``."""

    def __init__(self, answers: Dict[str, List[Tuple[bool, str]]]) -> None:
        self.answers = {k: list(v) for k, v in answers.items()}
        self.calls: List[Tuple[str, Dict[str, Any]]] = []

    async def __call__(self, name: str, args: Dict[str, Any]) -> Tuple[bool, str]:
        self.calls.append((name, dict(args)))
        return self.answers[name].pop(0)


def _run(tools: _Tools, name: str):
    return asyncio.new_event_loop().run_until_complete(run_named_play(tools, name))


def test_found_melody_is_played_by_request_music() -> None:
    tools = _Tools({
        "lookup_melody": [(True, _content(_FUR_ELISE))],
        "request_music": [(True, "{'ok': True, 'track_id': 'mel:01:A:aa'}")],
    })
    out = _run(tools, "к элизе")
    assert out.status is NamedPlayStatus.PLAYED
    assert out.hit.title == "Fur Elise"
    assert tools.calls == [
        ("lookup_melody", {"name": "к элизе"}),
        ("request_music", {"intent": "melody", "text": "к элизе"}),
    ]
    assert out.tools_done == ("lookup_melody", "request_music")
    assert play_ok_text(out.hit.title) == "Ставлю «Fur Elise»."


def test_not_found_is_miss_without_compose() -> None:
    tools = _Tools({"lookup_melody": [(False, "Мелодия 'абракадабра' не найдена")]})
    out = _run(tools, "абракадабра")
    assert out.status is NamedPlayStatus.MISS
    assert [c[0] for c in tools.calls] == ["lookup_melody"]


@pytest.mark.parametrize(
    "match",
    [
        # «марио и тетрис» → Tetris: «mario» не совпало
        {"matched": ["tetris"], "unmatched": ["mario"], "coverage": 0.56, "ignored": []},
        # «гимн германии» → гимн СССР: «германии» поиск не видел
        {"matched": ["soviet", "anthem"], "unmatched": [], "coverage": 1.0,
         "ignored": ["германии"]},
        # mcp_tools старше #3176 — поля ignored нет: не знаем, честнее в LLM
        {"matched": ["fur", "elise"], "unmatched": [], "coverage": 1.0},
    ],
)
def test_partial_match_is_miss(match: Dict[str, Any]) -> None:
    tools = _Tools({"lookup_melody": [(True, _content({**_FUR_ELISE, "match": match}))]})
    out = _run(tools, "что угодно")
    assert out.status is NamedPlayStatus.MISS
    assert [c[0] for c in tools.calls] == ["lookup_melody"]


def test_play_failure_is_failed_not_miss() -> None:
    tools = _Tools({
        "lookup_melody": [(True, _content(_FUR_ELISE))],
        "request_music": [(False, "мелодия не заиграла: not_started")],
    })
    out = _run(tools, "к элизе")
    assert out.status is NamedPlayStatus.FAILED
    assert out.hit.title == "Fur Elise"
    assert out.tools_done == ("lookup_melody",)


def test_tool_data_parses_adapter_render() -> None:
    assert tool_data(_content(_FUR_ELISE)) == _FUR_ELISE
    assert tool_data(repr(_FUR_ELISE)) == _FUR_ELISE  # message == data
    assert tool_data("просто текст") is None
    assert tool_data("") is None
    assert tool_data("msg\n{broken") is None


def test_melody_hit_without_data_is_miss() -> None:
    assert melody_hit(None)[0] is None
    assert melody_hit({"name": "x", "title": "X"})[0] is None  # SQLite-фолбэк без match


# ── 4. Возврат промаха в приём ─────────────────────────────────────────


class _Step:
    def __init__(self, name: str, verdict=PASS) -> None:
        self.name = name
        self.verdict = verdict
        self.seen: List[str] = []

    def apply(self, ctx: SttContext, host: Any):
        self.seen.append(ctx.text)
        return self.verdict


def _ctx(text: str) -> SttContext:
    return SttContext(raw=text, text=text, text_lower=text.lower(), skip_counter={})


def test_resume_after_runs_only_later_steps() -> None:
    before = _Step("before")
    media = _Step("media_command", handled("media_command", "media_command"))
    after = _Step("after")
    admission = SttAdmission(steps=[before, media, after])
    assert admission.evaluate(_ctx("поставь абракадабра"), MagicMock()).kind is (
        SttOutcomeKind.HANDLED
    )
    assert after.seen == []
    out = admission.resume_after("media_command", _ctx("поставь абракадабра"), MagicMock())
    assert out.kind is SttOutcomeKind.PASS
    assert before.seen == ["поставь абракадабра"]  # не повторяется
    assert after.seen == ["поставь абракадабра"]


def test_resume_after_unknown_step_is_caller_error() -> None:
    with pytest.raises(ValueError):
        SttAdmission(steps=[_Step("a")]).resume_after("nope", _ctx("x"), MagicMock())
