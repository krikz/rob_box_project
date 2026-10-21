"""ADR-0150 PR-6 ч.3 (#3696): музыку по словам человека решает код — генерация, библиотека, стоп в тишине.

Эпизод Б 20.10: «Робот, спой песенку про стрижей и ежей» → LLM ×8 ``music_library`` вместо ``generate_music``.
Ревизия 5 §9.1: «спой/сочини/сгенерируй» → ``generate_music(kind, theme)``; «включи из библиотеки» →
``music_library`` кодом; стоп музыки работает и в SILENCED (§4.2), молча (§6.3).
"""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest

from rob_box_voice.core.library_command import pick_track
from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_plan_run import run_media_plan
from rob_box_voice.core.media_router import MediaRouter, MediaState
from rob_box_voice.core.music_turn import ROUTER_LAUNCH_INTENTS, launch_requested
from rob_box_voice.core.stt_admission import SttContext, SttOutcomeKind, Tier1Step
from rob_box_voice.core.tier1_host import Tier1Runner

GENERATE = [
    ("спой песенку про стрижей и ежей", "robot_song", "стрижей и ежей"),
    ("спой песенку и сыграй песенку про стрижей и ежей", "robot_song", "стрижей и ежей"),
    ("Робот, спой песенку про стрижей и ежей", "robot_song", "стрижей и ежей"),
    ("сгенерируй песню про енота", "robot_song", "енота"),
    ("сгенерируй пиздатый музон своим голосом", "robot_song", ""),
    ("сгенерируй музыку", "instrumental", ""),
    ("давай сгенерируй трек про космос", "instrumental", "космос"),
    ("сочини песню про робота", "robot_song", "робота"),
    ("сочини мелодию про дождь", "instrumental", "дождь"),
    ("придумай песню о любви", "robot_song", "любви"),
    ("спой", "robot_song", ""),
    ("спой что-нибудь", "robot_song", ""),
    ("давай спой песенку пожалуйста", "robot_song", ""),
    ("можешь спеть песню про лето", "robot_song", "лето"),
    ("спой песню про стоп", "robot_song", "стоп"),
    ("спой песню про хватит", "robot_song", "хватит"),
    ("спой своим голосом про котов", "robot_song", "котов"),
    ("сгенерируй рок песню на тему дороги", "robot_song", "дороги"),
]

NOT_GENERATE = [
    "сочини сказку про ежа",       # не музыка — LLM
    "придумай имя",
    "спой калинку",                # название без слова музыки — LLM (кавер решает путь заказа)
    "спой со мной",
    "спой об этом",                # ссылка на контекст
    "спой про",                    # пустая тема
    "спой погромче",               # громкость — не тема
    "сыграй песенку про ежей",     # «сыграй» — не генерация (ADR §9.1: спой/сочини/сгенерируй)
    "поставь к элизе",
    "включи клубный трек",
    "расскажи анекдот",
]

LIBRARY = [
    ("включи что-нибудь из библиотеки", "play", ""),
    ("Робот, включи что-нибудь из библиотеки", "play", ""),
    ("включи из библиотеки песню про ежей", "play", "ежей"),
    ("поставь трек из библиотеки", "play", ""),
    ("найди в библиотеке дождь", "play", "дождь"),
    ("что у тебя в библиотеке", "list", ""),
    ("что есть в библиотеке", "list", ""),
    ("покажи библиотеку", "list", ""),
    ("какие треки лежат в библиотеке", "list", ""),
]

NOT_LIBRARY = ["выключи библиотеку", "расскажи про библиотеку в москве", "включи из библиотеки его снова"]


@pytest.mark.parametrize("text,kind,theme", GENERATE)
def test_sing_compose_generate_is_generation_by_code(text, kind, theme):
    command = parse_media_command(text)
    assert (command.intent, command.kind, command.name) == (MediaIntent.GENERATE, kind, theme)
    assert launch_requested(text)  # ход LLM без запуска на такой фразе — честный отказ, а не «спела»


@pytest.mark.parametrize("text", NOT_GENERATE)
def test_not_generation(text):
    assert parse_media_command(text).intent is not MediaIntent.GENERATE


@pytest.mark.parametrize("text,kind,query", LIBRARY)
def test_library_words_are_library_by_code(text, kind, query):
    command = parse_media_command(text)
    assert (command.intent, command.kind, command.name) == (MediaIntent.LIBRARY, kind, query)


@pytest.mark.parametrize("text", NOT_LIBRARY)
def test_not_library(text):
    assert parse_media_command(text).intent is not MediaIntent.LIBRARY


def test_router_plans_generate_music_with_kind_and_theme():
    plan = MediaRouter().route("Робот, спой песенку про стрижей и ежей", MediaState())
    assert [(c.name, c.arguments) for c in plan.tool_calls] == [
        ("generate_music", {"kind": "robot_song", "theme": "стрижей и ежей"})]
    assert plan.cancel_inflight and plan.confirm_started
    assert all(c.name != "music_library" for c in plan.tool_calls)
    plain = MediaRouter().route("сгенерируй музыку", MediaState())
    assert plain.tool_calls[0].arguments == {"kind": "instrumental"}


def test_library_plan_has_no_static_tools_and_does_not_reach_history():
    plan = MediaRouter().route("включи что-нибудь из библиотеки", MediaState())
    assert plan.library == "play" and plan.tool_calls == () and plan.cancel_inflight
    assert MediaIntent.LIBRARY in ROUTER_LAUNCH_INTENTS


def _tracks(*rows):
    return [dict(zip(("id", "title", "rating", "last_played_at"), row)) for row in rows]


def test_pick_track_skips_disliked_and_prefers_long_unplayed():
    rows = _tracks(("a", "A", -1, ""), ("b", "B", 5, "2026-10-20"), ("c", "C", 0, ""), ("d", "D", 0, "2026-10-01"))
    assert pick_track(rows, by_relevance=False)["id"] == "c"
    assert pick_track(rows, by_relevance=True)["id"] == "b"
    assert pick_track(_tracks(("a", "A", -1, "")), by_relevance=False) is None
    assert pick_track([], by_relevance=True) is None


class _Tools:
    """Исполнитель тулов роутера: ответы ``music_library`` по действию; ``calls`` — что звали."""

    def __init__(self, responses):
        self.responses = responses
        self.calls = []

    async def __call__(self, call):
        self.calls.append((call.name, dict(call.arguments)))
        return self.responses[call.arguments["action"]]


def _run(text, tools):
    plan = MediaRouter().route(text, MediaState())
    return asyncio.run(run_media_plan(plan, tools))


def test_library_play_without_words_lists_best_and_plays_the_code_pick():
    rows = _tracks(("x1", "Утро", 0, "2026-10-19"), ("x2", "Дождь", 0, ""))
    listed = "В библиотеке 2 трека.\n" + repr({"tracks": rows, "total": 2})
    tools = _Tools({"list": (True, listed), "play": (True, "Включаю «Дождь» из библиотеки — 2 минуты.\n{}")})
    ok, phrase, done = _run("Робот, включи что-нибудь из библиотеки", tools)
    assert tools.calls == [("music_library", {"action": "list", "limit": 20, "sort_by": "rating"}),
                           ("music_library", {"action": "play", "track_id": "x2"})]
    assert (ok, phrase, done) == (True, "Включаю «Дождь» из библиотеки — 2 минуты.", ["music_library"])


def test_library_play_with_words_searches_and_plays_the_first_hit():
    found = "По запросу «ежей» найдено 1.\n" + repr({"tracks": _tracks(("e1", "Ежи", 0, "")), "total": 1})
    tools = _Tools({"search": (True, found), "play": (True, "Включаю «Ежи» из библиотеки — минута.\n{}")})
    ok, phrase, _ = _run("включи из библиотеки песню про ежей", tools)
    assert tools.calls[0] == ("music_library", {"action": "search", "query": "ежей", "limit": 5})
    assert tools.calls[1] == ("music_library", {"action": "play", "track_id": "e1"})
    assert ok and phrase.startswith("Включаю «Ежи»")


def test_library_miss_says_the_tool_phrase_and_plays_nothing():
    empty = "По запросу «дождь» ничего не найдено.\n" + repr({"tracks": [], "total": 0})
    tools = _Tools({"search": (True, empty)})
    ok, phrase, _ = _run("найди в библиотеке дождь", tools)
    assert (ok, phrase) == (False, "По запросу «дождь» ничего не найдено.")
    assert [c[1]["action"] for c in tools.calls] == ["search"]


def test_library_play_failure_is_an_honest_phrase_of_code():
    found = "x\n" + repr({"tracks": _tracks(("e1", "Ежи", 0, ""))})
    tools = _Tools({"search": (True, found), "play": (False, "трек не заиграл: missing_sample")})
    ok, phrase, _ = _run("найди в библиотеке ежей", tools)
    assert not ok and "missing_sample" not in phrase


def test_library_list_phrase_is_the_tool_first_line():
    tools = _Tools({"list": (True, "В библиотеке 23 трека, например: «Утро».\n{'tracks': [], 'total': 23}")})
    assert _run("что у тебя в библиотеке", tools)[:2] == (True, "В библиотеке 23 трека, например: «Утро».")


def test_pending_song_without_bridge_says_the_wait_phrase_not_not_started():
    """Песня пишется (``pending``), моста нет — ``track_id`` нет: раньше роутер сказал бы «не заиграло»."""
    data = {"ok": True, "pending": "song", "source_message": "Пишу песню, это займёт около минуты."}

    async def tools(call):
        return True, repr(data)

    ok, phrase, done = _run("спой песенку про стрижей и ежей", tools)
    assert (ok, phrase, done) == (True, data["source_message"], ["generate_music"])


# -- стоп музыки в SILENCED (§4.2: Tier-1 в любом состоянии; §6.3: в тишине ничего не звучит) -----------------


class _Host:
    def __init__(self, can_stop=True):
        self.can_stop = can_stop
        self.calls = []

    def is_music_stop_command(self, text_lower):
        return "музык" in text_lower or "диджеить" in text_lower

    def stop_music_quietly(self):
        self.calls.append("stop_music_quietly")
        return self.can_stop

    def tier1_match(self, ctx):  # pragma: no cover — до него не доходит
        raise AssertionError("музыкальный стоп — не Tier-1 грамматика диалога")


def _ctx(text, silenced):
    return SttContext(raw=text, text=text, text_lower=text.lower(), tg_chat_id=None, speaker_tag=None,
                      speaker_duration_s=0.0, state_name="SILENCED" if silenced else "IDLE",
                      is_silenced=silenced, wake_words=("робот",))


@pytest.mark.parametrize("text", ["выключи музыку", "хватит диджеить"])
def test_music_stop_in_silence_is_executed_quietly_by_tier1(text):
    host = _Host()
    outcome = Tier1Step().apply(_ctx(text, silenced=True), host)
    assert outcome.kind is SttOutcomeKind.HANDLED and host.calls == ["stop_music_quietly"]


def test_music_stop_outside_silence_is_left_to_the_media_router():
    host = _Host()
    assert Tier1Step().apply(_ctx("выключи музыку", silenced=False), host).kind is SttOutcomeKind.PASS
    assert host.calls == []


def test_music_stop_in_silence_without_executor_passes_on():
    host = _Host(can_stop=False)
    assert Tier1Step().apply(_ctx("выключи музыку", silenced=True), host).kind is SttOutcomeKind.PASS


def test_runner_stop_music_needs_an_executor():
    node = SimpleNamespace(_tool_executor=None, get_logger=lambda: SimpleNamespace(info=lambda _m: None))
    assert Tier1Runner(node).stop_music() is False
