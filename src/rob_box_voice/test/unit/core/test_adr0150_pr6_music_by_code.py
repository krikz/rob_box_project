"""ADR-0150 PR-6 ч.3 (#3696): музыку по словам человека решает код — генерация, библиотека, стоп в тишине.

Эпизод Б 20.10: «Робот, спой песенку про стрижей и ежей» → LLM ×8 ``music_library`` вместо ``generate_music``.
Ревизия 5 §9.1: «спой/сочини/сгенерируй» → ``generate_music(kind, theme)``; «включи из библиотеки» →
``music_library`` кодом; стоп музыки работает и в SILENCED (§4.2), молча (§6.3).
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState
from rob_box_voice.core.music_turn import ROUTER_LAUNCH_INTENTS, launch_requested

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
