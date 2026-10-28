"""Issue #3400 — «давай тему X» посреди DJ-сета: мимо базы мелодий → dj_set(theme), без LLM.

Здесь — грамматика команды и план роутера (``core.media_command_grammar``, ``core.media_router``).
PR-15 (#3711, часть 15p): сквозные проверки через ноду диалога v1 (``_on_stt`` → роутер → исполнитель) удалены
вместе с ней.
"""

from __future__ import annotations

from rob_box_voice.core.media_command_grammar import MediaCommand, MediaIntent, parse_media_command
from rob_box_voice.core.media_router import (
    MediaPlan,
    MediaState,
    dj_theme_switch_plan,
    plan_media_command,
)


def test_grammar_keeps_theme_marker():
    for phrase in ("Робот, давай тему терминатора", "Робот, включи тему терминатора",
                   "Робот, поставь на тему терминатора"):
        cmd = parse_media_command(phrase)
        assert (cmd.intent, cmd.name, cmd.themed) == (MediaIntent.PLAY_NAMED, "терминатора", True), phrase
    plain = parse_media_command("Робот, поставь терминатора")
    assert (plain.name, plain.themed) == ("терминатора", False)
    assert parse_media_command("Робот, включи тему").themed is False


def test_router_theme_marker_only_switches_inside_set():
    cmd = MediaCommand(intent=MediaIntent.PLAY_NAMED, name="терминатора", themed=True)
    inside = plan_media_command(cmd, MediaState(music_playing=True, dj_enabled=True, dj_persona="Снупдог"))
    assert inside.tool_calls[0].arguments == {"action": "start", "theme": "терминатора", "persona": "Снупдог"}
    outside = plan_media_command(cmd, MediaState(music_playing=True))
    assert outside.play_name == "терминатора" and not outside.tool_calls


def test_switch_plan_is_none_without_set_or_name():
    command = MediaCommand(intent=MediaIntent.PLAY_NAMED)
    plan = MediaPlan(command=command, play_name="терминатора")
    assert dj_theme_switch_plan(plan, MediaState(music_playing=True)) is None
    assert dj_theme_switch_plan(MediaPlan(command=command), MediaState(dj_enabled=True)) is None
