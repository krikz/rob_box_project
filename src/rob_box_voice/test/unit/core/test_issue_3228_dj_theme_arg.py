"""Issue #3228 (umbrella #3223) — тема DJ-сета доезжает до ``compose_music(theme=...)``."""

from __future__ import annotations

import json
import logging
import re
from types import SimpleNamespace

from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.dj_set_walk import club_theme_arg

START = 1_759_000_000.0


def _controller():
    hook = DJHook(dispatch=lambda prompt, from_tick=False: None, is_active=lambda: False,
                  is_dialogue_active=lambda: False)
    return DJModeController(hook=hook, logger=logging.getLogger("test"), clock=lambda: START)


def _call(theme):
    ctrl = _controller()
    ctrl.handle_message(json.dumps({"enabled": True, "theme": theme}))
    return ctrl._club_call(2)


def test_theme_without_melody_pool_goes_into_club_call():
    call = _call("Очень странные дела")
    assert call.startswith('compose_music(style="club", theme="Очень странные дела", bpm=')
    assert "name=" not in call


def test_theme_with_melody_pool_keeps_name_and_omits_theme():
    call = _call("денди")
    assert 'name="' in call and "theme=" not in call


def test_no_theme_call_is_unchanged():
    call = _controller()._club_call(2)
    assert re.match(r'compose_music\(style="club", bpm=\d+, root="[A-G]#?", scale="\w+", seed=\d+, ', call)


def test_theme_arg_sanitizes_stt_text():
    state = SimpleNamespace(theme='Робот" ); drop_tables(\n`x` {y} <z> ' + "я" * 200)
    arg = club_theme_arg(state)
    assert arg.startswith('theme="') and arg.endswith('", ')
    inner = arg[len('theme="'):-len('", ')]
    assert not re.search(r'["\\\n`(){}<>\[\]]', inner) and len(inner) <= 80
    assert club_theme_arg(SimpleNamespace(theme="x"), hooked=True) == ""
    assert club_theme_arg(SimpleNamespace(theme="  ")) == ""
