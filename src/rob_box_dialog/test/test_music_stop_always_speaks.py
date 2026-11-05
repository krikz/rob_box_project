"""Стоп музыки Tier-1 - всегда словами, с вариациями (решение Шифу 05.11; М12 n511/n712, #4402).

«хватит, диджей, выключай» говорило «Выключил музыку.», а «глуши музыку» (Tier-1 ``stop_all``, и без вейка -
``MUSIC_STOP_UNWOKEN``) - молчало. Теперь каждый путь стопа музыки (``music_off``, ``stop_all`` под музыкой) говорит
вариант кода без повтора подряд, исход - по факту: погасил - ``MUSIC_STOP_SAID``, играть было нечему -
``MUSIC_STOP_QUIET``, отказ тула - его фраза. Стоп одной речи (музыки не было, тулов стопа нет) - молчит, как раньше.
"""

import random

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.grammar import Tier1Command
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.turn import tier1_phrase

SAID = frozenset(phrases.catalog()[kn.MUSIC_STOP_SAID])
QUIET = frozenset(phrases.catalog()[kn.MUSIC_STOP_QUIET])
STOP_ALL = Tier1Command("stop_all")
MUSIC_OFF = Tier1Command(kn.MUSIC_OFF_KIND)


def _stop(was_playing, status="done", tool="stop_music", **data):
    return ActionResult("t1/0", 0, "personality", tool, kn.TOOL_CLASSES[tool], status, None,
                        {"was_playing": was_playing, **data})


def test_stop_all_under_music_speaks_a_variant():
    """«Глуши музыку» / «стоп» под музыкой: стоп кодом и фраза кода, а не тишина."""
    pick = phrases.Picker(random.Random(1))
    assert tier1_phrase(STOP_ALL, [_stop(True)], pick) in SAID
    assert tier1_phrase(MUSIC_OFF, [_stop(True)], pick) in SAID


def test_variants_rotate_without_a_repeat_in_a_row():
    """Разные вариации, и один и тот же вариант дважды подряд не звучит."""
    pick = phrases.Picker(random.Random(7))
    said = [tier1_phrase(STOP_ALL, [_stop(True)], pick) for _ in range(30)]
    assert set(said) <= SAID and len(set(said)) > 2
    assert all(a != b for a, b in zip(said, said[1:]))


def test_nothing_was_playing_says_it_was_quiet():
    """Исход по факту: тул ответил ``was_playing=False`` - «И так тихо», а не «Выключил»."""
    pick = phrases.Picker(random.Random(3))
    assert tier1_phrase(STOP_ALL, [_stop(False)], pick) in QUIET
    assert tier1_phrase(MUSIC_OFF, [], pick) in QUIET


def test_set_stopped_then_track_idle_is_still_said_off():
    """Сет погашен ``dj_set stop``, ``stop_music`` следом застал тишину (``stopped_earlier``) - музыку выключил ход."""
    pick = phrases.Picker(random.Random(5))
    results = [_stop(True, tool="dj_set", action="stop"), _stop(False, **{kn.STOPPED_EARLIER_FIELD: True})]
    assert tier1_phrase(STOP_ALL, results, pick) in SAID


def test_failed_stop_says_the_failure():
    pick = phrases.Picker(random.Random(2))
    assert tier1_phrase(STOP_ALL, [_stop(None, status="rejected")], pick) == phrases.text("music.stop_fail")


def test_speech_only_stop_stays_silent():
    """«Хватит» без музыки (тулов стопа нет) и «стой, а музыку оставь» - о музыке не просили, слов нет."""
    pick = phrases.Picker(random.Random(4))
    assert tier1_phrase(STOP_ALL, [], pick) is None
    assert tier1_phrase(Tier1Command("stop_all", {"keep_music": True}), [], pick) is None
