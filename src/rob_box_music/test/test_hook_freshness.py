"""#3399: межсетовая свежесть хуков — очередь первого трека и окно истории по сетам (без архива, свои мелодии)."""

from __future__ import annotations

import random

from melodies import MELODIES, profile
from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import compose, hook_candidates, opening_order
from rob_box_music.diversity import last_opener, recent_hooks
from rob_box_music.set_plan import seeded_plan

HOOKS = ("long", "short", "slow")
#: ``long`` и его версия ``long_v2`` (та же RTTTL — общий контур начала), ``short`` — другая мелодия.
TUNES = {"long": "L", "long_v2": "L", "short": "S", "slow": "W"}


def _row(set_id, melody):
    return {"set_id": set_id, "melody_name": melody}


def test_previous_set_makes_every_version_of_its_tune_recent_but_the_current_set_counts_records():
    history = [_row("cur", "short"), _row("p1", "long")]
    assert recent_hooks(history, "cur", TUNES) == ["short", "long", "long_v2"]


def test_sets_older_than_the_window_are_forgotten():
    history = [_row(f"p{n}", name) for n, name in enumerate(("short", "slow", "long", "long_v2"), 1)]
    assert kn.HOOK_FRESH_SETS == 3
    recent = recent_hooks(history, "cur", {"short": "S", "slow": "W", "long": "L", "long_v2": "X"})
    assert recent == ["short", "slow", "long"]  # четвёртый сет назад — за окном


def test_last_opener_is_the_oldest_track_of_the_freshest_previous_set():
    history = [_row("cur", "slow"), _row("p1", "short"), _row("p1", "long"), _row("p2", "slow")]
    assert last_opener(history, "cur") == "long"
    assert last_opener([_row("cur", "slow")], "cur") is None


def test_opening_order_puts_fresh_first_then_played_in_profile_order_and_the_last_opener_last():
    ids = ["a", "b", "c", "d"]
    assert opening_order(ids, [], None) == ids  # без истории — порядок профиля (#3427)
    assert opening_order(ids, ["c", "a"], "a") == ["b", "d", "c", "a"]
    assert opening_order(ids, ["a", "b", "c", "d"], "a") == ["b", "c", "d", "a"]


def test_first_track_skips_the_hook_that_opened_the_previous_set():
    """Хук №1 профиля открыл прошлый сет — новый сет на ту же тему открывает следующий свежий."""
    prof = profile(hooks=HOOKS)
    history = [_row("prev", "slow"), _row("prev", "long")]  # прошлый сет: long (трек 1), slow (трек 2)
    order = [h.source for h, _k in hook_candidates(prof, MELODIES, random.Random(0), history, opening=True,
                                                   set_id="cur")]
    assert order[0] == "short"
    plan = seeded_plan(prof, 5, set_id="cur", history=history)
    assert compose(plan, 1, melodies=MELODIES, history=history).hook.source == "short"


def test_first_track_without_history_is_still_hook_number_one():
    for seed in range(4):
        plan = seeded_plan(profile(hooks=HOOKS), seed, set_id="cur")
        assert compose(plan, 1, melodies=MELODIES).hook.source == HOOKS[0]
