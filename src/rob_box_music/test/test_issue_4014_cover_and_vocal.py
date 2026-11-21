"""#4014 — «сыграй/сгенерируй X и спой»: честный отказ, не склейка кавера с вокалом и не ``named_song``.

Кавер (ADR-0158 §2.9) вокала не несёт. Песня робота — свой текст (ADR-0158 §3.4). Одной репликой оба пути
не склеиваются (ADR-0176): причина ``cover_and_vocal`` из результата маршрута, фраза — код из этой причины.
«Спой X» без просьбы сыграть или сгенерировать остаётся ``named_song`` (#4015). Новой регулярки нет.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from rob_box_music.source import (
    COVER_AND_VOCAL,
    NAMED_SONG,
    REASON_WORDS,
    CoverWork,
    SourceContext,
    cover_and_vocal_asked,
    decline_text,
    source_ask,
    source_candidates,
)

HILL = CoverWork(
    "Running Up That Hill", "runningu", "balalaika", "Running Up That Hill", named=True
)
PICKED = replace(HILL, named=False)
READY = SourceContext(gen_ready=True, cover_ready=True, day_left=120.0, balance=900.0)


@pytest.mark.parametrize(
    "text",
    [
        "сыграй Running Up That Hill и спой",
        "сгенерируй Running Up That Hill на балалайке и гуслях и спой",
        "сыграй полёт валькирий и спой своим голосом",
    ],
)
def test_one_request_asks_both_cover_and_vocal(text):
    ask = source_ask(text)
    assert ask.cover_and_vocal and ask.vocal
    assert cover_and_vocal_asked(text)


@pytest.mark.parametrize(
    "text",
    [
        "сыграй Running Up That Hill",
        "спой Running Up That Hill",
        "сгенерируй рок",
    ],
)
def test_a_single_ask_is_not_the_joint(text):
    assert not source_ask(text).cover_and_vocal
    assert not cover_and_vocal_asked(text)


def test_joint_named_piece_declines_with_its_own_reason_not_named_song():
    """Кавер-кандидата нет и заказа песни нет: ни склейки, ни ``named_song``."""
    ask = source_ask("сыграй Running Up That Hill и спой")
    options, reason = source_candidates(ask, READY, piece=True, work=HILL)
    assert reason == COVER_AND_VOCAL
    assert [o.id for o in options] == [f"decline:{COVER_AND_VOCAL}"]
    assert reason != NAMED_SONG
    assert not any(
        o.id.startswith("generate:") or o.id.startswith("cache:") for o in options
    )


def test_sing_alone_stays_named_song():
    options, reason = source_candidates(
        source_ask("спой Running Up That Hill"), READY, piece=True, work=HILL
    )
    assert reason == NAMED_SONG and [o.id for o in options] == [f"decline:{NAMED_SONG}"]


def test_play_alone_still_offers_a_cover():
    options, _reason = source_candidates(
        source_ask("сыграй Running Up That Hill"), READY, piece=True, work=HILL
    )
    assert "generate:cover:V6" in [o.id for o in options]
    assert f"decline:{COVER_AND_VOCAL}" not in [o.id for o in options]


def test_joint_on_a_piece_the_code_picked_is_not_this_refusal():
    """Пьесу подобрал код — человек её не называл: отказ «и сыграть, и спеть» был бы неправдой."""
    ask = replace(source_ask("сыграй что-нибудь и спой"), cover_and_vocal=True)
    options, reason = source_candidates(ask, READY, piece=True, work=PICKED)
    assert reason != COVER_AND_VOCAL and reason != NAMED_SONG
    assert all(o.id != f"decline:{COVER_AND_VOCAL}" for o in options)


def test_phrase_is_built_from_the_reason_and_does_not_glue():
    ask = source_ask("сыграй Running Up That Hill и спой")
    route_options, reason = source_candidates(ask, READY, piece=True, work=HILL)
    from rob_box_music.source import DECLINE, Route

    route = Route(DECLINE, route_options[0].id, ask, reason, work=HILL)
    text = decline_text(route)
    assert REASON_WORDS[COVER_AND_VOCAL] in text
    assert REASON_WORDS[NAMED_SONG] not in text
    assert "не подключена" not in text
    assert "кавер" in text and "своим текстом" in text
    assert "Running Up That Hill" in text
