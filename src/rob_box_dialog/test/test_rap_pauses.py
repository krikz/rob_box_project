"""Рэп в такт паузами MiniMax по сетке строк (О11, #3901): формула и лимиты :mod:`rap_pauses`, и как выступление
их применяет — разметка из сетки подложки; без подложки и не у MiniMax — как раньше, с причиной в логе."""

import re

import pytest

from perform_sim import Sim

from rob_box_dialog.command import Performance
from rob_box_dialog.plan import rap_pauses as rp
from rob_box_dialog.plan.perform import Say, context_block

VERSES = (
    "Я робот Робби, я читаю под бит,\nКолёса крутятся, мотор не молчит.\n"
    "Камера смотрит, микрофон не спит,\nКаждая строчка в такт звучит.",
    "А вот и енот, полосатый хвост,\nОн моет печенье и лезет на мост.",
)
_TAG = re.compile(r"<#(\d+\.\d\d)#>")


def rap(backing="beat"):
    return Performance(kind="rap", segments=VERSES, backing=backing)


def pauses(text):
    return [float(x) for x in _TAG.findall(text)]


def presynths(sim):
    return [e for _, e in sim.effects if isinstance(e, Say) and not e.commit and not e.key]


# --- формула и лимиты ---------------------------------------------------------------------------------------------

def test_pause_fills_the_lines_bars_minus_speech_and_overhead():
    bar = 4.0  # 60 BPM, 4/4: самая длинная строка (11 слогов ≈ 2.2 с) влезает в такт
    lines = rp.split_lines(VERSES[0])
    got = rp.markup(VERSES[0], bar)
    spans = [rp.syllables(ln) / rp.SYLLABLES_PER_S for ln in lines]
    assert got.bars_per_line == 1 and max(spans) <= rp.BAR_FILL * bar
    want = [round(bar - s - rp.PAUSE_OVERHEAD_S, 2) for s in spans[:-1]]
    assert got.pauses == tuple(want) and pauses(got.text) == want
    # каждая строка кроме последней — с паузой, знак конца строки уступил паузе; последняя — как была
    assert got.text.startswith("Я робот Робби, я читаю под бит <#")
    assert got.text.endswith("Каждая строчка в такт звучит.")
    assert got.text.count("<#") == len(lines) - 1


def test_line_longer_than_a_bar_takes_two_bars():
    bar = 240.0 / 160  # 160 BPM: такт 1.5 с, строка ~2.2 с — два такта
    got = rp.markup(VERSES[0], bar)
    assert got.bars_per_line == 2
    spans = [rp.estimate_span(ln) for ln in rp.split_lines(VERSES[0])]
    assert got.pauses == tuple(round(2 * bar - s - rp.PAUSE_OVERHEAD_S, 2) for s in spans[:-1])


def test_limits_clamp_the_pause_and_the_bars():
    long_line = "Очень длинная строка про енота и комара, которые ночью читают рэп у окна"
    fast = rp.markup(f"{long_line},\n{long_line}.", 240.0 / 200)  # такт 1.2 с: строке мало и двух тактов
    assert fast.bars_per_line == rp.MAX_BARS_PER_LINE and fast.pauses == (rp.MIN_PAUSE_S,)
    assert rp.pause_tag(500.0) == f"<#{rp.MAX_PAUSE_S:.2f}#>" and rp.pause_tag(-1.0) == "<#0.05#>"


def test_one_line_has_nothing_to_mark_and_commas_split_a_flat_segment():
    assert rp.markup("Я робот Робби", 2.0) is None
    assert rp.markup("Я робот Робби,\nя читаю под бит", 0.0) is None
    assert rp.split_lines("Я робот Робби, я читаю под бит. Мотор не молчит!") == [
        "Я робот Робби,", "я читаю под бит.", "Мотор не молчит!"]


def test_fallback_provider_gets_the_text_without_pause_marks():
    marked = rp.markup(VERSES[1], 2.0).text
    assert rp.text_for("minimax", marked) == marked
    for provider in ("yandex", "silero"):
        plain = rp.text_for(provider, marked)
        assert "<#" not in plain and "#>" not in plain
        assert plain == "А вот и енот, полосатый хвост, Он моет печенье и лезет на мост."


# --- выступление ----------------------------------------------------------------------------------------------------

def test_minimax_with_beat_marks_every_verse_from_the_backing_grid():
    sim = Sim(rap(), tts_provider="minimax", bpm=120.0)
    sim.start()
    assert presynths(sim) == []  # сетка ещё неизвестна — пред-синтез ждёт подложку
    sim.run(300)
    sent = presynths(sim)
    assert [rp.strip_pauses(s.text) for s in sent] == [rp.strip_pauses(rp.markup(v, 2.0).text) for v in VERSES]
    assert [s.text for s in sent] == [rp.markup(v, 2.0).text for v in VERSES]  # такт 2.0 с — из started{bpm}
    commits = {e.speech_id: e.text for _, e in sim.effects if isinstance(e, Say) and e.commit and not e.key}
    assert [commits[s.speech_id] for s in sent] == [s.text for s in sent]  # звучит то, что синтезировано
    assert sim.block.end_reason == "done" and all(e.synced for e in sim.events("segment_started"))
    assert any("perform: разметка пауз MiniMax по сетке строк, такт 2.00 с" in line for line in sim.logs)


def test_pauses_follow_the_backing_tempo():
    slow, fast = Sim(rap(), tts_provider="minimax", bpm=90.0), Sim(rap(), tts_provider="minimax", bpm=131.0)
    for sim in (slow, fast):
        sim.start()
        sim.run(300)
    first = [pauses(presynths(sim)[0].text) for sim in (slow, fast)]
    assert first[0] == pauses(rp.markup(VERSES[0], 240.0 / 90).text)
    assert first[1] == pauses(rp.markup(VERSES[0], 240.0 / 131).text)
    assert first[0] != first[1]


def test_context_for_the_llm_keeps_the_plain_verse():
    sim = Sim(rap(), tts_provider="minimax")
    sim.start()
    sim.run(5)
    ctx = context_block(sim.block, sim.t)
    assert all("<#" not in text for text in [*ctx["done"], *ctx["next_fixed"]])
    assert sim.block.lines.segments()[0].text == VERSES[0]


@pytest.mark.parametrize("provider, backing, music, reason", [
    ("minimax", "none", ("ok",), "нет подложки"),
    ("yandex", "beat", ("ok",), "синтез yandex, не MiniMax"),
    ("", "beat", ("ok",), "синтез неизвестен, не MiniMax"),
    ("minimax", "beat", ("rejected:quota",), "подложки нет: quota"),
])
def test_no_markup_without_backing_or_minimax_and_the_log_says_why(provider, backing, music, reason):
    sim = Sim(rap(backing), tts_provider=provider, music=music)
    sim.start()
    sim.run(300)
    assert [s.text for s in presynths(sim)] == list(VERSES)  # как раньше: текст сегмента без меток
    assert [h.text for h in sim.segments_heard()] == list(VERSES)
    assert sum(f"perform: разметка пауз выключена ({reason})" in line for line in sim.logs) == 1


def test_stop_while_waiting_for_the_backing_cancels_nothing_never_sent():
    sim = Sim(rap(), tts_provider="minimax", music=("silent",))
    sim.start()
    effects = sim.block.stop(sim.t)
    assert not [e for e in effects if type(e).__name__ == "Cancel"]
    assert presynths(sim) == []
