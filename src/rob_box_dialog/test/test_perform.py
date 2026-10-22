"""Выступление (ADR-0150 §5.6.3, §5.6.4; PR-7c #3700) на фейковых владельцах речи и музыки (:mod:`perform_sim`):
жизненный цикл под бит, эталонные сценарии X3 («и про енота»), X5 («хватит»), #993 («добавь бит»), отказы подложки
(вторая попытка, ``perform.no_backing.{quota,timeout}``), О7 (правка не успела к границе), блок контекста LLM."""

import pytest

from perform_sim import LATENCY_MS, Sim

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Expression, Performance
from rob_box_dialog.plan.backing import MusicStop
from rob_box_dialog.plan.perform import Cancel, PerformBlock, Say, context_block
from rob_box_dialog.plan.segments import FIXED

SEGS = (
    "Я комар, звеню над ухом всю ночь подряд. Ты не спишь, а я доволен и крылья гудят.",
    "Ищу тёплую руку, мне нужен ужин. Ты включаешь свет, а я уже не нужен.",
    "Прячусь за шторой, жду темноты опять. Комариный рэп нельзя перестать читать.",
    "Летом я король, зимой меня не видно. Кто меня прихлопнет — тому будет обидно.",
    "Пищу на басах, ловлю ритм с потолка. Моя песня тонкая, но очень долга.",
    "Вот и утро, я устал, пора домой. Завтра снова прилечу — я твой герой.",
)
NEW = (
    "А вот енот пришёл, полоскает бельё. Он ночной бандит, и всё тут своё.",
    "Енот и комар — два ночных брата. Один шуршит, другой звенит, и вся хата.",
    "Енот моет лапы, комар точит нос. Кто кого переиграет — вот вопрос.",
    "Утро наступило, енот ушёл в нору. Комар остался петь — и снова поутру.",
)
BAR_S = 2.0  # 120 BPM, 4/4


def rap(backing="beat", segments=SEGS):
    return Performance(kind="rap", segments=segments, backing=backing)


def on_bar(sim, t):
    k = (t - sim.track_started_at) / BAR_S
    return abs(k - round(k)) < 1e-6


# --- жизненный цикл ------------------------------------------------------------------------------------------------

def test_rap_with_beat_every_line_starts_on_a_bar_and_beat_outros():
    sim = Sim(rap())
    sim.start()
    started = [e for _, e in sim.effects]
    assert started[0].kind == "block_start"
    assert [s.commit for s in started if isinstance(s, Say)] == [False] * len(SEGS)  # пред-синтез всех сегментов
    (backing,) = sim.backings()
    estimate = sum(len(s) for s in SEGS) / kn.SPEECH_CHARS_PER_S_GUESS
    assert backing.mood == kn.PERFORM_BACKING_MOODS["beat"][0] and not backing.extend
    assert backing.min_s == pytest.approx(estimate * kn.PERFORM_FORM_MARGIN + kn.PERFORM_OUTRO_S, abs=0.1)

    sim.run(300)

    assert sim.block.ended and sim.block.end_reason == "done"
    assert sim.heard[0].text == phrases.text("perform.backing_started")  # «Поехали!» — на первом такте
    assert [h.text for h in sim.segments_heard()] == list(SEGS)
    assert all(h.outcome == "finished" for h in sim.heard)  # 0 cancelled между сегментами
    for h in sim.heard:
        assert h.start == pytest.approx(h.at) and on_bar(sim, h.start)
    for a, b in zip(sim.heard, sim.heard[1:]):
        assert a.end - 1e-6 <= b.start < a.end + BAR_S  # не внахлёст и не дольше такта паузы
    assert sim.music_stops() == [MusicStop("bar", "perform_end")]  # outro после речи
    last_end = max(h.end for h in sim.heard)
    stop_t = next(t for t, e in sim.music_log if isinstance(e, MusicStop))
    assert stop_t >= last_end - LATENCY_MS / 1000.0
    (end,) = sim.events("block_end")
    assert (end.segment_no, end.synced, end.reason) == (len(SEGS), True, "done")
    assert [e.segment_no for e in sim.events("segment_started")] == [1, 2, 3, 4, 5, 6]
    assert all(e.synced for e in sim.events("segment_started"))
    assert sim.block.summary()["spoken_chars"] == sum(len(s) for s in SEGS)


def test_without_backing_reads_back_to_back_and_honestly_unsynced():
    sim = Sim(rap(backing="none"))
    sim.start()
    sim.run(300)
    assert sim.backings() == [] and sim.music_stops() == []
    heard = sim.segments_heard()
    assert [h.text for h in heard] == list(SEGS) and all(h.at == 0.0 for h in heard)
    for a, b in zip(heard, heard[1:]):
        assert b.start == pytest.approx(a.end)  # очередь владельца речи — встык
    (end,) = sim.events("block_end")
    assert end.synced is False and not any(e.synced for e in sim.events("segment_started"))


def test_late_start_is_honestly_not_synced():
    """Сегмент встал на 0.3 с позже такта (сбой очереди владельца речи): ``synced=false`` у него и у итога."""
    sim = Sim(rap(), hiccup={SEGS[2]: 0.3})
    sim.start()
    sim.run(300)
    assert [e.synced for e in sim.events("segment_started")] == [True, True, False, True, True, True]
    assert sim.events("block_end")[0].synced is False and sim.block.summary()["synced"] is False


def test_prediction_eta_counts_down():
    sim = Sim(rap())
    sim.start()
    sim.run(300)
    etas = [e.eta_ms for e in sim.events("prediction")]
    assert len(etas) == len(SEGS) and etas == sorted(etas, reverse=True) and etas[-1] > 0


# --- отказ подложки: фолбек и фраза по причине (О10) -----------------------------------------------------------------

def test_quota_twice_then_honest_phrase_and_reading_without_beat():
    sim = Sim(rap(), music=("rejected:quota", "rejected:quota"))
    sim.start()
    sim.run(300)
    assert [b.mood for b in sim.backings()] == list(kn.PERFORM_BACKING_MOODS["beat"])  # вторая попытка — другим
    first = sim.heard[0]
    assert first.key == "perform.no_backing.quota"
    assert first.text == "Всё, братан, приплыли — бит сегодня без денег, читаю так."
    assert [h.text for h in sim.segments_heard()] == list(SEGS)
    assert sim.music_stops() == []
    assert sim.block.summary()["backing"] == "failed" and sim.block.summary()["backing_reason"] == "quota"
    assert not any(e.synced for e in sim.events("segment_started"))


def test_first_attempt_rejected_second_mood_plays():
    sim = Sim(rap(), music=("rejected:compose_error", "ok"))
    sim.start()
    sim.run(300)
    assert len(sim.backings()) == 2
    assert sim.heard[0].key == "perform.backing_started"
    assert all(on_bar(sim, h.start) for h in sim.heard)


def test_backing_silent_six_seconds_then_timeout_phrase_and_late_start_is_stopped():
    sim = Sim(rap(), music=("silent",))
    t0 = sim.t
    sim.start()
    sim.run(kn.PERFORM_BACKING_START_S + 0.05)
    assert MusicStop("now", "backing_timeout") in sim.music_stops()  # поздний старт снимается заранее
    sim.run(2.0)
    sim.music_event("started", "late", {"bpm": 120, "beat_at": sim.t, "bar_beats": 4})
    assert sim.music_stops()[-1] == MusicStop("now", "backing_late")
    sim.run(300)
    assert sim.heard[0].key == "perform.no_backing.timeout"
    assert sim.heard[0].start - t0 < kn.PERFORM_BACKING_START_S + 3
    assert [h.text for h in sim.segments_heard()] == list(SEGS)


# --- X3: «и про енота» — MERGE в PENDING ---------------------------------------------------------------------------

def test_x3_edit_pending_replaces_not_started_segments_and_keeps_the_beat():
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 2)
    assert sim.block.active_segment == 2
    sim.apply(sim.block.edit(Performance("rap", NEW, backing="beat", from_segment=3), sim.t))
    sim.run(300)
    heard = sim.segments_heard()
    assert [h.text for h in heard] == [SEGS[0], SEGS[1], *NEW]
    assert heard[1].outcome == "finished" and heard[1].spoken_chars == len(SEGS[1])  # сегмент 2 дочитан
    assert [e.segment_no for e in sim.events("segment_replaced")] == [3, 4, 5, 6]
    assert all(h.outcome == "finished" for h in sim.heard)
    assert sim.music_stops() == [MusicStop("bar", "perform_end")]  # подложка не прерывалась
    assert all(on_bar(sim, h.start) for h in sim.heard)


def test_x3_longer_edit_adds_segments():
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 5)
    sim.apply(sim.block.edit(Performance("rap", NEW, backing="beat", from_segment=6), sim.t))
    sim.run(300)
    assert [h.text for h in sim.segments_heard()] == [*SEGS[:5], *NEW]
    assert [e.segment_no for e in sim.events("segment_replaced")] == [6, 7, 8, 9]


def test_o7_edit_not_synthesized_by_the_boundary_yields_to_the_old_segment():
    slow = NEW[0]
    sim = Sim(rap(), synth_s=lambda text: 60.0 if text == slow else 1.0)
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 2)
    sim.apply(sim.block.edit(Performance("rap", NEW, backing="beat", from_segment=3), sim.t))
    sim.run(300)
    assert [h.text for h in sim.segments_heard()] == [SEGS[0], SEGS[1], SEGS[2], *NEW[1:]]


def test_edit_aimed_at_a_committed_segment_goes_from_the_next_one():
    """Сегмент 3 уже отдан на звучание (граница пройдена): правка «с 3» идёт с 4, её первый куплет не звучит."""
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 2 and sim.block.lines.top(FIXED) == 3)
    sim.apply(sim.block.edit(Performance("rap", NEW, backing="beat", from_segment=3), sim.t))
    sim.run(300)
    assert [h.text for h in sim.segments_heard()] == [*SEGS[:3], *NEW[1:]]
    assert [e.segment_no for e in sim.events("segment_replaced")] == [4, 5, 6]


def test_edit_of_a_started_segment_is_rejected_with_a_phrase():
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 3)
    sim.apply(sim.block.edit(Performance("rap", NEW, backing="beat", from_segment=2), sim.t))
    (rejected,) = sim.events("rejected")
    assert (rejected.segment_no, rejected.reason) == (2, "edit_started_segment")
    sim.run(300)
    assert [h.text for h in sim.segments_heard()] == list(SEGS)
    assert phrases.text("perform.edit_rejected") in [h.text for h in sim.heard]


def test_edit_after_the_end_is_rejected_silently():
    sim = Sim(rap(backing="none"))
    sim.start()
    sim.run(300)
    effects = sim.block.edit(Performance("rap", NEW, from_segment=3), sim.t)
    assert [e.reason for e in effects] == ["not_running"]


# --- X5: «хватит» — до конца предложения, outro ----------------------------------------------------------------------

def test_x5_stop_finishes_the_sentence_drops_the_rest_and_outros():
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 2)
    sim.run(1.0)
    seg2 = sim.block.lines.first("active")
    t_stop = sim.t
    effects = sim.block.stop(t_stop)
    assert Cancel(seg2.speech_id, "sentence") in effects
    sim.apply(effects)
    sim.run(60)
    heard = sim.segments_heard()
    assert [h.text for h in heard] == list(SEGS[:2])
    assert heard[1].outcome == "cancelled"
    assert heard[1].spoken_chars == SEGS[1].index(".") + 1  # граница первого предложения
    assert heard[1].end - t_stop <= 3.0
    assert sim.music_stops() == [MusicStop("bar", "perform_end")]
    (end,) = sim.events("block_end")
    assert end.reason == "stopped" and sim.block.ended


def test_stop_before_any_line_cancels_the_pending_backing():
    sim = Sim(rap(), music_delay_s=3.0)
    sim.start()
    sim.run(0.5)
    sim.apply(sim.block.stop(sim.t))
    assert sim.block.ended and sim.music_stops() == [MusicStop("now", "perform_stop")]
    sim.run(5.0)  # подложка встала поздно — снять
    assert sim.music_stops()[-1] == MusicStop("now", "backing_late")
    assert sim.heard == []


def test_halt_cuts_speech_and_music_now():
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 2)
    seg2 = sim.block.lines.first("active")
    effects = sim.block.halt(sim.t)
    assert Cancel(seg2.speech_id, "none") in effects and MusicStop("now", "halt") in effects
    assert [e.reason for e in effects if getattr(e, "kind", "") == "block_end"] == ["halt"]
    sim.apply(effects)
    sim.run(30)
    assert len(sim.segments_heard()) == 2 and sim.segments_heard()[1].outcome == "cancelled"


def test_someone_elses_cancel_stops_the_performance_with_outro():
    """Реплику выступления отменил не этот модуль (перебивание v1): дальше не читаем, подложке outro."""
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 2)
    sim._cancel(Cancel(sim.block.lines.first("active").speech_id))
    sim.run(30)
    assert len(sim.segments_heard()) == 2 and sim.block.end_reason == "stopped"
    assert sim.music_stops() == [MusicStop("bar", "perform_end")]


# --- #993: «добавь бит» во время чтения -----------------------------------------------------------------------------

def test_993_add_beat_during_reading_next_segment_on_the_bar_speech_not_cut():
    sim = Sim(rap(backing="none"))
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 1)
    effects = sim.block.add_backing("beat", sim.t)
    assert [type(e).__name__ for e in effects] == ["Backing"]
    sim.apply(effects)
    sim.run(300)
    heard = sim.segments_heard()
    assert [h.text for h in heard] == list(SEGS) and all(h.outcome == "finished" for h in heard)
    synced = [e.synced for e in sim.events("segment_started")]
    assert synced == [False, True, True, True, True, True]
    assert all(on_bar(sim, h.start) for h in heard[1:])
    assert sim.music_stops() == [MusicStop("bar", "perform_end")]
    assert sim.block.add_backing("beat", sim.t) == []  # выступления уже нет


# --- подложка: продление и конец -----------------------------------------------------------------------------------

def test_nearly_finished_extends_the_backing_when_speech_is_longer():
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 1)
    sim.music_event("nearly_finished", "trk1", {"form_end_beat": 32 + 16, "lead_beats": 8})  # конец формы через 8 с
    (ext,) = [b for b in sim.backings() if b.extend]
    assert ext.min_s > 20


def test_nearly_finished_ignored_when_form_outlasts_speech():
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 1)
    sim.music_event("nearly_finished", "trk1", {"form_end_beat": 32 + 400, "lead_beats": 8})
    assert [b for b in sim.backings() if b.extend] == []


def test_backing_ending_early_continues_without_bars():
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 2)
    sim.music_event("finished", "trk1")
    sim.run(300)
    assert [h.text for h in sim.segments_heard()] == list(SEGS)
    assert [e.synced for e in sim.events("segment_started")][2:] == [False] * 4
    assert sim.music_stops() == []  # нечего снимать


def test_presynth_rejected_line_goes_with_synthesis_in_path():
    sim = Sim(rap(), fail_presynth=(SEGS[2],))
    sim.start()
    sim.run(300)
    assert [h.text for h in sim.segments_heard()] == list(SEGS)


# --- что видят валидатор и LLM -------------------------------------------------------------------------------------

def test_context_block_has_done_and_active_verbatim_and_first_pending_number():
    sim = Sim(rap())
    assert sim.block.active_segment == 0
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 2)
    ctx = context_block(sim.block, sim.t)
    assert ctx["done"] == [SEGS[0]] and ctx["active"] == {"no": 2, "text": SEGS[1]}
    assert ctx["first_pending"] == 3 and ctx["next_fixed"] == [] and ctx["segments_total"] == 6
    assert ctx["backing"] == "playing" and ctx["synced"] is True and ctx["remaining_s"] > 10
    assert sim.block.active_segment == 2
    sim.run(300)
    assert sim.block.active_segment is None


def test_lost_events_end_the_block():
    """Владелец речи молчит (ни synthesized, ни rejected): выступление не висит вечно — снимается по сроку."""
    block = PerformBlock(rap(backing="none"), "b1", log=lambda _: None)
    block.start(0.0)
    assert block.tick(kn.RESOURCE_STALE_S - 1) == []
    effects = block.tick(kn.RESOURCE_STALE_S + 1)
    assert block.ended and block.end_reason == "lost"
    assert [e.kind for e in effects if hasattr(e, "kind")][-1] == "block_end"


# --- голос и эмоция сегмента ---------------------------------------------------------------------------------------

def test_segment_voice_and_emotion_reach_the_speech_owner():
    """Сказка разными голосами (#3269): голос и эмоция сегмента — в запросе речи; голос робота это не меняет."""
    perf = Performance(kind="tale", segments=("Жил-был кот.", "А потом пришла лиса."),
                       voice_per_segment=("", "Wise_Woman"),
                       expression_per_segment=(Expression("happy"), Expression("neutral")))
    says = [e for e in PerformBlock(perf, "b1", log=lambda _: None).start(0.0) if isinstance(e, Say)]
    assert [(s.text, s.voice, s.emotion, s.commit) for s in says] == [
        ("Жил-был кот.", "", "happy", False), ("А потом пришла лиса.", "Wise_Woman", "", False)]


# --- пред-синтез, который уже не прозвучит, отменяется (PR-7c ч.3, Г4) --------------------------------------------

def test_replaced_and_dropped_lines_release_their_presynth():
    """Правка заменила сегменты, «хватит» снял не начатые: их отмена выбрасывает готовый PCM у владельца речи."""
    sim = Sim(rap())
    sim.start()
    sim.run(200, stop=lambda: sim.active_no() == 2)
    old_tail = {line.speech_id for line in sim.block.lines.lines if line.no >= 3}
    effects = sim.block.edit(Performance("rap", NEW, backing="beat", from_segment=3), sim.t)
    released = {e.speech_id for e in effects if isinstance(e, Cancel)}
    spare = {line.fallback.speech_id for line in sim.block.lines.lines if line.fallback is not None}
    assert released == old_tail - spare  # готовые прежние версии — запас на границу (О7), остальные — отменены
    sim.apply(effects)
    sim.run(200)
    released_later = {e.speech_id for _, e in sim.effects if isinstance(e, Cancel)}
    assert spare <= released_later  # запас, уступивший правке на границе, тоже отменён


def test_busy_music_is_not_retried_and_reading_goes_over_it():
    """Играет другая музыка (``busy``): вторая попытка бессмысленна — фраза и чтение без бита поверх неё."""
    sim = Sim(rap(), music=("rejected:busy", "ok"))
    sim.start()
    sim.run(300)
    assert len(sim.backings()) == 1
    assert sim.heard[0].text == phrases.text("perform.no_backing.busy")
    assert [h.text for h in sim.segments_heard()] == list(SEGS)
