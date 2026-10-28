"""Длинное выступление (#4151, марафон 28.10): сказка n412 на 12 сегментов и счёт n802 на 11 после «Поехали!» не
начинались — пачка ``SpeechRequest`` «вступление + все сегменты» больше очереди ``/voice/speech/request`` (KEEP_LAST 10),
сегмент 1 до владельца речи не доходил, выступление ждало его ``synthesized`` вечно, а подложка продлевалась без речи.

Здесь — на фейковых владельцах (:mod:`perform_sim`) с той же очередью (``qos_depth=10``): пред-синтез идёт окном
``PERFORM_PRESYNTH_WINDOW`` и ни один запрос не теряется; сегменты звучат по порядку при любом их числе; короткие
выступления — как раньше (все сегменты в пред-синтез сразу); сторож речи снимает выступление, которое стоит без речи
дольше ``PERFORM_SILENCE_MAX_S``, — фраза кода, подложка сразу, без продлений."""

import pytest

from perform_sim import Sim

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Performance
from rob_box_dialog.plan.backing import MusicStop
from rob_box_dialog.plan.perform import PerformBlock, Say

QOS = 10  # глубина очереди /voice/speech/request у speech_client и подписки speech_owner (28.10)
BPM = 136  # подложка n412: такт 1.76 с


def _segments(n):
    return tuple(f"Часть {i}: жила-была девочка, звали её Красная Шапочка, и шла она лесом к бабушке." for i in
                 range(1, n + 1))


def tale(n, backing="calm", kind="tale"):
    return Performance(kind=kind, segments=_segments(n), backing=backing)


def _run(perf, **kw):
    sim = Sim(perf, bpm=BPM, qos_depth=QOS, **kw)
    sim.start()
    sim.run(600)
    return sim


def _presynth_bursts(sim):
    """Сколько запросов пред-синтеза хост отдал за один свой вызов (момент времени симуляции)."""
    bursts = {}
    for t, effect in sim.effects:
        if isinstance(effect, Say) and not effect.commit:
            bursts[t] = bursts.get(t, 0) + 1
    return bursts


# --- n412 / n802: длинное выступление начинается и идёт по порядку --------------------------------------------------

@pytest.mark.parametrize("n, kind", [(12, "tale"), (11, "poem")])
def test_long_performance_starts_after_lead_in_and_goes_in_order(n, kind):
    """Как n412 (сказка, 12 сегментов) и n802 (счёт, 11): MiniMax, подложка calm, очередь KEEP_LAST 10 — все запросы
    доходят, после «Поехали!» звучат все сегменты по порядку, конец ``done``."""
    sim = _run(tale(n, kind=kind), tts_provider="minimax")
    assert sim.lost == []
    assert sim.heard[0].text == phrases.text("perform.backing_started")
    assert [h.text for h in sim.segments_heard()] == list(_segments(n))
    assert [e.segment_no for e in sim.events("segment_started")] == list(range(1, n + 1))
    assert sim.block.end_reason == "done" and sim.events("block_end")[0].segment_no == n
    assert max(_presynth_bursts(sim).values()) <= kn.PERFORM_PRESYNTH_WINDOW + 1  # окно + «Поехали!»
    # такт старта (``at``) не уезжает вперёд на длинной форме: между репликами — хвост владельца и не больше такта
    gaps = [b.start - a.end for a, b in zip(sim.heard, sim.heard[1:])]
    assert max(gaps) <= kn.SPEECH_RELEASE_TAIL_S + 4 * 60.0 / BPM + 0.01


def test_long_performance_without_pause_markup_or_backing_also_goes_in_order():
    """Не MiniMax (пред-синтез сразу при старте) и без подложки: 12 сегментов — тоже окном и по порядку."""
    for perf, provider in ((tale(12), "yandex"), (tale(12, backing="none"), "minimax")):
        sim = _run(perf, tts_provider=provider)
        assert sim.lost == [] and sim.block.end_reason == "done"
        assert [e.segment_no for e in sim.events("segment_started")] == list(range(1, 13))


def _numbers(block, effects, commit):
    return [block.lines.find(e.speech_id).no for e in effects if isinstance(e, Say) and e.commit is commit]


def test_presynth_window_sends_the_next_segment_when_one_goes_to_sound():
    """Вперёд не больше окна; место освобождает реплика, отданная на звучание: следом за её ``commit=true`` — следующий
    по порядку сегмент в пред-синтез. Готовый пред-синтез места не освобождает: отмен «хватит» не больше окна."""
    block = PerformBlock(tale(12, backing="none"), "b1", log=lambda _: None)
    window = kn.PERFORM_PRESYNTH_WINDOW
    sent = [e for e in block.start(0.0) if isinstance(e, Say)]
    assert _numbers(block, sent, False) == list(range(1, window + 1))
    assert _numbers(block, block.on_speech("synthesized", sent[1].speech_id, 1.0, duration_ms=3000), False) == []
    effects = block.on_speech("synthesized", sent[0].speech_id, 1.1, duration_ms=3000)
    assert (_numbers(block, effects, True), _numbers(block, effects, False)) == ([1], [window + 1])
    block.on_speech("started", sent[0].speech_id, 1.2, duration_ms=3000)
    effects = block.on_speech("finished", sent[0].speech_id, 4.2)
    assert (_numbers(block, effects, True), _numbers(block, effects, False)) == ([2], [window + 2])


def test_without_window_the_queue_loses_segment_one_and_the_watchdog_ends_it_honestly(monkeypatch):
    """Модель очереди воспроизводит робота: окно шире очереди (как до #4151) — из 13 запросов доходят 11, сегмент 1
    потерян (n412). Выступление больше не висит: сторож снимает его с фразой, подложка — сразу."""
    monkeypatch.setattr(kn, "PERFORM_PRESYNTH_WINDOW", 100)
    sim = Sim(tale(12), bpm=BPM, qos_depth=QOS, tts_provider="minimax")
    sim.start()
    sim.run(300)
    lost = [sim.block.lines.find(e.speech_id) for e in sim.lost if isinstance(e, Say)]
    assert [line.no for line in lost] == [1, 2]
    assert [h.text for h in sim.segments_heard()] == []
    assert sim.block.end_reason == "stalled"
    assert sim.heard[-1].text == phrases.text("perform.stalled")


# --- короткие выступления и правки — как раньше -----------------------------------------------------------------------

def test_short_performance_presynthesizes_all_segments_at_once_as_before():
    """Сказка на 3 сегмента (n804) и счёт на 4 (n806): все сегменты в пред-синтез одной пачкой, звучат по порядку."""
    for n in (3, 4):
        block = PerformBlock(tale(n, backing="none"), "b1", log=lambda _: None)
        says = [e for e in block.start(0.0) if isinstance(e, Say)]
        assert [(block.lines.find(s.speech_id).no, s.commit) for s in says] == [(i, False) for i in range(1, n + 1)]
        sim = _run(tale(n), tts_provider="minimax")
        assert [h.text for h in sim.segments_heard()] == list(_segments(n)) and sim.block.end_reason == "done"


def test_edit_of_a_long_performance_goes_in_order_without_losses():
    """«И про волка добавь» во время сегмента 2 из 12: правка с сегмента 3, звучит 1, 2, затем правка — по порядку."""
    sim = Sim(tale(12), bpm=BPM, qos_depth=QOS, tts_provider="minimax")
    sim.start()
    sim.run(600, stop=lambda: sim.active_no() == 2)
    wolf = tuple(f"А тут волк {i}: серый, зубастый, бежит по тропинке наперерез." for i in range(1, 9))
    sim.apply(sim.block.edit(Performance("tale", wolf, backing="calm", from_segment=3), sim.t))
    sim.run(600)
    assert sim.lost == []
    assert [h.text for h in sim.segments_heard()] == [*_segments(2), *wolf]
    assert sim.block.end_reason == "done"


def test_stop_of_a_long_performance_cancels_every_sent_segment():
    """«Хватит» на сегменте 2 из 12: отмены доходят все (очередь отмен — та же глубина), конец ``stopped``."""
    sim = Sim(tale(12), bpm=BPM, qos_depth=QOS, tts_provider="minimax")
    sim.start()
    sim.run(600, stop=lambda: sim.active_no() == 2)
    sim.apply(sim.block.stop(sim.t))
    sim.run(60)
    assert sim.lost == [] and sim.block.end_reason == "stopped"
    assert len(sim.segments_heard()) == 2


# --- сторож речи -----------------------------------------------------------------------------------------------------

def _stalled(**kw):
    """Пред-синтез сегмента 1 не отвечает вовсе (провайдер завис): после «Поехали!» речи нет."""
    return Sim(tale(12), bpm=BPM, tts_provider="minimax", synth_never=(_segments(12)[0],), **kw)


def _nearly(sim):
    """Плеер: скоро конец формы подложки — выступление продлевает её, пока речь впереди."""
    sim.music_event("nearly_finished", "trk1", {"form_end_beat": 64.0})


def test_watchdog_ends_a_speechless_performance_with_a_code_phrase_and_stops_the_backing():
    sim = _stalled()
    sim.start()
    sim.run(60, stop=lambda: bool(sim.heard) and sim.heard[0].outcome == "finished")  # «Поехали!» прозвучало
    lead_in_end = sim.t
    _nearly(sim)
    assert [b.extend for b in sim.backings()] == [False, True]  # пока выступление живо — подложку продлевают
    sim.run(kn.PERFORM_SILENCE_MAX_S - 1)
    assert not sim.block.ended  # сторож не торопится: синтез длинного сегмента ещё мог идти
    sim.run(5)
    assert sim.block.ended and sim.block.end_reason == "stalled"
    end_t = next(t for t, e in sim.effects if getattr(e, "kind", "") == "block_end")
    assert end_t - lead_in_end == pytest.approx(kn.PERFORM_SILENCE_MAX_S, abs=0.2)
    assert sim.music_stops() == [MusicStop("now", "stalled", "trk1")]  # снимается свой бит, по id (#4085)
    (end,) = sim.events("block_end")
    assert (end.reason, end.segment_no, end.spoken_chars) == ("stalled", 0, 0)
    sim.run(30)
    assert sim.heard[-1].text == phrases.text("perform.stalled")  # исход говорит код
    _nearly(sim)
    assert [b.extend for b in sim.backings()] == [False, True]  # без речи подложку больше не продлевают
    assert any("сторож: речи нет" in line and "реплика 1 (pending, synthesized не пришёл)" in line
               for line in sim.logs)


def test_watchdog_waits_while_the_voice_speaks_someone_elses_reply():
    """Голос занят чужой речью (ответ хода, чтение сообщения) — наша реплика ждёт в очереди владельца речи, это не
    сбой: сторож отсчитывает тишину с её конца."""
    busy = {"on": True}
    sim = _stalled(voice_busy=lambda: busy["on"])
    sim.start()
    sim.run(3 * kn.PERFORM_SILENCE_MAX_S)
    assert not sim.block.ended
    busy["on"] = False
    freed = sim.t
    sim.run(3 * kn.PERFORM_SILENCE_MAX_S)
    assert sim.block.end_reason == "stalled"
    end_t = next(t for t, e in sim.effects if getattr(e, "kind", "") == "block_end")
    assert end_t - freed <= kn.PERFORM_SILENCE_MAX_S + 0.2


def test_watchdog_does_not_fire_on_slow_synthesis():
    """Медленный синтез (20 с на реплику, «Поехали!» тоже) — тишина меньше срока сторожа: дочитано целиком."""
    sim = Sim(tale(12), bpm=BPM, tts_provider="minimax", synth_s=lambda text: 20.0)
    sim.start()
    sim.run(900)
    assert sim.block.end_reason == "done" and len(sim.segments_heard()) == 12
