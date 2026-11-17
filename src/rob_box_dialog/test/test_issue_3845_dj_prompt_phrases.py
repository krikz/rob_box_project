"""#3845 (Шифу 22.10): сет по подробному промту готовился 46 с, а робот на 10-й секунде сказал «Не получилось включить
диджей-сет» - ложь (на 46-й сет заиграл). В v2 исход говорит код из РЕЗУЛЬТАТА: срок класса истёк - «Музыка пока не
заиграла.» (не провал), заигравший позже сет озвучивает поздний итог; фраза владельца ``source_note`` («Из промта не
использую: …») звучит перед фразой исхода."""

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.turn import say_late

from test_music_outcome_phrases_4075 import STARTED, Ports, _engine, _said

START = ToolCall("dj_set", {"action": "start", "theme": "Очень странные дела"})
NOTE = "Из промта не использую: темп 124.5 BPM, тональность, ноты и MIDI — сет играет по теме и стилю."


def test_set_still_preparing_at_the_class_deadline_is_not_said_to_have_failed():
    said = _said(START, ("timeout", None, {}), "Ты диджей Пауль Оакенфольд и у нас вечеринка на тему Очень странные дела")
    assert said == [phrases.text("media.timeout")]
    for failure in ("media.rejected", "music.dj_fail"):
        assert said[0] not in phrases.catalog()[failure]


def test_set_that_started_after_the_deadline_is_announced_by_the_late_outcome():
    ports = Ports()
    engine = _engine(ports)
    late = ActionResult("t1/0", 0, "personality", "dj_set", kn.TOOL_CLASSES["dj_set"], "done", None,
                        {"action": "start", "ok": True, "theme": "Очень странные дела"})
    say_late(engine, late)
    assert ports.said == [STARTED]


def test_what_the_prompt_asked_and_the_set_cannot_do_is_said_before_the_outcome():
    said = _said(START, ("done", None, {"ok": True, "theme": "Очень странные дела", "source_note": NOTE}))
    assert said == [f"{NOTE} {STARTED}"]


def test_set_without_a_note_says_only_the_outcome():
    assert _said(START, ("done", None, {"ok": True, "theme": "космос"})) == [STARTED]
