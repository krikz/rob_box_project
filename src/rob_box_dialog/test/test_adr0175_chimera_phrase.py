"""Фраза Химеры — из результата ``chimera_perform`` (ADR-0175 §2.1, ADR-0148), не из текста тула.

«Спел «Этюд».» — только когда пение или игра закончились (``sang=True``). Конец вызова — «Химера сказала»,
без слова «спел». Ещё играющее (``pending``) этим словом не хвалится.
"""

from rob_box_dialog import knowledge as kn
from rob_box_dialog import respond
from rob_box_dialog.respond import ActionResult


def _result(status, data, reason=None):
    return ActionResult(
        "t/0", 0, "personality", "chimera_perform", kn.TOOL_CLASSES["chimera_perform"], status, reason, data
    )


def test_sang_phrase_quotes_the_title_and_call_does_not_say_sang():
    sung = respond.phrase_from_result(_result("done", {"sang": True, "title": "Этюд", "action": "sing"}))
    assert sung is not None and sung.text == "Спел «Этюд»."
    heard = respond.phrase_from_result(
        _result("done", {"sang": False, "clip": "owl-hoot", "action": "call"}, "finished")
    )
    assert heard is not None and heard.text == "Химера сказала «owl-hoot»."
    assert "Спел" not in heard.text


def test_pending_chimera_does_not_say_sang():
    pending = respond.phrase_from_result(_result("pending", {"sang": False, "title": "Этюд", "action": "sing"}))
    assert pending is None or "Спел" not in pending.text
