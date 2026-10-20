"""Фраза об исполнении строится из события владельца (ADR-0150 §5.2, §5.4): шаблон, а не слова LLM."""

import pytest

from rob_box_core.utterance import Sink
from rob_box_dialog import knowledge as kn
from rob_box_dialog.respond import ActionResult, join, phrase_from_result, phrase_key
from rob_box_dialog.session import Session


def _result(tool, status, reason=None, **data):
    return ActionResult("turn-1", 0, "personality", tool, kn.TOOL_CLASSES[tool], status, reason, data)


def test_memory_saved_comes_from_the_owner_event():
    """Ровно случай «Записала…» при пустых tools: фраза только по событию saved с данными владельца."""
    said = phrase_from_result(_result("memory_save", "done", fact="Саша любит зелёный чай"))
    assert said.text == "Запомнил: Саша любит зелёный чай."
    assert said.sink is Sink.SPEAKERS and said.emotion == "neutral"


def test_failure_has_reason_phrase_and_sad_emotion():
    said = phrase_from_result(_result("navigate_to_waypoint", "rejected", "no_motors"))
    assert said.text == "Не могу поехать — моторы выключены."
    assert said.emotion == "sad"
    assert phrase_from_result(_result("navigate_to_waypoint", "rejected", "unheard_of")).text == "Не могу поехать."


def test_motion_promise_and_query_data_go_their_way():
    assert phrase_from_result(_result("navigate_to_waypoint", "pending", target="кухне")).text == "Еду к кухне."
    assert phrase_key(_result("search_web", "done")) is None  # данные query — обратно LLM, не шаблон
    assert phrase_from_result(_result("search_web", "timeout")).text.startswith("Не дождался")


def test_operator_phrases_go_to_the_headset():
    result = ActionResult("t", 0, "operator", "say", "action.speak_aloud", "done", data={"seconds": 12})
    said = phrase_from_result(result, Session("operator").snapshot)
    assert said.text == "Озвучил (12 с)." and said.sink is Sink.HEADSET


def test_missing_slot_is_loud():
    with pytest.raises(KeyError):
        phrase_from_result(_result("memory_save", "done"))


def test_join_collects_turn_phrases_in_order():
    results = [_result("set_volume", "done"), _result("request_music", "rejected"), _result("get_current_time", "done")]
    said = join(results)
    assert said.text == "Готово. Не получилось включить музыку."
    assert said.emotion == "sad"
    assert join([_result("get_current_time", "done")]) is None
