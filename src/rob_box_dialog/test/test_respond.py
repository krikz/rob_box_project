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


def test_sound_success_is_silent_failure_is_one_honest_phrase():
    """Шифу 21.10: «включил щелчок и ещё сказал, что он заиграл, бред же» — звук сам себе доказательство (§4.1)."""
    assert phrase_from_result(_result("play_sound", "done", sound="robot_happy")) is None
    assert join([_result("play_sound", "done", sound="robot_happy")]) is None
    busy = phrase_from_result(_result("play_sound", "rejected", "busy", sound="robot_happy"))
    assert busy.text == "Звук robot_happy не сыграл: другой звук ещё играет." and busy.emotion == "sad"
    assert phrase_from_result(_result("play_sound", "rejected", "superseded", sound="ui_bell")).text == (
        "Звук ui_bell не сыграл: следом попросили другой.")
    assert phrase_from_result(_result("play_sound", "timeout", sound="x")).text.startswith("Не дождался ответа")
    # «включи звук и поставь громкость»: говорится только то, что не звук
    assert join([_result("play_sound", "done", sound="robot_happy"), _result("set_volume", "done")]).text == "Готово."


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


def test_identical_outcomes_are_said_once():
    """Живой прогон 27.10 18:56 UTC: «сделай свой голос помягче» - ``set_pitch lower`` + ``set_speed slower``, оба
    ``local.done`` -> TTS говорил «Готово. Готово.». Одинаковые исходы - одна фраза."""
    said = join([_result("set_pitch", "done", action="lower"), _result("set_speed", "done", action="slower")])
    assert said.text == "Готово."
    assert said.emotion == "neutral"


def test_identical_outcomes_are_said_once_wherever_they_stand():
    """Одинаковость - по тексту, порядок - по первому появлению: «Готово.» между и после других исходов не повторяется."""
    results = [_result("set_volume", "done"), _result("request_music", "rejected"), _result("set_speed", "done")]
    assert join(results).text == "Готово. Не получилось включить музыку."
    twice = [_result("request_music", "rejected"), _result("request_music", "rejected")]
    said = join(twice)
    assert said.text == "Не получилось включить музыку." and said.emotion == "sad"


def test_outcomes_with_their_own_slots_are_all_said():
    """Фразы со слотами (факт, имя) различаются - схлопываются только одинаковые: два разных факта звучат оба."""
    said = join([_result("memory_save", "done", fact="Саша любит чай"), _result("memory_save", "done", fact="Нина любит кофе")])
    assert said.text == "Запомнил: Саша любит чай. Запомнил: Нина любит кофе."


def test_3974_result_data_named_like_text_parameters_cannot_crash_the_phrase():
    """n707: у трека gen-библиотеки есть поле ``variant`` -> ``TypeError: text() got multiple values for argument
    'variant'`` и ход падал без фразы. Данные результата - слоты, а не параметры ``text()``."""
    said = phrase_from_result(_result("navigate_to_waypoint", "pending", target="кухне", variant="b", key="k", seed=3))
    assert said.text == "Еду к кухне."


def test_3974_phrases_text_takes_any_slot_name():
    from rob_box_dialog import phrases
    assert phrases.text("memory.saved", fact="x", variant="v", key="k") == "Запомнил: x."
    assert phrases.text("memory.saved", 0, fact="x") == "Запомнил: x."
