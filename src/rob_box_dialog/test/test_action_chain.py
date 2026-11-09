"""Вереница действий, этап 2 (ADR-0165 §12, #4403): чистый модуль ``action_chain`` и нарушение ``chain_mixed``.

Ход вереницу ещё не исполняет (этап 3): здесь - путь по оценке времени и тип задачи, лимиты, фраза плана, итог по
исходам шагов и валидатор команды."""

import pytest

from rob_box_dialog import action_chain as ac
from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.command import Command, Performance, ToolCall

TOOLS = [
    "delete_waypoint",
    "save_waypoint",
    "navigate_to_waypoint",
    "register_speaker",
    "clear_waypoints",
    "set_volume",
    "request_music",
    "list_waypoints",
    "say_aloud",
]


def delete(name):
    return ToolCall("delete_waypoint", {"name": name})


def trip(name):
    return ToolCall("navigate_to_waypoint", {"waypoint": name})


def chain(*calls):
    return ac.chain_of(calls)


def outcomes(ch, *statuses, notes=()):
    notes = dict(notes)
    return [ac.StepOutcome(step, status, notes.get(i, "")) for i, (step, status) in enumerate(zip(ch.steps, statuses))]


# --- §12 п. 1: путь по оценке времени и тип задачи ---------------------------------------------------------------


def test_one_delete_is_a_short_command():
    ch = chain(delete("хил-один"))
    assert ch.estimate_s == kn.ACTION_CLASSES["action.local"].timeout_s and not ch.long
    assert ch.task == kn.CHAIN_ERRAND


def test_estimate_over_five_seconds_is_a_chain():
    """До замеров HIL оценка - сроки классов: две точки по 3 с - уже 6 с (§4, верхняя граница)."""
    assert chain(delete("хил-один"), delete("хил-два")).long


def test_any_trip_is_a_chain_and_one_trip_is_an_errand():
    ch = chain(trip("кухня"))
    assert ch.long and ch.task == kn.CHAIN_ERRAND and ch.trips == 1


def test_more_than_one_trip_is_a_route():
    ch = chain(trip("кухня"), delete("хил-один"), trip("диван"))
    assert ch.task == kn.CHAIN_ROUTE and ch.trips == 2


def test_query_identity_and_perform_are_not_steps():
    ch = chain(ToolCall("list_waypoints", {}), ToolCall("register_speaker", {"name": "Саша"}), delete("хил-один"))
    assert [step.call.tool for step in ch.steps] == ["delete_waypoint"]


def test_measured_duration_wins_over_class_term(monkeypatch):
    monkeypatch.setattr(kn, "CHAIN_EXPECTED_S", {"action.local": 0.5})
    assert not chain(*(delete(f"т{i}") for i in range(4))).long


def test_speak_aloud_estimate_is_text_length_on_speech_rate():
    text = "а" * 110
    step = chain(ToolCall("say_aloud", {"text": text})).steps[0]
    assert step.expected_s == pytest.approx(110 / kn.SPEECH_CHARS_PER_S_GUESS)


# --- §12 п. 8: лимиты типа задачи --------------------------------------------------------------------------------


def test_route_over_eight_actions_is_over_limit():
    route = chain(*(trip(f"т{i}") for i in range(kn.CHAIN_ROUTE_ACTIONS_MAX + 1)))
    assert route.task == kn.CHAIN_ROUTE and ac.over_limit(route)
    assert not ac.over_limit(chain(*(trip(f"т{i}") for i in range(kn.CHAIN_ROUTE_ACTIONS_MAX))))


def test_errand_up_to_schema_limit_fits():
    assert not ac.over_limit(chain(*(delete(f"т{i}") for i in range(kn.ACTIONS_ACCEPT_MAX))))
    assert ac.over_limit(chain(*(delete(f"т{i}") for i in range(kn.ACTIONS_ACCEPT_MAX + 1))))


def test_term_limit_of_a_task(monkeypatch):
    tasks = dict(kn.CHAIN_TASKS)
    tasks[kn.CHAIN_ERRAND] = kn.ChainTask(steps_max=10, trips_max=1, term_s=5.0)
    monkeypatch.setattr(kn, "CHAIN_TASKS", tasks)
    assert ac.over_limit(chain(delete("а"), delete("б")))


# --- §12 п. 2: фраза плана ---------------------------------------------------------------------------------------


def test_code_plan_names_steps_by_verb_in_model_order():
    ch = chain(delete("хил-один"), delete("хил-два"), trip("кухня"))
    assert ac.code_plan(ch) == "Удаляю: хил-один, хил-два. Еду: кухня."


def test_code_plan_names_codes_and_nameless_steps_by_class_word():
    ch = chain(
        ToolCall("set_volume", {"action": "louder"}),
        ToolCall("clear_waypoints", {}),
        ToolCall("request_music", {"intent": "track", "text": "что-нибудь весёлое"}),
    )
    assert ac.code_plan(ch) == "Делаю: настройка. Включаю: что-нибудь весёлое."


def test_model_plan_is_spoken_when_it_claims_nothing():
    ch = chain(delete("хил-один"), delete("хил-два"))
    assert ac.plan(ch, "Сейчас уберу две точки.") == ("model", "Сейчас уберу две точки.")


@pytest.mark.parametrize("speech", ["Удалил четыре точки.", "Готово, хил-окно удалена.", "", None, "   "])
def test_false_or_empty_model_plan_falls_back_to_code(speech):
    ch = chain(delete("хил-один"), delete("хил-два"))
    assert ac.plan(ch, speech) == ("code", "Удаляю: хил-один, хил-два.")


def test_plan_claim_is_a_claim_even_with_the_tool_in_the_command():
    """У короткой команды «удалил» рядом с ``delete_waypoint`` проходит ``done_claim``; у плана - нет: до шага
    сделанного нет."""
    assert ac.plan_claims("Удалил четыре точки.") == ("delete",)


def test_code_plan_can_be_the_only_source(monkeypatch):
    monkeypatch.setattr(kn, "CHAIN_PLAN_BY", "code")
    assert ac.plan(chain(delete("а"), delete("б")), "Сейчас уберу.")[0] == "code"


# --- §12 пп. 4-7: итог по исходам шагов ---------------------------------------------------------------------------


def test_all_done():
    ch = chain(delete("хил-один"), delete("хил-два"))
    assert ac.summary(outcomes(ch, "done", "done")) == "Готово. Удалил: хил-один, хил-два."


def test_partial_names_owner_reason_after_missed():
    ch = chain(delete("хил-один"), delete("хил-три"))
    said = ac.summary(outcomes(ch, "done", "rejected", notes={1: "Такой точки нет."}))
    assert said == "Удалил: хил-один. Не удалил: хил-три. Такой точки нет."


def test_stop_between_steps_names_what_was_not_started():
    ch = chain(delete("хил-один"), delete("хил-два"), delete("хил-три"))
    said = ac.summary(outcomes(ch, "done", kn.CHAIN_STEP_SKIPPED, kn.CHAIN_STEP_SKIPPED), stopped=True)
    assert said == "Остановился. Удалил: хил-один. Не стал: хил-два, хил-три."


def test_failed_trip_stops_the_rest():
    ch = chain(trip("склад"), delete("хил-один"), trip("вход"))
    said = ac.summary(
        outcomes(ch, "rejected", kn.CHAIN_STEP_SKIPPED, kn.CHAIN_STEP_SKIPPED, notes={0: "Впереди препятствие."})
    )
    assert said == "Не доехал: склад. Впереди препятствие. Дальше не еду: хил-один, вход."


def test_survived_step_after_stop_counts_as_done():
    """§12 п. 7: непрерываемый шаг дожил до ``done`` после стопа - в итоге «удалил», а не «отменил»."""
    ch = chain(delete("хил-один"), delete("хил-два"))
    said = ac.summary(outcomes(ch, "done", kn.CHAIN_STEP_SKIPPED), stopped=True)
    assert said.startswith("Остановился. Удалил: хил-один.")


def test_chain_timeout_head():
    ch = chain(trip("кухня"), trip("диван"))
    said = ac.summary(outcomes(ch, "done", "timeout"), timed_out=True)
    assert said == "Не успел за отведённое время. Доехал: кухня. Не доехал: диван."


def test_outcome_statuses_are_executor_statuses_or_skipped():
    """``skipped`` - только у вереницы: исполнитель его не отдаёт (ADR-0165 §7)."""
    assert kn.CHAIN_STEP_SKIPPED not in kn.ACTION_STATUSES


# --- §12 п. 9: chain_mixed в валидаторе ---------------------------------------------------------------------------


def test_identity_next_to_a_long_chain_is_chain_mixed():
    cmd = Command(actions=(ToolCall("register_speaker", {"name": "Саша"}), delete("а"), delete("б")))
    res = cm.validate(cmd, TOOLS)
    assert isinstance(res, cm.Invalid) and kn.VIOLATION_CHAIN_MIXED in [v.reason for v in res.all]
    assert kn.CHAIN_MIXED_HINT in cm.explain(res)


def test_perform_next_to_a_route_is_chain_mixed():
    cmd = Command(actions=(trip("кухня"), trip("диван")), perform=Performance(kind="rap", segments=("йо", "йо")))
    assert ac.mixed(cmd) == ("action.perform",)


@pytest.mark.parametrize("call", [ToolCall("play_sound", {"sound": "robot_happy"}), trip("кухня")])
def test_one_long_step_next_to_perform_stays_as_before(call):
    """«Звук и шутка» (живой прогон 7c): один шаг - не вереница по шагам, хоть срок класса и больше 5 с."""
    cmd = Command(actions=(call,), perform=Performance(kind="tale", segments=("раз", "два")))
    assert ac.mixed(cmd) == ()


def test_identity_next_to_a_short_command_stays_as_before():
    cmd = Command(actions=(ToolCall("register_speaker", {"name": "Саша"}), ToolCall("save_waypoint", {"name": "а"})))
    assert ac.mixed(cmd) == ()
    assert not isinstance(cm.validate(cmd, TOOLS), cm.Invalid)


def test_query_next_to_a_long_chain_is_fine():
    cmd = Command(actions=(ToolCall("list_waypoints", {}), delete("а"), delete("б")))
    assert ac.mixed(cmd) == ()


def test_unknown_tool_is_left_to_its_own_violation():
    cmd = Command(actions=(ToolCall("drive", {}), delete("а"), delete("б")))
    reasons = [v.reason for v in cm.validate(cmd, TOOLS).all]
    assert kn.VIOLATION_CHAIN_MIXED not in reasons and "tool_not_available" in reasons


# --- §12 п. 11: полнота таблиц ------------------------------------------------------------------------------------


def test_every_step_class_has_a_noun():
    assert {cls for cls, rule in kn.CHAIN_CLASSES.items() if rule.step} == set(kn.CHAIN_CLASS_NOUNS)


def test_every_chain_phrase_renders():
    for key in kn.CHAIN_ITEM_PHRASES:
        assert phrases.text(key, items="а")
