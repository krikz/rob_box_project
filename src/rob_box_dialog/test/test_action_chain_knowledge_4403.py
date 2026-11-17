"""Вереница действий, этап 1 (ADR-0165 §13, #4403): знание вереницы в ``knowledge`` полно и согласовано.

Ход эти данные ещё не читает (этапы 2-3): тест держит полноту таблиц (§12 п. 11) и то, что фраза плана кода не
утверждает сделанного (§3, §9.1)."""

import string

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.words import done_claims


def _slots(key):
    return {name for variant in phrases.catalog()[key] for _, name, _, _ in string.Formatter().parse(variant) if name}


def test_every_action_class_has_a_chain_rule():
    assert set(kn.CHAIN_CLASSES) == set(kn.ACTION_CLASSES)


def test_a_step_class_may_stand_next_to_a_chain():
    assert [cls for cls, rule in kn.CHAIN_CLASSES.items() if rule.step and not rule.alongside] == []


def test_chain_table_matches_adr_0165():
    """§4.1: query, знакомство и выступление шагами не бывают; рядом с вереницей - только query."""
    not_steps = {cls for cls, rule in kn.CHAIN_CLASSES.items() if not rule.step}
    assert not_steps == {"query", "action.identity", "action.perform"}
    not_alongside = {cls for cls, rule in kn.CHAIN_CLASSES.items() if not rule.alongside}
    assert not_alongside == {"action.identity", "action.perform"}
    assert {cls for cls, rule in kn.CHAIN_CLASSES.items() if rule.stops_rest} == {"action.motion"}


def test_single_per_chain_media_stays_single_per_command():
    """Музыка - одна на вереницу, как сейчас одна на команду (``SINGLE_ACTION_CLASSES`` остаётся, §4.1)."""
    assert kn.CHAIN_CLASSES["action.media"].single
    assert "action.media" in kn.SINGLE_ACTION_CLASSES


def test_expected_durations_name_only_step_classes():
    assert all(kn.CHAIN_CLASSES[cls].step for cls in kn.CHAIN_EXPECTED_S)
    assert kn.CHAIN_PLAN_AFTER_S == 5.0


def test_plan_source_is_known():
    assert kn.CHAIN_PLAN_BY in kn.CHAIN_PLAN_SOURCES


def test_task_limits():
    """Поручение - одна поездка и шагов не больше, чем примет схема; маршрут - до восьми действий (В9-а)."""
    errand, route = kn.CHAIN_TASKS[kn.CHAIN_ERRAND], kn.CHAIN_TASKS[kn.CHAIN_ROUTE]
    assert errand.trips_max == 1 and errand.steps_max == kn.ACTIONS_ACCEPT_MAX
    assert route.steps_max == kn.CHAIN_ROUTE_ACTIONS_MAX == 8 <= kn.ACTIONS_ACCEPT_MAX
    assert all(task.steps_max >= 1 and (task.trips_max or 1) >= 1 for task in kn.CHAIN_TASKS.values())


def test_trip_tools_are_motion_steps():
    assert all(kn.TOOL_CLASSES[tool] == "action.motion" for tool in kn.CHAIN_TRIP_TOOLS)


def test_verb_tools_are_chain_steps():
    for tool in kn.CHAIN_VERBS:
        assert kn.CHAIN_CLASSES[kn.TOOL_CLASSES[tool]].step, tool


def test_every_chain_phrase_is_in_the_catalog():
    assert [key for key in kn.CHAIN_PHRASES if key not in phrases.catalog()] == []
    assert set(kn.CHAIN_PHRASES) <= set(kn.PHRASE_KEYS)
    assert {key for key in phrases.catalog() if key.startswith("chain.")} == set(kn.CHAIN_PHRASES)


def test_chain_phrase_slots():
    for key in kn.CHAIN_PHRASES:
        assert _slots(key) == ({"items"} if key in kn.CHAIN_ITEM_PHRASES else set()), key


def test_code_plan_claims_nothing_done():
    """Фраза плана звучит до шагов: «удалил», «поехал» в ней - ложь (§3, §9.1, ``done_claim``)."""
    plans = {verb.plan for verb in (kn.CHAIN_VERB_DEFAULT, *kn.CHAIN_VERBS.values())}
    for key in plans:
        said = phrases.text(key, items="хил-один, хил-два")
        assert done_claims(said) == (), said


def test_done_phrases_do_claim_what_was_done():
    """Обратная сверка той же проверки: итог «Удалил: …» - утверждение, его говорит код только после событий."""
    assert done_claims(phrases.text(kn.CHAIN_VERBS["delete_waypoint"].did, items="хил-один")) == ("delete",)
