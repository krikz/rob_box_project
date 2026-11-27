"""#5235: ``register_speaker`` без имени — не успех, и «None» не звучит вслух.

Живой марафон 24.11: модель позвала ``register_speaker`` с пустыми аргументами (``{}``). Тул вернул
``success=True`` (``ask_required``, ``name=None``), исполнитель записал событие ``done``, и фраза кода
«Запомнил тебя, {name}» прозвучала как «Запомнил тебя, None.» — ``str.format(name=None)`` подставляет
слово «None».

Имя из ``self_intro_name`` молча не подставляется: это скрытый контекст гейта (как человек назвал себя
в реплике), а не аргумент тула. Догадка робота о личности — не регистрация (ADR-0148).
"""

from test_owner_event import _request, _tool

from rob_box_dialog import knowledge as kn
from rob_box_dialog.execute import owner_event
from rob_box_dialog.respond import ActionResult, phrase_from_result
from rob_box_dialog.turn import outcome_phrase


def _result(status, reason=None, **data):
    return ActionResult(
        "t/0",
        0,
        "personality",
        "register_speaker",
        kn.TOOL_CLASSES["register_speaker"],
        status,
        reason,
        data,
    )


def test_empty_name_is_not_a_done_event() -> None:
    """Пустые аргументы (``{}``) и ``name=None`` — отказ валидации, не ``done``."""
    asked = _tool(
        "не передано имя",
        is_error=True,
        data={
            "status": "rejected",
            "reason": "name_required",
            "error": "name_required",
        },
    )
    for args in (
        {},
        {"name": None},
        {"name": ""},
        {"name": "None"},
        {"self_intro_name": "Тамара", "known_speaker_name": "Нина"},
    ):
        event = owner_event(_request("register_speaker", args), asked)
        assert event.status == "rejected"
        assert event.reason == "name_required"
        assert event.data.get("name") in (None, "", "None") or "name" not in args


def test_success_phrase_is_never_built_from_a_missing_name() -> None:
    """Фраза «Запомнил тебя, X» — только из записанного имени. ``None``, пусто и литерал ``None`` в речь не попадают.

    Даже если событие ошибочно пришло ``done`` (старый тул): слот без имени фразу успеха не строит.
    """
    for name in (None, "", "   ", "None", "null"):
        said = phrase_from_result(_result("done", name=name))
        text = "" if said is None else said.text
        assert "None" not in text and "null" not in text.lower()
        assert not text.startswith("Запомнил тебя")
        assert outcome_phrase([_result("done", name=name)]) in (None, text)
        joined = outcome_phrase([_result("done", name=name)])
        assert joined is None or (
            "None" not in joined and not joined.startswith("Запомнил тебя")
        )


def test_recorded_name_still_says_remembered() -> None:
    """Записанное имя звучит как раньше: фраза кода из результата, не из догадки."""
    said = phrase_from_result(_result("done", name="Глеб"))
    assert said is not None and said.text == "Запомнил тебя, Глеб."
    assert outcome_phrase([_result("done", name="Глеб")]) == "Запомнил тебя, Глеб."


def test_self_intro_name_is_not_silently_spoken_as_the_registered_name() -> None:
    """``self_intro_name`` в аргументах вызова не становится слотом успеха: имя не передано — фразы «Запомнил» нет."""
    said = phrase_from_result(
        _result(
            "rejected",
            "name_required",
            self_intro_name="Тамара",
            known_speaker_name="Нина",
        )
    )
    text = "" if said is None else said.text
    assert "Тамара" not in text and "Запомнил тебя" not in text
    assert "None" not in text
