"""Описание ``chimera_perform``, которое видит модель, согласовано с enum схемы (#5152, ADR-0148).

Каталог клипов показывает только ``call`` и не обещает пение. Форма синтеза показывает ``sing`` и
пение не отрицает. Поля чужих действий в схему не входят. Проверяется поведение модуля, не дословный
промпт: текст и поля строит ``chimera_offer`` из того же списка, что стоит в enum.
"""

from rob_box_core.tool_catalog import get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog.chimera_offer import ACTIONS, fields_for, perform_description
from rob_box_dialog.command import Form, tool_schema
from rob_box_dialog_host.dialog_v2_ports import tool_schema as model_schema


def _entry():
    return get_tool(kn.CHIMERA_TOOL)


def _shown(form):
    from rob_box_dialog.command import turn_form

    with turn_form(form):
        return model_schema(_entry())["function"]


def test_clips_schema_talks_only_about_call_and_hides_sing_fields():
    shown = _shown(Form())
    actions = shown["parameters"]["properties"]["action"]["enum"]
    assert actions == ["call"]
    assert shown["description"] == perform_description(actions)
    assert "sing" not in shown["description"]
    assert "пение" in shown["description"]  # отказ, не обещание: слова «пение» в отрицании есть
    names = set(shown["parameters"]["properties"])
    assert names == {"action", *fields_for(actions)}
    assert "syllables" not in names and "title" not in names and "timbre" not in names


def test_synth_form_offers_sing_and_does_not_deny_it():
    form = Form(sing="live:song", text="Робот, спой химерой песню про енота")
    shown = _shown(form)
    actions = shown["parameters"]["properties"]["action"]["enum"]
    assert actions == ["sing"]
    assert shown["description"] == perform_description(actions)
    assert "sing" in shown["description"]
    offered = shown["description"].split("сейчас это ", 1)[1].split(".", 1)[0]
    assert "пение" in offered and "не обещает" not in offered
    names = set(shown["parameters"]["properties"])
    assert names == {"action", *fields_for(actions)}
    assert {"title", "timbre", "syllables"} <= names
    assert "creature" not in names and "phrase_id" not in names


def test_validator_keeps_the_catalog_fields_outside_the_sing_form():
    """Вне хода валидатор не режет поля: ``sing`` — «не из enum», не «лишнее поле»."""
    shown = tool_schema(kn.CHIMERA_TOOL, Form())
    assert "title" in shown["properties"]
    assert shown["properties"]["action"]["enum"] == ["call"]
    assert perform_description(["call"]) != perform_description(list(ACTIONS))
