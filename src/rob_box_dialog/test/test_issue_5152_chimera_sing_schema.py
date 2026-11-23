"""Семья ``chimera`` не теряет ``sing``, каталог поставки остаётся ``call`` (#5152, #5074).

Диалог не читает ``sound_pack``: пакет ядра ставится без него. Форма хода сужает схему тула до
``sing`` — того же значения, что в :data:`knowledge.SING_FORMS`. Вне этой формы валидатор судит
каталог как есть, и ``sing`` в ``['call']`` не проходит. Второй таблицы клипов нет.
"""

from rob_box_core.tool_catalog import get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Form, parse, tool_schema, validate

SING = {"tool": kn.CHIMERA_TOOL, "args": {"action": "sing", "title": "енот"}}


def _chimera_form() -> Form:
    return Form(sing="live:song", text="Робот, спой химерой песню про енота")


def test_catalog_stays_call_and_the_chimera_form_accepts_sing():
    catalog = get_tool(kn.CHIMERA_TOOL).parameters["properties"]["action"]["enum"]
    assert catalog == ["call"]  # поставка клипов; sing в каталог не добавляли
    plain = validate(parse({"actions": [SING]}), [kn.CHIMERA_TOOL])
    assert plain.reason == "bad_args" and "'sing'" in plain.detail and "call" in plain.detail
    form = _chimera_form()
    assert form.sing_form is kn.SING_FORMS["chimera"]
    assert form.sing_form.tool_actions == ("sing",)
    shown = tool_schema(kn.CHIMERA_TOOL, form)
    assert shown["properties"]["action"]["enum"] == ["sing"]
    assert "creature" not in shown["properties"] and "call" not in shown["properties"]
    accepted = validate(parse({"actions": [SING]}), [kn.CHIMERA_TOOL], form=form)
    assert accepted.actions[0].args["action"] == "sing"
    assert get_tool(kn.CHIMERA_TOOL).parameters["properties"]["action"]["enum"] == ["call"]
