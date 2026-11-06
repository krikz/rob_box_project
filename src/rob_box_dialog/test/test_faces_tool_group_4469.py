"""Срез тулов чата содержит пульт по лицам (#4469): слова про лица и id лица (8 hex); у голоса срез не меняется."""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools, groups_for

FACES = {"people_list", "person_photo", "name_person"}


def _names(text, agent):
    return {t.name for t in available_tools(Session(agent).snapshot, None, agent, text)}


@pytest.mark.parametrize("text", [
    "Задай для лица fff371fe имя Кирилл", "покажи лица без имени", "как зовут fff371fe", "fff371fe", "кто это на фото",
    "покажи людей", "дай кличку незнакомцу",
])
def test_chat_slice_has_faces_tools(text):
    assert FACES <= _names(text, "telegram")


def test_hex_id_opens_group_without_words():
    assert kn.FACES_GROUP in groups_for(Session("telegram").snapshot, "fff371fe", agent="telegram")


@pytest.mark.parametrize("text", ["покажи CPU", "расскажи анекдот", "feedback tomorrow", "привет"])
def test_unrelated_text_does_not_open_faces(text):
    assert not FACES & _names(text, "telegram")


@pytest.mark.parametrize("text", ["Задай для лица fff371fe имя Кирилл", "покажи лица без имени", "как зовут fff371fe"])
def test_voice_slice_unchanged(text):
    snap = Session("personality").snapshot
    assert not FACES & _names(text, "personality")
    assert kn.FACES_GROUP not in groups_for(snap, text, agent="personality")


def test_empty_faces_group_does_not_displace_default_status_for_voice():
    assert set(kn.TOOL_GROUPS["status"].tools) & _names("fff371fe", "personality")


def test_faces_group_keeps_priority_over_status_and_fits_cap():
    names = _names("покажи лица и сколько заряда", "telegram")
    assert FACES <= names and len(names) <= kn.TOOLS_VISIBLE_MAX
