"""«Пришли их» без id - снимки последней группы лиц зовёт код (issue #4505, ADR-0148)."""

from rob_box_dialog import face_group
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult


def _done(tool, data, args=None):
    return ToolCall(tool, args or {}), ActionResult("t/0", 0, "telegram", tool, "query", "done", None, data)


def test_send_them_is_their_photos_only_without_an_id_in_the_utterance():
    assert face_group.asks_for_them("ну пришли их")
    assert face_group.asks_for_them("их снимки")
    assert face_group.asks_for_them("пришли их фото")
    assert not face_group.asks_for_them("пришли 71cda108")
    assert not face_group.asks_for_them("пришли ссылку")
    assert not face_group.asks_for_them("а ещё?")


def test_ids_come_from_a_done_people_list_and_a_done_photo_not_from_a_refusal():
    listed, list_result = _done("people_list", {"people": [{"id": "71cda108"}, {"id": "d7f642fc"}, {"name": "нет"}]})
    photo, photo_result = _done("person_photo", {"photo_id": "566331d3", "caption": "x"}, {"id": "566331d3"})
    refused = ActionResult("t/1", 0, "telegram", "person_photo", "query", "rejected", "no_snapshot", {})
    ids = face_group.ids_of(
        (listed, photo, ToolCall("person_photo", {"id": "ffffffff"})), (list_result, photo_result, refused)
    )
    assert ids == ("71cda108", "d7f642fc", "566331d3")


def test_a_saved_group_is_not_handed_to_another_source():
    group = face_group.FaceGroup("telegram", ("71cda108",))
    assert group.for_source("telegram") == ("71cda108",)
    assert group.for_source("voice") == ()


def test_missing_group_is_an_honest_refusal_not_a_send_promise():
    cmd, done = face_group.with_their_photos(
        "ну пришли их", Command(speech="Сейчас пришлю снимки.", actions=(ToolCall(kn.ALOUD_TOOL, {"text": "x"}),)), ()
    )
    assert done == "missing" and cmd.actions == () and "пришлю" not in (cmd.speech or "").lower()


def test_model_photo_call_is_left_alone():
    own = Command(actions=(ToolCall(kn.PHOTO_TOOL, {"id": "71cda108"}),))
    cmd, done = face_group.with_their_photos("ну пришли их", own, ("d7f642fc",))
    assert done == "" and cmd is own
