"""#4217: старт сета с объявлением говорит название трека 1, а не общее «Включаю музыку.» (n605 марафона-5).

Фразу строит код по данным исхода ``dj_set`` (``opening``, ``tracks_text``), ключ - ``knowledge.DJ_DONE``. Примечание
пути (``source_note``: «Из промта не использую…», «играю своё») стоит перед ней. Стоп и старт без названия - прежние.
"""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases, respond
from rob_box_dialog.respond import ActionResult

START = "Включаю музыку."


def _result(data, status="done"):
    return ActionResult("t1/0", 1, "chat", "dj_set", kn.TOOL_CLASSES["dj_set"], status, None, data)


def _said(data):
    utterance = respond.phrase_from_result(_result(data))
    return None if utterance is None else utterance.text


def test_set_start_with_opening_names_track_one_not_generic_phrase():
    data = {"action": "start", "ok": True, "tracks": 3, "opening": "Рок дискотека у тёти Гриши",
            "tracks_text": "3 трека"}
    text = _said(data)
    assert "«Рок дискотека у тёти Гриши»" in text and "3 трека" in text
    assert START not in text


def test_every_variant_of_the_opening_phrase_names_the_track():
    for variant in range(len(phrases.catalog()["music.set_opening"])):
        text = phrases.text("music.set_opening", variant, opening="Гитарный шторм", tracks_text="5 треков")
        assert "«Гитарный шторм»" in text and "5 треков" in text


def test_set_start_without_opening_keeps_the_class_phrase():
    assert _said({"action": "start", "ok": True, "tracks": 3}) == START
    assert _said({"action": "start", "ok": True, "opening": ""}) == START


def test_path_note_still_precedes_the_opening_phrase():
    note = "Из промта не использую: темп 124 BPM — сет играет по теме и стилю."
    text = _said({"action": "start", "ok": True, "tracks": 3, "opening": "Гитарный шторм",
                  "tracks_text": "3 трека", "source_note": note})
    assert text.startswith(note) and "«Гитарный шторм»" in text and START not in text


def test_stop_phrases_are_unchanged():
    assert _said({"action": "stop", "was_playing": True}) == phrases.text("music.stop_ok")
    assert _said({"action": "stop", "was_playing": False}) == phrases.text("music.stop_idle")
    assert _said({"action": "stop", "was_playing": False, kn.STOPPED_EARLIER_FIELD: True}) == phrases.text("music.stop_ok")


def test_failed_start_is_not_an_opening():
    assert respond.phrase_key(_result({"action": "start", "opening": "X"}, status="rejected")) == "media.rejected"


@pytest.mark.parametrize("key", sorted(set(kn.DJ_DONE.all_keys())))
def test_all_dj_done_keys_exist_in_catalog(key):
    assert key in phrases.catalog()
