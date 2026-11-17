"""#4060: ``track`` раздела ``<music_state>`` - короткое описание трека (его строит владелец плеера), а не номер и id."""

from types import SimpleNamespace

from rob_box_dialog.music_state import UNNAMED_TRACK, MusicStateMemory


def _snap(dj_info, track_id="gen48211:00:A", ts=100.0):
    """Снимок плеера с полями ``MusicPlayerState``, которые читает раздел (пакет диалога voice не импортирует)."""
    return SimpleNamespace(ts=ts, track_id=track_id, stops_at=None, finished_track_id=None, dj=bool(dj_info),
                           dj_info=dj_info, is_playing=lambda now=None: True)


def _track(dj_info, name=None):
    memory = MusicStateMemory()
    memory.observe_state(_snap(dj_info))
    if name:
        memory.observe_track_name(name)
    return memory.section(now=101.0)["track"]


def test_track_is_the_description_the_owner_built_not_the_title_with_numbers():
    info = {"brief": "«Ночной город», синтвейв", "title": "Очень странные дела · трек 3 из 10", "track_no": 3}
    track = _track(info)
    assert track == "«Ночной город», синтвейв"
    assert "gen48211" not in track and "трек 3" not in track


def test_snapshot_without_the_description_falls_back_to_the_title_then_to_unnamed():
    assert _track({"title": "космос · 120 BPM"}) == "космос · 120 BPM"
    assert _track({"brief": "", "title": ""}) == UNNAMED_TRACK
    assert _track({}) == UNNAMED_TRACK


def test_name_from_the_form_topic_still_wins():
    assert _track({"brief": "«Ночной город», синтвейв"}, name="Имя темы") == "Имя темы"
