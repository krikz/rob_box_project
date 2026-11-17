"""#4060: вопрос «выключить и зачитать?» называет играющее описанием из фактов трека, а не меткой заказа с номером."""

from types import SimpleNamespace

from rob_box_voice.core.plan_host import _foreign_music


def _node(dj_info, playing=True, dj=False):
    return SimpleNamespace(
        _music_player_state=SimpleNamespace(
            is_playing=lambda: playing, dj=dj, dj_info=dj_info
        )
    )


def test_question_names_the_track_by_its_description_not_by_the_order_label():
    info = {
        "brief": "«Ночной город», синтвейв",
        "title": "Очень странные дела · трек 3 из 10",
    }
    assert _foreign_music(_node(info)) == "«Ночной город», синтвейв"


def test_snapshot_without_description_keeps_the_title_then_a_plain_word():
    assert (
        _foreign_music(_node({"title": "поставь фоном ровный ритм · 125 BPM"}))
        == "поставь фоном ровный ритм · 125 BPM"
    )
    assert _foreign_music(_node({})) == "музыка"


def test_silence_has_no_name():
    assert _foreign_music(_node({"brief": "«Ночной город»"}, playing=False)) == ""
    assert _foreign_music(SimpleNamespace(_music_player_state=None)) == ""
