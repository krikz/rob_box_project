"""#4060: «что сейчас играет» - короткое описание трека из названия, стиля и темпа, а не номер и id (Шифу 27.10)."""

import pytest

from rob_box_music import dj_line as dl
from rob_box_music import knowledge as kn
from rob_box_music import track_brief as tb

#: Факты как в снимке плеера у трека сета из генерации: рядом с описанием лежат ключи кода - id, номер, метка заказа.
GEN_TRACK = {"name": "Ночной город", "style": "synthwave", "source": "generated", "bpm": 130.0,
             "track_id": "gen48211:00:A", "library_id": "a1b2c3", "set_id": "gen48211", "track_no": 3, "tracks": 10,
             "title": "Очень странные дела · трек 3 из 10"}


def test_track_with_title_and_style_is_named_by_title_style_and_tempo_without_id():
    track = {k: v for k, v in GEN_TRACK.items() if k != "source"}   # источник не назван: темп приписать некому
    brief = tb.track_brief({**track, "bpm": 120})
    assert brief == "«Ночной город», синтвейв, 120 ударов в минуту"
    for key_of_code in ("gen48211", "00:A", "a1b2c3", "трек 3", "из 10"):
        assert key_of_code not in brief


def test_track_without_title_is_told_by_style_and_tempo():
    assert tb.track_brief({"style": "club", "bpm": 130, "track_id": "gen48211:00:A"}) == (
        "трек в стиле клуб, 130 ударов в минуту")
    assert tb.track_brief({"bpm": 128, "source": "synth", "instrument_played": "organ"}) == ""   # без стиля описывать нечем
    assert tb.track_brief({"style": "jazz"}) == "трек в стиле джаз"


def test_nothing_to_tell_is_empty_not_an_invention():
    assert tb.track_brief({}) == ""
    assert tb.track_brief({"track_id": "gen48211:00:A", "title": "космос · 124 BPM", "track_no": 2, "tracks": 5}) == ""
    assert tb.track_brief({"style": "нет такого", "bpm": 0, "name": " "}) == ""


def test_generated_file_does_not_state_a_tempo_the_deck_does_not_know():
    """``bpm`` снимка у файла - темп деки (60 или темп сета), а не темп звучащего файла: темп не называем."""
    assert tb.track_brief(GEN_TRACK) == "«Ночной город», синтвейв"
    assert "ударов" not in tb.track_brief({**GEN_TRACK, "bpm": 60})
    assert tb.tempo_text({"source": "generated", "bpm": 130}) == ""
    assert tb.tempo_text({"source": "synth", "bpm": 130}) == "130 ударов в минуту"


def test_synth_track_names_its_engine_and_a_named_engine_is_kept():
    assert tb.track_brief({"style": "club", "bpm": 124, "source": "synth"}) == (
        "трек в стиле клуб, синтез Ренардо, 124 удара в минуту")
    assert tb.engine_word({"engine": "synth"}) == "синтез Ренардо" and tb.engine_word({"source": "piece"}) == ""


def test_4060_order_words_of_a_synth_track_are_not_its_title():
    """Живой 28.10 06:13: ``music_state.track`` = «какую нибудь спокойную музыку», клуб, синтез Ренардо, 120 ударов -
    слова заказа легли в кавычки как название. Свой синт-трек названия не имеет: стиль, движок, темп."""
    order = {"name": "какую нибудь спокойную музыку", "style": "club", "bpm": 120, "source": "synth"}
    assert tb.track_brief(order) == "трек в стиле клуб, синтез Ренардо, 120 ударов в минуту"
    assert tb.track_name(order) == "" and tb.is_untitled(order)
    assert tb.track_brief({**order, "name": "клубный трек"}) == tb.track_brief(order), "и «клубный трек» - не название"
    assert tb.track_brief({"title": "космос · 124 BPM", "style": "club", "source": "synth"}) == (
        "трек в стиле клуб, синтез Ренардо"), "метка деки «… · 124 BPM» названием не служит"


def test_4060_titled_sources_keep_their_title_and_a_set_track_keeps_its_melody():
    for source in ("generated", "piece", "melody"):
        assert tb.track_name({"name": "Ночной город", "source": source}) == "Ночной город"
        assert not tb.is_untitled({"source": source})
    assert tb.track_name({"name": "заказ", "melody": "Тетрис", "source": "synth"}) == "Тетрис"
    assert tb.track_brief({"melody": "Тетрис", "style": "chiptune", "bpm": 140, "source": "synth"}) == (
        "«Тетрис», чиптюн, синтез Ренардо, 140 ударов в минуту")


def test_cover_tells_its_original():
    assert tb.track_brief({"name": "Тихий вечер", "cover": "Группа крови", "style": "rock", "source": "generated"}) == (
        "«Тихий вечер», кавер на «Группа крови», рок")
    assert tb.track_brief({"name": "Группа крови (кавер)", "cover": "группа крови", "source": "generated"}) == (
        "«Группа крови (кавер)», кавер")
    assert tb.track_brief({"cover": "Группа крови", "style": "rock"}) == "кавер на «Группа крови», рок"


def test_piece_keeps_the_author_outside_the_quotes_and_the_instrument():
    piece = {"name": "Toccata", "composer": "Louis Vierne", "source": "piece", "bpm": 96, "instrument_played": "organ"}
    assert tb.track_brief(piece) == "«Toccata», Louis Vierne, 96 ударов в минуту, на органе"
    assert tb.track_brief({**piece, "name": "Toccata, Louis Vierne"}) == (
        "«Toccata, Louis Vierne», 96 ударов в минуту, на органе")


def test_melody_of_a_set_track_is_its_title_when_there_is_no_other():
    assert tb.track_brief({"melody": "Тетрис", "style": "chiptune", "bpm": 140}) == "«Тетрис», чиптюн, 140 ударов в минуту"
    assert tb.track_brief({"name": "Своё", "melody": "Тетрис"}) == "«Своё»"


@pytest.mark.parametrize("bpm,word", [(1, "удар"), (2, "удара"), (5, "ударов"), (11, "ударов"), (21, "удар"),
                                      (112, "ударов"), (122, "удара"), (130, "ударов")])
def test_tempo_agrees_with_the_number(bpm, word):
    assert tb.tempo_text({"bpm": bpm}) == f"{bpm} {word} в минуту"


def test_tables_agree_with_the_music_knowledge():
    assert set(tb.ENGINE_WORDS) <= set(kn.MUSIC_ENGINES)
    assert set(kn.STYLES) <= set(kn.GEN_STYLES)   # у каждого стиля синтеза есть имя словами
    assert not {"track_id", "library_id", "set_id", "track_no", "tracks", "title"} & set(tb.BRIEF_FACTS)


# -- ответ диджея «что играет» (rob_box_music.dj_line.now_playing_text) ----------------------------------------------

def test_dj_now_playing_answers_with_the_description_not_the_track_number():
    facts = {**GEN_TRACK, "melody": "", "next_melodies": [], "not_found": []}
    text = dl.now_playing_text(facts)
    assert text == "Сейчас играет «Ночной город», синтвейв."
    assert "3 из 10" not in text and "gen48211" not in text and "свой мотив" not in text


def test_dj_now_playing_of_an_own_motif_track_says_style_and_tempo_and_that_it_is_the_djs_motif():
    facts = {"track_no": 2, "tracks": 5, "melody": "", "style": "club", "bpm": 130, "source": "synth"}
    assert dl.now_playing_text(facts) == ("Сейчас играет трек в стиле клуб, синтез Ренардо, 130 ударов в минуту — "
                                          "свой мотив диджея, без мелодии из библиотеки.")


def test_dj_now_playing_keeps_the_plan_and_the_end_of_the_set():
    facts = {"track_no": 2, "tracks": 3, "melody": "Тетрис", "style": "chiptune", "bpm": 140, "source": "synth",
             "next_melodies": ["Аладдин"], "not_found": ["Марио"]}
    assert dl.now_playing_text(facts) == ("Сейчас играет «Тетрис», чиптюн, синтез Ренардо, 140 ударов в минуту. "
                                          "Дальше по плану: «Аладдин». Не нашлось в библиотеке: «Марио».")
    assert dl.now_playing_text({**GEN_TRACK, "track_no": 10}).endswith("Это последний трек сета.")


def test_dj_now_playing_with_nothing_to_describe_falls_back_to_the_number_honestly():
    assert dl.now_playing_text({"track_no": 2, "tracks": 4}) == "Сейчас трек 2 из 4."
    assert dl.now_playing_text({"track_no": 1, "tracks": 4, "melody": ""}) == (
        "Сейчас трек 1 из 4: свой мотив диджея, без мелодии из библиотеки.")
