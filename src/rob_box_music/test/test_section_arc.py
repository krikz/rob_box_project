"""Дуга громкости трека по секциям (``mix.section_arc``, ``knowledge.SECTION_TRIM_DB``)."""

from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import SECTIONS, compose
from rob_box_music.arrange.mix import section_arc
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile

PLAN = seeded_plan(ThemeProfile("тест", "club", 132, 9, "minor", (), None), 7, set_id="s7")


def test_every_club_section_has_a_trim_and_none_is_above_the_track():
    assert {name for name, *_ in SECTIONS} == set(kn.SECTION_TRIM_DB)
    assert max(offset for offset, _rise in kn.SECTION_TRIM_DB.values()) == 0.0
    assert kn.SECTION_TRIM_DB["intro"][0] == kn.SECTION_TRIM_DB["outro_tail"][0], "блэнд двух дек без скачка"


def test_arc_of_a_club_track_starts_each_section_and_ramps_the_build():
    track = compose(PLAN, 1, deck="A")
    arc = render(track, "A").arc
    assert arc == section_arc(track.form, track.bpm)
    assert [beat for beat, _o, _l in arc] == [sum(s.bars for s in track.form.sections[:i]) * 4.0
                                              for i in range(len(track.form.sections))]
    by_name = {sec.name: (offset, lag) for sec, (_b, offset, lag) in zip(track.form.sections, arc)}
    build_s = 8 * 4 * 60.0 / track.bpm
    assert by_name["build"] == (kn.SECTION_TRIM_DB["build"][0], round(build_s, 3))
    assert by_name["break"] == (kn.SECTION_TRIM_DB["break"][0], kn.TRIM_LAG_S)
    assert by_name["drop2"][0] == 0.0


def test_lead_sits_under_the_drums_and_bass():
    """A9 (приёмка 02.10): лид забивал середину в дропах; ударные слышны (отзыв эксперта 05.10)."""
    lv = kn.ROLE_LEVEL_DB
    assert lv["lead"] <= lv["kick"] - 12 and lv["lead"] <= lv["clap"] - 6
    assert lv["hats"] >= -58 and lv["clap"] >= -44
