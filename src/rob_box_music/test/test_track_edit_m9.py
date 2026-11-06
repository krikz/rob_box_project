"""Марафон М9 03.11, акт 5 «Живой музыкант» (run 41740604142): правка играющего на живых фразах STT.

n504: STT дал «робот продолжай развивать эту тему и не добавь баса пожирнее» — лишнее «не» перед повелительным
совершенного вида сняло бас и «пожирнее»: ``edit_music`` вызван, а сделано=[] («Играю дальше эту тему.»). n506: «роботы
ускорить потом мы тут засыпаем оба» — инфинитив вместо «ускорь» не начинает правку (``TrackEdit.lead``). n505 «сделай
атмосферу помрачнее, как в киберпанке» прошёл и должен остаться; заказ новой музыки — не правка.
"""

import pytest

from rob_box_music.track_edit import SYNTH, EditTarget, edit_route, edits_in, plan_edit

N504 = "продолжай развивать эту тему и не добавь баса пожирнее"
N506 = "роботы ускорить потом мы тут засыпаем оба"
TRACK = EditTarget(SYNTH, 134, "club", "warm", 0.0)


@pytest.mark.parametrize(
    "text,route",
    [
        (N504, "edit:develop+more_bass+heavier"),
        ("робот " + N504, "edit:develop+more_bass+heavier"),
        (N506, "edit:faster"),
        ("ускорить потом мы тут засыпаем оба", "edit:faster"),
        ("Робот, ускорить", "edit:faster"),
        ("Робот, быстрее", "edit:faster"),
        ("замедлить, а то не успеваем", "edit:slower"),
        # n505 и заказ нового — как были
        ("робот сделай атмосферу помрачнее как в киберпанке", "edit:darker"),
        ("Робот, включи другую", ""),
        ("Робот, поставь джаз", ""),
        ("Робот, включи другую, побыстрее", ""),
    ],
)
def test_m9_live_phrases_route_to_the_edit_of_the_playing_track(text, route):
    assert edit_route(text, playing=True) == route
    assert edit_route(text, playing=False) == ""


@pytest.mark.parametrize(
    "text,kinds",
    [
        # прошение-запрет — несовершенный вид («не добавляй», «не ускоряй»): отрицание по-прежнему снимает правку
        ("не добавляй баса", ()),
        ("не надо больше баса", ()),
        ("не ускоряй, мы тут засыпаем", ()),
        ("не быстрее, а громче", ()),
        # «не» перед повелительным совершенного вида — оговорка STT (запрет так не говорят): правка остаётся
        ("не добавь баса пожирнее", ("more_bass", "heavier")),
        ("не ускорь темп", ("faster",)),
    ],
)
def test_negation_before_a_perfective_imperative_is_an_stt_slip(text, kinds):
    assert edits_in(text) == kinds


def test_n504_plan_adds_bass_and_heaviness_and_keeps_the_theme():
    plan = plan_edit(edits_in(N504), TRACK)
    assert plan.applied == ("more_bass", "heavier") and plan.kept == ("develop",) and plan.changes_sound
