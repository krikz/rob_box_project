"""Структуру выступления подгоняет код, а не отвергает (М11 05.11 n804_long_story, ADR-0148).

02:27:46 сказка: ``invalid:bad_segments '10 сегментов'`` - один из десяти сегментов пустой; на повторе модель
прислала роли не по числу сегментов (``role_per_segment не по числу сегментов``) - прозвучало «Не понял, как это
сделать», ход ``status=done``. Теперь пустой сегмент убирается вместе со своей ролью и лицом, длина ролей добивается
рассказчиком (``""``); отвергается только то, где после подгонки сегментов не осталось (или меньше минимума).
"""

import pytest

from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, Invalid, parse, validate

ALL = tuple(kn.TOOL_CLASSES)
POOL = cm.Busy(voices=("alena", "zahar", "filipp"),
               voice_genders=(("alena", "female"), ("zahar", "male"), ("filipp", "male")))
CAST = [{"name": "Волк", "gender": "male"}, {"name": "Лиса", "gender": "female"}]
#: Сказка n804 в десять сегментов: рассказчик и реплики двух героев; пятый сегмент пустой (как в логе М11).
TALE = ["Жил-был в лесу серый волк.", "Однажды он встретил хитрую лису.", "Куда спешишь, лиса?",
        "К реке, рыбу ловить.", "", "Возьми и меня с собой!", "Пошли они вместе к реке.", "Ну и как тут ловить?",
        "Хвост в прорубь опусти и жди.", "Так волк и остался без хвоста."]
ROLES = ["", "", "Волк", "Лиса", "Лиса", "Волк", "", "Волк", "Лиса", ""]


def _validate(perform):
    return validate(parse({"perform": {"kind": "tale", "cast": CAST, **perform}}), ALL, busy=POOL)


def test_n804_empty_segment_is_dropped_with_its_role():
    cmd = _validate({"segments": TALE, "role_per_segment": ROLES})
    assert isinstance(cmd, Command), cmd
    assert cmd.perform.segments == tuple(TALE[:4] + TALE[5:])
    assert cmd.perform.role_per_segment == tuple(ROLES[:4] + ROLES[5:])
    assert len(cmd.perform.voice_per_segment) == 9
    assert "perform.segments[10->9]" in cmd.dropped


def test_n804_retry_roles_shorter_than_segments_get_the_narrator():
    """Повтор n804: роли не по числу сегментов - недостающие говорит рассказчик, выступление идёт."""
    segments = [s for s in TALE if s]
    cmd = _validate({"segments": segments, "role_per_segment": ROLES[:7]})
    assert isinstance(cmd, Command), cmd
    assert cmd.perform.role_per_segment == tuple(ROLES[:7]) + ("", "")
    assert "perform.role_per_segment[7->9]" in cmd.dropped


def test_extra_roles_are_cut():
    cmd = _validate({"segments": TALE[:3], "role_per_segment": ["", "", "Волк", "Лиса", ""]})
    assert isinstance(cmd, Command), cmd
    assert cmd.perform.role_per_segment == ("", "", "Волк")


def test_faces_of_the_empty_segment_go_with_it():
    faces = [{"emotion": e} for e in ("neutral", "happy", "sad")]
    cmd = _validate({"segments": ["Жил-был волк.", "  ", "Конец."], "role_per_segment": ["", "Волк", ""],
                     "expression_per_segment": faces})
    assert isinstance(cmd, Command), cmd
    assert cmd.perform.segments == ("Жил-был волк.", "Конец.")
    assert [f.emotion for f in cmd.perform.expression_per_segment] == ["neutral", "sad"]
    assert cmd.perform.role_per_segment == ("", "")


@pytest.mark.parametrize("segments", [["", "  "], ["Жил-был волк.", ""], []])
def test_nothing_left_after_fitting_is_still_bad_segments(segments):
    res = _validate({"segments": segments, "role_per_segment": [""] * len(segments)})
    assert isinstance(res, Invalid) and res.reason == "bad_segments", res


def test_fitted_command_survives_a_second_validation_unchanged():
    first = _validate({"segments": TALE, "role_per_segment": ROLES[:6]})
    again = validate(first, ALL, busy=POOL)
    assert isinstance(again, Command) and again.perform == first.perform
