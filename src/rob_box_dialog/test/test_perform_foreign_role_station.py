"""Реплика героя не из ``cast`` не валит выступление (колонка 07.11, ADR-0148: структуру подгоняет код).

16:19 «Игорёк, расскажи анекдотик»: ``cast`` [Мужик, Попугай], а в ``role_per_segment`` ещё «Продавец» —
``bad_cast``, повтор такой же, прозвучало «Не понял, как это сделать». Теперь герой вписывается в ``cast``, пока
героев не больше ``PERFORM_CAST_MAX``, дальше его реплики говорит рассказчик.
"""

from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, parse, validate

ALL = tuple(kn.TOOL_CLASSES)
POOL = cm.Busy(
    voices=("alena", "zahar", "filipp", "ermil", "jane"),
    voice_genders=(("alena", "female"), ("zahar", "male"), ("filipp", "male"), ("ermil", "male"), ("jane", "female")),
)
JOKE = [
    "Приходит мужик в зоомагазин.",
    "У вас есть говорящий попугай?",
    "Есть, вот он.",
    "Привет!",
    "Мужик ушёл с попугаем.",
]


def _validate(cast, roles, kind="tale"):
    return validate(
        parse({"perform": {"kind": kind, "cast": cast, "segments": JOKE, "role_per_segment": roles}}), ALL, busy=POOL
    )


def test_station_joke_foreign_hero_joins_the_cast():
    cast = [{"name": "Мужик", "gender": "male"}, {"name": "Попугай", "gender": "male"}]
    cmd = _validate(cast, ["", "Мужик", "Продавец", "Попугай", ""])
    assert isinstance(cmd, Command), cmd
    assert [r.name for r in cmd.perform.cast] == ["Мужик", "Попугай", "Продавец"]
    assert cmd.perform.role_per_segment == ("", "Мужик", "Продавец", "Попугай", "")
    assert "perform.cast+'Продавец'" in cmd.dropped
    assert len(cmd.perform.voice_per_segment) == len(JOKE)


def test_foreign_hero_over_the_cap_goes_to_the_narrator():
    cast = [{"name": f"Герой{i}", "gender": "male"} for i in range(kn.PERFORM_CAST_MAX)]
    cmd = _validate(cast, ["", "Герой0", "Продавец", "Герой1", ""])
    assert isinstance(cmd, Command), cmd
    assert cmd.perform.role_per_segment == ("", "Герой0", "", "Герой1", "")
    assert "perform.role_per_segment[2]='Продавец'->рассказчик" in cmd.dropped


def test_known_heroes_and_narrator_are_untouched():
    cast = [{"name": "Мужик", "gender": "male"}]
    cmd = _validate(cast, ["Рассказчик", "мужик", "", "Мужик", ""])
    assert isinstance(cmd, Command), cmd
    assert not any("perform.cast+" in note for note in cmd.dropped)
