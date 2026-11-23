"""#5080 (n810, марафон 22.11): рэп отвергнут целиком, потому что модель сложила строки куплета в один сегмент.

Промпт велит «каждая строка куплета — с новой строки», а ``_perform`` перевод строки не режет. Один сегмент с ``\\n``
внутри ниже ``SEGMENTS_MIN`` — ``bad_segments`` и «Не понял, как это сделать», хотя строк в тексте больше одной.
Код режет рэп и стих по строкам до проверки числа (:func:`rob_box_dialog.command._split_verse_lines`) только когда
сегментов меньше ``SEGMENTS_MIN`` — тем же приёмом, что :func:`_fit_segments` и :func:`_fit_expression_count`.
Уже валидное число сегментов не трогается: нарезка TTS и потолок ``SEGMENTS_MAX`` остаются на сегментах, не на строках.
``SEGMENTS_MIN`` не меняется: сегмент одной строкой по-прежнему честный отказ. Строки выдуманы, не из песен."""

from rob_box_core.tool_catalog import TOOL_CATALOG
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Busy, Command, Invalid, parse, validate

ALL = [t.name for t in TOOL_CATALOG]
LINES = ("Лапка топает по луже", "Хвост считает до пяти", "Нос чихает на капусту")


def _check(kind, segments, **extra):
    raw = {"perform": {"kind": kind, "backing": "none", "segments": segments, **extra}}
    return validate(parse(raw), ALL)


def _ok(kind, segments, **extra):
    res = _check(kind, segments, **extra)
    assert isinstance(res, Command), res
    return res


def test_5080_one_rap_segment_of_three_lines_is_accepted_as_three_segments():
    cmd = _ok("rap", ["\n".join(LINES)], backing="beat", expression_per_segment=[{"emotion": "happy"}])
    assert cmd.perform.segments == LINES
    assert len(cmd.perform.segments) >= kn.SEGMENTS_MIN
    assert [e.emotion for e in cmd.perform.expression_per_segment] == ["happy", "happy", "happy"]
    assert "perform.segments[1->3:lines]" in cmd.dropped


def test_5080_one_plain_rap_line_is_still_bad_segments():
    res = _check("rap", ["Одна строка без перевода"], backing="beat")
    assert isinstance(res, Invalid) and res.reason == "bad_segments"
    assert res.detail == "1 сегментов"


def test_5080_tale_and_song_keep_a_newline_inside_one_segment():
    verse = "\n".join(LINES[:2])
    song = _check("song", [verse, "Вторая реплика без перевода"])
    assert isinstance(song, Command), song
    assert song.perform.segments[0] == verse
    tale = _check("tale", [verse, "Вторая реплика без перевода"], cast=[], role_per_segment=["", ""])
    assert isinstance(tale, Command), tale
    assert tale.perform.segments[0] == verse
    assert not any(":lines]" in note for note in (*song.dropped, *tale.dropped))


def test_5080_valid_poem_is_not_split_and_keeps_the_role_of_its_second_segment():
    """Два сегмента — уже не ниже ``SEGMENTS_MIN``. Перевод строки внутри первого не режется, роль второго цела."""
    first = "\n".join(LINES[:2])
    cmd = _ok("poem", [first, "Третья строфа целиком"], cast=[{"name": "Заяц"}], role_per_segment=["", "Заяц"])
    assert cmd.perform.segments == (first, "Третья строфа целиком")
    assert cmd.perform.role_per_segment == ("", "Заяц")
    assert not any(":lines]" in note for note in cmd.dropped)


def test_5080_rap_of_three_segments_two_lines_each_is_not_split():
    """Форма фикстуры стенда: три уже валидных сегмента по две строки. Рез по строкам дал бы шесть и сменил бы TTS."""
    rights = ("Нос мокрый от дождя", "Ухо слушает грозу", "Кот спит на капусте")
    segments = [f"{left}\n{right}" for left, right in zip(LINES, rights)]
    cmd = _ok("rap", segments, backing="beat")
    assert cmd.perform.segments == tuple(segments)
    assert not any(":lines]" in note for note in cmd.dropped)


def test_5080_seven_by_two_stays_under_segments_max():
    """Семь сегментов по две строки (14 строк) валидны. Счёт строк упёрся бы в ``SEGMENTS_MAX`` и отверг команду."""
    segments = [f"строка {i} а\nстрока {i} б" for i in range(7)]
    cmd = _ok("rap", segments, backing="beat")
    assert cmd.perform.segments == tuple(segments)
    assert len(cmd.perform.segments) <= kn.SEGMENTS_MAX
    assert not any(":lines]" in note for note in cmd.dropped)


def test_5080_segments_max_still_rejects_too_many_lines():
    lines = [f"строка номер {i}" for i in range(kn.SEGMENTS_MAX + 1)]
    res = _check("rap", ["\n".join(lines)], backing="beat")
    assert isinstance(res, Invalid) and res.reason == "bad_segments"
    assert res.detail == f"{kn.SEGMENTS_MAX + 1} сегментов"


def test_5080_already_split_rap_is_untouched():
    cmd = _ok("rap", list(LINES), backing="beat")
    assert cmd.perform.segments == LINES
    assert not any(":lines]" in note for note in cmd.dropped)


def test_5080_blank_line_inside_a_verse_is_dropped_with_its_face():
    cmd = _ok("rap", ["Первая\n\nВторая"], backing="beat", expression_per_segment=[{"emotion": "happy"}])
    assert cmd.perform.segments == ("Первая", "Вторая")
    assert [e.emotion for e in cmd.perform.expression_per_segment] == ["happy", "happy"]


def test_5080_running_performance_still_accepts_one_new_line():
    raw = {"perform": {"kind": "rap", "backing": "beat", "segments": ["Один новый куплет"]}, "edit_pending": True}
    res = validate(parse(raw), ALL, busy=Busy(active_segment=1))
    assert isinstance(res, Command), res
    assert res.perform.segments == ("Один новый куплет",)
