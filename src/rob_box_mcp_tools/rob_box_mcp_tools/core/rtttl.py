"""RTTTL (Nokia ringtone) → Renardo code.

RTTTL — машинный формат мелодий ``длительность+нота+октава``, по которому в
интернете лежат готовые точные ноты (Nokia ringtones, BLHeli-тона переводимы
в него). Парсер конвертирует их в Renardo ``degree + dur`` — детерминированно,
без ручного переписывания нот (корень ошибок #1810).

Формат: ``Name:d=4,o=5,b=140:8c5,8c5,8g5,...``
  * ``d`` — длительность по умолчанию (4 = четверть, 8 = восьмая…);
  * ``o`` — октава по умолчанию (4-7);
  * ``b`` — темп;
  * нота — ``[длительность][a-g/p][#|_][октава][.]``; ``p`` — пауза, ``.`` —
    точка (увеличивает длительность в полтора раза).

Альтерации: ``#`` и ``_`` — оба обозначают диез (в коллекциях PICAXE ``_`` —
историческая замена ``#``; это доказано парой файлов одной песни в двух
тональностях — V1 с ``_`` и V2 с ``#``, ноты совпадают со сдвигом на полутон).
"""

from __future__ import annotations

from rob_box_music.rtttl import parse_rtttl  # разбор перенесён в rob_box_music (ADR-0149 PR-3)

__all__ = ["ABSOLUTE_MIDI_ARGS", "parse_rtttl", "rtttl_to_renardo"]


#: Как передать АБСОЛЮТНЫЙ MIDI в Renardo.
#:
#: 🔴 FIX (live 14.09, «робот играет не ту мелодию»): здесь было
#: ``midinote=[...]`` — и высота не доезжала ВООБЩЕ. Renardo считает
#: ``freq``/``midinote`` из ``degree`` безусловно и затирает ими то, что
#: передал вызывающий (renardo_lib/Players.py::new_message_header:
#: ``message.update({'freq': freq, 'midinote': midinote})``, следом
#: ``event.update(message)``). ``degree`` при этом не передавался и был
#: равен 0, поэтому ЛЮБАЯ RTTTL-мелодия звучала как её ритм на одной
#: ноте MIDI 60. Парсер всё это время был исправен — не работал
#: последний шаг, и потому путь «точного воспроизведения темы» не
#: отработал ни разу.
#:
#: Рабочий канал — ступень ХРОМАТИЧЕСКОГО лада от нуля: в
#: ``renardo_lib/Scale.py::midi`` высота считается как
#: ``12*octave + root + scale[degree % 12]``, поэтому при
#: ``oct=0, root=0, scale=Scale.chromatic`` ступень численно РАВНА
#: MIDI-ноте. ``root=0`` здесь обязателен: без него ``Root.default``
#: (в аранжировке — ``var`` с прогрессией) транспонирует дословную тему
#: вслед за гармонией.
#:
#: Паузу даёт ``None`` именно в ``degree``: при ``degree is None``
#: Renardo получает ``freq=None`` и не шлёт ноту на сервер вовсе
#: (Players.py:1635, условие ``message["freq"] != None``). В ``midinote``
#: ``None`` не значил ничего.
ABSOLUTE_MIDI_ARGS = "oct=0, root=0, scale=Scale.chromatic"


def rtttl_to_renardo(
    rtttl: str,
    synth: str = "pianovel",
    amp: float = 0.5,
) -> str:
    """Собрать Renardo-код (готовый для ``execute_music_code``) из RTTTL.

    Ноты идут ПОЗИЦИОННЫМ аргументом (``degree``) вместе с
    :data:`ABSOLUTE_MIDI_ARGS`, а не через ``midinote=`` — почему именно
    так, см. комментарий к константе. Паузы — ``None`` в том же списке.
    Код начинается с ``Clock.clear()``, чтобы стереть предыдущий паттерн.
    """
    _name, bpm, notes = parse_rtttl(rtttl)
    degrees = [("None" if m is None else str(m)) for m, _ in notes]
    durs = [f"{d:g}" for _, d in notes]
    return (
        "Clock.clear()\n"
        f"Clock.bpm = {bpm}\n"
        f"p1 >> {synth}([{', '.join(degrees)}], {ABSOLUTE_MIDI_ARGS}, "
        f"dur=[{', '.join(durs)}], amp={amp})"
    )
