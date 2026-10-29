"""«Эта песня» - ссылка на только что сочинённую, и подставляет её код, а не модель (#4216, ADR-0148).

Марафон-5 29.10 (n707/n708): робот сочинил «Енота-механика» (``[song] track=db2045…``), через полторы минуты человек
сказал «что за песня» и «включи эту песню про енота», а модель взяла ``track_id`` старой версии с тем же названием
из списка (``b90b61bf…``) - рассказала про вчерашний трек и включила его. Выбор трека по названию из списка - угадывание.

Правило кода: реплика называет песню указательным словом («эта/эту/этот … песню/трек») или «только что сочинённую», и
вызов ``music_library`` берёт id трека - тогда ему ставится ссылка ``track_ref=last_generated``. Сам id знает один владелец -
библиотека сочинённых треков (``GeneratedMusicLibrary.last_generated``: из двух вариантов заказа тот, что сыграл; свежесть - её
срок): тул подставляет его вместо ``track_id`` модели. Свежего сочинённого нет - ссылка пуста, ``track_id`` модели работает
как прежде (прежнее поведение). ``rate`` без id и так берёт играющий трек («этот трек нравится» про играющий) - ему ссылку
код не ставит; ``list``/``search`` id не берут.
"""

from __future__ import annotations

from types import MappingProxyType
from typing import Any, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import ToolCall
from rob_box_dialog.words import name_tokens

#: Аргумент тула и его значение «последний сочинённый» (то же, что ``MusicLibraryTool`` принимает в схеме).
REF_ARG = "track_ref"
LAST_GENERATED = "last_generated"
#: Действия тула, где нужен id трека и «эта песня» - это последний сочинённый (``rate`` и ``play`` играющего - нет).
REF_ACTIONS: Tuple[str, ...] = ("info", "play", "update", "delete")
#: Указательные слова («эта песня», «эту песню про енота»): целые, строчные, без «ё».
_THIS = ("эта", "эту", "этой", "этот", "этого", "этом")
#: Основы существительных песни/трека; начало слова.
_NOUNS = ("песн", "трек", "композиц", "мелоди", "инструментал")
#: Сколько слов между указательным и существительным допустимо («эту новую песню», «этот твой трек»); дальше -
#: уже другая фраза («на этой неделе включи песню»).
_GAP = 1
#: «Только что сочинённая»: ``только что`` + глагол создания в окне ``_JUST_WINDOW`` слов; основы.
_MADE = ("сочин", "сгенер", "написа", "созда", "приду", "сделал", "спел", "пропел")
_JUST_WINDOW = 4


def refers_to_last_generated(text: str) -> bool:
    """Реплика называет песню, которую робот сочинил только что: «эта/эту песню», «этот трек», «только что сочинённую»."""
    tokens = name_tokens(text)
    for i, token in enumerate(tokens):
        if token in _THIS and any(w.startswith(_NOUNS) for w in tokens[i + 1:i + 2 + _GAP]):
            return True
        if token == "только" and tokens[i + 1:i + 2] == ("что",):
            if any(w.startswith(_MADE) for w in tokens[i + 2:i + 2 + _JUST_WINDOW]):
                return True
    return False


def bind_last_generated(actions: Sequence[ToolCall], text: str) -> Tuple[ToolCall, ...]:
    """Вызовы с ссылкой ``track_ref=last_generated`` там, где реплика (``text``) говорит «эта песня» и действие берёт id
    трека (:data:`REF_ACTIONS`); остальные вызовы - как были. Что бы модель ни написала в ``track_id``, тул заменит его id
    свежего сочинённого трека; свежего нет - останется её ``track_id``."""
    if not any(a.tool == kn.LIBRARY_TOOL for a in actions) or not refers_to_last_generated(text):
        return tuple(actions)
    return tuple(_ref(a) if _takes_ref(a) else a for a in actions)


def _takes_ref(call: ToolCall) -> bool:
    """Вызов ``music_library`` с id трека, где выбор ещё не сделан: подтверждённое удаление (``confirm``) - ответ на вопрос
    о КОНКРЕТНОМ треке (``tool_ask``), «да, удаляй эту песню» его не меняет."""
    return call.tool == kn.LIBRARY_TOOL and call.args.get("action") in REF_ACTIONS and not call.args.get("confirm")


def _ref(call: ToolCall) -> ToolCall:
    args: Any = {**call.args, REF_ARG: LAST_GENERATED}
    return ToolCall(call.tool, MappingProxyType(args))


__all__ = ["LAST_GENERATED", "REF_ACTIONS", "REF_ARG", "bind_last_generated", "refers_to_last_generated"]
