"""Эксперимент: чойсер вайба лица по короткому контексту (Шифу 17.11, #3780).

Не боевой путь. Боевой путь — код по событиям (шар, инерция, решение Шифу 05.11).
К ходу и к лицу не подключён. Здесь только проверка, что выбор по куску контекста
осмыслен: код собирает факты, которые уже есть (плеер, окно реплик, стейт,
текущий и предыдущий вайб, момент включения текущего из события смены),
промпт с критериями Шифу 21.10 и закрытый список имён из
:data:`.knowledge.FACE_SCENE_SLOTS` (``bg`` и ``fx``). Новых эффектов нет.

Выбор делает переданная функция. Сеть и LLM этот модуль не вызывает. Ответ не из
списка код не применяет: наружу уходит ``neutral``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional, Sequence, Tuple

from . import knowledge as kn
from .music_state import UNNAMED_TRACK
from .shift_log import FACE, VOICE, ShiftLog
from .turn_log import ASSISTANT, USER

#: Нет значимого вайба. Не слот рендера и не имя эффекта.
NEUTRAL = "neutral"
#: Слоты, чьи значения — вайб полотна: фон под чертами и разовый импульс.
VIBE_SLOTS: Tuple[str, ...] = ("bg", "fx")
#: Сколько последних реплик окна класть в контекст. Окна нет — фрагмента нет, не выдумываем.
LOG_LINES = 6
#: Приход в журнале смены старше этого — не «сейчас в кадре». Поля «сейчас в кадре» у диалога нет.
PRESENT_S = 60.0
#: Чего нет среди уже существующих фактов. Пишется в контекст, а не додумывается.
NO_DATA = "нет данных"
#: Тишина плеера — слово, которое просил Шифу, не «idle» и не пустая строка.
SILENCE = "тишина"
#: Критерии суждения — только слова Шифу 21.10. Соответствия «рок → fire» здесь нет.
#: ``plasma`` и ``rain`` — как он сказал: медленный фон и редкие капли, без жанра.
CRITERIA: Tuple[Tuple[str, str], ...] = (
    ("rainbow", "вайб"),
    ("fire", "разогрев"),
    ("matrix", "думаю, ищу, считаю"),
    ("slash", "удар, акцент"),
    ("glitch", "сбой, не понял"),
    ("boot", "пробуждение"),
    ("power_down", "засыпание"),
    ("plasma", "медленный фон"),
    ("rain", "редкие капли"),
)
#: Что сказать выбирающей функции. Критерии и варианты дописывает :func:`prompt`.
PROMPT = "По контексту выбери один вайб лица. Суди только по критериям ниже, не по жанру музыки. Ответь одним именем."


@dataclass(frozen=True)
class VibeChange:
    """Событие смены вайба: что включили, что было, когда включили текущий.

    ``previous`` пустой — предыдущего не было: в контексте «нет данных», не догадка.
    ``at`` — момент включения текущего, из этого события.
    """

    current: str
    at: float
    previous: str = ""


def changed(last: Optional[VibeChange], name: str, at: float) -> Optional[VibeChange]:
    """Событие смены: ``name`` включается в ``at``. Прежний current — previous; его не было — пусто.

    Пустое имя и повтор того же имени — не смена: момент включения не сдвигается.
    """
    name = str(name or "").strip()
    if not name:
        return last
    if last is not None and last.current == name:
        return last
    previous = last.current if last is not None and last.current else ""
    return VibeChange(current=name, previous=previous, at=float(at))


def slot_names(slots: Optional[Mapping[str, Sequence[str]]] = None) -> Tuple[str, ...]:
    """Имена слотов ``bg`` и ``fx``, кроме ``none``. Порядок — как в таблице знания."""
    table = slots if slots is not None else kn.FACE_SCENE_SLOTS
    names = []
    for slot in VIBE_SLOTS:
        for name in table.get(slot, ()):
            if name and name != "none" and name not in names:
                names.append(name)
    return tuple(names)


def criteria_text(names: Sequence[str]) -> str:
    """Критерии только тех имён, что есть в списке. Чужого соответствия нет."""
    known = {name: when for name, when in CRITERIA}
    lines = [f"{name} — {known[name]}" for name in names if name in known]
    return "\n".join(lines)


def prompt(context: str, names: Sequence[str]) -> str:
    """Промпт выбора: критерии, кусок контекста и закрытый список. Пустой контекст — пустая строка контекста."""
    listed = ", ".join([*names, NEUTRAL])
    how = criteria_text(names)
    return f"{PROMPT}\nКак судить:\n{how}\n{NEUTRAL} — ничего из этого.\nКонтекст: {context or ''}\nВарианты: {listed}"


def accept(answer: object, names: Sequence[str]) -> str:
    """Имя из списка, иначе ``neutral``. Регистр и крайние пробелы не считаются новым эффектом."""
    if not isinstance(answer, str):
        return NEUTRAL
    name = answer.strip().lower()
    allowed = {item.lower() for item in names}
    if name in allowed and name != NEUTRAL:
        return name
    return NEUTRAL


def _ask(picker: Callable[[str], object], text: str) -> object:
    """Сырой ответ выбирающей функции. Сбой функции — не выбор."""
    try:
        return picker(text)
    except Exception:
        return None


def choose(context: str, names: Sequence[str], picker: Callable[[str], object]) -> str:
    """Один вайб из ``names`` или ``neutral``.

    ``picker`` получает промпт и возвращает сырой ответ. Пустой список, пустой контекст
    и ответ вне списка не падают и не применяются.
    """
    allowed = tuple(dict.fromkeys(name for name in names if name and name != NEUTRAL))
    if not allowed:
        return NEUTRAL
    return accept(_ask(picker, prompt(context or "", allowed)), allowed)


def vibe_context(
    music: Any = None,
    log: Any = None,
    state: Any = None,
    *,
    now: Optional[float] = None,
    present_s: float = PRESENT_S,
    vibe: Any = None,
) -> str:
    """Кусок контекста из уже существующих фактов. Поля нет — строка «нет данных», не догадка.

    ``music`` — плеер (:class:`.music_state.MusicStateMemory`): играет имя трека или «тишина».
    ``log`` — журнал ходов (:class:`.turn_log.TurnLog`): последние реплики окна, если оно есть.
    ``state`` — снимок сессии и, если передан, журнал смены: кадры, речь, люди по приходам.
    Текущий вайб, предыдущий и момент включения — из события смены (:class:`VibeChange`):
    аргумент ``vibe`` или ``state.vibe``. Предыдущего не было — «нет данных».
    """
    change = _vibe_of(state, vibe)
    lines = (
        f"Музыка: {_music_line(music)}",
        f"Реплики: {_lines_of(log)}",
        f"Стейт: {_state_line(state, now, present_s)}",
        f"Текущий вайб: {_current_of(change)}",
        f"Предыдущий вайб: {_previous_of(change)}",
        f"Включён: {_since_of(change, now)}",
    )
    return "\n".join(lines)


def _vibe_of(state: Any, vibe: Any) -> Any:
    """Событие смены: явный аргумент, иначе поле ``state.vibe``. Нет — ``None``, не выдумываем."""
    if vibe is not None:
        return vibe
    return getattr(state, "vibe", None)


def _current_of(change: Any) -> str:
    """Текущий вайб из события смены. Имени нет — «нет данных»."""
    name = str(getattr(change, "current", "") or "").strip()
    return name or NO_DATA


def _previous_of(change: Any) -> str:
    """Предыдущий вайб из события смены. Смены или предыдущего не было — «нет данных»."""
    if change is None or not hasattr(change, "previous"):
        return NO_DATA
    name = str(change.previous or "").strip()
    return name or NO_DATA


def _since_of(change: Any, now: Optional[float]) -> str:
    """Момент включения текущего из ``at`` события смены. Поля нет — «нет данных»."""
    at = getattr(change, "at", None)
    if isinstance(at, bool) or not isinstance(at, (int, float)):
        return NO_DATA
    if now is None:
        return str(at)
    return f"{max(0, int(float(now) - float(at)))} с назад"


def _music_line(music: Any) -> str:
    """Что играет, из раздела плеера. Раздела нет — «нет данных», не играет — «тишина»."""
    section = _call(music, "section")
    if not isinstance(section, Mapping) or "playing" not in section:
        return NO_DATA
    if section.get("playing") != "yes":
        return SILENCE
    title = str(section.get("track") or "").strip()
    return f"играет {title or UNNAMED_TRACK}"


def _lines_of(log: Any) -> str:
    """Фрагмент окна реплик. Журнала или окна нет — «нет данных»."""
    window = _call(log, "window")
    if not isinstance(window, Sequence) or isinstance(window, (str, bytes)):
        return NO_DATA
    lines = [_reply_line(entry) for entry in window]
    kept = [line for line in lines if line][-LOG_LINES:]
    return " | ".join(kept) if kept else NO_DATA


def _reply_line(entry: Any) -> str:
    """Одна реплика окна: человек или робот. Тул и пустое — не реплика."""
    role = getattr(entry, "role", "")
    if role == USER:
        who = getattr(entry, "speaker", "") or "человек"
        return f"{who}: {_entry_text(entry)}"
    if role == ASSISTANT:
        return f"робот: {_entry_text(entry)}"
    return ""


def _entry_text(entry: Any) -> str:
    content = entry.for_llm() if hasattr(entry, "for_llm") else getattr(entry, "content", "")
    return str(content or "").replace("\n", " ").strip()


def _state_line(state: Any, now: Optional[float], present_s: float) -> str:
    """Стейт: кадры сессии, речь, люди. Снимка нет — «нет данных»."""
    snap = getattr(state, "snapshot", state)
    if snap is None or not hasattr(snap, "scenes"):
        return NO_DATA
    scenes = ", ".join(frame.kind for frame in getattr(snap, "scenes", ()) or ()) or "кадров нет"
    speaking = "речь идёт" if getattr(snap, "speaking", None) else "речь молчит"
    people = _people_line(state, now, present_s)
    return f"{scenes}; {speaking}; музыка сессии: {_session_music(snap)}; люди: {people}"


def _session_music(snap: Any) -> str:
    """Музыка снимка сессии, если поле есть. Нет поля — «нет данных»."""
    music = getattr(snap, "music", None)
    state = getattr(music, "state", None)
    if not state:
        return NO_DATA
    title = str(getattr(music, "title", "") or "").strip()
    if state == "playing":
        return f"играет {title}" if title else "играет"
    return SILENCE if state == "idle" else str(state)


def _people_line(state: Any, now: Optional[float], present_s: float) -> str:
    """Люди по журналу смены, если он передан. Поля «сейчас в кадре» нет — так и пишем."""
    shift = getattr(state, "shift", None)
    if not isinstance(shift, ShiftLog):
        return NO_DATA
    events = shift.events(now)
    at = max((float(event.until) for event in events), default=None)
    if now is not None:
        at = float(now)
    if at is None:
        return "пусто"
    return _who(events, at, present_s)


def _who(events: Sequence[Any], now: float, present_s: float) -> str:
    """Кто пришёл недавно: имя или «лицо»/«голос». Никого — «пусто»."""
    seen = []
    for event in events:
        if getattr(event, "kind", "") != "visit" or now - float(event.until) > present_s:
            continue
        seen.append(_who_label(event))
    return ", ".join(dict.fromkeys(seen)) if seen else "пусто"


def _who_label(event: Any) -> str:
    if getattr(event, "who", ""):
        return str(event.who)
    return "лицо" if FACE in str(event.detail).split(",") else "голос" if VOICE in str(event.detail) else "кто-то"


def _call(obj: Any, name: str) -> Any:
    """Метод есть — его результат. Нет объекта или метода — ``None``, поле не выдумываем."""
    fn = getattr(obj, name, None)
    if not callable(fn):
        return None
    try:
        return fn()
    except Exception:
        return None


__all__ = [
    "CRITERIA",
    "LOG_LINES",
    "NEUTRAL",
    "NO_DATA",
    "PRESENT_S",
    "PROMPT",
    "SILENCE",
    "VIBE_SLOTS",
    "VibeChange",
    "accept",
    "changed",
    "choose",
    "criteria_text",
    "prompt",
    "slot_names",
    "vibe_context",
]
