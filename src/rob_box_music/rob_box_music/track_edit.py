"""Правка ИГРАЮЩЕГО трека (#4029, ADR-0148: решает код, LLM говорит) — чистые функции, без ROS и Renardo.

Живой робот 27.10 (кейс Шифу): посреди рэпа под бит «сделай бит потяжелее, более металлический» шло в
``request_music(intent=track)`` → платная генерация, мост ``synth:breaks``, рэп без тактов и «Включаю музыку.»
посреди выступления; марафон 27.10: «ускорь», «добавь баса», «помрачнее, как в киберпанке» — новый трек 60 BPM или
ничего. Правка играющего — отдельный узкий путь, а не заказ нового:

* :func:`edits_in` — виды правки в словах человека (единственная таблица ``knowledge.TRACK_EDITS``);
* :func:`edit_route` — путь хода «правка играющего» для формы команды диалога v2: музыка играет, в реплике слова
  правки, а не заказ другой музыки (``knowledge.EDIT_NEW_ORDER_WORDS``);
* :func:`plan_edit` — что правка сделает с тем, что играет (:class:`EditTarget`): темп, семья тембров, бас — или
  честный отказ (готовая запись, пьеса, сет, темп под выступлением: ``knowledge.EDIT_REFUSALS``);
* :func:`edit_text` — фраза исхода из исхода, а не из слов модели.

Исполняет решение ``rob_box_mcp_tools.engine.track_edit`` (пересборка своего синт-трека и замена на такте).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import List, Sequence, Tuple

from . import knowledge as kn
from .model import Track
from .source import _words

#: Голова id пути хода «правка играющего»: ``edit:<вид>+<вид>`` (``edit:heavier+more_bass``).
EDIT_ROUTE = "edit"
#: Виды играющего (:attr:`EditTarget.kind`): свой синт-трек правится на ходу, остальное — честный отказ.
SYNTH, FILE, PIECE, SET, OTHER, NONE = "synth", "file", "piece", "set", "other", "none"


@dataclass(frozen=True)
class EditTarget:
    """Что играет. ``kind`` — :data:`SYNTH` (свой синт-трек, его можно пересобрать), :data:`FILE` (готовая запись: кэш
    или генерация), :data:`PIECE` (пьеса или мелодия по нотам), :data:`SET` (диджей-сет), :data:`OTHER`, :data:`NONE`
    (тишина); ``bpm`` — темп, ``style`` — стиль (``knowledge.STYLES``), ``timbre`` — семья тембров, ``bass_db`` —
    сдвиг баса прошлыми правками, ``backing`` — бит идущего выступления."""

    kind: str
    bpm: float = 0.0
    style: str = kn.DEFAULT_STYLE
    timbre: str = ""
    bass_db: float = 0.0
    backing: bool = False


@dataclass(frozen=True)
class EditPlan:
    """Решение правки: параметры трека после неё и исход по видам. ``applied`` — виды, что меняют звук; ``limited`` —
    упёрлись в предел (звук тот же); ``kept`` — «развивай»: играет дальше как есть; ``refused`` — ключ
    ``knowledge.EDIT_REFUSALS``: что не сделано и почему."""

    kinds: Tuple[str, ...]
    bpm: float = 0.0
    timbre: str = ""
    bass_db: float = 0.0
    applied: Tuple[str, ...] = ()
    limited: Tuple[str, ...] = ()
    kept: Tuple[str, ...] = ()
    refused: str = ""

    @property
    def changes_sound(self) -> bool:
        """Правка меняет звучащее (пересобрать трек и заменить на такте)."""
        return bool(self.applied)

    def limit_all(self) -> "EditPlan":
        """Пересобранный трек звучит так же, как играющий (бас уже на потолке ``amp``): всё сделанное — предел."""
        return EditPlan(
            self.kinds, self.bpm, self.timbre, self.bass_db, (), self.limited + self.applied, self.kept, self.refused
        )


def _matches(words: Sequence[str]) -> List[Tuple[int, int, str]]:
    """Все записи ``TRACK_EDITS`` в речи: ``(первое слово, слово после записи, вид)``."""
    found: List[Tuple[int, int, str]] = []
    for kind, edit in kn.TRACK_EDITS.items():
        for entry in edit.words:
            parts = entry.split()
            for i in range(len(words) - len(parts) + 1):
                hit = words[i].startswith(parts[0]) if len(parts) == 1 else list(words[i : i + len(parts)]) == parts
                if hit:
                    found.append((i, i + len(parts), kind))
    return found


def edits_in(text: str) -> Tuple[str, ...]:
    """Виды правки играющего в словах человека — по порядку в речи, без повторов. Запись длиннее побеждает перекрытую
    («поменьше баса» — меньше, а не больше баса); слово после отрицания («не быстрее, а громче») — не правка."""
    words = _words(text)
    taken: set = set()
    picked: List[Tuple[int, str]] = []
    for first, end, kind in sorted(_matches(words), key=lambda m: (m[0] - m[1], m[0])):
        if taken & set(range(first, end)):
            continue
        taken.update(range(first, end))
        if not (first and words[first - 1] in kn.EDIT_NEGATIONS):
            picked.append((first, kind))
    return tuple(dict.fromkeys(kind for _first, kind in sorted(picked)))


def new_order(text: str) -> bool:
    """Реплика заказывает другую музыку («поставь что-нибудь помрачнее», «включи другую»): слово правки там — каким
    быть новому треку, а не правка играющего."""
    return bool(set(_words(text)) & set(kn.EDIT_NEW_ORDER_WORDS))


def edit_route(text: str, playing: bool) -> str:
    """Путь хода «правка играющего» (``edit:<вид>+…``) или «»: музыка играет, в реплике слова правки, и это не заказ
    другой музыки. Решает код по снимку плеера и словам, а не модель (форма команды закрывает заказ нового)."""
    if not playing or new_order(text):
        return ""
    kinds = edits_in(text)
    return f"{EDIT_ROUTE}:{'+'.join(kinds)}" if kinds else ""


def route_edits(route: str) -> Tuple[str, ...]:
    """Виды правки из id пути (:func:`edit_route`); не путь правки — пусто."""
    head, _, tail = (route or "").partition(":")
    if head != EDIT_ROUTE:
        return ()
    return tuple(k for k in tail.split("+") if k in kn.TRACK_EDITS)


def next_family(style: str, current: str) -> str:
    """Следующая семья тембров стиля по кругу (``Style.timbres``): «смени тембр» без названной семьи."""
    families = tuple(kn.STYLES.get(style, kn.STYLES[kn.DEFAULT_STYLE]).timbres)
    at = families.index(current) if current in families else -1
    return families[(at + 1) % len(families)]


def _tempo(bpm: float, step: float) -> float:
    low, high = kn.BPM_RANGE
    return float(min(high, max(low, round(bpm * step))))


def _bass(db: float, step: float) -> float:
    low, high = kn.EDIT_BASS_DB_RANGE
    return min(high, max(low, db + step))


def _step(edit: kn.TrackEdit, style: str, sound: Tuple[float, str, float]) -> Tuple[float, str, float]:
    """Звук ``(темп, семья тембров, сдвиг баса)`` после одной правки — в пределах темпа и баса."""
    bpm, timbre, bass_db = sound
    if edit.tempo != 1.0:
        bpm = _tempo(bpm, edit.tempo)
    if edit.timbre:
        timbre = next_family(style, timbre) if edit.timbre == kn.EDIT_NEXT_TIMBRE else edit.timbre
    if edit.bass_db:
        bass_db = _bass(bass_db, edit.bass_db)
    return bpm, timbre, bass_db


def plan_edit(kinds: Sequence[str], target: EditTarget) -> EditPlan:
    """Что правка ``kinds`` сделает с играющим ``target``. Не свой синт-трек (готовая запись, пьеса, сет) или тишина —
    честный отказ (``refused``), звук не трогаем; видов нет — «не понял, что поменять». Темп бита выступления не
    меняется: сетка куплетов стоит на тактах этого темпа (``backing_tempo``), остальные виды правки — делаются."""
    kinds = tuple(k for k in dict.fromkeys(kinds) if k in kn.TRACK_EDITS)
    if target.kind != SYNTH:
        return EditPlan(kinds, refused=target.kind if target.kind in kn.EDIT_REFUSALS else OTHER)
    if not kinds:
        return EditPlan(kinds, target.bpm, target.timbre, target.bass_db, refused="unknown")
    sound = (target.bpm, target.timbre, target.bass_db)
    outcome: dict = {"applied": [], "limited": [], "kept": [], "refused": []}
    for kind in kinds:
        edit = kn.TRACK_EDITS[kind]
        if edit.group == "develop":
            outcome["kept"].append(kind)
        elif edit.tempo != 1.0 and target.backing:
            outcome["refused"].append("backing_tempo")
        else:
            after = _step(edit, target.style, sound)
            outcome["applied" if after != sound else "limited"].append(kind)
            sound = after
    return EditPlan(
        kinds,
        *sound,
        tuple(outcome["applied"]),
        tuple(outcome["limited"]),
        tuple(outcome["kept"]),
        next(iter(outcome["refused"]), ""),
    )


def _bass_room_db(track: Track) -> float:
    """Потолок уровня баса трека, дБ: потолок роли и сумма пиков секций с басом (``model._check_levels``)."""
    limit = 10.0 ** (kn.LEVEL_CEILINGS["master_peak_db"] / 10.0)
    room = kn.role_ceiling("bass")
    for sec in track.form.sections:
        if "bass" not in sec.roles:
            continue
        others = sum(10.0 ** (track.parts[r].level_db / 10.0) for r in sec.roles if r != "bass")
        room = min(room, 10.0 * math.log10(limit - others) if limit > others else -math.inf)
    return room


def with_bass(track: Track, db: float) -> Track:
    """Трек с басом громче/тише на ``db`` дБ (правка «добавь баса»): уровень партии и микса — одна ось громкости, не
    выше потолка роли и суммы пиков секций. Упёрлось в потолок — уровень на потолке (звук может не измениться: это
    видно по программе, ``engine.track_edit``)."""
    part = track.parts.get("bass")
    if not db or part is None:
        return track
    level = round(part.level_db + db, 2)
    if db > 0:
        level = max(part.level_db, min(level, math.floor(_bass_room_db(track) * 100) / 100))
    parts = {**track.parts, "bass": replace(part, level_db=level)}
    return replace(track, parts=parts, mix=replace(track.mix, level_db={**track.mix.level_db, "bass": level}))


def _sentence(parts: Sequence[str]) -> str:
    text = " и ".join(p for p in parts if p)
    return f"{text[:1].upper()}{text[1:]}." if text else ""


def edit_text(plan: EditPlan) -> str:
    """Фраза исхода правки (ADR-0148: из исхода, а не из слов модели): сделанное, предел, отказ — по порядку."""
    bpm = f"{plan.bpm:g}"
    done = [kn.TRACK_EDITS[k].done.format(bpm=bpm) for k in (*plan.applied, *plan.kept)]
    limits = [(kn.TRACK_EDITS[k].limit or kn.TRACK_EDITS[k].done).format(bpm=bpm) for k in plan.limited]
    refusal = kn.EDIT_REFUSALS.get(plan.refused, "")
    return " ".join(s for s in (_sentence(done), _sentence(limits), refusal) if s)


__all__ = [
    "EDIT_ROUTE",
    "EditPlan",
    "EditTarget",
    "FILE",
    "NONE",
    "OTHER",
    "PIECE",
    "SET",
    "SYNTH",
    "edit_route",
    "edit_text",
    "edits_in",
    "new_order",
    "next_family",
    "plan_edit",
    "route_edits",
    "with_bass",
]
