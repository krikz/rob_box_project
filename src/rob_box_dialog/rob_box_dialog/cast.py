"""Роли выступления и их голоса (ADR-0148 «код решает, LLM говорит»).

Модель называет героев выступления (``perform.cast``: имя и пол) и у каждого сегмента — чья это реплика
(``perform.role_per_segment``, ``""`` — рассказчик). Голоса ролям даёт этот модуль, а не модель:

* у роли один голос на всё выступление, разных голосов звучит ровно столько, сколько ролей с репликами, плюс
  рассказчик — голосом робота (``""``);
* голос роли — первый свободный голос каталога ролей её пола, затем нейтральный, затем любой; каталог ролей приходит
  снаружи (``command.Busy``: реестр голосов звучащего провайдера без рассказчика и чужого языка - каст марафона
  нерусский, отдельного списка нет);
* песня и рэп ролей не имеют (``knowledge.PERFORM_ROLE_KINDS``): все куплеты — голосом робота. Живой случай 27.10:
  песня про енота шла двумя голосами через куплет — модель сама раскладывала ``voice_per_segment`` (Шифу: «каждый
  куплет разными голосами без причины — это очень напрягает; роли только когда нужны роли»);
* правка идущего выступления (``edit_pending``) роли не перекрашивает: голоса уже звучавших ролей (``fixed``) прежние.

Модуль не знает ни ROS, ни реестра голосов, ни команды: только имена, пол и голоса.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn

#: Каталог ролей: ``(голос, пол голоса)`` в порядке каталога; пол ``""`` — неизвестен.
Pool = Sequence[Tuple[str, str]]
#: Нарушение структуры ролей: ``(причина, подробность, допустимое)`` — валидатор команды делает из него ``Violation``.
Problem = Tuple[str, str, Tuple[str, ...]]

#: Причины нарушений (подсказки модели — ``command.HINTS``).
TALE_WITHOUT_CAST = "tale_without_cast"
BAD_CAST = "bad_cast"


@dataclass(frozen=True)
class Role:
    """Герой выступления: имя (им модель подписывает сегменты в ``role_per_segment``) и пол (``ROLE_GENDERS``;
    ``""`` — не задан, подойдёт любой голос)."""

    name: str
    gender: str = ""


def _norm(name: str) -> str:
    return " ".join(str(name or "").casefold().replace("ё", "е").split())


_NARRATORS = frozenset(_norm(name) for name in kn.PERFORM_NARRATOR_ROLES)


def role_key(name: str) -> str:
    """Ключ роли: без регистра, ``ё`` = ``е``, пробелы схлопнуты; рассказчик (``PERFORM_NARRATOR_ROLES``) — ``""``."""
    key = _norm(name)
    return "" if key in _NARRATORS else key


def has_roles(kind: str) -> bool:
    """У выступления этого вида бывают роли (сказка, анекдот, басня); песня и рэп — нет."""
    return kind in kn.PERFORM_ROLE_KINDS


def speaking(roles: Sequence[str]) -> List[str]:
    """Ключи ролей с репликами (не рассказчик) в порядке первой реплики."""
    out: List[str] = []
    for key in map(role_key, roles):
        if key and key not in out:
            out.append(key)
    return out


def _unclaimed(pool: Pool, taken: set) -> List[Tuple[str, str]]:
    """Голоса каталога (с полом), которых нет в ``taken``, в порядке каталога."""
    return [(voice, sex) for voice, sex in pool if voice and voice not in taken]


def _free(pool: Pool, gender: str, taken: set) -> str:
    """Первый свободный голос каталога: своего пола, затем нейтральный, затем любой; свободных нет — ``""``."""
    free = _unclaimed(pool, taken)
    wants: Tuple[Optional[str], ...] = (gender, kn.ROLE_VOICE_ANY_GENDER, None) if gender else (None,)
    for want in wants:
        hit = next((voice for voice, sex in free if want is None or sex == want), "")
        if hit:
            return hit
    return ""


def assign(cast: Sequence[Role], roles: Sequence[str], pool: Pool,
           fixed: Optional[Mapping[str, str]] = None) -> Dict[str, str]:
    """Голос каждой роли с репликами: уже звучавшая в этом выступлении (``fixed``) — прежний, новая — свободный голос
    каталога (:func:`_free`); голоса ролей не повторяются. Каталог кончился — ``""`` (голос рассказчика): валидатор
    такого не пропускает (:func:`problem`, ролей больше, чем голосов)."""
    fixed = dict(fixed or {})
    genders = {role_key(role.name): role.gender for role in cast}
    taken = set(fixed.values())
    out: Dict[str, str] = {}
    for key in speaking(roles):
        voice = fixed.get(key) or _free(pool, genders.get(key, ""), taken)
        out[key] = voice
        taken.add(voice)
    return out


def segment_voices(kind: str, cast: Optional[Sequence[Role]], roles: Optional[Sequence[str]], pool: Pool,
                   fixed: Optional[Mapping[str, str]] = None) -> Optional[Tuple[str, ...]]:
    """``voice_per_segment`` выступления — решение кода: песня и рэп, выступление без героев с репликами — ``None``
    (весь текст голосом робота); иначе голос роли сегмента, у рассказчика — ``""``."""
    if not has_roles(kind) or not roles or not speaking(roles):
        return None
    voices = assign(cast or (), roles, pool, fixed)
    return tuple(voices.get(role_key(name), "") for name in roles)


def voiced(roles: Optional[Sequence[str]], voices: Optional[Sequence[str]]) -> Dict[str, str]:
    """Роли выступления и их голоса (``ключ роли → голос``) по сегментам; рассказчик и голос робота — мимо."""
    return {role_key(name): voice for name, voice in zip(roles or (), voices or ()) if role_key(name) and voice}


def free_voices(pool: Pool, fixed: Optional[Mapping[str, str]] = None) -> List[str]:
    """Голоса каталога ролей, не занятые героями идущего выступления (``fixed``: ключ роли → голос): по ним код даёт
    голоса новым героям (:func:`assign`) и по ним же валидатор считает, хватит ли голосов (:func:`problem`)."""
    return [voice for voice, _ in _unclaimed(pool, set((fixed or {}).values()))]


def problem(kind: str, segments: int, cast: Optional[Sequence[Role]], roles: Optional[Sequence[str]],
            pool: Pool = (), fixed: Optional[Mapping[str, str]] = None) -> Optional[Problem]:
    """Нарушение структуры ролей выступления вида с ролями (``has_roles``) или ``None``: у сказки нет ``cast``; герои
    есть, а чья реплика в сегменте — нет; реплика роли не из ``cast``; героев больше ``PERFORM_CAST_MAX``; героев с
    репликами, которым ещё нужен голос, больше, чем свободных голосов каталога ролей (``pool``; пуст — каталог
    неизвестен, не сверяется). Голоса героев идущего выступления (``fixed``, ``Busy.cast``) заняты и из каталога
    вычитаются, а сами эти герои нового голоса не просят: правка выступления с новым героем при маленьком каталоге
    (Silero: четыре голоса ролей) иначе давала герою голос рассказчика."""
    if not has_roles(kind):
        return None
    if kind == "tale" and cast is None:
        return TALE_WITHOUT_CAST, f"{segments} сегментов без cast", ()
    names = tuple(dict.fromkeys(role.name for role in cast or () if role_key(role.name)))
    return _foreign_role(names, roles) or _too_many(names, roles or (), pool, fixed or {})


def _foreign_role(names: Tuple[str, ...], roles: Optional[Sequence[str]]) -> Optional[Problem]:
    """Герои есть, а чья реплика в сегменте — нет; реплика роли, которой нет в ``cast``."""
    if names and roles is None:
        return BAD_CAST, f"cast {list(names)} без role_per_segment", ()
    known = {role_key(name) for name in names} | {""}
    for i, name in enumerate(roles or ()):
        if role_key(name) not in known:
            return BAD_CAST, f"role_per_segment[{i}]={name!r} нет в cast", (*names, "")
    return None


def _too_many(names: Tuple[str, ...], roles: Sequence[str], pool: Pool,
              fixed: Mapping[str, str]) -> Optional[Problem]:
    """Героев больше ``PERFORM_CAST_MAX``; героев с репликами без голоса больше, чем свободных голосов каталога ролей
    (каталог пуст — не сверяется). Герой с голосом в ``fixed`` нового голоса не просит (:func:`assign`)."""
    heroes = len({role_key(name) for name in names})
    if heroes > kn.PERFORM_CAST_MAX:
        return BAD_CAST, f"героев {heroes}, можно не больше {kn.PERFORM_CAST_MAX}", ()
    needing = len([key for key in speaking(roles) if not fixed.get(key)])
    free = len(free_voices(pool, fixed))
    if pool and needing > free:
        held = " (остальные заняты героями идущего выступления)" if fixed else ""
        return BAD_CAST, f"героев с репликами {needing}, свободных голосов для них {free}{held}", ()
    return None


__all__ = ["BAD_CAST", "Pool", "Problem", "Role", "TALE_WITHOUT_CAST", "assign", "free_voices", "has_roles", "problem",
           "role_key", "segment_voices", "speaking", "voiced"]
