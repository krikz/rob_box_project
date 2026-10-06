"""Уточнение плана сета от LLM: схема, промпт, валидатор, применение (ADR-0149 §4.5, §4.7, §4.8; PR-10).

Чистая часть ризонера — без сети и ROS; сетевой клиент с дедлайном и circuit breaker живёт в процессе
плеера (``rob_box_mcp_tools.engine.reasoner``). LLM отвечает ОДНИМ вызовом ``submit_set_profile`` по JSON-схеме,
которую строит код из ``knowledge``: перечисления, а не свободный текст. Ответ проверяет :func:`validate`;
невалидно — ``PlanInvalid(path)``, сет играет seeded-план, ретраев нет (ADR-0148 §2.2, ADR-0143).

Что LLM может поменять (и только это):

* ``theme_row`` — строку закрытой таблицы тем (``knowledge.THEMES``) или ``none``: тема человека словами,
  которых нет в таблице («ночной город»), получает тембры и хуки ближайшей строки;
* ``mode`` — лад плана из окна жанра;
* ``hooks`` — до :data:`MAX_HOOKS` хуков только из кандидатов, которые показал код (:func:`hook_candidates`):
  нашёл код хуки по словам темы (#3399) — кандидаты ТОЛЬКО они (ADR-0152 §5), строки таблицы и пул — при
  пустых находках; ``theme_row`` хуки не подменяет. Хуки плана сета LLM не урезает (:func:`plan_hooks`): найденные
  по теме остаются все и в порядке кода, без находок выбор LLM идёт первым, пул по хешу — следом (06.10: план из
  трёх хуков LLM крутил 50 треков по кругу);
* ``energy`` — дугу энергии первых треков, 1..5 (поправка ``SetPlan``) в пределах длины сета, кроме последнего трека
  (спад, ``set_plan.arc_energy``); длину сета LLM не меняет; кульминация (5) обязана быть не позже
  ``Style.peak_by_track``-го трека (A6, #3459): дугу без пика в этом окне :func:`validate` отвергает
  (``PlanInvalid("energy")``, сет играет seeded-волну с пиком на 4-м треке) — LLM не может выключить кульминацию;

Реплики диджея на переходах — не здесь: факты каждого трека и фраза — ``dj_line`` (§12 В2, решение Шифу 06.10).

Темп в схеме отсутствует: сет уже звучит в темпе seeded-плана, один темп на сет (§4.4, I7) — :func:`apply`
его не трогает.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, replace
from typing import Any, Dict, List, Mapping, Optional, Tuple

from . import knowledge as kn
from .set_plan import DEFAULT_TRACKS, SetPlan, TrackPlan, track_energy
from .theme import ThemeProfile

#: Имя структурного выхода; это не тул исполнения — ризонер ничего не исполняет (ADR-0142 §9).
SUBMIT_TOOL = "submit_set_profile"
MAX_HOOKS = 3
_LOG = logging.getLogger(__name__)
NO_ROW = "none"
ENERGY_RANGE = (1, 5)


class PlanInvalid(ValueError):
    """Ответ LLM не прошёл схему; ``path`` — какое поле (в лог ``plan_invalid{path}``)."""

    def __init__(self, path: str, message: str) -> None:
        super().__init__(f"{path}: {message}")
        self.path = path


@dataclass(frozen=True)
class Refinement:
    """Проверенная поправка плана; ``row=None`` — тема вне таблицы (общий пул тембров)."""

    row: Optional[str]
    mode: str
    hook_ids: Tuple[str, ...]
    energy: Tuple[int, ...]


#: Ключ кандидатов профиля сета: хуки, которые код выбрал по словам темы (или пул по хешу).
SEEDED = "seeded"


def hook_candidates(profile: Optional[ThemeProfile] = None) -> Dict[str, Tuple[str, ...]]:
    """Хуки, из которых LLM выбирает. Код нашёл хуки по теме (``profile.theme_hooks``) — кандидаты только они
    (ADR-0152 §5, ADR-0148: решение кода, LLM берёт порядок и подмножество). Находок нет — строки таблицы тем
    и общий пул (``pool``)."""
    if profile is not None and profile.theme_hooks:
        return {SEEDED: profile.theme_hooks}
    seeded = {SEEDED: profile.hook_ids} if profile is not None and profile.hook_ids else {}
    return {**seeded, **{name: row.hooks for name, row in kn.THEMES.items()}, "pool": kn.DEFAULT_HOOKS}


def _all_hooks(profile: Optional[ThemeProfile] = None) -> List[str]:
    return list(dict.fromkeys(h for hooks in hook_candidates(profile).values() for h in hooks))


def schema(genre: str = "club", profile: Optional[ThemeProfile] = None) -> Dict[str, Any]:
    """JSON-схема ответа: перечисления из ``knowledge`` — одна таблица знания."""
    lo, hi = ENERGY_RANGE
    props: Dict[str, Any] = {
        "theme_row": {"type": "string", "enum": [*kn.THEMES, NO_ROW],
                      "description": "строка таблицы тем, ближайшая к теме человека; none — ни одна"},
        "mode": {"type": "string", "enum": list(kn.STYLES[genre].modes), "description": "лад сета"},
        "hooks": {"type": "array", "items": {"type": "string", "enum": _all_hooks(profile)}, "minItems": 1,
                  "maxItems": MAX_HOOKS, "description": "узнаваемые мелодии-хуки под тему, только из кандидатов"},
        "energy": {"type": "array", "items": {"type": "integer", "minimum": lo, "maximum": hi},
                   "minItems": 1, "maxItems": DEFAULT_TRACKS, "description": f"энергия треков 1, 2, … (1..5); пик 5 обязан быть не позже трека {kn.STYLES[genre].peak_by_track}"},
    }
    return {"type": "object", "properties": props, "required": ["theme_row", "mode", "hooks", "energy"],
            "additionalProperties": False}


def tool(genre: str = "club", profile: Optional[ThemeProfile] = None) -> Dict[str, Any]:
    """Схема как функция в формате OpenAI-совместимых провайдеров."""
    return {"type": "function", "function": {
        "name": SUBMIT_TOOL, "description": "Отдать профиль DJ-сета по теме. Вызвать ровно один раз.",
        "parameters": schema(genre, profile)}}


def prompt(theme: str, seeded: ThemeProfile) -> Tuple[str, str]:
    """``(system, user)``: правила выбора — в промпте, решение проверяет :func:`validate`."""
    system = ("Ты музыкальный редактор DJ-сета робота. Темп сета уже выбран кодом и не меняется. По теме "
              f"человека выбери строку таблицы тем, лад, до {MAX_HOOKS} хуков только из кандидатов "
              "ниже (если код нашёл хуки по теме, других нет; найденные код сыграет все по очереди) и дугу энергии: "
              "разгон, пик, спад; пик 5 — не позже "
              f"трека {kn.STYLES[kn.DEFAULT_STYLE].peak_by_track}, иначе дуга отвергается. Ответ — один вызов "
              f"{SUBMIT_TOOL}, без текста.")
    rows = "\n".join(f"- {name}: {', '.join(hooks)}" for name, hooks in hook_candidates(seeded).items())
    user = (f"Тема: «{theme}».\nБез тебя код выбрал: строка={seeded.row or NO_ROW}, лад={seeded.mode}, "
            f"хуки={', '.join(seeded.hook_ids)}.\nКандидаты хуков по строкам:\n{rows}")
    return system, user


def _enum(payload: Mapping[str, Any], key: str, allowed: Any) -> Any:
    value = payload.get(key)
    if value not in allowed:
        raise PlanInvalid(key, f"{value!r} не из {sorted(allowed)}")
    return value


def _hooks(value: Any, profile: Optional[ThemeProfile]) -> Tuple[str, ...]:
    allowed = set(_all_hooks(profile))
    """Выбор хуков LLM. Схема просит ``maxItems: MAX_HOOKS``, но MiniMax её не соблюдает, а порядок хуков сета решает
    код (:func:`plan_hooks`): лишние хуки не губят весь план (лад, энергию) — берутся первые :data:`MAX_HOOKS`
    (без повторов), остальное отбрасывается с записью в лог. Пустое, не список и неизвестные хуки — ``PlanInvalid``."""
    if not isinstance(value, list) or not value:
        raise PlanInvalid("hooks", f"нужен непустой список (до {MAX_HOOKS})")
    bad = [h for h in value if h not in allowed]
    if bad:
        raise PlanInvalid("hooks", f"не из кандидатов: {bad}")
    uniq = tuple(dict.fromkeys(value))
    if len(uniq) > MAX_HOOKS:
        _LOG.info("hooks обрезаны: %d→%d", len(uniq), MAX_HOOKS)
    return uniq[:MAX_HOOKS]


def _energy(value: Any) -> Tuple[int, ...]:
    lo, hi = ENERGY_RANGE
    if not isinstance(value, list) or not 0 < len(value) <= DEFAULT_TRACKS:
        raise PlanInvalid("energy", f"нужен список из 1..{DEFAULT_TRACKS}")
    if any(type(e) is not int or not lo <= e <= hi for e in value):
        raise PlanInvalid("energy", f"значения только целые {lo}..{hi}: {value}")
    return tuple(value)


def _peak(energy: Tuple[int, ...], by_track: int) -> None:
    """Кульминация в окне: энергия 5 среди первых ``by_track`` треков. Треки за концом дуги LLM играют по волне
    seeded (:func:`apply`), поэтому проверяется то, что реально прозвучит."""
    heard = [energy[no - 1] if no <= len(energy) else track_energy(no) for no in range(1, by_track + 1)]
    if ENERGY_RANGE[1] not in heard:
        raise PlanInvalid("energy", f"нет пика {ENERGY_RANGE[1]} в первых {by_track} треках: {list(energy)}")


def validate(payload: Any, genre: str = "club", profile: Optional[ThemeProfile] = None) -> Refinement:
    """Ответ LLM → :class:`Refinement` или ``PlanInvalid(path)``; лишних полей схема не допускает. ``profile`` —
    seeded-профиль сета: его хуки (найденные по теме) тоже кандидаты."""
    if not isinstance(payload, Mapping):
        raise PlanInvalid("$", f"ожидался объект, пришёл {type(payload).__name__}")
    extra = sorted(set(payload) - set(schema(genre)["properties"]))
    if extra:
        raise PlanInvalid(extra[0], "поля нет в схеме")
    row = _enum(payload, "theme_row", [*kn.THEMES, NO_ROW])
    mode = _enum(payload, "mode", kn.STYLES[genre].modes)
    energy = _energy(payload.get("energy"))
    _peak(energy, kn.STYLES[genre].peak_by_track)
    return Refinement(None if row == NO_ROW else row, mode, _hooks(payload.get("hooks"), profile), energy)


def plan_hooks(profile: ThemeProfile, ref: Refinement) -> Tuple[str, ...]:
    """Хуки плана после поправки — решает код (ADR-0148): нашёл код хуки по словам темы — все найденные в его
    порядке (у темы-перечисления — по кругу частей, ``search.round_robin``); выбор LLM их не урезает и не
    переставляет: три хука LLM из одной франшизы подряд ломают круг. Находок нет — хуки LLM первыми, затем
    пул по хешу темы (не короче seeded-плана)."""
    if profile.theme_hooks:
        return profile.hook_ids
    return tuple(dict.fromkeys((*ref.hook_ids, *profile.hook_ids)))


def apply(plan: SetPlan, ref: Refinement) -> SetPlan:
    """План сета с поправкой: тема/лад/хуки (:func:`plan_hooks`) и дуга энергии; темп, сид, тоника и свинг —
    прежние."""
    profile = replace(plan.profile, row=ref.row, mode=ref.mode, hook_ids=plan_hooks(plan.profile, ref))
    timbre = kn.THEME_TIMBRE.get(ref.row or "", plan.family)  # строка от LLM — её семья; none — прежняя семья сета
    # длина сета — решение кода (``seeded_plan(n_tracks)``): дуга LLM её не удлиняет и не трогает последний трек —
    # спад закрывает сет (``set_plan.arc_energy``); треки за концом дуги LLM — по seeded-дуге
    n, shaped = len(plan.tracks), min(len(ref.energy), len(plan.tracks) - 1)
    tracks = tuple(_with_energy(plan.track(no), ref.energy[no - 1]) if no <= shaped else plan.track(no)
                   for no in range(1, n + 1))
    return replace(plan, profile=profile, tracks=tracks, timbre=timbre)


def _with_energy(step: TrackPlan, energy: int) -> TrackPlan:
    """Трек с новой энергией; форма плана подбиралась под старую — при смене ``compose`` выберет её заново."""
    return step if energy == step.energy else replace(step, energy=energy, template="")


__all__ = ["MAX_HOOKS", "NO_ROW", "PlanInvalid", "Refinement", "SEEDED", "SUBMIT_TOOL", "apply",
           "hook_candidates", "plan_hooks", "prompt", "schema", "tool", "validate"]
