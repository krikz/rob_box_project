"""Смысл разового выступления Химерой и проверяемый исход (ADR-0175 §2.1, ADR-0148).

Код решает, что просили и чем это кончилось. Устройство, рендер, кэш и отмена принадлежат хосту: сюда он отдаёт
только стадии. «Спел» строится из исхода ``finished`` и ни из чего другого. Неизвестное название, неподдержанное
произношение или тембр, отмена, отсутствие файла и отсутствие сети — разные причины, не один отказ.

Мелодию по названию ищет действующая библиотека (``ScoreLibrary.work_candidates``), второй поиск не пишется.
Пакет шагов 0.1/0.2 может отсутствовать: профиль ниже — заглушка, не запись настоящих файлов Химеры.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple

from rob_box_mcp_tools.core.translit_ru import transliterate_ru
from rob_box_music.works import WorkRef, WorkWord

#: Действия одного инструмента (ADR-0175, «Два входа и один владелец воспроизведения»).
ACTIONS: Tuple[str, ...] = ("call", "speak", "sing", "play")
#: Пение и речь — только проверенные человеческие тембры. Список отражает профиль хоста, не желание модели.
TIMBRES: Tuple[str, ...] = ("male", "female", "child")
#: Короткий клип идёт путём ``play_sound``/``SoundEvent``; длинное действие — владельцу исполнения и его стопу.
ROUTE_CLIP, ROUTE_LONG = "clip", "long"
#: Стадии, которые хост вправе сообщить. ``started`` — ещё не «спел».
HOST_STAGES: Tuple[str, ...] = ("started", "finished", "skipped", "cancelled", "missing_file", "offline")
#: Причины, которые код отличает до обращения к хосту.
DECIDE_REASONS: Tuple[str, ...] = (
    "unsupported_action",
    "unsupported_pairing",
    "unsupported_pronunciation",
    "unsupported_timbre",
    "unknown_title",
    "ambiguous_title",
    "missing_file",
    "offline",
)
#: «Спел» — конец пения или игры. Вызов и речь звучат, но этим словом не называются (ADR-0175 §2.1).
SUNG_ACTIONS = ("sing", "play")
SANG = "chimera.sang"  # ключ фразы «спел» — только после фактического конца проигрывания
DONE_HEARD = "chimera.done"  # конец вызова или речи: звук был, слова «спел» нет

Library = Any  # ``ScoreLibrary``: ``work_candidates(WorkRef) -> Sequence[WorkCandidate]``; тесты подставляют заглушку


@dataclass(frozen=True)
class ChimeraProfile:
    """Что установленный хост умеет. Нет в профиле — честный отказ, а не попытка.

    ``manifest`` помечен ``stub``: шаги 0.1/0.2 ещё не привезли пакет. Настоящих файлов Химеры здесь нет.
    """

    actions: Tuple[str, ...] = ("call", "speak", "sing", "play")
    timbres: Tuple[str, ...] = TIMBRES
    creatures: Mapping[str, Tuple[str, ...]] = field(default_factory=dict)  # существо → разрешённые вызовы
    phrases: Mapping[str, str] = field(default_factory=dict)  # id подготовленной IPA-фразы → текст-заглушка
    calls: Mapping[str, str] = field(default_factory=dict)  # «существо:вызов» → id клипа профиля (не файл с диска)
    stub: bool = True
    note: str = "заглушка профиля: пакет шагов 0.1/0.2 отсутствует, файлов Химеры нет"

    def allows(self, action: str) -> bool:
        return action in self.actions

    def call_clip(self, creature: str, call_name: str) -> Optional[str]:
        if call_name not in self.creatures.get(creature, ()):
            return None
        return self.calls.get(f"{creature}:{call_name}")


def stub_profile() -> ChimeraProfile:
    """Профиль-заглушка для тестов и для робота, пока пакета 0.1/0.2 нет. Не выдавать за запись Химеры."""
    return ChimeraProfile(
        creatures={"сова": ("уханье",), "тюлень": ("писк",)},
        phrases={"privet": "привет"},  # подготовленная фраза, не произвольный русский (это шаг 3.1)
        calls={"сова:уханье": "clip:owl-hoot", "тюлень:писк": "clip:seal-squeak"},
    )


@dataclass(frozen=True)
class PieceChoice:
    """Что поиск библиотеки сказал о названии: одна пьеса, несколько или ничего."""

    material_id: str = ""
    title: str = ""
    composer: str = ""
    variants: Tuple[Tuple[str, str, str], ...] = ()  # (material_id, title, composer)


@dataclass(frozen=True)
class ChimeraPlan:
    """Решение кода до хоста: играть (``play``) или отказать (``refuse``) с различимой причиной."""

    kind: str  # "play" | "refuse"
    action: str
    reason: str = ""
    route: str = ""
    clip: str = ""
    material_id: str = ""
    title: str = ""
    composer: str = ""
    timbre: str = ""
    syllables: str = ""
    variants: Tuple[Tuple[str, str, str], ...] = ()
    request: Mapping[str, str] = field(default_factory=dict)
    stub: bool = False  # профиль-заглушка: клипа на диске нет, публиковать clip:* нельзя

    @property
    def plays(self) -> bool:
        return self.kind == "play"


@dataclass(frozen=True)
class ChimeraOutcome:
    """Проверяемый исход одного запроса. Фраза — из него, не из текста модели."""

    status: str  # done | rejected | cancelled | pending | timeout
    reason: str
    action: str
    request_id: str = ""
    material_id: str = ""
    title: str = ""
    composer: str = ""
    timbre: str = ""
    route: str = ""
    clip: str = ""
    variants: Tuple[Tuple[str, str, str], ...] = ()
    sang: bool = False  # True только когда проигрывание фактически закончилось
    stub: bool = False  # план шёл по профилю-заглушке: файла Химеры нет

    def as_data(self) -> Dict[str, Any]:
        return {
            "status": self.status,
            "reason": self.reason,
            "action": self.action,
            "request_id": self.request_id,
            "material_id": self.material_id,
            "title": self.title,
            "composer": self.composer,
            "timbre": self.timbre,
            "route": self.route,
            "clip": self.clip,
            "variants": [{"material_id": m, "title": t, "composer": c} for m, t, c in self.variants],
            "sang": self.sang,
            "stub": self.stub,
        }


def title_ref(title: str) -> WorkRef:
    """Название словами человека → ``WorkRef`` для ``ScoreLibrary.work_candidates``.

    ``latin`` — транслит: индекс партитур хранит звуковые ключи латинских слов (``score_library._keys``), а
    ``_phrase`` ищет написание как есть. Кириллица без транслита не пересекается с индексом. Композитора отдельно
    нет: его выделит сам поиск, если слово чаще стоит в поле композитора, чем в названиях.
    """
    words = tuple(WorkWord((), (transliterate_ru(w.lower()),), w) for w in title.split() if w.strip())
    return WorkRef((), words, "")


def choose_piece(library: Optional[Library], title: str) -> PieceChoice:
    """Одна пьеса, несколько или ничего — из ``work_candidates``. Библиотеки нет — ничего (не выдумываем)."""
    if library is None or not title.strip():
        return PieceChoice()
    found = list(library.work_candidates(title_ref(title)))
    if not found:
        return PieceChoice()
    if len(found) == 1:
        one = found[0]
        return PieceChoice(one.id, one.title, one.composer)
    return PieceChoice(variants=tuple((c.id, c.title, c.composer) for c in found))


def _plan(profile: ChimeraProfile, *args: Any, **kwargs: Any) -> ChimeraPlan:
    """План с пометкой заглушки профиля: ветки ``decide`` её не дублируют."""
    return ChimeraPlan(*args, stub=profile.stub, **kwargs)


def decide(
    action: str,
    profile: ChimeraProfile,
    creature: str = "",
    call_name: str = "",
    phrase_id: str = "",
    timbre: str = "",
    title: str = "",
    syllables: str = "",
    piece: Optional[PieceChoice] = None,
    library: Optional[Library] = None,
) -> ChimeraPlan:
    """Разобрать просьбу профилем хоста. Отказ — до хоста и со своей причиной."""
    asked = {
        "action": action,
        "creature": creature,
        "call": call_name,
        "phrase_id": phrase_id,
        "timbre": timbre,
        "title": title,
    }
    if not profile.allows(action):
        return _plan(profile, "refuse", action, "unsupported_action", request=asked)
    if action == "call":
        return _call(profile, creature, call_name, asked)
    if action == "speak":
        return _speak(profile, phrase_id, timbre, asked)
    return _melody(action, profile, timbre, title, syllables, piece or choose_piece(library, title), asked)


def _call(profile: ChimeraProfile, creature: str, call_name: str, asked: Mapping[str, str]) -> ChimeraPlan:
    clip = profile.call_clip(creature, call_name)
    if clip is None:
        return _plan(profile, "refuse", "call", "unsupported_pairing", request=asked)
    if not clip:
        return _plan(profile, "refuse", "call", "missing_file", request=asked)
    return _plan(profile, "play", "call", route=ROUTE_CLIP, clip=clip, request=asked)


def _speak(profile: ChimeraProfile, phrase_id: str, timbre: str, asked: Mapping[str, str]) -> ChimeraPlan:
    if timbre not in profile.timbres:
        return _plan(profile, "refuse", "speak", "unsupported_timbre", timbre=timbre, request=asked)
    if phrase_id not in profile.phrases:
        return _plan(profile, "refuse", "speak", "unsupported_pronunciation", timbre=timbre, request=asked)
    return _plan(
        profile, "play", "speak", route=ROUTE_LONG, timbre=timbre, syllables=profile.phrases[phrase_id], request=asked
    )


def _melody(
    action: str,
    profile: ChimeraProfile,
    timbre: str,
    title: str,
    syllables: str,
    piece: PieceChoice,
    asked: Mapping[str, str],
) -> ChimeraPlan:
    if action == "sing" and timbre not in profile.timbres:
        return _plan(profile, "refuse", action, "unsupported_timbre", timbre=timbre, request=asked)
    if piece.variants:
        return _plan(profile, "refuse", action, "ambiguous_title", variants=piece.variants, request=asked)
    if not piece.material_id:
        return _plan(profile, "refuse", action, "unknown_title", request=asked)
    sung = syllables if action == "sing" else ""  # play — мелодия без текста, нейтральный вокализ
    return _plan(
        profile,
        "play",
        action,
        route=ROUTE_LONG,
        material_id=piece.material_id,
        title=piece.title,
        composer=piece.composer,
        timbre=timbre if action == "sing" else "",
        syllables=sung,
        request=asked,
    )


def _outcome(plan: ChimeraPlan, *args: Any, **kwargs: Any) -> ChimeraOutcome:
    """Исход с пометкой заглушки плана: ветки ``outcome_of`` её не дублируют."""
    return ChimeraOutcome(*args, stub=plan.stub, **kwargs)


def outcome_of(plan: ChimeraPlan, stage: Optional[Mapping[str, Any]] = None, request_id: str = "") -> ChimeraOutcome:
    """Исход: отказ плана — сразу; иначе стадия хоста. ``sang`` — только ``finished``."""
    if not plan.plays:
        return _outcome(
            plan,
            "rejected",
            plan.reason,
            plan.action,
            request_id=request_id,
            title=plan.title,
            variants=plan.variants,
            timbre=plan.timbre,
        )
    stage = stage or {}
    kind = str(stage.get("stage") or "")
    base = _outcome(
        plan,
        "pending",
        kind or "no_stage",
        plan.action,
        request_id,
        plan.material_id,
        plan.title,
        plan.composer,
        plan.timbre,
        plan.route,
        plan.clip,
    )
    if kind == "finished":
        return _done(base, "finished", sang=plan.action in SUNG_ACTIONS)
    if kind == "started":
        return _done(base, "started", status="pending", sang=False)
    if kind == "cancelled":
        return _outcome(
            plan,
            "cancelled",
            _cancel_reason(stage),
            plan.action,
            request_id,
            plan.material_id,
            plan.title,
            plan.composer,
            plan.timbre,
            plan.route,
            plan.clip,
        )
    if kind in ("missing_file", "offline"):
        return _outcome(
            plan,
            "rejected",
            kind,
            plan.action,
            request_id,
            plan.material_id,
            plan.title,
            plan.composer,
            plan.timbre,
            plan.route,
            plan.clip,
        )
    if kind == "skipped":
        return _outcome(
            plan,
            "rejected",
            str(stage.get("reason") or "skipped"),
            plan.action,
            request_id,
            plan.material_id,
            plan.title,
            plan.composer,
            plan.timbre,
            plan.route,
            plan.clip,
        )
    if not kind:
        return _outcome(
            plan,
            "timeout",
            "timeout",
            plan.action,
            request_id,
            plan.material_id,
            plan.title,
            plan.composer,
            plan.timbre,
            plan.route,
            plan.clip,
        )
    return base


def _done(base: ChimeraOutcome, reason: str, status: str = "done", sang: bool = False) -> ChimeraOutcome:
    return ChimeraOutcome(
        status,
        reason,
        base.action,
        base.request_id,
        base.material_id,
        base.title,
        base.composer,
        base.timbre,
        base.route,
        base.clip,
        sang=sang,
        stub=base.stub,
    )


def _cancel_reason(stage: Mapping[str, Any]) -> str:
    """Отмена во время счёта и отмена во время звука — разные причины."""
    when = str(stage.get("when") or "")
    if when == "render":
        return "cancelled_render"
    if when == "playback":
        return "cancelled_playback"
    return "cancelled"


def sang_phrase(
    outcome: ChimeraOutcome, text_of: Callable[..., str], key_of: Optional[Callable[..., Optional[str]]] = None
) -> Optional[str]:
    """«Спел» — только когда проигрывание закончилось. Иначе фраза причины или ничего (ещё играет — не хвалимся).

    Ключ причины — тот же, что у ``respond.phrase_key``: ``chimera.{status}.{reason}``, затем ``chimera.{status}``.
    Голого ``chimera.{reason}`` в каталоге нет.
    """
    if outcome.sang:
        name = outcome.title
        return text_of(SANG, title=f" «{name}»" if name else "", action=outcome.action)
    if outcome.status == "done":
        name = outcome.title or outcome.clip
        return text_of(DONE_HEARD, title=f" «{name}»" if name else "", action=outcome.action)
    if outcome.status == "pending":
        return None
    key = _phrase_key(outcome, key_of)
    return None if key is None else text_of(key, title=outcome.title, action=outcome.action, timbre=outcome.timbre)


def _phrase_key(outcome: ChimeraOutcome, key_of: Optional[Callable[..., Optional[str]]]) -> str:
    specific = f"chimera.{outcome.status}.{outcome.reason}"
    base = f"chimera.{outcome.status}"
    if key_of is None:
        return specific
    return key_of(specific, base, "chimera.rejected") or specific


def distinct(reasons: Sequence[str]) -> bool:
    """Причины исходов не схлопнулись в одну: каждая из списка встречается и все разные."""
    return len(set(reasons)) == len(list(reasons)) and all(reasons)


__all__ = [
    "ACTIONS",
    "TIMBRES",
    "HOST_STAGES",
    "DECIDE_REASONS",
    "SANG",
    "SUNG_ACTIONS",
    "DONE_HEARD",
    "ChimeraProfile",
    "ChimeraPlan",
    "ChimeraOutcome",
    "PieceChoice",
    "stub_profile",
    "title_ref",
    "choose_piece",
    "decide",
    "outcome_of",
    "sang_phrase",
    "distinct",
]
