#!/usr/bin/env python3
"""persona — описание персонажа и кличка из него (ADR-0164 §4.1, этап 5).

Цепочка одна для лица и для голоса:

1. Код собирает материал (:func:`collect_material`) — структуру, а не свободный текст: есть ли снимок лица,
   реплики и факты памяти, сколько было встреч.
2. LLM одним ответом по узкой схеме (:data:`PROPOSAL_SCHEMA`) предлагает описание (внешность, речь) и кличку
   из него. Запрос строит :func:`build_proposal_prompt`; сам вызов — у вызывающего, модуль сеть не трогает.
3. Код проверяет ответ (:func:`check_proposal`): схема, описание опирается только на поданный материал,
   кличка — форма, уникальность, не «Незнакомец», без имён (``epithets.check_llm_epithet``).
4. Код решает (:func:`decide`): принятая кличка, прежняя или словарная (:func:`fallback_epithet`, без облака).
   Записывает результат владелец хранилища (ADR-0148), строку лога строит :func:`decision_log_line`.

Поведение (раздел ``behavior``) пишет код из чисел материала, а не модель: число встреч модель не знает лучше
счётчика.

Уникальность — по обоим хранилищам (лица и голоса): :func:`taken_union`. Потолок вызовов vision-LLM в час —
:class:`CallBudget`, величина по умолчанию — :data:`VISION_CALLS_PER_HOUR` (одно место; на роботе не измерена,
ADR-0164 §9).

Описание и кличка — метки, а не приметы (ADR-0130): имени они не дают и в опознании не участвуют.
"""

from __future__ import annotations

import json
import re
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, Iterable, List, Mapping, Optional, Tuple

from rob_box_core import epithets

# ── Разделы описания ─────────────────────────────────────────────────────────

SECTION_APPEARANCE = "appearance"
SECTION_SPEECH = "speech"
SECTION_BEHAVIOR = "behavior"

#: Причина в ``epithet_history``: кличка выведена из описания персонажа (LLM предложила, код проверил).
REASON_PERSONA = "persona"

#: Длина раздела описания от модели. Длиннее — отказ, а не обрезка: обрывок описания хуже его отсутствия.
SECTION_MAX_LEN = 240

#: Сколько последних реплик и фактов идёт в материал и сколько знаков в каждой.
MAX_UTTERANCES = 12
MAX_FACTS = 12
ITEM_MAX_LEN = 200

#: Потолок вызовов vision-LLM в скользящий час (ADR-0164 §4.1 «Цена и мусор»). Стартовое значение, не замер:
#: величину измеряют на роботе (§9). Переопределяется параметром ноды, но знание о дефолте живёт здесь.
VISION_CALLS_PER_HOUR = 20
HOUR_SEC = 3600.0

# Причины отказа описания (причины отказа клички — ``epithets.REJECT_*``).
REJECT_SCHEMA = "schema"
REJECT_UNSUPPORTED = "unsupported_section"
REJECT_SECTION_LENGTH = "section_length"
REJECT_DESCRIPTION_EMPTY = "description_empty"

_PROPOSAL_KEYS = (SECTION_APPEARANCE, SECTION_SPEECH, "epithet")
_FENCE_RE = re.compile(r"^```(?:json)?\s*|\s*```$", re.IGNORECASE)
_THINK_OPEN, _THINK_CLOSE = "<think>", "</think>"

#: Сколько знаков ответа модели попадает в строку лога об отказе схемы.
RAW_PREVIEW_LEN = 120

#: Узкая схема ответа модели. ``behavior`` в ней нет — его пишет код.
PROPOSAL_SCHEMA: Dict[str, Any] = {
    "type": "object",
    "properties": {
        SECTION_APPEARANCE: {"type": "string", "maxLength": SECTION_MAX_LEN},
        SECTION_SPEECH: {"type": "string", "maxLength": SECTION_MAX_LEN},
        "epithet": {"type": "string", "maxLength": epithets.LLM_EPITHET_MAX_LEN},
    },
    "required": list(_PROPOSAL_KEYS),
    "additionalProperties": False,
}


# ── Материал ─────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class PersonaMaterial:
    """Что код знает о человеке к моменту запроса. Структура, а не текст (ADR-0164 §4.1 п.1).

    ``has_snapshot`` — к запросу приложен снимок лица с точками лица; без него раздел «внешность» модели
    писать не из чего. ``names`` — имена знакомых людей: в кличке их быть не должно (``REJECT_NAME``).
    """

    subject_id: str
    session_id: str
    has_snapshot: bool = False
    utterances: Tuple[str, ...] = ()
    facts: Tuple[str, ...] = ()
    encounter_count: int = 0
    names: Tuple[str, ...] = ()

    @property
    def has_speech(self) -> bool:
        return bool(self.utterances or self.facts)

    def sources(self) -> Tuple[str, ...]:
        """Из чего можно строить описание — в порядке разделов."""
        out = []
        if self.has_snapshot:
            out.append(SECTION_APPEARANCE)
        if self.has_speech:
            out.append(SECTION_SPEECH)
        if self.encounter_count > 0:
            out.append(SECTION_BEHAVIOR)
        return tuple(out)

    def speech_lines(self) -> List[str]:
        return list(self.facts) + list(self.utterances)


def _clean_items(items: Iterable[Any], limit: int) -> Tuple[str, ...]:
    cleaned = [" ".join(str(x).split())[:ITEM_MAX_LEN] for x in items or () if x and str(x).strip()]
    return tuple(cleaned[-limit:])


def collect_material(
    subject_id: str,
    session_id: str,
    *,
    has_snapshot: bool = False,
    utterances: Iterable[Any] = (),
    facts: Iterable[Any] = (),
    encounter_count: int = 0,
    names: Iterable[Any] = (),
) -> PersonaMaterial:
    """Собрать материал: последние реплики и факты (обрезаны по числу и длине), снимок, встречи, имена."""
    return PersonaMaterial(
        subject_id=str(subject_id or ""),
        session_id=str(session_id or ""),
        has_snapshot=bool(has_snapshot),
        utterances=_clean_items(utterances, MAX_UTTERANCES),
        facts=_clean_items(facts, MAX_FACTS),
        encounter_count=max(0, int(encounter_count or 0)),
        names=tuple(str(n).strip() for n in names or () if n and str(n).strip()),
    )


def describe_behavior(material: PersonaMaterial) -> str:
    """Раздел «поведение» — пишет код из чисел материала."""
    if material.encounter_count <= 0:
        return ""
    if material.encounter_count == 1:
        return "Встречен впервые."
    return f"Встреч: {material.encounter_count}."


# ── Описание ─────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class PersonaDescription:
    """Описание персонажа одной сессии. Хранится как ``persona`` в записи (``to_meta``/``from_meta``)."""

    session_id: str
    appearance: str = ""
    speech: str = ""
    behavior: str = ""
    sources: Tuple[str, ...] = ()
    ts: float = 0.0

    def is_empty(self) -> bool:
        return not (self.appearance or self.speech)

    def to_meta(self) -> Dict[str, Any]:
        return {
            "session_id": self.session_id,
            SECTION_APPEARANCE: self.appearance,
            SECTION_SPEECH: self.speech,
            SECTION_BEHAVIOR: self.behavior,
            "sources": list(self.sources),
            "ts": float(self.ts),
        }

    @classmethod
    def from_meta(cls, data: Any) -> Optional["PersonaDescription"]:
        """Прочитать из ``meta.json``/БД; старые записи без поля и битые значения → ``None``."""
        if not isinstance(data, Mapping):
            return None
        try:
            return cls(
                session_id=str(data.get("session_id") or ""),
                appearance=str(data.get(SECTION_APPEARANCE) or ""),
                speech=str(data.get(SECTION_SPEECH) or ""),
                behavior=str(data.get(SECTION_BEHAVIOR) or ""),
                sources=tuple(str(s) for s in data.get("sources") or ()),
                ts=float(data.get("ts") or 0.0),
            )
        except (TypeError, ValueError):
            return None


# ── Запрос к модели ──────────────────────────────────────────────────────────


def build_proposal_prompt(material: PersonaMaterial, *, current_epithet: Optional[str] = None) -> str:
    """Запрос к (vision-)LLM: описание по поданному материалу и кличка из него, ответ — JSON по схеме."""
    speech = "\n".join(f"- {line}" for line in material.speech_lines()) or "(нет)"
    photo = (
        "К запросу приложен снимок лица. Опиши, как человек выглядит: лицо, причёска, очки, борода, одежда, "
        "что в руках."
        if material.has_snapshot
        else f"Снимка нет — поле «{SECTION_APPEARANCE}» оставь пустой строкой."
    )
    current = f"Нынешняя кличка: {current_epithet}.\n" if current_epithet else ""
    return (
        "Ты составляешь для робота ВНУТРЕННЕЕ описание человека и кличку из него. Кличка нужна, чтобы различать "
        "людей; человеку в лицо её не говорят.\n\n"
        f"{photo}\n"
        f"Что человек говорил и что о нём известно:\n{speech}\n"
        f"{current}\n"
        "Пиши только то, что видно на снимке или есть в списке выше; если речи нет — поле "
        f"«{SECTION_SPEECH}» оставь пустой строкой. Каждое поле — до {SECTION_MAX_LEN} знаков.\n"
        f"Кличка — {epithets.LLM_EPITHET_MIN_WORDS}–{epithets.LLM_EPITHET_MAX_WORDS} слова на русском из этого "
        "описания, первое с большой буквы, например «Бородач в синем свитере». Одежда допустима. Без имён, "
        "без насмешек над внешностью, здоровьем, национальностью, без грубых слов, не «незнакомец» и не «гость». "
        "Если описать нечего — в поле кличка поставь «-».\n"
        "Ответь ОДНИМ JSON-объектом с ключами "
        f"{', '.join(_PROPOSAL_KEYS)} и ничем больше."
    )


def preview_raw(raw: Any) -> str:
    """Начало ответа модели для лога отказа: одна строка, не длиннее :data:`RAW_PREVIEW_LEN` знаков."""
    text = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else str(raw)
    text = " ".join(text.split())
    return text if len(text) <= RAW_PREVIEW_LEN else text[:RAW_PREVIEW_LEN] + "…"


def _strip_wrappers(text: str) -> Tuple[Optional[str], str]:
    """Снять обёртки ответа: ведущий блок ``<think>…</think>`` (рассуждение M3 в content) и ```json-ограду.

    ``(текст, "")`` или ``(None, почему)``: рассуждение без закрывающего тега — ответ оборван (потолок токенов).
    """
    text = text.strip()
    if text[: len(_THINK_OPEN)].lower() == _THINK_OPEN:
        close_at = text.lower().find(_THINK_CLOSE)
        if close_at < 0:
            return None, f"ответ оборван внутри {_THINK_OPEN} (нет {_THINK_CLOSE})"
        text = text[close_at + len(_THINK_CLOSE) :].strip()  # noqa: E203 — срез по black
    return _FENCE_RE.sub("", text).strip(), ""


def _schema_mismatch(data: Any) -> str:
    """Чем разобранный JSON не совпал со схемой; пустая строка — совпал."""
    if not isinstance(data, Mapping):
        return f"не объект, а {type(data).__name__}"
    extra = sorted(str(k) for k in set(data) - set(_PROPOSAL_KEYS))
    missing = [k for k in _PROPOSAL_KEYS if k not in data]
    if extra or missing:
        return f"лишние ключи {extra}, нет ключей {missing}"
    wrong = [k for k in _PROPOSAL_KEYS if not isinstance(data[k], str)]
    return f"не строка в {wrong}" if wrong else ""


def _parse(raw: Any) -> Tuple[Optional[Dict[str, str]], str]:
    """``(поля, "")`` или ``(None, подробность отказа)``."""
    data = raw
    if isinstance(raw, (str, bytes)):
        text, problem = _strip_wrappers(raw.decode("utf-8", "replace") if isinstance(raw, bytes) else raw)
        if text is None:
            return None, problem
        try:
            data = json.loads(text)
        except ValueError as exc:
            return None, f"не JSON ({exc.msg})"
    problem = _schema_mismatch(data)
    if problem:
        return None, problem
    return {k: " ".join(data[k].split()) for k in _PROPOSAL_KEYS}, ""


def parse_proposal(raw: Any) -> Tuple[Optional[Dict[str, str]], Optional[str]]:
    """Разобрать ответ модели по :data:`PROPOSAL_SCHEMA`: ``(поля, None)`` или ``(None, REJECT_SCHEMA)``.

    Принимает dict (structured output) или строку JSON, в том числе в ```json-ограде и после блока
    ``<think>…</think>``. Лишний ключ, не-строка или отсутствующий ключ — отказ: схема узкая, догадок нет.
    Подробность отказа — в :attr:`ProposalVerdict.detail` (:func:`check_proposal`).
    """
    fields, _ = _parse(raw)
    return (fields, None) if fields is not None else (None, REJECT_SCHEMA)


@dataclass(frozen=True)
class ProposalVerdict:
    """Итог проверки ответа модели. ``description`` может быть принято при отклонённой кличке."""

    description: Optional[PersonaDescription] = None
    epithet: Optional[str] = None
    reject_reason: Optional[str] = None
    #: Подробность отказа схемы для лога: что не так и начало ответа (:func:`preview_raw`).
    detail: str = ""


def _description_problem(fields: Mapping[str, str], material: PersonaMaterial) -> Optional[str]:
    appearance, speech = fields[SECTION_APPEARANCE], fields[SECTION_SPEECH]
    if (appearance and not material.has_snapshot) or (speech and not material.has_speech):
        return REJECT_UNSUPPORTED  # модель описала то, чего ей не показывали
    if len(appearance) > SECTION_MAX_LEN or len(speech) > SECTION_MAX_LEN:
        return REJECT_SECTION_LENGTH
    if not (appearance or speech):
        return REJECT_DESCRIPTION_EMPTY
    return None


def check_proposal(
    raw: Any,
    material: PersonaMaterial,
    *,
    taken: Iterable[str] = (),
    names: Iterable[str] = (),
    now: float = 0.0,
) -> ProposalVerdict:
    """Проверить предложение модели. Решение о записи — :func:`decide`."""
    fields, detail = _parse(raw)
    if fields is None:
        return ProposalVerdict(reject_reason=REJECT_SCHEMA, detail=f"{detail}; ответ: «{preview_raw(raw)}»")
    problem = _description_problem(fields, material)
    if problem:
        return ProposalVerdict(reject_reason=problem)
    description = PersonaDescription(
        session_id=material.session_id,
        appearance=fields[SECTION_APPEARANCE],
        speech=fields[SECTION_SPEECH],
        behavior=describe_behavior(material),
        sources=material.sources(),
        ts=float(now),
    )
    label, reason = epithets.check_llm_epithet(
        fields["epithet"], taken=taken, names=list(material.names) + list(names or ())
    )
    return ProposalVerdict(description=description, epithet=label, reject_reason=reason)


# ── Решение ──────────────────────────────────────────────────────────────────


def taken_union(*stores: Iterable[Optional[str]], exclude: Optional[str] = None) -> List[str]:
    """Занятые клички из всех хранилищ (лица и голоса) без повторов; ``exclude`` — своя нынешняя кличка.

    Своя кличка исключается, иначе повторное предложение той же клички было бы отклонено как «занятая».
    """
    skip = epithets.normalize_epithet(exclude) if exclude else None
    seen: Dict[str, str] = {}
    for store in stores:
        for label in store or ():
            key = epithets.normalize_epithet(label)
            if key and key != skip and key not in seen:
                seen[key] = str(label).strip()
    return list(seen.values())


def fallback_epithet(material: PersonaMaterial, *, taken: Iterable[str] = ()) -> str:
    """Словарная кличка без облака (одно слово, как у голоса): по темам речи, иначе из общего пула."""
    lines = material.speech_lines()
    candidate = epithets.choose_epithet(
        epithets.extract_tags(lines),
        speaker_id=material.subject_id,
        taken=taken,
        sentiment=epithets.score_sentiment(lines),
    )
    return candidate.label


@dataclass(frozen=True)
class EpithetDecision:
    """Что записать: кличку, причину для ``epithet_history`` (``None`` — без перемены), описание, пояснение."""

    epithet: str
    reason: Optional[str]
    description: Optional[PersonaDescription] = None
    note: str = ""
    previous: Optional[str] = None

    @property
    def changed(self) -> bool:
        return self.reason is not None


def _usable(label: Optional[str]) -> bool:
    return bool(label and str(label).strip()) and not epithets.is_stranger_epithet(label)


def _reject_note(verdict: Optional[ProposalVerdict]) -> str:
    if verdict is None:
        return "кандидат отклонён: нет ответа"
    detail = f" — {verdict.detail}" if verdict.detail else ""
    return f"кандидат отклонён: {verdict.reject_reason}{detail}"


def decide(
    material: PersonaMaterial,
    verdict: Optional[ProposalVerdict],
    *,
    current: Optional[str] = None,
    taken: Iterable[str] = (),
    unavailable: str = "",
) -> EpithetDecision:
    """Принятая кличка из описания → она; иначе прежняя; иначе словарная (ADR-0164 §4.1 п.4).

    ``verdict=None`` — модель не спрашивали (нет облака, режим, потолок): причину кладёт ``unavailable``, она
    уходит в лог честно, без тихой деградации. Плейсхолдер «Незнакомец» за прежнюю кличку не считается.
    """
    description = verdict.description if verdict else None
    previous = current if _usable(current) else None
    if verdict and verdict.epithet:
        same = previous and epithets.normalize_epithet(previous) == epithets.normalize_epithet(verdict.epithet)
        return EpithetDecision(
            epithet=verdict.epithet, reason=None if same else REASON_PERSONA,
            description=description, note="по описанию", previous=previous,
        )
    note = unavailable or _reject_note(verdict)
    if previous:
        return EpithetDecision(epithet=previous, reason=None, description=description, note=note, previous=previous)
    label = fallback_epithet(material, taken=taken)
    return EpithetDecision(
        epithet=label, reason=epithets.REASON_FIRST_SEEN, description=description,
        note=f"словарная ({note})", previous=current or None,
    )


def history_entry(old: Optional[str], new: str, reason: str, ts: float) -> Dict[str, Any]:
    """Запись ``epithet_history`` — тот же формат, что у голоса (``speaker_embeddings.set_epithet``)."""
    return {"ts": float(ts), "old": old, "new": new, "reason": reason}


def decision_log_line(subject: str, decision: EpithetDecision) -> str:
    """Строка лога о решении: ``кличка «старая» → «новая» (причина: пояснение)``; строит код, не модель."""
    if not decision.changed:
        return f"кличка {subject}: «{decision.epithet}» без перемены ({decision.note})"
    return (
        f"кличка {subject}: «{decision.previous or '—'}» → «{decision.epithet}» "
        f"({decision.reason}: {decision.note})"
    )


# ── Потолок вызовов ──────────────────────────────────────────────────────────


@dataclass
class CallBudget:
    """Потолок вызовов vision-LLM в скользящем окне (по умолчанию час). Потокобезопасен."""

    per_window: int = VISION_CALLS_PER_HOUR
    window_sec: float = HOUR_SEC
    _calls: Deque[float] = field(default_factory=deque, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def _expire(self, now: float) -> None:
        while self._calls and now - self._calls[0] >= self.window_sec:
            self._calls.popleft()

    def try_acquire(self, now: float) -> bool:
        """Занять вызов, если потолок не выбран; ``False`` — вызывать нельзя."""
        with self._lock:
            self._expire(now)
            if len(self._calls) >= max(0, int(self.per_window)):
                return False
            self._calls.append(float(now))
            return True

    def remaining(self, now: float) -> int:
        with self._lock:
            self._expire(now)
            return max(0, int(self.per_window) - len(self._calls))


__all__ = [
    "CallBudget",
    "EpithetDecision",
    "HOUR_SEC",
    "PROPOSAL_SCHEMA",
    "PersonaDescription",
    "PersonaMaterial",
    "ProposalVerdict",
    "REASON_PERSONA",
    "REJECT_DESCRIPTION_EMPTY",
    "REJECT_SCHEMA",
    "REJECT_SECTION_LENGTH",
    "REJECT_UNSUPPORTED",
    "SECTION_APPEARANCE",
    "SECTION_BEHAVIOR",
    "SECTION_MAX_LEN",
    "SECTION_SPEECH",
    "VISION_CALLS_PER_HOUR",
    "build_proposal_prompt",
    "check_proposal",
    "collect_material",
    "decide",
    "decision_log_line",
    "describe_behavior",
    "fallback_epithet",
    "history_entry",
    "parse_proposal",
    "preview_raw",
    "RAW_PREVIEW_LEN",
    "taken_union",
]
