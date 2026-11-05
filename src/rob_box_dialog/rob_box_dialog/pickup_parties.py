"""Автор фразы и инициатор подхвата: кто что сказал и чьи права у хода (решение Шифу 05.11, М12 n213).

Решение в формате ADR (дополняет ADR-0150 §7.3 В12, §7.5; ADR-0148 «код решает, LLM говорит").

**Контекст.** Подхват бэклога (``address._woken``, ``Backlog.pickup``) исполняет фразы, сказанные без вейка, по
голому «Робот». У такого хода два человека: **автор фразы** — кто сказал саму просьбу (метка ``Backlog.label``:
голос длинной фразы, биометрия по её ``utterance_id``) и **инициатор подхвата** — кто сказал «Робот» (собеседник
сессии на начало хода, ``SessionSnapshot.speaker``). До 05.11 права хода (память о человеке: ``memory_search``,
``memory_save``, раздел ``speaker_memory``) брались только по инициатору. М12 n213 (run 41969501900): Саша сказал
«ребят, поищи у себя в памяти, что я говорил про чай» (STT потерял «Робот»; голос 4.77 с — «Саша» 0.956), потом
«Робот!» (речь 0.96 с — спор «Саша» 0.764 / «Борис» 0.671, ``contested``) — ``memory_search`` отклонён
``voice_unconfirmed``: «Голос знакомый, но не уверен чей…». Модель видела автора тегом ``[говорит: Саша]``, а
инициатора — только разделом ``speaker`` без слова «инициатор».

**Решение.**

1. Права хода — по автору исполняемой фразы, если его голос уверенный (:func:`turn_speaker`): все подхваченные фразы
   с узнанным голосом — одного профиля (имя ``confirmed`` или свой профиль без имени), и ни одна не звучит чужим или
   спорным голосом (спор, гипотеза, незнакомый голос). Короткий вейк инициатора тогда прав не решает.
2. Уверенного автора нет — права инициатора, только если каждая фраза помечена кодом как сказанная им же
   (``same_as_caller``: голос, лицо в кадре, направление — ``speaker_adapter.same_interlocutor``). Нет такого признака
   или авторов несколько — права ничьи (``SpeakerRef`` без ``person_ref``): память одного человека другому
   код не подставляет (память Саши не уйдёт Борису без признака).
3. Модель видит обоих: раздел снимка ``backlog_pickup`` (:func:`pickup_section`) — у каждой фразы автор (имя,
   кличка, «голос без имени» или «незнакомый голос»), уверенность (``certainty``, ``score``) и текст; отдельно —
   инициатор с уверенностью; ``same_person`` — один ли это человек по данным кода; ``memory_of`` — чья память у хода
   (решение кода из п. 1–2). Кому отвечать (автору или инициатору, который просит за него) решает модель.

**Последствия.** ``speaker`` снимка подхвата — собеседник по правам (автор), а не тот, кто сказал «Робот»; спорный
вейк больше не закрывает память уверенного автора. Инициатор — сессия на начало хода: если биометрия вейка опоздала
за STT, это собеседник предыдущей фразы (на правила п. 1 не влияет — права берутся у автора). Формат раздела и
правило — только здесь; ход (``turn.pickup_parts``) и хост (``dialog_node._note_speaker``) импортируют.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Dict, List, Mapping, Optional, Sequence

from rob_box_core.speaker_wire import (
    NAME_CONFIRMED,
    NAME_CONTESTED,
    NAME_NAMELESS,
    NAME_TENTATIVE,
    NAME_UNCONFIRMED,
    NAME_UNKNOWN,
)
from rob_box_dialog import knowledge as kn
from rob_box_dialog.session import SessionSnapshot, SpeakerRef

#: Раздел снимка хода подхвата (``knowledge.CONTEXT_SECTIONS``).
PICKUP_SECTION = "backlog_pickup"
#: Биометрия о фразе молчит (короткая, ``inconclusive``; результат не пришёл, ``none``): не довод ни за, ни против.
VOICE_SILENT = "inconclusive"
#: Уверенность, при которой автор фразы получает права хода: имя подтверждено или свой профиль без имени.
_TRUSTED = (NAME_CONFIRMED, NAME_NAMELESS)


def phrase_certainty(speaker: Mapping[str, Any]) -> str:
    """Уверенность в авторе фразы бэклога по метке ``speaker_adapter.phrase_speaker`` словами ``speaker_wire``:
    ``confirmed`` (узнан), ``nameless`` (свой профиль без имени), ``tentative`` (гипотеза), ``contested`` (спор),
    ``unknown`` (голос не узнан), :data:`VOICE_SILENT` (биометрия не оценила или не пришла)."""
    if speaker.get("identified"):
        return NAME_CONFIRMED
    bio = speaker.get("bio")
    if bio == "known":
        return NAME_TENTATIVE if speaker.get("candidate") else NAME_NAMELESS
    return {"contested": NAME_CONTESTED, "unknown_voice": NAME_UNKNOWN}.get(str(bio), VOICE_SILENT)


def initiator_certainty(ref: Optional[SpeakerRef]) -> str:
    """Уверенность в инициаторе (``SpeakerRef`` сессии) теми же словами, что у автора: профиль без имени —
    ``nameless``."""
    if ref is None:
        return NAME_UNKNOWN
    if ref.name_state == NAME_UNKNOWN and ref.person_ref:
        return NAME_NAMELESS
    return ref.name_state


def phrase_author(label: Mapping[str, Any]) -> Optional[SpeakerRef]:
    """Автор фразы (метка ``Backlog.label``) как собеседник с правами — только при уверенном голосе (:data:`_TRUSTED`)
    и известном профиле; иначе ``None``."""
    speaker = label.get("speaker") or {}
    person = str(label.get("speaker_id") or "")
    certainty = phrase_certainty(speaker)
    if certainty not in _TRUSTED or not person or person == NAME_UNKNOWN:
        return None
    named = certainty == NAME_CONFIRMED
    return SpeakerRef(person_ref=person, name=str(speaker.get("name") or "") or None if named else None,
                      name_state=NAME_CONFIRMED if named else NAME_UNKNOWN,
                      channels=tuple(speaker.get("channels") or ("voice",)))


def by_caller(picked: Sequence[Mapping[str, Any]]) -> bool:
    """Каждая фраза сказана инициатором по признаку кода: ``same_as_caller`` метки
    (``speaker_adapter.same_interlocutor``) у фразы, голос которой никому не приписан. У приписанной фразы
    ``same_as_caller`` бывает от одного канала «голос длинной фразы узнан» — это знание об авторе, а не о том, что
    «Робот» сказал он же (n213: фраза Саши 0.956 ``same_as_caller`` при спорном вейке); такую фразу решает её автор
    (:func:`phrase_author`)."""
    unattributed = [p for p in picked if str(p.get("speaker_id") or NAME_UNKNOWN) == NAME_UNKNOWN]
    return bool(picked) and len(unattributed) == len(picked) and all(p.get("same_as_caller") for p in picked)


def turn_speaker(picked: Sequence[Mapping[str, Any]], initiator: Optional[SpeakerRef]) -> SpeakerRef:
    """Собеседник подхвата по правам (правило модуля, п. 1–2): уверенный единственный автор; иначе инициатор, если
    каждая фраза — его по признаку кода (:func:`by_caller`); иначе никто — ``SpeakerRef`` без профиля
    (``contested``, если в ходе звучал знакомый голос: память отклоняется честным ``voice_unconfirmed``, а не
    «не узнаю голос»)."""
    authors = {ref.person_ref: ref for ref in map(phrase_author, picked) if ref is not None}
    doubtful = [phrase_certainty(p.get("speaker") or {}) for p in picked if phrase_author(p) is None]
    doubtful = [c for c in doubtful if c != VOICE_SILENT]  # чужой или спорный голос среди фраз
    if len(authors) == 1 and not doubtful:
        return next(iter(authors.values()))
    if not authors and initiator is not None and by_caller(picked):
        return initiator
    return _nobody(bool(authors) or any(c in NAME_UNCONFIRMED for c in doubtful), initiator)


def _nobody(familiar: bool, initiator: Optional[SpeakerRef]) -> SpeakerRef:
    """Права ничьи: собеседник без профиля; ``contested``, если в ходе звучал знакомый голос (фразы или вейк)."""
    familiar = familiar or initiator_certainty(initiator) != NAME_UNKNOWN
    return SpeakerRef(name_state=NAME_CONTESTED if familiar else NAME_UNKNOWN)


def pickup_speaker(addressed: Any, initiator: Optional[SpeakerRef]) -> Optional[SpeakerRef]:
    """Собеседник хода подхвата по правам (:func:`turn_speaker`); не подхват — ``None``: права у собеседника сессии,
    как раньше (смену собеседника посреди хода ловит ``memory.speaker_changed``)."""
    picked = getattr(addressed, "picked", ())
    return turn_speaker(picked, initiator) if getattr(addressed, "pickup", False) and picked else None


def rights_speaker(addressed: Any, session_speaker: Optional[SpeakerRef]) -> Optional[SpeakerRef]:
    """Чей профиль и память у хода: подхват — :func:`pickup_speaker`, иначе собеседник сессии (хост,
    ``dialog_node._note_speaker``: раздел ``speaker_memory`` и счёт диалогов — автору подхваченной фразы)."""
    speaker = pickup_speaker(addressed, session_speaker)
    return session_speaker if speaker is None else speaker


def own_snapshot(snap: SessionSnapshot, speaker: Optional[SpeakerRef]) -> SessionSnapshot:
    """Снимок сессии с собеседником хода подхвата (``speaker`` — :func:`pickup_speaker`; ``None`` — как есть)."""
    return snap if speaker is None else replace(snap, speaker=speaker)


def who(ref: Optional[SpeakerRef]) -> str:
    """Кто это словами кода для раздела :func:`pickup_section`: имя — только подтверждённое; профиль без имени —
    ``NAMELESS_SPEAKER_LABEL``; спор или гипотеза — ``CONTESTED_SPEAKER_LABEL``; иначе ``UNKNOWN_SPEAKER_LABEL``."""
    certainty = initiator_certainty(ref)
    if certainty == NAME_CONFIRMED and ref is not None and ref.name:
        return ref.name.strip()
    return {NAME_NAMELESS: kn.NAMELESS_SPEAKER_LABEL, NAME_CONTESTED: kn.CONTESTED_SPEAKER_LABEL,
            NAME_TENTATIVE: kn.CONTESTED_SPEAKER_LABEL}.get(certainty, kn.UNKNOWN_SPEAKER_LABEL)


def _phrase(label: Mapping[str, Any], author_label: str) -> Dict[str, Any]:
    """Фраза подхвата в разделе: автор, его профиль и уверенность, кличка (если есть), текст, возраст."""
    speaker = label.get("speaker") or {}
    out: Dict[str, Any] = {"author": author_label, "speaker_id": label.get("speaker_id") or NAME_UNKNOWN,
                           "certainty": phrase_certainty(speaker)}
    score = speaker.get("score") if speaker.get("identified") else speaker.get("candidate_score")
    if score is not None:
        out["score"] = score
    if speaker.get("epithet"):
        out["epithet"] = speaker["epithet"]
    return {**out, "text": str(label.get("text") or ""), "age_s": label.get("age_s")}


def _same_person(picked: Sequence[Mapping[str, Any]], initiator: Optional[SpeakerRef]) -> Optional[bool]:
    """Один ли человек автор и инициатор: ``True``/``False`` — когда оба уверенно узнаны (или фразы его по признаку
    кода, :func:`by_caller`); ``None`` — данных нет (спорный вейк, незнакомый голос)."""
    if by_caller(picked):
        return True
    authors = {ref.person_ref for ref in map(phrase_author, picked) if ref is not None}
    sure = initiator is not None and initiator.person_ref and initiator_certainty(initiator) in _TRUSTED
    if len(authors) != 1 or not sure:
        return None
    return initiator.person_ref in authors


def ask_intro(picked: Sequence[Mapping[str, Any]], initiator: Optional[SpeakerRef], speaker: Optional[SpeakerRef],
              warm: Sequence[str]) -> bool:
    """Просить автора назваться вместо отказа памяти (решение Шифу 05.11, после #4427): права хода ничьи, ни одна
    подхваченная фраза никому не приписана (автор не узнан), а в беседе есть прогретый голос без имени, кроме
    инициатора (``voice_warmth.WarmVoices.warm``). Короткий разговор (прогретых нет) — прежний отказ."""
    unknown_author = bool(picked) and all(str(p.get("speaker_id") or NAME_UNKNOWN) == NAME_UNKNOWN for p in picked)
    nobody = speaker is not None and not speaker.person_ref
    caller = initiator.person_ref if initiator is not None else None
    return nobody and unknown_author and any(person != caller for person in warm)


def intro_reasons(results: Sequence[Any], view: Mapping[str, Any]) -> List[Any]:
    """Исходы тулов памяти, отклонённые кодом без собеседника (``knowledge.UNBOUND_REASONS``), в ходе с просьбой
    назваться (``view["ask_intro"]``, :func:`pickup_section`) — с причиной ``ASK_INTRO_REASON``: фраза кода
    «Представься — и я поищу / запомню» вместо «Голос знакомый, но не уверен чей». Иначе исходы как есть."""
    if not view.get("ask_intro"):
        return list(results)
    return [replace(r, reason=kn.ASK_INTRO_REASON)
            if r.tool in kn.PERSON_SCOPED_TOOLS and r.status == "rejected" and r.reason in kn.UNBOUND_REASONS else r
            for r in results]


def pickup_section(picked: Sequence[Mapping[str, Any]], authors: Sequence[str], initiator: Optional[SpeakerRef],
                   memory_of: Optional[SpeakerRef], intro: bool = False) -> Dict[str, Any]:
    """Раздел снимка :data:`PICKUP_SECTION` — кто и что сказал в подхвате: ``phrases``
    (автор словами ``authors`` — метка ``turn_log.phrase_label`` каждой фразы, уверенность, текст), ``initiator``
    (кто сказал «Робот» и уверенность), ``same_person``, ``memory_of`` — чья память у хода (``None`` — ничья) и
    ``ask_intro`` — код просит автора назваться (:func:`ask_intro`), только когда так. Не подхват — ``{}``."""
    if not picked:
        return {}
    out = {"phrases": [_phrase(p, a) for p, a in zip(picked, authors)],
           "initiator": {"who": who(initiator), "certainty": initiator_certainty(initiator)},
           "same_person": _same_person(picked, initiator),
           "memory_of": None if memory_of is None else who(memory_of)}
    return {**out, "ask_intro": True} if intro else out
