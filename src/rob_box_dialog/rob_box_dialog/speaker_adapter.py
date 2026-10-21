"""Собеседник через один адаптер — шов с ADR-0130 (ADR-0150 §7.5, ADR-0130 §2.5–2.6, ADR-0139; карточка #3695).

Диалог сам не гадает, кто говорит: всё знание «кто это и тот ли это человек» — здесь.

* :func:`speaker_ref` — :class:`~rob_box_dialog.session.SpeakerRef` из результата голосовой биометрии (словарь
  ``rob_box_core.speaker_wire``) и лиц из кадра: ``confirmed`` — только уверенное имя биометрии (отрыв от соседа
  проверяет владелец биометрии) или голос-гипотеза, которую подтвердило лицо с тем же именем (два канала);
  ``tentative`` — гипотеза голоса без второго канала; иначе ``unknown``.
* :func:`same_interlocutor` — тот ли это собеседник (подхват бэклога голым «робот», В12/В12′): **ИЛИ** по каналам
  ``knowledge.PICKUP_CHANNELS`` — голос длинной фразы узнан; человек, чей голос узнан в фразе бэклога, виден в кадре
  (по имени — у лица и голоса разные id); (слабо) DOA ±60° при ком-то в кадре. Короткому «робот» по голосу не верим,
  а просто «кто-то в кадре» — не признак того же человека (перед роботом сидят и под телевизор; Шифу 21.10).
* :class:`AskPolicy` — когда можно спросить имя (В17): не на команду, не на пустой ход, не раньше N-й фразы/T с,
  один раз на собеседника.

Когда ADR-0130 этапы 2/3/5 дадут треки и реестр Знакомых, меняется только этот модуль. Чистый: время передают
вызывающие, сообщения читаются утиной типизацией.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, FrozenSet, Iterable, Mapping, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.doa import angle_diff_deg
from rob_box_dialog.session import SpeakerRef

#: Ключ :class:`AskPolicy` для незнакомца без ``person_ref``.
STRANGER = "stranger"
#: Вердикты :meth:`AskPolicy.verdict`.
ASK, COMMAND, NO_SPEECH, ASKED, ACCUMULATING = "ask", "command", "no_speech", "asked_already", "accumulating"


@dataclass(frozen=True)
class FaceSeen:
    person_id: str
    name: str
    t: float


class FaceSightings:
    """Лица из кадра за последние :data:`~rob_box_dialog.knowledge.FACE_HISTORY_S` секунд."""

    def __init__(self, history_s: float = kn.FACE_HISTORY_S) -> None:
        self.history_s = history_s
        self._seen: Deque[FaceSeen] = deque()

    def note(self, person_id: str, name: str, t: float) -> None:
        if person_id:
            self._seen.append(FaceSeen(person_id, name or "", t))
        while self._seen and t - self._seen[0].t > self.history_s:
            self._seen.popleft()

    def note_event(self, msg: Any, t: float) -> None:
        """``VisionEvent`` с лицом (``event_type=face``, ``embedding_id`` из FaceStore) → наблюдение."""
        if getattr(msg, "event_type", "") == "face":
            self.note(str(getattr(msg, "embedding_id", "") or ""), str(getattr(msg, "display_name", "") or ""), t)

    def during(self, start: float, end: float) -> FrozenSet[str]:
        """Кто был в кадре в ``[start, end]``: имя в нижнем регистре, у безымянного — ``#<id FaceStore>``."""
        return frozenset(f.name.lower() or f"#{f.person_id}" for f in self._seen if start <= f.t <= end)

    def names_during(self, start: float, end: float) -> FrozenSet[str]:
        return frozenset(f.name.lower() for f in self._seen if f.name and start <= f.t <= end)


def speaker_ref(result: Mapping[str, Any], *, now: float, face_names: Iterable[str] = (),
                azimuth_deg: Optional[float] = None, prev: Optional[SpeakerRef] = None) -> SpeakerRef:
    """Собеседник этой фразы по правилам ADR-0130 §2.6; пороги биометрии не здесь и не меняются."""
    faces = {n.lower() for n in face_names if n}
    person = (str(result.get("speaker_id") or "") or None) if result.get("is_known") else None
    state, name = _name_state(result) if person else ("unknown", None)
    channels = ("voice", "face") if name and name.lower() in faces else ("voice",)
    if state == "tentative" and len(channels) == 2:
        state = "confirmed"  # два канала согласны (ADR-0130 §2.6, ADR-0139)
    since = prev.since if prev is not None and person and prev.person_ref == person else now
    return SpeakerRef(person_ref=person, name=name, name_state=state, azimuth_deg=azimuth_deg,
                      since=since, channels=channels)


def _name_state(result: Mapping[str, Any]) -> Tuple[str, Optional[str]]:
    """Имя узнанного голоса: уверенное (``name``, отрыв проверил владелец биометрии) или гипотеза."""
    if result.get("name"):
        return "confirmed", str(result["name"])
    if result.get("tentative_name"):
        return "tentative", str(result["tentative_name"])
    return "unknown", None


def trusted_voice(heard: Any) -> str:
    """``person_ref`` фразы, если она достаточно длинная, чтобы верить голосу; иначе ``""``."""
    long_enough = float(getattr(heard, "voice_s", 0.0) or 0.0) >= kn.VOICE_MIN_PHRASE_S
    return str(getattr(heard, "person_ref", "") or "") if long_enough else ""


def same_interlocutor(entry: Any, heard: Any) -> Tuple[str, ...]:
    """Каналы, по которым ``heard`` — тот же собеседник, что сказал ``entry``; ``()`` — не тот.

    Узнанный длинный голос другого человека в ``heard`` — противоречие, остальные каналы его не перебивают.
    """
    voice_a, voice_b = trusted_voice(entry), trusted_voice(heard)
    if voice_a and voice_b and voice_a != voice_b:
        return ()
    faces_now = frozenset(getattr(heard, "faces", ()) or ())
    entry_name = str(getattr(entry, "speaker_name", "") or "").lower()
    doa_a, doa_b = getattr(entry, "doa_deg", None), getattr(heard, "doa_deg", None)
    votes = {
        "voice": bool(voice_a),
        "face": bool(entry_name) and entry_name in faces_now,
        "doa": (doa_a is not None and doa_b is not None and angle_diff_deg(doa_a, doa_b) <= kn.PICKUP_DOA_SECTOR_DEG
                and (bool(faces_now) or not kn.PICKUP_DOA_NEEDS_FACE)),
    }
    return tuple(ch for ch in kn.PICKUP_CHANNELS if votes.get(ch))


class AskPolicy:
    """Когда робот может спросить имя (В17): копит фразы собеседника и помнит, кого уже спрашивал."""

    def __init__(self, min_phrases: int = kn.IDENTITY_ASK_MIN_PHRASES, min_s: float = kn.IDENTITY_ASK_MIN_S,
                 stranger_gap_s: float = kn.IDENTITY_ASK_STRANGER_GAP_S) -> None:
        self.min_phrases, self.min_s, self.stranger_gap_s = min_phrases, min_s, stranger_gap_s
        self._heard: Dict[str, Tuple[float, int]] = {}
        self._asked: Dict[str, float] = {}

    def note(self, key: str, t: float) -> int:
        """Ещё одна фраза собеседника ``key``; возвращает, сколько их накоплено."""
        first, count = self._heard.get(key, (t, 0))
        self._heard[key] = (first, count + 1)
        return count + 1

    def verdict(self, key: str, t: float, *, has_actions: bool, spoken: bool) -> str:
        """``ASK`` — спросить сейчас; иначе причина, почему нет (для лога)."""
        if has_actions:
            return COMMAND
        if not spoken:
            return NO_SPEECH
        asked_at = self._asked.get(key)
        if asked_at is not None and (key != STRANGER or t - asked_at < self.stranger_gap_s):
            return ASKED
        first, count = self._heard.get(key, (t, 0))
        enough = count >= self.min_phrases or (
            t - first >= self.min_s and count >= kn.IDENTITY_ASK_MIN_PHRASES_BY_TIME)
        return ASK if enough else ACCUMULATING

    def mark_asked(self, key: str, t: float) -> None:
        self._asked[key] = t
        self._heard.pop(key, None)


@dataclass(frozen=True)
class AskCandidate:
    """Кого можно спросить после хода: ``single`` с именем-гипотезой — «<Имя>, это ты?», иначе — «Как тебя зовут?»."""

    key: str
    kind: str
    name: Optional[str] = None
    utterance_id: Optional[str] = None

    def question(self) -> str:
        if self.kind == "single" and self.name:
            return phrases.text("identity.confirm", name=self.name)
        return phrases.text("identity.ask_name")


def ask_after_turn(policy: AskPolicy, cand: AskCandidate, *, t: float, tools: Sequence[str] = (),
                   spoken_text: Optional[str] = None) -> str:
    """Вердикт В17 по итогам хода: действие (любой тул, кроме речи) — без вопроса; ход без речи — без вопроса
    (пустой ответ — не повод спрашивать имя, эпизод Б 20.10). ``ASK`` сразу помечает собеседника спрошенным."""
    tools = tuple(tools or ())
    actions = [name for name in tools if name not in kn.SPEECH_CHANNEL_TOOLS]
    spoken = bool((spoken_text or "").strip()) or any(name in kn.SPEECH_CHANNEL_TOOLS for name in tools)
    verdict = policy.verdict(cand.key, t, has_actions=bool(actions), spoken=spoken)
    if verdict == ASK:
        policy.mark_asked(cand.key, t)
    return verdict
