"""Бэклог неадресованных фраз (ADR-0150 §7.3, ревизия 5, решения В12, В12′; #3767): помнит, но LLM не отдаёт.

Фраза без вейка роботу не адресована (В2) и копится здесь как :class:`Heard` на
:data:`~rob_box_dialog.knowledge.BACKLOG_TTL_S`. Наружу она выходит только двумя путями, оба — решение кода:

* :meth:`Backlog.pickup` — голое «робот» (исправление пропущенного вейка) забирает **всегда** все фразы не старше
  :data:`~rob_box_dialog.knowledge.BACKLOG_FRESH_S` и более ранние куски того же рассказа (речь длиннее 15 с
  audio_node режет на куски, #3763), по порядку; они становятся текстом хода и из бэклога уходят (решение Шифу
  21.10, #3767: «бэклог должен уходить по любому всегда»). Свежего нет — берёт последнюю фразу окна
  :data:`~rob_box_dialog.knowledge.BACKLOG_PICKUP_S` (3 минуты, решение Шифу 28.10, #3975), которую робот ещё не слышал
  адресованной, и её куски рассказа. «Тот же собеседник» не фильтр, а поле метки;
* :meth:`Backlog.recent_unaddressed` — в снимок адресованного хода с фразой не больше одной записи того же
  собеседника (:func:`~rob_box_dialog.speaker_adapter.same_interlocutor`) — телевизор туда не попадает;
* :meth:`Backlog.background_since_reply` — в снимок адресованного хода с фразой отдельным разделом все фоновые
  фразы всех авторов за последние :data:`~rob_box_dialog.knowledge.BACKGROUND_WINDOW_S` (решение Шифу 23.10, В23: «в
  бэклоге должна быть инфа кто чего сказал и когда») — чтобы «кто говорил на фоне?» было из чего отвечать.

Каждая отданная фраза — :meth:`Backlog.label`: текст, возраст, длительность, DOA и кто её сказал
(:func:`~rob_box_dialog.speaker_adapter.phrase_speaker`: узнанный спикер или «неопознанный» + что есть по голосу;
``speaker_id`` — профиль голоса или ``unknown``, у каждой фразы, #4145). Эхо собственной речи робота сюда не
попадает: ``stt_node`` его в диалог не отдаёт (``rob_box_voice.core.echo_window``, #4145).
Всё остальное (телевизор, чужие разговоры) живёт только для счётчика «фоновая речь» и умирает по TTL.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Dict, FrozenSet, List, Mapping, Optional, Sequence

from rob_box_core.speaker_wire import NAME_UNKNOWN
from rob_box_dialog.doa import angle_diff_deg
from rob_box_dialog.knowledge import (
    BACKGROUND_MAX_PHRASES,
    BACKGROUND_PHRASE_MAX_CHARS,
    BACKGROUND_WINDOW_S,
    BACKLOG_FRESH_S,
    BACKLOG_MAX_ENTRIES,
    BACKLOG_PICKUP_MAX,
    BACKLOG_PICKUP_S,
    BACKLOG_STORY_GAP_S,
    BACKLOG_TTL_S,
    PICKUP_DOA_SECTOR_DEG,
)
from rob_box_dialog.speaker_adapter import VoiceTag, phrase_speaker, same_interlocutor


@dataclass(frozen=True)
class Heard:
    """Реплика в момент приёма: текст, источник, когда прозвучала, откуда, от кого и кто был в кадре."""

    text: str
    t: float
    source: str = "respeaker"
    doa_deg: Optional[float] = None
    person_ref: str = ""
    #: Длительность фразы, с: голосу короткой фразы не верим (``knowledge.VOICE_MIN_PHRASE_S``).
    voice_s: float = 0.0
    #: Кто был в кадре, пока звучала фраза (``FaceSightings.during``: имя или ``#id``).
    faces: FrozenSet[str] = frozenset()
    utterance_id: str = ""
    #: Имя узнанного голоса фразы (биометрия, :meth:`Backlog.note_voice`) — для сверки с лицом в кадре.
    speaker_name: str = ""
    #: Что биометрия сказала о фразе (:meth:`Backlog.note_voice`); ``None`` — результат не пришёл.
    voice: Optional[VoiceTag] = None


def same_direction(a: Optional[float], b: Optional[float]) -> bool:
    """Оба направления известны и лежат в одном секторе ±``PICKUP_DOA_SECTOR_DEG``."""
    return a is not None and b is not None and angle_diff_deg(a, b) <= PICKUP_DOA_SECTOR_DEG


class Backlog:
    """Неадресованные фразы на TTL; наружу — по голому «робот» все свежие, в снимок — свежая того же собеседника."""

    def __init__(self, ttl_s: float = BACKLOG_TTL_S, max_entries: int = BACKLOG_MAX_ENTRIES) -> None:
        self.ttl_s, self.max_entries = ttl_s, max_entries
        self._entries: List[Heard] = []
        #: Когда роботу в последний раз сказали фразу с вейком (или подхватили бэклог): всё, что звучало раньше, он уже
        #: слышал разделами снимка того хода и голым «робот» издалека не подхватывается (:meth:`_last_unheard`).
        self._addressed_t = float("-inf")

    def add(self, heard: Heard) -> None:
        if not heard.text.strip():
            return
        self._prune(heard.t)
        self._entries.append(heard)
        del self._entries[:-self.max_entries]

    def clear(self) -> None:
        self._entries.clear()
        self._addressed_t = float("-inf")

    def note_addressed(self, t: float) -> None:
        """Фраза с вейком в момент ``t``: фоновое до неё робот уже видел (снимок того хода) — издалека не подхватывать."""
        self._addressed_t = max(self._addressed_t, t)

    def background_phrases(self, now: float) -> int:
        """Сколько фраз фоновой речи лежит в бэклоге (метрика ``backlog.background_phrases``)."""
        self._prune(now)
        return len(self._entries)

    def note_voice(self, utterance_id: str, voice: VoiceTag) -> None:
        """Биометрия фразы пришла позже STT: дописать её записи с этим ``utterance_id``."""
        name = voice.name or voice.tentative
        for i, entry in enumerate(self._entries):
            if utterance_id and entry.utterance_id == utterance_id:
                self._entries[i] = replace(entry, person_ref=voice.person_ref, speaker_name=name, voice=voice)

    def pickup(self, heard: Heard) -> List[Heard]:
        """Голое «робот» (В12, #3767): забрать все свежие фразы и примыкающие к ним куски рассказа, по порядку; свежего
        нет — последнюю фразу окна ``BACKLOG_PICKUP_S`` и её куски (#3975). Подхват тоже считается «слышал»."""
        self._prune(heard.t)
        fresh = [i for i, e in enumerate(self._entries) if 0.0 <= heard.t - e.t <= BACKLOG_FRESH_S]
        fresh = fresh or self._last_unheard(heard.t)
        if not fresh:
            return []
        first = fresh[0]
        while first > 0 and _continues(self._entries[first - 1], self._entries[first]):
            first -= 1
        taken = self._entries[first:fresh[-1] + 1][-BACKLOG_PICKUP_MAX:]
        self._entries = [e for e in self._entries if all(e is not t for t in taken)]
        self.note_addressed(heard.t)
        return taken

    def _last_unheard(self, now: float) -> List[int]:
        """Индекс последней фразы окна ``BACKLOG_PICKUP_S``, прозвучавшей после последнего адресованного хода; ``[]`` -
        такой нет. Одна фраза, а не всё окно: чужие разговоры мастерской за три минуты в ход не тянутся (#3975)."""
        heard_late = [i for i, e in enumerate(self._entries)
                      if e.t > self._addressed_t and 0.0 <= now - e.t <= BACKLOG_PICKUP_S]
        return heard_late[-1:]

    def recent_unaddressed(self, heard: Heard) -> Optional[Dict[str, Any]]:
        """Не больше одной свежей записи того же собеседника для снимка адресованного хода; чужого не отдаёт."""
        self._prune(heard.t)
        for entry in reversed(self._entries):
            if heard.t - entry.t <= BACKLOG_FRESH_S and same_interlocutor(entry, heard):
                return self.label(entry, heard)
        return None

    def background_since_reply(self, heard: Heard) -> List[Dict[str, Any]]:
        """Все фоновые фразы всех авторов за ``BACKGROUND_WINDOW_S``, по порядку, не больше ``BACKGROUND_MAX_PHRASES``
        последних, текст — до ``BACKGROUND_PHRASE_MAX_CHARS``; каждая — :meth:`label`. Из бэклога не уходят."""
        self._prune(heard.t)
        since = [e for e in self._entries if 0.0 <= heard.t - e.t <= BACKGROUND_WINDOW_S]
        return [{**self.label(e, heard), "text": e.text[:BACKGROUND_PHRASE_MAX_CHARS]}
                for e in since[-BACKGROUND_MAX_PHRASES:]]

    @staticmethod
    def label(entry: Heard, heard: Heard) -> Dict[str, Any]:
        """Фраза для LLM структурой (#3767): что сказано, когда, сколько звучала, откуда и кто сказал. ``speaker_id`` —
        автор у каждой фразы (решение Шифу 28.10, #4145): профиль, которому биометрия приписала голос (и безымянный
        с кличкой, #3858), или ``unknown`` — голос не узнан, спорный или результата нет."""
        return {"text": entry.text, "age_s": round(heard.t - entry.t, 1), "duration_s": round(entry.voice_s, 1),
                "doa_deg": None if entry.doa_deg is None else round(entry.doa_deg),
                "same_direction": same_direction(entry.doa_deg, heard.doa_deg),
                "speaker": phrase_speaker(entry.voice, entry.faces),
                "speaker_id": entry.person_ref or NAME_UNKNOWN,
                "same_as_caller": bool(same_interlocutor(entry, heard))}

    def _prune(self, now: float) -> None:
        self._entries = [e for e in self._entries if now - e.t <= self.ttl_s]


def _continues(prev: Heard, nxt: Heard) -> bool:
    """``prev`` — более ранний кусок того же рассказа: кончился не раньше чем за паузу до начала ``nxt``."""
    return prev.t >= nxt.t - nxt.voice_s - BACKLOG_STORY_GAP_S


def pickup_summary(picked: Sequence[Mapping[str, Any]]) -> str:
    """``n=<k> speaker=<…> channels=<…> span=<с>`` подхвата (#3767): кто сказал каждую фразу, по каким каналам он
    узнан и сколько секунд занял весь подхваченный рассказ."""
    speakers = [p["speaker"].get("name") or p["speaker"].get("label") or "-" for p in picked]
    channels = sorted({ch for p in picked for ch in p["speaker"].get("channels", ())})
    span = max((p["age_s"] + p["duration_s"] for p in picked), default=0.0) - min(
        (p["age_s"] for p in picked), default=0.0)
    return (f"n={len(picked)} speaker={','.join(speakers) or '-'} channels={','.join(channels) or '-'} "
            f"span={span:.1f}s")


def phrase_count(recent: Optional[Sequence[Any]]) -> int:
    """Сколько фраз в разделе ``recent_unaddressed`` (``None`` — ни одной) — для строк лога."""
    return len(recent or ())
