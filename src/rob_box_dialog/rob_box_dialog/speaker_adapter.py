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
  С #3767 — не фильтр подхвата, а поле ``same_as_caller`` метки фразы.
* :func:`voice_tag` / :func:`phrase_speaker` — кто сказал фразу бэклога (#3767, Шифу 21.10): узнан уверенно
  (имя биометрии или гипотеза голоса, подтверждённая лицом с тем же именем) — конкретный спикер с id, каналами и
  score; иначе ``неопознанный`` + что есть по голосу: ``bio``, гипотеза/лучший кандидат со score — не имя.
* :func:`merge_plan` / :func:`merge_answer` / :func:`merge_phrase` — вопрос «вы разные люди или это ты?» после
  регистрации (#2828, ADR-0127; #3765): что спрашиваем, склеивать ли по ответу (и тот ли человек ответил, #2829), что
  робот говорит об исходе — кодом, без подсказки LLM (ADR-0148).
* :func:`voice_certainty` / :class:`IdentityStep` — #3801 (решение Шифу 21.10): один голос — одна личность
  (:func:`same_voice_plan`), имя меняется фразой (:func:`rename_step`), неуверенный голос подтверждается шаблоном
  нормальной длины (:func:`template_step`, :func:`voice_check_step`); что сделать и что сказать — кодом.
* :class:`AskPolicy` — когда можно спросить имя (В17): не на команду, не на пустой ход, не раньше N-й фразы/T с,
  один раз на собеседника. Команда — по ЗАПРОСУ человека (:func:`~rob_box_dialog.tools_view.command_groups`),
  а не по тому, какие тулы выбрала LLM.

Когда ADR-0130 этапы 2/3/5 дадут треки и реестр Знакомых, меняется только этот модуль. Чистый: время передают
вызывающие, сообщения читаются утиной типизацией.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, FrozenSet, Iterable, List, Mapping, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.doa import angle_diff_deg
from rob_box_dialog.greeting_reply import is_greeting_reply
from rob_box_dialog.reply_grammar import NAME, NO, YES, Rename, Reply, classify_rename, classify_reply, classify_same
from rob_box_dialog.session import SceneFrame, SpeakerRef
from rob_box_dialog.tools_view import command_groups

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


@dataclass(frozen=True)
class VoiceTag:
    """Что биометрия сказала о фразе: ``bio`` — ``known`` | ``unknown_voice`` | ``inconclusive``; ``name`` — только
    уверенное имя; ``tentative`` — гипотеза при ``known``; ``candidate`` — лучший кандидат без порога у незнакомого."""

    bio: str
    person_ref: str = ""
    name: str = ""
    tentative: str = ""
    score: Optional[float] = None
    candidate: str = ""
    candidate_score: Optional[float] = None
    #: Внутренняя кличка узнанного профиля (#1787) — различает тёзок; плейсхолдер «незнакомца» сюда не кладут.
    epithet: str = ""


def _score(value: Any) -> Optional[float]:
    return None if value is None else round(float(value), 3)


def voice_tag(result: Mapping[str, Any]) -> VoiceTag:
    """Результат биометрии фразы (словарь ``rob_box_core.speaker_wire``) → :class:`VoiceTag`."""
    candidate = str(result.get("candidate_name") or "")
    cand_score = _score(result.get("candidate_score")) if candidate else None
    if result.get("inconclusive"):
        return VoiceTag("inconclusive", candidate=candidate, candidate_score=cand_score)
    if not result.get("is_known"):
        return VoiceTag("unknown_voice", candidate=candidate, candidate_score=cand_score)
    state, name = _name_state(result)
    return VoiceTag("known", person_ref=str(result.get("speaker_id") or ""),
                    name=name if state == "confirmed" else "", tentative=name if state == "tentative" else "",
                    score=_score(result.get("confidence")), epithet=str(result.get("epithet") or ""))


def phrase_speaker(voice: Optional[VoiceTag], faces: Iterable[str] = ()) -> Dict[str, Any]:
    """Кто сказал фразу (#3767): узнан — ``{identified, name, person_ref, channels, score}``; нет —
    ``{identified: false, label: неопознанный, bio, candidate?, candidate_score?}``. ``faces`` — кто был в кадре во
    время фразы (имена в нижнем регистре): одно лицо в кадре фразу не подписывает, только подтверждает гипотезу
    голоса."""
    in_frame = {f for f in faces if f and not f.startswith("#")}
    if voice is not None and voice.name:
        return _identified(voice, voice.name, ["voice"])
    if voice is not None and voice.tentative and voice.tentative.lower() in in_frame:
        return _identified(voice, voice.tentative, ["voice", "face"])
    out: Dict[str, Any] = {"identified": False, "label": kn.UNIDENTIFIED_SPEAKER,
                           "bio": voice.bio if voice is not None else "none"}
    if voice is not None and (voice.tentative or voice.candidate):
        out["candidate"] = voice.tentative or voice.candidate
        out["candidate_score"] = voice.score if voice.tentative else voice.candidate_score
    return out


def _identified(voice: VoiceTag, name: str, channels: List[str]) -> Dict[str, Any]:
    out = {"identified": True, "name": name, "person_ref": voice.person_ref, "channels": channels,
           "score": voice.score}
    if voice.epithet:
        out["epithet"] = voice.epithet  # тёзки: два профиля «Дэнчик» (#1787); вслух не произносится
    return out


def speaker_block(ref: Optional[SpeakerRef]) -> Dict[str, Any]:
    """Раздел ``speaker`` снимка хода (§7.3, §7.5): ``{name, name_state}``. Имя — только при ``confirmed``:
    по гипотезе обращаться нельзя (эпизод Б 20.10 — «Литвин» к Дэнчику; A30). Вместо префиксов в тексте реплики."""
    if ref is None:
        return {"name": None, "name_state": "unknown"}
    return {"name": ref.name if ref.name_state == "confirmed" else None, "name_state": ref.name_state}


def question_frame(person_ref: str, kind: str, name: Optional[str], azimuth_deg: Optional[float],
                   asked_at: Optional[float], speech_id: str = "") -> Dict[str, Any]:
    """Данные кадра ``Identity(ask)`` (§7.2, §7.5 п.2): кого спросили, о чём, с какой стороны и когда.

    ``asked_at`` — когда микрофон снова слышит (конец вопроса + ``IDENTITY_MIC_GRACE_S``); ``None`` — вопрос ещё не договорён (кадр ждёт ``SpeechEvent finished`` речи ``speech_id``, #3771)."""
    return {"person_ref": person_ref, "kind": kind, "name": name, "azimuth_deg": azimuth_deg, "asked_at": asked_at,
            "speech_id": speech_id}


def question_open(frame: Optional[SceneFrame]) -> bool:
    """Вопрос договорён и окно ответа идёт: кадр ``Identity`` с ``asked_at`` (до этого он «ждёт», ответа нет)."""
    return frame is not None and frame.kind == "Identity" and frame.payload.get("asked_at") is not None


def answer_to_question(frame: Optional[SceneFrame], heard: Any) -> Optional[Reply]:
    """Фраза без вейка — ответ на вопрос робота (В17(б), решение Шифу 21.10): кадр ``Identity`` жив (TTL — у сессии),
    фраза пришла после открытия окна (``asked_at`` = конец вопроса + грейс микрофона), не позже ``IDENTITY_TTL_S``,
    и ПОХОЖА НА ОТВЕТ по грамматике (:func:`reply_grammar.classify_reply`: имя, да/нет, отказ). DOA и голос — не
    условия (DOA в комнате прыгает), только признаки в логе. Не похоже — ``None`` (бэклог). Это не окно адресации
    (§17 Ж): ответить можно только на свой вопрос."""
    if not question_open(frame) or not kn.IDENTITY_ANSWER_WITHOUT_WAKE:
        return None
    age = float(getattr(heard, "t", 0.0) or 0.0) - float(frame.payload["asked_at"])
    if not 0.0 <= age <= kn.IDENTITY_TTL_S:
        return None
    return question_reply(str(frame.payload.get("kind") or ""), str(getattr(heard, "text", "") or ""))


def question_reply(kind: str, text: str) -> Optional[Reply]:
    """Похожа ли фраза на ответ на вопрос вида ``kind`` (кадр ``Identity``): на вопрос склейки (``IDENTITY_MERGE_KINDS``)
    — «это я»/«мы разные» (``YES``/``NO``) или да/нет; на вопрос об имени — имя/да/нет/отказ."""
    rename = classify_rename(text)
    if rename is not None:  # «меня зовут не X, а Y» — ответ-имя, а не «Не» (#3801)
        return Reply(NAME, rename.name)
    if kind in kn.IDENTITY_MERGE_KINDS + (kn.IDENTITY_SAME_VOICE_KIND,):
        same = classify_same(text)
        if same is not None:
            return Reply(YES if same else NO)
    return classify_reply(text, asks_name=kind in kn.IDENTITY_ASKS_NAME)


def merge_plan(ack: Mapping[str, Any], question: str) -> Dict[str, Any]:
    """Данные вопроса склейки из ack регистрации (#2828): ``new_id`` — профиль этой регистрации, ``known_id`` — с кем его
    можно спутать (тёзка или похожий голос). Склейка при «это я» — ``new_id`` → ``known_id``: у старого профиля больше
    эмбеддингов и история, его имя и остаётся (``speaker_id_node._on_merge_request``)."""
    other = ack.get("name_twin") or ack.get("voice_conflict") or {}
    return {"kind": "twin" if ack.get("name_twin") else "conflict", "question": question,
            "new_id": str(ack.get("speaker_id") or ""), "new_name": str(ack.get("name") or ""),
            "known_id": str(other.get("speaker_id") or ""), "known_name": str(other.get("name") or "")}


def said_same(plan: Mapping[str, Any], text: str) -> Optional[bool]:
    """Что человек СКАЗАЛ на вопрос «один человек или двое?»: ``True`` — один, ``False`` — разные, ``None`` — не понять.
    Голое да/нет — ответ только на ``twin`` («Ты тот самый X?») и ``same_voice``; на ``conflict`` вопрос двойной."""
    same = classify_same(text, max_words=None)
    if same is None and plan.get("kind") in ("twin", kn.IDENTITY_SAME_VOICE_KIND):
        reply = classify_reply(text, asks_name=False)
        same = {YES: True, NO: False}.get(reply.kind) if reply is not None else None
    return same


def merge_answer(plan: Mapping[str, Any], text: str, speaker: Mapping[str, Any]) -> Tuple[Optional[bool], bool]:
    """Решение по ответу на вопрос склейки: ``(same, speaker_ok)``. ``same`` — ``True`` склеить, ``False`` — разные,
    ``None`` — не понять (:func:`said_same`).
    «Да» засчитывается, только если ответил один из двух профилей (``speaker`` — результат биометрии ЭТОЙ фразы,
    #2829): посторонний голос чужую склейку не подтверждает. Длина фразы не ограничена: её уже приняли как ответ."""
    same = said_same(plan, text)
    current = str(speaker.get("speaker_id") or "") if speaker.get("is_known") else ""
    speaker_ok = bool(current) and current in (plan.get("new_id"), plan.get("known_id"))
    return (None if same and not speaker_ok else same), speaker_ok


def merge_phrase(plan: Mapping[str, Any], same: Optional[bool]) -> str:
    """Что робот говорит об исходе (ADR-0148: фразу строит код, а не подсказка LLM)."""
    if same is True:
        return phrases.text("identity.merged", name=plan.get("known_name") or plan.get("new_name"))
    return phrases.text("identity.kept_apart" if same is False else "identity.merge_unclear")


# ── #3801 (решение Шифу 21.10): один голос — одна личность, имя меняется фразой, подтверждение шаблоном ───────────
#: :func:`voice_certainty`
VOICE_SURE, VOICE_UNSURE, VOICE_STRANGER = "sure", "unsure", "stranger"


def voice_certainty(speaker: Mapping[str, Any]) -> str:
    """Годится ли голос ЭТОЙ фразы для действий с личностью. ``sure`` — у результата биометрии есть имя (порог 0.72 и
    отрыв от соседа проверил ``speaker_id_node``; порогов здесь нет); ``unsure`` — узнан без имени (гипотеза, спор),
    биометрия фразу не оценила (мало речи) или лучший кандидат не ниже ``IDENTITY_TEMPLATE_MIN_SCORE`` — тогда робот
    просит шаблон; иначе ``stranger`` (чужой голос: чужую личность не трогает, #2829)."""
    if speaker.get("is_known"):
        return VOICE_SURE if speaker.get("name") and speaker.get("speaker_id") else VOICE_UNSURE
    if speaker.get("inconclusive"):
        return VOICE_UNSURE
    score = speaker.get("candidate_score")
    return VOICE_UNSURE if score is not None and float(score) >= kn.IDENTITY_TEMPLATE_MIN_SCORE else VOICE_STRANGER


@dataclass(frozen=True)
class IdentityStep:
    """Что код делает с личностью в этом ходе: ``phrase`` — что робот говорит (0 LLM, ADR-0148); ``rename`` —
    ``(speaker_id, новое имя)``; ``merge`` — ``(src, dst, имя dst)``; ``frame`` — кадр ``Identity(voice_check)`` после фразы."""

    phrase: str
    rename: Tuple[str, ...] = ()
    merge: Tuple[str, ...] = ()
    frame: Optional[Mapping[str, Any]] = None


def _same_name(left: Any, right: Any) -> bool:
    a, b = (str(x or "").strip().lower().replace("ё", "е") for x in (left, right))
    return bool(a) and a == b


def same_voice_plan(speaker_id: str, known: str, name: str) -> Dict[str, Any]:
    """Голос уверенно узнан как ``known``, а человек назвался ``name``: второй профиль не заводится, после ответа хода
    робот говорит ``identity.same_voice``, ответ ждёт кадр ``Identity(same_voice)`` (окно ``REPLY_WINDOW_S``)."""
    return {"kind": kn.IDENTITY_SAME_VOICE_KIND, "question": phrases.text("identity.same_voice", known=known, name=name),
            "new_id": "", "new_name": name, "known_id": speaker_id, "known_name": known}


def template_step(action: str, name: str, plan: Optional[Mapping[str, Any]] = None) -> IdentityStep:
    """Попросить сказать шаблон ``identity.template``; кадр ``voice_check`` помнит, что сделать (``rename`` | ``merge``),
    если голос шаблона уверенно совпадёт."""
    frame = {"kind": kn.IDENTITY_VOICE_CHECK_KIND, "action": action, "name": name}
    frame.update({k: str((plan or {}).get(k) or "") for k in ("new_id", "new_name", "known_id", "known_name")})
    template = phrases.text("identity.template", name=name)
    return IdentityStep(phrases.text("identity.say_template", template=template), frame=frame)


def rename_step(rename: Optional[Rename], speaker: Mapping[str, Any]) -> Optional[IdentityStep]:
    """«Зови меня Y» / «меня зовут не X, а Y»: уверенный голос — переименовать ЕГО профиль; неуверенный — просьба
    шаблона; чужой голос или просьбы нет — ``None`` (ход обычный)."""
    certainty = voice_certainty(speaker) if rename is not None else VOICE_STRANGER
    if rename is None or certainty == VOICE_STRANGER:
        return None
    if certainty == VOICE_UNSURE:
        return template_step("rename", rename.name)
    if _same_name(speaker.get("name"), rename.name):
        return IdentityStep(phrases.text("identity.name_same", name=rename.name))
    return IdentityStep(phrases.text("identity.renamed", name=rename.name),
                        rename=(str(speaker["speaker_id"]), rename.name))


def same_voice_step(plan: Mapping[str, Any], text: str) -> IdentityStep:
    """Ответ на ``identity.same_voice``: «это я» — согласие; «мы разные» и прочее — голос один, второй записи нет."""
    if said_same(plan, text):
        return IdentityStep(phrases.text("identity.same_voice_ok", name=plan.get("known_name")))
    return IdentityStep(phrases.text("identity.one_voice", name=plan.get("new_name")))


def merge_step(plan: Mapping[str, Any], text: str, speaker: Mapping[str, Any]) -> IdentityStep:
    """Ответ на вопрос склейки: «это я» тем же голосом — склейка ``new → known``; «это я», а голос неуверенный
    (короткий ответ) — просьба шаблона вместо «Не понял ответ»; остальное — :func:`merge_answer`."""
    same, _ = merge_answer(plan, text, speaker)
    if same is None and said_same(plan, text) and voice_certainty(speaker) == VOICE_UNSURE:
        return template_step("merge", str(plan.get("known_name") or plan.get("new_name") or ""), plan)
    return IdentityStep(merge_phrase(plan, same), merge=_merge_ids(plan) if same is True else ())


def _merge_ids(plan: Mapping[str, Any]) -> Tuple[str, ...]:
    return tuple(str(plan.get(k) or "") for k in ("new_id", "known_id", "known_name"))


def voice_check_step(frame: Mapping[str, Any], speaker: Mapping[str, Any]) -> IdentityStep:
    """Ответ шаблоном: голос уверенно совпал — действие кадра (переименовать этот профиль / склеить, если голос — один из
    двух профилей вопроса); нет — честная фраза, ничего не меняется."""
    sid = str(speaker.get("speaker_id") or "") if voice_certainty(speaker) == VOICE_SURE else ""
    if sid and frame.get("action") == "rename":
        return IdentityStep(phrases.text("identity.renamed", name=frame.get("name")), rename=(sid, str(frame["name"])))
    if sid and frame.get("action") == "merge" and sid in (frame.get("new_id"), frame.get("known_id")):
        return IdentityStep(merge_phrase(frame, True), merge=_merge_ids(frame))
    return IdentityStep(phrases.text("identity.voice_check_failed"))


def greeting_frame(person_ref: str, name: str, speech_id: str) -> Dict[str, Any]:
    """Данные кадра ``GreetingReply`` (#3776): кого поприветствовали; ``asked_at`` ``None`` — приветствие ещё звучит
    (кадр ждёт ``SpeechEvent finished`` речи ``speech_id``)."""
    return {"person_ref": person_ref, "name": name, "asked_at": None, "speech_id": speech_id}


def greeting_open(frame: Optional[SceneFrame]) -> bool:
    return frame is not None and frame.kind == "GreetingReply" and frame.payload.get("asked_at") is not None


def answer_to_greeting(frame: Optional[SceneFrame], heard: Any) -> str:
    """Фраза без вейка после приветствия при встрече (решение Шифу 21.10, #3776): окно ``REPLY_WINDOW_S`` открыто, и
    это либо ответ по грамматике приветствия (``"grammar"``), либо лицо приветствованного в кадре в момент фразы
    (``"face"``; канал лица надёжен). DOA — не условие. Иначе ``""`` (бэклог)."""
    if not greeting_open(frame) or not kn.GREETING_REPLY_WITHOUT_WAKE:
        return ""
    age = float(getattr(heard, "t", 0.0) or 0.0) - float(frame.payload["asked_at"])
    if not 0.0 <= age <= kn.REPLY_WINDOW_S:
        return ""
    if is_greeting_reply(str(getattr(heard, "text", "") or "")):
        return "grammar"
    name = str(frame.payload.get("name") or "").lower()
    return "face" if name and name in (getattr(heard, "faces", ()) or ()) else ""


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

    def progress(self, key: str, t: float) -> Tuple[int, float, Optional[float]]:
        """Накоплено у ``key``: фраз, секунд с первой фразы, когда спрашивали (``None`` — не спрашивали)."""
        first, count = self._heard.get(key, (t, 0))
        return count, t - first, self._asked.get(key)

    def mark_asked(self, key: str, t: float) -> None:
        self._asked[key] = t
        self._heard.pop(key, None)


class TentativeStates:
    """Гипотеза имени по собеседнику («<Имя>, это ты?», #2809/#2888; ADR-0150 §7.5): спрашивали ли, что он ответил,
    подтверждено ли имя и рос ли профиль. Ключ — ``person_ref``.

    Состояние переживает паузу ``answer_window_s``, пока ответа нет (ответ бывает через 84 с — run 35857257981), и
    ``session_gap_s`` после решения; дольше — новый разговор, состояние с нуля. Время передаёт вызывающий.
    """

    def __init__(self, answer_window_s: float = 180.0, session_gap_s: float = 120.0) -> None:
        self.answer_window_s, self.session_gap_s = answer_window_s, session_gap_s
        self._states: Dict[str, Dict[str, Any]] = {}

    def get(self, person_ref: str) -> Optional[Dict[str, Any]]:
        return self._states.get(person_ref)

    def fresh(self, person_ref: str, now: float) -> Dict[str, Any]:
        """Состояние собеседника на эту фразу: прежнее, если пауза не дольше окна, иначе новое."""
        state = self._states.get(person_ref)
        if state is None or now - state.get("last_seen_at", 0.0) > self._window(state):
            state = self._states[person_ref] = {"asked": False, "confirmed": None, "name": None,
                                                "growth_registered": False}
        state["last_seen_at"] = now
        return state

    def _window(self, state: Mapping[str, Any]) -> float:
        waiting = state.get("asked") and state.get("confirmed") is None
        return self.answer_window_s if waiting else self.session_gap_s


@dataclass(frozen=True)
class AskCandidate:
    """Кого можно спросить после хода: ``single`` с именем-гипотезой — «<Имя>, это ты?», иначе — «Как тебя зовут?»."""

    key: str
    kind: str
    name: Optional[str] = None
    utterance_id: Optional[str] = None
    #: реплика человека этого хода — по ней (а не по тулам LLM) решается, была ли это команда
    request: str = ""

    def question(self) -> str:
        if self.kind == "single" and self.name:
            return phrases.text("identity.confirm", name=self.name)
        return phrases.text("identity.ask_name")


@dataclass(frozen=True)
class AskVerdict:
    """Вердикт В17 с причиной — одна строка лога объясняет, почему спросили или нет.

    ``command_by`` — группы ``knowledge.TOOL_GROUPS`` с действием, названные человеком (``()`` — не команда);
    ``phrases``/``min_phrases``, ``since_first_s`` — накопление до хода; ``asked_at`` — когда спрашивали раньше.
    """

    verdict: str
    command_by: Tuple[str, ...]
    phrases: int
    min_phrases: int
    since_first_s: float
    asked_at: Optional[float]

    def describe(self, t: float) -> str:
        asked = "no" if self.asked_at is None else f"yes({t - self.asked_at:.0f}s ago)"
        return (f"{self.verdict} command_by={'+'.join(self.command_by) or 'none'} "
                f"phrases={self.phrases}/{self.min_phrases} since_first={self.since_first_s:.0f}s asked_before={asked}")


def ask_after_turn(policy: AskPolicy, cand: AskCandidate, *, t: float, tools: Sequence[str] = (),
                   spoken_text: Optional[str] = None) -> AskVerdict:
    """Вердикт В17 по итогам хода. Команда — исполненная просьба человека (:func:`command_groups`: группа названа
    словами ``cand.request`` и её действие вызвано): без вопроса. Тулы, о которых человек не просил, командой ход не
    делают (живой робот 21.10: «как у тебя дела» → ``get_robot_status``, «расскажи про луну» → LLM добавила
    ``play_animation`` — это не команды). Ход без речи — без вопроса (пустой ответ — не повод, эпизод Б 20.10).
    ``ASK`` сразу помечает собеседника спрошенным."""
    tools = tuple(tools or ())
    spoken = bool((spoken_text or "").strip()) or any(name in kn.SPEECH_CHANNEL_TOOLS for name in tools)
    command_by = command_groups(cand.request, tools)
    count, since_first, asked_at = policy.progress(cand.key, t)
    verdict = policy.verdict(cand.key, t, has_actions=bool(command_by), spoken=spoken)
    if verdict == ASK:
        policy.mark_asked(cand.key, t)
    return AskVerdict(verdict, command_by, count, policy.min_phrases, since_first, asked_at)
