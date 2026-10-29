"""Кадр ``Identity(ask)`` и ``/dialog/session`` в ``dialogue_node`` v1 (ADR-0150 §7.1, §7.2, §7.5 п.2; #3695).

Вопрос об имени — кадр стека сессии (:class:`rob_box_dialog.session.Session`): код кладёт его, когда вопрос ушёл в речь (до конца вопроса он ждёт),
окно ответа ``IDENTITY_TTL_S`` (15 с) открывается, когда вопрос договорён (``SpeechEvent finished``), и снимается первой адресованной фразой. Пока кадр жив, фраза без вейка
— ответ на него, если похожа на ответ по грамматике (В17(б), правило — ``speaker_adapter.answer_to_question``). Ответ «меня зовут X» регистрирует
голос кодом (#2925, ``_register_self_intro``), голое имя на «Как тебя зовут?» — тоже (:func:`answer_name`).

Вопрос склейки после регистрации («вы разные люди или это ты?», #2828; #3765) — тот же кадр (``kind`` twin/conflict,
:func:`open_merge_question`) с тем же окном. Ответ решает код (:func:`identity_reply`): склеить или нет и тот ли человек
ответил — ``speaker_adapter.merge_answer``; что сказать — фраза ``phrases``, LLM на этот ход не зовётся.

#3801 (решение Шифу 21.10) — на тех же кадре и окне: голос уверенно узнан, а человек назвался другим именем — второй
профиль не заводится (:func:`refuse_second_profile`, кадр ``same_voice``); «зови меня Y» / «меня зовут не X, а Y» —
переименование профиля штатным ``/voice/speaker/rename``; неуверенный голос (короткий ответ) — просьба сказать шаблон
нормальной длины (кадр ``voice_check``). Решения — ``speaker_adapter`` (``rename_step`` и др.), фразы — ``phrases``.

Каждое изменение сессии (собеседник, кадр) уходит latched ``SessionState`` в ``/dialog/session``: собеседник с
азимутом его фразы — для взгляда лица (``face_node``), до PR-7 пишет v1 ``dialogue_node``.

Функции, а не методы: ``DialogueNode`` в бюджете классов ADR-0145 расти не может.
"""

from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from typing import Any, Dict, Mapping, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.reply_grammar import NAME, classify_rename, classify_reply
from rob_box_dialog.session import Session, SessionChanged, SessionSnapshot, SpeakerRef
from rob_box_dialog.speaker_adapter import (
    VOICE_STRANGER,
    AskCandidate,
    IdentityStep,
    merge_answer,
    merge_step,
    question_frame,
    question_open,
    question_reply,
    rename_step,
    same_voice_plan,
    same_voice_step,
    voice_certainty,
    voice_check_step,
)
from rob_box_voice.core.self_intro import KNOWN_FIRST_NAMES, extract_self_intro_name, same_person_name

#: Топик снимка сессии (``architecture/ownership.yml``).
SESSION_TOPIC = "/dialog/session"
#: ``Identity.kind`` кадров, где ответ — имя: голое имя («Гриша») — представление.
ASKS_NAME = kn.IDENTITY_ASKS_NAME
#: Кадр вопроса, который задала сама LLM (#3765): окно открывает конец речи хода, а не конкретный ``speech_id``.
ANY_SPEECH = "*"
#: ``SpeechEvent.kind``: finished / cancelled / rejected.
_FINISHED, _CANCELLED, _REJECTED = 4, 5, 7


def session(node: Any) -> Session:
    """Сессия ноды (лениво — для нод из тестов через ``object.__new__``); изменения — в ``/dialog/session``."""
    current = getattr(node, "_session", None)
    if current is None:
        current = node._session = Session("personality", clock=time.time)
        current.subscribe(lambda event: publish(node, event))
    return current


def set_speaker(node: Any, ref: SpeakerRef) -> None:
    session(node).set_speaker(ref)


def open_question(node: Any, cand: AskCandidate, speech_id: str) -> None:
    """Вопрос отдан в речь — кадр ``Identity`` с азимутом фразы собеседника, пока ЖДЁТ конца речи ``speech_id``.

    Окно ответа открывает :func:`on_speech_event` (``SpeechEvent finished`` этого вопроса): ход мог отвечать 12 с,
    вопрос звучит после, и 15 с от конца хода истекли бы до вопроса (#3771)."""
    _push(node, question_frame(cand.key, cand.kind, cand.name, _azimuth(node), None, speech_id))


def open_merge_question(node: Any, plan: Mapping[str, Any], speech_id: str) -> None:
    """Вопрос склейки (#2828) отдан в речь — тот же кадр ``Identity`` (``kind`` twin/conflict), окно — как у
    вопроса об имени (#3765). Данные вопроса (кого с кем склеивать) — в кадре. Так же — фраза ``same_voice`` (#3801):
    нового профиля нет, кадр про узнанного."""
    frame = question_frame(plan["new_id"] or plan["known_id"], plan["kind"], plan.get("known_name"), _azimuth(node), None, speech_id)
    _push(node, dict(frame, **{k: plan.get(k, "") for k in ("new_id", "new_name", "known_id", "known_name")}))


def _azimuth(node: Any) -> Optional[float]:
    ref = getattr(node, "_speaker_ref", None)
    return ref.azimuth_deg if ref is not None else None


def _push(node: Any, frame: Mapping[str, Any]) -> None:
    session(node).pop_scene("Identity")
    session(node).push_scene("Identity", "code", ttl_s=float(kn.IDENTITY_WAIT_QUESTION_S), payload=frame)
    azimuth = frame.get("azimuth_deg")
    node.get_logger().info(
        f"❓ [Identity #3695] кадр ждёт конца вопроса: kind={frame['kind']} key={str(frame['person_ref'])[:8]} "
        f"azimuth={'-' if azimuth is None else f'{azimuth:.0f}'} speech_id={str(frame['speech_id'])[:8]}"
    )


def on_speech_event(node: Any, kind: int, speech_id: str, agent: str) -> None:
    """``SpeechEvent`` речи личности: вопрос договорён — открыть окно ответа; отменён/отклонён — снять кадр."""
    if agent != "personality" or not speech_id or kind not in (_FINISHED, _CANCELLED, _REJECTED):
        return
    current = session(node)
    frame = current.snapshot.top("Identity")
    if frame is None or frame.payload.get("speech_id") not in (speech_id, ANY_SPEECH) or question_open(frame):
        return
    if frame.payload.get("speech_id") == ANY_SPEECH:
        if kind == _FINISHED and not _speaking(node):
            _arm(node, frame)  # вопрос задала LLM (#3765): окно — от конца речи хода
        return
    if kind != _FINISHED:
        current.pop_scene("Identity")
        node.get_logger().info(
            f"❓ [Identity #3695] кадр снят: вопрос не прозвучал (SpeechEvent kind={kind} speech_id={speech_id[:8]})")
        return
    _arm(node, frame)


def _speaking(node: Any) -> bool:
    watch = getattr(node, "_speaking_watch", None)
    return bool(watch is not None and watch.speaking)  # SpeakingWatch.speaking — свойство (живой прогон 3)


def _arm(node: Any, frame: Any) -> None:
    current = session(node)
    ttl = float(kn.IDENTITY_MIC_GRACE_S + kn.IDENTITY_TTL_S)
    current.arm_scene("Identity", ttl, {"asked_at": current.now() + kn.IDENTITY_MIC_GRACE_S})
    node.get_logger().info(
        f"❓ [Identity #3695] кадр открыт для ответа: kind={frame.payload.get('kind')} "
        f"azimuth={frame.payload.get('azimuth_deg')} ttl={kn.IDENTITY_TTL_S}s (вопрос договорён, "
        f"микрофон после речи +{kn.IDENTITY_MIC_GRACE_S}s)")


def question(node: Any) -> Optional[Any]:
    """Кадр ``Identity`` с открытым окном ответа (истёкшие сессия снимает) или ``None``; ждущий конца вопроса — ``None``."""
    current = session(node)
    current.expire()
    frame = current.snapshot.top("Identity")
    return frame if question_open(frame) else None


def take_answer(node: Any, source: str, heard: Any) -> None:
    """Адресованная голосовая фраза при живом кадре — ответ на вопрос: кадр снимается (первая же фраза, как #2809),
    данные вопроса остаются на ход (``_identity_answer``) — для :func:`answer_name`."""
    node._identity_answer = None
    frame = question(node)
    if frame is None or (getattr(heard, "source", "") or "respeaker") != "respeaker":
        return
    session(node).pop_scene("Identity")
    text = str(getattr(heard, "text", "") or "")
    reply = question_reply(str(frame.payload.get("kind") or ""), text)
    node._identity_answer = dict(frame.payload, reply=reply.kind if reply else "other", text=text)
    doa = getattr(heard, "doa_deg", None)
    azimuth = frame.payload.get("azimuth_deg")
    node.get_logger().info(
        f"❓ [Identity #3695] ответ на вопрос: kind={reply.kind if reply else 'other'} source={source} doa={'-' if doa is None else f'{doa:.0f}'} "
        f"azimuth={'-' if azimuth is None else f'{azimuth:.0f}'} "
        f"age={float(getattr(heard, 't', 0.0) or 0.0) - float(frame.payload.get('asked_at') or 0.0):.1f}s"
    )


def answer_name(node: Any, text: str) -> Optional[str]:
    """Имя из ответа на «Как тебя зовут?» («Гриша», «зови меня Гриша») — представление; иначе ``None``.

    Только имя из словаря ``self_intro.KNOWN_FIRST_NAMES`` (точность, а не полнота — как короткое «я X», #2925).
    """
    asked = getattr(node, "_identity_answer", None)
    if not asked or asked.get("kind") not in ASKS_NAME:
        return None
    reply = classify_reply(text)
    known = reply is not None and reply.kind == NAME and reply.name.lower().replace("ё", "е") in KNOWN_FIRST_NAMES
    return reply.name if known else None


def identity_reply(node: Any) -> Optional[str]:
    """Личность в этом ходе решает код — фраза об исходе; ``None`` — ход обычный (LLM).

    Звать после того, как биометрия ЭТОЙ фразы резолвнута (``_prepare_user_input_context``): действие с личностью
    подтверждает только её голос (#2829). Ответ на кадр ``Identity`` кода (``IDENTITY_CODED_KINDS``): склейка (#2828),
    «один голос — одна личность» и шаблон (#3801); без кадра — просьба звать иначе («зови меня Y», #3801)."""
    asked = getattr(node, "_identity_answer", None)
    if asked and asked.get("kind") in kn.IDENTITY_CODED_KINDS:
        node._identity_answer = None
        step = _answer_step(node, asked, _speaker(node))
    else:
        step = rename_step(classify_rename(str(getattr(node, "_turn_heard_text", "") or "")), _turn_speaker(node))
        if step is not None:
            node.get_logger().info(
                f"✏️ [#3801] смена имени фразой: rename={step.rename or '-'} шаблон={step.frame is not None} — 0 LLM")
    if step is None:
        return None
    _apply(node, step)
    node._identity_question_asked_in_turn = True  # гарды: ход ответил кодом — ретрая поверх нет (#2914)
    return step.phrase


def code_decides(node: Any, text: str) -> bool:
    """Ответ этого хода решит :func:`identity_reply` — по представлению он не регистрируется (#3801: шаблон «Меня
    зовут Y, и это мой голос» и «меня зовут не X, а Y» — не новый человек)."""
    asked = getattr(node, "_identity_answer", None)
    if asked and asked.get("kind") in kn.IDENTITY_CODED_KINDS:
        return True
    return classify_rename(text) is not None and voice_certainty(_turn_speaker(node)) != VOICE_STRANGER


def _speaker(node: Any) -> Dict[str, Any]:
    with node._speaker_lock:
        return dict(getattr(node, "_current_speaker", None) or {})


def _turn_speaker(node: Any) -> Dict[str, Any]:
    """Результат биометрии фразы ЭТОГО хода; у хода без своей фразы (Telegram, ретрай) — ``{}``: имя не меняется."""
    uid = getattr(node, "_current_turn_utterance_id", None)
    if not uid or uid != getattr(node, "_last_resolved_utterance_id", None):
        return {}
    return _speaker(node)


def _answer_step(node: Any, asked: Mapping[str, Any], speaker: Mapping[str, Any]) -> Optional[IdentityStep]:
    kind, text = asked.get("kind"), str(asked.get("text", ""))
    node.get_logger().info(
        f"👥 [#3801] ответ на кадр {kind}: reply={asked.get('reply')} голос={voice_certainty(speaker)} "
        f"({str(speaker.get('speaker_id') or 'unknown')[:8]})")
    if kind == kn.IDENTITY_VOICE_CHECK_KIND:
        return voice_check_step(asked, speaker)
    renamed = rename_step(classify_rename(text), speaker)
    if renamed is not None:
        return renamed
    if kind == kn.IDENTITY_SAME_VOICE_KIND:
        return same_voice_step(asked, text) if asked.get("reply") != "other" else None
    return _merge_answer_step(node, asked, text, speaker)


def _merge_answer_step(node: Any, asked: Mapping[str, Any], text: str,
                       speaker: Mapping[str, Any]) -> Optional[IdentityStep]:
    """Ответ на вопрос склейки (#2828): решение — ``speaker_adapter.merge_step``; фраза не по теме — ход обычный."""
    same, speaker_ok = merge_answer(asked, text, speaker)
    if same is None and asked.get("reply") == "other":
        node.get_logger().info("👥 [issue #2828] вопрос склейки снят фразой не по теме — ничего не склеено")
        return None
    if not speaker_ok:
        node.get_logger().warning(
            f"👥 [issue #2829] ответ на переспрос #2828 не подтверждает склейку — диктор реплики "
            f"({str(speaker.get('speaker_id') or 'unknown')[:8]}) не совпадает ни с new={asked['new_id'][:8]}, "
            f"ни с known={asked['known_id'][:8]}")
    node.get_logger().info(
        f"👥 [issue #2828] ответ на переспрос: same={same} kind={asked['kind']} new={asked['new_id'][:8]} "
        f"known={asked['known_id'][:8]} — фраза кодом, 0 LLM (#3765)")
    return merge_step(asked, text, speaker)


def _apply(node: Any, step: IdentityStep) -> None:
    if step.merge:
        _merge(node, dict(zip(("new_id", "known_id", "known_name"), step.merge)))
    if step.rename:
        _rename(node, *step.rename)
    node._identity_followup = step.frame  # кадр voice_check откроет deliver_turn_result после фразы


def deliver_turn_result(node: Any, result: Any, coded_reply: Optional[str], **kwargs: Any) -> None:
    """Выдать ответ хода. Фраза кода (:func:`identity_reply`) — терминальная: звучит как есть, мимо гардов LLM-текста
    (живой робот 21.10: гард #2549 счёл «Запомню обоих» галлюцинацией и отправил CRITICAL-ретрай в LLM). Просьба
    шаблона (#3801) кладёт кадр ``Identity(voice_check)`` — окно от конца этой фразы, как у вопроса."""
    if not coded_reply:
        node._handle_result(result, **kwargs)
        return
    speech_id = node._speak_direct(coded_reply)
    frame = getattr(node, "_identity_followup", None)
    node._identity_followup = None
    if frame and speech_id:
        person = frame.get("known_id") or frame.get("new_id") or ""
        _push(node, dict(question_frame(person, frame["kind"], frame.get("name"), _azimuth(node), None, speech_id),
                         **{k: v for k, v in frame.items() if k != "kind"}))


def _merge(node: Any, plan: Mapping[str, Any]) -> None:
    """«Это я» — склеить ``new_id`` в ``known_id`` и поправить собеседника хода."""
    pub = getattr(node, "_speaker_merge_pub", None)
    if pub is None or not (plan["new_id"] and plan["known_id"]):
        node.get_logger().warning("👥 [issue #2828] склеить нечем: нет merge-паблишера или id")
        return
    from std_msgs.msg import String

    msg = String()
    msg.data = json.dumps({"src_speaker_id": plan["new_id"], "dst_speaker_id": plan["known_id"]}, ensure_ascii=False)
    pub.publish(msg)
    with node._speaker_lock:
        if node._current_speaker.get("speaker_id") == plan["new_id"]:
            node._current_speaker["speaker_id"] = plan["known_id"]
            node._current_speaker["name"] = plan["known_name"]


def _rename(node: Any, speaker_id: str, name: str) -> None:
    """Переименовать профиль штатным путём (``/voice/speaker/rename`` → ``speaker_id_node._on_rename_request``, тот же,
    что у тула ``register_speaker`` с ``old_name``) и поправить собеседника хода."""
    pub = getattr(node, "_speaker_rename_pub", None)
    if pub is None or not speaker_id:
        node.get_logger().warning("✏️ [#3801] переименовать нечем: нет rename-паблишера или id")
        return
    from std_msgs.msg import String

    msg = String()
    msg.data = json.dumps({"speaker_id": speaker_id, "new_name": name}, ensure_ascii=False)
    pub.publish(msg)
    node.get_logger().info(f"✏️ [#3801] переименование профиля {speaker_id[:8]} → {name!r}")
    with node._speaker_lock:
        if node._current_speaker.get("speaker_id") == speaker_id:
            node._current_speaker["name"] = name


def settled_by_voice(node: Any, intro: str, known: Optional[str]) -> bool:
    """Регистрация по представлению не нужна (``True``): фраза уверенно узнана под этим же именем (#2925) — регистрировать
    нечего; под другим — один голос, одна личность (#3801, :func:`refuse_second_profile`)."""
    if not known:
        return False
    if same_person_name(known, intro):
        node.get_logger().info(f"👤 [issue #2925] {intro!r} уже узнан по голосу -- регистрация по представлению не нужна")
        return True
    return refuse_second_profile(node, intro, known)


def refuse_second_profile(node: Any, intro: str, known: str) -> bool:
    """Один голос — одна личность (#3801): фраза уверенно узнана как ``known``, человек назвался ``intro`` — регистрации
    нет, ``register_speaker`` LLM в этом ходе не видит, после ответа хода робот говорит ``identity.same_voice`` и ждёт
    ответа кадром ``Identity(same_voice)``. ``False`` — у узнанного нет id (тогда регистрация, как раньше)."""
    speaker_id = str(_speaker(node).get("speaker_id") or "")
    if not speaker_id:
        return False
    node._turn_hidden_tools = ("register_speaker",)
    held = node._queue_identity_question(same_voice_plan(speaker_id, known, intro))
    node.get_logger().info(
        f"👥 [#3801] один голос — одна личность: голос уверенно {known!r} ({speaker_id[:8]}), назвался {intro!r} — "
        f"второй профиль не заводится, register_speaker скрыт, фраза после ответа хода={held}")
    return True


async def settle_identity(node: Any, timeout_s: float = kn.REGISTER_OUTCOME_WAIT_S) -> None:
    """До вызова LLM: ход отправил регистрацию по представлению (#2925) — дождаться исхода. Голос похож на другого
    (ack с voice_conflict/name_twin, вопрос склейки отложен до конца хода) — личность выясняет робот: в снимке хода
    собеседник неизвестен (ни имени, ни сведений о совпадении голоса), ``register_speaker`` LLM не видит (#3765;
    живой робот 21.10: LLM сама обсуждала совпадение голоса и переименовывала профиль, а потом звучал вопрос кода)."""
    if not getattr(node, "_turn_intro_registered", False):
        return
    deadline = time.monotonic() + timeout_s
    while getattr(node, "_turn_register_outcome", None) is None and time.monotonic() < deadline:
        await asyncio.sleep(0.02)
    plan = getattr(node, "_after_turn_question", None)
    if not plan or plan.get("kind") not in kn.IDENTITY_MERGE_KINDS:
        return
    with node._speaker_lock:
        # собеседник — тот, кем назвался; ни статуса биометрии, ни заметки о неопознанном голосе (живой робот 21.10:
        # «unknown» + privacy_note → LLM говорила «голосом я тебя пока не опознал» перед вопросом кода)
        node._current_speaker = {"is_known": False, "introduced_name": getattr(node, "_turn_self_intro", None) or ""}
    node._turn_hidden_tools = ("register_speaker",)
    node.get_logger().info(
        f"👥 [#3765] личность выясняет робот (kind={plan['kind']}): ход LLM без register_speaker и без сведений о голосе")


def start_after_turn_question(node: Any, plan: Optional[Mapping[str, Any]], result: Any) -> None:
    """Вопрос личности после ответа хода (#2828/#2908). LLM в ходе что-то сказала и есть Chooser
    (:mod:`.identity_dedup`) — решение «уже спросила?» идёт ПАРАЛЛЕЛЬНО с речью хода (:func:`ask_after_turn` в своём
    потоке: loop ноды между ходами не крутится — живой прогон 3, задача в loop'е заснула до следующего хода); иначе
    вопрос уходит в речь сразу, как без Chooser'а."""
    if plan is None:
        return
    said = list(getattr(node, "_turn_llm_said", None) or ())
    spoken = str(getattr(result, "spoken_text", "") or "").strip()
    if spoken and spoken.lower() not in ("done", "готово"):
        said.append(spoken)
    if getattr(node, "_identity_dedup", None) is None or not any(s.strip() for s in said):
        node._speak_identity_question(plan)
        return
    node._after_turn_task = threading.Thread(target=ask_after_turn, args=(node, plan, said), daemon=True,
                                             name="identity-dedup")
    node._after_turn_task.start()


def ask_after_turn(node: Any, plan: Mapping[str, Any], said: Any) -> None:
    """Дождаться вердикта Chooser'а не дольше конца речи хода + ``IDENTITY_DEDUP_AFTER_SPEECH_S``. ``already_said`` —
    фраза кода не звучит, кадр ``Identity`` всё равно открывается — от конца речи хода; не успел — ``not_said``."""
    if _dedup_verdict(node, said, plan["question"]) != "already_said":
        node._speak_identity_question(plan)
        return
    node._identity_question_asked_in_turn = True
    if plan.get("kind") in kn.IDENTITY_CODED_KINDS:
        open_merge_question(node, plan, ANY_SPEECH)
        if not _speaking(node):
            _arm(node, session(node).snapshot.top("Identity"))


def _dedup_verdict(node: Any, said: Any, phrase: str) -> str:
    started = time.monotonic()
    box: dict = {}
    worker = threading.Thread(target=lambda: box.update(v=node._identity_dedup.decide(list(said), phrase)),
                              daemon=True, name="identity-dedup-model")
    worker.start()
    ended_at: Optional[float] = None
    heard = False  # речь хода ещё синтезируется (живой прогон 3: «не говорит» сразу после хода) — конец считаем после начала
    while worker.is_alive():
        now = time.monotonic()
        if _speaking(node):
            heard, ended_at = True, None
        elif not heard:
            pass
        elif ended_at is None:
            ended_at = now
        elif now - ended_at >= kn.IDENTITY_DEDUP_AFTER_SPEECH_S:
            break
        time.sleep(0.05)
    waited = 0.0 if ended_at is None else (time.monotonic() - ended_at) * 1000.0
    done = "v" in box
    verdict = box["v"][0] if done else "not_said"
    node.get_logger().info(
        f"🔁 [identity-dedup] итог verdict={verdict} ms={(time.monotonic() - started) * 1000.0:.0f} ждали после конца "
        f"речи: {waited:.0f} мс{'' if done else ' (Chooser не успел — not_said)'}")
    return verdict


def introduced_name(node: Any, text: str) -> Optional[str]:
    """Имя, которым человек назвал себя в реплике хода: «меня зовут X» (#2925) или голое имя в ответ на вопрос.
    Ответ, который решает код (:func:`code_decides`: шаблон, склейка, смена имени, #3801), — не представление."""
    if code_decides(node, text):
        return None
    return extract_self_intro_name(text) or answer_name(node, text)


def publish(node: Any, event: SessionChanged) -> None:
    pub = getattr(node, "_session_pub", None)
    if pub is None:
        return
    try:
        pub.publish(session_state(event.snapshot))
    except Exception as exc:  # noqa: BLE001 — снимок для взгляда и панели не должен ронять ход
        node.get_logger().warning(f"⚠️ [#3695] /dialog/session не опубликован: {type(exc).__name__}: {exc}")


def session_state(snap: SessionSnapshot) -> Any:
    """``SessionSnapshot`` → ``rob_box_dialog_msgs/SessionState`` (§10.1)."""
    from rob_box_dialog_msgs.msg import SceneFrame, SessionState

    msg = SessionState()
    msg.agent, msg.session_id, msg.epoch = snap.agent, snap.session_id, int(snap.epoch)
    msg.scenes = [_scene(SceneFrame(), f) for f in snap.scenes]
    msg.voice_requested, msg.voice_applied = snap.voice.requested or "", snap.voice.applied or ""
    speaker = snap.speaker or SpeakerRef()
    msg.speaker_id, msg.speaker_name = speaker.person_ref or "", speaker.name or ""
    msg.speaker_name_state = speaker.name_state
    msg.speaker_azimuth_deg = math.nan if speaker.azimuth_deg is None else float(speaker.azimuth_deg)
    msg.pending_ask = str(getattr(snap.pending_ask, "id", "") or "")
    _time(msg.stamp, time.time())
    return msg


def _scene(msg: Any, frame: Any) -> Any:
    msg.kind, msg.set_by = frame.kind, frame.set_by
    _time(msg.ttl_until, frame.ttl_until or 0.0)
    msg.payload_json = json.dumps(dict(frame.payload), ensure_ascii=False)
    return msg


def _time(msg: Any, t: float) -> Any:
    msg.sec, msg.nanosec = int(t), int((t - int(t)) * 1e9)
    return msg


__all__ = ["SESSION_TOPIC", "answer_name", "code_decides", "identity_reply", "introduced_name", "on_speech_event",
           "open_merge_question", "open_question", "question", "refuse_second_profile", "session", "session_state",
           "set_speaker", "settle_identity", "settled_by_voice", "take_answer", "ask_after_turn", "deliver_turn_result",
           "start_after_turn_question", "ANY_SPEECH"]
