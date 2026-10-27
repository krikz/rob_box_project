"""speech_client - единственный клиент речи робота (ADR-0150 §6.1, PR-3 #3688).

Всё, что должно прозвучать, идёт одним путём: ``say()`` публикует
``SpeechRequest`` в ``/voice/speech/request``, владелец речи (``tts_node``)
отвечает событиями ``SpeechEvent`` в ``/voice/speech/event``. Отмена - одним
каналом ``/voice/speech/cancel`` (``cancel()``): по ``speech_id`` или по эпохе
агента; реплики другой эпохи и другого агента не трогаются.

JSON-тело ``/voice/tts/request`` вызывающие больше не собирают: SSML
по-прежнему строит :class:`rob_box_core.utterance.Utterance` (экранирование и
санитайзер разметки - в одном месте), клиент кладёт его в ``text``.

Эпоха (``epoch``): номер поколения сессии агента. ``cancel(epoch=E)`` отменяет
реплики агента с эпохой не новее ``E``; реплика с эпохой новее ``E``
(подтверждение «новой сессии») не отменяется - это и заменяет
окно игнора STOP (#1563). ``epoch=0`` - реплика вне эпох (вызывающий их не ведёт):
её отменяет любая отмена агента, устаревшей по эпохе она не бывает.

``rob_box_dialog_msgs`` импортируется лениво: модуль ``rob_box_core`` живёт и
в образах без сообщений диалога, пока они не зовут клиент.
"""

from __future__ import annotations

import time
import uuid
from typing import Any, Callable, Dict, Optional, Tuple

from rob_box_core.utterance import PRIORITY_OPERATOR, Utterance

SPEECH_REQUEST_TOPIC = "/voice/speech/request"
SPEECH_CANCEL_TOPIC = "/voice/speech/cancel"
SPEECH_EVENT_TOPIC = "/voice/speech/event"
SPEECH_ENVELOPE_TOPIC = "/voice/speech/envelope"

#: Автор речи, не ведущий сессию (джойстик, служебные фразы).
SYSTEM_AGENT = "system"
#: Авторы речи (ADR-0150 §6.1). Агенты движка (``knowledge.AGENTS``) плюс
#: :data:`SYSTEM_AGENT` - писатели, не ведущие сессию.
SPEECH_AGENTS = ("personality", "operator", "telegram", SYSTEM_AGENT)
#: Автор события владельца голоса (``speech_owner.voice_changed``: ответ на ``/voice/tts/voice_request``). Только
#: его ``voice_changed`` меняет голос робота (``dialog_node.robot_voice_changed``, #4045); одно место знания для
#: того, кто событие шлёт, и того, кто его читает (ревью #4053).
VOICE_OWNER_AGENT = SYSTEM_AGENT
#: Отмена реплик любого агента (STOP оператора по PTT, старый STOP).
ANY_AGENT = "*"
#: ``cancel(epoch=None)``: все эпохи агента.
EPOCH_ALL = 0xFFFFFFFF

#: ``SpeechRequest.boundary``: где можно резать реплику при отмене.
BOUNDARIES = {"none": 0, "sentence": 1}
#: ``SpeechCancel.boundary``: резать звучащую реплику по её границе из запроса или сразу («стой», ADR-0150 PR-7c).
CANCEL_BOUNDARIES = {"as_requested": 0, "now": 1}
#: ``SpeechRequest.priority`` (uint8, больше - важнее). ``operator`` -
#: врезка сразу за играющим чанком (``tts_node._assign_priority_play_seq``).
PRIORITY_NORMAL = 0
PRIORITY_OPERATOR_U8 = 10

_QOS_DEPTH = 10
_PUBS_ATTR = "_speech_client_pubs"


def _msgs() -> Any:
    from rob_box_dialog_msgs import msg  # noqa: PLC0415 - см. докстринг модуля

    return msg


def _publisher(node: Any, kind: str) -> Any:
    """Один публикатор каждого вида на ноду (кэш в ``node.__dict__``)."""
    pubs: Dict[str, Any] = node.__dict__.setdefault(_PUBS_ATTR, {})
    pub = pubs.get(kind)
    if pub is None:
        msg = _msgs()
        if kind == "request":
            pub = node.create_publisher(msg.SpeechRequest, SPEECH_REQUEST_TOPIC, _QOS_DEPTH)
        else:
            pub = node.create_publisher(msg.SpeechCancel, SPEECH_CANCEL_TOPIC, _QOS_DEPTH)
        pubs[kind] = pub
    return pub


def prepare(node: Any, *kinds: str) -> None:
    """Создать публикаторы заранее (в ``__init__`` ноды): первая отмена после
    старта не должна теряться, пока Zenoh сводит только что созданного писателя."""
    for kind in kinds or ("request", "cancel"):
        _publisher(node, kind)


def _check_agent(agent: str, *, allow_any: bool = False) -> None:
    if agent in SPEECH_AGENTS or (allow_any and agent == ANY_AGENT):
        return
    raise ValueError(f"speech agent {agent!r} не из {SPEECH_AGENTS}")


def _stamp(node: Any) -> Any:
    return node.get_clock().now().to_msg()


def build_request(
    node: Any,
    utterance: Utterance,
    *,
    agent: str,
    epoch: int = 0,
    turn_id: str = "",
    interruptible: bool = True,
    at: float = 0.0,
    boundary: str = "none",
    commit: bool = True,
    ssml: Optional[str] = None,
) -> Any:
    """``SpeechRequest`` из реплики; ``speech_id`` - из реплики или новый uuid4.

    ``ssml`` - готовый SSML вместо ``utterance.ssml`` (prosody эмоции у
    ``speak_text``); экранирует его тот, кто собрал.
    """
    _check_agent(agent)
    if boundary not in BOUNDARIES:
        raise ValueError(f"boundary {boundary!r} не из {sorted(BOUNDARIES)}")
    req = _msgs().SpeechRequest()
    req.speech_id = utterance.speech_id or str(uuid.uuid4())
    req.turn_id = turn_id or ""
    req.epoch = int(epoch)
    req.agent = agent
    req.sink = utterance.sink.value
    req.priority = PRIORITY_OPERATOR_U8 if utterance.priority == PRIORITY_OPERATOR else PRIORITY_NORMAL
    req.interruptible = bool(interruptible)
    req.text = ssml or utterance.ssml
    req.voice = utterance.voice or ""
    req.language = utterance.language or ""
    req.emotion = utterance.emotion or ""
    req.at = float(at)
    req.boundary = BOUNDARIES[boundary]
    req.commit = bool(commit)
    req.stamp = _stamp(node)
    return req


def say(node: Any, utterance: Utterance, **kwargs: Any) -> str:
    """Отдать реплику владельцу речи; вернуть ``speech_id``.

    Аргументы - как у :func:`build_request` (``agent`` обязателен). Итог
    приходит событием ``SpeechEvent`` с тем же ``speech_id``.
    """
    req = build_request(node, utterance, **kwargs)
    _publisher(node, "request").publish(req)
    return req.speech_id


def cancel(
    node: Any,
    *,
    agent: str,
    speech_id: Optional[str] = None,
    epoch: Optional[int] = None,
    reason: str = "",
    boundary: str = "as_requested",
) -> None:
    """Отменить реплику ``speech_id`` или все реплики агента с эпохой не новее ``epoch``.

    ``epoch=None`` - все эпохи агента. ``agent=ANY_AGENT`` - реплики всех
    агентов (STOP оператора); реплики, помеченные непрерываемыми, отмена по
    эпохе не трогает, отмена по ``speech_id`` - трогает. ``boundary="now"`` -
    звучащую реплику резать сразу, даже если в запросе была граница предложения.
    """
    _check_agent(agent, allow_any=True)
    if boundary not in CANCEL_BOUNDARIES:
        raise ValueError(f"boundary {boundary!r} не из {sorted(CANCEL_BOUNDARIES)}")
    msg = _msgs().SpeechCancel()
    msg.speech_id = speech_id or ""
    msg.epoch = EPOCH_ALL if epoch is None else int(epoch)
    msg.agent = agent
    msg.reason = reason
    msg.boundary = CANCEL_BOUNDARIES[boundary]
    _publisher(node, "cancel").publish(msg)


class EpochWatch:
    """Последняя эпоха речи агента по ``/voice/speech/event`` (для отмены «того, что звучит»).

    Писатель отмены, который сам эпох не ведёт (stt_node: wake-word во время
    речи), отменяет реплики не новее увиденной эпохи - подтверждение новой
    сессии, ещё не дошедшее до событий, отмена не задевает.
    """

    def __init__(self, agent: str) -> None:
        _check_agent(agent)
        self.agent = agent
        self.epoch = 0

    def on_event(self, event: Any) -> None:
        if getattr(event, "agent", "") == self.agent:
            self.epoch = max(self.epoch, int(getattr(event, "epoch", 0) or 0))


#: ``SpeechEvent.kind`` (``rob_box_dialog_msgs/msg/SpeechEvent.msg``): началось звучание / реплика закончилась.
KIND_STARTED = 2
#: Итог реплики по виду события: статус результата действия (``knowledge.ACTION_STATUSES``). Единственная таблица:
#: ею читают итог ``say_aloud`` (тул, ожидающий начала) и хост чата (поздний итог по ``speech_id``).
KIND_STATUS = {4: "done", 5: "cancelled", 7: "rejected"}


def is_speech_end(event: Any) -> bool:
    """Это событие конца реплики: ``finished``, ``cancelled`` или ``rejected`` (:data:`KIND_STATUS`)? ``queued`` /
    ``synthesized`` / ``started`` / ``progress`` / ``voice_changed`` - реплика ещё ждёт или звучит. Единственное
    место, где «конец» определён: читатели конца речи (``speaker_id_node``, дирижёр сцен, ``speak_text``, фронт
    ``SpeakingWatch``) зовут его, а не держат свою копию таблицы видов (PR-15 15-0c, #3711)."""
    return int(getattr(event, "kind", -1)) in KIND_STATUS


def event_outcome(event: Any) -> Tuple[str, Optional[str], int]:
    """``SpeechEvent`` -> ``(статус, причина, секунды)`` результата действия: звучит (``started``) - ``pending``,
    конец - по :data:`KIND_STATUS`; ``rejected`` без причины - ``error``, секунды - целые (фраза «Озвучил (12 с)»)."""
    status = KIND_STATUS.get(int(getattr(event, "kind", -1)), "pending")
    reason = str(getattr(event, "reason", "") or "") or ("error" if status == "rejected" else None)
    return status, reason, int(round(float(getattr(event, "seconds", 0.0) or 0.0)))


#: Реплика без события конца дольше этого срока считается отзвучавшей (событие потерялось).
SPEAKING_STALE_S = 120.0


class SpeakingWatch:
    """Звучит ли сейчас речь агента — по ``/voice/speech/event``: ``started`` без ``finished/cancelled/rejected``.

    Канал громкости «громче» (ADR-0150 §4.2): звучит речь — голос робота, а не музыка.
    ``agent=ANY_AGENT`` — речь любого автора, то есть всё, что играет динамик: рот лица и эхо-гейты микрофона
    (``face_node``, ``audio_node``, ``stt_node``; вместо старого String-топика состояния TTS, PR-15 15-0b #3711).
    """

    def __init__(self, agent: str, clock: Callable[[], float] = time.monotonic) -> None:
        _check_agent(agent, allow_any=True)
        self.agent = agent
        self._clock = clock
        self._live: Dict[str, float] = {}

    def on_event(self, event: Any) -> Optional[bool]:
        """Учесть событие; вернуть новое «звучит», если событие его сменило (фронт), иначе ``None``.

        По фронту читатель делает своё один раз: эхо-гейт сбрасывает буфер на ``True`` и запоминает конец речи на
        ``False``; реплики, звучащие внахлёст (оператор поверх личности), фронта не дают."""
        if self.agent != ANY_AGENT and getattr(event, "agent", "") != self.agent:
            return None
        was = self.speaking
        kind, sid = int(getattr(event, "kind", -1)), str(getattr(event, "speech_id", "") or "")
        if kind == KIND_STARTED:
            self._live[sid] = self._clock()
        elif is_speech_end(event):
            self._live.pop(sid, None)
        now = self.speaking
        return now if now != was else None

    @property
    def speaking(self) -> bool:
        now = self._clock()
        self._live = {sid: at for sid, at in self._live.items() if now - at <= SPEAKING_STALE_S}
        return bool(self._live)


def watch_speaking(node: Any, on_change: Callable[[bool], None], agent: str = ANY_AGENT) -> SpeakingWatch:
    """Подписать узел на ``/voice/speech/event``: ``on_change(speaking)`` — на каждый фронт «звучит» речи ``agent``
    (по умолчанию любого автора — всё, что играет динамик). Эхо-гейты микрофона (``audio_node``, ``stt_node``)
    держат только реакцию на фронт; что такое «звучит» — :class:`SpeakingWatch`. Возвращает наблюдателя."""
    watch = SpeakingWatch(agent)

    def on_event(event: Any) -> None:
        speaking = watch.on_event(event)
        if speaking is not None:
            on_change(speaking)

    node.create_subscription(_msgs().SpeechEvent, SPEECH_EVENT_TOPIC, on_event, _QOS_DEPTH)
    return watch


__all__ = [
    "ANY_AGENT",
    "BOUNDARIES",
    "CANCEL_BOUNDARIES",
    "EPOCH_ALL",
    "EpochWatch",
    "KIND_STARTED",
    "KIND_STATUS",
    "PRIORITY_NORMAL",
    "PRIORITY_OPERATOR_U8",
    "SPEECH_AGENTS",
    "SPEECH_CANCEL_TOPIC",
    "SPEECH_ENVELOPE_TOPIC",
    "SPEECH_EVENT_TOPIC",
    "SPEECH_REQUEST_TOPIC",
    "SPEAKING_STALE_S",
    "SpeakingWatch",
    "build_request",
    "cancel",
    "event_outcome",
    "is_speech_end",
    "prepare",
    "say",
    "watch_speaking",
]
