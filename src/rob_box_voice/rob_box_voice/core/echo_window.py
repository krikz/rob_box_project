"""Эхо-окно микрофона после речи робота (#989 Fix B, #4072): решает, КОГДА звучал голос фразы, а не когда она доехала.

Было два окна по 2,5 с после КАЖДОЙ реплики робота: ``audio_node`` глушил VAD (микрофон глух), ``stt_node``
выбрасывал фразу, доставленную в окне. Выступление звучит сегментами с паузой 1–2 с, поэтому:

* «Робот, хватит» в паузе между сегментами микрофон не слышал, а старт следующего сегмента сбрасывал начатую
  фразу — до STT доезжал хвост без вейка («замолчи», марафон 27.10, n805_hard_stop);
* фраза, сказанная во время реплики, доезжает до STT через ``speech_continuation`` (3 с) после последнего VAD —
  уже в окне после конца реплики — и выбрасывалась как эхо (va_marathon.log 23.10, 22:48:33 и 22:53:21: голос
  человека, speaker_id 0.89 и 0.888).

Стало: окно одно — здесь, у ``stt_node``. ``audio_node`` слышит всегда, как во время речи (barge-in #993), а
:class:`EchoWindow` по фронтам ``/audio/vad`` помнит, где звучал голос ещё не доставленной фразы: во время речи
робота (:data:`SPEECH`), в эхо-окне после её конца, пока робот молчит (:data:`GRACE`), или в тишине (:data:`FREE`).
Решение по фразе (:meth:`EchoWindow.admit`):

* голос звучал только в эхо-окне — это эхо собственного голоса или хвост реплики: фраза распознаётся, но проходит
  только ясная стоп-команда с вейком (:func:`stop_passes`, «стоп важнее»); остальное — молча, без «не расслышал»
  (петля #989);
* иначе — как раньше: software-AEC глушит фразы, пока робот говорит; hardware-AEC короткий обрывок при звучащей речи
  (< :data:`SHORT_DURING_SPEECH_S`) распознаёт, но пропускает только ясный стоп с вейком: «Робот, стоп» бывает короче
  порога, и режь мы его до STT — стоп в паузе между сегментами пропал бы (ревью #4122); щелчок и эхо до диалога не
  доходят, как и раньше. Остальное идёт дальше — перебивание через вейк решает диалог.

Эхо словами (#4145, агитация из Telegram 28.10, speech_id=46fd3b27): hardware-AEC пропускает длинную фразу во время
речи, чтобы человек мог перебить, а голос робота, который ReSpeaker не вычел, STT распознал его же словами
(«дорогие друзья слушайте и не говорите что не слышали»). Фраза легла в бэклог как реплика человека, биометрия завела
из неё профиль «Наблюдатель». Теперь журнал речи (:class:`SpeechJournal`: текст реплики — из ``SpeechRequest`` по
``speech_id``, когда звучала — из ``SpeechEvent``) даёт фразе, звучавшей рядом с речью, реплики робота; после
распознавания :meth:`Admission.echo` сверяет с ними её слова (:func:`echo_of`) — эхо не идёт ни в бэклог, ни к
модели. Решение одно, его слушает и биометрия: эхо уходит в :data:`ECHO_TOPIC`, ``speaker_id_node`` по фразе рядом с
речью ждёт вердикта (:class:`EchoVerdicts`) и по эху профиль не заводит и не растит.

Длина окна — одна константа знания ``knowledge.IDENTITY_MIC_GRACE_S`` (дефолт параметра ``stt_node.tts_grace_s``).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from typing import (
    AbstractSet,
    Any,
    Callable,
    Dict,
    FrozenSet,
    Optional,
    Sequence,
    Set,
    Tuple,
)

from rob_box_core import speech_client
from rob_box_dialog import knowledge as kn
from rob_box_dialog.words import (
    STOP_CLEAR,
    has_wake_word,
    name_tokens,
    stop_verdict,
    words_in_order,
)

#: Где звучал голос: во время речи робота, в эхо-окне после её конца (робот молчит), в тишине.
SPEECH, GRACE, FREE = "speech", "grace", "free"

#: Вердикты :meth:`EchoWindow.admit`.
PASS = "pass"
STOP_ONLY = "stop_only"  # голос звучал только в эхо-окне после речи робота
SHORT_STOP_ONLY = "short_stop_only"  # короткая фраза при звучащей речи (hardware-AEC)
DROP_SPEAKING = "drop_speaking"
#: Вердикты, после которых фраза идёт дальше только как ясный стоп с вейком (``Admission.keeps``).
STOP_ONLY_VERDICTS = (STOP_ONLY, SHORT_STOP_ONLY)

#: Короче этого фраза при звучащей речи робота — эхо/щелчок (hardware-AEC, #989).
SHORT_DURING_SPEECH_S = 0.8
#: Длинную речь audio_node режет на куски и шлёт их подряд (#3762): кусок без своих фронтов VAD берёт места
#: предыдущего куска той же фразы, если доехал не позже этого.
CHUNK_GAP_S = 1.0

#: Эхо словами (#4145): фраза повторяет звучавшую реплику робота, если не меньше стольких её слов идут в реплике по
#: порядку (:func:`~rob_box_dialog.words.words_in_order`) и это не меньше доли :data:`ECHO_WORD_SHARE` слов фразы.
#: Фраза короче — как раньше: пара слов человека («да, давай») совпасть с длинной речью робота может случайно.
ECHO_MIN_WORDS = 3
#: Доля слов фразы, совпавших с репликой робота: запас на ошибки STT в эхе (7 из 9 слов — эхо).
ECHO_WORD_SHARE = 0.6
#: Сколько последних реплик робота помнит журнал речи.
JOURNAL_SIZE = 32
#: Биометрия ждёт вердикта ``stt_node`` по фразе рядом с речью робота не дольше (распознавание с повтором и запасным
#: Vosk); не дождалась — текста нет или фраза отброшена до распознавания — профили как раньше.
VERDICT_TIMEOUT_S = 20.0
#: Окно «рядом с речью» у биометрии шире, чем у ``stt_node``: то же PCM ноды получают с разницей в миллисекунды.
VERDICT_SLACK_S = 1.0
#: Сколько вердиктов, пришедших раньше эмбеддинга, помнит биометрия (холодный энкодер считает до ~50 с, #2885).
VERDICTS_KEPT = 64
#: Как часто биометрия снимает просроченное ожидание вердикта.
VERDICT_TICK_S = 1.0

VAD_TOPIC = "/audio/vad"
#: Фраза ReSpeaker, признанная эхом речи робота (``Utterance``): её слушает биометрия (#4145).
ECHO_TOPIC = "/voice/stt/echo"
#: Фраза человека, ушедшая в диалог: для биометрии — вердикт «не эхо».
UTTERANCE_TOPIC = "/voice/stt/utterance"
_QOS_DEPTH = 10
#: Строки лога фронта речи робота: software-AEC глушит распознавание, hardware — XVF-3000 фильтрует эхо.
_SOFTWARE_EDGE_LOG = {
    True: "🔇 [software AEC] Робот говорит - распознавание отключено",
    False: "🎙️ [software AEC] Робот замолчал - распознавание включено",
}
_HARDWARE_EDGE_LOG = {
    True: "🎤 [hardware AEC] Робот говорит - XVF-3000 фильтрует эхо",
    False: "🎙️ [hardware AEC] Робот замолчал — эхо-окно {grace_s}с",
}


def stop_passes(text: str, wake_words: Sequence[str]) -> bool:
    """Фраза из эхо-окна проходит, только если это ясная стоп-команда с вейком.

    Правило то же, по которому диалог рвёт речь по ``/dialog/interrupt_request``: вейк (``has_wake_word``) и
    ``stop_verdict == clear`` — «Робот, хватит», «Робот, всё, хватит, замолчи». Эхо собственной речи без вейка
    и стоп-слова первым словом команды не проходит."""
    return (
        has_wake_word(text, wake_words) and stop_verdict(text, wake_words) == STOP_CLEAR
    )


@dataclass(frozen=True)
class Spoken:
    """Реплика робота из журнала речи: ``speech_id`` и её текст (``""`` — запроса журнал не видел)."""

    speech_id: str
    text: str


@dataclass(frozen=True)
class Echo:
    """Фраза — эхо собственной речи робота: дальше не идёт; ``note`` — строка лога, ``speech_id`` — чья речь
    (``""`` — голос звучал только в эхо-окне, #4072)."""

    note: str
    speech_id: str = ""


def echo_of(text: str, spoken: Sequence[Spoken]) -> Optional[Echo]:
    """Реплика робота, чьи слова фраза в основном повторяет (#4145); ``None`` — не эхо, фраза идёт как раньше.

    Эхо — не меньше :data:`ECHO_MIN_WORDS` слов фразы идут в реплике по порядку и это не меньше
    :data:`ECHO_WORD_SHARE` её слов; нормализация слов одна — :mod:`rob_box_dialog.words`.
    """
    total = len(name_tokens(text))
    for said in spoken:
        same = words_in_order(text, said.text)
        if same >= ECHO_MIN_WORDS and same >= ECHO_WORD_SHARE * total:
            return Echo(
                f"🔁 [echo] фраза совпадает с речью робота speech_id={said.speech_id} — не беру "
                f"(слов {same}/{total}: {text!r})",
                said.speech_id,
            )
    return None


def _remember(table: Dict[str, Any], key: str, value: Any, size: int) -> None:
    """Записать ``key`` последним; старейшие записи сверх ``size`` забыть."""
    table.pop(key, None)
    table[key] = value
    while len(table) > size:
        table.pop(next(iter(table)))


class SpeechJournal:
    """Журнал речи робота (#4145): текст реплики — из ``SpeechRequest`` по ``speech_id``, когда она звучала — из
    ``SpeechEvent`` (``started`` … ``finished|cancelled|rejected``). Фронт «звучит / замолчал» отдаёт тот же
    :class:`~rob_box_core.speech_client.SpeakingWatch`, что у всех эхо-гейтов микрофона.

    ``plain`` — текст реплики без SSML (``speech_owner.plain_text`` у ``stt_node``)."""

    def __init__(
        self,
        grace_s: float,
        clock: Callable[[], float] = time.monotonic,
        plain: Callable[[str], str] = str,
        size: int = JOURNAL_SIZE,
    ) -> None:
        self.grace_s = float(grace_s)
        self._clock, self._plain, self._size = clock, plain, size
        self._texts: Dict[str, str] = {}
        self._spans: Dict[str, Tuple[float, float]] = {}
        self._speaking = speech_client.SpeakingWatch(speech_client.ANY_AGENT, clock)

    def on_request(self, msg: Any) -> None:
        """``SpeechRequest``: запомнить текст реплики (прогрев кэша ``commit=false`` — не речь)."""
        if getattr(msg, "commit", True):
            _remember(
                self._texts,
                str(msg.speech_id or ""),
                self._plain(str(msg.text or "")),
                self._size,
            )

    def on_event(self, event: Any) -> Optional[bool]:
        """``SpeechEvent``: отметить начало/конец звучания реплики; вернуть фронт «звучит» (``None`` — фронта нет)."""
        sid, now = str(getattr(event, "speech_id", "") or ""), self._clock()
        if int(getattr(event, "kind", -1)) == speech_client.KIND_STARTED:
            _remember(self._spans, sid, (now, float("inf")), self._size)
        elif speech_client.is_speech_end(event) and sid in self._spans:
            self._spans[sid] = (self._spans[sid][0], now)
        return self._speaking.on_event(event)

    def sounded(self, seconds: float) -> Tuple[Spoken, ...]:
        """Реплики, звучавшие за последние ``seconds`` или кончившиеся в эхо-окне перед ними, по порядку начала.
        Реплика без события конца дольше ``SPEAKING_STALE_S`` — отзвучала (событие потерялось).
        """
        since = self._clock() - float(seconds) - self.grace_s
        return tuple(
            Spoken(sid, self._texts.get(sid, ""))
            for sid, (start, end) in self._spans.items()
            if min(end, start + speech_client.SPEAKING_STALE_S) >= since
        )


@dataclass(frozen=True)
class Admission:
    """Решение по доставленной фразе ReSpeaker: ``verdict`` и строка лога ``note`` (``""`` — молча); ``spoken`` —
    реплики робота, звучавшие рядом с фразой: с ними после распознавания сверяются её слова (#4145).
    """

    verdict: str
    note: str = ""
    spoken: Tuple[Spoken, ...] = ()

    @property
    def dropped(self) -> bool:
        """Фраза отброшена до распознавания."""
        return self.verdict == DROP_SPEAKING

    def keeps(self, text: Optional[str], wake_words: Sequence[str]) -> bool:
        """После распознавания (и на каждом промежуточном результате STT): фраза идёт дальше? Из эхо-окна и короткая при
        звучащей речи — только ясный стоп с вейком. Пока фраза не пропущена, диалог о ней не знает: ни результата,
        ни ``/dialog/interrupt_request`` (``stt_node._offer_interrupt``)."""
        return self.verdict not in STOP_ONLY_VERDICTS or stop_passes(
            text or "", wake_words
        )

    def echo_note(self, text: Optional[str], duration_s: float) -> str:
        """Строка лога отброшенной после распознавания фразы: эхо-окно или короткий обрывок при речи робота."""
        if self.verdict == SHORT_STOP_ONLY:
            return (
                f"🔇 [hardware AEC] Короткая фраза {duration_s:.2f}с {text or ''!r} во время TTS и не стоп с вейком — "
                "молчу (эхо или щелчок)"
            )
        return (
            f"🔇 [issue 989/#4072] Эхо-окно: фраза {duration_s:.2f}с {text or ''!r} звучала только после речи "
            "робота и не стоп с вейком — молчу (эхо собственного голоса)"
        )

    def echo(
        self, text: Optional[str], wake_words: Sequence[str], duration_s: float
    ) -> Optional[Echo]:
        """Итог по распознанной фразе: эхо ли она собственной речи робота; ``None`` — идёт дальше, как раньше.

        * не прошла :meth:`keeps` (голос только в эхо-окне, #4072, или короткий обрывок при речи, ревью #4122) — эхо;
        * ясный стоп с вейком (:func:`stop_passes`) идёт всегда — «стоп важнее»;
        * слова фразы в основном повторяют реплику робота, звучавшую рядом с ней (:func:`echo_of`, #4145) — эхо;
          фраза человека, не совпадающая со словами робота, идёт как раньше (бэклог, перебивание).

        Промежуточным результатам STT хватает :meth:`keeps`: по partial с вейком диалог рвёт речь, только если это
        ясный стоп, а он проходит и здесь.
        """
        text = text or ""
        if not self.keeps(text, wake_words):
            return Echo(self.echo_note(text, duration_s))
        if stop_passes(text, wake_words):
            return None
        return echo_of(text, self.spoken)


def _no_echo_sink(utterance_id: str, text: str) -> None:
    """Эхо-окно без ROS (тесты): вердикт «эхо» никому не публикуется."""


class EchoWindow:
    """Речь робота и эхо-окно после неё для одного микрофона; что звучало в окне, решает :meth:`admit`.

    Фронты речи — ``SpeechEvent`` через журнал речи (:class:`SpeechJournal`: реплика любого агента), фронты голоса —
    ``/audio/vad``. Всплеск VAD отмечает место на подъёме и на спаде, фронт речи робота посреди всплеска — тоже.
    Фраза доезжает после ``speech_continuation`` тишины, поэтому её всплески к доставке закрыты; открытый
    всплеск — уже следующая фраза, он остаётся ей. ``publish_echo(utterance_id, text)`` — фраза признана эхом
    (:data:`ECHO_TOPIC` у :func:`watch`).
    """

    def __init__(
        self,
        grace_s: float,
        aec_mode: str = "hardware",
        clock: Callable[[], float] = time.monotonic,
        publish_echo: Callable[[str, str], None] = _no_echo_sink,
        plain: Callable[[str], str] = str,
    ) -> None:
        self.grace_s = float(grace_s)
        self.aec_mode = aec_mode
        self.speaking = False
        self.ended_at = float("-inf")
        self.journal = SpeechJournal(self.grace_s, clock, plain)
        self.publish_echo = publish_echo
        self._clock = clock
        self._open: Optional[Set[str]] = None
        self._closed: Set[str] = set()
        self._last: Tuple[float, FrozenSet[str]] = (float("-inf"), frozenset())

    def where(self) -> str:
        """Где мы сейчас: говорит робот, эхо-окно после его речи или тишина."""
        if self.speaking:
            return SPEECH
        return GRACE if self._clock() - self.ended_at < self.grace_s else FREE

    def on_speech(self, speaking: bool) -> None:
        """Фронт речи робота: конец открывает эхо-окно; всплеск голоса, идущий через фронт, отмечает новое место."""
        if not speaking:
            self.ended_at = self._clock()
        self.speaking = bool(speaking)
        if self._open is not None:
            self._open.add(self.where())

    def on_vad(self, active: bool) -> None:
        """Фронт ``/audio/vad``: подъём открывает всплеск, спад закрывает его (с местом конца)."""
        if active and self._open is None:
            self._open = {self.where()}
        elif not active and self._open is not None:
            self._closed |= self._open | {self.where()}
            self._open = None

    def heard(self) -> FrozenSet[str]:
        """Где звучал голос доставленной фразы (и забыть его). Фронтов VAD не было — кусок той же фразы
        (:data:`CHUNK_GAP_S`) или место в момент доставки (поведение до #4072)."""
        now = self._clock()
        if self._closed:
            places = frozenset(self._closed)
        elif now - self._last[0] < CHUNK_GAP_S:
            places = self._last[1]
        else:
            places = frozenset({self.where()})
        self._closed = set()
        self._last = (now, places)
        return places

    def admit(self, duration_s: float) -> Admission:
        """Решение по доставленной фразе ReSpeaker (длина ``duration_s``) — до распознавания. Голос звучал рядом с
        речью робота (не только в тишине) — к решению приложены реплики робота из журнала (#4145).
        """
        heard = self.heard()
        admission = decide(
            heard,
            speaking=self.speaking,
            duration_s=duration_s,
            aec_mode=self.aec_mode,
        )
        if heard - {FREE}:
            return replace(admission, spoken=self.journal.sounded(duration_s))
        return admission


#: Фраза шлема (ptt/wake): эхо-окно ReSpeaker не про неё.
OPEN = Admission(PASS)


def decide(
    heard: AbstractSet[str], *, speaking: bool, duration_s: float, aec_mode: str
) -> Admission:
    """Вердикт по местам голоса фразы ``heard`` и речи робота сейчас (``speaking``)."""
    software = aec_mode == "software"
    if software and speaking:
        return Admission(
            DROP_SPEAKING,
            f"🔇 [software AEC] Игнор фразы {duration_s:.2f}с: робот говорит",
        )
    if heard and set(heard) <= {GRACE}:
        return Admission(
            STOP_ONLY,
            f"🔇 [issue 989/#4072] Фраза {duration_s:.2f}с звучала только в эхо-окне после речи робота — "
            "распознаю, пропущу только стоп с вейком",
        )
    if speaking and duration_s < SHORT_DURING_SPEECH_S:
        return Admission(
            SHORT_STOP_ONLY,
            f"🔇 [hardware AEC] Короткая фраза {duration_s:.2f}с при звучащем TTS — распознаю, пропущу только стоп "
            "с вейком",
        )
    if speaking:
        return Admission(
            PASS,
            f"🎤 [hardware AEC] Фраза {duration_s:.2f}с во время TTS — обрабатываем (возможно прерывание)",
        )
    return Admission(PASS)


def _dialog_msgs() -> Any:
    """Сообщения диалога (``rob_box_dialog_msgs``) — лениво, как у ``speech_client``: модуль живёт и без них."""
    from rob_box_dialog_msgs import msg

    return msg


def watch(
    node: Any,
    *,
    grace_s: float,
    aec_mode: str,
    vad_type: Any,
    clock: Callable[[], float] = time.monotonic,
) -> EchoWindow:
    """Подписать узел на речь робота (``SpeechRequest`` — текст, ``SpeechEvent`` — звучание) и ``/audio/vad``; эхо —
    в :data:`ECHO_TOPIC`. Вернуть эхо-окно узла.

    ``vad_type`` — ``std_msgs.msg.Bool`` (модуль ROS-сообщений не тянет); текст реплики без SSML — тот же
    ``speech_owner.plain_text``, что у владельца речи (импорт здесь: модуль тянет ``std_msgs``).
    """
    from rob_box_voice.core.speech_owner import plain_text

    msgs = _dialog_msgs()
    echo_pub = node.create_publisher(msgs.Utterance, ECHO_TOPIC, _QOS_DEPTH)

    def publish_echo(utterance_id: str, text: str) -> None:
        msg = msgs.Utterance()
        msg.utterance_id = utterance_id or ""
        msg.text = text or ""
        msg.source = msgs.Utterance.SOURCE_RESPEAKER
        echo_pub.publish(msg)

    window = EchoWindow(grace_s, aec_mode, clock, publish_echo, plain_text)

    def on_speech_event(event: Any) -> None:
        speaking = window.journal.on_event(event)
        if speaking is None:
            return
        window.on_speech(speaking)
        if aec_mode == "software":
            node.get_logger().info(_SOFTWARE_EDGE_LOG[speaking])
        else:
            node.get_logger().debug(
                _HARDWARE_EDGE_LOG[speaking].format(grace_s=window.grace_s)
            )

    node.create_subscription(
        msgs.SpeechEvent, speech_client.SPEECH_EVENT_TOPIC, on_speech_event, _QOS_DEPTH
    )
    node.create_subscription(
        msgs.SpeechRequest,
        speech_client.SPEECH_REQUEST_TOPIC,
        window.journal.on_request,
        _QOS_DEPTH,
    )
    node.create_subscription(
        vad_type, VAD_TOPIC, lambda msg: window.on_vad(bool(msg.data)), _QOS_DEPTH
    )
    return window


# --- биометрия слушает то же решение (#4145) -------------------------------------------------------------------------


def _call(run: Callable[[], None]) -> None:
    run()


class EchoVerdicts:
    """Биометрия слушает решение ``stt_node`` об эхе (#4145): фраза, звучавшая рядом с речью робота, меняет профили
    (рост галереи, профиль без имени) и даёт результат только после вердикта по ней; эхо не меняет ничего.

    Сравнения здесь нет — решение одно, у ``stt_node`` (:meth:`Admission.echo`); эмбеддинг готов раньше текста,
    поэтому биометрия по такой фразе ждёт. Вердикт — ``Utterance`` фразы в :data:`UTTERANCE_TOPIC` (человек: фраза
    ушла в диалог) или в :data:`ECHO_TOPIC` (эхо); не пришёл за :data:`VERDICT_TIMEOUT_S` (текста нет, фраза
    отброшена до распознавания) — как раньше. Фраза в тишине (журнал речи пуст рядом с ней) не ждёт — как раньше.

    ``submit`` — исполнитель биометрии (однопоточный пул ``speaker_id_node``): решение, пришедшее из ROS-колбэка,
    исполняется там же, где считались эмбеддинги."""

    def __init__(
        self,
        grace_s: float,
        *,
        submit: Callable[[Callable[[], None]], Any],
        log: Any,
        clock: Callable[[], float] = time.monotonic,
        timeout_s: float = VERDICT_TIMEOUT_S,
    ) -> None:
        self.journal = SpeechJournal(grace_s, clock)
        self.timeout_s = float(timeout_s)
        self._submit, self._log, self._clock = submit, log, clock
        self._lock = threading.Lock()
        self._waiting: Dict[str, float] = {}  # фраза рядом с речью -> срок вердикта
        self._verdicts: Dict[str, bool] = {}  # вердикт пришёл раньше эмбеддинга: эхо ли
        self._held: Dict[str, Callable[[], None]] = (
            {}
        )  # эмбеддинг готов, вердикта ещё нет

    def heard(self, utterance_id: str, seconds: float) -> None:
        """Фраза доехала (PCM длиной ``seconds``): звучала рядом с речью робота — её профили ждут вердикта."""
        if utterance_id and self.journal.sounded(seconds + VERDICT_SLACK_S):
            with self._lock:
                self._waiting[utterance_id] = self._clock() + self.timeout_s

    def settle(self, utterance_id: str, run: Callable[[], None]) -> None:
        """Биометрия фразы готова: ``run`` (рост галереи, профиль, результат) — сейчас, после вердикта или никогда."""
        with self._lock:
            echo = self._verdicts.pop(utterance_id, None)
            if echo is None and utterance_id in self._waiting:
                self._held[utterance_id] = run
                return
            self._waiting.pop(utterance_id, None)
        self._finish(utterance_id, bool(echo), _call, run)

    def on_verdict(self, utterance_id: str, echo: bool) -> None:
        """Вердикт ``stt_node`` по фразе: человек — профили как раньше, эхо — не трогать."""
        with self._lock:
            if utterance_id not in self._waiting:
                self._unexpected(utterance_id, echo)
                return
            run = self._held.pop(utterance_id, None)
            if run is None:  # эмбеддинг ещё считается: вердикт его дождётся
                _remember(self._verdicts, utterance_id, echo, VERDICTS_KEPT)
                return
            del self._waiting[utterance_id]
        self._finish(utterance_id, echo, self._submit, run)

    def expire(self) -> None:
        """Вердикт не пришёл в срок (текста нет, фраза отброшена до распознавания): профили — как раньше."""
        now = self._clock()
        with self._lock:
            due = [
                u
                for u, deadline in self._waiting.items()
                if deadline <= now and u not in self._verdicts
            ]
            runs = [(u, self._held.pop(u, None)) for u in due]
            for u in due:
                del self._waiting[u]
        for utterance_id, run in runs:
            if run is not None:
                self._log.info(
                    f"🔁 [echo] биометрия: вердикта по utterance_id={utterance_id} нет за {self.timeout_s:.0f}с — "
                    "профиль как раньше"
                )
                self._submit(run)

    def _finish(
        self,
        utterance_id: str,
        echo: bool,
        call: Callable[[Callable[[], None]], Any],
        run: Callable,
    ) -> None:
        if echo:
            self._log.info(
                f"🔁 [echo] биометрия: фраза utterance_id={utterance_id} — эхо речи робота, профиль не завожу и не "
                "обновляю"
            )
            return
        call(run)

    def _unexpected(self, utterance_id: str, echo: bool) -> None:
        """Вердикт по фразе, которой биометрия не ждала: человек — обычное дело (фраза в тишине); эхо — окна речи у
        нод разошлись, профиль мог измениться — громко."""
        if echo and utterance_id:
            self._log.warning(
                f"⚠️ [echo] эхо utterance_id={utterance_id}, а биометрия его не ждала (окно речи разошлось) — "
                "профиль мог измениться"
            )


def listen(
    node: Any,
    *,
    submit: Callable[[Callable[[], None]], Any],
    grace_s: float = kn.IDENTITY_MIC_GRACE_S,
    clock: Callable[[], float] = time.monotonic,
) -> EchoVerdicts:
    """Подписать биометрию на речь робота (``SpeechEvent``) и вердикты ``stt_node``: фраза человека
    (:data:`UTTERANCE_TOPIC`) и эхо (:data:`ECHO_TOPIC`); просроченное ожидание снимает таймер.
    """
    msgs = _dialog_msgs()
    verdicts = EchoVerdicts(grace_s, submit=submit, log=node.get_logger(), clock=clock)
    node.create_subscription(
        msgs.SpeechEvent,
        speech_client.SPEECH_EVENT_TOPIC,
        verdicts.journal.on_event,
        _QOS_DEPTH,
    )
    node.create_subscription(
        msgs.Utterance,
        UTTERANCE_TOPIC,
        lambda m: verdicts.on_verdict(m.utterance_id or "", False),
        _QOS_DEPTH,
    )
    node.create_subscription(
        msgs.Utterance,
        ECHO_TOPIC,
        lambda m: verdicts.on_verdict(m.utterance_id or "", True),
        _QOS_DEPTH,
    )
    node.create_timer(VERDICT_TICK_S, verdicts.expire)
    return verdicts


def heard(node: Any, utterance_id: str, seconds: float) -> None:
    """Биометрия получила фразу: отметить, ждёт ли она вердикта (нода без слушателя — из тестов — не ждёт)."""
    verdicts = getattr(node, "_echo_verdicts", None)
    if verdicts is not None:
        verdicts.heard(utterance_id, seconds)


def settle(node: Any, utterance_id: str, run: Callable[[], None]) -> None:
    """Биометрия фразы готова: :meth:`EchoVerdicts.settle`; нода без слушателя (из тестов) — сразу, как раньше."""
    verdicts = getattr(node, "_echo_verdicts", None)
    if verdicts is None:
        run()
        return
    verdicts.settle(utterance_id, run)
