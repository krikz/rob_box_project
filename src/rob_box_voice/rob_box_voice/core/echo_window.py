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
``speech_id``, когда звучала — из ``SpeechEvent``) даёт фразе, чей голос звучал только рядом с речью (во время неё
или в эхо-окне), реплики робота; после распознавания :meth:`Admission.echo` сверяет с ними её слова (:func:`echo_of`)
— эхо не идёт ни в бэклог, ни к модели. Голос, дошедший до тишины после эхо-окна, и вейк, которого робот не говорил,
— человек: такие фразы эхом по словам не считаются. Решение одно, его слушает и биометрия: эхо уходит в
:data:`ECHO_TOPIC`; ``speaker_id_node`` по фразе рядом с речью сразу публикует опознание (профилей оно не меняет), а
изменения профилей исполняет только по вердикту «человек» (:class:`EchoVerdicts`) — эхо профиль не заводит и не
растит.

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
#: Изменения профилей по фразе рядом с речью робота ждут вердикта ``stt_node`` не дольше (распознавание с повтором и
#: запасным Vosk). Не дождались — фраза не стала репликой человека (пустой или короткий текст STT, распознавание
#: зависло): профили не меняются. Результат опознания такой фразы уходит сразу, его срок не держит.
VERDICT_TIMEOUT_S = 20.0
#: Окно «рядом с речью» у биометрии шире, чем у ``stt_node``: то же PCM ноды получают с разницей в миллисекунды.
VERDICT_SLACK_S = 1.0
#: Сколько фраз рядом с речью помнит биометрия: вердикт мог прийти раньше эмбеддинга (холодный энкодер считает до
#: ~50 с, #2885), а эмбеддинг — позже срока; старейшие сверх этого забываются.
VERDICTS_KEPT = 64
#: Как часто биометрия снимает просроченное ожидание вердикта.
VERDICT_TICK_S = 1.0
#: Итог фразы рядом с речью робота у биометрии: реплика человека (профили меняются как раньше), эхо, вердикта нет.
HUMAN, ECHO, NO_VERDICT = "human", "echo", "no_verdict"

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


def echo_of(
    text: str, spoken: Sequence[Spoken], wake_words: Sequence[str] = ()
) -> Optional[Echo]:
    """Реплика робота, чьи слова фраза в основном повторяет (#4145); ``None`` — не эхо, фраза идёт как раньше.

    Эхо — не меньше :data:`ECHO_MIN_WORDS` слов фразы идут в реплике по порядку и это не меньше
    :data:`ECHO_WORD_SHARE` её слов; нормализация слов одна — :mod:`rob_box_dialog.words`. Вейк во фразе, которого
    нет в реплике робота, — голос человека, не эхо: с такой репликой фраза не сверяется (ревью #4161: «робот а ты
    что» во время длинной речи совпадала служебными словами 3 из 4).
    """
    total = len(name_tokens(text))
    woken = bool(wake_words) and has_wake_word(text, wake_words)
    for said in spoken:
        if woken and not has_wake_word(said.text, wake_words):
            continue
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
          фраза человека, не совпадающая со словами робота, и фраза с вейком, которого робот не говорил, идут как
          раньше (бэклог, перебивание).

        Промежуточным результатам STT хватает :meth:`keeps`: по partial с вейком диалог рвёт речь, только если это
        ясный стоп, а он проходит и здесь.
        """
        text = text or ""
        if not self.keeps(text, wake_words):
            return Echo(self.echo_note(text, duration_s))
        if stop_passes(text, wake_words):
            return None
        return echo_of(text, self.spoken, wake_words)


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
        """Решение по доставленной фразе ReSpeaker (длина ``duration_s``) — до распознавания. Голос звучал только рядом
        с речью робота (во время неё или в эхо-окне) — к решению приложены реплики робота из журнала (#4145). Голос
        дошёл до тишины после эхо-окна — в фразе звучал человек (эхо собственного голоса за эхо-окном не звучит),
        слова с речью робота не сверяются: ответ «сказку про репку» на «Рассказать сказку про колобка или про
        репку?», начатый в эхо-окне, — реплика человека (ревью #4161).
        """
        heard = self.heard()
        admission = decide(
            heard,
            speaking=self.speaking,
            duration_s=duration_s,
            aec_mode=self.aec_mode,
        )
        if FREE not in heard:
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

    def on_speech(speaking: bool) -> None:
        window.on_speech(speaking)
        if aec_mode == "software":
            node.get_logger().info(_SOFTWARE_EDGE_LOG[speaking])
        else:
            node.get_logger().debug(
                _HARDWARE_EDGE_LOG[speaking].format(grace_s=window.grace_s)
            )

    speech_client.watch_speaking(node, on_speech, watch=window.journal)
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


def _call(run: Callable[[], Any]) -> None:
    run()


def _ignore_front(_speaking: bool) -> None:
    """Биометрии нужен только журнал речи, фронт «звучит» ей не нужен."""


@dataclass
class _Held:
    """Фраза рядом с речью робота у биометрии: срок вердикта, вердикт (``None`` — ещё нет) и изменения профилей
    (``None`` — эмбеддинг ещё считается)."""

    deadline: float
    verdict: Optional[str] = None
    changes: Optional[Callable[[], Any]] = None


class EchoVerdicts:
    """Биометрия слушает решение ``stt_node`` об эхе (#4145). Результат опознания фразы рядом с речью робота уходит
    сразу — он профилей не меняет и доходит до диалога раньше её ``Utterance``, как у любой фразы (ревью #4161:
    адресация и собеседник хода берут голос фразы в момент её ``Utterance``). Ждут вердикта только изменения профилей
    (рост галереи, профиль без имени и его результат ``enroll``):

    * ``Utterance`` фразы в :data:`UTTERANCE_TOPIC` — реплика человека: профили меняются как раньше;
    * ``Utterance`` в :data:`ECHO_TOPIC` — эхо собственной речи робота: не меняется ничего;
    * вердикта нет за :data:`VERDICT_TIMEOUT_S` (пустой или короткий текст STT, распознавание зависло) — фраза не
      стала репликой человека: профили тоже не меняются, позднего результата в случайный момент нет.

    Фраза в тишине (журнал речи пуст рядом с ней) ничего не ждёт — как раньше. Сравнения здесь нет — решение одно, у
    ``stt_node`` (:meth:`Admission.echo`). ``submit`` — исполнитель биометрии (однопоточный пул ``speaker_id_node``):
    изменения по вердикту из ROS-колбэка исполняются там же, где считались эмбеддинги.
    """

    def __init__(
        self,
        grace_s: float,
        *,
        submit: Callable[[Callable[[], Any]], Any],
        log: Any,
        clock: Callable[[], float] = time.monotonic,
        timeout_s: float = VERDICT_TIMEOUT_S,
    ) -> None:
        self.journal = SpeechJournal(grace_s, clock)
        self.timeout_s = float(timeout_s)
        self._submit, self._log, self._clock = submit, log, clock
        self._lock = threading.Lock()
        self._held: Dict[str, _Held] = {}

    def heard(self, utterance_id: str, seconds: float) -> None:
        """Фраза доехала (PCM длиной ``seconds``): звучала рядом с речью робота — её изменения профилей ждут вердикта."""
        if utterance_id and self.journal.sounded(seconds + VERDICT_SLACK_S):
            with self._lock:
                held = _Held(self._clock() + self.timeout_s)
                _remember(self._held, utterance_id, held, VERDICTS_KEPT)

    def waits(self, utterance_id: str) -> bool:
        """Изменения профилей по фразе ждут вердикта ``stt_node`` (фраза звучала рядом с речью робота)."""
        with self._lock:
            return utterance_id in self._held

    def settle(self, utterance_id: str, changes: Callable[[], Any]) -> None:
        """Эмбеддинг фразы готов: ``changes`` — сейчас (фраза в тишине или вердикт «человек» уже пришёл), после
        вердикта или никогда (эхо, вердикта нет в срок)."""
        with self._lock:
            held = self._held.get(utterance_id)
            if held is not None and held.verdict is None:
                held.changes = changes
                return
            self._held.pop(utterance_id, None)
        self._finish(
            utterance_id, HUMAN if held is None else held.verdict, changes, _call
        )

    def on_verdict(self, utterance_id: str, echo: bool) -> None:
        """Вердикт ``stt_node`` по фразе: человек — профили как раньше, эхо — не трогать."""
        verdict = ECHO if echo else HUMAN
        with self._lock:
            held = self._held.get(utterance_id)
            pending = held is not None and held.verdict is None
            if pending and held.changes is None:
                # эмбеддинг ещё считается: вердикт его дождётся
                held.verdict = verdict
                return
            if pending:
                del self._held[utterance_id]
        if not pending:
            self._unexpected(utterance_id, echo, late=held is not None)
            return
        self._finish(utterance_id, verdict, held.changes, self._submit)

    def expire(self) -> None:
        """Срок вердикта вышел: фраза не стала репликой человека — изменения профилей не исполняются. Итог помнится до
        вытеснения: его найдут и эмбеддинг, досчитанный позже срока, и вердикт, опоздавший к нему.
        """
        now = self._clock()
        with self._lock:
            due = [
                (u, h, h.changes)
                for u, h in self._held.items()
                if h.verdict is None and h.deadline <= now
            ]
            for _utterance_id, held, _changes in due:
                held.verdict, held.changes = NO_VERDICT, None
        for utterance_id, _held, changes in due:
            if changes is not None:
                self._finish(utterance_id, NO_VERDICT, changes, _call)

    def forget(self, utterance_id: str) -> None:
        """Фраза до изменений профилей не дошла (эмбеддинга нет, фраза ушла в регистрацию по имени): ждать нечего."""
        with self._lock:
            self._held.pop(utterance_id, None)

    def _finish(
        self,
        utterance_id: str,
        verdict: Optional[str],
        changes: Callable[[], Any],
        call: Callable[[Callable[[], Any]], Any],
    ) -> None:
        if verdict == HUMAN:
            call(changes)
        elif verdict == ECHO:
            self._log.info(
                f"🔁 [echo] биометрия: фраза utterance_id={utterance_id} — эхо речи робота, профиль не завожу и не "
                "обновляю"
            )
        else:
            self._log.info(
                f"🔁 [echo] биометрия: вердикта по utterance_id={utterance_id} нет за {self.timeout_s:.0f}с — фраза "
                "рядом с речью робота не стала репликой человека, профиль не завожу и не обновляю"
            )

    def _unexpected(self, utterance_id: str, echo: bool, *, late: bool) -> None:
        """Вердикт по фразе, которой биометрия не ждала. Человек — обычное дело (фраза в тишине); человек после срока —
        профиль по ней не менялся; эхо — окна речи у нод разошлись, профиль мог измениться — громко.
        """
        if late and not echo:
            self._log.info(
                f"🔁 [echo] биометрия: вердикт «человек» по utterance_id={utterance_id} пришёл после срока "
                f"{self.timeout_s:.0f}с — профиль по фразе не менялся"
            )
        elif echo and utterance_id and not late:
            self._log.warning(
                f"⚠️ [echo] эхо utterance_id={utterance_id}, а биометрия его не ждала (окно речи разошлось) — "
                "профиль мог измениться"
            )


def listen(
    node: Any,
    *,
    submit: Callable[[Callable[[], Any]], Any],
    grace_s: float = kn.IDENTITY_MIC_GRACE_S,
    clock: Callable[[], float] = time.monotonic,
) -> EchoVerdicts:
    """Подписать биометрию на речь робота (``SpeechEvent`` через ``speech_client.watch_speaking``) и вердикты
    ``stt_node``: фраза человека (:data:`UTTERANCE_TOPIC`) и эхо (:data:`ECHO_TOPIC`); просроченное ожидание снимает
    таймер.
    """
    msgs = _dialog_msgs()
    verdicts = EchoVerdicts(grace_s, submit=submit, log=node.get_logger(), clock=clock)
    speech_client.watch_speaking(node, _ignore_front, watch=verdicts.journal)
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


def _verdicts_of(node: Any) -> Optional[EchoVerdicts]:
    """Слушатель вердиктов биометрии; у ноды из тестов (``object.__new__``) его нет — всё как раньше."""
    return getattr(node, "_echo_verdicts", None)


def heard(node: Any, utterance_id: str, seconds: float) -> None:
    """Биометрия получила фразу: отметить, ждут ли её изменения профилей вердикта."""
    verdicts = _verdicts_of(node)
    if verdicts is not None:
        verdicts.heard(utterance_id, seconds)


def waits(node: Any, utterance_id: str) -> bool:
    """Изменения профилей по фразе ждут вердикта :meth:`EchoVerdicts.waits`; ноде без слушателя ждать нечего."""
    verdicts = _verdicts_of(node)
    return verdicts is not None and verdicts.waits(utterance_id)


def settle(node: Any, utterance_id: str, changes: Callable[[], Any]) -> None:
    """Изменения профилей по фразе: :meth:`EchoVerdicts.settle`; нода без слушателя — сразу, как раньше."""
    verdicts = _verdicts_of(node)
    if verdicts is None:
        changes()
        return
    verdicts.settle(utterance_id, changes)


def forget(node: Any, utterance_id: Optional[str]) -> None:
    """Фраза до изменений профилей не дошла: :meth:`EchoVerdicts.forget` (``None`` — у фразы нет id, ждать нечего)."""
    verdicts = _verdicts_of(node)
    if verdicts is not None and utterance_id:
        verdicts.forget(utterance_id)
