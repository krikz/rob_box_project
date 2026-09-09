"""Единый сергментатор фраз для всех голосовых путей rob_box (issue #2199).

Зачем
-----
Раньше «фраза кончилась» решалось в **четырёх** местах четырьмя способами
(см. таблицу в issue #2199):

  1. ``webxr_client/src/input/voice_capture.ts`` — RMS > 200, hangover 200 мс,
     только wake-канал. PTT-канал VAD'ом **не** гейтится.
  2. ``rob_box_quest/quest_node.py:118-119,128-138`` — пик int16 < 500,
     пауза ≥ 300 мс (robot_voice на PTT-канале).
  3. ``rob_box_quest/core/wake_segmenter.py`` — gap в потоке кадров ≥ 400 мс,
     min 0.25 с, max 15 с (wake-канал на сервере, per-frame VAD делает клиент).
  4. ``rob_box_voice/stt_node.py:572`` — строка докстринга: «распознаём
     VAD-сегмент», отдельного правила нет (вход уже сегментирован).

Этот модуль заменяет правила 1-3 одним классом :class:`PhraseSegmenter` с
общим конфигом :class:`SpeechSegmentationConfig`. Правило 4 не трогаем —
это контракт, не алгоритм.

Где что гейтится (а не «всё симметрично»)
-----------------------------------------
Три пути голоса гейтятся **по-разному** — это видно из профилей YAML
(``rob_box_core/config/speech_segmentation.yaml``) и важно помнить при
правке любого порога:

  ====================  =====================  ============================
  Путь                   Где гейтится клиент    Что делает сервер
  ====================  =====================  ============================
  wake-канал             RMS-VAD + hangover     Только накапливает и
   (stream_id=2,         200 мс в TS            закрывает фразу по паузе в
   шлем → /audio/        (``voice_capture.ts``,  потоке кадров
   quest_wake)           ``statistic="none"``    (``gap_timeout_s=0.4``).
                          на сервере).
  -------------------  ---------------------  ----------------------------
  PTT robot_voice        НИЧЕГО.                per-frame peak-VAD
   (stream_id=1,         Клиент шлёт всё         (500 int16), фраза
   шлем → /audio/        подряд; голосова         закрывается через
   quest_in)             канал отключён          ``gap_timeout_s=0.3`` с.
                          в TS, чтобы
                          сервер сам
                          решил.
  -------------------  ---------------------  ----------------------------
  PTT radio              НИЧЕГО.                Не сегментируется вообще
   (stream_id=1,         Клиент шлёт всё;        (echo-canceller держит
   шлем → /avatar/       сегментация              тайминг). Профиль
   voice_in)             считается              ``radio`` оставлен для
                          ответственностью      будущих расширений.
                          echo-canceller'а.
  ====================  =====================  ============================

  Следствие: ``gap_timeout_s`` сервера для wake-канала должен быть
  **≥ клиентского hangover** + запас на сетевой джиттер; иначе сервер
  закроет фразу раньше, чем клиент дослал её «хвост». Сейчас
  ``client_vad.hangover_ms=200`` + ``wake.gap_timeout_s=400`` — вместе
  600 мс запаса.

Контракт
--------
``PhraseSegmenter`` принимает int16 LE PCM-кадры одинаковой длины
(типично 20 мс @ 16 кГц = 320 семплов = 640 байт) и решает, когда
накопленный буфер превращается в фразу:

* ``add_frame(pcm, now)`` — очередной кадр. Возвращает готовую фразу
  (``bytes``) или ``None``.
* ``tick(now)`` — таймерный тик. Закрывает фразу, если с последнего
  кадра прошло больше ``gap_timeout_s``.
* ``reset()`` — выбросить недособранную фразу (разрыв сессии).

Per-frame VAD настраивается через ``statistic``:

* ``"rms"`` — RMS int16. Подходит для каналов, где клиент **не**
  фильтрует тишину (PTT robot_voice).
* ``"peak"`` — пик |сэмпла|. Совместимо со старым ``_chunk_is_silent``
  (500 int16 единиц).
* ``"none"`` — клиент уже отфильтровал тишину, каждый кадр считается
  речью. Используется на сервере для wake-канала: на стороне TS
  wake-кадры прошли через VAD до отправки.

Чистый Python, без rclpy/aiohttp — тестируется на dev-машине без ROS
(см. ``test/test_speech_segmentation.py``).

Зачем именно один класс, а не три
---------------------------------
Три вызывающих (рация, пайплайн грипа, wake) описывают одни и те же
инварианты: «конец фразы — пауза дольше N мс; фраза короче M — блип;
фраза длиннее L — режем по потолку». Эти инварианты должны двигаться
**синхронно**: подкручивая hangover на клиенте, мы не хотим отдельно
перевычислять gap на сервере. Поэтому параметры едут одним YAML
(``rob_box_core/config/speech_segmentation.yaml``), а клиент TS берёт
RMS-порог и hangover из зеркала
``webxr_client/src/input/voice_segmentation_constants.ts`` (синхронность
проверяется ``rob_box_core.tools.check_ts_sync``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Literal, Optional


# int16 mono PCM 16 кГц — общий формат для wake-канала и PTT-канала
# webxr_client. Контракт WebXR-Voice-Capture: VOICE_SAMPLE_RATE=16000,
# VOICE_CHUNK_SAMPLES=320.
SAMPLE_RATE_HZ: int = 16000
BYTES_PER_SAMPLE: int = 2  # int16
BYTES_PER_S: int = SAMPLE_RATE_HZ * BYTES_PER_SAMPLE  # 32000 Б/с

Statistic = Literal["rms", "peak", "none"]


@dataclass(frozen=True)
class SpeechSegmentationConfig:
    """Единый конфиг сегментации фраз (issue #2199).

    Поля отражают ровно те четыре параметра, что раньше были магическими
    числами в ``voice_capture.ts:174-180``, ``quest_node.py:118-119`` и
    ``wake_segmenter.py:53,59,65``. Меняя их здесь, мы синхронно
    подкручиваем все три пути.

    Attributes
    ----------
    statistic:
        Какая статистика int16 PCM-кадра определяет «речь». ``"rms"``
        — среднеквадратичная, ``"peak"`` — пиковый |сэмпла|,
        ``"none"`` — клиент уже отфильтровал, кадр всегда считается
        речью (wake-канал на сервере).
    speech_threshold:
        Порог статистики в единицах int16 (RMS 0..32767, |peak| 0..32767).
        Игнорируется при ``statistic="none"``.
    gap_timeout_s:
        Пауза в потоке кадров, после которой буфер закрывается как фраза.
        Для wake — должно быть ≥ клиентского hangover + запас на джиттер.
        Для robot_voice — клиентского VAD нет, поэтому порог — это и есть
        «конец фразы».
    min_phrase_s:
        Фраза короче этого — блип VAD, в STT не идёт. Защита от «ТАРС»
        из шума (issue #2135).
    max_phrase_s:
        Потолок буфера. Защита от «залипшего» клиентского VAD, который
        лил бы кадры бесконечно.
    """

    statistic: Statistic = "none"
    speech_threshold: int = 0
    gap_timeout_s: float = 0.4
    min_phrase_s: float = 0.25
    max_phrase_s: float = 15.0

    @property
    def gap_timeout_bytes(self) -> int:
        """Граница в байтах (для метрик / тестов)."""
        return int(self.gap_timeout_s * BYTES_PER_S)

    @property
    def min_phrase_bytes(self) -> int:
        """Граница в байтах."""
        return int(self.min_phrase_s * BYTES_PER_S)

    @property
    def max_phrase_bytes(self) -> int:
        """Граница в байтах."""
        return int(self.max_phrase_s * BYTES_PER_S)

    @classmethod
    def from_mapping(cls, data: dict) -> "SpeechSegmentationConfig":
        """Сконструировать из произвольного dict (YAML/JSON)."""
        statistic = data.get("statistic", "none")
        if statistic not in ("rms", "peak", "none"):
            raise ValueError(
                f"unknown statistic={statistic!r}; expected rms|peak|none"
            )
        return cls(
            statistic=statistic,  # type: ignore[arg-type]
            speech_threshold=int(data.get("speech_threshold", 0)),
            gap_timeout_s=float(data.get("gap_timeout_s", 0.4)),
            min_phrase_s=float(data.get("min_phrase_s", 0.25)),
            max_phrase_s=float(data.get("max_phrase_s", 15.0)),
        )


# Дефолты «wake-канал, клиент уже отфильтровал» — эквивалент старых
# WAKE_PHRASE_GAP_TIMEOUT_S / WAKE_PHRASE_MIN_S / WAKE_PHRASE_MAX_S.
DEFAULT_WAKE_CONFIG = SpeechSegmentationConfig(
    statistic="none",
    speech_threshold=0,
    gap_timeout_s=0.4,
    min_phrase_s=0.25,
    max_phrase_s=15.0,
)

# Дефолты «PTT robot_voice, клиент не фильтрует» — эквивалент старых
# VOICE_SILENCE_THRESHOLD=500 (peak) и VOICE_SILENCE_TIMEOUT_MS=300.
DEFAULT_ROBOT_VOICE_CONFIG = SpeechSegmentationConfig(
    statistic="peak",
    speech_threshold=500,
    gap_timeout_s=0.3,
    min_phrase_s=0.25,
    max_phrase_s=15.0,
)


def _frame_is_speech(pcm: bytes, statistic: Statistic, threshold: int) -> bool:
    """Решает «кадр — речь или тишина» по заявленной статистике.

    Для ``statistic="none"`` — всегда True (клиент уже отфильтровал).
    """
    if statistic == "none":
        return True
    if len(pcm) < 2:
        return False
    if statistic == "peak":
        # Пиковый |сэмпл| int16 LE. Точно совместимо со старым
        # quest_node._chunk_is_silent(payload, threshold).
        for i in range(0, len(pcm) - 1, 2):
            s = pcm[i] | (pcm[i + 1] << 8)
            if s >= 0x8000:
                s -= 0x10000
            if s < 0:
                s = -s
            if s >= threshold:
                return True
        return False
    if statistic == "rms":
        # RMS int16 (формула как в voice_capture.ts:rmsInt16). 0..32767.
        n = len(pcm) // 2
        if n == 0:
            return False
        sum_sq = 0
        for i in range(0, n * 2, 2):
            s = pcm[i] | (pcm[i + 1] << 8)
            if s >= 0x8000:
                s -= 0x10000
            sum_sq += s * s
        rms = math.sqrt(sum_sq / n)
        return rms >= threshold
    raise ValueError(f"unsupported statistic={statistic!r}")


@dataclass
class PhraseSegmenter:
    """Кадры int16 PCM → фразы. Один буфер, метка времени, метрики.

    Не потокобезопасен: предполагается, что вызывающий сериализует
    доступ (rclpy таймер + aiohttp event-loop публикуют в одном мосте,
    GIL достаточно на уровне одного вызова).

    Буфер — список bytes, склейка один раз на фразу, а не конкатенация
    на каждый кадр.
    """

    config: SpeechSegmentationConfig = field(default_factory=lambda: DEFAULT_WAKE_CONFIG)
    _frames: List[bytes] = field(default_factory=list)
    _buffered_bytes: int = 0
    _last_speech_at: Optional[float] = None
    _silence_bytes_since_speech: int = 0
    dropped_short_phrases: int = 0
    truncated_phrases: int = 0

    @property
    def buffered_bytes(self) -> int:
        """Сколько байт лежит в незакрытой фразе (0 — буфер пуст)."""
        return self._buffered_bytes

    @property
    def buffered_seconds(self) -> float:
        """Сколько секунд аудио в незакрытой фразе (для логов/метрик)."""
        return self._buffered_bytes / BYTES_PER_S

    def add_frame(self, payload: bytes, now_monotonic: float) -> Optional[bytes]:
        """Принять кадр. Вернуть фразу, если она закрылась этим вызовом.

        Семантика «фраза закрылась»:

        1. Перед этим кадром была пауза ≥ ``gap_timeout_s`` — значит кадр
           открывает НОВУЮ фразу, а старую надо отдать (таймер мог не
           успеть, если пауза совпала с приходом следующей реплики).
        2. После последнего речевого кадра накопилось ≥ ``gap_timeout_s``
           тишины — буква «конец фразы» (старая логика
           ``_voice_silence_ms`` в ``quest_node.publish_voice_audio``).
        3. Буфер упёрся в ``max_phrase_bytes`` — режем принудительно.

        Кадры-тишины (VAD решил «не речь») НЕ попадают в буфер, но
        обновляют счётчик ``_silence_bytes_since_speech``. Это счётчик
        байтов PCM-тишины, пересчитывается в секунды через ``BYTES_PER_S``
        — эквивалентно старому «ms per 20ms-chunk».
        """
        phrase = self._close_if_gap(now_monotonic)
        is_speech = _frame_is_speech(
            payload, self.config.statistic, self.config.speech_threshold
        )
        if is_speech:
            self._frames.append(payload)
            self._buffered_bytes += len(payload)
            self._last_speech_at = now_monotonic
            self._silence_bytes_since_speech = 0
            if (
                phrase is None
                and self._buffered_bytes >= self.config.max_phrase_bytes
            ):
                self.truncated_phrases += 1
                phrase = self._close()
        else:
            # Тишина: НЕ добавляем в буфер, но инкрементим счётчик.
            # Когда счётчик перевалит за gap_timeout_s × BYTES_PER_S — фраза
            # закрывается. Проверяем ПОСЛЕ инкремента, чтобы фрейм,
            # достигший порога, был «тем самым».
            self._silence_bytes_since_speech += len(payload)
            if (
                phrase is None
                and self._frames
                and self._last_speech_at is not None
                and self._silence_bytes_since_speech
                >= self.config.gap_timeout_bytes
            ):
                phrase = self._close()
        return phrase

    def tick(self, now_monotonic: float) -> Optional[bytes]:
        """Таймерный тик: закрыть фразу, если поток кадров замолчал.

        Используется, когда кадры вообще перестали приходить
        (``add_frame`` не вызывается). Для каналов, где клиент не фильтрует
        тишину (robot_voice), обычно хватает внутреннего флаша в
        :meth:`add_frame`; здесь — резервный путь.
        """
        return self._close_if_gap(now_monotonic)

    def reset(self) -> int:
        """Выбросить недособранную фразу. Возвращает число потерянных байт.

        Вызывается при разрыве сессии: половина фразы прошлого оператора
        не должна приклеиться к речи следующего.
        """
        dropped = self._buffered_bytes
        self._frames = []
        self._buffered_bytes = 0
        self._last_speech_at = None
        self._silence_bytes_since_speech = 0
        return dropped

    def force_close(self) -> Optional[bytes]:
        """Закрыть буфер принудительно (без проверки gap/min).

        Используется на границах сессии, где пользователь явно завершил
        ввод (PTT release, voice mode change): содержимое буфера должно
        уйти в STT **как есть**, без требования «подожди ещё gap/min_phrase»
        — иначе первая короткая реплика теряется.

        В отличие от :meth:`tick`, не учитывает ``min_phrase_bytes``: тут
        короткий буфер — это явный «что было, то отдаём», а не блип VAD.

        Возвращает ``None``, если буфер пуст.
        """
        if not self._frames:
            self._buffered_bytes = 0
            self._last_speech_at = None
            self._silence_bytes_since_speech = 0
            return None
        # Склеиваем без min-проверки: пользователь явно сказал «хватит».
        data = b"".join(self._frames)
        self._frames = []
        self._buffered_bytes = 0
        self._last_speech_at = None
        self._silence_bytes_since_speech = 0
        return data

    def _close_if_gap(self, now_monotonic: float) -> Optional[bytes]:
        """Фраза закончена по time-monotonic-разрыву (wake-канал).

        Закрывает фразу, если с момента последнего РЕЧЕВОГО кадра прошло
        ≥ ``gap_timeout_s`` **без новых речевых кадров**. Используется,
        когда клиент уже отфильтровал тишину (wake-канал, ``statistic="none"``)
        и кадры-речи приходят с фиксированным периодом 20 мс; если оператор
        замолчал, кадры вообще перестают приходить, и :meth:`tick` дёргает
        эту проверку.
        """
        if not self._frames:
            return None
        if self._last_speech_at is None:
            return None
        if now_monotonic - self._last_speech_at < self.config.gap_timeout_s:
            return None
        return self._close()

    def _close(self) -> Optional[bytes]:
        """Склеить буфер в фразу и очистить состояние.

        None — если фраза короче ``min_phrase_bytes`` (блип VAD): буфер
        всё равно очищается, наружу ничего не идёт.
        """
        data = b"".join(self._frames)
        self._frames = []
        self._buffered_bytes = 0
        self._last_speech_at = None
        self._silence_bytes_since_speech = 0
        if len(data) < self.config.min_phrase_bytes:
            self.dropped_short_phrases += 1
            return None
        return data
