"""Сегментатор wake-потока шлема: кадры 20 мс → одна фраза (issue #2135).

Зачем это существует
--------------------
Клиент (`webxr_client/src/input/voice_capture.ts`) режет микрофон шлема на
кадры по 320 сэмплов (20 мс @ 16 кГц, 640 байт int16) и гейтит их
RMS-VAD'ом с hangover 200 мс. VAD решает **«слать или не слать»**, но не
**«где кончилась фраза»** — на проводе идёт ровный поток кадров, пока
оператор говорит, и пауза, когда молчит.

`stt_node.quest_wake_audio_callback` запускает ПОЛНЫЙ цикл распознавания
на КАЖДОЕ сообщение в `/audio/quest_wake`. Пока мост публиковал по одному
кадру на сообщение, распознавание шло на 640 байтах и всегда возвращало
пусто — вейк «ТАРС» из шлема не мог сработать никогда (live-лог робота
2026-09-08: `🎤 Получена фраза: 0.02с (640 bytes)` → `❌ ОТКЛОНЕНО
(пустое)` на каждом кадре, при том что ReSpeaker в том же логе отдавал
`4.32с (138112 bytes)` → `✅ ПРИНЯТО`).

У ReSpeaker роль сегментатора играет `audio_node` (VAD +
`speech_continuation`), у шлема эквивалента не было. Этот модуль —
недостающее звено: он накапливает кадры и закрывает фразу по **паузе в
потоке кадров** (клиентский hangover уже гарантирует, что пауза в кадрах
= конец речи, а не межслоговая тишина).

Контракт
--------
* :meth:`WakePhraseSegmenter.add_frame` — очередной кадр из WS. Вернёт
  готовую фразу, если этот кадр открыл новую (пауза перед ним) или если
  буфер упёрся в потолок.
* :meth:`WakePhraseSegmenter.tick` — вызывается таймером. Закрыть фразу по
  паузе может ТОЛЬКО таймер: когда оператор замолчал, кадры перестают
  приходить, и `add_frame` больше не вызовется.
* :meth:`WakePhraseSegmenter.reset` — WS-сессия оборвалась: недособранную
  фразу выбросить, чтобы она не склеилась с речью следующей сессии.

Чистый Python: без rclpy/aiohttp, тестируется на dev-машине без ROS
(см. `test/unit/core/test_wake_segmenter.py`).
"""

from __future__ import annotations

from typing import List, Optional

# int16 mono PCM 16 кГц — формат wake-канала (voice_capture.ts).
WAKE_SAMPLE_RATE_HZ: int = 16000
WAKE_BYTES_PER_S: int = WAKE_SAMPLE_RATE_HZ * 2  # 32000 Б/с

# Пауза в потоке кадров, после которой фраза считается законченной.
# Клиент держит hangover 200 мс (voice_capture.ts, VAD_HANGOVER_MS_DEFAULT),
# то есть после последнего «речевого» кадра он шлёт ещё 200 мс и только
# потом замолкает. 400 мс = hangover + запас на джиттер WS: меньше —
# рискуем порвать фразу на сетевой икоте, больше — оператор ждёт ответа.
WAKE_PHRASE_GAP_TIMEOUT_S: float = 0.4

# Потолок буфера. Клиентский VAD теоретически может «залипнуть» на шуме
# (вентилятор, музыка) и лить кадры бесконечно — без потолка буфер съест
# память ноды. 15 с — заведомо больше любой реплики оператора; на потолке
# фразу режем и отдаём как есть, а не выбрасываем (речь важнее границы).
WAKE_PHRASE_MAX_S: float = 15.0
WAKE_PHRASE_MAX_BYTES: int = int(WAKE_PHRASE_MAX_S * WAKE_BYTES_PER_S)

# Нижняя граница: сегмент короче — это блип VAD (щелчок, хлопок двери),
# в котором физически не помещается слово «ТАРС». Отдавать такое в STT
# бессмысленно — ровно из этого состоял дефект #2135.
WAKE_PHRASE_MIN_S: float = 0.25
WAKE_PHRASE_MIN_BYTES: int = int(WAKE_PHRASE_MIN_S * WAKE_BYTES_PER_S)


class WakePhraseSegmenter:
    """Кадры wake-канала → фразы. Состояние — один буфер и метка времени.

    Не потокобезопасен: и `add_frame` (aiohttp event-loop), и `tick`
    (ROS-таймер) вызываются из моста, где публикация в rclpy уже
    сериализована GIL'ом на уровне одного вызова. Буфер — список bytes,
    склейка один раз на фразу (а не конкатенация на каждый кадр).
    """

    def __init__(
        self,
        gap_timeout_s: float = WAKE_PHRASE_GAP_TIMEOUT_S,
        max_bytes: int = WAKE_PHRASE_MAX_BYTES,
        min_bytes: int = WAKE_PHRASE_MIN_BYTES,
    ) -> None:
        self._gap_timeout_s = gap_timeout_s
        self._max_bytes = max_bytes
        self._min_bytes = min_bytes
        self._frames: List[bytes] = []
        self._buffered_bytes = 0
        self._last_frame_at: Optional[float] = None
        # Observability: сколько блипов отброшено как «короче min_bytes»
        # и сколько раз фразу пришлось резать по потолку.
        self.dropped_short_phrases = 0
        self.truncated_phrases = 0

    @property
    def buffered_bytes(self) -> int:
        """Сколько байт лежит в незакрытой фразе (0 — буфер пуст)."""
        return self._buffered_bytes

    def add_frame(self, payload: bytes, now_monotonic: float) -> Optional[bytes]:
        """Принять кадр. Вернуть фразу, если она закрылась этим вызовом.

        Два повода закрыть фразу здесь:

        1. Перед этим кадром была пауза ≥ ``gap_timeout_s`` — значит кадр
           открывает НОВУЮ фразу, а старую надо отдать (таймер мог не
           успеть, если пауза совпала с приходом следующей реплики).
        2. Буфер упёрся в ``max_bytes`` — режем принудительно.
        """
        phrase = self._close_if_gap(now_monotonic)
        self._frames.append(payload)
        self._buffered_bytes += len(payload)
        self._last_frame_at = now_monotonic
        if phrase is None and self._buffered_bytes >= self._max_bytes:
            self.truncated_phrases += 1
            phrase = self._close()
        return phrase

    def tick(self, now_monotonic: float) -> Optional[bytes]:
        """Таймерный тик: закрыть фразу, если поток кадров замолчал.

        Единственный путь, по которому фраза закрывается в реальной жизни:
        оператор договорил → клиентский VAD отпустил → кадры кончились →
        `add_frame` больше не вызывается.
        """
        return self._close_if_gap(now_monotonic)

    def reset(self) -> int:
        """Выбросить недособранную фразу. Возвращает число потерянных байт.

        Вызывается при разрыве WS-сессии: половина фразы прошлого оператора
        не должна приклеиться к речи следующего.
        """
        dropped = self._buffered_bytes
        self._frames = []
        self._buffered_bytes = 0
        self._last_frame_at = None
        return dropped

    def _close_if_gap(self, now_monotonic: float) -> Optional[bytes]:
        """Фраза закончена, если с последнего кадра прошло ≥ gap_timeout_s."""
        if self._last_frame_at is None or not self._frames:
            return None
        if now_monotonic - self._last_frame_at < self._gap_timeout_s:
            return None
        return self._close()

    def _close(self) -> Optional[bytes]:
        """Склеить буфер в фразу и очистить состояние.

        None — если фраза короче ``min_bytes`` (блип VAD, см. модульный
        докстринг): буфер всё равно очищается, наружу ничего не идёт.
        """
        data = b"".join(self._frames)
        self._frames = []
        self._buffered_bytes = 0
        self._last_frame_at = None
        if len(data) < self._min_bytes:
            self.dropped_short_phrases += 1
            return None
        return data
