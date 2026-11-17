"""Нарезка длинной речи на куски для STT (issue #3762, эпик #3683).

Раньше audio_node выбрасывал фразу длиннее ``speech_max_duration`` (15 с)
ДО распознавания: человек объяснял задачу 16-20 с, в бэклог ничего не
попадало, и голое «робот» после этого не имело что подхватывать.

Теперь длинная речь режется на куски ``<= max_s``; каждый уходит в STT как
обычная фраза. Лимиты провайдеров не растут (Yandex gRPC - таймаут 5 с на
фразу, поэтому потолок не поднимаем): граница остаётся одна - параметр
``speech_max_duration`` audio_node. Режем по самой тихой точке в последних
``SEARCH_WINDOW_S`` секундах куска (внутренняя пауза), в худшем случае это
просто конец окна.

Чистая функция без rclpy: решение (где резать) принимает код, не LLM.
"""
from __future__ import annotations

import array
from typing import List

#: Окно поиска паузы в конце куска, с.
SEARCH_WINDOW_S: float = 3.0
#: Шаг анализа энергии, с.
FRAME_S: float = 0.05


def _quietest_offset(buf: bytes, sample_rate: int, lo: int, hi: int) -> int:
    """Смещение (байты, чётное) самого тихого кадра FRAME_S в [lo, hi)."""
    frame_bytes = max(2, int(sample_rate * FRAME_S) * 2)
    best_off, best_energy = hi - hi % 2, None
    for off in range(lo - lo % 2, hi - frame_bytes + 1, frame_bytes):
        samples = array.array("h")
        samples.frombytes(buf[off:off + frame_bytes])
        energy = sum(abs(s) for s in samples)
        if best_energy is None or energy < best_energy:
            best_off, best_energy = off, energy
    return best_off


def split_long_speech(
    buf: bytes, sample_rate: int, max_s: float, min_s: float
) -> List[bytes]:
    """Режет PCM16 на куски ``<= max_s``, хвост не короче ``min_s``.

    Фраза в пределах ``max_s`` возвращается как есть (один элемент).
    """
    max_bytes = int(sample_rate * max_s) * 2
    min_tail = int(sample_rate * min_s) * 2
    if max_bytes <= 0 or len(buf) <= max_bytes:
        return [buf]
    window = int(sample_rate * SEARCH_WINDOW_S) * 2
    chunks: List[bytes] = []
    rest = buf
    while len(rest) > max_bytes:
        hi = min(max_bytes, len(rest) - min_tail)
        cut = _quietest_offset(rest, sample_rate, max(0, max_bytes - window), hi)
        if cut <= 0:
            cut = max_bytes
        chunks.append(rest[:cut])
        rest = rest[cut:]
    if rest:
        chunks.append(rest)
    return chunks


def publish_speech_chunks(buf, pub, msg_cls, logger, sample_rate, max_s, min_s) -> int:
    """Нарезать речь и опубликовать каждый кусок как отдельную фразу; число кусков."""
    chunks = split_long_speech(buf, sample_rate, max_s, min_s)
    if len(chunks) > 1:
        logger.info(f'✂️ Длинная речь {len(buf) / (sample_rate * 2):.2f}с нарезана '
                    f'на {len(chunks)} кусков (max={max_s}с)')
    for chunk in chunks:
        msg = msg_cls()
        msg.data = list(chunk)
        pub.publish(msg)
    return len(chunks)
