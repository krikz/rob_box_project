"""
Per-provider TTS chunking + retry-halve (issue #933).

Решает double-fail, обнаруженный в issue #933:
- Yandex gRPC v3 падает на ~291 chars ("Too long text")
- Silero v5 падает на ~1005 chars ("Model couldn't generate your text")
- До #931/#933 был только chunking для Yandex с лимитом 2400 (выше безопасного)
- На 1005-char ответе LLM: Yandex fail → Silero fallback → Silero fail → тишина

Решение (per Подходы 1+2 issue #933):
1. ``CHUNK_LIMITS`` — per-provider max chunk size, настраивается через ROS/YAML.
   Defaults: yandex=700, silero=800, minimax=5000.
2. ``split_text`` — sentence-boundary split (``.!?…\\n``), word-level fallback.
3. ``synthesize_with_retry`` — retry-halve: при ``TooLongError`` режет chunk
   пополам и ретраит, max 3 попытки. Когда ``len(text) < MIN_CHUNK_CHARS`` —
   отдает ошибку наверх (caller переключает provider).

Модуль pure-Python, без зависимостей (ROS, numpy, grpc) — чтобы его можно было
тестировать ``pytest`` без тяжелых dev-deps (torch, grpc, rclpy). Сам
``tts_node.py`` его импортирует.

Связанные issue:
- #931 — chunking для Yandex (initial fix, недостаточный лимит)
- #933 — Silero падает на 1005 chars, нужен per-provider + retry-halve
- #929 — OOM killer (lazy-load Silero 7s задержка)
"""

from __future__ import annotations

import re
from typing import Callable, Iterable, List, Sequence, Tuple


# Per-provider max chunk size (chars). При превышении — split_text() + retry-halve.
# Defaults выбраны эмпирически (issue #933, observation table):
#   yandex_grpc_v3: 700 (text ≤700 chars обычно OK; >700 → halve до ≤350 → 175)
#   silero_v5:      800 (silero stable до ~800; >1000 уже "Synthesis error")
#   minimax:        5000 (HTTP T2A v2 принимает длинные тексты)
#
# W5: ROS-параметры / YAML могут переопределить эти дефолты.
CHUNK_LIMITS: dict[str, int] = {
    "yandex_grpc_v3": 700,
    "silero_v5": 800,
    "minimax": 5000,
}

# Минимально допустимый кусок для retry-halve. Ниже — нет смысла дробить:
# либо провайдер вообще не может или не стоит тратить попытки.
MIN_CHUNK_CHARS: int = 50

# Кол-во попыток в synthesize_with_retry: 1 оригинал + (max_retries-1) halved retries.
DEFAULT_MAX_RETRIES: int = 3


# ---------------------------------------------------------------------------
# Sentence-boundary split
# ---------------------------------------------------------------------------

# Границы предложений: . ! ? … \n (Unicode ellipsis).
_SENTENCE_SENTINELS = ".!?…\n"
# Регексп: lookahead на whitespace после sentence-terminator.
_SENTENCE_RE = re.compile(r"(?<=[.!?…])\s+")


def split_text(
    text: str,
    max_chars: int,
    *,
    sentence_separators: str = _SENTENCE_SENTINELS,
) -> List[str]:
    """Разбить ``text`` на чанки ≤ ``max_chars`` по границам предложений.

    Алгоритм (greedy, O(n)):

    1. Если ``len(text) <= max_chars`` → ``[text]``.
    2. Идём по тексту, копим ``current``.
    3. На границе предложения (sentinel+whitespace) — закрываем чанк, **только**
       если остаток текста сам не влезет в новый чанк (look-ahead), иначе
       хватаем дальше.
    4. Если ``len(current) >= max_chars`` и границы предложения ещё нет —
       бэкофимся до последнего whitespace в текущем окне (word-level split).
       Если whitespace нет в верхней половине — hard slice.
    5. Возвращаемые чанки <= ``max_chars`` (если один «абсурдный» word длиннее
       лимита — он остаётся как есть, но это обработка провайдера).

    Args:
        text: исходный текст (после нормализации).
        max_chars: жёсткий лимит на длину чанка.
        sentence_separators: символы-разделители предложений.

    Returns:
        list[str] — фрагменты, объединение которых покрывает исходный текст;
        каждый ≤ ``max_chars``. Пустой вход → ``[]``.

    Examples:
        >>> split_text("Короткий текст.", max_chars=100)
        ['Короткий текст.']
        >>> chunks = split_text("А. " * 2000, max_chars=100)
        >>> all(len(c) <= 100 for c in chunks)
        True
    """
    if not text:
        return []
    text = text.strip()
    if not text:
        return []
    if max_chars <= 0:
        raise ValueError("max_chars must be > 0")
    if len(text) <= max_chars:
        return [text]

    sep_set = set(sentence_separators)
    chunks: List[str] = []
    current = ""
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        current += ch
        # Close at sentence boundary only if we're under the limit.
        if ch in sep_set and len(current) > 0 and len(current) < max_chars:
            # Look ahead: если остаток текста сам влезет в следующий чанк —
            # закрываем текущий.
            remaining = text[i + 1:].lstrip()
            if len(remaining) <= max_chars - len(current):
                current += text[i + 1:]
                i = n
                chunks.append(current.strip())
                current = ""
                break
            if len(current) >= max_chars * 0.4:
                # Reasonable sentence — close the chunk.
                chunks.append(current.strip())
                current = ""
        elif len(current) >= max_chars:
            # Sentence didn't end in time — force a word-level split.
            last_space = current.rfind(" ")
            if last_space > max_chars * 0.5:
                head = current[:last_space].strip()
                tail = current[last_space + 1:]
                if head:
                    chunks.append(head)
                current = tail
            else:
                # No whitespace in the second half — hard slice.
                chunks.append(current[:-1].strip())
                current = current[-1]
            # If even this single segment exceeds the limit (one absurdly
            # long "word"), accept it; provider will reject it and we'll
            # retry with halved size (handled by retry-halve layer).
            if len(current) > max_chars:
                chunks.append(current.strip())
                current = ""
        i += 1

    if current.strip():
        chunks.append(current.strip())

    # Filter empty strings (defensive — should not happen).
    chunks = [c for c in chunks if c]
    if not chunks:
        return [text] if text else []
    return chunks


# ---------------------------------------------------------------------------
# Per-provider config helpers
# ---------------------------------------------------------------------------


def get_chunk_limit(provider: str, overrides: dict[str, int] | None = None) -> int:
    """Вернуть max chunk size для ``provider``.

    Args:
        provider: ключ в :data:`CHUNK_LIMITS` (``yandex_grpc_v3``
            / ``silero_v5`` / ``minimax``).
        overrides: словарь override-ов (например, из ROS-параметров).
            Если ключ есть — вернёт его значение. Иначе — дефолт.

    Returns:
        int: лимит в chars.

    Raises:
        KeyError: если ``provider`` неизвестен и нет override.
    """
    overrides = overrides or {}
    if provider in overrides:
        return int(overrides[provider])
    if provider not in CHUNK_LIMITS:
        raise KeyError(
            f"Unknown TTS provider {provider!r}; "
            f"known: {sorted(CHUNK_LIMITS.keys())} "
            f"(или передайте override через overrides={{'{provider}': N}})"
        )
    return CHUNK_LIMITS[provider]


# ---------------------------------------------------------------------------
# Retry-halve — Подход 2 issue #933
# ---------------------------------------------------------------------------


class TooLongError(Exception):
    """Поднимается, когда провайдер отверг chunk из-за длины текста.

    Используется для активации retry-halve в :func:`synthesize_with_retry`.
    """


def _safe_halve_point(text: str, mid: int) -> int:
    """Найти индекс ближайшего whitespace к ``mid`` (в окне ±30).

    При отсутствии — возврат ``mid`` (hard slice).
    """
    if not text:
        return mid
    n = len(text)
    mid = max(0, min(mid, n))
    # search in [mid-30, mid+30]
    lo = max(0, mid - 30)
    hi = min(n, mid + 30)
    window = text[lo:hi]
    rel = window.rfind(" ")
    if rel < 0:
        return mid
    cut = lo + rel
    # Гарантия non-zero progress
    if cut <= 0:
        return mid
    return cut


def synthesize_with_retry(
    text: str,
    provider: str,
    synthesize_fn: Callable[[str], object],
    *,
    max_chars: int,
    max_retries: int = DEFAULT_MAX_RETRIES,
    is_too_long: Callable[[BaseException], bool],
    min_chunk_chars: int = MIN_CHUNK_CHARS,
) -> List[object]:
    """Синтезировать ``text`` с retry-halve при ``TooLongError``.

    Алгоритм (per issue #933, Подход 2):

    1. Первая попытка — отправить ``text`` как есть (если <= ``max_chars``)
       или разбить через :func:`split_text`.
    2. Если провайдер бросает ``TooLongError`` (см. ``is_too_long``) —
       разрезать chunk пополам по whitespace и ретраить каждую половину.
    3. До ``max_retries`` попыток. На последней — если всё ещё слишком
       длинно, пробрасываем ``TooLongError`` наверх (caller переключает
       provider / падает).

    IMPORTANT: эта функция синхронная. ROS-вызовы внутри ``synthesize_fn``
    должны быть совместимы (Yandex gRPC ``UtteranceSynthesis`` — streaming,
    Silero ``apply_tts`` — sync, MiniMax — async wrapper).

    Args:
        text: текст для синтеза (уже нормализованный).
        provider: имя провайдера (для логирования).
        synthesize_fn: callable(text) → любой объект (audio-payload,
            np.ndarray, etc.). **Не должна** ловить ``TooLongError`` —
            мы её перехватываем здесь.
        max_chars: max chunk size (per-provider, из :func:`get_chunk_limit`).
        max_retries: максимум попыток (default 3).
        is_too_long: callable(exc) → bool, возвращает True если exc —
            это «Too long text» от провайдера.
        min_chunk_chars: ниже этого порога retry-halve не дробит дальше.

    Returns:
        list[object]: аудио-фрагменты (по чанкам). caller склеивает.

    Raises:
        TooLongError: после ``max_retries`` неудачных halved retries
            (chunk короче ``min_chunk_chars``, дальше дробить бессмысленно).
    """
    if not text:
        return []
    if max_chars <= 0:
        raise ValueError("max_chars must be > 0")
    if max_retries < 1:
        raise ValueError("max_retries must be >= 1")

    # Stage 1: split на уровне max_chars (если text > max_chars)
    initial_chunks = split_text(text, max_chars=max_chars)

    # Recursive halving — будем работать как над списком чанков: если
    # один из них fails — режем его пополам и заменяем в списке.
    pending: List[str] = list(initial_chunks)
    out_audio: List[object] = []
    retries_used = 0

    while pending:
        chunk = pending.pop(0)
        try:
            audio = synthesize_fn(chunk)
        except Exception as exc:  # noqa: BLE001 — callback решает какой класс
            if not is_too_long(exc):
                raise
            # TooLongError — нужно дробить.
            if retries_used >= max_retries - 1:
                # последняя попытка — отдаём наверх
                raise TooLongError(
                    f"[{provider}] chunk {len(chunk)} chars still too long "
                    f"after {retries_used} halved retries"
                ) from exc
            if len(chunk) <= min_chunk_chars:
                raise TooLongError(
                    f"[{provider}] chunk {len(chunk)} chars <= min_chunk_chars "
                    f"({min_chunk_chars}); cannot halve further"
                ) from exc
            mid = len(chunk) // 2
            cut = _safe_halve_point(chunk, mid)
            head = chunk[:cut].strip()
            tail = chunk[cut:].lstrip()
            if not head or not tail:
                # Halve дал пустую половину — не прогресс, отдаём наверх.
                raise TooLongError(
                    f"[{provider}] safe-halve produced empty side "
                    f"(chunk={len(chunk)}, cut={cut})"
                ) from exc
            retries_used += 1
            # head идёт раньше (порядок важен — точность синтеза).
            pending.insert(0, tail)
            pending.insert(0, head)
            continue
        out_audio.append(audio)

    return out_audio


__all__ = [
    "CHUNK_LIMITS",
    "MIN_CHUNK_CHARS",
    "DEFAULT_MAX_RETRIES",
    "TooLongError",
    "split_text",
    "get_chunk_limit",
    "synthesize_with_retry",
]
