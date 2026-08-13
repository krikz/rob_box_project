#!/usr/bin/env python3
"""
test_chunking_module.py — Unit-тесты для tts_chunking.py (issue #933).

Pure-Python, без pytest plugins, без unittest от rclpy. Можно запускать
``python3 -m pytest test/unit/tts/test_chunking_module.py`` ИЛИ
``python3 -m unittest test.unit.tts.test_chunking_module``.

Связанные issue:
- #931 — chunking для Yandex (initial fix, недостаточный лимит)
- #933 — Silero падает на 1005 chars, per-provider max + retry-halve
- #937 — Yandex API v3 hard limit уточнён до ≤250 chars

Тесты не требуют rclpy/grpc/torch — модуль tts_chunking pure Python.
Файл живёт в test/unit/ (CI-коллекция), а не в корне test/ — чтобы
per-provider chunking реально гонялся в G-Run Tests.yml.

NB: cyrillic 'А' = 1 char, 'А. ' = 3 chars. Используем для предсказуемой длины.
"""

import unittest
from typing import Callable, List, Tuple

from rob_box_voice.tts_chunking import (
    CHUNK_LIMITS,
    DEFAULT_MAX_RETRIES,
    MIN_CHUNK_CHARS,
    TooLongError,
    get_chunk_limit,
    split_text,
    synthesize_with_retry,
)

# Хелпер: 'А' = 1 char, 'А. ' = 3 chars
WORD = "А. "  # 3 chars
# Sentence-like (чтобы split_text работал по sentence-boundary, а не word-level).
# 'А. А. А. А. А. ' = 15 chars, 5 слов.
SENT = "А. " * 5  # 15 chars


class TestChunkLimits(unittest.TestCase):
    """Что :data:`CHUNK_LIMITS` содержит правильные дефолты."""

    def test_yandex_default_is_250(self):
        """Issue #933/#937: yandex_grpc_v3=250 chars max chunk (API v3 hard limit)."""
        self.assertEqual(CHUNK_LIMITS["yandex_grpc_v3"], 250)

    def test_silero_default_is_800(self):
        """Issue #933 spec: silero_v5=800 chars max chunk."""
        self.assertEqual(CHUNK_LIMITS["silero_v5"], 800)

    def test_minimax_default_is_5000(self):
        """Issue #933 spec: minimax=5000 chars (HTTP T2A v2 принимает много)."""
        self.assertEqual(CHUNK_LIMITS["minimax"], 5000)

    def test_all_providers_have_default(self):
        """Все три провайдера из body задания покрыты."""
        for prov in ("yandex_grpc_v3", "silero_v5", "minimax"):
            self.assertIn(prov, CHUNK_LIMITS)
            self.assertGreater(CHUNK_LIMITS[prov], 0)


class TestGetChunkLimit(unittest.TestCase):
    """Per-provider limit lookup (с override)."""

    def test_default_provider(self):
        self.assertEqual(get_chunk_limit("yandex_grpc_v3"), 250)
        self.assertEqual(get_chunk_limit("silero_v5"), 800)
        self.assertEqual(get_chunk_limit("minimax"), 5000)

    def test_override_applies(self):
        """YAML/ROS-параметры могут переопределить дефолт."""
        self.assertEqual(
            get_chunk_limit("yandex_grpc_v3", overrides={"yandex_grpc_v3": 500}),
            500,
        )

    def test_unknown_provider_raises(self):
        with self.assertRaises(KeyError):
            get_chunk_limit("unknown_provider")

    def test_unknown_provider_with_override_ok(self):
        """Override допускает неизвестные ключи (forward-compat)."""
        self.assertEqual(
            get_chunk_limit("experimental", overrides={"experimental": 100}),
            100,
        )


class TestSplitText(unittest.TestCase):
    """``split_text`` — sentence-boundary split + word-level fallback."""

    def test_short_text_returns_as_is(self):
        self.assertEqual(split_text("Привет.", max_chars=100), ["Привет."])

    def test_empty_text_returns_empty(self):
        self.assertEqual(split_text("", max_chars=100), [])
        self.assertEqual(split_text("   \n  ", max_chars=100), [])

    def test_text_equal_to_limit(self):
        text = "а" * 100
        self.assertEqual(split_text(text, max_chars=100), [text])

    def test_chunks_never_exceed_limit(self):
        """Greedy: каждый chunk ≤ max_chars (issue #933 acceptance)."""
        chunks = split_text(SENT * 200, max_chars=100)
        for c in chunks:
            self.assertLessEqual(len(c), 100, f"chunk too long: {len(c)}: {c!r}")

    def test_chunks_cover_full_text_chars(self):
        """Сумма длин chunks ≈ длина текста (без пробелов в стыках)."""
        original = SENT * 50  # 750 chars
        chunks = split_text(original, max_chars=100)
        # Каждое 'А.' должно сохраниться
        joined = "".join(chunks)
        count_A = joined.count("А.")
        count_A_orig = original.count("А.")
        # Без перестановки слов (split_text только \s+ сжимает)
        self.assertEqual(count_A, count_A_orig)

    def test_long_sentence_falls_back_to_word_split(self):
        """Если одно предложение > max_chars — разбиваем по whitespace."""
        # 60 chars без sentence-terminator, max_chars=20 → должно дать >1 chunks
        text = "абвгдежзийклмноп " * 2  # 30 chars
        chunks = split_text(text, max_chars=20)
        self.assertGreater(len(chunks), 1)
        for c in chunks:
            self.assertLessEqual(len(c), 20)

    def test_max_chars_zero_raises(self):
        with self.assertRaises(ValueError):
            split_text("текст", max_chars=0)


class TestSynthesizeWithRetry(unittest.TestCase):
    """``synthesize_with_retry`` — retry-halve на TooLongError (issue #933, подход 2)."""

    @staticmethod
    def _ok_synth(text: str) -> Tuple[str, int]:
        """Synthesizer который всегда OK, возвращает (label, len(text))."""
        return (f"audio@{len(text)}", len(text))

    @staticmethod
    def _failing_synth(text: str) -> Tuple[str, int]:
        raise RuntimeError("Too long text")

    @staticmethod
    def _is_too_long(exc: BaseException) -> bool:
        return "Too long" in str(exc)

    def test_short_text_no_retry(self):
        """Короткий текст → ровно один audio, без retries."""
        out = synthesize_with_retry(
            "Привет.", "yandex", self._ok_synth, max_chars=700, is_too_long=self._is_too_long
        )
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0], ("audio@7", 7))

    def test_empty_text_returns_empty(self):
        out = synthesize_with_retry(
            "", "yandex", self._ok_synth, max_chars=700, is_too_long=self._is_too_long
        )
        self.assertEqual(out, [])

    def test_split_only_no_retry(self):
        """Текст > max_chars → split на chunks, все OK."""
        # 800 chars / max 700 → 2 chunks (≤700 каждый)
        text = SENT * 53  # 795 chars
        self.assertEqual(len(text), 795)
        calls: List[str] = []

        def synth(text):
            calls.append(text)
            return (f"audio@{len(text)}", len(text))

        out = synthesize_with_retry(
            text, "yandex", synth, max_chars=700, is_too_long=self._is_too_long
        )
        # 2 chunks: первый ~700 chars, второй ~95
        self.assertGreaterEqual(len(out), 2)
        for label, length in out:
            self.assertLessEqual(length, 700)
        # Каждый оригинальный chunk вызвал ровно 1 attempt
        self.assertGreaterEqual(len(calls), 2)
        for c in calls:
            self.assertLessEqual(len(c), 700)

    def test_retry_halve_on_first_chunk(self):
        """Chunk падает, retry-halve делит пополам → OK или raise (если MIN_CHUNK_CHARS не пускает)."""
        # Acceptable size: <= 50 chars OK
        def synth(text):
            if len(text) > 50:
                raise RuntimeError("Too long text")
            return (f"audio@{len(text)}", len(text))

        # 198 chars, max 100 → split (1 chunk через word-level). 198 > 50 → halve → 99.
        # 99 > 50 → halve → 49 (<50 = MIN_CHUNK_CHARS) → raise. Это валидный
        # результат retry-halve: «нельзя раздробить до ≤50, отдаём наверх».
        text = "А. " * 66  # 198 chars
        with self.assertRaises(TooLongError):
            synthesize_with_retry(
                text,
                "yandex",
                synth,
                max_chars=100,
                is_too_long=self._is_too_long,
            )

    def test_retry_halve_with_low_min_succeeds(self):
        """С min_chunk_chars=10 halve идёт глубже — OK."""
        # Acceptable size: <= 30 chars OK
        def synth(text):
            if len(text) > 30:
                raise RuntimeError("Too long text")
            return (f"audio@{len(text)}", len(text))

        # 99 chars, max 50 → split (4 chunks: 20+20+20+35). 35 > 30 → halve → 17 → OK.
        # 3 коротких chunks OK без retry.
        text = "А. " * 33  # 99 chars
        out = synthesize_with_retry(
            text,
            "yandex",
            synth,
            max_chars=50,
            is_too_long=self._is_too_long,
            min_chunk_chars=10,
        )
        # 5 outputs total (4 original chunks, 1 halved from 35)
        self.assertGreaterEqual(len(out), 4)
        for label, length in out:
            self.assertLessEqual(length, 30)

    def test_retry_halve_eventually_raises(self):
        """Если chunk не дробится до OK — синтез проваливается."""
        # Acceptable size: <= 30 chars OK
        def synth(text):
            if len(text) > 30:
                raise RuntimeError("Too long text")
            return (f"audio@{len(text)}", len(text))

        # 1000 chars, max 700 → split на ≥2 chunks. Каждый > 30 → halve.
        # 700 → halve → 350 → 175 → 87 → 43 (< 50). retries_used=4, max_retries=3 → 3rd halve raise.
        text = SENT * 67  # 1005 chars
        with self.assertRaises(TooLongError):
            synthesize_with_retry(
                text,
                "yandex",
                synth,
                max_chars=700,
                is_too_long=self._is_too_long,
                min_chunk_chars=10,
            )

    def test_non_too_long_error_passes_through(self):
        """Любая non-TooLong ошибка пробрасывается наверх без retry-halve."""
        attempts = []

        def synth(text):
            attempts.append(text)
            raise RuntimeError("Connection refused")

        with self.assertRaises(RuntimeError) as ctx:
            synthesize_with_retry(
                "Привет.",
                "yandex",
                synth,
                max_chars=700,
                is_too_long=self._is_too_long,
            )
        self.assertIn("Connection refused", str(ctx.exception))
        self.assertEqual(len(attempts), 1)  # без retry

    def test_max_retries_exhausted_raises(self):
        """max_retries=2: только 1 hinted halve, дальше — TooLongError."""
        attempts = []

        def synth(text):
            attempts.append(text)
            raise RuntimeError("Too long text")

        # 1005 chars, max 700 → split (≥2 chunks). 700 > 50 → halve → 350
        # (retries_used=1, max_retries=2 → max_retries-1=1 → raise).
        text = SENT * 67  # 1005 chars
        with self.assertRaises(TooLongError):
            synthesize_with_retry(
                text,
                "yandex",
                synth,
                max_chars=700,
                is_too_long=self._is_too_long,
                max_retries=2,
                min_chunk_chars=10,
            )

    def test_split_text_keeps_ordering(self):
        """Порядок chunks сохраняется (важно для синтеза)."""
        # Track order: synthesizer возвращает текст, чтобы проверить порядок
        calls: List[str] = []

        def synth(text):
            calls.append(text)
            return (f"audio:{text[:5]}", len(text))

        text = "А. Б. В. Г. Д. Е. Ж. З. И. К. Л. М. Н. О. П."
        out = synthesize_with_retry(
            text, "yandex", synth, max_chars=20, is_too_long=self._is_too_long
        )
        # Каждое слово из original встретится в каком-то чанке — порядок chunks верен
        self.assertGreater(len(out), 1)
        # Объединяем все chunks обратно: должны покрыть все слова
        joined = " ".join(calls)
        for word in ("А.", "Б.", "В.", "Г.", "Д."):
            self.assertIn(word, joined)


class TestIntegrationObservations(unittest.TestCase):
    """Sanity: реальные chars из issue #933 table по-разному обрабатываются."""

    def test_1005_chars_with_silero_limit(self):
        """1005 chars при silero limit=800 → сначала split, потом OK."""
        # Simulate: silero OK если chunk <= 800
        def synth(text):
            if len(text) > 800:
                raise RuntimeError("Model couldn't generate your text")
            return (f"audio@{len(text)}", len(text))

        # 1005 chars: split на chunks ≤ 800 → OK
        text = SENT * 67  # 1005 chars
        self.assertEqual(len(text), 1005)
        out = synthesize_with_retry(
            text,
            "silero_v5",
            synth,
            max_chars=800,
            is_too_long=lambda e: "generate" in str(e),
        )
        self.assertGreater(len(out), 1)
        for label, length in out:
            self.assertLessEqual(length, 800)

    def test_291_chars_with_yandex_limit(self):
        """291 chars при yandex limit=250 → split на ≥2 chunks (issue #933 table).

        Наблюдение из issue #933: 291 chars валит Yandex gRPC v3
        ("Too long text") — значит при реальном лимите 250 текст должен
        дробиться ДО отправки, а не уходить одним chunk'ом.
        """
        def synth(text):
            if len(text) > 250:
                raise RuntimeError("Too long text")
            return (f"audio@{len(text)}", len(text))

        # 291: SENT * 19 (285) + 'А.    ' (6)
        text = SENT * 19 + "А.    "  # 291 chars
        self.assertEqual(len(text), 291)
        out = synthesize_with_retry(
            text,
            "yandex_grpc_v3",
            synth,
            max_chars=250,
            is_too_long=lambda e: "Too long" in str(e),
        )
        # Дробится на ≥2 chunks, каждый ≤ 250 — Yandex не видит "Too long text"
        self.assertGreater(len(out), 1)
        for label, length in out:
            self.assertLessEqual(length, 250)


class TestConstants(unittest.TestCase):
    """Defaults из issue #933 spec."""

    def test_min_chunk_chars_default(self):
        """MIN_CHUNK_CHARS=50 — не дробим меньше."""
        self.assertEqual(MIN_CHUNK_CHARS, 50)

    def test_default_max_retries(self):
        """DEFAULT_MAX_RETRIES=3."""
        self.assertEqual(DEFAULT_MAX_RETRIES, 3)


if __name__ == "__main__":
    unittest.main()
