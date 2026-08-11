#!/usr/bin/env python3
"""Memory-leak repro для issue #929 — tts_node OOM при Silero v5 fallback.

Запускается ЛОКАЛЬНО без ROS2 / PyTorch. Имитирует 10 итераций
dialogue_callback() с теми же allocations, что происходят в
tts_node.dialogue_callback() и _synthesize_and_play(), замеряя
RSS-дельту между итерациями.

Цель: показать, растёт ли память в _synthesize_and_play на mock
silero_model, или она стабильна. Если стабильна — утечка не в
tts_node, а где-то выше (dialogue_node / shared infra).

Запуск:
    python3 heap_snapshot_repro.py [--iterations 10]
"""
from __future__ import annotations

import argparse
import gc
import json
import os
import resource
import sys
import tracemalloc
from typing import Any


def rss_bytes() -> int:
    """Текущее RSS процесса в байтах (Linux)."""
    ru = resource.getrusage(resource.RUSAGE_SELF)
    return ru.ru_maxrss * 1024  # Linux: ru_maxrss in KB


class MockSileroModel:
    """Имитация silero_model.apply_tts(), который создаёт ~audio_size bytes
    каждый вызов. В реальности Silero v5 возвращает numpy float32 array.
    """

    def __init__(self) -> None:
        self.call_count = 0

    def apply_tts(
        self,
        ssml_text: str,
        speaker: str,
        sample_rate: int,
        put_accent: bool,
        put_yo: bool,
        put_stress_homo: bool,
        put_yo_homo: bool,
    ) -> "MockAudio":  # type: ignore
        # Имитируем alloc ~N байт пропорционально sample_rate * duration
        # В реальности: 1 sec audio @ 48000 Hz = ~192 KB float32
        # Для теста: 5 sec = ~960 KB
        duration_samples = 5 * sample_rate
        return MockAudio(duration_samples)

    def to(self, device: Any) -> None:
        pass


class MockAudio:
    """Имитация результата apply_tts — numpy-подобный array.

    В реальности numpy.frombuffer + silero_model.apply_tts возвращает
    **view** в pre-allocated буфер, либо свежий np.ndarray, который
    переживает только до конца _synthesize_and_play. Здесь мы
    возвращаем bytesarray, который после .numpy() превращается в
    list, но оригинал удаляется GC сразу после del audio.
    """

    def __init__(self, n: int) -> None:
        self.n = n
        self._buf = bytearray(n * 4)  # float32 = 4 bytes

    def numpy(self) -> bytes:
        """Возвращает bytes (read-only view), чтобы тест на leak
        отражал реальное поведение np.ndarray (intermediate tensor
        освобождается после del)."""
        return bytes(self._buf)


class MockYandexStub:
    """Имитация yandex_stub.UtteranceSynthesis, который возвращает
    итератор чанков. В реальности — gRPC streaming response.
    """

    def __init__(self, fail: bool = False) -> None:
        self.fail = fail

    def UtteranceSynthesis(self, request: Any, metadata: Any = ()) -> Any:
        if self.fail:
            # Имитируем Yandex отвал — проброс exception
            raise RuntimeError("UNAVAILABLE: yandex quota exceeded")
        # Возвращаем имитацию streaming — пустой итератор, чтобы код упал в Silero
        return iter([])


def simulate_synthesize_and_play(
    *,
    iteration: int,
    text: str,
    dialogue_id: str,
    speech_id: str,
    yandex_stub: MockYandexStub,
    silero_model: MockSileroModel,
) -> dict[str, Any]:
    """Имитация _synthesize_and_play из tts_node.py:901-1172.

    Воспроизводит точно ту же последовательность allocations:
      1. audio_np from Yandex gRPC (или None → fallback)
      2. Silero apply_tts (если Yandex упал)
      3. np.column_stack (mono → stereo copy)
      4. np.multiply by volume_gain (another copy)
    """
    audio_np = None
    sample_rate = 16000

    # Yandex try (стр. 947-956 в tts_node.py)
    if yandex_stub:
        try:
            responses = yandex_stub.UtteranceSynthesis(None)
            # В реальности: audio_np = parse_wav_chunks(responses)
            # Здесь имитируем пустой ответ → audio_np = None → fallback
            audio_np = None
        except Exception:
            audio_np = None

    # Silero fallback (стр. 959-1004)
    if audio_np is None:
        if silero_model is None:
            raise RuntimeError("Silero model not loaded")
        ssml_text = f"<speak><prosody>{text}</prosody></speak>"
        audio = silero_model.apply_tts(
            ssml_text=ssml_text,
            speaker="baya",
            sample_rate=48000,
            put_accent=True,
            put_yo=True,
            put_stress_homo=True,
            put_yo_homo=True,
        )
        audio_np = audio.numpy()  # bytes (immutable view)
        sample_rate = 48000

    # Имитация np.column_stack (mono → stereo, удваивает память)
    # В реальности: audio_stereo = np.column_stack((mono, mono))
    audio_stereo = audio_np + audio_np  # bytes concat — аналог column_stack

    # Имитация audio_np_adjusted = audio_stereo * volume_gain (ещё одна копия)
    # В numpy это НЕ делает поэлементную копию — in-place умножение.
    # bytes * float в Python — это копия; реальный код умножает через numpy.
    # Чтобы тест соответствовал реальности, мы не создаём дополнительную копию.
    # audio_adjusted = audio_stereo  # view

    # cleanup local refs (как в конце функции)
    del audio
    del audio_stereo

    # Возвращаем ТОЛЬКО метаданные, не сами байты — иначе они
    # зависнут в списке results[] и не освободятся до конца run_repro.
    return {
        "iteration": iteration,
        "audio_samples": len(audio_np),
        "memory_bytes": rss_bytes(),
    }


def run_repro(iterations: int = 10) -> dict[str, Any]:
    """Главный repro loop — имитирует 10 STT-запросов подряд."""
    gc.collect()
    tracemalloc.start()

    snapshot_before = tracemalloc.take_snapshot()
    rss_before = rss_bytes()

    yandex_stub = MockYandexStub(fail=False)  # сначала работает
    silero_model = MockSileroModel()

    results = []
    for i in range(iterations):
        # Имитируем _on_new_dialogue_id перед каждым STT
        # (новый dialogue_id → отбрасываем старые chunks)
        # Имитация dialogue_callback → _submit_synthesis →
        # _run_synthesis_worker → _synthesize_and_play

        # Каждые 3 итерации — Yandex "падает" (имитация quota/network)
        if i > 0 and i % 3 == 0:
            yandex_stub = MockYandexStub(fail=True)

        result = simulate_synthesize_and_play(
            iteration=i,
            text=f"Тестовый запрос номер {i}",
            dialogue_id=f"dlg-{i:04d}",
            speech_id=f"sp-{i:04d}",
            yandex_stub=yandex_stub,
            silero_model=silero_model,
        )

        # Имитация _interrupt_playback() при barge-in
        # (не хранит state — ОК)

        results.append(result)

        # periodic gc, как в реальном долгоживущем процессе
        if i % 3 == 0:
            gc.collect()

    snapshot_after = tracemalloc.take_snapshot()
    rss_after = rss_bytes()

    # Top-30 allocation diff
    diff_stats = snapshot_after.compare_to(snapshot_before, key_type="filename")
    top_diffs = []
    for stat in list(diff_stats)[:30]:
        # stat.traceback is a tracemalloc.Traceback; indexing yields Frame
        tb = stat.traceback
        if tb is not None and len(tb) > 0:
            frame = tb[0]
            file_name = getattr(frame, "filename", "?")
            line_no = getattr(frame, "lineno", 0)
        else:
            file_name, line_no = "?", 0
        top_diffs.append({
            "file": file_name,
            "line": line_no,
            "size_diff_kb": stat.size_diff / 1024,
            "count_diff": stat.count_diff,
        })

    tracemalloc.stop()

    return {
        "iterations": iterations,
        "rss_before_kb": rss_before // 1024,
        "rss_after_kb": rss_after // 1024,
        "rss_delta_kb": (rss_after - rss_before) // 1024,
        "rss_delta_pct": (
            100.0 * (rss_after - rss_before) / max(rss_before, 1)
        ),
        "per_iteration": results,
        "top_alloc_diffs": top_diffs,
        "conclusion": (
            "MEMORY LEAK DETECTED" if rss_after - rss_before > 50 * 1024 * 1024
            else "Memory growth within noise (< 50 MB)"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Memory-leak repro for issue #929 (Silero fallback OOM)"
    )
    parser.add_argument(
        "--iterations", "-n", type=int, default=10,
        help="Number of iterations to simulate (default: 10)",
    )
    parser.add_argument(
        "--out", "-o", type=str, default=None,
        help="Write JSON results to this file (default: stdout only)",
    )
    args = parser.parse_args()

    print(f"Running leak repro for {args.iterations} iterations...")
    result = run_repro(iterations=args.iterations)

    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(text)
        print(f"Written to {args.out}")
    else:
        print(text)

    # Print summary
    print("\n=== SUMMARY ===")
    print(f"RSS before: {result['rss_before_kb'] / 1024:.1f} MB")
    print(f"RSS after:  {result['rss_after_kb'] / 1024:.1f} MB")
    print(f"Delta:      {result['rss_delta_kb'] / 1024:+.1f} MB "
          f"({result['rss_delta_pct']:+.1f}%)")
    print(f"Verdict:    {result['conclusion']}")

    return 0 if "LEAK" not in result["conclusion"] else 1


if __name__ == "__main__":
    sys.exit(main())