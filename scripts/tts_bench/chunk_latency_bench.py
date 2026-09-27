#!/usr/bin/env python3
"""TTS chunk-to-chunk latency benchmark for issue #2003 / ADR-0056.

Контракт DoD #2 (ADR-0056 §3.7): speculative pre-generate уменьшает
латентность между последовательными чанками TTS. Этот скрипт измеряет
разницу «на живом» роботе в двух режимах:

  • baseline  — ``pregenerate_enabled=false`` (классический pipeline)
  • speculat. — ``pregenerate_enabled=true``  (default после ADR-0056)

Метрика (per ADR-0056 §3.7 / issue #2003):
    delta_t(i) = t_chunk_{i+1}_ready − t_chunk_i_ready

где ``t_chunk_k_ready`` — wall-clock момент публикации PCM-чанка k в
``/voice/audio/speech`` (по умолчанию). Это соответствует моменту, с
которого нижестоящий audio_play_node начинает воспроизведение.

Из каждой серии N=20 реплик фиксированной длины считаем p50 и p95
серии ``delta_t`` (без первого чанка — у него нет предыдущего).

Использование (ВНУТРИ контейнера voice-assistant):
    docker exec voice-assistant bash -lc \\
      "source /ws/install/setup.bash && \\
       python3 /config/tts_bench/chunk_latency_bench.py \\
         --repeats 20 \\
         --phrase 'робот расскажи короткую историю' \\
         --out /tmp/chunk_latency.json"

Скрипт НЕ переключает ``pregenerate_enabled`` сам — оператор делает
это через ``ros2 param set`` между прогонами (см. README.md).
Это сознательно: нода выводит running mean сама, и скрипт должен
уметь работать в обе стороны, чтобы серии были сравнимы.

Что НЕ делает скрипт
--------------------
* Не публикует в ``/voice/stt/result`` — это делает существующий
  ``scripts/voice_bench/speaker.py`` через rclpy. Здесь мы только
  *читаем* результат (двухпоточная архитектура).
* Не дёргает LLM/инструменты — намеренно берётся та реплика, у
  которой чанков несколько. Выбирайте фразу, на которую
  dialogue_node отвечает связной речью (см. README).
* Не судит о качестве — это контракт ``scripts/tts_bench/quality.py``,
  который живёт отдельно и использует ``pregens_*`` метрики из
  ``/voice/tts/metrics``.

Raw-вывод (по ADR-0018)
-----------------------
Скрипт пишет JSON со списком ``deltas_ms`` по каждой реплике
(per-replication sample). Это и есть «сырьё», на основе которого
пишется p50/p95 в сводной таблице. На основании этого JSON Шифу
или ревьюер может пересчитать статистику любым инструментом.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any

try:
    import rclpy
    from audio_common_msgs.msg import AudioData  # type: ignore
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
except ImportError as exc:  # noqa: BLE001
    sys.exit(
        f"❌ требует ROS2 окружения (rclpy + audio_common_msgs): {exc!r}\n"
        "   запускать внутри voice-assistant: source /ws/install/setup.bash"
    )


#: Топик, в котором TTSNode публикует готовые PCM-чанки.
#: Совпадает с дефолтом src/rob_box_voice/config/tts_node.yaml:audio_topic.
DEFAULT_AUDIO_TOPIC = "/voice/audio/speech"

#: Топик, в который speaker пишет «распознанную речь» от лица STT.
DEFAULT_STT_TOPIC = "/voice/stt/result"


class ChunkLatencyRecorder:
    """Подписчик на ``/voice/audio/speech``: фиксирует wall-clock каждого PCM-чанка.

    Топик QoS — ``RELIABLE/KEEP_LAST(depth=1)``, чтобы recorder
    гарантированно получал все чанки в порядке публикации.
    """

    def __init__(self, node: "rclpy.node.Node", audio_topic: str) -> None:
        self._node = node
        self._qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = node.create_subscription(
            AudioData,
            audio_topic,
            self._cb,
            self._qos,
        )
        # wall-clock моменты получения каждого чанка (монотонно растёт)
        self.chunk_recv_times: list[float] = []

    def _cb(self, msg: "audio_common_msgs.msg.AudioData") -> None:
        # Используем ``time.monotonic()`` — устойчив к скачкам NTP,
        # нам важны интервалы, а не абсолютное время.
        self.chunk_recv_times.append(time.monotonic())

    def reset(self) -> None:
        self.chunk_recv_times.clear()

    def deltas_ms(self) -> list[float]:
        """Возвращает ``[t_{i+1} - t_i] * 1000`` по последнему окну.

        Если чанков 0 или 1 — возвращает ``[]``.
        """
        t = self.chunk_recv_times
        if len(t) < 2:
            return []
        return [(t[i + 1] - t[i]) * 1000.0 for i in range(len(t) - 1)]


class PhraseSpeaker:
    """Публикует реплику в ``/voice/stt/result`` от лица STT.

    Аналог speaker из ``scripts/voice_bench/run_bench.py``,
    но в режиме «один раз на реплику» — без DJ-логики.
    """

    def __init__(self, node: "rclpy.node.Node", stt_topic: str) -> None:
        self._node = node
        self.pub = node.create_publisher(
            String,
            stt_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

    def wait_ready(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self._node, timeout_sec=0.2)
        return False

    def say(self, text: str) -> None:
        self.pub.publish(String(data=text))
        for _ in range(6):
            rclpy.spin_once(self._node, timeout_sec=0.1)


def percentile(values: list[float], p: float) -> float:
    """Перцентиль по «linear interpolation» (эквивалент ``np.percentile``).

    Совпадает с numpy/scipy по умолчанию:
        idx = max(0, min(n - 1, int(round(p / 100 * (n - 1)))))

    Для p=50, n=100 → idx=50 → ``xs[50]`` (медиана списка 1..100 = 51).
    Для p=95, n=20 → idx=round(17.85)=18 → ``xs[18]``.
    """
    if not values:
        return 0.0
    s = sorted(values)
    n = len(s)
    k = max(0, min(n - 1, int(round(p / 100.0 * (n - 1)))))
    return float(s[k])


def _wait_idle(recorder: ChunkLatencyRecorder, settle_s: float = 1.5) -> None:
    """Ждём, пока чанки перестанут сыпаться (холостая пауза между репликами).

    Сравниваем длину списка с прошлым тиком; как только она не растёт
    ``settle_s`` секунд подряд — считаем, что pipeline доиграл.
    """
    deadline = time.monotonic() + max(8.0, settle_s * 6)
    last = -1
    while time.monotonic() < deadline:
        size = len(recorder.chunk_recv_times)
        if size == last:
            return
        last = size
        time.sleep(settle_s)


def run_one_replication(
    recorder: ChunkLatencyRecorder,
    speaker: PhraseSpeaker,
    phrase: str,
    pause_s: float,
) -> dict[str, Any]:
    """Один прогон: опубликовать реплику, собрать delta_t, вернуть сырьё."""
    _wait_idle(recorder)
    recorder.reset()
    t0 = time.monotonic()
    speaker.say(phrase)
    # Прокручиваемся, пока есть хотя бы один новый чанк + холостая пауза.
    # Нижняя граница — 4 секунды: <~10 с реплика должна успеть родить
    # первый чанк даже на медленном MiniMax.
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline:
        rclpy.spin_once(recorder._node, timeout_sec=0.3)
        if len(recorder.chunk_recv_times) >= 2:
            _wait_idle(recorder)
            break
    deltas_ms = recorder.deltas_ms()
    elapsed_total_ms = (time.monotonic() - t0) * 1000.0
    if pause_s > 0:
        time.sleep(pause_s)
    return {
        "chunks": len(recorder.chunk_recv_times),
        "deltas_ms": deltas_ms,
        "wall_ms": elapsed_total_ms,
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repeats", type=int, default=20,
                    help="сколько раз повторить реплику (default 20)")
    ap.add_argument("--phrase", required=True,
                    help="реплика от лица STT, формат: 'робот <cmd>'")
    ap.add_argument("--pause", type=float, default=4.0,
                    help="пауза между репликами, сек")
    ap.add_argument(
        "--mode", default="speculative",
        help=("метка в JSON-отчёте: speculative|baseline. "
              "Переключение режима делается через "
              "`ros2 param set /tts_node pregenerate_enabled {true|false}` "
              "МЕЖДУ запусками скрипта. Сам скрипт не дёргает параметр."),
    )
    ap.add_argument("--stt-topic", default=DEFAULT_STT_TOPIC)
    ap.add_argument("--audio-topic", default=DEFAULT_AUDIO_TOPIC)
    ap.add_argument("--out", required=True, type=Path,
                    help="куда писать JSON-сырьё")
    args = ap.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node("tts_chunk_latency_bench")
    recorder = ChunkLatencyRecorder(node, args.audio_topic)
    speaker = PhraseSpeaker(node, args.stt_topic)
    if not speaker.wait_ready():
        print(
            f"⚠️  подписчика на {args.stt_topic} нет — "
            "dialogue_node/tts_node жив?",
            file=sys.stderr,
        )

    replications: list[dict[str, Any]] = []
    try:
        for rep in range(1, args.repeats + 1):
            print(f"[{rep}/{args.repeats}] mode={args.mode} phrase={args.phrase!r}",
                  flush=True)
            turn = run_one_replication(recorder, speaker, args.phrase, args.pause)
            turn.update(repeat=rep)
            replications.append(turn)
            if turn["deltas_ms"]:
                m = statistics.fmean(turn["deltas_ms"])
                print(
                    f"    → chunks={turn['chunks']} "
                    f"deltas_n={len(turn['deltas_ms'])} "
                    f"mean={m:.1f}ms",
                    flush=True,
                )
            else:
                print(
                    f"    → chunks={turn['chunks']} (не хватило чанков для delta)",
                    flush=True,
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Сводим статистику: каждый delta считается как отдельный sample
    # (это намеренно — p95 по всему распределению, не по среднему серии).
    all_deltas: list[float] = []
    for turn in replications:
        all_deltas.extend(turn["deltas_ms"])
    summary = {
        "mode": args.mode,
        "phrase": args.phrase,
        "repeats": args.repeats,
        "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        "n_chunks_total": sum(t["chunks"] for t in replications),
        "n_deltas_total": len(all_deltas),
        "p50_ms": percentile(all_deltas, 50),
        "p95_ms": percentile(all_deltas, 95),
        "mean_ms": statistics.fmean(all_deltas) if all_deltas else 0.0,
        # Полный список — для воспроизводимой пересборки p95
        # (на случай если автор скрипта поменяет определение перцентиля).
        "all_deltas_ms": all_deltas,
    }
    payload = {
        "config": {
            "audio_topic": args.audio_topic,
            "stt_topic": args.stt_topic,
            "pause_s": args.pause,
        },
        "summary": summary,
        "replications": replications,
    }
    args.out.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(
        f"\n[{args.mode}] p50={summary['p50_ms']:.1f}ms "
        f"p95={summary['p95_ms']:.1f}ms "
        f"n_deltas={summary['n_deltas_total']}",
        flush=True,
    )
    print(f"сырьё записано: {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
