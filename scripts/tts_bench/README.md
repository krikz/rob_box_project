# scripts/tts_bench — измерение TTS chunk-to-chunk latency (ADR-0056 §3.7)

## Контекст

ADR-0056 §3.7 / issue **#2003** требуют доказать, что speculative
pre-generate уменьшает латентность между соседними PCM-чанками,
которыми TTSNode кормит `audio_play_node`. Контракт DoD #2:

> При включённом `pregenerate_enabled=true` медиана интервала
> «между двумя соседними готовыми PCM-чанками» падает на ≥200 мс
> относительно baseline (`pregenerate_enabled=false`).

**Единица измерения:**
```
delta_t(i) = t_chunk_{i+1}_ready − t_chunk_i_ready   # в мс
```

где `t_chunk_k_ready` — wall-clock момент публикации PCM-чанка k
в топике `/voice/audio/speech` (см. `config/tts_node.yaml:audio_topic`).
Это та точка, в которой нижестоящий `audio_play_node` начинает
воспроизведение — пользовательская «задержка между чанками».

## Что лежит в каталоге

| файл                              | роль                                                      |
|-----------------------------------|-----------------------------------------------------------|
| `chunk_latency_bench.py`          | ROS2-скрипт (запускается внутри `voice-assistant`).       |
| `README.md`                       | этот файл.                                                |

Юнит-тесты на чистую логику (percentile, `deltas_ms`) живут в
`src/rob_box_voice/test/unit/scripts/test_chunk_latency_bench_stats.py`
— без ROS2.

## Процедура запуска (на живом роботе)

### 0. Предусловия

* Стенд жив (`docker ps | grep voice-assistant` — контейнер работает).
* Реплика выбрана так, что dialogue_node отвечает **связной речью** длиной
  не менее ~8 секунд. Иначе в реплике будет 1–2 чанка, и статистика
  потеряет смысл. Авторская рекомендация:

  ```
  "робот расскажи короткую историю"
  ```

  Подойдёт и любой другой «повествовательный» wake-call. Не подходят
  короткие команды («робот, тише»), одиночные ответы LLM и команды
  с `set_dj_mode` (DJ-цикл сам публикует речь ≈каждые 45–75 с — будет
  перехватывать сигнал).

### 1. Baseline (pregenerate OFF)

```bash
# Стенд жив, диалог с предыдущего теста «успокоен».
docker exec voice-assistant bash -lc \
  "source /ws/install/setup.bash && \
   ros2 param set /tts_node pregenerate_enabled false"

docker exec voice-assistant bash -lc \
  "source /ws/install/setup.bash && \
   python3 /config/tts_bench/chunk_latency_bench.py \
     --mode baseline \
     --repeats 20 \
     --phrase 'робот расскажи короткую историю' \
     --out /tmp/chunk_latency_baseline.json"
```

Файл `/tmp/chunk_latency_baseline.json` будет содержать:

```json
{
  "config": { ... },
  "summary": {
    "mode": "baseline",
    "p50_ms": 812.0,
    "p95_ms": 1180.0,
    "mean_ms": 845.1,
    "n_deltas_total": 47,
    "all_deltas_ms": [...]   // raw-данные для пересчёта
  },
  "replications": [ ... 20 реплик ... ]
}
```

### 2. Speculative (pregenerate ON)

```bash
docker exec voice-assistant bash -lc \
  "source /ws/install/setup.bash && \
   ros2 param set /tts_node pregenerate_enabled true"

docker exec voice-assistant bash -lc \
  "source /ws/install/setup.bash && \
   python3 /config/tts_bench/chunk_latency_bench.py \
     --mode speculative \
     --repeats 20 \
     --phrase 'робот расскажи короткую историю' \
     --out /tmp/chunk_latency_speculative.json"
```

### 3. Сводная таблица (для PR-описания)

```text
| mode        | n_deltas | p50_ms | p95_ms | mean_ms | Δp95 vs baseline |
|-------------|----------|--------|--------|---------|-----------------|
| baseline    |       47 |   812  |  1180  |   845   |              —  |
| speculative |       51 |   214  |   378  |   245   |          -802ms |
```

Δp95 ≥200 мс — DoD выполнен (положительное значение Δp95 = уменьшение
латентности относительно baseline; выше в таблице значит лучше).

## Почему скрипт НЕ переключает параметр сам

`pregenerate_enabled` — runtime ROS2-parameter (см. `tts_node.py:4700`),
но `_PrefetchEngine` строится **lazily через `_ensure_prefetch()`**.
Когда скрипт включает/выключает параметр между двумя прогонами, между
ними проходит RESTART pipeline (от первого чанка нового режима), и
первые 1–2 наблюдения надо выкинуть. Проще потребовать отдельный
прогон с явным `--mode baseline` / `--mode speculative`.

## Почему именно /voice/audio/speech, а не /voice/tts/metrics

`tts_node.py:2575` публикует в `/voice/tts/metrics` JSON-сводку с
**running mean** `latency_chunk_to_chunk_ms_mean` — это среднее по
всему времени работы контейнера. Чтобы считать p50/p95 по
распределению, нужны сырые `delta_t`, а не агрегаты. Подписка на
`/voice/audio/speech` даёт wall-clock каждого PCM-чанка без
посредников: `len=Σ` ровно равна числу chunks в реплике.

## Quality (pregens_rejected_*) — отдельный контракт

Качество (`pregens_rejected_quality` / `pregens_rejected_confidence`)
выводится через существующий `/voice/tts/metrics` snapshot и
**не дублируется** здесь — это зона ответственности
`tts_node.publish_prefetch_metrics()`. Для сводки в PR достаточно

```bash
ros2 topic echo /voice/tts/metrics --once
```

сделать дважды: один раз в режиме `baseline`, второй — в `speculative`.
Метрики `pregens_*` в baseline будут нулями (executor не активен) — это
**ожидаемо** и не означает «не работает».

## Troubleshooting

| симптом                                      | вероятная причина                            |
|----------------------------------------------|----------------------------------------------|
| `n_deltas_total=0`                           | реплика слишком короткая, <2 чанков          |
| `chunks=1, deltas=[]` на каждой реплике      | dialogue_node ответил одной репликой         |
| `wait_ready failed: подписчика нет`          | нода упала / не поднялась в этом контейнере  |
| `Δp95 < 0` (speculative хуже baseline)       | см. ADR-0056 §3.6 — снизить confidence_floor |

---

Скрипт **не** прогонялся живым автором этой ветки — стенда в среде
разработки нет (issue #2003 будет закрыт через live run Шифу или
merge-gate → e2e-process → real robot). См. PR-описание в #2003.
