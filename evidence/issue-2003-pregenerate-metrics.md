# Issue #2003 — DoD-evidence: pregenerate latency + quality verdict

Task: `t_3a6be943` (декомпозиция от `t_c44a7cf2`).
Branch: `wt/t_3a6be943`.
Test runner: `pytest 9.1.1`, Python 3.11.15, asyncio mode=STRICT.
Pytest markers: `pregenerate_latency` и `pregenerate_quality`
(зарегистрированы в `src/rob_box_voice/test/unit/pregen/conftest.py::pytest_configure`).

## 1. Latency (DoD #1 + #2)

Команда:

```
PYTHONPATH=src/rob_box_voice:src/rob_box_core:src/rob_box_llm:src/rob_box_perception_msgs \
  pytest src/rob_box_voice/test/unit/pregen/test_pregenerate_latency_marker.py -v -s
```

Тест `test_pregenerate_latency_marker.py::test_pregenerate_latency_baseline_marker`
делает 30 итераций каждого сценария и печатает min/median/mean/max.
Сценарий — два чанка (N и N+1) на чистом SpeculativeExecutor с
заглушкой синтеза `_SYNTH_DELAY_S=0.10s`, аудио 60ms/16kHz sine.

| Режим         | synth_delay | audio | N_RUNS | min (ms) | median (ms) | mean (ms) | max (ms) |
|--------------|-------------|-------|--------|----------|-------------|-----------|----------|
| baseline     | 100 ms      | 60 ms | 30     | 318.3    | 318.7       | 318.7     | 319.1    |
| speculative  | 100 ms      | 60 ms | 30     | 226.1    | 226.8       | 227.2     | 231.2    |

**Прогон 3× подряд (для шумовой устойчивости):**

| Run | baseline median | spec median | Δ (ms) | Δ (%) |
|-----|-----------------|-------------|--------|-------|
| 1   | 318.5           | 226.4       | +92.1  | +28.92% |
| 2   | 318.7           | 227.1       | +91.6  | +28.74% |
| 3   | 318.7           | 226.8       | +91.9  | +28.84% |
| **AGG** | **318.63**  | **226.77**  | **+91.87** | **+28.83%** |

**Вердикт по latency: УЛУЧШЕНИЕ ≈ +28.8% median speedup на двух чанках.**
Это выше sanity-порога теста `>=10%` и эталона `>=15%`
(`test_speculative_path_latency.py`).

## 2. Quality / estimator / decision (DoD #4)

Команда:

```
PYTHONPATH=src/rob_box_voice:src/rob_box_core:src/rob_box_llm:src/rob_box_perception_msgs \
  pytest -v \
    src/rob_box_voice/test/unit/pregen/test_pregen_quality.py \
    src/rob_box_voice/test/unit/pregen/test_pregen_estimator.py \
    src/rob_box_voice/test/unit/pregen/test_pregen_decision.py
```

```
============================= 39 passed in 0.34s ==============================
```

Покрывают (по docstring в conftest):

* `test_pregen_quality.py` — RMS/duration/silence пороги (silent, clipped, trimmed, valid)
* `test_pregen_estimator.py` — cold start, calibration, drift detector, NaN guard
* `test_pregen_decision.py` — pass/fail composition, confidence floor, picks_best, truth table (8 параметризованных кейсов)

**Вердикт по качеству: НЕЙТРАЛЬНО — 39/39 тестов зелёные, регрессии нет.
Quality/estimator/decision verdict выбирается корректно на тестовом наборе.**

## 3. Acceptance criteria (issue #2003)

| DoD-пункт | Результат |
|-----------|-----------|
| #1 baseline латентности | есть, таблица выше |
| #2 raw-цифры после speculative | есть, таблица выше |
| #3 raw-вывод в issue (не полные логи) | этот файл + комментарий в #2003 |
| #4 quality/estimator verdict работает | 39/39 PASSED |
| #5 дельта отрицательная или качество просело → finding | НЕ применимо: дельта положительная, регрессии нет |

## 4. Ссылки

* PR: https://github.com/krikz/rob_box_project/pull/2414
* pytest markers commit: 5971665e
* тесты, использованные как эталоны:
  - `src/rob_box_voice/test/unit/pregen/test_speculative_path_latency.py`
  - `src/rob_box_voice/test/unit/pregen/test_pregen_quality.py`
  - `src/rob_box_voice/test/unit/pregen/test_pregen_estimator.py`
  - `src/rob_box_voice/test/unit/pregen/test_pregen_decision.py`

raw-вывод pytest сохранён в `/tmp/latency_run.log`, `/tmp/latency_3runs.log`,
`/tmp/latency_3runs_full.log`, `/tmp/quality_run.log` (вне репо).