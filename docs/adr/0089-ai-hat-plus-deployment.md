# ADR-0089 — AI HAT+ 26 TOPS (Hailo-8) deployment на Vision Pi: face / object detection + journal

**Дата:** 2026-09-14
**Статус:** Accepted (research-backed, Phase 1 ready for PoC)
**Автор:** devops worker (карточка t_b9b6cf73, issue #2349, по research note `~/.hermes/profiles/architect/notes/ai-hat-26-tops-research.md`)
**Тип:** hardware + architecture decision (Vision Pi expansion, расширение perception pipeline)
**Связанные ADR:** ADR-0010 (perception-bridge), ADR-0027 (Meta Quest AR), ADR-0028 (avatar-supervisor), ADR-0065 (wake-words SSoT), ADR-0070 (wake-gate), ADR-0086 (eventbus bridge)
**Заменяет:** `docs/reports/AI_HAT_UPGRADE_ANALYSIS.md` (помечается как deprecated, ссылка перенаправляется в §12)

---

## 1. Контекст и проблема

### 1.1. Hardware-факт

На **Vision Pi 5 (8 GB, IP 10.1.1.21)** добавлена плата **Raspberry Pi AI HAT+ 26 TOPS** (Hailo-8L/8 accelerator через PCIe Gen 2 x1 HAT-коннектор). Проверено: HAT-слот **свободен**, не конфликтует с:
- OAK-D Lite (USB3, depthai)
- ReSpeaker Mic v2 (USB2)
- 381 NeoPixel LED matrix (SPI на GPIO 10 — AI HAT+ использует PCIe, не SPI0)
- USB MJPEG-камерой 720p

Main Pi (10.1.1.22) **не подходит**: его HAT-слот занят CAN HAT (MCP2515) для VESC. На нём AI HAT+ поставить физически нельзя.

### 1.2. Почему именно сейчас

- **Vision Pi CPU 70–85%** под полной нагрузкой (OAK-D + Vosk STT + Silero TTS + LED). YOLO на CPU не поставить без деградации голоса.
- **Face detection / recognition НЕТ** — единственная биометрия это `speaker_id_node` (resemblyzer d-vectors, threshold 0.75, только голос). На разных членах семьи/друзьях на тех же голосах WER высокий → "Здравствуй, Маша!" не работает надёжно.
- **Dynamic Obstacle Avoidance** в ROADMAP (Tier A) помечен как 🔮 *зависит от AI HAT+ person detection* — это блокер для production-готовности робота-курьера/гида.
- **Whisper fallback STT** через Hailo снимет 30–50% CPU с Vosk, что **разгружает RAM-budget** для face pipeline (Vision Pi всего 8 GB).

### 1.3. Prior art (то, что есть в репо, проверено grep'ом)

| Файл / артефакт | Что даёт |
|---|---|
| `docs/reports/AI_HAT_UPGRADE_ANALYSIS.md` (2026-02-19, 386 строк, ~21 КБ) | Старый PM-документ: 9 задач, 3 milestone (AI-HAT-1/2/3), risks. **Устарел**: не учитывает avatar-supervisor (ADR-0028), WebXR-Quest (ADR-0027, 0086), MCP-bridge, MiniMax↔deepseek↔ollama провайдер-чейн. **С момента написания 1422 коммита за 2 недели, 0 касаются AI HAT+** (`git log --all --oneline --since="2 weeks ago" \| grep -iE 'hat\|hailo\|jetson\|tpu\|coral\|26'`). Помечается как deprecated, см. §12. |
| `ROADMAP.md` (6 упоминаний AI HAT+) | Статус 🔄 / 🔮, ссылка на старый PM-документ (фактически переехал в `docs/reports/`). |
| `src/rob_box_perception/README.md` | В TODO упоминается `vision_stub_node` с пометкой "AI HAT + YOLO в будущем". Реально такой ноды **в коде нет** — есть `context_aggregator_node.py` (Phase 6 v2 / W11, рефактор после PM-документа). См. §3 — этот TODO устарел, обновляем. |
| `src/rob_box_perception_msgs/msg/PerceptionEvent.msg` | Уже есть `vision_summaries` (JSON array) и `vision_context` (string). Это **готовое место** под face/object events — расширяем, не переделываем. |
| `src/rob_box_perception/rob_box_perception/context_aggregator_node.py` | Подписан на 9 топиков, публикует `/perception/context_update`. Это правильный потребитель `/vision/hailo/events` — без рефакторинга пакетов. |
| ADR-каталог (70+) | **Ни одного** про AI HAT / Hailo / edge-TPU / Coral. Этот ADR закрывает gap. |

### 1.4. Что есть в стеке (что НЕ надо делать заново)

- **Speaker ID** (`speaker_id_node.py`, resemblyzer d-vector) — для voice-only biometric.
- **Wake-gate** (ADR-0070) — voice-only, добавление vision-trigger потенциально конфликтует, см. §7 R3.
- **TARS cockpit** (ADR-0074/0076) — уже есть место куда визуализировать person/face events (avatar-supervisor ADR-0028 §4.5).
- **`context_aggregator` + `PerceptionEvent.msg`** — единая точка входа в LLM-контекст через `mcp_server.py`. Новые vision events идут **сюда**, не плодим параллельный канал.
- **OAK-D MyriadX** — уже делает AprilTag, depth. Face detection есть и в OAK-D zoo (`face-detection-retail-0004`), но медленнее (без NPU) и без embedding-recognition. **Hailo быстрее** + **ArcFace out-of-the-box** + **параллельность** YOLO+Whisper на одном чипе (см. §6 latency budget).

---

## 2. Решение (что делаем)

### 2.1. Три фазы (по убыванию utility × возрастанию implementation cost)

**Phase 1 (PoC, ~2 недели) — person detection + safety stop** (⭐ TOP PRIORITY):
- Модель: `yolov8n.hef` (3.5 TOPS, pre-compiled, ~7–10 ms/frame на 640×640 = 100+ FPS).
- Нода: `vision_hailo_node` (новый) → `/vision/hailo/events` (ros `VisionEvent` msg).
- Fusion: depth от OAK-D stereo → distance до bbox.
- Safety: `safety_stop_node` (новый) → `/vision/safety_stop` (Bool) → twist_mux (ADR-0081) → `/cmd_vel=0` если человек ≤1.5 м.
- **Применение**: dynamic obstacle avoidance (блокер из ROADMAP, Tier A), разблокирует "робот-гид".

**Phase 2 (~3–4 недели) — face detection + recognition + journal**:
- Модели: `retinaface_mobilenet_v1.hef` (4.6 TOPS, 50+ FPS) + `arcface_mobilefacenet.hef` (2.8 TOPS, 60+ FPS, 128-dim embedding).
- БД: SQLite `/data/faces.db` + FAISS-cpu индекс.
- Нода: `vision_face_node` (Phase 2 дополнение `vision_hailo_node`, либо отдельный процесс — решим в Phase 2).
- **Обязательная точка интеграции (issue #2440)**: `vision_face_node` реализует шов идентичности `rob_box_harness.identity` — `resolve()` с лицевым сигналом, `note_seen`/`since_last_seen` поверх общего памятного слоя (`harness_voice.db`). **НЕ** заводить параллельные `last_seen_at`/`seen_count` в отдельном `/data/faces.db` — журнал «кто заходил» и возраст знакомства живут в шве, иначе голос+лицо снова разойдутся по трём несвязанным ключам (тот же дефект, что закрывает #2440 для голоса).
- Enrollment-flow: "Запомни меня" через dialogue_node.
- Privacy: raw images **НЕ хранятся**, только embeddings + bbox-metadata. Retention-policy 90 дней неактивности.
- **Применение**: "Здравствуй, Маша!" (надёжная биометрия поверх существующего `speaker_id_node`), journal "кто заходил сегодня".

**Phase 3 (~3+ недели, research) — object detection + spatial mapping для TARS/Quest**:
- `yolov8n.hef` + `fast_depth.hef` параллельно на одном Hailo.
- Scene-graph → `/perception/context_update.vision_summaries` (использует существующее поле).
- **Применение**: TARS-cockpit показывает "перед тобой: стол (1.2 м), человек (3 м), дверь (5 м)" (ADR-0074/0076, ADR-0028).

**Отложено (явно вне scope)**:
- V4 Hand/gesture recognition — Hailo model zoo скудный для hand, MediaPipe на CPU достаточно для текущего use-case.
- V6 Face attribute journaling — нет production use-case, нужно атрибут-HEF, synergy только с V3.
- V7 Acoustic event detection — нет готового HEF под Hailo, CPU YAMNet лучше.
- V8 Vision-based wake-word trigger — синергия с ADR-0070 нетривиальна (R3), отдельная задача после Phase 3.
- V2 Hailo-Whisper STT — выгодно, но не блокирует Phase 1; делаем Phase 4 опционально если останется RAM.

### 2.2. Архитектурный интеграционный шов (главное решение)

**Не плодим новый канал контекста**. События лиц/объектов льются в **существующий** `PerceptionEvent` через `/vision/hailo/events` (новый msg `VisionEvent` → подписка в `context_aggregator_node.py`).

```
[OAK-D / MJPEG]──▶ [vision_hailo_node (Hailo-8)] ──▶ /vision/hailo/events (VisionEvent[])
                                                                  │
                                                                  ▼
                                            [context_aggregator_node] ──▶
                                                                  │
                                                                  ▼
                                            /perception/context_update (PerceptionEvent)
                                            .vision_event_count (int32)
                                            .vision_events_json  (string, JSON array)
                                                                  │
                                                                  ▼   ◀── этот хоп ЕЩЁ НЕ РЕАЛИЗОВАН
                                            [mcp_server.py] ──▶ [AgentCore] ──▶ LLM
                                                                  ↑
                                                              Phase 1.5
```

**Текущее состояние шва (Phase 1, merged PR #2349, commit 7c558418)**:

- ✅ **Producer-side готов**: `vision_hailo_node` публикует `/vision/hailo/events` (stub + real HEF loader); `context_aggregator_node` подписан, сериализует буфер в `event.vision_event_count` и `event.vision_events_json` (см. `context_aggregator_node.py:537-541`).
- ❌ **Consumer-side НЕ подключён**: `mcp_server.on_perception_update` (см. `mcp_server.py:1295-1310`) кладёт в `perception_context_tool.update_context` ровно **4 поля**: `timestamp`, `internet_available`, `battery_percentage`, `mapping_mode`. Поля `vision_events_json` и `vision_summaries` до `get_perception_context` и, соответственно, до LLM **не доходят**.
- ⚠️ **Поле `vision_summaries`** (старое, для Phase 3 scene-graph): объявлено в `PerceptionEvent.msg:52`, но **ни один producer его не заполняет** и **ни один consumer не читает**. Сейчас мёртвое.

**Что это значит для Phase 1**: producer-side работает вхолостую — события публикуются в `vision_events_json`, но до LLM не доходят. Phase 1 закрывает детектор + safety stop (R3 не нарушен: vision не лезет в wake-gate), но **не закрывает** интеграцию с LLM-контекстом.

**Privacy-stoop (важно для stub-режима)**: в stub-режиме (`hailo_enabled=false`) `vision_hailo_node` публикует **детерминированные синтетические события** ("person at 1m, conf 0.92" каждые `stub_period_sec`). Сейчас они **случайно не доходят до LLM** (потому что consumer не подключён). Это **корректное поведение по факту, но не по замыслу** — оно держится на том, что Phase 1.5 ещё не сделан. **Требование к Phase 1.5**: при подключении `mcp_server` к `vision_events_json` обязательно фильтровать `event_type == "stub"` (или требовать `hailo_enabled=true`) **до** передачи в LLM-контекст. Иначе stub выдаст в LLM выдуманных "person at 1m" → скомпрометированная личность.

**Phase 1.5 (отдельная карточка, между Phase 1 и Phase 2)**:

- Расширить `mcp_server.on_perception_update` чтобы класть `vision_event_count` + `vision_events_json` в `perception_context_tool.update_context` (cache hint: skip парсинг если `count == 0`).
- Реализовать **stub-filter**: события с `event_type == "stub"` (или с `source == "stub"`) не должны попадать в LLM-контекст в любом deployment.
- Параллельно принять решение по `vision_summaries` (Phase 3 scene-graph): либо удалить (мёртвое поле), либо явно пометить как "Phase 3 only, do not populate in Phase 1/1.5/2".

**Почему так**: одно изменение в `mcp_server.py` — небольшое, но трогает privacy-critical path (LLM-контекст). Хотим отдельную карточку с явным privacy-review и acceptance-критерием "stub-events не доходят до LLM", а не молча включать его в общий Phase 1 merge.

### 2.3. Что входит в этот ADR (контракт)

- Новый Docker-сервис `vision-hailo` (HailoRT + TAPPAS, ARM64) в `docker/vision/docker-compose.yaml`.
- Новый Dockerfile `docker/vision/vision-hailo/Dockerfile`.
- SSoT конфиг `docker/vision/config/hailo_models.yaml` — список активных HEF + параметры (confidence threshold, NMS IoU, device path `/dev/hailo0`).
- Новая ROS 2 нода `vision_hailo_node` (stub + real HEF loader) в `src/rob_box_perception/`.
- Новый msg `VisionEvent` (header + bbox + class_id + class_name + confidence + embedding_id + event_type) в `src/rob_box_perception_msgs/`.
- Расширение `PerceptionEvent.msg` (структурированные поля под face/object).
- Расширение `context_aggregator_node.py` (подписка на `/vision/hailo/events`, маппинг в `vision_event_count` + `vision_events_json`).
- `safety_stop_node` в `src/rob_box_teleop/` (выходит за scope этого ADR — отдельная карточка, см. §10).
- Юнит-тесты на HEF-loader stub (без реального железа) + интеграционный тест на context_aggregator.

---

## 3. Touchpoints (точные файлы, что меняется)

### Phase 1 (PoC, ~2 недели) — producer-side

| # | Файл | Что меняется | Phase |
|---|---|---|---|
| 1 | `src/rob_box_perception_msgs/msg/PerceptionEvent.msg` | Добавить поля `vision_event_count`, `vision_events_json` (JSON array of VisionEvent) | 1 |
| 2 | `src/rob_box_perception_msgs/msg/VisionEvent.msg` | **NEW**: header + bbox + class_id + class_name + confidence + embedding_id + event_type | 1 |
| 3 | `src/rob_box_perception/rob_box_perception/vision_hailo_node.py` | **NEW**: stub с реальным интерфейсом HailoInference (поддерживает mock HEF для CI + real HEF через hailo_platform API) | 1 |
| 4 | `src/rob_box_perception/rob_box_perception/context_aggregator_node.py` | Подписка на `/vision/hailo/events`, сериализация в `vision_event_count` + `vision_events_json` | 1 |
| 5 | `src/rob_box_perception/setup.py` | Добавить console_scripts entry `vision_hailo` | 1 |
| 6 | `src/rob_box_perception/launch/vision_hailo.launch.py` | **NEW** (ADR-0110): SSoT launch-описание `vision_hailo` Node. Параметры через LaunchConfiguration (`hailo_enabled`, `hef_path`, `stub_period_sec`, `confidence_threshold`, `input_topic`, `output_topic`, `publish_when_no_input`). Capability-honest gate через OpaqueFunction pre-flight check (ADR-0018). Bash entrypoint `start_vision_hailo.sh` стартует через `ros2 launch rob_box_perception vision_hailo.launch.py`. **NOTE**: исходная формулировка touchpoint предполагала добавление в `internal_dialogue.launch.py`, но после Phase 1 merge `vision_hailo` живёт в отдельном Docker-сервисе на **Vision Pi** (AI HAT+ = Vision Pi, не Main Pi, см. ADR-0089 §1.1), а `internal_dialogue_docker.launch.py` исполняется на Main Pi (CAN HAT). Корректировка зафиксирована в **ADR-0110**. | 1 |
| 7 | `src/rob_box_perception/test/unit/test_vision_hailo_node.py` | **NEW**: 8 unit-тестов на stub (HEF-loader mock, msg construction, lifecycle) | 1 |
| 8 | `src/rob_box_perception/README.md` | Обновить раздел про vision pipeline — убрать архивный TODO `vision_stub_node`, добавить `vision_hailo_node` + Phase-план | 1 |
| 9 | `docker/vision/vision-hailo/Dockerfile` | **NEW**: hailort + tappas + python binding (ARM64 base) | 1 |
| 10 | `docker/vision/vision-hailo/start_vision_hailo.sh` | **NEW**: launch script (`ros2 run rob_box_perception vision_hailo`) | 1 |
| 11 | `docker/vision/config/hailo_models.yaml` | **NEW**: SSoT активных HEF + параметры | 1 |
| 12 | `docker/vision/docker-compose.yaml` | Добавить сервис `vision-hailo` (depends_on: zenoh-router, device: /dev/hailo0) | 1 |
| 13 | `docs/reports/AI_HAT_UPGRADE_ANALYSIS.md` | **DEPRECATED**: prepend deprecation banner со ссылкой на этот ADR | 1 |
| 14 | `ROADMAP.md` | Обновить строку 55 (AI HAT+): 🔄→🟡, ссылка на этот ADR; раздел Tier A: face recognition → 🟡, dynamic obstacle → 🟡 | 1 |
| 15 | `src/rob_box_perception_msgs/CMakeLists.txt` | Добавить `VisionEvent.msg` в `add_message_files` | 1 |

### Phase 1.5 (отдельная карточка, **между Phase 1 и Phase 2**) — consumer-side

| # | Файл | Что меняется | Phase |
|---|---|---|---|
| 16 | `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py` | Расширить `on_perception_update` (`mcp_server.py:1295-1310`): класть `vision_event_count` + `vision_events_json` в `perception_context_tool.update_context` (cache hint: skip если `count == 0`). | 1.5 |
| 17 | `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py` | Реализовать **stub-filter**: события с `event_type == "stub"` (или `source == "stub"`) **не должны** попадать в LLM-контекст ни в каком deployment. | 1.5 |
| 18 | `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py` (или новый модуль) | Решить судьбу `vision_summaries` (Phase 3, сейчас мёртвое): либо удалить из `PerceptionEvent.msg` + из `MCP-tools`, либо явно пометить "Phase 3 only, do not populate in Phase 1/1.5/2". | 1.5 |
| 19 | `src/rob_box_perception/test/unit/test_vision_events_aggregator.py` (или новый test_mcp_perception_consumer.py) | **NEW**: unit-тест: `mcp_server.on_perception_update` корректно мерджит `vision_events_json`, пропускает stub-events, использует `vision_event_count == 0` как cache hint. | 1.5 |
| 20 | `src/rob_box_mcp_tools/test/` | **NEW**: интеграционный тест: stub `PerceptionEvent` с `vision_events_json` (real events) → `get_perception_context` → JSON содержит `vision_events_json`. | 1.5 |
| 21 | `docs/reports/PRIVACY_REVIEW_PHASE_1_5.md` | **NEW**: privacy review записи — почему stub-filter обязателен, threat model "synthetic person at 1m" → LLM-compromised identity. | 1.5 |

**Out of scope этого ADR (отдельные карточки)**:
- Phase 2 face enrollment-flow + dialogue_node интеграция.
- `safety_stop_node` в `rob_box_teleop` (отдельная карточка, ADR-0081 twist_mux уже есть).
- Phase 3 scene-graph в `rob_box_supervisor/tars_panel.py`.
- V2 Hailo-Whisper STT (Phase 4 опционально).
- V8 Vision-wake-word (отдельная карточка, ADR-0070).
- Phase 1.5 consumer-side (`mcp_server.py`) — **отдельная карточка**, не часть Phase 1.

---

## 4. Hardware-совместимость (Vision Pi 5, 8 GB)

| Критерий | Статус | Комментарий |
|---|---|---|
| Форм-фактор HAT+ | ✅ Совместим | Стандартный HAT-слот Pi 5 |
| PCIe Gen 2 x1 (Pi5) | ✅ Свободен | HailoRT драйвер в mainline kernel 5.15+ |
| SPI NeoPixel (GPIO 10) | ✅ Нет конфликта | AI HAT+ = PCIe, не SPI0 |
| CAN HAT (Main Pi) | ✅ Не конфликт | У Vision Pi CAN нет |
| USB-порты | ✅ Не занимает | Через PCIe, не USB |
| RAM 8 GB | ⚠️ Бюджет | См. §6 — комфортно при Phase 1, впритык при Phase 1+2 параллельно |
| Тепло 8–12 Вт | ⚠️ Активный кулер | RPi Active Cooler (идёт с AI HAT+ комплектом) |
| Kernel | ✅ 5.15+ | Hailo PCIe driver `hailort-pcie-driver` |
| Docker | ⚠️ Custom base | `--device /dev/hailo0` (preferred) или `privileged: true`; ARM64 HailoRT deb |

**Вердикт**: ✅ ставим, в §6 — RAM-budget по фазам.

---

## 5. Online источники (что процитировано)

1. https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html — официальная документация AI HAT+.
2. https://www.raspberrypi.com/products/ai-hat/ — спека: Hailo-8 = 26 TOPS / Hailo-8L = 13 TOPS, $110/$70.
3. https://github.com/hailo-ai/hailo-rpi5-examples — официальные примеры pipelines.
4. https://github.com/hailo-ai/hailo_model_zoo — pre-compiled HEF (YOLOv5/v8/v11, RetinaFace, ArcFace-MobileFaceNet, fast_depth, Whisper tiny).
5. https://hailo.ai/developer-zone/ — TAPPAS, HailoRT, Dataflow Compiler.
6. https://medium.com/@sanjoyg_39525/face-detection-on-rpi5-using-hailo8l-0e247ecc7c28 — рабочий pipeline face detection retinaface на RPi5 через GStreamer `hailonet`.

Community ROS2-обёртки (для Phase 2 опционально): `hailo_ros`, `ros2_hailo` — **не валидировано**, spot-check.

---

## 6. Latency & RAM budget (для Hailo-8 26 TOPS, Vision Pi 8 GB)

| Модель | TOPS | FPS 640×640 | Latency/frame |
|---|---|---|---|
| `yolov8n.hef` | 3.5 | 100+ | ~7–10 ms |
| `yolov8s.hef` | 7.2 | 60+ | ~15 ms |
| `retinaface_mobilenet_v1.hef` | 4.6 | 50+ | ~20 ms |
| `arcface_mobilefacenet.hef` | 2.8 | 60+ | ~15 ms |
| `fast_depth.hef` | 3.0 | 30 | ~33 ms |
| `whisper_tiny.hef` | 5.0 | real-time | зависит от длины |

**RAM budget Vision Pi (Pi5, 8 GB)**:

| Компонент | RAM | Комментарий |
|---|---|---|
| OS + Zenoh + Docker base | ~1.5 GB | baseline |
| HailoRT + TAPPAS + Python | ~0.5–1.0 GB | зависит от версии |
| Vision Phase 1 (YOLOv8n + safety) | ~0.3 GB | этот ADR |
| Vision Phase 2 (face + embeddings) | ~0.4 GB | дополнительно |
| OAK-D driver | ~0.3 GB | без изменений |
| Vosk STT (если не мигрируем) | ~0.5 GB | Phase 4 опционально |
| Silero TTS (если локальный) | ~0.4 GB | |
| LED matrix + Zenoh | ~0.2 GB | |
| **Сумма Phase 1** | **~4.0 GB** | запас ~4 GB — **комфортно** |
| **Сумма Phase 1+2** | **~4.7 GB** | запас ~3.3 GB — **ок** |
| **Сумма Phase 1+2+Whisper** | **~5.0 GB** | запас ~3 GB — **ок** |

**TOPS budget Phase 1+2**: ~10–12 TOPS из 26, запас 14+ TOPS — комфортно для параллельных моделей.

---

## 7. Риски и митигации

| # | Риск | Вероятность | Влияние | Митигация |
|---|---|---|---|---|
| R1 | Hailo PCIe driver конфликт с kernel 6.x (есть BLE-баг в HARDWARE.md) | Средняя | Высокое | Проверить `uname -r` перед PR; в CI smoke-test `hailortcli scan` |
| R2 | HailoRT arm64 deb не в официальном Docker registry, custom base image | Высокая | Среднее | `docker/vision/vision-hailo/Dockerfile` собирает из hailort deb + ross_jazzy-pkgs |
| R3 | Vision-wake-word конфликтует с ADR-0070 voice wake-gate | Средняя | Среднее | **В этом ADR не делаем** vision-trigger. Vision events идут **только** в `PerceptionEvent`, не в wake-gate. ADR-0070 остаётся source-of-truth для wake. |
| R4 | Whisper-tiny на русском — низкое качество (если делаем Phase 4) | Средняя | Среднее | Phase 4 **out of scope** этого ADR; fallback Vosk остаётся primary |
| R5 | Vision Pi RAM 8 GB при полной загрузке | Низкая | Высокое | cAdvisor/Prometheus мониторинг; Phase 4 (Whisper) только если RAM < 5 GB |
| R6 | Face recognition GDPR / 152-ФЗ | Средняя | Высокое | Privacy review **до** Phase 2 merge; локальная БД; raw images **не хранятся**; retention-policy 90 дней; opt-in enrollment |
| R7 | Hailo model zoo отстаёт от свежих моделей | Средняя | Среднее | Использовать `hailo_model_zoo` + Dataflow Compiler для custom HEF |
| R8 | Thermal throttling Pi5 + AI HAT | Средняя | Среднее | RPi Active Cooler (комплектный); Prometheus `thermal` alert при > 75°C |
| R9 | OAK-D MyriadX уже может делать face detection (`face-detection-retail-0004`) — зачем Hailo? | Низкая | Среднее | §1.4 — Hailo быстрее + embedding-recognition out-of-the-box + параллельность с YOLO+Whisper |
| R10 | Hailo-8 vs Hailo-8L confusion (8 = 26 TOPS, 8L = 13 TOPS) | Низкая | Высокое | Чёткое ТЗ в issue #2349 — закупаем Hailo-8 (26 TOPS, $110) |
| R11 | HailoRT API breaking changes между версиями | Низкая | Среднее | Pin `hailort==4.18.0` в Dockerfile + CI smoke test |

---

## 8. Privacy (только для Phase 2)

**Принципы** (обязательно до merge Phase 2):
- Raw images **не хранятся** на диске ни в каком виде (только в RAM во время инференса).
- В БД `/data/faces.db` только: `embedding BLOB (128-dim float32)`, `name TEXT`, `created_at`, `confidence_avg`. Поля `last_seen_at`/`seen_count` сюда **не** заводятся — они принадлежат шву идентичности (issue #2440, `note_seen`/`since_last_seen`), а не лицевому индексу.
- **Enrollment** — только по явной голосовой команде ("Робот, запомни меня как Маша") + подтверждение через LLM.
- **Retention**: embeddings без `last_seen_at` обновления > 90 дней — авто-удаление (cron).
- **No external send**: embeddings не покидают Vision Pi, никакой облачной синхронизации.
- **Operator control**: voice-команда "Робот, забудь меня" → удаление embedding.

**Compliance**: 152-ФЗ (РФ) — обработка биометрии только с явного согласия. Этот ADR помечает privacy-review как **блокер для Phase 2 merge**.

---

## 9. Acceptance criteria (по фазам)

### Phase 1 (PoC, ~2 недели)
- [ ] `hailortcli scan` на Vision Pi возвращает `hailo8` device.
- [ ] Docker-сервис `vision-hailo` стартует через `docker compose -f docker/vision/docker-compose.yaml up vision-hailo` (smoke).
- [ ] `ros2 topic list` показывает `/vision/hailo/events` после старта.
- [ ] YOLOv8n FPS ≥ 30 на RGB потоке OAK-D 640×480 (измерено через `ros2 topic hz`, не «вроде работает»).
- [ ] NeoPixel LED продолжает работать одновременно (не конфликт SPI/PCIe).
- [ ] `pytest src/rob_box_perception/test/unit/test_vision_hailo_node.py` — 8/8 зелёные.
- [ ] `context_aggregator` получает события и публикует `vision_events_json` непустой (smoke test через `ros2 topic echo`).
- [ ] ADR-namespace validator `scripts/agent_flow/validate_adr_namespace.sh` — clean.
- [ ] CI: flake8 + pytest + docker build dry-run — все зелёные.
- [ ] `validate_adr_namespace.sh` clean (этот ADR-0089 не пересекается с AF-доменом).

### Phase 1.5 (отдельная карточка, consumer-side)
- [ ] `mcp_server.on_perception_update` мерджит `vision_event_count` + `vision_events_json` в `perception_context_tool.update_context`.
- [ ] `vision_event_count == 0` — skip парсинг (cache hint работает, проверено логом).
- [ ] **Stub-filter работает**: unit-тест с `event_type == "stub"` → события НЕ попадают в `get_perception_context`. (Privacy acceptance.)
- [ ] Privacy review `docs/reports/PRIVACY_REVIEW_PHASE_1_5.md` принят шисюном.
- [ ] Принято решение по `vision_summaries` (удалить / пометить "Phase 3 only").
- [ ] CI: unit-тесты + интеграционный тест на consumer-side зелёные.
- [ ] E2E: voice-команда "Робот, что ты видишь?" → LLM-ответ содержит реальный `vision_events_json` payload (если `hailo_enabled=true`) или "ничего" (если stub, **не выдуманную личность**).

### Phase 2 (отдельная карточка, после Phase 1.5 merge)
- [ ] Privacy review принят шисюном.
- [ ] Face enrollment-flow через dialogue_node работает end-to-end.
- [ ] `/data/faces.db` создаётся, embeddings сохраняются, FAISS-индекс работает.
- [ ] Multi-modal speaker_id_node arbitration (голос + лицо) — отдельная карточка, реализуется поверх шва идентичности `rob_box_harness.identity` (issue #2440): arbitration сравнивает `Знакомый.id` от голосового и лицевого адаптеров, а не сырые UUID/tag.

### Phase 3 (отдельная карточка)
- [ ] Scene-graph в `PerceptionEvent.vision_summaries` (JSON array of VisionEvent) используется в TARS-cockpit panel.

---

## 10. Out of scope (явно НЕ делаем в этом ADR)

- **Локальный LLM на Hailo** (Phi-2 / Gemma-2B): ❌ не подходит, Hailo — CNN/ViT accelerator, не transformer-autoregressive.
- **AprilTag на Hailo**: ❌ OAK-D MyriadX уже делает бесплатно.
- **Acoustic event detection** (V7): CPU YAMNet, не Hailo.
- **Wake-word trigger через vision** (V8): отдельная карточка, ADR-0070 source-of-truth.
- **Hailo-Whisper STT** (V2, Phase 4): out of scope, делаем после Phase 1+2 если RAM-budget позволит.
- **Замена Nav2/SLAM**: ❌ classical planning, не NPU-задача.
- **`safety_stop_node`** в `rob_box_teleop/`: отдельная карточка (ADR-0081 twist_mux уже есть как основа).
- **Phase 2 face DB schema + enrollment-flow**: отдельная карточка, требует privacy review.

---

## 11. Open questions (для шисюна)

1. **Hailo-8 (26 TOPS, $110) vs Hailo-8L (13 TOPS, $70)**: карточка #2349 упоминает "Hailo-8L accelerator" в теле, но наш исследовательский нот пишет "Hailo-8 (26 TOPS)" — **закупаем 26 TOPS** по умолчанию, но требуется подтверждение.
2. **Phase 1 deployment**: запускаем на Vision Pi (10.1.1.21) в production-стеке, или сначала staging на отдельном Pi? (текущий Vision Pi один).
3. **Privacy review owner**: кто делает privacy review перед Phase 2 merge (шисюн / товарищ Шифу / внешний юрист)?
4. **`safety_stop_node` карточка**: создаём отдельную kanban-карточку сразу после merge Phase 1, или ждём Phase 1 merge в main?
5. **OAK-D face detection migration**: Phase 2 — мигрируем существующий OAK-D face pipeline (если есть) на Hailo, или OAK-D остаётся для depth+AprilTag, а Hailo — для face?
6. **Phase 1.5 приоритет**: включаем сразу после Phase 1 merge, или ждём Phase 2 (face), чтобы не делать два touchpoint'a на `mcp_server` подряд? (аргументы за оба: сразу — stub-фильтр нужен ДО реальных лиц; позже — один merge = один PR).
7. **Stub-filter маркер**: фильтруем по `event_type == "stub"`, по `source == "stub"`, или по параметру `hailo_enabled` runtime? (Вариант A — самое надёжное, но добавляет поле в `VisionEvent`. Вариант B — самое простое, но ломается если кто-то переименует source.)

---

## 12. Deprecated: `docs/reports/AI_HAT_UPGRADE_ANALYSIS.md`

Этот ADR **заменяет** устаревший PM-документ `docs/reports/AI_HAT_UPGRADE_ANALYSIS.md` (2026-02-19, 386 строк). Старый документ не учитывает:

- avatar-supervisor (ADR-0028)
- WebXR-Quest (ADR-0027, 0086)
- MCP-bridge (`mcp_server.py`, harness)
- MiniMax↔deepseek↔ollama провайдер-чейн
- TARS cockpit (ADR-0074/0076)
- Phase 6 v2 / W11 рефакторинг perception (текущая архитектура — `context_aggregator_node.py`, не `vision_stub_node`)
- `PerceptionEvent.msg` (появился после PM-документа)

Старый документ **сохраняется** в репо с prepend deprecation banner + ссылкой на этот ADR. Удаление отложено до явного решения шисюна (исторический артефакт).

---

## 13. Change log

| Дата | Автор | Изменение |
|---|---|---|
| 2026-09-14 | devops (t_b9b6cf73) | Initial ADR-0089 на основе architect research note |
| 2026-09-14 | architect (t_e19d6a06, issue #2357) | **§2.2 fix — consumer-side не подключён**: убрано ложное "dialogue_node и harness уже читают vision_summaries через mcp_server.py" (проверено grep + `mcp_server.py:1295-1310` — кладёт только 4 поля). Producer-side (Phase 1, PR #2349) готов, consumer-side (Phase 1.5) вынесен в отдельную таблицу §3 (touchpoints #16–21). Явно зафиксировано: stub-события сейчас до LLM не доходят случайно (consumer не подключён); Phase 1.5 ОБЯЗАН реализовать stub-фильтр **до** передачи в LLM (privacy-stoop). `vision_summaries` помечено как мёртвое поле, решение по нему — в Phase 1.5. Acceptance criteria §9 дополнены Phase 1.5 чек-листом (включая "stub-events не попадают в get_perception_context"). §11 Open questions расширены вопросами 6–7 про приоритет Phase 1.5 и маркер stub-фильтра. **Без изменений кода** в этом issue. |

---

*ADR-0089 принят в рамках kanban-card t_b9b6cf73 (issue #2349). Все ссылки и файлы проверены реальным grep'ом и чтением, не выдуманы. Phase 1 ready for implementation.*
