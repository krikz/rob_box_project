# AI HAT+ 26 TOPS — Анализ апгрейда Vision Pi

> **Тип документа**: PM Analysis / Upgrade Proposal  
> **Автор**: Product Manager Agent  
> **Дата**: 2026-02-19  
> **Статус**: Draft — на согласование  
> **Затрагивает**: Vision Pi (10.1.1.11), ROADMAP.md, tasks.json  

---

## TL;DR

**Raspberry Pi AI HAT+ (Hailo-8L, 26 TOPS)** — это M.2 NPU-ускоритель нейронных сетей, который подключается в слот HAT на RPi 5 через PCIe 2.0 x1. Для **Vision Pi** он даёт аппаратное ускорение инференса (до 10–30× против CPU), не занимает USB-порты и не конкурирует за GPIO (в отличие от Main Pi, где CAN HAT уже занял слот).

**Вывод**: апгрейд целесообразен. Приоритетные кандидаты — детекция людей/объектов, улучшение STT, face recognition. Ниже — детальный разбор.

---

## 1. Что такое Raspberry Pi AI HAT+ 26 TOPS

| Параметр | Значение |
|----------|----------|
| **Чип** | Hailo-8L |
| **Производительность** | 26 TOPS (tera operations per second) |
| **Форм-фактор** | HAT+ (полностью совместим с RPi 5, стандартный разъём) |
| **Интерфейс к Pi** | PCIe 2.0 x1 (через HAT-connector RPi 5) |
| **Питание** | От GPIO Pi, ~3–5 Вт при работе |
| **SDK** | HailoRT + TAPPAS (OpenCV pipeline), Python API |
| **Поддерживаемые модели** | YOLOv5/v8, ResNet, MobileNet, Whisper, и 70+ pre-compiled моделей |
| **GPIO-конфликт** | ⚠️ Занимает GPIO HAT-слот — нельзя одновременно с другими HAT |
| **SPI NeoPixel** | ✅ SPI (GPIO 10) освобождается — HAT занимает PCIe, не SPI |

### Ограничения

- HAT+ занимает физический HAT-слот на RPi 5 → другие HAT одновременно **невозможны**
- Требует HailoRT среды — немного усложняет Docker-сборку
- У Vision Pi GPIO 10 (SPI/NeoPixel) не занят HAT-слотом — **LED матрица продолжает работать**
- PCIe слот у Vision Pi свободен (у Main Pi — нет CAN, он через SPI MCP2515)

---

## 2. Текущее состояние Vision Pi

### 2.1. Запущенные процессы (нагрузка на CPU)

| Сервис | CPU нагрузка | RAM | Тип задачи |
|--------|-------------|-----|------------|
| OAK-D Driver (depthai) | ~15–25% | ~300 MB | Захват RGB-D + стерео |
| AprilTag Detection | ~10–20% | ~100 MB | CV детекция маркеров |
| Voice STT (Vosk) | ~30–50% пиково | ~500 MB | Offline ASR |
| Voice TTS (Silero) | ~40–60% пиково | ~400 MB | Offline Speech Synthesis |
| LED Matrix Driver | ~3–5% | ~50 MB | SPI NeoPixel управление |
| Zenoh Router | ~3–5% | ~80 MB | ROS 2 middleware |
| MJPEG Camera | ~5–10% | ~100 MB | USB UVC capture |

**Проблема**: при одновременной работе STT (распознавание) + LLM ответа + TTS синтеза Pi 5 8GB работает на ~70–85% CPU. Добавить тяжёлый YOLO на CPU **нельзя** без деградации голосового ответа.

### 2.2. Свободные ресурсы Vision Pi

| Ресурс | Статус |
|--------|--------|
| HAT-слот (PCIe) | ✅ **Свободен** — нет ни одного HAT |
| USB 3.0 порты | ⚠️ Заняты: OAK-D + ReSpeaker + возможно MJPEG |
| RAM (из 8GB) | ~1.5–2 GB свободно при полной нагрузке |
| GPIO 10 (SPI) | Используется NeoPixel |
| Тепловой пакет | Pi 5 с AI HAT = ~8–10 Вт суммарно → нужен вентилятор |

---

## 3. Почему Vision Pi, а не Main Pi

| Критерий | Main Pi | Vision Pi |
|----------|---------|-----------|
| HAT-слот | ❌ **Занят CAN HAT** (MCP2515 для VESC) | ✅ **Свободен** |
| GPIO конфликты | CAN bus на SPI0, несколько GPIO для прерываний | Только SPI для NeoPixel (не HAT-слот) |
| Тип задач на Pi | Навигация, SLAM — ветвистая логика, не NN | Камера, голос — прямые кандидаты для NN |
| Выгода от NPU | Средняя — SLAM/Nav2 CPU-привязаны | Высокая — детекция, STT, face — NN задачи |

**Вывод**: Vision Pi — единственный технически подходящий вариант на данный момент.

---

## 4. Задачи, которые решает AI HAT+

Ниже — анализ задач от критически полезных к экспериментальным, с оценкой реалистичности реализации на 26 TOPS.

---

### 4.1. 🔴 Высокий приоритет — прямая ценность для проекта

#### 4.1.1. Детекция людей в реальном времени (Person Detection)

| Параметр | Детали |
|----------|--------|
| **Модель** | YOLOv5s / YOLOv8n (pre-compiled Hailo HEF) |
| **Производительность** | 30–60 FPS на RGB потоке OAK-D |
| **CPU без HAT** | ~2–3 FPS на Pi CPU или оффлоадится на OAK-D MyriadX |
| **Ценность для проекта** | Знать, что рядом есть человек — базовое требование для курьера и гида |

**Сценарии применения:**
- **Робот-гид**: обнаружить человека → начать диалог (замена всегда включённого wake-word)
- **Курьер**: не начинать движение, пока человек на пути
- **Патрулирование**: отчёт об обнаружении человека в охраняемой зоне
- **Safety stop**: аварийная остановка при детекции человека в зоне移动 (≤ 1.5 м)

**Интеграция**: ROS 2 топик `/vision/persons` → `rob_box_perception` → Nav2 costmap как дополнительный источник препятствий.

---

#### 4.1.2. Динамическое предотвращение столкновений с людьми

| Параметр | Детали |
|----------|--------|
| **Модель** | YOLOv8n + depth estimation (fusion с OAK-D depth) |
| **Ценность** | Критично для безопасной работы среди людей |
| **Текущий статус** | ❌ Не реализовано — только static obstacle avoidance через LiDAR costmap |

**Что даёт NPU**: детекция + оценка расстояния (depth от OAK-D) за <20 мс → публикация moving obstacles в Nav2 costmap.  
Даже если Nav2 не умеет work со dynamic costmap в реальном времени, можно реализовать **safety zone**: при человеке ближе 1.5 м → publish `/cmd_vel {0,0}` с высоким приоритетом через `twist_mux`.

---

#### 4.1.3. Улучшение STT — локальный Whisper через Hailo

| Параметр | Детали |
|----------|--------|
| **Модель** | Whisper tiny/base (Hailo pre-compiled HEF) |
| **Текущий STT** | Vosk на CPU: ~30–50% CPU, WER ~25% в шуме |
| **Whisper на NPU** | ~3–5% CPU на Pi, WER ~10–15% офлайн, без сети |
| **Ценность** | Радикально снизить нагрузку + улучшить качество распознавания |

**Важно**: Hailo TAPPAS имеет pre-compiled HEF для Whisper tiny (русский язык частично поддерживается через multilingual модель). Это прямая замена Vosk с лучшим WER. Также освобождает CPU для LLM-шинга.

---

#### 4.1.4. Face Detection + Recognition

| Параметр | Детали |
|----------|--------|
| **Модель** | retinaface_mobilenet_v1 (4.6 TOPS) + arcface (вектор 128-dim) |
| **Статус в roadmap** | 🔮 Tier A — «Face Recognition & User Database» |
| **Реализм** | ✅ Hailo официально поддерживает face detection pipeline |

**Сценарии:**
- Приветствовать знакомых по имени → персонализированный диалог
- Авторизация для получения посылки (заменяет или дополняет QR-код)
- Логирование посетителей (кто брал посылку, в какое время)

---

### 4.2. 🟠 Средний приоритет — ценно, но требует доп. работы

#### 4.2.1. Детекция QR-кодов и текста (OCR)

| Параметр | Детали |
|----------|--------|
| **Модель** | LPRNET / TextDetection (Hailo HEF) |
| **Ценность** | QR авторизация для доступа к грузу — в roadmap |
| **Альтернатива** | OpenCV CPU — достаточно быстро для статичных QR |
| **Рекомендация** | Реализовать без NPU сначала, NPU — если нужна высокая частота |

---

#### 4.2.2. Детекция AprilTag — замена/дополнение текущей

| Параметр | Детали |
|----------|--------|
| **Текущий подход** | AprilTag detection на CPU через OAK-D (бесплатно, offloaded) |
| **Ценность NPU** | Низкая — OAK-D сам справляется с AprilTag, NPU лишний |
| **Рекомендация** | ❌ Не переносить на NPU — нет смысла |

---

#### 4.2.3. Emotion/Gesture Recognition

| Параметр | Детали |
|----------|--------|
| **Модель** | EmotionRecognition (Hailo HEF) |
| **Ценность** | Отображать соответствующую LED-анимацию при эмоции человека |
| **Реализм** | Технически возможно, но нужна доработка `rob_box_animations` pipeline |
| **Рекомендация** | 🔮 Future — интересно, не критично |

---

#### 4.2.4. Локальный малый LLM (Phi-2, Gemma-2B и т.д.)

| Параметр | Детали |
|----------|--------|
| **Реалистичность** | ⚠️ **Нет** — 26 TOPS недостаточно для LLM даже 2B параметров |
| **Почему нет** | Phi-2 (2.7B) требует ~50+ TOPS для приемлемого latency, Hailo-8 (26T) — это для CNN/ViT, не transformer autoregressive |
| **Вариант** | Ollama llama3.2:1b на CPU Pi 5 (~5–8 tok/s) — реалистичнее без NPU |
| **Рекомендация** | ❌ Не рассчитывать на LLM через AI HAT+ |

---

### 4.3. 🟡 Низкий приоритет / Экспериментальные

#### 4.3.1. Semantic Segmentation сцены

- **Ценность**: понимать, что на изображении (пол, стена, дверь, человек)
- **Hailo модель**: `fast_depth` / `fcn_resnet_v1_18`
- **Применение**: вспомогательная картография (отмечать двери на карте автоматически)
- **Рекомендация**: исследовательски интересно, реализация — после завершения Nav2

#### 4.3.2. Трекинг объектов (Multi-Object Tracking)

- **Модель**: SORT/DeepSORT поверх YOLOv8
- **Ценность**: отслеживать конкретного человека при движении к нему
- **Рекомендация**: вместе с person detection как расширение

---

## 5. Аппаратная совместимость

### 5.1. Конфликт с NeoPixel LED (SPI)

Vision Pi использует **SPI0 (GPIO 10, MOSI)** для управления 381 NeoPixel LED матрицей.

**AI HAT+ использует PCIe через HAT-коннектор** — это другие пины (PCIe_CLK, PCIe_DATA и т.д.), **не SPI0**.  
→ **Конфликта нет**. SPI для NeoPixel и AI HAT работают независимо.

Подтверждение: согласно официальной документации RPi AI HAT+, PCIe x1 interface использует выделенные PCIe-пины RPi 5, не пересекающиеся с SPI/I2C/UART.

### 5.2. USB-порты

| USB | Устройство | Занят |
|-----|-----------|-------|
| USB 3.0 #1 | OAK-D Lite | ✅ |
| USB 3.0 #2 | Свободен / запасной | ✅ свободен |
| USB 2.0 #1 | ReSpeaker Mic v2 | ✅ |
| USB 2.0 #2 | MJPEG Camera | ✅ |

AI HAT+ не использует USB → **конфликтов нет**.

### 5.3. Тепловой режим

| Компонент | TDP |
|-----------|-----|
| RPi 5 под нагрузкой | ~5–7 Вт |
| AI HAT+ под нагрузкой | ~3–5 Вт |
| **Итого** | **~8–12 Вт** |

**Рекомендация**: установить активное охлаждение (вентилятор RPi) — Pi 5 с AI HAT поставляется с рекомендованным радиатором + вентилятором. Thermal throttling при перегреве ухудшает производительность.

---

## 6. Docker и архитектурные изменения

### 6.1. Что потребуется

```
docker/vision/
├── ai-inference/          # Новый сервис!
│   └── Dockerfile         # HailoRT + TAPPAS + Python binding
├── config/
│   └── hailo_inference.yaml    # Конфиги моделей, порогов
└── scripts/
    └── hailo_setup.sh     # Установка udev rules для Hailo
```

### 6.2. Docker требования

- Базовый образ должен включать **HailoRT** (Hailo официальный пакет для ARM64)
- Контейнер нужен `--device /dev/hailo0` или `privileged: true`
- Требует **Linux kernel 5.15+** с Hailo PCIe модулем (`hailort-pcie-driver`)
- Hailo предоставляет ARM64 deb-пакет — собирается отдельным base образом

### 6.3. Новые ROS 2 топики

```
/vision/persons               # BoundingBox[] + confidence + depth
/vision/faces                 # FaceDetection[] + embedding
/vision/person_approaching    # Bool — человек движется к роботу
/vision/safety_stop           # Bool — аварийная остановка
```

---

## 7. Приоритизированный план реализации

### Milestone AI-HAT-1: Базовая установка и person detection

**Оценка**: 1–2 недели

| Задача | Детали |
|--------|--------|
| Установить AI HAT+ на Vision Pi | Физическая установка, проверка питания |
| Настроить HailoRT + PCIe driver | Kernel module, udev rules, тест `hailortcli` |
| Сборка Docker base образа с HailoRT | ARM64, Python binding, тест demo |
| Интегрировать YOLOv8n person detection | ROS 2 нода `/vision/persons`, BBox + depth |
| Safety stop через twist_mux | При человеке < 1.5 м → `/cmd_vel` = 0 |

**Acceptance criteria**:
- `hailortcli scan` находит устройство
- YOLOv8n работает 30+ FPS на RGB потоке
- NeoPixel LED продолжает работать одновременно
- Safety stop срабатывает в тесте (человек перед роботом)

---

### Milestone AI-HAT-2: STT через Whisper на NPU

**Оценка**: 2–3 недели

| Задача | Детали |
|--------|--------|
| Компилировать Whisper tiny для Hailo | Или использовать pre-compiled HEF от Hailo |
| Замена Vosk → Whisper-Hailo в voice pipeline | Интерфейс должен остаться совместим |
| Тест WER на русском языке | Сравнение с текущим Vosk |
| Профилирование CPU освобождения | До и после замены |

**Acceptance criteria**:
- STT работает офлайн
- WER на тесте ≤ 20% (лучше Vosk)
- CPU load при STT ≤ 10% (было 30–50%)

---

### Milestone AI-HAT-3: Face Recognition + User Database

**Оценка**: 3–4 недели

| Задача | Детали |
|--------|--------|
| Face detection (retinaface) на Hailo | ROS 2 нода с embedding публикацией |
| User database (SQLite) | face_id, name, embedding, first_seen |
| Интеграция с dialogue manager | Привет, {name}! → персонализированный промпт |
| Enrollment flow | Команда «запомни меня» → сохранение embedding |

---

## 8. Риски

| Риск | Вероятность | Влияние | Митигация |
|------|-------------|---------|-----------|
| Hailo PCIe driver конфликт с RPi OS kernel на Pi 5 | Средняя | Высокое | Проверить kernel version, возможен downgrade |
| HailoRT несовместим с текущим Docker base | Средняя | Среднее | Новый base image с HailoRT |
| Whisper HEF для русского языка — низкое качество | Средняя | Среднее | Fallback на Vosk, итеративное тестирование |
| Перегрев Vision Pi при AI HAT + голосе одновременно | Низкая | Среднее | Активное охлаждение, thermal monitoring |
| Задержка в supply chain (AI HAT+ в наличии?) | Зависит от рынка | Низкое | Проверить наличие до старта milestone |

---

## 9. Итоговое резюме и рекомендации

### Что точно стоит делать (High ROI)

1. **Person detection (YOLOv8n)** — базовая safety feature + unlock робот-гид взаимодействие. Реализовать первым.
2. **STT через Whisper** — прямое улучшение голосового ассистента с меньшей CPU нагрузкой. Высокий приоритет после person detection.
3. **Face recognition** — красивая фича для демо и персонализации.

### Что не стоит ожидать

- Локальный LLM — 26 TOPS не хватит, используем Ollama на CPU или облако
- AprilTag на NPU — у нас уже хорошо работает через OAK-D
- Замена Nav2/SLAM на NPU — это не GPU задачи

### Следующий шаг (action items)

1. ✅ Проверить наличие **Raspberry Pi AI HAT+ (26 TOPS)** на складе/магазине
2. ✅ Создать задачу в `tasks.json`: `AI-HAT-001` — базовая установка и person detection
3. ✅ Обновить **ROADMAP.md**: статус «🔮 AI HAT (Hailo-8L NPU)» → «🔄 В работе»
4. ✅ Добавить в **PRD.md** секцию 12 (Расширение): AI HAT+ апгрейд Vision Pi

---

## Приложение: Pre-compiled Hailo Models для RPi AI HAT+

Официальный репозиторий Hailo: [hailo-ai/hailo_model_zoo](https://github.com/hailo-ai/hailo_model_zoo)

Готовые HEF-файлы для быстрого старта:

| Модель | Задача | TOPS | FPS (est.) |
|--------|--------|------|-----------|
| `yolov8n.hef` | Object detection | 3.5 | 60+ |
| `yolov8s.hef` | Object detection | 7.2 | 30+ |
| `retinaface_mobilenet_v1.hef` | Face detection | 4.6 | 50+ |
| `arcface_mobilefacenet.hef` | Face recognition | 2.8 | 60+ |
| `fast_depth.hef` | Depth estimation | 3.0 | 30+ |
| `whisper_tiny.hef` | ASR (STT) | 5.0 | real-time |

Суммарно даже при запуске person detection + face detection одновременно используется ~10 TOPS из 26 доступных — запас есть.

---

*Документ подготовлен Product Manager Agent | Rob Box Project | 2026-02-19*
