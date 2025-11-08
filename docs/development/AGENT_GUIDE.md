# Руководство для AI агентов - Rob Box Project

## 📋 Оглавление
- [🎯 Примеры эффективных запросов к AI](#-примеры-эффективных-запросов-к-ai)
- [Обзор системы](#обзор-системы)
- [Структура Docker проекта](#структура-docker-проекта)
- [Доступ к Raspberry Pi](#доступ-к-raspberry-pi)
- [Инструментарий мониторинга и диагностики](#инструментарий-мониторинга-и-диагностики)
- [Анализ последних изменений](#анализ-последних-изменений)
- [Текущая архитектура Zenoh](#текущая-архитектура-zenoh)
- [Скрипты обновления и управления](#скрипты-обновления-и-управления)

---

## 🎯 Примеры эффективных запросов к AI

**Best Practice от GitHub:** "Make your ask simple and specific. Break down complex tasks."

### ❌ ПЛОХО (vague, no context):

"Почини voice assistant"

**Почему плохо:**
- Нет информации о проблеме
- Нет контекста где искать
- Слишком широко

---

### ✅ ХОРОШО (specific + context + expected outcome):

"Voice assistant на Vision Pi падает с ошибкой `ModuleNotFoundError: No module named 'nav2_msgs'` в command_node. 

Проверь `docker/vision/voice_assistant/Dockerfile` - добавлен ли пакет `ros-humble-nav2-msgs` в секцию `RUN apt-get install`. Если нет - добавь его в список пакетов вместе с другими `ros-humble-*` зависимостями."

**Почему хорошо:**
- ✅ Конкретная ошибка с текстом
- ✅ Указана нода где проблема
- ✅ Указан файл для проверки
- ✅ Дано действие для исправления

---

### ❌ ПЛОХО (too complex, multiple tasks):

"Сделай чтобы камера работала с rtabmap и publishила в zenoh и чтобы все логи были видны и добавь мониторинг"

**Почему плохо:**
- Слишком много задач сразу
- Нет четкой последовательности
- Непонятно с чего начать

---

### ✅ ХОРОШО (step-by-step):

**Запрос 1:**
"Проверь что oak-d контейнер запущен и публикует топики. Выполни:
```bash
docker ps | grep oak-d
docker exec oak-d ros2 topic list | grep camera
```
Покажи результат."

**Запрос 2 (после результата):**
"Теперь проверь что zenoh-router видит эти топики:
```bash
curl http://localhost:8000/@/local/subscriber
```
Найди в выводе `/camera/rgb/image_raw` и `/camera/depth/image_rect_raw`."

**Запрос 3:**
"Проверь что rtabmap подписан на эти топики:
```bash
docker logs rtabmap --tail 100 | grep 'Subscribed to'
```"

**Почему хорошо:**
- ✅ Разбито на 3 простых шага
- ✅ Каждый шаг с конкретной командой
- ✅ Ожидаемый результат указан
- ✅ Можно проверять пошагово

---

### ✅ ОТЛИЧНО (with examples + desired outcome):

"В launch файле `voice_assistant_headless.launch.py` нужно добавить новую ноду `command_node`.

**Пример существующей ноды:**
```python
Node(
    package='rob_box_voice',
    executable='audio_node',
    name='audio_node',
    output='screen',
    parameters=[config_file],
    respawn=True,
    respawn_delay=2.0
)
```

**Задача:** Добавь аналогичную для `command_node` с:
- `respawn_delay=5.0` (больше чем у audio_node)
- В комментарии укажи 'Navigation commands processor'

**Желаемый результат:**
```python
# Navigation commands processor
Node(
    package='rob_box_voice',
    executable='command_node',
    name='command_node',
    output='screen',
    parameters=[config_file],
    respawn=True,
    respawn_delay=5.0
)
```"

**Почему отлично:**
- ✅ Указан файл
- ✅ Дан пример похожего кода
- ✅ Четкие требования
- ✅ Показан желаемый результат
- ✅ Используется few-shot learning

---

### 💡 Pro Tips для эффективных запросов

1. **Начинай с контекста:**
   - "Работаю с Voice Assistant на Vision Pi"
   - "Изменяю docker-compose.yaml для добавления env переменной"
   - "Фикшу Animation Editor - не сохраняются keyframes"

2. **Указывай файлы явно:**
   - ❌ "в конфиге"
   - ✅ "в `docker/vision/docker-compose.yaml`"

3. **Цитируй ошибки полностью:**
   - ❌ "какая-то ошибка с импортом"
   - ✅ "```ModuleNotFoundError: No module named 'nav2_msgs'```"

4. **Показывай что уже пробовал:**
   - "Проверил что контейнер запущен - `docker ps` показывает voice-assistant"
   - "Посмотрел логи - `docker logs voice-assistant` показывает RuntimeError"

5. **Используй AI Context Map:**
   - Открой 1-2 релевантных файла ПЕРЕД запросом
   - Смотри `docs/development/AI_CONTEXT_MAP.md` для подсказок

6. **Разбивай на шаги если сложно:**
   ```
   Задача: Настроить voice assistant с новыми API ключами
   
   Шаг 1: Создай .env.secrets из template
   Шаг 2: Добавь env_file в docker-compose.yaml
   Шаг 3: Перезапусти контейнер
   Шаг 4: Проверь что ключи загружены
   ```

---

### 📚 Шаблоны запросов

#### Debugging Issue:
```
Проблема: [описание]
Файл: [путь к файлу]
Ошибка: [текст ошибки]
Что проверил: [список действий]
Ожидаемое поведение: [что должно быть]
```

#### Adding Feature:
```
Задача: Добавить [фича] в [компонент]
Похожий пример: [файл:строки или код]
Требования:
  1. [требование 1]
  2. [требование 2]
Желаемый результат: [описание или код]
```

#### Configuration Change:
```
Изменяю: [файл конфигурации]
Цель: [что хочу настроить]
Текущее значение: [current config]
Желаемое значение: [desired config]
Зависимости: [что может сломаться]
```

---

## Обзор системы

**Rob Box Project** - автономный робот на базе двух Raspberry Pi 4 с OAK-D Lite камерой и RTAB-Map SLAM.

### Компоненты системы

**Vision Pi (10.1.1.11 eth0 / 10.1.1.21 wlan0)**
- OAK-D Lite камера (Intel Movidius MyriadX)
- AprilTag детекция
- LED Matrix контроллер
- Voice Assistant (ReSpeaker)
- Zenoh router для маршрутизации данных

**Main Pi (10.1.1.10 eth0 / 10.1.1.20 wlan0)**
- RTAB-Map SLAM система
- LSLIDAR N10 (2D LiDAR)
- Perception система (внутренний диалог)
- Navigation (Nav2)
- Motor control (ROS2 Control + VESC)
- Zenoh router (центральный узел)
- Подключение к внешнему серверу zenoh.robbox.online:7447

### Сетевая топология

```
Vision Pi (eth0: 10.1.1.11)  ←→  Main Pi (eth0: 10.1.1.10)
     ↓                                    ↓
WiFi (wlan0: 10.1.1.21)         WiFi (wlan0: 10.1.1.20)
   (управление)                        (управление)
                                         ↓
                               zenoh.robbox.online:7447
```

**⚠️ ВАЖНО**: 
- **Gigabit Ethernet (eth0)**: ТОЛЬКО для передачи данных между Pi
- **WiFi (wlan0)**: ТОЛЬКО для SSH доступа и управления
- В конфигурациях Zenoh router используются ТОЛЬКО Ethernet IP (10.1.1.10, 10.1.1.11)

---

## Структура Docker проекта

### 📁 Организация файлов

Проект использует стандартизированную структуру для управления Docker контейнерами:

```
docker/
├── DOCKER_STANDARDS.md          # ⭐ Полная документация стандартов
├── AGENT_GUIDE.md               # Этот файл - гайд для агентов
├── main/                        # Main Pi (10.1.1.20)
│   ├── docker-compose.yaml      # Оркестрация контейнеров
│   ├── config/                  # Общие конфиги для всех сервисов
│   ├── scripts/                 # Утилитарные скрипты
│   ├── maps/                    # Persistent данные RTAB-Map
│   └── <service_name>/          # Папки сервисов
│       ├── Dockerfile
│       ├── entrypoint.sh        (опционально)
│       └── config/              (опционально - специфичные конфиги)
└── vision/                      # Vision Pi (10.1.1.21)
    ├── docker-compose.yaml
    ├── config/                  # Общие конфиги
    ├── scripts/                 # Утилитарные скрипты
    └── <service_name>/
```

### 🎯 Ключевые принципы

1. **Volumes**: Всегда `./config:/config`, БЕЗ дублирования отдельных файлов
2. **Конфиги**: Общие в `config/`, специфичные в `<service>/config/`
3. **Скрипты**: Управление в `scripts/`, entrypoint в папке сервиса
4. **Environment**: Стандартный набор ROS 2 + Zenoh переменных
5. **Network**: Только `network_mode: host`
6. **Dependencies**: Все сервисы зависят от `zenoh-router`

### ⚠️ КРИТИЧЕСКИЕ ПРАВИЛА ДЛЯ DOCKERFILES

**🚫 ЗАПРЕЩЕНО копировать в Dockerfile:**
- ❌ `COPY config/` - конфиги монтируются через volumes!
- ❌ `COPY scripts/` - скрипты монтируются через volumes!
- ❌ `COPY launch/` - launch файлы монтируются через volumes!
- ❌ `COPY src/rob_box_description` - URDF монтируется через volumes!

**✅ РАЗРЕШЕНО в Dockerfile:**
- ✅ `RUN apt-get install` - установка пакетов
- ✅ `RUN git clone` - клонирование репозиториев
- ✅ `RUN colcon build` - компиляция ROS пакетов
- ✅ `RUN pip install` - установка Python зависимостей

**Почему это важно:**
- Изменение конфига/скрипта НЕ должно требовать пересборки образа
- Пересборка lslidar/apriltag занимает 5-10 минут
- Restart контейнера занимает 2-5 секунд
- Volumes позволяют применять изменения мгновенно

**См. подробности:** `BUILD_OPTIMIZATION.md`

### 📖 Полная документация

**Перед любыми изменениями Docker конфигурации читай:**
- `DOCKER_STANDARDS.md` - детальные стандарты, примеры, workflow
- `BUILD_OPTIMIZATION.md` - правила оптимизации сборки образов
- `CI_CD_PIPELINE.md` - автоматизация сборки через GitHub Actions

---

## 🚀 CI/CD через GitHub Actions

### Автоматическая сборка Docker образов

**⚠️ ВАЖНО**: Docker образы НЕ собираются локально на Pi! Сборка происходит через GitHub Actions.

### Архитектура workflow

```
Feature Branch (feature/*)
         ↓ push
    GitHub Actions
         ↓ build changed services
    Auto-merge → Develop (develop)
         ↓ push all changes
    Build ALL services
         ↓ success
    Auto-merge → Main (main)
         ↓
    Docker Images Published
    ghcr.io/krikz/rob_box:*-humble-latest
```

### Workflow файлы

| Workflow | Файл | Назначение |
|----------|------|------------|
| **Feature → Develop** | `.github/workflows/auto-merge-feature-to-develop.yml` | Собирает изменённые сервисы, автомерджит в develop |
| **Develop → Main** | `.github/workflows/auto-merge-to-main.yml` | Собирает ВСЕ сервисы, автомерджит в main |
| **Vision Services** | `.github/workflows/build-vision-services.yml` | Сборка oak-d, apriltag, led-matrix, voice-assistant |
| **Main Services** | `.github/workflows/build-main-services.yml` | Сборка rtabmap, nav2, ros2-control, lslidar, perception |
| **Base Images** | `.github/workflows/build-base-images.yml` | Сборка базовых образов (ros2-zenoh) |

### Docker Image Tagging

| Branch | Tag | Использование |
|--------|-----|---------------|
| **main** | `*-humble-latest` | Production (стабильная версия) |
| **develop** | `*-humble-dev` | Development (тестирование) |
| **feature/*** | `*-humble-test` | Feature testing (текущая разработка) |

**Пример для voice-assistant:**
- `ghcr.io/krikz/rob_box:voice-assistant-humble-latest` (main)
- `ghcr.io/krikz/rob_box:voice-assistant-humble-dev` (develop)
- `ghcr.io/krikz/rob_box:voice-assistant-humble-test` (feature/*)

### Workflow для изменения кода

```bash
# 1. Создаешь feature ветку
git checkout -b feature/fix-voice-assistant

# 2. Делаешь изменения (например, в Dockerfile или docker-compose.yaml)
vim docker/vision/voice_assistant/Dockerfile
vim docker/vision/docker-compose.yaml

# 3. Коммитишь и пушишь
git add .
git commit -m "fix: добавить nav2-msgs для command_node"
git push origin feature/fix-voice-assistant

# 4. GitHub Actions автоматически:
#    - Определяет что изменился Vision Pi
#    - Запускает build-vision-services.yml
#    - Собирает voice-assistant-humble-test
#    - Пушит образ в ghcr.io
#    - Мерджит в develop (если сборка успешна)
#    - Удаляет feature ветку

# 5. На Vision Pi - обновляешь код и подтягиваешь новый образ
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && \
   git pull && \
   docker compose pull voice-assistant && \
   docker compose up -d voice-assistant'
```

### Мониторинг сборки

**GitHub Actions:**
- https://github.com/krikz/rob_box_project/actions

**Проверка статуса:**
```bash
# Последний workflow run
gh run list --limit 5

# Логи конкретного workflow
gh run view <run-id> --log
```

**Проверка образов:**
```bash
# На Vision Pi - проверить какой образ используется
sshpass -p 'open' ssh ros2@10.1.1.21 'docker images | grep voice-assistant'

# Должен показать: ghcr.io/krikz/rob_box  voice-assistant-humble-test
```

### Когда НЕ нужна пересборка образа

**Изменения БЕЗ rebuild** (применяются мгновенно через volumes):
- Конфиги: `docker/vision/config/**`
- Скрипты: `docker/vision/scripts/**`
- Launch файлы: `docker/vision/config/voice/voice_assistant_headless.launch.py`

**Изменения С rebuild** (требуют GitHub Actions):
- `Dockerfile` (установка пакетов, зависимостей)
- `requirements.txt` (Python пакеты)
- Исходный код ROS пакетов (colcon build)

**Подробности:** См. `docs/CI_CD_PIPELINE.md`

---

### ⚡ Быстрая отладка через docker exec (БЕЗ пересборки образа)

**Когда использовать:**
- Нужно быстро проверить правку кода
- CI/CD сборка занимает 5-10 минут
- Конфиги монтируются через volumes и можно менять локально

**⚠️ ВАЖНО:** Изменения внутри контейнера **НЕ сохраняются** после рестарта! Это только для отладки.

**Workflow быстрой отладки:**

```bash
# 1. НА ЛОКАЛЬНОЙ МАШИНЕ - коммитим изменения (но НЕ пушим)
git add docker/vision/config/oak-d/oak_d_config.yaml
git commit -m "fix: включить IMU для RTAB-Map"

# 2. НА RASPBERRY PI - заходим в контейнер и меняем конфиг вручную
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21

# Внутри Vision Pi:
cd ~/rob_box_project/docker/vision
nano config/oak-d/oak_d_config.yaml  # Вручную меняем i_enable_imu: true

# 3. Перезапускаем контейнер чтобы применить изменения
docker restart oak-d

# 4. Проверяем что изменения работают
docker logs oak-d --tail 50

# 5. Если всё ОК - пушим коммит и запускаем CI/CD
exit  # Выходим из Pi
git push origin feature/enable-imu  # Это запустит сборку образа

# 6. После успешной сборки - pull новый образ на Pi
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && \
   git pull && \
   docker compose pull oak-d && \
   docker compose up -d oak-d'
```

**Альтернатива - редактирование внутри контейнера:**

```bash
# Зайти в контейнер и редактировать файл напрямую
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 \
  'docker exec -it oak-d /bin/bash'

# Внутри контейнера:
apt-get update && apt-get install -y nano
nano /config/oak_d_config.yaml
exit

# Перезапустить контейнер
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 \
  'docker restart oak-d'
```

**Плюсы:**
- ✅ Мгновенная проверка изменений (2-3 секунды)
- ✅ Не ждём CI/CD сборку (5-10 минут)
- ✅ Можно быстро итерировать правки

**Минусы:**
- ❌ Изменения ПОТЕРЯЮТСЯ при рестарте контейнера из нового образа
- ❌ Нужно помнить скоммитить и запушить после проверки
- ❌ Риск забыть запушить и потерять правки

**Правило:** ВСЕГДА коммитим СНАЧАЛА, отлаживаем внутри контейнера, потом пушим.

---

### 🛠️ Работа с Docker на Pi через WSL

**ВАЖНО**: Для управления Docker на Raspberry Pi из Windows используй WSL + sshpass:

```bash
# Общий формат команды
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@<PI_IP> '<команда>'

# Примеры:
# Vision Pi - статус контейнеров
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 'docker ps'

# Main Pi - логи RTAB-Map
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.20 'docker logs rtabmap'

# Обновление и пересборка (Vision Pi)
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && git pull && docker compose build lslidar && docker compose up -d lslidar'

# Полный рестарт системы (Main Pi)
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.20 \
  'cd ~/rob_box_project/docker/main && docker compose down && docker compose up -d'
```

**Почему WSL + sshpass:**
- Windows PowerShell не поддерживает sshpass напрямую
- SSH ключи не настроены (используется парольная аутентификация)
- `-o StrictHostKeyChecking=no` отключает проверку host key

### 🚀 Workflow добавления нового сервиса

См. раздел "Workflow для добавления нового сервиса" в `DOCKER_STANDARDS.md`

---

## Доступ к Raspberry Pi

### Credentials

| Параметр | Vision Pi | Main Pi |
|----------|-----------|---------|
| **Ethernet IP** | 10.1.1.11 | 10.1.1.10 |
| **WiFi IP** | 10.1.1.21 | 10.1.1.20 |
| **Username** | ros2 | ros2 |
| **Password** | open | open |

**⚠️ ВАЖНО ДЛЯ АГЕНТОВ**: Всегда используй `sshpass -p 'open'` для неинтерактивных SSH команд!

### SSH подключение

```bash
# ⚠️ ВСЕГДА используй sshpass для автоматизации!
# Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.11

# Main Pi  
sshpass -p 'open' ssh ros2@10.1.1.20

# Выполнение команд без интерактивного входа
sshpass -p 'open' ssh ros2@10.1.1.11 'docker ps'
sshpass -p 'open' ssh ros2@10.1.1.20 'docker logs rtabmap'
```

### Выполнение команд на удаленных Pi

```bash
# Vision Pi - проверка контейнеров
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 'docker ps'

# Main Pi - просмотр логов
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.20 'docker logs rtabmap'

# Обновление кода и перезапуск
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && git pull && docker-compose down && docker-compose up -d'
```

---

## 🔒 Управление секретами (API Keys)

### Voice Assistant секреты

Voice Assistant требует API ключи для работы DialogueNode (DeepSeek) и TTSNode (Yandex Cloud).

**⚠️ КРИТИЧЕСКИ ВАЖНО**: API ключи НЕ должны коммититься в git!

### Создание .env.secrets на Vision Pi

Файл `.env.secrets` должен быть создан вручную на Vision Pi:

```bash
# Подключаемся к Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21

# Создаем .env.secrets
cat > ~/rob_box_project/docker/vision/.env.secrets << 'EOF'
# 🔒 API Keys для Voice Assistant (НЕ коммитить в git!)

# DeepSeek API (для DialogueNode - LLM диалоги)
DEEPSEEK_API_KEY=your_deepseek_api_key_here

# Yandex Cloud API (для TTSNode - синтез речи)
YANDEX_API_KEY=your_yandex_api_key_here
YANDEX_FOLDER_ID=your_yandex_folder_id_here
EOF

# Проверяем что файл создан
cat ~/rob_box_project/docker/vision/.env.secrets
```

**Где взять ключи:**
- **DeepSeek API**: https://platform.deepseek.com/api_keys
- **Yandex Cloud**: https://console.cloud.yandex.ru/folders/{folder_id}/iam/service-accounts

**Защита от коммита:**
- Файл `.env.secrets` добавлен в `docker/vision/.gitignore`
- docker-compose.yaml использует `env_file: .env.secrets` вместо прямых environment переменных

### Проверка секретов

```bash
# На Vision Pi - проверить что ключи загружены в контейнер
sshpass -p 'open' ssh ros2@10.1.1.21 \
  'docker exec voice-assistant printenv | grep -E "DEEPSEEK|YANDEX"'

# Должен вернуть:
# DEEPSEEK_API_KEY=sk-...
# YANDEX_API_KEY=AQVN...
# YANDEX_FOLDER_ID=aje...
```

---

## Инструментарий мониторинга и диагностики

### 📊 Скрипты мониторинга

#### 1. `docker/monitor_system.sh` - Общий системный мониторинг
**Местоположение**: `docker/monitor_system.sh`  
**Назначение**: Универсальный мониторинг CPU, памяти, сети, температуры для обоих Pi  
**Использование**:
```bash
# На Vision Pi
ssh ros2@10.1.1.21
cd ~/rob_box_project/docker
./monitor_system.sh
```

**Что показывает**:
- Тип Pi (определяется автоматически по контейнерам)
- CPU нагрузка (общая система + контейнер)
- Память (общая + контейнер)
- Температура процессора
- Throttling состояние (пониженное напряжение, частота и т.д.)
- Сеть (RX/TX на eth0 и wlan0)
- Топ процессов
- Статус Docker контейнеров

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - универсальный инструмент для быстрой диагностики
- Запускать при подозрении на проблемы с производительностью

---

#### 2. `docker/vision/realtime_monitor.sh` - Real-time мониторинг камеры
**Местоположение**: `docker/vision/realtime_monitor.sh`  
**Назначение**: Детальный мониторинг работы OAK-D камеры в реальном времени  
**Использование**:
```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
./realtime_monitor.sh [duration_in_seconds]  # по умолчанию 30 секунд
```

**Что показывает** (каждые 2 секунды):
- CPU (usr/sys/idle)
- Сеть eth0 (RX/TX KB/s)
- RGB publisher count
- Depth publisher count
- Температура
- Throttling состояние

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - ключевой инструмент для отладки камеры
- Использовать для проверки стабильности публикации топиков
- Запускать после изменения конфигурации камеры

---

#### 3. `docker/vision/monitor_camera_startup.sh` - Мониторинг запуска камеры
**Местоположение**: `docker/vision/monitor_camera_startup.sh`  
**Назначение**: Комплексный мониторинг запуска камеры с сохранением логов  
**Использование**:
```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
./monitor_camera_startup.sh
```

**Что делает**:
- Перезапускает камеру
- Фоновый мониторинг CPU (60 секунд, каждые 2 секунды)
- Фоновый мониторинг питания/throttling
- Фоновый мониторинг сети eth0
- Логирование ROS 2 топиков
- Сохранение всех логов в `/tmp/camera_monitor_YYYYMMDD_HHMMSS/`

**Рекомендации**:
- ⚠️ **УСТАРЕЛ** - функционал дублируется `realtime_monitor.sh`
- 🗑️ **УДАЛИТЬ** или **ОБЪЕДИНИТЬ** с `realtime_monitor.sh`
- Сохранение логов полезно, но можно реализовать опцией в realtime_monitor

---

### 🔍 Скрипты диагностики

#### 4. `docker/diagnose_data_flow.sh` - Диагностика связи между Pi
**Местоположение**: `docker/diagnose_data_flow.sh`  
**Назначение**: Проверка всего пути передачи данных: Vision Pi → Main Pi  
**Использование**:
```bash
# На локальной машине (Windows PowerShell)
cd d:\PROJECTS\rob_box_project\docker
wsl bash ./diagnose_data_flow.sh
```

**Что проверяет**:
1. Полный перезапуск камеры через docker-compose
2. Publisher count на Vision Pi
3. Сетевой трафик eth0 на Vision Pi
4. Subscriber count на Main Pi
5. Итоговая диагностика

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - критичный инструмент для диагностики межплатной связи
- Запускать при проблеме "Did not receive data since 5 seconds"
- Автоматизирует весь процесс диагностики с использованием sshpass

---

#### 5. `docker/vision/diagnose.sh` - Диагностика камеры на Vision Pi
**Местоположение**: `docker/vision/diagnose.sh`  
**Назначение**: Быстрая проверка статуса камеры и топиков  
**Использование**:
```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
./diagnose.sh
```

**Что проверяет**:
1. Статус контейнера oak-d
2. Логи камеры (ERROR/WARN)
3. Список ROS 2 топиков от камеры
4. Информация о топиках (compressed)
5. Публикация топиков (5 секунд)
6. Нагрузка системы (CPU/память)

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - быстрая диагностика на Vision Pi
- Использовать для первичной проверки камеры

---

#### 6. `docker/vision/force_publish.sh` - Принудительная публикация топиков
**Местоположение**: `docker/vision/force_publish.sh`  
**Назначение**: Отключение lazy publisher для OAK-D  
**Использование**:
```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
./force_publish.sh
```

**Что делает**:
- Отключает `color.i_enable_lazy_publisher` 
- Отключает `depth.i_enable_lazy_publisher`
- Проверяет publisher count

**Рекомендации**:
- ⚠️ **УСТАРЕЛ** - `update_and_restart.sh` уже включает эту логику
- 🗑️ **УДАЛИТЬ** - функционал дублируется, можно оставить как справочник

---

### 🔧 Скрипты настройки

#### 7. `docker/tune.sh` - Настройка сетевых буферов (CycloneDDS)
**Местоположение**: `docker/tune.sh`  
**Назначение**: Увеличение сетевых буферов для CycloneDDS  
**Использование**:
```bash
# На любом Pi
cd ~/rob_box_project/docker
sudo bash tune.sh
```

**Рекомендации**:
- ⚠️ **УСТАРЕЛ** - система перешла на Zenoh, CycloneDDS больше не используется
- 🗑️ **УДАЛИТЬ** - оставить только как исторический артефакт в документации

---

### 🚀 Скрипты обновления и управления

#### 8. `docker/vision/update_and_restart.sh` - Обновление и перезапуск Vision Pi
**Местоположение**: `docker/vision/update_and_restart.sh`  
**Назначение**: Автоматизированное обновление кода и перезапуск контейнеров  
**Использование**:
```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
./update_and_restart.sh
```

**Что делает**:
1. Останавливает контейнеры (`docker-compose down --remove-orphans`)
2. Обновляет код из GitHub (`git pull origin main`)
3. Запускает контейнеры (`docker-compose up -d`)
4. Отключает lazy publisher (force publish)
5. Показывает логи камеры

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - ключевой инструмент для deployment
- Использовать после каждого коммита в репозиторий

---

#### 9. `docker/main/update_and_restart.sh` - Обновление и перезапуск Main Pi
**Местоположение**: `docker/main/update_and_restart.sh`  
**Назначение**: Автоматизированное обновление кода и перезапуск RTAB-Map  
**Использование**:
```bash
# На Main Pi
cd ~/rob_box_project/docker/main
./update_and_restart.sh
```

**Что делает**:
1. Останавливает контейнеры (`docker-compose down --remove-orphans`)
2. Обновляет код из GitHub (`git pull origin main`)
3. Запускает контейнеры (`docker-compose up -d`)
4. Показывает логи RTAB-Map

**Рекомендации**:
- ✅ **ОСТАВИТЬ** - ключевой инструмент для deployment
- Запускать после обновления Vision Pi

---

### 📝 Вспомогательные скрипты (в config/)

#### 10. `docker/vision/config/start_oak_d.sh` - Старт камеры OAK-D
**Местоположение**: `docker/vision/config/start_oak_d.sh`  
**Назначение**: Запуск ноды OAK-D внутри контейнера  
**Рекомендации**:
- ✅ **ОСТАВИТЬ** - используется Dockerfile для запуска камеры

#### 11. `docker/vision/config/start_apriltag.sh` - Старт AprilTag детекции
**Местоположение**: `docker/vision/config/start_apriltag.sh`  
**Назначение**: Запуск AprilTag детектора внутри контейнера  
**Рекомендации**:
- ✅ **ОСТАВИТЬ** - используется Dockerfile для запуска AprilTag

---

## Анализ последних изменений

### Коммиты (последние 7)

#### 1. `abd2629 - fixes`
**Дата**: Последний коммит  
**Изменения**:
- Удален `docker/main/discovery/Dockerfile` (FastDDS discovery сервер больше не нужен)
- Обновлены `zenoh_router_config.json5` для Vision и Main Pi
- Обновлены `docker-compose.yaml` для Vision и Main Pi

**Анализ**: Финальная очистка после перехода на Zenoh. Удалены артефакты CycloneDDS/FastDDS.

---

#### 2. `4bd59ec - default zenoh configuration`
**Дата**: Предыдущий коммит  
**Изменения**:
- Обновлены конфигурации Zenoh router и session для обоих Pi
- Удален `zenoh_local_session_config.json5` на Vision Pi
- Обновлен `Dockerfile` для AprilTag

**Анализ**: Переход на стандартные конфигурации Zenoh (по умолчанию из документации). Упрощение архитектуры.

---

#### 3-7. `520457e - c1e3d7a` - Revert коммиты
**Дата**: Откаты предыдущих попыток  
**Изменения**: Откат всех попыток использовать `rmw_zenohd` (ROS 2 Zenoh router)

**Анализ**: 
Пользователь экспериментировал с разными подходами:
1. Пытался использовать `rmw_zenohd` (ROS 2 специфичный Zenoh router)
2. Пытался исправить запуск через `source setup.bash`
3. Пытался создать кастомные Dockerfiles для роутеров с `rmw_zenoh_cpp`
4. Исправлял GPG ключи ROS 2 репозитория
5. Добавлял `LD_LIBRARY_PATH` для библиотек Zenoh

**В итоге**: Все откатил и перешёл на **стандартный Zenoh router** (`eclipse/zenoh:1.6.2`), который работает напрямую без ROS 2 обёртки.

---

### Ключевые выводы из изменений

1. **Zenoh router**: Используется стандартный `eclipse/zenoh:1.6.2`, НЕ `rmw_zenohd`
2. **FastDDS discovery**: Полностью удалён, больше не используется
3. **Конфигурации Zenoh**: Стандартные из документации, без кастомных модификаций
4. **Архитектура упрощена**: Меньше зависимостей, меньше кастомных Dockerfiles

---

## Текущая архитектура Zenoh

### Docker Compose конфигурация

#### Vision Pi (`docker/vision/docker-compose.yaml`)

```yaml
services:
  zenoh-router:
    image: eclipse/zenoh:1.6.2
    container_name: zenoh-router-vision
    network_mode: host
    environment:
      - RUST_LOG=zenoh=info
    volumes:
      - ./config/zenoh_router_config.json5:/zenoh_config.json5
      - /dev/shm:/dev/shm
    command: -c /zenoh_config.json5
    restart: unless-stopped
    healthcheck:
      test: ["CMD-SHELL", "wget -qO- http://localhost:8000/@/local/router || exit 1"]
      interval: 5s
      timeout: 10s
      retries: 20
      start_period: 30s

  oak-d:
    # ...
    environment:
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
      - ZENOH_ROUTER_CHECK_ATTEMPTS=10
      - RUST_LOG=zenoh=info
      - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      - ZENOH_CONFIG=/config/zenoh_session_config.json5
    depends_on:
      - zenoh-router
```

#### Main Pi (`docker/main/docker-compose.yaml`)

```yaml
services:
  zenoh-router:
    image: eclipse/zenoh:1.6.2
    container_name: zenoh-router
    network_mode: host
    environment:
      - RUST_LOG=zenoh=info
    volumes:
      - ./config/zenoh_router_config.json5:/zenoh_config.json5
    command: -c /zenoh_config.json5
    restart: unless-stopped
    healthcheck:
      test: ["CMD-SHELL", "wget -qO- http://localhost:8000/@/local/router || exit 1"]
      interval: 5s
      timeout: 10s
      retries: 20
      start_period: 30s

  rtabmap:
    # ...
    environment:
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
      - ZENOH_ROUTER_CHECK_ATTEMPTS=10
      - RUST_LOG=zenoh=info
      - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      - ZENOH_CONFIG=/config/zenoh_session_config.json5
      - LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib
    depends_on:
      - zenoh-router
```

### Конфигурации Zenoh

#### Router конфигурация (для обоих Pi)
**Файлы**: 
- `docker/vision/config/zenoh_router_config.json5`
- `docker/main/config/zenoh_router_config.json5`

**Ключевые параметры**:
```json5
{
  mode: "router",
  connect: {
    endpoints: []  // Зависит от Pi
  },
  // ... стандартная конфигурация из Zenoh документации
}
```

**Vision Pi router**: Подключается к Main Pi router (пусто в стандартной конфигурации, используется автообнаружение)  
**Main Pi router**: Подключается к внешнему серверу `zenoh.robbox.online:7447`

#### Session конфигурация (для ROS 2 нод)
**Файлы**:
- `docker/vision/config/zenoh_session_config.json5`
- `docker/main/config/zenoh_session_config.json5`

**Ключевые параметры**:
```json5
{
  mode: "peer",
  connect: {
    endpoints: ["tcp/localhost:7447"]  // Подключение к локальному router
  },
  // ... стандартная конфигурация
}
```

---

## Скрипты обновления и управления

### Workflow обновления системы

#### 1. Локальные изменения (на Windows)
```bash
cd d:\PROJECTS\rob_box_project
# Вносим изменения в код/конфигурации
git add .
git commit -m "Описание изменений"
git push
```

#### 2. Обновление Vision Pi
```bash
# Удалённое выполнение с Windows
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && ./update_and_restart.sh'

# Или через SSH
ssh ros2@10.1.1.21
cd ~/rob_box_project/docker/vision
./update_and_restart.sh
```

#### 3. Обновление Main Pi
```bash
# Удалённое выполнение с Windows
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.20 \
  'cd ~/rob_box_project/docker/main && ./update_and_restart.sh'

# Или через SSH
ssh ros2@10.1.1.20
cd ~/rob_box_project/docker/main
./update_and_restart.sh
```

#### 4. Проверка статуса
```bash
# Vision Pi
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 'docker ps'

# Main Pi
wsl sshpass -p 'open' ssh -o StrictHostKeyChecking=no ros2@10.1.1.20 'docker ps'
```

---

## Рекомендации по скриптам

### ✅ ОСТАВИТЬ (критичные)
1. **docker/monitor_system.sh** - универсальный мониторинг
2. **docker/vision/realtime_monitor.sh** - детальный мониторинг камеры
3. **docker/diagnose_data_flow.sh** - диагностика межплатной связи
4. **docker/vision/diagnose.sh** - быстрая диагностика камеры
5. **docker/vision/update_and_restart.sh** - deployment Vision Pi
6. **docker/main/update_and_restart.sh** - deployment Main Pi
7. **docker/vision/config/start_oak_d.sh** - запуск камеры (используется Dockerfile)
8. **docker/vision/config/start_apriltag.sh** - запуск AprilTag (используется Dockerfile)

### ⚠️ УСТАРЕЛИ / ТРЕБУЮТ ПЕРЕСМОТРА
1. **docker/vision/monitor_camera_startup.sh** - дублирует realtime_monitor.sh
   - Рекомендация: удалить или объединить с realtime_monitor.sh
2. **docker/vision/force_publish.sh** - логика в update_and_restart.sh
   - Рекомендация: удалить, оставить как справочник
3. **docker/tune.sh** - для CycloneDDS (больше не используется)
   - Рекомендация: удалить, оставить в TROUBLESHOOTING.md как историю

### 📚 Документация
Текущая документация актуальна и хорошо структурирована:
- **SYSTEM_OVERVIEW.md** - архитектура системы
- **TROUBLESHOOTING.md** - решение проблем
- **BUILD_OPTIMIZATION.md** - оптимизация системы
- **QUICK_START.md** - быстрый старт

---

## Следующие шаги для агентов

### При работе с системой

1. **Всегда проверяйте последние коммиты** перед внесением изменений
   ```bash
   git log -5 --oneline --name-status
   ```

2. **Используйте правильные IP адреса**:
   - SSH доступ: WiFi IP (10.1.1.21, 10.1.1.20)
   - Конфигурации Zenoh: Ethernet IP (10.1.1.11, 10.1.1.10)

3. **Диагностика проблем**:
   - Сначала `diagnose.sh` на Vision Pi
   - Затем `diagnose_data_flow.sh` для межплатной связи
   - `monitor_system.sh` для общей производительности

4. **Deployment**:
   - Всегда `git push` перед обновлением Pi
   - Обновляйте Vision Pi → Main Pi в таком порядке
   - Проверяйте логи после каждого обновления

5. **Zenoh router**:
   - Используйте стандартный `eclipse/zenoh:1.6.2`
   - НЕ используйте `rmw_zenohd` (откачено пользователем)
   - Конфигурации в `zenoh_router_config.json5` и `zenoh_session_config.json5`

---

## История миграции на Zenoh

### Было (CycloneDDS)
- Multicast discovery между Pi
- Проблемы с обнаружением нод (0 KB/s сетевой трафик)
- Необходимость FastDDS discovery сервера

### Стало (Zenoh)
- Router-to-router архитектура
- Vision Pi router ↔ Main Pi router ↔ zenoh.robbox.online
- Стандартный `eclipse/zenoh:1.6.2` router
- Peer mode для ROS 2 нод (подключение к localhost:7447)

### Попытки использования rmw_zenohd (откачены)
Пользователь пытался использовать специфичный для ROS 2 `rmw_zenohd` router:
- Создавал кастомные Dockerfiles
- Исправлял GPG ключи ROS 2
- Добавлял LD_LIBRARY_PATH для Zenoh библиотек

**В итоге**: Откатил все изменения и перешёл на стандартный Zenoh router, который работает стабильно.

---

**Документ создан**: 2025-10-09  
**Версия системы**: Zenoh architecture (после миграции с CycloneDDS)  
**Последний анализированный коммит**: abd2629
