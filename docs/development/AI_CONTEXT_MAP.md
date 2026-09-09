# AI Context Map - Какие файлы открывать для разных задач

**Назначение:** Этот файл помогает AI агенту быстро найти релевантные файлы для контекста при решении задач.

**Best Practice от GitHub:** "Keep 1-2 relevant tabs open" - Copilot лучше понимает код когда видит релевантный контекст.

---

## 🐳 Docker & Контейнеры

### Изменение Docker конфигурации

**Файлы для контекста:**
1. `docker/vision/docker-compose.yaml` ИЛИ `docker/main/docker-compose.yaml`
2. `docs/development/DOCKER_STANDARDS.md`
3. `docs/development/AGENT_GUIDE.md` (секция Docker)

**Примеры задач:**
- Добавление environment переменных
- Изменение volumes
- Обновление image tags
- Настройка depends_on

---

### Добавление нового Docker сервиса

**Файлы для контекста:**
1. `docs/development/DOCKER_STANDARDS.md` (секция "Workflow для добавления нового сервиса")
2. Похожий Dockerfile (например, `docker/vision/oak-d/Dockerfile` для камер)
3. `docker/vision/docker-compose.yaml` ИЛИ `docker/main/docker-compose.yaml`
4. `.github/workflows/build-vision-services.yml` ИЛИ `build-main-services.yml`

**Примеры задач:**
- Создание нового сервиса для сенсора
- Добавление нового ROS2 пакета в контейнер

---

### Фикс проблем с Docker сборкой

**Файлы для контекста:**
1. `docker/vision/<service>/Dockerfile` ИЛИ `docker/main/<service>/Dockerfile`
2. `docs/development/DOCKER_STANDARDS.md` (секция "Правила для Dockerfiles")
3. `docker/vision/docker-compose.yaml` (проверить volumes, environment)
4. `.github/workflows/build-vision-services.yml` (логи GitHub Actions)

**Примеры задач:**
- ModuleNotFoundError - проверить apt-get install
- Конфиг не найден - проверить volumes
- Build timeout - оптимизация слоев

---

## 🎤 Voice Assistant

### Фикс Voice Assistant нод

**Файлы для контекста:**
1. `src/rob_box_voice/rob_box_voice/<node_name>_node.py` (проблемная нода)
2. `docker/vision/voice_assistant/Dockerfile`
3. `docker/vision/docker-compose.yaml` (секция voice-assistant)
4. `docker/vision/config/voice_assistant/voice_assistant.yaml`
5. `docker/vision/config/voice_assistant/voice_assistant_headless.launch.py`

**Примеры задач:**
- DialogueNode падает - проверить DEEPSEEK_API_KEY
- CommandNode не импортирует nav2_msgs - проверить Dockerfile
- TTSNode не работает - проверить YANDEX_API_KEY

---

### Добавление новой ноды в Voice Assistant

**Файлы для контекста:**
1. `docker/vision/config/voice_assistant/voice_assistant_headless.launch.py`
2. Похожая нода (например, `src/rob_box_voice/rob_box_voice/audio_node.py`)
3. `src/rob_box_voice/setup.py` (для entry points)
4. `docker/vision/voice_assistant/Dockerfile` (для зависимостей)

**Примеры задач:**
- Добавить navigation_node
- Создать gesture_node

---

## 🚀 CI/CD & GitHub Actions

### Изменение CI/CD workflow

**Файлы для контекста:**
1. `.github/workflows/build-vision-services.yml` ИЛИ другой workflow
2. `docs/CI_CD_PIPELINE.md`
3. `docs/development/AGENT_GUIDE.md` (секция CI/CD)

**Примеры задач:**
- Добавить новый сервис в build matrix
- Изменить image tag стратегию
- Настроить auto-merge условия

---

### Debug GitHub Actions failures

**Файлы для контекста:**
1. `.github/workflows/<failed_workflow>.yml`
2. `docker/vision/<service>/Dockerfile` ИЛИ `docker/main/<service>/Dockerfile`
3. `docs/CI_CD_PIPELINE.md` (секция Troubleshooting)

**Примеры задач:**
- Build failed - проверить Dockerfile syntax
- Push failed - проверить GHCR credentials
- Test failed - проверить test files

---

## 📦 ROS2 Packages

### Изменение ROS2 ноды

**Файлы для контекста:**
1. `src/<package_name>/<package_name>/<node>.py`
2. `src/<package_name>/package.xml` (зависимости)
3. `src/<package_name>/setup.py` (entry points)
4. `src/<package_name>/config/*.yaml` (параметры)

**Примеры задач:**
- Добавить новый топик
- Изменить параметры ноды
- Фикс import errors

---

### Добавление новой ROS2 ноды

**Файлы для контекста:**
1. Похожая нода в том же пакете
2. `src/<package_name>/setup.py` (для entry points)
3. `src/<package_name>/package.xml` (для зависимостей)
4. Launch файл где нода будет запускаться

**Примеры задач:**
- Создать новый subscriber
- Добавить action server

---

## 🎨 Animation Editor

### Фикс Animation Editor

**Файлы для контекста:**
1. `tools/animation_editor/animation_editor/app.py`
2. `tools/animation_editor/animation_editor/models.py`
3. `tools/animation_editor/main.py`
4. `docs/guides/ANIMATION_EDITOR.md`

**Примеры задач:**
- UI не отображается
- Сохранение не работает
- Ошибка загрузки анимации

---

### Добавление новой фичи в Animation Editor

**Файлы для контекста:**
1. `tools/animation_editor/animation_editor/app.py` (UI logic)
2. `tools/animation_editor/animation_editor/models.py` (Data models)
3. `src/rob_box_animations/animations/manifests/<example>.yaml` (format)
4. `docs/guides/ANIMATION_EDITOR.md` (documentation)

**Примеры задач:**
- Добавить undo/redo
- Создать preview режим
- Экспорт в другой формат

---

## 🔧 Hardware & Sensors

### Фикс OAK-D камеры

**Файлы для контекста:**
1. `docker/vision/docker-compose.yaml` (секция oak-d)
2. `docker/vision/oak-d/Dockerfile`
3. `docker/vision/scripts/oak-d/start_oak_d.sh`
4. `docs/development/AGENT_GUIDE.md` (секция Monitoring)

**Примеры задач:**
- Камера не публикует топики
- Depth image искажена
- FPS низкий

---

### Фикс LSLIDAR

**Файлы для контекста:**
1. `docker/vision/docker-compose.yaml` (секция lslidar)
2. `docker/vision/lslidar/Dockerfile`
3. `docker/vision/config/lslidar/lsx10_custom.yaml`
4. `docker/vision/scripts/lslidar/start_lslidar.sh`

**Примеры задач:**
- Лидар не найден
- Точки облака неправильные
- Конфигурация не применяется

---

## 🔐 Secrets & Security

### Управление секретами

**Файлы для контекста:**
1. `docker/vision/.env.secrets.template`
2. `docker/vision/docker-compose.yaml` (env_file)
3. `docker/vision/.gitignore`
4. `docs/development/AGENT_GUIDE.md` (секция Secrets)

**Примеры задач:**
- Добавить новый API ключ
- Создать .env.secrets на новом Pi
- Проверить что секреты загружены

---

## 📚 Documentation

### Обновление документации

**Файлы для контекста:**
1. Файл который обновляется
2. `docs/README.md` (structure overview)
3. Похожие документы для reference

**Примеры задач:**
- Добавить новый раздел
- Обновить примеры кода
- Фикс broken links

---

## 🧪 Testing

### Добавление тестов

**Файлы для контекста:**
1. Файл который тестируется
2. Похожие тесты в `tests/`
3. `pytest.ini` или `setup.py` (test configuration)

**Примеры задач:**
- Unit test для ноды
- Integration test для сервиса
- Smoke test для Docker

---

## 💡 Быстрые команды

### Найти все файлы по паттерну:

```bash
# Docker compose файлы
find docker/ -name "docker-compose.yaml"

# Все Dockerfiles
find docker/ -name "Dockerfile"

# ROS2 launch файлы
find . -name "*.launch.py"

# Python ноды
find src/ -name "*_node.py"

# YAML конфиги
find docker/ -name "*.yaml" -o -name "*.yml"
```

---

## 🎯 Pro Tips для AI агента

1. **Всегда читай AGENT_GUIDE.md первым** - там основная информация
2. **Проверяй DOCKER_STANDARDS.md перед Docker изменениями** - там критичные правила
3. **Используй CI_CD_PIPELINE.md для понимания workflow** - там полная картина
4. **Держи открытым docker-compose.yaml** - там видна вся структура сервисов
5. **При ошибках сначала проверяй логи** - `docker compose logs -f <service>`

---

## 📊 Метрики использования

**Цель:** Сократить время поиска нужных файлов с 30 минут до 2 минут.

**Как измерять:**
- Time to first relevant file opened
- Number of irrelevant files opened
- Context switches during task

**Target:** 90% задач решаются с правильным контекстом с первого раза.
