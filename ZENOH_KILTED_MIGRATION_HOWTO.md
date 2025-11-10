# Практическое руководство: Миграция на ROS 2 Kilted (для RPi 5 + Docker)

**Дата:** 2025-11-10  
**Статус:** ✅ Docker образы для ARM64 доступны  
**Рекомендация:** Можно попробовать на тестовой среде

---

## 🎯 Обновлённая оценка для вашей конфигурации

### Ваша конфигурация:
- ✅ **Raspberry Pi 5** (не Pi 4!)
- ✅ **Всё под Docker** (изолированная среда)
- ✅ **Ubuntu/Debian ARM64** (совместимо с Kilted)

### Доступность Docker образов:

| Компонент | Humble (текущий) | Kilted | Статус ARM64 |
|-----------|------------------|--------|--------------|
| **ROS 2 Base** | ros:humble-ros-base | ros:kilted-ros-base | ✅ Доступен |
| **RTAB-Map** | Собственная сборка | introlab3it/rtabmap_ros:kilted-latest | ✅ Доступен |
| **Zenoh Router** | eclipse/zenoh:latest | eclipse/zenoh:latest | ✅ Универсальный |
| **DepthAI** | luxonis/depthai | luxonis/depthai | ✅ Универсальный |
| **VESC** | Собственная сборка | Требует пересборка | ⚠️ Нужна адаптация |
| **Nav2** | ros:humble-navigation | ros:kilted-navigation | ✅ Доступен |

**Вывод:** Миграция на Kilted ВОЗМОЖНА и БЕЗОПАСНЕЕ чем предполагалось!

---

## 📊 Стратегия миграции

### Вариант 1: Постепенная миграция (РЕКОМЕНДУЕТСЯ)

**Этап 1:** Тестовая ветка + базовые сервисы
**Этап 2:** Критичные сервисы (RTAB-Map, Camera)
**Этап 3:** Полная система
**Этап 4:** Production деплой

### Вариант 2: Параллельная конфигурация

Держать обе версии (Humble и Kilted) на разных ветках:
- `main` - Humble (стабильная production)
- `ros2/kilted` - Kilted (тестирование)

Переключаться через `git checkout` при необходимости

---

## 🚀 План миграции (пошаговый)

### Этап 1: Подготовка (1-2 дня)

#### 1.1. Создать ветку для миграции

```bash
# На локальной машине
cd ~/rob_box_project
git checkout develop
git pull origin develop
git checkout -b ros2/kilted
```

#### 1.2. Обновить базовые образы

**Файл:** `docker/base/Dockerfile.ros2-zenoh`

```dockerfile
# Было:
FROM ros:humble-ros-base

# Стало:
FROM ros:kilted-ros-base

# Остальное без изменений (rmw_zenoh_cpp уже в Kilted!)
RUN apt-get update && \
    apt-get install -y \
        ros-kilted-rmw-zenoh-cpp \
        git wget curl python3-pip \
        && rm -rf /var/lib/apt/lists/*

# Переменные окружения
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    RUST_LOG=zenoh=info \
    LD_LIBRARY_PATH=/opt/ros/kilted/opt/zenoh_cpp_vendor/lib:/opt/ros/kilted/lib
```

#### 1.3. Обновить RTAB-Map образ

**Файл:** `docker/main/rtabmap/Dockerfile`

```dockerfile
# Было:
FROM ghcr.io/krikz/rob_box:ros2-zenoh-humble-latest AS base
# или
FROM introlab3it/rtabmap_ros:humble-latest

# Стало:
FROM introlab3it/rtabmap_ros:kilted-latest

# Копирование конфигов остаётся без изменений!
# (они монтируются через volumes)
```

#### 1.4. Обновить остальные сервисы

**Глобальная замена в Dockerfiles:**

```bash
# На локальной машине
cd ~/rob_box_project

# Найти все Dockerfiles
find docker -name "Dockerfile*" -type f

# Глобальная замена (осторожно!)
find docker -name "Dockerfile*" -type f -exec sed -i 's/humble/kilted/g' {} \;

# Проверить изменения
git diff docker/
```

**⚠️ Внимание:** Проверьте каждый Dockerfile вручную! Не все пакеты могут иметь прямую замену.

#### 1.5. Обновить docker-compose.yaml

**Изменить image tags:**

```yaml
# docker/main/docker-compose.yaml
services:
  rtabmap:
    # Было:
    image: ghcr.io/krikz/rob_box:rtabmap-humble-latest
    # Стало:
    image: ghcr.io/krikz/rob_box:rtabmap-kilted-latest
    # ИЛИ используйте официальный образ:
    image: introlab3it/rtabmap_ros:kilted-latest
    
  nav2:
    # Было:
    image: ghcr.io/krikz/rob_box:nav2-humble-latest
    # Стало:
    image: ghcr.io/krikz/rob_box:nav2-kilted-latest
```

---

### Этап 2: Локальное тестирование (2-3 дня)

#### 2.1. Собрать базовые образы локально

```bash
# На локальной машине или Build Machine
cd ~/rob_box_project/docker/base

# Собрать ros2-zenoh образ
docker build -f Dockerfile.ros2-zenoh \
  -t ghcr.io/krikz/rob_box:ros2-zenoh-kilted-dev \
  --platform linux/arm64 .

# Проверить образ
docker run --rm ghcr.io/krikz/rob_box:ros2-zenoh-kilted-dev \
  bash -c "source /opt/ros/kilted/setup.bash && ros2 --version"

# Ожидается: ros2 version kilted
```

#### 2.2. Собрать сервисы

```bash
# Пример: OAK-D camera
cd ~/rob_box_project/docker/vision/oak-d

docker build -t ghcr.io/krikz/rob_box:oak-d-kilted-dev \
  --platform linux/arm64 .
```

#### 2.3. Тестовый запуск (локально или на одном Pi)

```bash
# На Vision Pi - тестовый запуск камеры
cd ~/rob_box_project/docker/vision

# Временно изменить image в docker-compose.yaml
# oak-d:
#   image: ghcr.io/krikz/rob_box:oak-d-kilted-dev

docker compose up oak-d

# Проверить в логах:
# - Нет ошибок запуска
# - ROS 2 топики публикуются
# - Zenoh соединение работает
```

---

### Этап 3: Тестирование на Raspberry Pi (1 неделя)

#### 3.1. Развернуть на одном Pi (Vision Pi)

```bash
# Закоммитить изменения в ветку
git add .
git commit -m "feat: migrate Vision Pi services to ROS 2 Kilted"
git push origin ros2/kilted

# На Vision Pi - переключиться на ветку
sshpass -p 'open' ssh ros2@10.1.1.21 << 'EOF'
cd ~/rob_box_project
git fetch origin
git checkout ros2/kilted
git pull origin ros2/kilted
EOF
```

#### 3.2. Пересобрать/обновить образы

**Вариант A: Использовать официальные образы** (быстро)

```bash
# На Vision Pi
cd ~/rob_box_project/docker/vision
docker compose pull  # Скачать kilted образы
docker compose up -d  # Запустить
```

**Вариант B: Собрать локально** (медленно, но контролируемо)

```bash
# На Build Machine
cd ~/rob_box_project/docker/vision/oak-d
docker buildx build --platform linux/arm64 \
  -t localhost:5000/rob_box:oak-d-kilted-dev \
  --push .

# На Vision Pi - использовать локальный registry
docker pull localhost:5000/rob_box:oak-d-kilted-dev
```

#### 3.3. Мониторинг и тестирование

**Проверка 1: Zenoh соединение**
```bash
# Vision Pi
docker logs zenoh-router --tail 50
# Искать: "Session opened", нет "Unable to push" ошибок
```

**Проверка 2: ROS 2 топики**
```bash
# Любой Pi
ros2 topic list
ros2 topic hz /camera/rgb/image_raw
ros2 topic hz /scan
```

**Проверка 3: RTAB-Map работает**
```bash
# Main Pi
docker logs rtabmap --tail 100
# Искать: "RTAB-Map started", "Receiving images", нет ошибок
```

#### 3.4. Нагрузочное тестирование

```bash
# Запустить ВСЮ систему на 24 часа
cd ~/rob_box_project/docker/vision
docker compose up -d

cd ~/rob_box_project/docker/main
docker compose up -d

# Мониторить логи zenoh-router
docker logs -f zenoh-router 2>&1 | grep -E "(ERROR|Unable to push)"
```

---

### Этап 4: Оценка результатов (3-5 дней)

#### Метрики для сравнения:

| Метрика | Humble (baseline) | Kilted | Улучшение |
|---------|-------------------|--------|-----------|
| **Ошибки "Unable to push" (за 24ч)** | ~50-100 | ? | ? |
| **Стабильность топиков** | 70-80% | ? | ? |
| **CPU usage (Vision Pi)** | ~60% | ? | ? |
| **CPU usage (Main Pi)** | ~70% | ? | ? |
| **Network bandwidth** | ~15-20 MB/s | ? | ? |
| **Memory usage** | ~2.5 GB | ? | ? |

#### Сценарии успеха/провала:

**✅ УСПЕХ - мигрировать в production:**
- Ошибки "Unable to push" < 10 за 24 часа
- Стабильность топиков > 90%
- Нет новых критичных ошибок
- Производительность не хуже чем на Humble

**⚠️ ЧАСТИЧНЫЙ УСПЕХ - продолжить тестирование:**
- Ошибки сократились на 30-50%
- Есть мелкие проблемы, но решаемые
- Производительность сопоставима

**❌ ПРОВАЛ - откатиться на Humble:**
- Ошибки не уменьшились или увеличились
- Появились новые критичные проблемы
- Производительность значительно хуже

---

## 🔄 Откат на Humble (если нужно)

### Быстрый откат

```bash
# На обоих Pi
cd ~/rob_box_project
git checkout develop  # или main
git pull origin develop
cd docker/vision  # или docker/main
docker compose down
docker compose pull
docker compose up -d
```

### Сохранение логов перед откатом

```bash
# Vision Pi
docker logs zenoh-router > /tmp/zenoh_kilted_vision.log 2>&1
docker logs oak-d > /tmp/oak-d_kilted.log 2>&1

# Main Pi
docker logs zenoh-router > /tmp/zenoh_kilted_main.log 2>&1
docker logs rtabmap > /tmp/rtabmap_kilted.log 2>&1

# Скопировать логи на локальную машину для анализа
sshpass -p 'open' scp ros2@10.1.1.21:/tmp/*_kilted.log ./logs/
sshpass -p 'open' scp ros2@10.1.1.20:/tmp/*_kilted.log ./logs/
```

---

## 📋 Чеклист миграции

### Подготовка
- [ ] Создана ветка `ros2/kilted`
- [ ] Обновлены базовые Dockerfiles (humble → kilted)
- [ ] Проверены доступные образы (RTAB-Map, Nav2)
- [ ] Изучены breaking changes API ROS 2 Kilted

### Локальное тестирование
- [ ] Собраны базовые образы (ros2-zenoh)
- [ ] Собраны сервисы (oak-d, lslidar, и т.д.)
- [ ] Тестовый запуск на локальной машине
- [ ] Проверка ROS 2 топиков

### Тестирование на Pi
- [ ] Развёрнуто на Vision Pi
- [ ] Проверка Zenoh соединения
- [ ] Проверка всех ROS топиков
- [ ] Нагрузочное тестирование 24 часа
- [ ] Развёрнуто на Main Pi
- [ ] Проверка RTAB-Map работы
- [ ] Полное системное тестирование 48 часов

### Оценка
- [ ] Собраны метрики производительности
- [ ] Сравнение с Humble baseline
- [ ] Решение: успех/частичный успех/провал
- [ ] Документированы findings

### Production (если успешно)
- [ ] Merge `ros2/kilted` → `develop`
- [ ] CI/CD сборка Kilted образов
- [ ] Обновление документации
- [ ] Развёртывание на production

---

## 💡 Советы и подводные камни

### Частые проблемы и решения

**1. Отсутствующие пакеты**
```bash
# Ошибка: ros-kilted-<package> not found
# Решение: Проверить доступность пакета
apt-cache search ros-kilted-<package>

# Если нет - собрать из исходников
```

**2. Breaking changes API**
```python
# Пример: Некоторые message types могут измениться
# Читать: https://docs.ros.org/en/kilted/Releases/Kilted-Kaiju-Complete-Changelog.html
```

**3. Проблемы совместимости Zenoh**
```bash
# Убедиться что все роутеры используют одинаковую конфигурацию
diff docker/main/config/zenoh_router_config.json5 \
     docker/vision/config/zenoh_router_config.json5
```

### Best Practices

1. **Одновременная миграция обоих Pi** - не мешайте Humble и Kilted
2. **Backup конфигураций** перед изменениями
3. **Детальное логирование** во время тестирования
4. **Сохранение метрик** для сравнения
5. **Документирование проблем** и решений

---

## 📚 Ссылки

- [ROS 2 Kilted Release Notes](https://docs.ros.org/en/kilted/Releases/Release-Kilted-Kaiju.html)
- [ROS 2 Kilted Complete Changelog](https://docs.ros.org/en/rolling/Releases/Kilted-Kaiju-Complete-Changelog.html)
- [RTAB-Map Kilted Docker Image](https://hub.docker.com/r/introlab3it/rtabmap_ros/tags?name=kilted)
- [ROS Docker Images](https://hub.docker.com/_/ros)
- [Raspberry Pi 5 ROS 2 Guide](https://docs.ros.org/en/kilted/How-To-Guides/Installing-on-Raspberry-Pi.html)

---

**Автор:** AI Agent  
**Дата:** 2025-11-10  
**Статус:** ✅ Готово к тестированию  
**Рекомендация:** Попробовать на тестовой ветке, оценить через 1 неделю
