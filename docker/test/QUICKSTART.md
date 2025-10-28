# Quick Start: Testing Rob Box on x86_64

**TL;DR:** Запуск всех сервисов Rob Box на x86_64 машине для разработки и тестирования без Raspberry Pi.

---

## ⚡ 5-минутный старт

### 1. Установка QEMU (один раз)

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y qemu-user-static binfmt-support

# Регистрация в kernel
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Проверка
docker run --rm --platform linux/arm64 arm64v8/ubuntu uname -m
# Должно вывести: aarch64
```

### 2. Запуск тестового окружения

```bash
cd docker/test

# Копировать .env
cp .env.example .env

# Запустить основные сервисы (без hardware)
./scripts/start_test_env.sh

# ИЛИ запустить ВСЕ сервисы (некоторые упадут)
docker compose -f docker-compose-x86-test.yaml up -d
```

### 3. Проверка работы

```bash
# Smoke test
./scripts/smoke_test.sh

# Посмотреть статус
docker compose -f docker-compose-x86-test.yaml ps

# Логи
docker compose -f docker-compose-x86-test.yaml logs -f
```

### 4. Остановка

```bash
./scripts/stop_test_env.sh
```

---

## 📊 Что работает, что нет

### ✅ Работает отлично

- **Zenoh router** - коммуникация ROS2 нод
- **robot-state-publisher** - URDF модель и TF дерево
- **twist-mux** - мультиплексирование команд движения
- **perception** - агрегация контекста робота

**Использование:** Разработка логики нод, тестирование коммуникации

### ⚠️ Работает, но без данных

- **rtabmap** - ждёт топики от камеры и лидара
- **nav2** - ждёт карту и /scan топик

**Использование:** Тестирование запуска, проверка параметров

### ❌ Не работает (требуют hardware)

- **oak-d** - нет USB камеры
- **lslidar** - нет serial устройства
- **voice-assistant** - нет микрофона
- **led-matrix** - нет SPI
- **micro-ros-agent** - нет UART
- **vesc-nexus** - нет VESC контроллера

**Ожидаемое поведение:** Падают с ошибками о missing devices - **это нормально**

---

## 🎯 Use Cases

### Use Case 1: Тестирование новой ноды

```bash
# 1. Собрать образ для arm64
docker buildx build --platform linux/arm64 -t my-new-node:test .

# 2. Добавить в docker-compose-x86-test.yaml
services:
  my-new-node:
    image: my-new-node:test
    platform: linux/arm64
    network_mode: host
    environment:
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
      - ZENOH_SESSION_CONFIG_URI=/config/zenoh_session_config.json5
    volumes:
      - ./config:/config:ro
    depends_on:
      - zenoh-router

# 3. Запустить
docker compose -f docker-compose-x86-test.yaml up -d my-new-node

# 4. Проверить логи
docker compose -f docker-compose-x86-test.yaml logs -f my-new-node
```

### Use Case 2: Integration Testing

```bash
# Запустить несколько нод и проверить их взаимодействие
cd docker/test
./scripts/start_test_env.sh

# В другом терминале - мониторинг топиков
./scripts/monitor_topics.sh continuous

# Проверить что ноды публикуют/подписаны на топики
curl http://localhost:8000/@/local/publisher | jq
curl http://localhost:8000/@/local/subscriber | jq
```

### Use Case 3: Smoke Testing перед деплоем

```bash
# 1. Собрать новую версию сервиса
cd ../../
IMAGE_TAG=test ./scripts/local-build.sh rtabmap linux/arm64

# 2. Обновить .env
echo "IMAGE_TAG=test" >> docker/test/.env

# 3. Запустить и проверить
cd docker/test
docker compose -f docker-compose-x86-test.yaml up -d rtabmap
./scripts/smoke_test.sh

# 4. Если OK - деплоить на Raspberry Pi
```

---

## 🔍 Мониторинг и отладка

### Zenoh REST API

```bash
# Статус роутера
curl http://localhost:8000/@/local/router | jq

# Список publishers
curl http://localhost:8000/@/local/publisher | jq -r 'keys[]'

# Список subscribers
curl http://localhost:8000/@/local/subscriber | jq -r 'keys[]'
```

### ROS2 CLI (если установлен локально)

```bash
# Настроить окружение
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="$(pwd)/config/zenoh_session_config.json5"

# Список топиков
ros2 topic list

# Эхо топика
ros2 topic echo /robot_description

# Информация о ноде
ros2 node info /robot_state_publisher

# TF дерево
ros2 run tf2_tools view_frames
```

### Логи контейнеров

```bash
# Все логи
docker compose -f docker-compose-x86-test.yaml logs -f

# Конкретный сервис
docker compose -f docker-compose-x86-test.yaml logs -f rtabmap

# Последние N строк
docker compose -f docker-compose-x86-test.yaml logs --tail=100 perception

# Только ошибки
docker compose -f docker-compose-x86-test.yaml logs | grep -i error
```

---

## ⚠️ Частые проблемы

### Проблема: "exec format error"

**Причина:** QEMU не установлен или не зарегистрирован

**Решение:**
```bash
sudo apt-get install --reinstall qemu-user-static binfmt-support
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

### Проблема: "no matching manifest for linux/arm64"

**Причина:** Образ не собран для ARM64

**Решение:**
```bash
# Проверить что образ существует для arm64
docker manifest inspect ghcr.io/krikz/rob_box:rtabmap-humble-latest

# Если нет - собрать локально
cd ../..
./scripts/local-build.sh rtabmap linux/arm64
```

### Проблема: Zenoh connection timeout

**Причина:** Zenoh router не запущен или неправильная конфигурация

**Решение:**
```bash
# Проверить что router работает
docker compose -f docker-compose-x86-test.yaml ps zenoh-router

# Перезапустить router
docker compose -f docker-compose-x86-test.yaml restart zenoh-router

# Проверить порты
ss -tulnp | grep 7447
ss -tulnp | grep 8000
```

### Проблема: Всё очень медленно

**Причина:** QEMU emulation 5-10x медленнее native

**Решение:** Это нормально для эмуляции. Рекомендации:
- Используйте только для smoke testing
- Не запускайте все сервисы одновременно
- Выделите больше CPU для Docker
- Не используйте для performance-critical задач

```bash
# Запускать только нужные сервисы
docker compose -f docker-compose-x86-test.yaml up -d \
  zenoh-router robot-state-publisher twist-mux perception
```

---

## 📚 Дополнительные ресурсы

- **Теория и исследование:** [ARM64_EMULATION_ON_X86.md](../docs/development/ARM64_EMULATION_ON_X86.md)
- **Полное руководство:** [docker/test/README.md](README.md)
- **Docker стандарты:** [DOCKER_STANDARDS.md](../docs/development/DOCKER_STANDARDS.md)
- **CI/CD pipeline:** [CI_CD_PIPELINE.md](../docs/CI_CD_PIPELINE.md)

---

## 🎓 Best Practices

### DO ✅

- Используйте для smoke testing перед деплоем
- Тестируйте логику ROS2 нод
- Проверяйте коммуникацию между сервисами
- Разрабатывайте без наличия Raspberry Pi
- Запускайте только нужные сервисы

### DON'T ❌

- Не используйте для performance benchmarking
- Не тестируйте hardware-зависимый функционал
- Не запускайте в production
- Не ожидайте native скорости
- Не запускайте SLAM/vision если не нужно

---

## 💡 Pro Tips

1. **Сохраняйте состояние:** Используйте volumes для persistent данных
2. **Mock данные:** Создавайте bag файлы для тестирования RTAB-Map без камеры
3. **Мониторинг:** Используйте Zenoh REST API для быстрой проверки
4. **Selective testing:** Запускайте только релевантные сервисы
5. **CI/CD integration:** Используйте для автоматических тестов в GitHub Actions

---

**Создано:** October 28, 2025  
**Версия:** 1.0.0
