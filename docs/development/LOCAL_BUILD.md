# Локальное тестирование сборки Docker образов

## Требования

1. **Docker** установлен
2. **QEMU** для эмуляции ARM64:
   ```bash
   sudo apt-get install qemu qemu-user-static binfmt-support
   ```

3. **Docker Buildx** (входит в Docker 19.03+)

## Быстрый старт

### 1. Подготовка окружения

```bash
# Регистрируем QEMU для ARM64
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Создаем buildx builder
docker buildx create --name arm64-builder --use

# Проверяем поддержку платформ
docker buildx inspect --bootstrap
```

### 2. Запуск тестовой сборки

```bash
# Запустить тест всех проблемных образов
sudo ./scripts/test_docker_local_arm64.sh

# Или отдельный образ:
docker buildx build \
  --platform linux/arm64 \
  --file docker/main/micro_ros_agent/Dockerfile \
  --tag rob_box_test:micro-ros-agent \
  . 
```

## Важно: Context сборки

**Все Dockerfiles используют context = корень репозитория!**

```yaml
# GitHub Actions workflow:
- name: Build and push image
  uses: docker/build-push-action@v5
  with:
    context: .  # ← Корень репозитория!
    file: docker/main/some_service/Dockerfile
```

Поэтому локально собираем так:

```bash
cd /путь/к/rob_box_project  # ← Корень!
docker buildx build -f docker/main/micro_ros_agent/Dockerfile .
#                                                              ^ точка = корень
```

**НЕ делайте так:**
```bash
# ❌ НЕПРАВИЛЬНО
cd docker/main/micro_ros_agent
docker build .  
# COPY src/robot_sensor_hub_msg не найдет файлы!
```

## Проверка конкретного образа

### micro-ros-agent
```bash
docker buildx build \
  --platform linux/arm64 \
  --file docker/main/micro_ros_agent/Dockerfile \
  --tag test-micro-ros:local \
  --progress=plain \
  .
```

### nav2
```bash
docker buildx build \
  --platform linux/arm64 \
  --file docker/main/nav2/Dockerfile \
  --tag test-nav2:local \
  --progress=plain \
  .
```

### vesc_nexus
```bash
# Важно: submodule должен быть инициализирован
git submodule update --init --recursive

docker buildx build \
  --platform linux/arm64 \
  --file docker/main/vesc_nexus/Dockerfile \
  --tag test-vesc:local \
  --progress=plain \
  .
```

### led_matrix
```bash
# Важно: submodule ros2leds
git submodule update --init --recursive

docker buildx build \
  --platform linux/arm64 \
  --file docker/vision/led_matrix/Dockerfile \
  --tag test-led:local \
  --progress=plain \
  .
```

## Отладка ошибок

### Остановка на первой ошибке
```bash
docker buildx build \
  --file docker/main/micro_ros_agent/Dockerfile \
  --platform linux/arm64 \
  --progress=plain \
  . 2>&1 | tee build.log
```

### Сохранение промежуточных слоев
```bash
docker buildx build \
  --file docker/main/micro_ros_agent/Dockerfile \
  --platform linux/arm64 \
  --progress=plain \
  --no-cache \
  . 
```

### Запуск с отладочным shell
```bash
# Добавить в Dockerfile перед проблемной строкой:
RUN ls -la /ws/src  # Проверить что файлы скопированы
```

## Производительность

⚠️ **ARM64 эмуляция медленная!**
- Обычная сборка: ~5-10 минут
- На QEMU: ~20-40 минут
- Используйте для финальной проверки перед push

### Быстрая проверка (AMD64)
```bash
# Только для проверки синтаксиса Dockerfile
docker build \
  --file docker/main/micro_ros_agent/Dockerfile \
  --platform linux/amd64 \
  --target=build_stage \  # Если есть multi-stage
  .
```

## Очистка

### Удалить тестовые образы
```bash
docker images | grep rob_box_test | awk '{print $3}' | xargs docker rmi -f
```

### Удалить buildx builder
```bash
docker buildx rm arm64-builder
```

### Очистить buildx cache
```bash
docker buildx prune -a -f
```

## Сравнение с GitHub Actions

| Аспект | GitHub Actions | Локально |
|--------|----------------|----------|
| Платформа | ARM64 (native runner) | AMD64 + QEMU (эмуляция) |
| Скорость | Быстро (~5-10 мин) | Медленно (~20-40 мин) |
| Cache | GitHub Actions cache | Локальный cache |
| Результат | Push в registry | Локальный образ |

## Troubleshooting

### "no matching manifest for linux/amd64"
**Проблема:** Пытаетесь собрать без указания `--platform linux/arm64`

**Решение:**
```bash
docker buildx build --platform linux/arm64 ...
```

### "COPY failed: file not found"
**Проблема:** Неправильный context сборки

**Решение:** Context должен быть корень репозитория:
```bash
cd /path/to/rob_box_project
docker buildx build -f docker/main/.../Dockerfile .
#                                                  ^ корень!
```

### "cannot load arm64 image"
**Проблема:** `--load` работает только для native платформы

**Решение:** Для ARM64 используйте:
```bash
docker buildx build --platform linux/arm64 ... --output type=docker
# Или
docker buildx build --platform linux/arm64 ... --push  # В registry
```

---

**Автор:** GitHub Copilot  
**Дата:** 2025-10-11
