# Инвалидация кэша Docker при изменениях субмодулей и внешних файлов

## Проблема

При использовании git submodules или внешних файлов (URDF, конфиги) в Docker build контексте, Docker не отслеживает изменения в этих файлах. Это приводит к следующей ситуации:

1. Субмодуль обновляется (новый коммит)
2. Docker build запускается
3. Docker видит что `COPY src/submodule/...` не изменился (путь тот же)
4. Docker использует кэш и не копирует новые файлы
5. В результате образ содержит старый код из субмодуля

## Решение

Мы используем механизм **ARG с SHA субмодуля** для принудительной инвалидации кэша.

### Как это работает

#### 1. В Dockerfile добавляем ARG

```dockerfile
ARG ROS_DISTRO=kilted
ARG BASE_IMAGE=ghcr.io/krikz/rob_box_base:ros2-zenoh
ARG ROS2LEDS_SHA=unknown  # SHA субмодуля

FROM ${BASE_IMAGE}
```

#### 2. Используем ARG перед COPY

```dockerfile
# Инвалидация кэша при изменении субмодуля ros2leds
# Этот ARG используется для принудительной пересборки при обновлении субмодуля
RUN echo "Building with ros2leds SHA: ${ROS2LEDS_SHA}"

# Теперь COPY будет выполнен заново, т.к. предыдущий слой изменился
COPY src/ros2leds/led_matrix_driver /ws/src/led_matrix_driver
```

#### 3. В workflow передаём SHA через build-arg

**Локальные runners (L-workflows):**
```yaml
- name: Build led-matrix
  run: |
    # Получаем SHA субмодуля
    ROS2LEDS_SHA=$(git submodule status src/ros2leds | awk '{print $1}' | sed 's/^[+-]//')
    
    docker buildx build \
      --build-arg="ROS2LEDS_SHA=${ROS2LEDS_SHA}" \
      ...
```

**GitHub-hosted runners (G-workflows):**
```yaml
- name: Build and push led-matrix
  uses: docker/build-push-action@v5
  with:
    build-args: |
      BASE_IMAGE=...
      ROS2LEDS_SHA=${{ hashFiles('src/ros2leds/**') }}
```

### Принцип работы

1. **Субмодуль изменяется** → SHA коммита в субмодуле меняется
2. **Workflow получает новый SHA** → через `git submodule status` или `hashFiles`
3. **Docker получает новый ARG** → `--build-arg ROS2LEDS_SHA=новый_sha`
4. **ARG изменился** → кэш для этого слоя инвалидируется
5. **RUN echo выполняется заново** → все последующие слои тоже
6. **COPY выполняется заново** → новые файлы попадают в образ

## Затронутые сервисы

### Vision Pi
- **led-matrix** → использует субмодуль `ros2leds` → ARG `ROS2LEDS_SHA`

### Main Pi
- **ros2-control** → использует субмодуль `vesc_nexus` → ARG `VESC_NEXUS_SHA`
- **vesc-nexus** (standalone) → использует субмодуль `vesc_nexus` → ARG `VESC_NEXUS_SHA`
  - **Примечание:** В production используется только ros2-control образ
  - vesc-nexus Dockerfile сохранён для разработки и тестирования
- **robot-state-publisher** → использует URDF файлы из `src/rob_box_description/urdf/` → ARG `URDF_FILES_HASH`
  - **Примечание:** URDF файлы не копируются в образ, монтируются через volumes
  - Хеш вычисляется из всех `.xacro` и `.urdf` файлов для инвалидации кеша

## Тестирование

Запустите скрипт проверки:
```bash
./scripts/test_submodule_cache_invalidation.sh
```

Скрипт проверяет:
- ✓ Наличие ARG в Dockerfiles
- ✓ Использование ARG в RUN echo
- ✓ Передачу SHA в workflows

## Важные замечания

### ❗ Когда добавлять новый субмодуль

Если вы добавляете новый субмодуль в проект:

1. **Добавьте ARG в Dockerfile:**
   ```dockerfile
   ARG YOUR_SUBMODULE_SHA=unknown
   ```

2. **Используйте ARG перед COPY:**
   ```dockerfile
   RUN echo "Building with your_submodule SHA: ${YOUR_SUBMODULE_SHA}"
   COPY src/your_submodule/... /ws/src/...
   ```

3. **Обновите workflow:**
   ```yaml
   YOUR_SUBMODULE_SHA=$(git submodule status src/your_submodule | awk '{print $1}' | sed 's/^[+-]//')
   --build-arg="YOUR_SUBMODULE_SHA=${YOUR_SUBMODULE_SHA}"
   ```

4. **Обновите тест:**
   Добавьте проверки в `scripts/test_submodule_cache_invalidation.sh`

### ⚠️ Альтернативные подходы (не используются)

**Почему не используем `--no-cache`:**
- Слишком радикально — пересобирает ВСЁ
- Долгая сборка (apt-get, pip install каждый раз заново)
- Нет смысла пересобирать базовые слои

**Почему не используем `.git` в контексте:**
- Увеличивает размер build context
- Требует сложной логики в Dockerfile
- Менее явно что происходит

**Почему не используем `COPY .gitmodules`:**
- Не отслеживает фактические изменения в коде
- Только факт что субмодуль подключен

### 📝 Случай с URDF файлами (robot-state-publisher)

URDF файлы представляют особый случай — они **НЕ копируются в образ**, а монтируются через volumes в runtime. Это создаёт проблему: Docker не видит изменений в URDF файлах.

**Решение:** Вычисляем хеш всех URDF/xacro файлов и передаём как build argument.

#### Пример для robot-state-publisher

**В Dockerfile:**
```dockerfile
ARG URDF_FILES_HASH=unknown

# Инвалидация кеша при изменении URDF файлов
ARG URDF_FILES_HASH
RUN echo "Building with URDF files hash: ${URDF_FILES_HASH}"
```

**В L-workflow (локальные runners):**
```bash
# Вычисляем хеш всех URDF/xacro файлов
URDF_FILES_HASH=$(find src/rob_box_description/urdf -type f \( -name "*.xacro" -o -name "*.urdf" \) | sort | xargs sha256sum | sha256sum | awk '{print $1}')

docker buildx build \
  --build-arg="URDF_FILES_HASH=${URDF_FILES_HASH}" \
  ...
```

**В G-workflow (GitHub-hosted runners):**
```yaml
build-args: |
  BASE_IMAGE=...
  URDF_FILES_HASH=${{ hashFiles('src/rob_box_description/urdf/**/*.xacro', 'src/rob_box_description/urdf/**/*.urdf') }}
```

**Почему не копируем URDF в образ:**
- Соответствует принципу проекта: "конфиги монтируются, не копируются"
- URDF файлы — это конфигурация робота, может меняться между запусками
- Позволяет обновлять URDF без пересборки образа (в development режиме)
- Единообразие с другими конфигами (Zenoh, launch файлы)

## Отладка

### Проверка что SHA передаётся

В логах сборки должно быть:
```
Building with ros2leds SHA: de19bb348ba22c026e3696771839c35c7066a4b2
```

### Проверка что кэш инвалидируется

При изменении субмодуля в логах должно быть:
```
#6 [8/14] RUN echo "Building with ros2leds SHA: de19bb348ba..."
#6 0.234 Building with ros2leds SHA: de19bb348ba22c026e3696771839c35c7066a4b2
#6 DONE 0.3s
```

Если вместо этого `#6 CACHED` — механизм не работает!

### Ручная пересборка с новым SHA

```bash
# Получить текущий SHA
ROS2LEDS_SHA=$(git submodule status src/ros2leds | awk '{print $1}' | sed 's/^[+-]//')
echo "Current SHA: $ROS2LEDS_SHA"

# Собрать с явным указанием SHA
docker build \
  --build-arg="ROS2LEDS_SHA=${ROS2LEDS_SHA}" \
  --file docker/vision/led_matrix/Dockerfile \
  .
```

## Ссылки

- Оригинальный issue: commit `8a1716e` "chore(submodule): update ros2leds to fix volume config loading"
- Изменения в субмодуле: `de19bb3` "fix: use volume configs instead of embedded ones"
- PR с исправлением: `copilot/check-submodule-build-issue`
