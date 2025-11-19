# Исправление: Инвалидация кэша Docker образа robot-state-publisher

**Дата:** 2025-11-19  
**Автор:** GitHub Copilot Agent  
**Статус:** ✅ Исправлено  
**Затронутые компоненты:** GitHub Actions workflows, Docker сборка robot-state-publisher

---

## 📋 Описание проблемы

Docker образ `robot-state-publisher` не пересобирался после изменений в файлах метаданных пакета `rob_box_description`, таких как `package.xml` и `CMakeLists.txt`. Это приводило к использованию устаревших кэшированных слоёв Docker, даже когда важные файлы конфигурации изменялись.

### Симптомы

1. После добавления `package.xml` и `CMakeLists.txt` в коммите `0144a59`, workflow показывал:
   ```
   #5 [2/4] RUN apt-get update && apt-get install -y ...
   #5 CACHED
   
   #6 [3/4] RUN echo "Building with URDF files hash: 1c15c1db5b69a8c7304977a0660f3d06a60a50449d03c7ed37e3e57bb0b0ad7e"
   #6 CACHED
   ```

2. Образ был помечен как успешно собранный, но фактически использовались старые кэшированные слои

3. Хеш URDF_FILES_HASH не изменился, хотя в пакет были добавлены новые файлы

### Пример проблемного запуска

**Workflow Run:** https://github.com/krikz/rob_box_project/actions/runs/19501611468/job/55816894742

---

## 🔍 Корневая причина

Workflow вычислял хеш только для файлов `.xacro` и `.urdf` в поддиректории `urdf/`:

### До исправления

**L-Build Main Pi Services.yml (строка 66):**
```bash
URDF_FILES_HASH=$(find src/rob_box_description/urdf -type f -name "*.xacro" -o -name "*.urdf" | sort | xargs sha256sum | sha256sum | awk '{print $1}')
```

**G-Build Main Pi Services.yml (строка 119):**
```yaml
URDF_FILES_HASH=${{ hashFiles('src/rob_box_description/urdf/**/*.xacro', 'src/rob_box_description/urdf/**/*.urdf') }}
```

### Что упускалось

Следующие важные файлы **НЕ включались** в хеш:
- ✗ `src/rob_box_description/package.xml` - метаданные ROS пакета
- ✗ `src/rob_box_description/CMakeLists.txt` - конфигурация сборки
- ✗ `src/rob_box_description/URDF_EXPORT/` - экспортированные URDF файлы
- ✗ Любые другие файлы вне директории `urdf/`

---

## ✅ Решение

Обновлено вычисление хеша для включения **всех** релевантных файлов во всей директории `src/rob_box_description`.

### После исправления

**L-Build Main Pi Services.yml:**
```bash
# Вычисляем хеш всех важных файлов описания робота для инвалидации кеша при изменении
# Включаем: .xacro, .urdf, package.xml, CMakeLists.txt из src/rob_box_description
URDF_FILES_HASH=$(find src/rob_box_description -type f \
  \( -name "*.xacro" -o -name "*.urdf" -o -name "package.xml" -o -name "CMakeLists.txt" \) \
  | sort | xargs sha256sum | sha256sum | awk '{print $1}')
```

**G-Build Main Pi Services.yml:**
```yaml
- name: Calculate URDF files hash
  id: urdf-hash
  run: |
    # Вычисляем хеш всех важных файлов описания робота
    HASH="${{ hashFiles(
      'src/rob_box_description/**/*.xacro',
      'src/rob_box_description/**/*.urdf',
      'src/rob_box_description/**/package.xml',
      'src/rob_box_description/**/CMakeLists.txt'
    ) }}"
    echo "hash=${HASH}" >> $GITHUB_OUTPUT

- name: Build and push robot-state-publisher
  uses: docker/build-push-action@v5
  with:
    build-args: |
      BASE_IMAGE=${{ env.LOCAL_BASE_REGISTRY || 'ghcr.io/krikz/rob_box_base' }}:ros2-zenoh
      URDF_FILES_HASH=${{ steps.urdf-hash.outputs.hash }}
```

### Что теперь включается

Теперь хеш вычисляется для **всех** следующих файлов:
- ✅ `src/rob_box_description/**/*.xacro` - все Xacro файлы
- ✅ `src/rob_box_description/**/*.urdf` - все URDF файлы
- ✅ `src/rob_box_description/**/package.xml` - метаданные пакета
- ✅ `src/rob_box_description/**/CMakeLists.txt` - конфигурация сборки

**Всего файлов в хеше (на момент исправления):** 9 файлов
```
src/rob_box_description/CMakeLists.txt
src/rob_box_description/URDF_EXPORT/CMakeLists.txt
src/rob_box_description/URDF_EXPORT/package.xml
src/rob_box_description/URDF_EXPORT/urdf/URDF_ROBBOX.xacro
src/rob_box_description/URDF_EXPORT/urdf/materials.xacro
src/rob_box_description/package.xml
src/rob_box_description/urdf/materials/rob_box_materials.xacro
src/rob_box_description/urdf/rob_box.xacro
src/rob_box_description/urdf/rob_box_ros2_control.xacro
```

---

## 📊 Проверка исправления

### Сравнение хешей

**Старый метод (только urdf/):**
```bash
$ find src/rob_box_description/urdf -type f -name "*.xacro" -o -name "*.urdf" | sort | xargs sha256sum | sha256sum | awk '{print $1}'
c877d45bd4f93ff36e96fe6502075a4bacc84132049e0e2fa357fe079e46566e
```

**Новый метод (весь rob_box_description):**
```bash
$ find src/rob_box_description -type f \( -name "*.xacro" -o -name "*.urdf" -o -name "package.xml" -o -name "CMakeLists.txt" \) | sort | xargs sha256sum | sha256sum | awk '{print $1}'
5402d9f5ab90e7e11d4b3aa7c1a6b43d3db028f93c2a9e3fb6854b97dba83492
```

**Результат:** Хеши различаются ✅ - кэш будет правильно инвалидирован

### Тестирование

После применения исправления:
1. При следующем запуске workflow образ **будет пересобран** с новым хешем
2. Любые изменения в `package.xml` или `CMakeLists.txt` **будут триггерить пересборку**
3. Изменения в `URDF_EXPORT/` **также будут учитываться**

---

## 🔧 Изменённые файлы

### Workflow файлы
- `.github/workflows/L-Build Main Pi Services.yml` (строки 65-68)
- `.github/workflows/G-Build Main Pi Services.yml` (строки 106-119, 131)

### Коммиты
- Коммит с исправлением: `bdcfb4f` - "fix: Include package.xml and CMakeLists.txt in robot-state-publisher cache hash"
- PR: `copilot/fix-image-rebuild-issue`

---

## 📚 Связанные документы

- [CI/CD Pipeline](../CI_CD_PIPELINE.md) - Общая документация по CI/CD
- [Docker Tag Management](../DOCKER_TAG_MANAGEMENT.md) - Управление тегами Docker
- [AGENT_GUIDE.md](../development/AGENT_GUIDE.md) - Руководство для AI агентов

---

## 💡 Выводы и рекомендации

### Что мы узнали
1. **Будьте внимательны к контексту кэширования:** При использовании `--build-arg` для инвалидации кэша, убедитесь, что включены ВСЕ релевантные файлы
2. **Используйте широкие паттерны:** Лучше включить больше файлов в хеш, чем пропустить важные
3. **Тестируйте инвалидацию кэша:** Проверяйте, что образы действительно пересобираются при изменениях

### Рекомендации для будущих сборок
1. ✅ Используйте `src/rob_box_description/**/*` вместо `src/rob_box_description/urdf/**/*`
2. ✅ Всегда включайте метаданные пакетов (`package.xml`, `CMakeLists.txt`)
3. ✅ Добавляйте комментарии в workflow, объясняющие, что включается в хеш
4. ✅ При добавлении новых типов файлов в пакет, обновляйте паттерны хеширования

### Потенциальные будущие улучшения
- 🔄 Рассмотреть включение `meshes/` в хеш (если меши изменяются редко, это может не потребоваться)
- 🔄 Рассмотреть включение `rviz/` конфигураций (если они влияют на runtime поведение)
- 🔄 Создать shared action для вычисления хеша, чтобы избежать дублирования логики

---

## ✅ Чеклист проверки

- [x] Проблема идентифицирована и задокументирована
- [x] Корневая причина найдена
- [x] Решение реализовано в обоих workflows (L-Build и G-Build)
- [x] YAML синтаксис проверен (yamllint passed)
- [x] Команда find протестирована локально
- [x] Хеши сравнены (старый vs новый)
- [x] Документация создана
- [ ] Workflow протестирован на реальной сборке
- [ ] PR смержен

---

**Статус:** ✅ Исправление готово к тестированию  
**Следующий шаг:** Запустить workflow для проверки пересборки образа
