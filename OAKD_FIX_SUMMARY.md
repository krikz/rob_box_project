# OAK-D Camera Disconnection Fix - Финальная сводка

## 🎯 Проблема

OAK-D Lite камера отключается после ~8.5 часов непрерывной работы с ошибками:
```
[ERROR] [camera]: No data on logger queue!
[ERROR] [camera]: Camera diagnostics error: Communication exception
Original message 'Couldn't read data from stream: 'sys_logger_queue' (X_LINK_ERROR)'
```

## ✅ Решение

Реализовано комплексное решение из 3-х компонентов:

### 1. USB Power Management
- Отключение USB autosuspend для устройств OAK-D
- Предотвращение автоматического отключения USB портов
- Файл: `docker/vision/scripts/oak-d/setup_usb_power.sh`

### 2. Health Monitoring Watchdog
- Автоматический мониторинг здоровья камеры каждые 30 секунд
- Обнаружение X_LINK_ERROR и других критических ошибок
- Автоматический перезапуск контейнера после 5 ошибок подряд
- Логирование всех событий в `/tmp/oak-d-watchdog.log`
- Файлы: 
  - `docker/vision/scripts/oak-d/watchdog.sh`
  - `docker/vision/scripts/oak-d/oak-d-watchdog.service`
  - `docker/vision/scripts/oak-d/install_watchdog.sh`

### 3. Camera Configuration Optimization
- Уменьшение USB chunk size для стабильности (64 вместо 256 KB)
- Отключение ненужных функций (pipeline dump, calibration dump)
- Файл: `docker/vision/config/oak-d/oak_d_config.yaml`

## 📊 Статистика изменений

```
10 файлов изменено
620 строк добавлено
2 строки удалено
```

### Новые файлы (7):
1. `docker/vision/scripts/oak-d/setup_usb_power.sh` - USB power management
2. `docker/vision/scripts/oak-d/watchdog.sh` - health monitoring
3. `docker/vision/scripts/oak-d/oak-d-watchdog.service` - systemd сервис
4. `docker/vision/scripts/oak-d/install_watchdog.sh` - установочный скрипт
5. `docker/vision/scripts/oak-d/README_OAKD_FIX.md` - подробная документация
6. `OAKD_DISCONNECT_FIX_QUICKSTART.md` - быстрая инструкция
7. `test_oakd_fix.sh` - автоматизированные тесты

### Изменённые файлы (3):
1. `docker/vision/scripts/oak-d/start_oak_d.sh` - добавлен вызов setup_usb_power.sh
2. `docker/vision/config/oak-d/oak_d_config.yaml` - добавлены USB параметры
3. `docs/guides/TROUBLESHOOTING.md` - добавлен новый раздел

## 🔍 Качество кода

- ✅ Все bash скрипты проверены shellcheck (0 warnings)
- ✅ YAML файлы проверены yamllint (0 warnings)
- ✅ Bash синтаксис валидирован
- ✅ Все скрипты имеют правильные права выполнения
- ✅ Автоматические тесты проходят успешно

## 🚀 Установка на Vision Pi

### Быстрая установка (одна команда):

```bash
cd ~/rob_box_project/docker/vision && \
git pull origin main && \
cd scripts/oak-d && \
./install_watchdog.sh && \
cd ~/rob_box_project/docker/vision && \
./update_and_restart.sh
```

### Проверка работы:

```bash
# Статус watchdog сервиса
sudo systemctl status oak-d-watchdog

# Просмотр логов watchdog
tail -f /tmp/oak-d-watchdog.log

# Просмотр логов камеры
docker logs oak-d -f
```

## 📖 Документация

### Для пользователей:
- **Быстрый старт**: `OAKD_DISCONNECT_FIX_QUICKSTART.md`
- **Полное руководство**: `docker/vision/scripts/oak-d/README_OAKD_FIX.md`
- **Устранение неполадок**: `docs/guides/TROUBLESHOOTING.md` (обновлён)

### Для разработчиков:
- **Тесты**: `test_oakd_fix.sh`
- **Код скриптов**: `docker/vision/scripts/oak-d/`

## 🎯 Ожидаемые результаты

После установки:
- ✅ USB устройство не переходит в режим энергосбережения
- ✅ При возникновении X_LINK_ERROR контейнер автоматически перезапускается
- ✅ Камера работает стабильно 24/7 без ручного вмешательства
- ✅ Все события логируются для последующего анализа

## 🔄 Следующие шаги

1. **Установить на Vision Pi** используя команду выше
2. **Мониторить логи** в течение 24+ часов
3. **Проверить стабильность** работы камеры
4. **Настроить параметры** (если нужно):
   - Интервал проверки: `CHECK_INTERVAL` в `watchdog.sh`
   - Порог ошибок: `ERROR_THRESHOLD` в `watchdog.sh`

## 📝 Коммиты

```
a03d893 test: add automated tests for OAK-D watchdog implementation
990a76b fix: apply shellcheck and yamllint fixes to OAK-D scripts
b34bd48 docs: add OAK-D disconnection troubleshooting guide
48ae94c feat(vision): add OAK-D camera watchdog and USB stability fixes
ded0a1e Initial plan
```

## 👥 Авторы

- GOODWORKRINKZ (GitHub Copilot Agent)

## 📅 Дата

11 ноября 2025

---

**Статус**: ✅ Готово к тестированию на реальном железе
