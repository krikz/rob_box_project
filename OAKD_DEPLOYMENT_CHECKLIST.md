# Чек-лист перед развёртыванием OAK-D Fix на Vision Pi

## ✅ Предварительная проверка (выполнено)

- [x] Все скрипты проверены shellcheck
- [x] YAML конфиги проверены yamllint  
- [x] Bash синтаксис валидирован
- [x] Автоматические тесты пройдены (`./test_oakd_fix.sh`)
- [x] Документация создана (3 файла)
- [x] Все изменения закоммичены

## 📋 Шаги развёртывания на Vision Pi

### 1. Подготовка (на локальной машине)

```bash
# Проверить что ветка готова
git log --oneline -5

# Слить в main (или создать PR)
git checkout main
git merge copilot/fix-oakd-disconnection-issue
git push origin main
```

### 2. Развёртывание на Vision Pi (ros2@10.1.1.21)

```bash
# SSH на Vision Pi
sshpass -p 'open' ssh ros2@10.1.1.21

# Обновить код
cd ~/rob_box_project/docker/vision
git pull origin main

# Установить watchdog сервис
cd scripts/oak-d
./install_watchdog.sh

# Перезапустить контейнер с новыми настройками
cd ~/rob_box_project/docker/vision
./update_and_restart.sh
```

### 3. Проверка работы

```bash
# На Vision Pi

# 1. Статус watchdog сервиса
sudo systemctl status oak-d-watchdog
# Ожидаем: ● oak-d-watchdog.service - OAK-D Camera Health Monitor Watchdog
#          Loaded: loaded (/etc/systemd/system/oak-d-watchdog.service; enabled)
#          Active: active (running)

# 2. Логи watchdog
tail -f /tmp/oak-d-watchdog.log
# Ожидаем: "🐕 OAK-D Watchdog запущен"

# 3. Логи контейнера камеры
docker logs oak-d --tail 50
# Ожидаем: "🔌 Настройка USB power management..."
#          "✅ USB power management настроен"
#          "[INFO] [camera]: Camera ready!"

# 4. Проверка USB настроек
lsusb | grep -i "03e7"
# Ожидаем: устройство Movidius/OAK-D

# 5. Проверка топиков ROS 2
ros2 topic list | grep camera
# Ожидаем: /camera/camera/color/image_raw
#          /camera/camera/depth/image_raw
```

### 4. Мониторинг (первые 24 часа)

```bash
# Периодически проверять логи watchdog
tail -f /tmp/oak-d-watchdog.log

# Если видим перезапуски:
# [YYYY-MM-DD HH:MM:SS] 🔄 КРИТИЧНО: 5 ошибок подряд - перезапуск контейнера
# [YYYY-MM-DD HH:MM:SS] ✅ Контейнер перезапущен

# Это нормально! Watchdog работает как надо.

# Если перезапусков нет в течение 24 часов - отлично!
# Проблема полностью решена.
```

## 🔧 Настройка параметров (опционально)

Если watchdog перезапускает слишком часто или слишком редко:

```bash
# Отредактировать параметры в watchdog.sh
sudo nano /home/ros2/rob_box_project/docker/vision/scripts/oak-d/watchdog.sh

# CHECK_INTERVAL=30  # Интервал проверки в секундах (по умолчанию 30)
# ERROR_THRESHOLD=5  # Количество ошибок для перезапуска (по умолчанию 5)

# Перезапустить watchdog сервис
sudo systemctl restart oak-d-watchdog
```

## 🚨 Откат изменений (если что-то пошло не так)

```bash
# На Vision Pi

# 1. Остановить и отключить watchdog
sudo systemctl stop oak-d-watchdog
sudo systemctl disable oak-d-watchdog

# 2. Удалить сервис
sudo rm /etc/systemd/system/oak-d-watchdog.service
sudo systemctl daemon-reload

# 3. Откатить git
cd ~/rob_box_project
git checkout main  # или предыдущую рабочую ветку
git pull origin main

# 4. Перезапустить контейнер
cd docker/vision
./update_and_restart.sh
```

## 📊 Метрики успеха

Через 24 часа после установки проверить:

- [ ] Камера работает непрерывно без ошибок X_LINK_ERROR
- [ ] Watchdog сервис активен (systemctl status)
- [ ] Логи не показывают частые перезапуски (< 1 раз в час)
- [ ] ROS топики камеры публикуются стабильно
- [ ] RTAB-Map получает данные без предупреждений

## 📝 Отчёт о тестировании

После 24-48 часов работы заполнить:

```
Дата установки: _________________
Время установки: ________________

Результаты после 24 часов:
- Uptime контейнера: ____________
- Количество автоперезапусков: ___
- Ошибки X_LINK_ERROR: Да / Нет
- Камера работает стабильно: Да / Нет

Результаты после 48 часов:
- Uptime контейнера: ____________
- Количество автоперезапусков: ___
- Ошибки X_LINK_ERROR: Да / Нет
- Камера работает стабильно: Да / Нет

Комментарии:
_________________________________
_________________________________
```

## 🎯 Критерии успешного решения

✅ **Успех**: Камера работает > 24 часов без ошибок X_LINK_ERROR
✅ **Частичный успех**: Watchdog автоматически перезапускает, работа восстанавливается
❌ **Неудача**: Перезапуски не помогают, требуется ручное вмешательство

---

**Последнее обновление**: 11 ноября 2025  
**Подготовил**: GitHub Copilot Agent
