# Исправление OAK-D Disconnection - Быстрый старт

## Проблема
OAK-D камера отключается через 8+ часов с ошибками X_LINK_ERROR.

## Решение в один шаг

На **Vision Pi** (ros2@10.1.1.21):

```bash
cd ~/rob_box_project/docker/vision && \
git pull origin main && \
cd scripts/oak-d && \
./install_watchdog.sh && \
cd ~/rob_box_project/docker/vision && \
./update_and_restart.sh
```

## Проверка

```bash
# Статус watchdog
sudo systemctl status oak-d-watchdog

# Логи
tail -f /tmp/oak-d-watchdog.log
```

## Готово! ✅

Камера теперь будет автоматически перезапускаться при ошибках USB.

📖 Подробности: `docker/vision/scripts/oak-d/README_OAKD_FIX.md`
