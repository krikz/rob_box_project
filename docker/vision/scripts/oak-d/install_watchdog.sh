#!/bin/bash
# Установка OAK-D Watchdog Service
# Запускать на Vision Pi (ros2@10.1.1.21)

set -e

echo "🐕 Установка OAK-D Watchdog Service..."

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SERVICE_FILE="$SCRIPT_DIR/oak-d-watchdog.service"
WATCHDOG_SCRIPT="$SCRIPT_DIR/watchdog.sh"

# Проверяем наличие файлов
if [ ! -f "$SERVICE_FILE" ]; then
    echo "❌ Файл $SERVICE_FILE не найден!"
    exit 1
fi

if [ ! -f "$WATCHDOG_SCRIPT" ]; then
    echo "❌ Файл $WATCHDOG_SCRIPT не найден!"
    exit 1
fi

# Делаем скрипты исполняемыми
chmod +x "$WATCHDOG_SCRIPT"
chmod +x "$SCRIPT_DIR/setup_usb_power.sh"

echo "✅ Скрипты сделаны исполняемыми"

# Копируем service файл в systemd
sudo cp "$SERVICE_FILE" /etc/systemd/system/

echo "✅ Service файл скопирован в /etc/systemd/system/"

# Перезагружаем systemd
sudo systemctl daemon-reload

echo "✅ systemd daemon перезагружен"

# Включаем сервис (автозапуск)
sudo systemctl enable oak-d-watchdog.service

echo "✅ Watchdog service включен (автозапуск)"

# Запускаем сервис
sudo systemctl start oak-d-watchdog.service

echo "✅ Watchdog service запущен"

# Показываем статус
echo ""
echo "📊 Статус сервиса:"
sudo systemctl status oak-d-watchdog.service --no-pager

echo ""
echo "✅ Установка завершена!"
echo ""
echo "📋 Полезные команды:"
echo "  sudo systemctl status oak-d-watchdog   # Статус"
echo "  sudo systemctl stop oak-d-watchdog     # Остановить"
echo "  sudo systemctl start oak-d-watchdog    # Запустить"
echo "  sudo systemctl restart oak-d-watchdog  # Перезапустить"
echo "  sudo journalctl -u oak-d-watchdog -f   # Смотреть логи"
echo "  tail -f /tmp/oak-d-watchdog.log        # Смотреть логи watchdog"
