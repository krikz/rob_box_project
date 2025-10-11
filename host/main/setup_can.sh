#!/bin/bash
# setup_can.sh - Инициализация CAN интерфейса для VESC
# 
# Этот скрипт настраивает CAN интерфейс (can0 или can1) для работы с VESC регуляторами.
# Запускается на Main Pi перед стартом ros2_control контейнера.
#
# Использование:
#   sudo ./setup_can.sh can0    # Первый CAN интерфейс
#   sudo ./setup_can.sh can1    # Второй CAN интерфейс

set -e

# Параметр: CAN интерфейс (can0 или can1)
CAN_INTERFACE="${1:-can0}"
CAN_BITRATE="500000"  # 500 kbit/s - стандартный bitrate для VESC

# Проверка допустимости интерфейса
if [[ ! "$CAN_INTERFACE" =~ ^can[01]$ ]]; then
    echo "❌ Ошибка: недопустимый CAN интерфейс: $CAN_INTERFACE"
    echo "Используйте: can0 или can1"
    exit 1
fi

echo "🔧 Настройка CAN интерфейса ${CAN_INTERFACE}..."

# Проверка прав root
if [ "$EUID" -ne 0 ]; then 
  echo "❌ Ошибка: этот скрипт требует root привилегий"
  echo "Запустите: sudo $0"
  exit 1
fi

# Проверка наличия CAN модуля ядра
if ! lsmod | grep -q "can"; then
    echo "📦 Загружаем can модуль ядра..."
    modprobe can || {
        echo "❌ Не удалось загрузить can модуль"
        exit 1
    }
fi

if ! lsmod | grep -q "can_raw"; then
    echo "📦 Загружаем can_raw модуль ядра..."
    modprobe can_raw || {
        echo "❌ Не удалось загрузить can_raw модуль"
        exit 1
    }
fi

# Проверка наличия CAN драйвера (для MCP2515 SPI CAN контроллера)
if ! lsmod | grep -q "mcp251x"; then
    echo "📦 Загружаем mcp251x драйвер..."
    modprobe mcp251x || {
        echo "⚠️  Не удалось загрузить mcp251x драйвер"
        echo "Проверьте настройки Device Tree и /boot/config.txt"
    }
fi

# Остановка интерфейса, если он запущен
if ip link show ${CAN_INTERFACE} > /dev/null 2>&1; then
    echo "🛑 Останавливаем ${CAN_INTERFACE}..."
    ip link set ${CAN_INTERFACE} down 2>/dev/null || true
fi

# Настройка и запуск CAN интерфейса
echo "🚀 Настраиваем ${CAN_INTERFACE} с bitrate ${CAN_BITRATE}..."

if ip link show ${CAN_INTERFACE} > /dev/null 2>&1; then
    ip link set ${CAN_INTERFACE} type can bitrate ${CAN_BITRATE} || {
        echo "❌ Не удалось настроить bitrate"
        exit 1
    }
    
    ip link set ${CAN_INTERFACE} up || {
        echo "❌ Не удалось поднять ${CAN_INTERFACE}"
        exit 1
    }
    
    echo "✅ CAN интерфейс ${CAN_INTERFACE} успешно настроен"
    echo ""
    echo "📊 Состояние интерфейса:"
    ip -details link show ${CAN_INTERFACE}
    
    echo ""
    echo "💡 Для мониторинга CAN трафика используйте:"
    echo "   candump ${CAN_INTERFACE}"
    echo "   cansniffer ${CAN_INTERFACE}"
    
else
    echo "❌ CAN интерфейс ${CAN_INTERFACE} не найден!"
    echo ""
    echo "Проверьте:"
    echo "  1. Подключен ли CAN Shield к Raspberry Pi"
    echo "  2. Настроен ли Device Tree в /boot/config.txt:"
    echo "     dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25"
    echo "  3. Перезагружен ли Raspberry Pi после изменения config.txt"
    echo ""
    echo "Доступные сетевые интерфейсы:"
    ip link show
    exit 1
fi
