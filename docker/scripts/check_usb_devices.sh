#!/bin/bash
# Скрипт мониторинга USB портов Raspberry Pi
# Показывает детальную информацию о подключенных USB устройствах и их энергопотреблении

set -euo pipefail

# Цвета
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo "========================================"
echo "  Детальный мониторинг USB портов"
echo "========================================"
echo ""

# === ЛИМИТ USB ТОКА ===
echo -e "${BLUE}[КОНФИГУРАЦИЯ USB]${NC}"

usb_max_current=$(vcgencmd get_config usb_max_current_enable | cut -d'=' -f2)
if [ "$usb_max_current" = "1" ]; then
    echo -e "USB Max Current: ${GREEN}1600mA (полный режим)${NC}"
    echo "  → Каждый порт может использовать до 1600mA суммарно"
else
    echo -e "USB Max Current: ${YELLOW}600mA (ограниченный режим)${NC}"
    echo "  → Суммарное ограничение для всех USB портов"
    echo -e "  ${YELLOW}⚠ Для повышения лимита: добавьте usb_max_current_enable=1 в /boot/firmware/config.txt${NC}"
fi

echo ""

# === USB ДЕРЕВО ===
echo -e "${BLUE}[USB ТОПОЛОГИЯ]${NC}"
echo "Дерево подключенных устройств:"
echo ""

lsusb -t

echo ""
echo "========================================"
echo ""

# === ДЕТАЛЬНАЯ ИНФОРМАЦИЯ ПО УСТРОЙСТВАМ ===
echo -e "${BLUE}[ДЕТАЛИ USB УСТРОЙСТВ]${NC}"
echo ""

# Получить список устройств
device_count=0

lsusb | while IFS= read -r line; do
    device_count=$((device_count + 1))
    
    # Парсинг строки lsusb
    bus=$(echo "$line" | awk '{print $2}')
    device=$(echo "$line" | awk '{print $4}' | sed 's/://')
    id=$(echo "$line" | awk '{print $6}')
    name=$(echo "$line" | cut -d':' -f3- | xargs)
    
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}Устройство #$device_count${NC}"
    echo "  Название: $name"
    echo "  Bus: $bus, Device: $device"
    echo "  ID: $id"
    
    # Попытка получить детальную информацию из sysfs
    dev_path="/sys/bus/usb/devices/${bus}-*"
    
    # Поиск соответствующего пути в sysfs
    for path in $dev_path; do
        if [ -d "$path" ]; then
            # Скорость
            if [ -f "$path/speed" ]; then
                speed=$(cat "$path/speed")
                echo "  Скорость: ${speed} Mbps"
            fi
            
            # Максимальный ток
            if [ -f "$path/bMaxPower" ]; then
                max_power=$(cat "$path/bMaxPower" | sed 's/mA//')
                if [ "$max_power" -gt 500 ]; then
                    echo -e "  Max Power: ${YELLOW}${max_power}mA${NC}"
                else
                    echo "  Max Power: ${max_power}mA"
                fi
            fi
            
            # Версия USB
            if [ -f "$path/version" ]; then
                version=$(cat "$path/version")
                echo "  USB Version: $version"
            fi
            
            # Производитель
            if [ -f "$path/manufacturer" ]; then
                manufacturer=$(cat "$path/manufacturer" 2>/dev/null)
                [ -n "$manufacturer" ] && echo "  Производитель: $manufacturer"
            fi
            
            # Серийный номер
            if [ -f "$path/serial" ]; then
                serial=$(cat "$path/serial" 2>/dev/null)
                [ -n "$serial" ] && echo "  Serial: $serial"
            fi
            
            # Продукт
            if [ -f "$path/product" ]; then
                product=$(cat "$path/product" 2>/dev/null)
                [ -n "$product" ] && echo "  Продукт: $product"
            fi
            
            break
        fi
    done
    
    echo ""
done

echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# === ПРОВЕРКА ИЗВЕСТНЫХ УСТРОЙСТВ ===
echo -e "${BLUE}[ПРОВЕРКА СПЕЦИФИЧНЫХ УСТРОЙСТВ]${NC}"
echo ""

# OAK-D камера (Luxonis)
if lsusb | grep -qi "Luxonis\|03e7"; then
    echo -e "${GREEN}✓ OAK-D камера обнаружена${NC}"
    oak_device=$(lsusb | grep -i "Luxonis\|03e7")
    echo "  $oak_device"
else
    echo -e "${YELLOW}○ OAK-D камера не найдена${NC}"
fi

# LSLIDAR (обычно как USB Serial или CDC ACM)
if lsusb | grep -qi "ACM\|Serial\|CH340\|CP210" || [ -e /dev/ttyACM0 ] || [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}✓ USB Serial устройство обнаружено (возможно LSLIDAR)${NC}"
    if [ -e /dev/ttyACM0 ]; then
        echo "  Найден: /dev/ttyACM0"
        # Проверить права доступа
        ls -l /dev/ttyACM0 | awk '{print "  Права:", $1, "Владелец:", $3, "Группа:", $4}'
    fi
    if [ -e /dev/ttyUSB0 ]; then
        echo "  Найден: /dev/ttyUSB0"
        ls -l /dev/ttyUSB0 | awk '{print "  Права:", $1, "Владелец:", $3, "Группа:", $4}'
    fi
else
    echo -e "${YELLOW}○ USB Serial устройства не найдены${NC}"
fi

# USB хабы
hub_count=$(lsusb | grep -ci "hub")
if [ "$hub_count" -gt 1 ]; then
    echo -e "${GREEN}✓ Обнаружено USB хабов: $hub_count${NC}"
    echo "  (один из них - встроенный хаб Pi)"
else
    echo -e "○ USB хабы: только встроенный"
fi

echo ""

# === МОНИТОРИНГ В РЕАЛЬНОМ ВРЕМЕНИ ===
echo -e "${BLUE}[ИНФОРМАЦИЯ]${NC}"
echo ""
echo "Для мониторинга USB в реальном времени используйте:"
echo "  watch -n 1 'lsusb && echo && vcgencmd get_throttled'"
echo ""
echo "Для просмотра подробной информации об устройстве:"
echo "  sudo lsusb -v -s BUS:DEVICE"
echo "  Например: sudo lsusb -v -s 001:005"
echo ""
echo "Для отслеживания подключений/отключений USB:"
echo "  sudo dmesg -wH | grep -i usb"
echo ""

echo "========================================"
echo "  Мониторинг завершен"
echo "========================================"
