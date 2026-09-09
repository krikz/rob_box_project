#!/bin/bash
# Скрипт мониторинга питания Raspberry Pi 5
# Показывает текущее состояние питания, USB портов, троттлинга и температуры

set -euo pipefail

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "========================================"
echo "  Мониторинг питания Raspberry Pi"
echo "========================================"
echo ""

# === РЕЖИМ ПИТАНИЯ ===
echo -e "${BLUE}[РЕЖИМ ПИТАНИЯ]${NC}"

# USB Max Current
usb_max_current=$(vcgencmd get_config usb_max_current_enable | cut -d'=' -f2)
if [ "$usb_max_current" = "1" ]; then
    echo -e "USB Max Current: ${GREEN}Включен (1600mA)${NC}"
else
    echo -e "USB Max Current: ${YELLOW}Ограничен (600mA)${NC}"
fi

# PSU Max Current из EEPROM
if command -v rpi-eeprom-config &> /dev/null; then
    psu_max=$(sudo rpi-eeprom-config 2>/dev/null | grep PSU_MAX_CURRENT || echo "PSU_MAX_CURRENT=не задан")
    echo "EEPROM: $psu_max"
fi

echo ""

# === ТРОТТЛИНГ И НАПРЯЖЕНИЕ ===
echo -e "${BLUE}[ТРОТТЛИНГ]${NC}"

throttled=$(vcgencmd get_throttled | cut -d'=' -f2)
throttled_dec=$((throttled))

if [ "$throttled_dec" -eq 0 ]; then
    echo -e "Статус: ${GREEN}Нет проблем (0x0)${NC}"
else
    echo -e "Статус: ${RED}Обнаружены проблемы ($throttled)${NC}"
    
    # Расшифровка битов
    [ $((throttled_dec & 0x1)) -ne 0 ] && echo -e "  ${RED}⚠ Undervoltage detected (напряжение < 4.63V)${NC}"
    [ $((throttled_dec & 0x2)) -ne 0 ] && echo -e "  ${RED}⚠ ARM frequency capped (частота ограничена)${NC}"
    [ $((throttled_dec & 0x4)) -ne 0 ] && echo -e "  ${RED}⚠ Currently throttled (троттлинг активен)${NC}"
    [ $((throttled_dec & 0x8)) -ne 0 ] && echo -e "  ${YELLOW}⚠ Soft temperature limit (температурное ограничение)${NC}"
    
    [ $((throttled_dec & 0x10000)) -ne 0 ] && echo -e "  ${YELLOW}○ Undervoltage occurred (было в прошлом)${NC}"
    [ $((throttled_dec & 0x20000)) -ne 0 ] && echo -e "  ${YELLOW}○ ARM frequency capped occurred${NC}"
    [ $((throttled_dec & 0x40000)) -ne 0 ] && echo -e "  ${YELLOW}○ Throttling occurred${NC}"
    [ $((throttled_dec & 0x80000)) -ne 0 ] && echo -e "  ${YELLOW}○ Soft temperature limit occurred${NC}"
fi

echo ""

# === НАПРЯЖЕНИЕ И ЧАСТОТА ===
echo -e "${BLUE}[ЭЛЕКТРОПИТАНИЕ]${NC}"

# Напряжение
voltage=$(vcgencmd measure_volts | cut -d'=' -f2)
voltage_val=$(echo $voltage | sed 's/V//')
if (( $(echo "$voltage_val < 4.8" | bc -l) )); then
    echo -e "Напряжение: ${RED}$voltage (низкое!)${NC}"
elif (( $(echo "$voltage_val < 5.0" | bc -l) )); then
    echo -e "Напряжение: ${YELLOW}$voltage (нормальное)${NC}"
else
    echo -e "Напряжение: ${GREEN}$voltage (отличное)${NC}"
fi

# Частота CPU
cpu_freq=$(vcgencmd measure_clock arm | cut -d'=' -f2)
cpu_freq_mhz=$((cpu_freq / 1000000))
echo "Частота CPU: ${cpu_freq_mhz} MHz"

# Температура
temp=$(vcgencmd measure_temp | cut -d'=' -f2 | sed 's/°C//')
temp_val=$(echo $temp | sed "s/'C//")
if (( $(echo "$temp_val > 80" | bc -l) )); then
    echo -e "Температура: ${RED}${temp}°C (высокая!)${NC}"
elif (( $(echo "$temp_val > 70" | bc -l) )); then
    echo -e "Температура: ${YELLOW}${temp}°C (тепло)${NC}"
else
    echo -e "Температура: ${GREEN}${temp}°C (норма)${NC}"
fi

echo ""

# === USB УСТРОЙСТВА ===
echo -e "${BLUE}[USB УСТРОЙСТВА]${NC}"

echo "Подключенные устройства:"
lsusb | while read -r line; do
    # Убрать "Bus XXX Device XXX: ID"
    device=$(echo "$line" | sed 's/Bus [0-9]* Device [0-9]*: //')
    echo "  • $device"
done

# Попытка получить информацию о токе USB портов
if [ -d /sys/bus/usb/devices ]; then
    echo ""
    echo "Потребление USB портов (если поддерживается драйвером):"
    for dev in /sys/bus/usb/devices/*/power/current_now; do
        if [ -f "$dev" ]; then
            current=$(cat "$dev" 2>/dev/null || echo "0")
            if [ "$current" != "0" ]; then
                current_ma=$((current / 1000))
                port=$(echo "$dev" | cut -d'/' -f6)
                echo "  Port $port: ${current_ma}mA"
            fi
        fi
    done
fi

echo ""

# === ХРАНИЛИЩЕ ===
echo -e "${BLUE}[ХРАНИЛИЩЕ]${NC}"

# SD карта или основной диск
root_usage=$(df -h / | awk 'NR==2 {print $5}' | sed 's/%//')
root_size=$(df -h / | awk 'NR==2 {print $2}')
root_avail=$(df -h / | awk 'NR==2 {print $4}')

if [ "$root_usage" -gt 90 ]; then
    echo -e "Root (/): ${RED}${root_usage}% занято${NC} из ${root_size} (свободно: ${root_avail})"
elif [ "$root_usage" -gt 75 ]; then
    echo -e "Root (/): ${YELLOW}${root_usage}% занято${NC} из ${root_size} (свободно: ${root_avail})"
else
    echo -e "Root (/): ${GREEN}${root_usage}% занято${NC} из ${root_size} (свободно: ${root_avail})"
fi

echo ""

# === СЕТЬ ===
echo -e "${BLUE}[СЕТЕВЫЕ ИНТЕРФЕЙСЫ]${NC}"

# Показать активные интерфейсы
ip -brief addr show | grep -v "lo " | while read -r line; do
    iface=$(echo "$line" | awk '{print $1}')
    state=$(echo "$line" | awk '{print $2}')
    ip=$(echo "$line" | awk '{print $3}' | cut -d'/' -f1)
    
    if [ "$state" = "UP" ]; then
        echo -e "  ${GREEN}$iface${NC}: $ip ($state)"
    else
        echo -e "  ${YELLOW}$iface${NC}: $state"
    fi
done

echo ""

# === ПАМЯТЬ ===
echo -e "${BLUE}[ПАМЯТЬ]${NC}"

mem_info=$(free -h | grep "Mem:")
mem_total=$(echo $mem_info | awk '{print $2}')
mem_used=$(echo $mem_info | awk '{print $3}')
mem_avail=$(echo $mem_info | awk '{print $7}')
mem_percent=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100}')

if [ "$mem_percent" -gt 90 ]; then
    echo -e "RAM: ${RED}${mem_used}/${mem_total}${NC} использовано (${mem_percent}%, доступно: ${mem_avail})"
elif [ "$mem_percent" -gt 75 ]; then
    echo -e "RAM: ${YELLOW}${mem_used}/${mem_total}${NC} использовано (${mem_percent}%, доступно: ${mem_avail})"
else
    echo -e "RAM: ${GREEN}${mem_used}/${mem_total}${NC} использовано (${mem_percent}%, доступно: ${mem_avail})"
fi

echo ""

# === UPTIME ===
echo -e "${BLUE}[СИСТЕМА]${NC}"
uptime_info=$(uptime -p | sed 's/up //')
echo "Uptime: $uptime_info"
load_avg=$(uptime | awk -F'load average:' '{print $2}' | xargs)
echo "Load Average: $load_avg"

echo ""
echo "========================================"
echo "  Мониторинг завершен"
echo "========================================"
