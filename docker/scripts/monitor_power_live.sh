#!/bin/bash
# Скрипт непрерывного мониторинга питания в реальном времени
# Показывает ключевые метрики с автообновлением

set -euo pipefail

# Интервал обновления в секундах
INTERVAL=${1:-2}

# Цвета
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# Функция для очистки экрана и вывода заголовка
print_header() {
    clear
    echo -e "${BOLD}${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}${BLUE}║       Мониторинг питания Raspberry Pi (реальное время)    ║${NC}"
    echo -e "${BOLD}${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}Обновление каждые ${INTERVAL}с | Для выхода: Ctrl+C${NC}"
    echo ""
    date "+%Y-%m-%d %H:%M:%S"
    echo "────────────────────────────────────────────────────────────"
    echo ""
}

# Функция для отображения метрики с цветовым индикатором
print_metric() {
    local label="$1"
    local value="$2"
    local status="$3"  # good, warning, error
    
    case "$status" in
        good)
            echo -e "${label}: ${GREEN}${value}${NC}"
            ;;
        warning)
            echo -e "${label}: ${YELLOW}${value}${NC}"
            ;;
        error)
            echo -e "${label}: ${RED}${value}${NC}"
            ;;
        *)
            echo -e "${label}: ${value}"
            ;;
    esac
}

# Основной цикл мониторинга
while true; do
    print_header
    
    # === ПИТАНИЕ ===
    echo -e "${BOLD}⚡ ПИТАНИЕ${NC}"
    
    # Напряжение
    voltage=$(vcgencmd measure_volts | cut -d'=' -f2)
    voltage_val=$(echo $voltage | sed 's/V//')
    if (( $(echo "$voltage_val < 4.8" | bc -l) )); then
        print_metric "  Напряжение" "$voltage" "error"
    elif (( $(echo "$voltage_val < 5.0" | bc -l) )); then
        print_metric "  Напряжение" "$voltage" "warning"
    else
        print_metric "  Напряжение" "$voltage" "good"
    fi
    
    # USB Max Current
    usb_max_current=$(vcgencmd get_config usb_max_current_enable | cut -d'=' -f2)
    if [ "$usb_max_current" = "1" ]; then
        print_metric "  USB Limit" "1600mA (полный)" "good"
    else
        print_metric "  USB Limit" "600mA (ограничен)" "warning"
    fi
    
    # Троттлинг
    throttled=$(vcgencmd get_throttled | cut -d'=' -f2)
    throttled_dec=$((throttled))
    if [ "$throttled_dec" -eq 0 ]; then
        print_metric "  Троттлинг" "Нет (0x0)" "good"
    else
        issues=""
        [ $((throttled_dec & 0x1)) -ne 0 ] && issues="${issues}UV "
        [ $((throttled_dec & 0x2)) -ne 0 ] && issues="${issues}FREQ "
        [ $((throttled_dec & 0x4)) -ne 0 ] && issues="${issues}THROT "
        [ $((throttled_dec & 0x8)) -ne 0 ] && issues="${issues}TEMP "
        print_metric "  Троттлинг" "$throttled ($issues)" "error"
    fi
    
    echo ""
    
    # === СИСТЕМА ===
    echo -e "${BOLD}💻 СИСТЕМА${NC}"
    
    # Частота CPU
    cpu_freq=$(vcgencmd measure_clock arm | cut -d'=' -f2)
    cpu_freq_mhz=$((cpu_freq / 1000000))
    print_metric "  CPU Частота" "${cpu_freq_mhz} MHz" "good"
    
    # Температура
    temp=$(vcgencmd measure_temp | cut -d'=' -f2 | sed 's/°C//')
    temp_val=$(echo $temp | sed "s/'C//")
    if (( $(echo "$temp_val > 80" | bc -l) )); then
        print_metric "  Температура" "${temp}°C" "error"
    elif (( $(echo "$temp_val > 70" | bc -l) )); then
        print_metric "  Температура" "${temp}°C" "warning"
    else
        print_metric "  Температура" "${temp}°C" "good"
    fi
    
    # Load Average
    load_avg=$(cat /proc/loadavg | awk '{print $1, $2, $3}')
    load_1min=$(cat /proc/loadavg | awk '{print $1}')
    cpu_count=$(nproc)
    if (( $(echo "$load_1min > $cpu_count" | bc -l) )); then
        print_metric "  Load Avg (1/5/15)" "$load_avg" "warning"
    else
        print_metric "  Load Avg (1/5/15)" "$load_avg" "good"
    fi
    
    echo ""
    
    # === ПАМЯТЬ ===
    echo -e "${BOLD}🧠 ПАМЯТЬ${NC}"
    
    mem_info=$(free -h | grep "Mem:")
    mem_total=$(echo $mem_info | awk '{print $2}')
    mem_used=$(echo $mem_info | awk '{print $3}')
    mem_avail=$(echo $mem_info | awk '{print $7}')
    mem_percent=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100}')
    
    if [ "$mem_percent" -gt 90 ]; then
        print_metric "  RAM" "${mem_used}/${mem_total} (${mem_percent}%)" "error"
    elif [ "$mem_percent" -gt 75 ]; then
        print_metric "  RAM" "${mem_used}/${mem_total} (${mem_percent}%)" "warning"
    else
        print_metric "  RAM" "${mem_used}/${mem_total} (${mem_percent}%)" "good"
    fi
    
    print_metric "  Доступно" "$mem_avail" "good"
    
    echo ""
    
    # === USB УСТРОЙСТВА ===
    echo -e "${BOLD}🔌 USB УСТРОЙСТВА${NC}"
    
    usb_count=$(lsusb | wc -l)
    echo "  Подключено: $usb_count устройств"
    
    # Показать только значимые устройства (не хабы)
    lsusb | grep -v "Hub" | head -n 5 | while read -r line; do
        device=$(echo "$line" | cut -d':' -f3- | xargs | cut -c1-45)
        echo "    • $device"
    done
    
    echo ""
    
    # === СЕТЬ ===
    echo -e "${BOLD}🌐 СЕТЬ${NC}"
    
    # Показать активные интерфейсы
    active_count=0
    ip -brief addr show | grep -v "lo " | while read -r line; do
        iface=$(echo "$line" | awk '{print $1}')
        state=$(echo "$line" | awk '{print $2}')
        ip=$(echo "$line" | awk '{print $3}' | cut -d'/' -f1)
        
        if [ "$state" = "UP" ] && [ -n "$ip" ]; then
            active_count=$((active_count + 1))
            printf "  %-8s %s\n" "$iface:" "$ip"
        fi
    done
    
    echo ""
    
    # === ДИСК ===
    echo -e "${BOLD}💾 ХРАНИЛИЩЕ${NC}"
    
    root_usage=$(df -h / | awk 'NR==2 {print $5}' | sed 's/%//')
    root_size=$(df -h / | awk 'NR==2 {print $2}')
    root_avail=$(df -h / | awk 'NR==2 {print $4}')
    
    if [ "$root_usage" -gt 90 ]; then
        print_metric "  Root (/)" "${root_usage}% из ${root_size}" "error"
    elif [ "$root_usage" -gt 75 ]; then
        print_metric "  Root (/)" "${root_usage}% из ${root_size}" "warning"
    else
        print_metric "  Root (/)" "${root_usage}% из ${root_size}" "good"
    fi
    
    print_metric "  Свободно" "$root_avail" "good"
    
    echo ""
    echo "────────────────────────────────────────────────────────────"
    echo -e "${CYAN}Следующее обновление через ${INTERVAL}с...${NC}"
    
    # Ждать перед следующим обновлением
    sleep "$INTERVAL"
done
