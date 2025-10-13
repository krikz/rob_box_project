#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🤖 РОББОКС Vision Pi - Automated Setup Script
# ═══════════════════════════════════════════════════════════════════════════
# Автоматическая настройка Vision Pi с нуля
#
# Использование:
#   curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_vision_pi.sh | bash
#
# Что делает скрипт:
#   1. Проверяет и устанавливает Docker + Docker Compose
#   2. Настраивает Git
#   3. Клонирует репозиторий rob_box_project
#   4. Настраивает кастомный MOTD (приветствие при входе)
#   5. Подготавливает систему к запуску Docker контейнеров
#
# Автор: AI Agent + krikz
# Дата: 2025-10-13
# ═══════════════════════════════════════════════════════════════════════════

set -e  # Остановка при ошибке

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Логотип РОББОКС
print_logo() {
    echo -e "${PURPLE}"
    cat << 'EOF'
    ╔═══════════════════════════════════════════════════════════════════╗
    ║                                                                   ║
    ║   ██████╗  ██████╗ ██████╗ ██████╗  ██████╗ ██╗  ██╗███████╗    ║
    ║   ██╔══██╗██╔═══██╗██╔══██╗██╔══██╗██╔═══██╗██║ ██╔╝██╔════╝    ║
    ║   ██████╔╝██║   ██║██████╔╝██████╔╝██║   ██║█████╔╝ ███████╗    ║
    ║   ██╔══██╗██║   ██║██╔══██╗██╔══██╗██║   ██║██╔═██╗ ╚════██║    ║
    ║   ██║  ██║╚██████╔╝██████╔╝██████╔╝╚██████╔╝██║  ██╗███████║    ║
    ║   ╚═╝  ╚═╝ ╚═════╝ ╚═════╝ ╚═════╝  ╚═════╝ ╚═╝  ╚═╝╚══════╝    ║
    ║                                                                   ║
    ║            🤖 Vision Pi - Automated Setup v1.0 🤖                 ║
    ║                                                                   ║
    ╚═══════════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

# Функция для красивого вывода
log_info() {
    echo -e "${CYAN}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

log_error() {
    echo -e "${RED}[✗]${NC} $1"
}

log_step() {
    echo ""
    echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}▶ $1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
}

# Проверка что скрипт запущен на Raspberry Pi
check_raspberry_pi() {
    if [ ! -f /proc/device-tree/model ]; then
        log_warning "Не обнаружен Raspberry Pi, продолжаем..."
        return
    fi
    
    model=$(cat /proc/device-tree/model)
    log_info "Обнаружено устройство: $model"
    
    if [[ ! "$model" =~ "Raspberry Pi" ]]; then
        log_warning "Это не Raspberry Pi, но продолжаем установку..."
    fi
}

# Проверка и установка Docker
install_docker() {
    log_step "Проверка Docker"
    
    if command -v docker &> /dev/null; then
        docker_version=$(docker --version)
        log_success "Docker уже установлен: $docker_version"
    else
        log_info "Docker не найден, начинаем установку..."
        
        # Установка Docker
        log_info "Скачивание и установка Docker..."
        curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
        sudo sh /tmp/get-docker.sh
        rm /tmp/get-docker.sh
        
        # Добавление пользователя в группу docker
        sudo usermod -aG docker $USER
        
        log_success "Docker установлен успешно!"
        log_warning "Необходимо перелогиниться для применения группы docker"
        log_warning "Выполните: exit, затем подключитесь заново"
    fi
}

# Проверка Docker Compose
check_docker_compose() {
    log_step "Проверка Docker Compose"
    
    if docker compose version &> /dev/null; then
        compose_version=$(docker compose version)
        log_success "Docker Compose установлен: $compose_version"
    else
        log_error "Docker Compose не найден!"
        log_info "Docker Compose обычно устанавливается вместе с Docker"
        log_info "Попробуйте: sudo apt install docker-compose-plugin"
        exit 1
    fi
}

# Установка необходимых пакетов
install_dependencies() {
    log_step "Установка системных зависимостей"
    
    log_info "Обновление списка пакетов..."
    sudo apt update
    
    packages="git curl wget vim htop net-tools usbutils"
    
    log_info "Установка пакетов: $packages"
    sudo apt install -y $packages
    
    log_success "Зависимости установлены"
}

# Клонирование репозитория
clone_repository() {
    log_step "Клонирование репозитория rob_box_project"
    
    REPO_DIR="$HOME/rob_box_project"
    
    if [ -d "$REPO_DIR" ]; then
        log_warning "Репозиторий уже существует в $REPO_DIR"
        log_info "Обновляем до последней версии..."
        cd "$REPO_DIR"
        git fetch origin
        git checkout develop
        git pull origin develop
        log_success "Репозиторий обновлён"
    else
        log_info "Клонирование из GitHub..."
        git clone https://github.com/krikz/rob_box_project.git "$REPO_DIR"
        cd "$REPO_DIR"
        git checkout develop
        log_success "Репозиторий склонирован в $REPO_DIR"
    fi
}

# Настройка кастомного MOTD
setup_motd() {
    log_step "Настройка кастомного MOTD (Message of the Day)"
    
    MOTD_SCRIPT="/usr/local/bin/robbox-motd"
    
    log_info "Создание скрипта MOTD..."
    sudo tee "$MOTD_SCRIPT" > /dev/null << 'MOTDEOF'
#!/bin/bash
# РОББОКС Vision Pi - Dynamic MOTD

# Цвета
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

# Получение информации о системе
HOSTNAME=$(hostname)
UPTIME=$(uptime -p | sed 's/up //')
MEMORY=$(free -h | awk '/^Mem:/ {print $3 "/" $2}')
DISK=$(df -h / | awk 'NR==2 {print $3 "/" $2 " (" $5 ")"}')
LOAD=$(cat /proc/loadavg | awk '{print $1", "$2", "$3}')
CPU_TEMP=$(vcgencmd measure_temp 2>/dev/null | cut -d'=' -f2 || echo "N/A")
VOLTAGE=$(vcgencmd measure_volts core 2>/dev/null | cut -d'=' -f2 || echo "N/A")

# IP адреса
ETH_IP=$(ip -4 addr show eth0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || echo "N/A")
WLAN_IP=$(ip -4 addr show wlan0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || echo "N/A")

# Docker контейнеры
if command -v docker &> /dev/null; then
    DOCKER_RUNNING=$(docker ps --format "{{.Names}}" 2>/dev/null | wc -l)
    DOCKER_TOTAL=$(docker ps -a --format "{{.Names}}" 2>/dev/null | wc -l)
    DOCKER_STATUS="${GREEN}${DOCKER_RUNNING}${NC}/${DOCKER_TOTAL} running"
else
    DOCKER_STATUS="${RED}Not installed${NC}"
fi

# Логотип РОББОКС
echo -e "${PURPLE}"
cat << 'EOF'
    ╔═══════════════════════════════════════════════════════════════════╗
    ║                                                                   ║
    ║   ██████╗  ██████╗ ██████╗ ██████╗  ██████╗ ██╗  ██╗███████╗    ║
    ║   ██╔══██╗██╔═══██╗██╔══██╗██╔══██╗██╔═══██╗██║ ██╔╝██╔════╝    ║
    ║   ██████╔╝██║   ██║██████╔╝██████╔╝██║   ██║█████╔╝ ███████╗    ║
    ║   ██╔══██╗██║   ██║██╔══██╗██╔══██╗██║   ██║██╔═██╗ ╚════██║    ║
    ║   ██║  ██║╚██████╔╝██████╔╝██████╔╝╚██████╔╝██║  ██╗███████║    ║
    ║   ╚═╝  ╚═╝ ╚═════╝ ╚═════╝ ╚═════╝  ╚═════╝ ╚═╝  ╚═╝╚══════╝    ║
    ║                                                                   ║
    ║                    🎥 Vision Pi (10.1.1.11) 🎥                    ║
    ║                                                                   ║
    ╚═══════════════════════════════════════════════════════════════════╝
EOF
echo -e "${NC}"

# Системная информация
echo -e "${BOLD}${CYAN}SYSTEM INFO${NC}"
echo -e "  ${BLUE}Hostname:${NC}       $HOSTNAME"
echo -e "  ${BLUE}Uptime:${NC}         $UPTIME"
echo -e "  ${BLUE}Load Average:${NC}   $LOAD"
echo -e "  ${BLUE}Memory:${NC}         $MEMORY"
echo -e "  ${BLUE}Disk Usage:${NC}     $DISK"
echo ""

# Температура и питание
echo -e "${BOLD}${YELLOW}HARDWARE${NC}"
echo -e "  ${BLUE}CPU Temp:${NC}       $CPU_TEMP"
echo -e "  ${BLUE}Core Voltage:${NC}   $VOLTAGE"
echo ""

# Сеть
echo -e "${BOLD}${GREEN}NETWORK${NC}"
echo -e "  ${BLUE}Ethernet (eth0):${NC}  $ETH_IP ${CYAN}[Data Link]${NC}"
echo -e "  ${BLUE}WiFi (wlan0):${NC}     $WLAN_IP ${CYAN}[SSH/Management]${NC}"
echo ""

# Docker контейнеры
echo -e "${BOLD}${PURPLE}DOCKER CONTAINERS${NC}"
echo -e "  ${BLUE}Status:${NC}         $DOCKER_STATUS"

if command -v docker &> /dev/null && [ $DOCKER_RUNNING -gt 0 ]; then
    echo ""
    docker ps --format "  ${GREEN}✓${NC} {{.Names}}\t{{.Status}}" 2>/dev/null | column -t -s $'\t'
fi

echo ""
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}Quick Commands:${NC}"
echo -e "  ${BLUE}cd ~/rob_box_project/docker/vision${NC}  - Go to Vision Pi config"
echo -e "  ${BLUE}docker compose ps${NC}                    - Check containers status"
echo -e "  ${BLUE}docker compose logs -f${NC}               - View all container logs"
echo -e "  ${BLUE}docker compose restart${NC}               - Restart all containers"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
MOTDEOF
    
    sudo chmod +x "$MOTD_SCRIPT"
    
    # Отключаем стандартные MOTD скрипты
    log_info "Отключение стандартных MOTD скриптов..."
    if [ -d /etc/update-motd.d ]; then
        sudo chmod -x /etc/update-motd.d/* 2>/dev/null || true
    fi
    
    # Добавляем наш скрипт в профиль
    if ! grep -q "robbox-motd" "$HOME/.bashrc"; then
        log_info "Добавление MOTD в .bashrc..."
        echo "" >> "$HOME/.bashrc"
        echo "# РОББОКС Custom MOTD" >> "$HOME/.bashrc"
        echo "$MOTD_SCRIPT" >> "$HOME/.bashrc"
    fi
    
    log_success "Кастомный MOTD настроен!"
}

# Настройка автозапуска Docker контейнеров
setup_autostart() {
    log_step "Настройка автозапуска Docker контейнеров"
    
    COMPOSE_DIR="$HOME/rob_box_project/docker/vision"
    SERVICE_FILE="/etc/systemd/system/robbox-vision.service"
    
    log_info "Создание systemd service..."
    sudo tee "$SERVICE_FILE" > /dev/null << SERVICEEOF
[Unit]
Description=ROBBOX Vision Pi Docker Containers
After=docker.service network-online.target
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=$COMPOSE_DIR
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down
User=$USER

[Install]
WantedBy=multi-user.target
SERVICEEOF
    
    log_info "Активация service..."
    sudo systemctl daemon-reload
    sudo systemctl enable robbox-vision.service
    
    log_success "Автозапуск настроен!"
    log_info "Контейнеры будут автоматически запускаться при загрузке системы"
}

# Итоговая информация
print_summary() {
    log_step "Установка завершена!"
    
    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                     ✅ SETUP COMPLETED! ✅                         ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}Что было сделано:${NC}"
    echo -e "  ${GREEN}✓${NC} Docker и Docker Compose установлены"
    echo -e "  ${GREEN}✓${NC} Репозиторий rob_box_project склонирован в ~/rob_box_project"
    echo -e "  ${GREEN}✓${NC} Кастомный MOTD с логотипом РОББОКС настроен"
    echo -e "  ${GREEN}✓${NC} Автозапуск Docker контейнеров настроен"
    echo ""
    echo -e "${YELLOW}Следующие шаги:${NC}"
    echo ""
    echo -e "${BLUE}1. Перелогиниться для применения группы docker:${NC}"
    echo -e "   ${CYAN}exit${NC}"
    echo -e "   ${CYAN}ssh pi@vision-pi${NC}"
    echo ""
    echo -e "${BLUE}2. Логин в GitHub Container Registry:${NC}"
    echo -e "   ${CYAN}echo YOUR_TOKEN | docker login ghcr.io -u YOUR_USERNAME --password-stdin${NC}"
    echo ""
    echo -e "${BLUE}3. Запуск Docker контейнеров:${NC}"
    echo -e "   ${CYAN}cd ~/rob_box_project/docker/vision${NC}"
    echo -e "   ${CYAN}docker compose pull${NC}   # Скачать образы"
    echo -e "   ${CYAN}docker compose up -d${NC}  # Запустить контейнеры"
    echo ""
    echo -e "${BLUE}4. Проверка статуса:${NC}"
    echo -e "   ${CYAN}docker compose ps${NC}"
    echo -e "   ${CYAN}docker compose logs -f${NC}"
    echo ""
    echo -e "${GREEN}Готово! Приятной работы с РОББОКС! 🤖${NC}"
    echo ""
}

# ═══════════════════════════════════════════════════════════════════════════
# MAIN EXECUTION
# ═══════════════════════════════════════════════════════════════════════════

main() {
    print_logo
    
    log_info "Начинаем автоматическую настройку Vision Pi..."
    log_info "Скрипт выполняется от пользователя: $USER"
    
    check_raspberry_pi
    install_dependencies
    install_docker
    check_docker_compose
    clone_repository
    setup_motd
    setup_autostart
    
    print_summary
}

# Запуск главной функции
main
