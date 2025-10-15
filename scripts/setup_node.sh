#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🤖 РОББОКС - Unified Node Setup Script
# ═══════════════════════════════════════════════════════════════════════════
# Универсальный скрипт настройки узлов РОББОКС
#
# Использование:
#   curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash
#
# Или с указанием типа узла:
#   curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash -s vision
#   curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash -s main
#   curl -fsSL https://raw.githubusercontent.com/krikz/rob_box_project/develop/scripts/setup_node.sh | bash -s sensor-hub
#
# Что делает скрипт:
#   1. Определяет тип узла (интерактивно или через параметр)
#   2. Устанавливает Docker + Docker Compose
#   3. Настраивает Git и клонирует репозиторий
#   4. Настраивает кастомный MOTD с логотипом РОББОКС
#   5. Настраивает автозапуск соответствующих Docker контейнеров
#
# Автор: AI Agent + krikz
# Дата: 2025-10-15
# ═══════════════════════════════════════════════════════════════════════════

set -e  # Остановка при ошибке

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Глобальные переменные
NODE_TYPE=""
NODE_NAME=""
NODE_IP=""
NODE_EMOJI=""
DOCKER_COMPOSE_DIR=""
SERVICE_NAME=""

# ═══════════════════════════════════════════════════════════════════════════
# ФУНКЦИИ ДЛЯ ВЫВОДА
# ═══════════════════════════════════════════════════════════════════════════

print_logo() {
    echo -e "${PURPLE}"
    cat << 'EOF'
    ╔═══════════════════════════════════════════════════════════════════╗
    ║                                                                   ║
    ║  ██████╗   ██████╗   ╔██████ ██████╗   ██████╗  ██╗  ██╗ ██████╗  ║ 
    ║  ██╔══██╗ ██╔═══██╗ ╔╩═══╗██ ██╔═══╩╗ ██╔═══██╗ ██║ ██╔╝██╔════╝  ║
    ║  ██████╔╝ ██║   ██║ ╚╗██████ ██████╔╝ ██║   ██║ █████╔╝ ██║       ║
    ║  ██╔═══╝  ██║   ██║ ╔██══╗██ ██╔══██╗ ██║   ██║ ██╔═██╗ ██║       ║
    ║  ██║      ╚██████╔╝ ╚╗██████ ██████╔╝ ╚██████╔╝ ██║  ██╗╚██████╗  ║
    ║  ╚═╝       ╚═════╝   ╚═════╝ ╚═════╝   ╚═════╝  ╚═╝  ╚═╝ ╚═════╝  ║
    ║                                                                   ║
    ╚═══════════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
    echo -e "${CYAN}                    🤖  Node Setup v2.0  🤖${NC}"
    echo ""
}

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

# ═══════════════════════════════════════════════════════════════════════════
# ВЫБОР ТИПА УЗЛА
# ═══════════════════════════════════════════════════════════════════════════

show_node_menu() {
    echo ""
    echo -e "${BOLD}${CYAN}Выберите тип узла РОББОКС:${NC}"
    echo ""
    echo -e "  ${GREEN}1)${NC} 🎥 Vision Pi      - Камера, лидар, LED матрица, голосовой ассистент"
    echo -e "  ${GREEN}2)${NC} 🧠 Main Pi        - Главный узел, навигация, планирование"
    echo -e "  ${GREEN}3)${NC} 📡 Sensor Hub     - Датчики (IMU, GPS, энкодеры, мотор-драйверы)"
    echo -e "  ${GREEN}4)${NC} 🦾 Manipulator    - Манипулятор/рука (в разработке)"
    echo -e "  ${GREEN}5)${NC} 🛠️  Custom         - Пользовательская конфигурация"
    echo ""
    echo -e "${YELLOW}Примечание:${NC} Для Sensor Hub используйте ESP32 с micro-ROS"
    echo ""
}

detect_node_type() {
    log_step "Определение типа узла"
    
    # Проверяем аргумент командной строки
    if [ -n "$1" ]; then
        case "$1" in
            vision|Vision|VISION)
                NODE_TYPE="vision"
                log_info "Тип узла задан через параметр: Vision Pi"
                return
                ;;
            main|Main|MAIN)
                NODE_TYPE="main"
                log_info "Тип узла задан через параметр: Main Pi"
                return
                ;;
            sensor-hub|sensor|SENSOR)
                NODE_TYPE="sensor-hub"
                log_info "Тип узла задан через параметр: Sensor Hub"
                return
                ;;
            manipulator|arm|ARM)
                NODE_TYPE="manipulator"
                log_info "Тип узла задан через параметр: Manipulator"
                return
                ;;
            custom|CUSTOM)
                NODE_TYPE="custom"
                log_info "Тип узла задан через параметр: Custom"
                return
                ;;
            *)
                log_warning "Неизвестный тип узла: $1"
                ;;
        esac
    fi
    
    # Интерактивный выбор
    show_node_menu
    
    while true; do
        read -p "Введите номер (1-5): " choice
        case $choice in
            1)
                NODE_TYPE="vision"
                break
                ;;
            2)
                NODE_TYPE="main"
                break
                ;;
            3)
                NODE_TYPE="sensor-hub"
                break
                ;;
            4)
                NODE_TYPE="manipulator"
                break
                ;;
            5)
                NODE_TYPE="custom"
                break
                ;;
            *)
                echo -e "${RED}Неверный выбор. Попробуйте снова.${NC}"
                ;;
        esac
    done
    
    log_success "Выбран тип узла: $NODE_TYPE"
}

configure_node_params() {
    case "$NODE_TYPE" in
        vision)
            NODE_NAME="Vision Pi"
            NODE_IP="10.1.1.11"
            NODE_EMOJI="🎥"
            DOCKER_COMPOSE_DIR="$HOME/rob_box_project/docker/vision"
            SERVICE_NAME="robbox-vision"
            ;;
        main)
            NODE_NAME="Main Pi"
            NODE_IP="10.1.1.10"
            NODE_EMOJI="🧠"
            DOCKER_COMPOSE_DIR="$HOME/rob_box_project/docker/main"
            SERVICE_NAME="robbox-main"
            ;;
        sensor-hub)
            NODE_NAME="Sensor Hub"
            NODE_IP="10.1.1.20"
            NODE_EMOJI="📡"
            DOCKER_COMPOSE_DIR=""  # Нет Docker для ESP32
            SERVICE_NAME=""
            ;;
        manipulator)
            NODE_NAME="Manipulator"
            NODE_IP="10.1.1.30"
            NODE_EMOJI="🦾"
            DOCKER_COMPOSE_DIR="$HOME/rob_box_project/docker/manipulator"
            SERVICE_NAME="robbox-manipulator"
            ;;
        custom)
            NODE_NAME="Custom Node"
            NODE_IP="10.1.1.100"
            NODE_EMOJI="🛠️"
            DOCKER_COMPOSE_DIR=""
            SERVICE_NAME=""
            ;;
    esac
    
    log_info "Конфигурация узла:"
    log_info "  Имя: $NODE_EMOJI $NODE_NAME"
    log_info "  IP: $NODE_IP"
    [ -n "$DOCKER_COMPOSE_DIR" ] && log_info "  Docker: $DOCKER_COMPOSE_DIR"
}

# ═══════════════════════════════════════════════════════════════════════════
# УСТАНОВКА И НАСТРОЙКА
# ═══════════════════════════════════════════════════════════════════════════

check_raspberry_pi() {
    if [ ! -f /proc/device-tree/model ]; then
        log_warning "Не обнаружен Raspberry Pi, продолжаем..."
        return
    fi
    
    model=$(cat /proc/device-tree/model)
    log_info "Обнаружено устройство: $model"
}

install_docker() {
    log_step "Проверка Docker"
    
    if command -v docker &> /dev/null; then
        docker_version=$(docker --version)
        log_success "Docker уже установлен: $docker_version"
    else
        log_info "Docker не найден, начинаем установку..."
        
        log_info "Скачивание и установка Docker..."
        curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
        sudo sh /tmp/get-docker.sh
        rm /tmp/get-docker.sh
        
        sudo usermod -aG docker $USER
        
        log_success "Docker установлен успешно!"
        log_warning "Необходимо перелогиниться для применения группы docker"
    fi
}

check_docker_compose() {
    log_step "Проверка Docker Compose"
    
    if docker compose version &> /dev/null; then
        compose_version=$(docker compose version)
        log_success "Docker Compose установлен: $compose_version"
    else
        log_error "Docker Compose не найден!"
        log_info "Попробуйте: sudo apt install docker-compose-plugin"
        exit 1
    fi
}

install_dependencies() {
    log_step "Установка системных зависимостей"
    
    log_info "Обновление списка пакетов..."
    sudo apt update
    
    packages="git curl wget vim htop net-tools usbutils"
    
    log_info "Установка пакетов: $packages"
    sudo apt install -y $packages
    
    log_success "Зависимости установлены"
}

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

# ═══════════════════════════════════════════════════════════════════════════
# MOTD (Message of the Day)
# ═══════════════════════════════════════════════════════════════════════════

setup_motd() {
    log_step "Настройка кастомного MOTD"
    
    MOTD_SCRIPT="/usr/local/bin/robbox-motd"
    
    log_info "Создание скрипта MOTD..."
    sudo tee "$MOTD_SCRIPT" > /dev/null << MOTDEOF
#!/bin/bash
# РОББОКС - Dynamic MOTD for $NODE_NAME

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
HOSTNAME=\$(hostname)
UPTIME=\$(uptime -p | sed 's/up //')
MEMORY=\$(free -h | awk '/^Mem:/ {print \$3 "/" \$2}')
DISK=\$(df -h / | awk 'NR==2 {print \$3 "/" \$2 " (" \$5 ")"}')
LOAD=\$(cat /proc/loadavg | awk '{print \$1", "\$2", "\$3}')

# Температура (если доступна)
if command -v vcgencmd &> /dev/null; then
    CPU_TEMP=\$(vcgencmd measure_temp 2>/dev/null | cut -d'=' -f2 || echo "N/A")
    VOLTAGE=\$(vcgencmd measure_volts core 2>/dev/null | cut -d'=' -f2 || echo "N/A")
else
    CPU_TEMP="N/A"
    VOLTAGE="N/A"
fi

# IP адреса
ETH_IP=\$(ip -4 addr show eth0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || echo "N/A")
WLAN_IP=\$(ip -4 addr show wlan0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || echo "N/A")

# Docker контейнеры
if command -v docker &> /dev/null; then
    DOCKER_RUNNING=\$(docker ps --format "{{.Names}}" 2>/dev/null | wc -l)
    DOCKER_TOTAL=\$(docker ps -a --format "{{.Names}}" 2>/dev/null | wc -l)
    DOCKER_STATUS="\${GREEN}\${DOCKER_RUNNING}\${NC}/\${DOCKER_TOTAL} running"
else
    DOCKER_STATUS="\${RED}Not installed\${NC}"
fi

# Логотип РОББОКС
echo -e "\${PURPLE}"
cat << 'LOGO'
    ╔═══════════════════════════════════════════════════════════════════╗
    ║                                                                   ║
    ║  ██████╗   ██████╗   ╔██████ ██████╗   ██████╗  ██╗  ██╗ ██████╗  ║
    ║  ██╔══██╗ ██╔═══██╗ ╔╩═══╗██ ██╔═══╩╗ ██╔═══██╗ ██║ ██╔╝██╔════╝  ║
    ║  ██████╔╝ ██║   ██║ ╚╗██████ ██████╔╝ ██║   ██║ █████╔╝ ██║       ║
    ║  ██╔═══╝  ██║   ██║ ╔██══╗██ ██╔══██╗ ██║   ██║ ██╔═██╗ ██║       ║
    ║  ██║      ╚██████╔╝ ╚╗██████ ██████╔╝ ╚██████╔╝ ██║  ██╗╚██████╗  ║
    ║  ╚═╝       ╚═════╝   ╚═════╝ ╚═════╝   ╚═════╝  ╚═╝  ╚═╝ ╚═════╝  ║
    ║                                                                   ║
    ╚═══════════════════════════════════════════════════════════════════╝
LOGO
echo -e "\${NC}"
echo -e "\${CYAN}              $NODE_EMOJI $NODE_NAME ($NODE_IP) $NODE_EMOJI\${NC}"
echo ""

# Системная информация
echo -e "\${BOLD}\${CYAN}SYSTEM INFO\${NC}"
echo -e "  \${BLUE}Hostname:\${NC}       \$HOSTNAME"
echo -e "  \${BLUE}Uptime:\${NC}         \$UPTIME"
echo -e "  \${BLUE}Load Average:\${NC}   \$LOAD"
echo -e "  \${BLUE}Memory:\${NC}         \$MEMORY"
echo -e "  \${BLUE}Disk Usage:\${NC}     \$DISK"
echo ""

# Температура и питание
if [ "\$CPU_TEMP" != "N/A" ]; then
    echo -e "\${BOLD}\${YELLOW}HARDWARE\${NC}"
    echo -e "  \${BLUE}CPU Temp:\${NC}       \$CPU_TEMP"
    echo -e "  \${BLUE}Core Voltage:\${NC}   \$VOLTAGE"
    echo ""
fi

# Сеть
echo -e "\${BOLD}\${GREEN}NETWORK\${NC}"
echo -e "  \${BLUE}Ethernet (eth0):\${NC}  \$ETH_IP"
echo -e "  \${BLUE}WiFi (wlan0):\${NC}     \$WLAN_IP"
echo ""

# Docker контейнеры
echo -e "\${BOLD}\${PURPLE}DOCKER CONTAINERS\${NC}"
echo -e "  \${BLUE}Status:\${NC}         \$DOCKER_STATUS"

if command -v docker &> /dev/null && [ \$DOCKER_RUNNING -gt 0 ]; then
    echo ""
    docker ps --format "  \${GREEN}✓\${NC} {{.Names}}\t{{.Status}}" 2>/dev/null | column -t -s \$'\t'
fi

echo ""
echo -e "\${CYAN}═══════════════════════════════════════════════════════════════\${NC}"
echo -e "\${CYAN}Quick Commands:\${NC}"
if [ -n "$DOCKER_COMPOSE_DIR" ]; then
    echo -e "  \${BLUE}cd $DOCKER_COMPOSE_DIR\${NC}"
    echo -e "  \${BLUE}docker compose ps\${NC}                    - Check containers status"
    echo -e "  \${BLUE}docker compose logs -f\${NC}               - View all container logs"
    echo -e "  \${BLUE}docker compose restart\${NC}               - Restart all containers"
else
    echo -e "  \${BLUE}cd ~/rob_box_project\${NC}                 - Go to project directory"
fi
echo -e "\${CYAN}═══════════════════════════════════════════════════════════════\${NC}"
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

# ═══════════════════════════════════════════════════════════════════════════
# АВТОЗАПУСК DOCKER КОНТЕЙНЕРОВ
# ═══════════════════════════════════════════════════════════════════════════

setup_autostart() {
    if [ -z "$DOCKER_COMPOSE_DIR" ] || [ -z "$SERVICE_NAME" ]; then
        log_warning "Автозапуск Docker не настраивается для этого типа узла"
        return
    fi
    
    log_step "Настройка автозапуска Docker контейнеров"
    
    SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"
    
    log_info "Создание systemd service..."
    sudo tee "$SERVICE_FILE" > /dev/null << SERVICEEOF
[Unit]
Description=ROBBOX $NODE_NAME Docker Containers
After=docker.service network-online.target
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=$DOCKER_COMPOSE_DIR
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down
User=$USER

[Install]
WantedBy=multi-user.target
SERVICEEOF
    
    log_info "Активация service..."
    sudo systemctl daemon-reload
    sudo systemctl enable $SERVICE_NAME.service
    
    log_success "Автозапуск настроен!"
    log_info "Контейнеры будут автоматически запускаться при загрузке системы"
}

# ═══════════════════════════════════════════════════════════════════════════
# СПЕЦИФИЧНАЯ НАСТРОЙКА ДЛЯ РАЗНЫХ ТИПОВ УЗЛОВ
# ═══════════════════════════════════════════════════════════════════════════

setup_node_specific() {
    log_step "Специфичная настройка для $NODE_NAME"
    
    case "$NODE_TYPE" in
        vision)
            setup_vision_node
            ;;
        main)
            setup_main_node
            ;;
        sensor-hub)
            setup_sensor_hub
            ;;
        manipulator)
            setup_manipulator_node
            ;;
        custom)
            setup_custom_node
            ;;
    esac
}

setup_vision_node() {
    log_info "Настройка Vision Pi..."
    log_info "Проверка USB устройств..."
    
    # OAK-D камера
    if lsusb | grep -q "03e7"; then
        log_success "✓ OAK-D камера обнаружена"
    else
        log_warning "! OAK-D камера не найдена (подключите позже)"
    fi
    
    # LSLidar
    if lsusb | grep -q "10c4:ea60"; then
        log_success "✓ LSLidar обнаружен"
    else
        log_warning "! LSLidar не найден (подключите позже)"
    fi
    
    # ReSpeaker
    if lsusb | grep -q "2886"; then
        log_success "✓ ReSpeaker Mic Array обнаружен"
    else
        log_warning "! ReSpeaker не найден (подключите позже)"
    fi
    
    log_info "Проверка SPI для LED матрицы..."
    if grep -q "dtparam=spi=on" /boot/firmware/config.txt 2>/dev/null || grep -q "dtparam=spi=on" /boot/config.txt 2>/dev/null; then
        log_success "✓ SPI уже включен"
    else
        log_warning "! SPI не включен, добавьте в config.txt: dtparam=spi=on"
    fi
}

setup_main_node() {
    log_info "Настройка Main Pi..."
    log_info "Main Pi будет управлять:"
    log_info "  - micro-ROS agent (связь с ESP32)"
    log_info "  - Zenoh router (DDS мост)"
    log_info "  - Navigation stack (в разработке)"
    log_success "Базовая настройка завершена"
}

setup_sensor_hub() {
    log_info "Настройка Sensor Hub (ESP32)..."
    log_warning "Sensor Hub использует micro-ROS на ESP32"
    log_info "Для прошивки используйте:"
    log_info "  cd ~/rob_box_project/src/sensor_hub"
    log_info "  pio run -t upload"
    log_success "Инструкции добавлены"
}

setup_manipulator_node() {
    log_warning "Manipulator узел в разработке!"
    log_info "Эта функциональность будет доступна позже"
}

setup_custom_node() {
    log_info "Пользовательская настройка..."
    log_info "Вы можете настроить этот узел вручную"
    log_info "Репозиторий склонирован в ~/rob_box_project"
}

# ═══════════════════════════════════════════════════════════════════════════
# ИТОГОВАЯ ИНФОРМАЦИЯ
# ═══════════════════════════════════════════════════════════════════════════

print_summary() {
    log_step "Установка завершена!"
    
    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                     ✅ SETUP COMPLETED! ✅                         ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}Настроен узел:${NC} $NODE_EMOJI ${BOLD}$NODE_NAME${NC}"
    echo -e "${CYAN}IP адрес:${NC} $NODE_IP"
    echo ""
    echo -e "${CYAN}Что было сделано:${NC}"
    echo -e "  ${GREEN}✓${NC} Docker и Docker Compose установлены"
    echo -e "  ${GREEN}✓${NC} Репозиторий rob_box_project склонирован"
    echo -e "  ${GREEN}✓${NC} Кастомный MOTD с логотипом РОББОКС настроен"
    
    if [ -n "$DOCKER_COMPOSE_DIR" ]; then
        echo -e "  ${GREEN}✓${NC} Автозапуск Docker контейнеров настроен"
    fi
    
    echo ""
    echo -e "${YELLOW}Следующие шаги:${NC}"
    echo ""
    
    if groups $USER | grep -q docker; then
        echo -e "${BLUE}1. Docker готов к использованию${NC}"
    else
        echo -e "${BLUE}1. Перелогиниться для применения группы docker:${NC}"
        echo -e "   ${CYAN}exit${NC}"
        echo -e "   ${CYAN}ssh $USER@$NODE_IP${NC}"
        echo ""
    fi
    
    if [ -n "$DOCKER_COMPOSE_DIR" ]; then
        echo -e "${BLUE}2. Логин в GitHub Container Registry:${NC}"
        echo -e "   ${CYAN}echo YOUR_TOKEN | docker login ghcr.io -u YOUR_USERNAME --password-stdin${NC}"
        echo ""
        echo -e "${BLUE}3. Запуск Docker контейнеров:${NC}"
        echo -e "   ${CYAN}cd $DOCKER_COMPOSE_DIR${NC}"
        echo -e "   ${CYAN}docker compose pull${NC}   # Скачать образы"
        echo -e "   ${CYAN}docker compose up -d${NC}  # Запустить контейнеры"
        echo ""
        echo -e "${BLUE}4. Проверка статуса:${NC}"
        echo -e "   ${CYAN}docker compose ps${NC}"
        echo -e "   ${CYAN}docker compose logs -f${NC}"
    fi
    
    echo ""
    echo -e "${GREEN}Готово! Приятной работы с РОББОКС! 🤖${NC}"
    echo ""
}

# ═══════════════════════════════════════════════════════════════════════════
# MAIN EXECUTION
# ═══════════════════════════════════════════════════════════════════════════

main() {
    print_logo
    
    log_info "Начинаем автоматическую настройку узла РОББОКС..."
    log_info "Скрипт выполняется от пользователя: $USER"
    
    # Определяем тип узла
    detect_node_type "$1"
    configure_node_params
    
    # Базовая установка
    check_raspberry_pi
    install_dependencies
    install_docker
    check_docker_compose
    clone_repository
    
    # Настройка узла
    setup_motd
    setup_autostart
    setup_node_specific
    
    # Итоги
    print_summary
}

# Запуск главной функции с первым аргументом (тип узла)
main "$1"
