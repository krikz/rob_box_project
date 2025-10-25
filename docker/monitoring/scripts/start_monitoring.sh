#!/bin/bash
# Скрипт для запуска мониторинга на отдельной машине
# Использование: ./start_monitoring.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Функция для вывода ошибок
print_error() {
    echo -e "${RED}❌ ОШИБКА: $1${NC}"
}

# Функция для вывода предупреждений
print_warning() {
    echo -e "${YELLOW}⚠️  ПРЕДУПРЕЖДЕНИЕ: $1${NC}"
}

# Функция для вывода успеха
print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

# Функция для вывода информации
print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

echo "================================================"
echo "  Запуск стека мониторинга Rob Box"
echo "================================================"
echo ""

# Проверка 1: Docker установлен
print_info "Проверка зависимостей..."
if ! command -v docker &> /dev/null; then
    print_error "Docker не установлен"
    echo ""
    echo "Установите Docker следуя официальной инструкции:"
    echo "  https://docs.docker.com/engine/install/ubuntu/"
    echo ""
    echo "Или выполните команды:"
    echo "  curl -fsSL https://get.docker.com -o get-docker.sh"
    echo "  sudo sh get-docker.sh"
    echo ""
    exit 1
fi
print_success "Docker установлен"

# Проверка 2: Docker Compose установлен
if ! command -v docker-compose &> /dev/null; then
    print_error "docker-compose не установлен"
    echo ""
    echo "Установите docker-compose:"
    echo "  sudo apt update"
    echo "  sudo apt install docker-compose -y"
    echo ""
    echo "Для Ubuntu 22.04+ также можно использовать docker compose v2:"
    echo "  (уже включен в Docker Desktop и современные версии Docker)"
    echo ""
    exit 1
fi
print_success "docker-compose установлен"

# Проверка 3: Docker daemon запущен
if ! docker info &> /dev/null; then
    print_error "Docker daemon не запущен или нет доступа к Docker"
    echo ""
    echo "Возможные причины:"
    echo ""
    echo "1. Docker daemon не запущен:"
    echo "   sudo systemctl start docker"
    echo "   sudo systemctl enable docker"
    echo ""
    echo "2. Текущий пользователь не в группе docker:"
    echo "   sudo usermod -aG docker \$USER"
    echo "   newgrp docker"
    echo "   (или перелогиньтесь)"
    echo ""
    echo "3. Ошибка подключения к /var/run/docker.sock:"
    echo "   sudo chmod 666 /var/run/docker.sock"
    echo ""
    exit 1
fi
print_success "Docker daemon работает"

# Проверка 4: Пользователь в группе docker
if ! groups | grep -q docker; then
    print_warning "Пользователь не в группе docker"
    echo ""
    echo "Рекомендуется добавить пользователя в группу docker:"
    echo "  sudo usermod -aG docker \$USER"
    echo "  newgrp docker"
    echo ""
    echo "Продолжаем (используется sudo для docker команд)..."
    DOCKER_CMD="sudo docker-compose"
else
    print_success "Пользователь в группе docker"
    DOCKER_CMD="docker-compose"
fi

echo ""

# Проверяем наличие .env файла
if [ ! -f ".env" ]; then
    print_warning "Файл .env не найден. Создаём из .env.example..."
    cp .env.example .env
    echo ""
    echo "📝 Пожалуйста, отредактируйте .env файл:"
    echo "   - Укажите IP адреса Raspberry Pi (MAIN_PI_IP, VISION_PI_IP)"
    echo "   - Измените пароль Grafana (GRAFANA_PASSWORD)"
    echo ""
    echo "Затем запустите скрипт снова."
    exit 1
fi

# Загружаем переменные окружения
source .env

echo "Конфигурация:"
echo "  Main Pi:    ${MAIN_PI_IP}"
echo "  Vision Pi:  ${VISION_PI_IP}"
echo "  Robot ID:   ${ROBOT_ID:-rob_box_01}"
echo ""

# Запускаем стек мониторинга
print_info "Запуск контейнеров мониторинга..."
$DOCKER_CMD up -d

echo ""
echo "================================================"
print_success "Мониторинг успешно запущен!"
echo "================================================"
echo ""
echo "Доступ к сервисам:"
echo "  • Grafana:    http://localhost:3000 (admin/${GRAFANA_PASSWORD})"
echo "  • Prometheus: http://localhost:9090"
echo "  • Loki:       http://localhost:3100"
echo ""
echo "Для остановки: ./scripts/stop_monitoring.sh"
echo ""
print_info "Проверка статуса контейнеров:"
$DOCKER_CMD ps
echo ""
