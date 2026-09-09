#!/bin/bash
# Скрипт для быстрой валидации Dockerfiles
# Проверяет синтаксис и основные ошибки БЕЗ полной сборки

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "🔍 Validating Dockerfiles..."
echo ""

ERRORS=0
WARNINGS=0

# Функция проверки Dockerfile
check_dockerfile() {
    local dockerfile=$1
    local name=$(basename $(dirname "$dockerfile"))
    
    echo "📝 Checking: $dockerfile"
    
    # 1. Проверка существования файла
    if [ ! -f "$dockerfile" ]; then
        echo -e "${RED}❌ File not found${NC}"
        ((ERRORS++))
        return
    fi
    
    # 2. Проверка синтаксиса с помощью hadolint (если установлен)
    if command -v hadolint &> /dev/null; then
        if hadolint "$dockerfile" 2>&1 | grep -v "DL3008\|DL3009"; then
            echo -e "${YELLOW}⚠️  Hadolint warnings${NC}"
            ((WARNINGS++))
        fi
    fi
    
    # 3. Проверка COPY путей (не должно быть ../)
    if grep -n "COPY \.\./\.\." "$dockerfile" 2>/dev/null; then
        echo -e "${RED}❌ Found invalid COPY path with ../../${NC}"
        ((ERRORS++))
    fi
    
    # 4. Проверка устаревших пакетов
    if grep -n "nav2-recoveries" "$dockerfile" 2>/dev/null; then
        echo -e "${RED}❌ Found deprecated package: nav2-recoveries${NC}"
        ((ERRORS++))
    fi
    
    # 5. Проверка --break-system-packages без upgrade pip
    if grep "pip.*--break-system-packages" "$dockerfile" 2>/dev/null; then
        if ! grep -B5 "pip.*--break-system-packages" "$dockerfile" | grep -q "upgrade pip"; then
            echo -e "${YELLOW}⚠️  Using --break-system-packages without upgrading pip first${NC}"
            ((WARNINGS++))
        fi
    fi
    
    # 6. Проверка FROM с переменными
    if ! grep -q "^FROM.*\${" "$dockerfile" && ! grep -q "^FROM [a-z]" "$dockerfile"; then
        echo -e "${YELLOW}⚠️  FROM instruction might be missing${NC}"
        ((WARNINGS++))
    fi
    
    echo -e "${GREEN}✅ Basic checks passed${NC}"
    echo ""
}

# Проверяем проблемные Dockerfiles
echo "=== Main Services ==="
check_dockerfile "docker/main/micro_ros_agent/Dockerfile"
check_dockerfile "docker/main/nav2/Dockerfile"
check_dockerfile "docker/main/vesc_nexus/Dockerfile"
check_dockerfile "docker/main/robot_state_publisher/Dockerfile"

echo "=== Vision Services ==="
check_dockerfile "docker/vision/led_matrix/Dockerfile"

echo "=== Base Images ==="
check_dockerfile "docker/base/Dockerfile.ros2-zenoh"

echo ""
echo "================================"
echo "📊 Summary:"
echo -e "   Errors:   ${RED}$ERRORS${NC}"
echo -e "   Warnings: ${YELLOW}$WARNINGS${NC}"
echo "================================"

if [ $ERRORS -gt 0 ]; then
    echo -e "${RED}❌ Validation failed with $ERRORS error(s)${NC}"
    exit 1
else
    echo -e "${GREEN}✅ All Dockerfiles passed validation${NC}"
fi
