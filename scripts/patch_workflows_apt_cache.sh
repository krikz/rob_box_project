#!/bin/bash

# Патч для добавления APT_PROXY во все GitHub Actions workflows
# Автор: AI Assistant
# Дата: 2025-10-25

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WORKFLOWS_DIR="$PROJECT_ROOT/.github/workflows"

APT_PROXY_URL="http://192.168.1.125:3142"

echo "🔧 Патчинг GitHub Actions workflows для использования APT cache..."

# Функция для добавления APT_PROXY в build-args
add_apt_proxy() {
    local file="$1"
    local service_name="$2"
    
    echo "  📝 Патчинг $file для сервиса $service_name..."
    
    # Проверяем, есть ли уже APT_PROXY
    if grep -q "APT_PROXY=" "$file"; then
        echo "    ✅ APT_PROXY уже настроен в $file"
        return 0
    fi
    
    # Ищем секцию build-args и добавляем APT_PROXY
    if grep -q "build-args:" "$file"; then
        # Есть существующие build-args, добавляем APT_PROXY
        sed -i "/build-args:/a\\            APT_PROXY=$APT_PROXY_URL" "$file"
        echo "    ✅ Добавлен APT_PROXY в существующие build-args"
    else
        # Нет build-args, добавляем новую секцию перед cache-from
        sed -i "/cache-from: type=gha/i\\          build-args: |\n            APT_PROXY=$APT_PROXY_URL" "$file"
        echo "    ✅ Создана новая секция build-args с APT_PROXY"
    fi
}

# Список файлов для патчинга
WORKFLOW_FILES=(
    "build-vision-services.yml"
    "build-main-services.yml"
    "build-all.yml"
)

for workflow_file in "${WORKFLOW_FILES[@]}"; do
    file_path="$WORKFLOWS_DIR/$workflow_file"
    
    if [[ ! -f "$file_path" ]]; then
        echo "⚠️  Файл $workflow_file не найден, пропускаем..."
        continue
    fi
    
    echo "🔍 Обрабатываем $workflow_file..."
    
    # Создаем backup
    cp "$file_path" "$file_path.backup.$(date +%Y%m%d_%H%M%S)"
    
    # Патчим файл (добавляем APT_PROXY ко всем docker/build-push-action)
    # Используем более универсальный подход - ищем все uses: docker/build-push-action
    
    # Временный файл для обработки
    temp_file=$(mktemp)
    
    # Обрабатываем файл построчно
    while IFS= read -r line; do
        echo "$line" >> "$temp_file"
        
        # Если нашли docker/build-push-action@, то в следующих строках будет with:
        if [[ "$line" =~ uses:.*docker/build-push-action@ ]]; then
            in_build_section=true
        fi
        
        # Если мы в секции build и нашли cache-from, добавляем build-args перед ним
        if [[ "$in_build_section" == true ]] && [[ "$line" =~ cache-from: ]] && ! grep -q "APT_PROXY" "$temp_file"; then
            # Определяем отступ
            indent=$(echo "$line" | sed 's/cache-from.*//')
            echo "${indent}build-args: |" >> "$temp_file"
            echo "${indent}  APT_PROXY=$APT_PROXY_URL" >> "$temp_file"
        fi
        
        # Сбрасываем флаг после выхода из секции with:
        if [[ "$line" =~ ^[[:space:]]*- ]] && [[ "$in_build_section" == true ]]; then
            in_build_section=false
        fi
        
    done < "$file_path"
    
    # Заменяем оригинальный файл
    mv "$temp_file" "$file_path"
    
    echo "  ✅ Патч применен к $workflow_file"
done

echo ""
echo "🎉 Патчинг завершен!"
echo ""
echo "📊 Сводка изменений:"
for workflow_file in "${WORKFLOW_FILES[@]}"; do
    file_path="$WORKFLOWS_DIR/$workflow_file"
    if [[ -f "$file_path" ]]; then
        apt_proxy_count=$(grep -c "APT_PROXY=" "$file_path" || true)
        echo "  📁 $workflow_file: $apt_proxy_count секций с APT_PROXY"
    fi
done

echo ""
echo "🔍 Для проверки изменений:"
echo "  git diff .github/workflows/"
echo ""
echo "🚀 Для тестирования:"
echo "  git add .github/workflows/"
echo "  git commit -m 'feat: add APT cache proxy to all GitHub Actions workflows'"
echo "  git push"
