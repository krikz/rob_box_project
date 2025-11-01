#!/bin/bash

# Скрипт для автоматического создания релиза с семантическим версионированием
# Автоматически определяет следующую версию на основе существующих тегов
#
# Использование:
#   ./create_release.sh          # Интерактивный режим с меню выбора
#   ./create_release.sh major    # Автоматический режим для major релиза
#   ./create_release.sh minor    # Автоматический режим для minor релиза
#   ./create_release.sh patch    # Автоматический режим для patch релиза

set -e

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Функция для вывода заголовка
print_header() {
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
}

# Функция для вывода информации
print_info() {
    echo -e "${BLUE}ℹ${NC}  $1"
}

# Функция для вывода успеха
print_success() {
    echo -e "${GREEN}✓${NC}  $1"
}

# Функция для вывода предупреждения
print_warning() {
    echo -e "${YELLOW}⚠${NC}  $1"
}

# Функция для вывода ошибки
print_error() {
    echo -e "${RED}✗${NC}  $1"
}

# Получить последний тег версии
get_latest_version() {
    # Получаем все теги, которые соответствуют формату v?.?.? или ?.?.?
    local tags=$(git tag -l | grep -E '^v?[0-9]+\.[0-9]+\.[0-9]+$' | sed 's/^v//' | sort -V | tail -1)
    
    if [ -z "$tags" ]; then
        echo "0.0.0"
    else
        echo "$tags"
    fi
}

# Разобрать версию на компоненты
parse_version() {
    local version=$1
    # Убираем префикс 'v' если есть
    version=${version#v}
    
    IFS='.' read -r MAJOR MINOR PATCH <<< "$version"
    
    # Проверяем что все компоненты - числа
    if ! [[ "$MAJOR" =~ ^[0-9]+$ ]] || ! [[ "$MINOR" =~ ^[0-9]+$ ]] || ! [[ "$PATCH" =~ ^[0-9]+$ ]]; then
        print_error "Неверный формат версии: $version"
        exit 1
    fi
}

# Увеличить версию
bump_version() {
    local type=$1
    local current_version=$2
    
    parse_version "$current_version"
    
    case "$type" in
        major)
            MAJOR=$((MAJOR + 1))
            MINOR=0
            PATCH=0
            ;;
        minor)
            MINOR=$((MINOR + 1))
            PATCH=0
            ;;
        patch)
            PATCH=$((PATCH + 1))
            ;;
        *)
            print_error "Неверный тип версии: $type (должен быть major, minor или patch)"
            exit 1
            ;;
    esac
    
    echo "${MAJOR}.${MINOR}.${PATCH}"
}

# Показать меню выбора типа релиза
show_menu() {
    local current_version=$1
    
    print_header "Выбор типа релиза"
    
    parse_version "$current_version"
    
    local new_major="${CYAN}$((MAJOR + 1)).0.0${NC}"
    local new_minor="${CYAN}${MAJOR}.$((MINOR + 1)).0${NC}"
    local new_patch="${CYAN}${MAJOR}.${MINOR}.$((PATCH + 1))${NC}"
    
    echo "Текущая версия: ${YELLOW}${current_version}${NC}"
    echo ""
    echo "Выберите тип релиза:"
    echo ""
    echo "  ${GREEN}1)${NC} Major (несовместимые изменения API)"
    echo "     ${current_version} → ${new_major}"
    echo ""
    echo "  ${GREEN}2)${NC} Minor (новая функциональность, обратно совместимо)"
    echo "     ${current_version} → ${new_minor}"
    echo ""
    echo "  ${GREEN}3)${NC} Patch (исправления ошибок)"
    echo "     ${current_version} → ${new_patch}"
    echo ""
    echo "  ${RED}0)${NC} Отмена"
    echo ""
}

# Создать тег и запушить
create_and_push_tag() {
    local version=$1
    local tag_name="v${version}"
    local push_tag=$2
    
    print_header "Создание тега"
    
    # Проверяем что мы на правильной ветке
    local current_branch=$(git rev-parse --abbrev-ref HEAD)
    print_info "Текущая ветка: ${current_branch}"
    
    if [ "$current_branch" != "main" ] && [ "$current_branch" != "master" ]; then
        print_warning "Вы не на ветке main/master"
        read -p "Продолжить создание тега на ветке ${current_branch}? (y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            print_info "Отменено пользователем"
            exit 0
        fi
    fi
    
    # Проверяем что нет незакоммиченных изменений
    if ! git diff-index --quiet HEAD --; then
        print_error "Есть незакоммиченные изменения. Закоммитьте или отмените их перед созданием тега."
        exit 1
    fi
    
    # Создаём аннотированный тег
    print_info "Создание тега: ${tag_name}"
    
    # Запрашиваем описание релиза
    echo ""
    echo "Введите описание релиза (или нажмите Enter для использования по умолчанию):"
    read -p "> " release_description
    
    if [ -z "$release_description" ]; then
        release_description="Release version ${version}"
    fi
    
    # Создаём тег
    if git tag -a "$tag_name" -m "$release_description"; then
        print_success "Тег ${tag_name} успешно создан"
    else
        print_error "Ошибка при создании тега"
        exit 1
    fi
    
    # Пушим тег если требуется
    if [ "$push_tag" = true ]; then
        print_info "Отправка тега в удалённый репозиторий..."
        
        if git push origin "$tag_name"; then
            print_success "Тег ${tag_name} успешно отправлен в origin"
        else
            print_error "Ошибка при отправке тега"
            print_warning "Тег создан локально, но не был отправлен в удалённый репозиторий"
            print_info "Вы можете отправить его позже командой: git push origin ${tag_name}"
            exit 1
        fi
    else
        print_warning "Тег создан только локально"
        print_info "Для отправки в удалённый репозиторий выполните: git push origin ${tag_name}"
    fi
    
    echo ""
    print_success "Релиз ${version} успешно создан!"
    echo ""
    print_info "Следующие шаги:"
    echo "  1. Проверьте тег: git show ${tag_name}"
    echo "  2. Создайте release на GitHub: https://github.com/krikz/rob_box_project/releases/new?tag=${tag_name}"
    if [ "$push_tag" = false ]; then
        echo "  3. Отправьте тег: git push origin ${tag_name}"
    fi
    echo ""
}

# Основная функция
main() {
    print_header "🚀 Создание релиза РОББОКС"
    
    # Проверяем что мы в git репозитории
    if ! git rev-parse --git-dir > /dev/null 2>&1; then
        print_error "Это не git репозиторий!"
        exit 1
    fi
    
    # Получаем текущую версию
    print_info "Поиск существующих тегов версий..."
    local current_version=$(get_latest_version)
    
    if [ "$current_version" = "0.0.0" ]; then
        print_warning "Теги версий не найдены. Начинаем с версии 0.0.0"
    else
        print_success "Найдена текущая версия: ${current_version}"
    fi
    
    # Определяем режим работы
    local release_type=""
    
    if [ $# -eq 0 ]; then
        # Интерактивный режим
        while true; do
            show_menu "$current_version"
            read -p "Ваш выбор: " choice
            
            case $choice in
                1)
                    release_type="major"
                    break
                    ;;
                2)
                    release_type="minor"
                    break
                    ;;
                3)
                    release_type="patch"
                    break
                    ;;
                0)
                    print_info "Отменено пользователем"
                    exit 0
                    ;;
                *)
                    print_error "Неверный выбор. Попробуйте снова."
                    sleep 1
                    ;;
            esac
        done
    else
        # Автоматический режим
        release_type=$1
        
        if [[ ! "$release_type" =~ ^(major|minor|patch)$ ]]; then
            print_error "Неверный тип релиза: $release_type"
            echo ""
            echo "Использование: $0 [major|minor|patch]"
            echo ""
            echo "Типы релиза:"
            echo "  major - несовместимые изменения API (X.0.0)"
            echo "  minor - новая функциональность, обратно совместимо (x.Y.0)"
            echo "  patch - исправления ошибок (x.y.Z)"
            exit 1
        fi
    fi
    
    # Вычисляем новую версию
    local new_version=$(bump_version "$release_type" "$current_version")
    
    print_header "Подтверждение"
    echo "Текущая версия: ${YELLOW}${current_version}${NC}"
    echo "Новая версия:   ${GREEN}${new_version}${NC}"
    echo "Тип релиза:     ${CYAN}${release_type}${NC}"
    echo ""
    
    # Спрашиваем нужно ли пушить тег
    local push_tag=false
    read -p "Отправить тег в удалённый репозиторий сразу? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        push_tag=true
    fi
    
    echo ""
    read -p "Создать релиз с версией ${new_version}? (y/n): " -n 1 -r
    echo
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Отменено пользователем"
        exit 0
    fi
    
    # Создаём тег
    create_and_push_tag "$new_version" "$push_tag"
}

# Запуск
main "$@"
