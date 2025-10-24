#!/bin/bash
#
# fix_respeaker_noise.sh - Утилита для устранения белого шума ReSpeaker
#
# Проблема: После воспроизведения звука через ReSpeaker возникает постоянный белый шум
# Решение: Программное управление ALSA mixer для очистки playback channel
#
# Usage:
#   ./fix_respeaker_noise.sh        # Выполнить cleanup
#   ./fix_respeaker_noise.sh status # Показать текущие уровни
#

set -e

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ReSpeaker card name (может быть ArrayUAC10 или card2)
CARD_NAME="ArrayUAC10"
CARD_NUM="2"

# Функция для вывода с цветом
log_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_warn() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

# Проверка доступности ReSpeaker
check_respeaker() {
    log_info "Проверка доступности ReSpeaker..."
    
    if ! aplay -l | grep -q "ArrayUAC10"; then
        log_error "ReSpeaker (ArrayUAC10) не найден!"
        log_info "Доступные аудио устройства:"
        aplay -l
        return 1
    fi
    
    log_success "ReSpeaker найден: card $CARD_NUM (ArrayUAC10)"
    return 0
}

# Показать текущие настройки ALSA mixer
show_status() {
    log_info "Текущие настройки ALSA mixer для ReSpeaker:"
    echo ""
    
    # Playback controls
    if amixer -c $CARD_NAME sget 'Playback' &>/dev/null; then
        echo "Playback channel:"
        amixer -c $CARD_NAME sget 'Playback' | grep -E "Limits|Mono|Front"
    else
        log_warn "Playback control не найден"
    fi
    
    echo ""
    
    # Capture controls
    if amixer -c $CARD_NAME sget 'Capture' &>/dev/null; then
        echo "Capture channel:"
        amixer -c $CARD_NAME sget 'Capture' | grep -E "Limits|Mono|Front"
    else
        log_warn "Capture control не найден"
    fi
    
    echo ""
}

# Выполнить noise cleanup
cleanup_noise() {
    log_info "Выполнение noise cleanup для ReSpeaker..."
    
    # 1. Установить Playback в 0% (mute)
    log_info "Шаг 1/3: Mute playback channel..."
    if amixer -c $CARD_NAME sset 'Playback' 0% &>/dev/null; then
        log_success "Playback muted"
    else
        log_warn "Playback control не найден, пропускаю"
    fi
    
    # 2. Короткая пауза для стабилизации
    sleep 0.2
    
    # 3. Unmute playback channel (вернуть к нормальному уровню)
    log_info "Шаг 2/3: Unmute playback channel..."
    if amixer -c $CARD_NAME sset 'Playback' 95% &>/dev/null; then
        log_success "Playback unmuted (95%)"
    else
        log_warn "Playback control не найден, пропускаю"
    fi
    
    # 4. Оптимизировать capture level (если доступно)
    log_info "Шаг 3/3: Оптимизация capture level..."
    if amixer -c $CARD_NAME sset 'Capture' 95% &>/dev/null; then
        log_success "Capture level: 95%"
    else
        log_warn "Capture control не найден, пропускаю"
    fi
    
    echo ""
    log_success "Noise cleanup завершен!"
    log_info "Проверьте микрофон - белый шум должен уменьшиться"
}

# Reset mixer к значениям по умолчанию
reset_mixer() {
    log_info "Сброс ALSA mixer к значениям по умолчанию..."
    
    # Установить playback и capture в оптимальные значения
    if amixer -c $CARD_NAME sset 'Playback' 95% &>/dev/null; then
        log_success "Playback: 95%"
    fi
    
    if amixer -c $CARD_NAME sset 'Capture' 95% &>/dev/null; then
        log_success "Capture: 95%"
    fi
    
    log_success "Reset завершен"
}

# Main функция
main() {
    echo "=============================================="
    echo "🔊 ReSpeaker Noise Fix Utility"
    echo "=============================================="
    echo ""
    
    # Проверить доступность ReSpeaker
    if ! check_respeaker; then
        exit 1
    fi
    
    echo ""
    
    # Обработка аргументов
    case "${1:-cleanup}" in
        status)
            show_status
            ;;
        cleanup)
            cleanup_noise
            ;;
        reset)
            reset_mixer
            ;;
        *)
            echo "Usage: $0 {cleanup|status|reset}"
            echo ""
            echo "Commands:"
            echo "  cleanup  - Выполнить noise cleanup (по умолчанию)"
            echo "  status   - Показать текущие настройки mixer"
            echo "  reset    - Сбросить mixer к оптимальным значениям"
            exit 1
            ;;
    esac
    
    echo ""
    echo "=============================================="
}

# Запуск
main "$@"
