# gh_config_dir.sh — откуда читать конфиг gh.
#
# Подключается через ``source`` из scripts/dev/gh-pr-create-via-gh-api.sh и
# scripts/dev/push-via-gh-api.sh. Сам токен не читает и сеть не трогает.
#
# Резолв (первый существующий каталог):
#   1. GH_CONFIG_DIR из окружения — явный выбор вызывающего, не подменяем.
#   2. ${XDG_CONFIG_HOME:-$HOME/.config}/gh — конфиг текущего пользователя.
#   3. /home/builder/.config/gh — прежний путь билдовой машины, если он есть.
# Нет ни одного каталога — пустая строка: вызывающий не задаёт GH_CONFIG_DIR,
# и ``gh auth token`` берёт свой конфиг сам.

gh_config_dir_resolve() {
    if [ -n "${GH_CONFIG_DIR:-}" ]; then
        printf '%s\n' "$GH_CONFIG_DIR"
        return 0
    fi
    local user_dir="${XDG_CONFIG_HOME:-${HOME:-}/.config}/gh"
    if [ -d "$user_dir" ]; then
        printf '%s\n' "$user_dir"
        return 0
    fi
    if [ -d /home/builder/.config/gh ]; then
        printf '%s\n' /home/builder/.config/gh
        return 0
    fi
    printf '\n'
}
