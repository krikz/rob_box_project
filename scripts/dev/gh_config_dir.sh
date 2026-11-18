# gh_config_dir.sh — откуда читать конфиг gh.
#
# Подключается через ``source`` из scripts/dev/gh-pr-create-via-gh-api.sh и
# scripts/dev/push-via-gh-api.sh. Сам токен не читает и сеть не трогает.
#
# Резолв (первый существующий каталог):
#   1. GH_CONFIG_DIR из окружения — явный выбор вызывающего, не подменяем.
#   2. $XDG_CONFIG_HOME/gh — только если XDG задан и каталог есть.
#   3. $HOME/.config/gh — конфиг текущего пользователя.
#   4. /home/builder/.config/gh — прежний путь билдовой машины, если он есть.
# Заданный, но пустой XDG не прячет домашний конфиг: на CI XDG_CONFIG_HOME
# бывает без подкаталога gh. Нет ни одного каталога — пустая строка:
# вызывающий не задаёт GH_CONFIG_DIR, ``gh auth token`` берёт свой конфиг сам.

gh_config_dir_resolve() {
    if [ -n "${GH_CONFIG_DIR:-}" ]; then
        printf '%s\n' "$GH_CONFIG_DIR"
        return 0
    fi
    if [ -n "${XDG_CONFIG_HOME:-}" ] && [ -d "${XDG_CONFIG_HOME}/gh" ]; then
        printf '%s\n' "${XDG_CONFIG_HOME}/gh"
        return 0
    fi
    if [ -n "${HOME:-}" ] && [ -d "${HOME}/.config/gh" ]; then
        printf '%s\n' "${HOME}/.config/gh"
        return 0
    fi
    if [ -d /home/builder/.config/gh ]; then
        printf '%s\n' /home/builder/.config/gh
        return 0
    fi
    printf '\n'
}
