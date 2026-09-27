#
# Решение: ensure_pr_backlog_digest_cron() — идемпотентная функция,
# регистрирующая interval-job (every 24h, target 09:00 Europe/Berlin) в
# devops-профиле, no_agent (скрипт = agent-flow-pr-backlog-digest.sh).
# Сам скрипт дополнительно фильтрует окно по DIGEST_HOUR (default 9) и
# sentinel /tmp/agent-flow-pr-backlog-digest-YYYY-MM-DD.done, чтобы
# двойной cron-tick не слал 2 раза в день. Interval "every 24h" даёт
# cron-планировщику шанс запустить (09:00-09:59 local); скрипт сам решит,
# отправлять ли.
#
# Регистрация переживает install.sh: каждый запуск (в т.ч. auto-fix из
# drift-detect) проверяет jobs.json и создаёт недостающий job.
ensure_pr_backlog_digest_cron() {
    ensure_cron_job devops "Agent Flow PR Backlog Digest (Шифу daily)" "agent-flow-pr-backlog-digest.sh" "every 1h" interval
}
ensure_pr_backlog_digest_cron
echo "==> Ensure cron job registration: orphan needs-e2e sweep (ретро t_78a6ffa3)"
# Проблема: agent-flow-needs-e2e-orphan-watchdog.sh раскладывается install.sh,
# но cron-job НЕ создаётся автоматически. Без него паттерн «needs-e2e без PR»
# (11 issues на 14.09: 8 never-had-PR, 3 PR merged but orphan-stale) будет
# повторяться каждые сутки — Шифу придётся делать ручной cleanup по