#!/bin/bash
# agent-flow-install-daily.sh — wrapper для ежедневного install.sh тика.
# Ретро 28.08 t_7ebdfce0: PR #1710 cleanup-логика доезжала только в
# часть профилей; ежедневный запуск install.sh закрывает gap без ручного
# запуска оператора. Используется из cron-job `agent-flow-install-daily`
# (no_agent=True, daily 03:00, devops-профиль). Stdout — silent если
# всё ОК (anti-escape OK + verify md5), иначе печатает diff.
set -e
REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
exec bash "$REPO_DIR/scripts/agent_flow/install.sh"
