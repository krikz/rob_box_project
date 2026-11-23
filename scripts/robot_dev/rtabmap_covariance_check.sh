#!/usr/bin/env bash
# #5161: приёмка ковариации позы rtabmap ПОСЛЕ следующего деплоя. Не запускать сейчас.
#
# Этот PR НЕ убирает WARN «has hessian not valid» из docker logs rtabmap.
# WARN пишет апстрим (OptimizerG2O), правка — честная строка в health perception,
# а не заглушка и не смена Optimizer/Strategy. Поэтому:
#   * count WARN за 25 мин ожидается НЕНУЛЕВЫМ. Ненулевой count — не провал этого PR.
#   * приёмка PR — строка «ковариация позы недоступна» в /perception/context_update
#     (health_issues), когда ковариация /rtabmap/localization_pose — единица или 9999.
#   * ноль WARN был бы бонусом, которого этот PR не обещает. Ноль без строки — тоже
#     не приёмка: строка должна быть, пока маргиналы не считаются.
#
# Робот заморожен (23.11): ни целей Nav2, ни /cmd_vel, ни голосовых команд движения,
# ни рестартов, ни правок на роботе. Этот скрипт — только чтение, и только когда
# координатор даст окно после деплоя.
#
# Сбор (чтение, без рестарта), когда окно будет:
#   ssh pi@10.1.1.10 'docker logs --since 25m rtabmap' > /tmp/rtabmap_5161.log
#   ssh pi@10.1.1.10 'docker logs --since 25m perception' > /tmp/perception_5161.log
# Разбор снятых логов (эта команда робота не касается):
#   bash scripts/robot_dev/rtabmap_covariance_check.sh /tmp/rtabmap_5161.log /tmp/perception_5161.log
set -euo pipefail

rtabmap_log=${1:?docker logs --since 25m rtabmap}
perception_log=${2:?docker logs --since 25m perception}

python3 - "$rtabmap_log" "$perception_log" <<'PY'
import sys

rtabmap_path, perception_path = sys.argv[1], sys.argv[2]
warn = "has hessian not valid"
honest = "ковариация позы недоступна"

with open(rtabmap_path, encoding="utf-8", errors="replace") as handle:
    warn_count = sum(1 for line in handle if warn in line)
with open(perception_path, encoding="utf-8", errors="replace") as handle:
    honest_count = sum(1 for line in handle if honest in line)

# WARN не глушим: ненулевой count — ожидаемо, не FAIL этого PR.
print(f"has hessian not valid: {warn_count} (expected non-zero; this PR does not remove the upstream WARN)")
print(f"ковариация позы недоступна: {honest_count}")
if honest_count == 0:
    raise SystemExit("FAIL: честной строки нет. Приёмка #5161 — строка, а не ноль WARN.")
print("PASS: честная строка есть. WARN оставлен апстриму, его count не критерий этого PR.")
PY
