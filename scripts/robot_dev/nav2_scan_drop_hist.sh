#!/usr/bin/env bash
# #5151: гистограмма лага дропа скана costmap по уже снятому логу. Сеть и робот не трогает.
#
# Лаг = log_stamp − scan_stamp. Окно таймаута фильтра —
# [transform_tolerance, transform_tolerance + 0.05]
# (scripts/nav2/scan_drop_lag.py: buffer_timeout + setTolerance(0.05)).
# Лаг в этом окне — таймаут MessageFilter, не «скан старше кэша».
# Строка лога троттлится 2500 мс, поэтому n — нижняя граница, не число сканов.
#
# НЕ запускать сбор лога без окна координатора. Робот заморожен (23.11):
# ни целей, ни /cmd_vel, ни голосовых команд, ни ros2 lifecycle get
# (гасит узлы через bond 4 с, см. #5150).
#
# Сбор, когда окно даст координатор (чтение, без рестарта):
#   ssh pi@10.1.1.10 'docker logs nav2' > /tmp/nav2_5151.log
#   ssh pi@10.1.1.10 'docker exec nav2 bash -lc \
#     "source /opt/ros/humble/setup.bash && ros2 param get \
#      /local_costmap/local_costmap transform_tolerance"'
#   ssh pi@10.1.1.10 'docker exec nav2 bash -lc \
#     "source /opt/ros/humble/setup.bash && ros2 param get \
#      /local_costmap/local_costmap tf_buffer_duration"'
# Ожидание по коду: Double value is: 1.0 и Parameter not set.
# Разбор снятого лога (эта команда робота не касается):
#   bash scripts/robot_dev/nav2_scan_drop_hist.sh /tmp/nav2_5151.log 1.0
set -euo pipefail

log_file=${1:?log file from docker logs nav2}
tolerance=${2:?transform_tolerance from ros2 param get}

python3 - "$log_file" "$tolerance" <<'PY'
import re
import sys

path, tolerance_text = sys.argv[1], sys.argv[2]
tolerance = float(tolerance_text)
if tolerance < 0.0:
    raise SystemExit(f"transform_tolerance must be >= 0, got {tolerance}")

log_re = re.compile(
    r"\[(\d+\.\d+)\].*Message Filter dropping message: frame 'lslidar_n10' at time (\d+\.\d+)"
)
lags = []
with open(path, encoding="utf-8", errors="replace") as handle:
    for line in handle:
        match = log_re.search(line)
        if match is None:
            continue
        log_stamp, scan_stamp = (float(part) for part in match.groups())
        lags.append(log_stamp - scan_stamp)

window_hi = tolerance + 0.05
on_timeout = [lag for lag in lags if tolerance <= lag <= window_hi]
tail = [lag for lag in lags if lag >= 3.0]
print(f"drops {len(lags)}")
print(f"on_filter_timeout {len(on_timeout)}/{len(lags)} window=[{tolerance:.3f}, {window_hi:.3f}]")
print(f"lag>=3s {len(tail)}")
if on_timeout:
    print(
        f"timeout_lag min={min(on_timeout):.5f} max={max(on_timeout):.5f} "
        f"avg={sum(on_timeout) / len(on_timeout):.5f}"
    )
if lags and not on_timeout:
    raise SystemExit("FAIL: ни один дроп не сидит на таймауте фильтра — гипотеза #5151 не подтверждена этим логом")
PY
