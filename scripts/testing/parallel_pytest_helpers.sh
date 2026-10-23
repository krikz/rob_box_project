#!/usr/bin/env bash
# Хелперы параллельного прогона unit-тестов (#3937). Подключается через `source` из шага
# «Run unit tests» в .github/workflows/G-Run Tests.yml.
#
# Вынесено из workflow в файл: шаг `run:` с этими хелперами (≈22 КБ в UTF-8) переваливал за
# лимит GitHub на размер скрипта (≈21000 байт): workflow переставал парситься, прогоны не
# создавались (develop, 14:00 UTC 23.10, run 39965919381). Не раздувай шаг обратно.

# ── Параллельный прогон (#3937) ─────────────────────────────────────
# У раннера 4 vCPU.  Два приёма, оба сохраняют изоляцию там, где она
# нужна:
#   * пакеты «один pytest» (voice / music / animations) — pytest-xdist
#     ``-n $XDIST_N`` (плагин запечён в образ rob-box-ci:humble).
#     Если с -n 4 падает то, что последовательно зелёное, — это
#     настоящая зависимость от порядка/общего состояния: чинить её, а
#     не возвращать пакет в последовательный режим молча.
#   * пофайловые циклы (telegram / quest) — по-прежнему ОДИН
#     ПРОЦЕСС НА ФАЙЛ (cross-file pollution, issue #1196 / #2232),
#     но 4 процесса одновременно через ``xargs -P``.  ``xdist
#     --dist loadfile`` сюда не годится: воркер xdist гоняет много
#     файлов в одном интерпретаторе, и загрязнение вернулось бы.
#     Вывод каждого файла идёт в свой лог и печатается целиком
#     после прогона, в алфавитном порядке, — не перемешан.
export PF_JOBS=4

# pf_run_one <файл>: окружение — PF_DIR (cwd для pytest), PF_TIMEOUT,
# PF_LOGDIR, PF_PYTEST_ARGS (строка дополнительных флагов pytest).
# Всегда возвращает 0: код выхода pytest уходит в <ключ>.rc, чтобы
# xargs не прерывал остальные файлы, а решение принималось ниже по
# коду выхода (шаг идёт под ``set -e``).
pf_run_one() {
  local f="$1" key rc=0
  key="${f#"$PF_DIR"/}"; key="${key//\//__}"
  ( cd "$PF_DIR" && PYTHONPATH=".:$PYTHONPATH" timeout "$PF_TIMEOUT" python3 -m pytest \
      -p no:launch_testing \
      -p no:ament_flake8 -p no:ament_pep257 \
      -p no:ament_copyright -p no:ament_xmllint \
      -p no:ament_lint -p no:colcon_core \
      --tb=short --no-header $PF_PYTEST_ARGS \
      "$f" ) > "$PF_LOGDIR/$key.log" 2>&1 || rc=$?
  echo "$rc" > "$PF_LOGDIR/$key.rc"
  return 0
}
export -f pf_run_one

# pf_run <метка> <каталог-пакета> <таймаут> <флаги-pytest> <файл>...
# Гонит файлы по PF_JOBS штук одновременно, затем печатает логи по
# порядку.  Коды выхода остаются в $PF_LOGDIR/*.rc (читает pf_rc).
pf_run() {
  local label="$1"; shift
  export PF_DIR="$1" PF_TIMEOUT="$2" PF_PYTEST_ARGS="$3"; shift 3
  PF_LOGDIR="$(mktemp -d)"; export PF_LOGDIR
  printf '%s\0' "$@" | xargs -r -0 -n 1 -P "$PF_JOBS" bash -c 'pf_run_one "$1"' _
  local f key
  for f in "$@"; do
    key="${f#"$PF_DIR"/}"; key="${key//\//__}"
    echo "--- $label: $(basename "$f") ---"
    cat "$PF_LOGDIR/$key.log"
  done
}
# pf_rc <файл>: код выхода pytest для файла из последнего pf_run.
pf_rc() {
  local key="${1#"$PF_DIR"/}"
  cat "$PF_LOGDIR/${key//\//__}.rc"
}
