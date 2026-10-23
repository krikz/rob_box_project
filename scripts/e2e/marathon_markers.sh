#!/usr/bin/env bash
# Поиск маркеров вердикта харнесса в логе акта (#3934).
#
# Маркер — это строка ВЫВОДА харнесса: `<ISO-время>Z E2E_NO_REACTION`.
# Эхо исходника шага workflow (`<время>Z ␛[36;1m… grep -q "E2E_NO_REACTION"`)
# маркером НЕ считается: после метки времени там идёт не маркер.
# Одно правило на все маркеры (E2E_NO_REACTION, E2E_INFRA_FAIL,
# E2E_FEATURE_FAIL, E2E_FATAL, E2E_VERDICT ...).

# marathon_has_marker <log> <alt1|alt2|...>  → 0, если есть строка-маркер.
marathon_has_marker() {
    local log="$1" markers="$2"
    grep -qE "(^|[[:space:]])[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9:.]+Z (>>> )?(${markers})([[:space:]]|\$)" "$log" 2>/dev/null
}
