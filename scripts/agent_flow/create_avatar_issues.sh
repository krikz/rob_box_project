#!/usr/bin/env bash
# scripts/agent_flow/create_avatar_issues.sh
#
# Создаёт 11 worker-issue для Avatar-эпика (AV-2 ... AV-11) в
# krikz/rob_box_project под milestone M4. Идемпотентный: если issue с
# таким title уже есть — пропускает.
#
# Usage:
#   ./scripts/agent_flow/create_avatar_issues.sh              # создать
#   ./scripts/agent_flow/create_avatar_issues.sh --dry-run    # показать payload, не создавать
#
# Зависимости: gh (GitHub CLI), авторизация через gh auth status.
# Автор плана: architect, kanban t_c035d460.

set -euo pipefail

REPO="${GH_REPO:-krikz/rob_box_project}"
MILESTONE_NUM="${MILESTONE_NUM:-4}"   # M4: Avatar (Quest + Telegram)
MILESTONE_TITLE="${MILESTONE_TITLE:-M4: Avatar (Quest + Telegram)}"
DRY_RUN=0

[[ "${1:-}" == "--dry-run" ]] && DRY_RUN=1

# Общие labels для всех issue (AGENTS.md / kanban convention)
COMMON_LABELS=(hermes source:gsd)

# Проверка gh auth
if ! gh auth status >/dev/null 2>&1; then
  echo "� gh не авторизован. Выполни: gh auth login" >&2
  exit 1
fi

# Проверка milestone
milestone_title=$(gh api "repos/${REPO}/milestones/${MILESTONE_NUM}" --jq '.title' 2>/dev/null || echo "")
if [[ -z "${milestone_title}" ]]; then
  echo "❌ Milestone ${MILESTONE_NUM} не найден в ${REPO}. Проверь:" >&2
  gh api "repos/${REPO}/milestones" --jq '.[] | "\(.number)\t\(.title)"' >&2
  exit 1
fi
echo "✅ Repo: ${REPO}, milestone: ${milestone_title} (#${MILESTONE_NUM})"

# ---------------------------------------------------------------------------
# Helper: создать issue с проверкой дубликата по title
# ---------------------------------------------------------------------------
create_issue() {
  local title="$1"
  local assignee_label="$2"
  local type_label="$3"
  local priority_label="$4"
  local e2e_label="$5"  # 'needs-e2e' или 'no-e2e-required'
  local body="$6"

  # Проверка дубликата (точный поиск по in:title — substring-match ломает на кириллице).
  # Кодируем title для URL (квадратные скобки, кириллица, плюсы).
  local encoded_query
  encoded_query=$(python3 -c "import urllib.parse, sys; print(urllib.parse.quote(sys.argv[1]))" "${title}" 2>/dev/null || echo "")
  local existing
  if [[ -n "${encoded_query}" ]]; then
    existing=$(gh api "search/issues?q=repo:${REPO}+is:issue+in:title+${encoded_query}" \
      --jq '.items[0].number // empty' 2>/dev/null || echo "")
  else
    existing=""
  fi

  if [[ -n "${existing}" ]]; then
    echo "�  SKIP: ${title} (уже #${existing})"
    return 0
  fi

  # Собрать labels
  local labels=("${COMMON_LABELS[@]}" "${assignee_label}" "${type_label}" "${priority_label}" "${e2e_label}")

  if [[ ${DRY_RUN} -eq 1 ]]; then
    echo "🟦 DRY-RUN: would create: ${title}"
    echo "   labels: ${labels[*]}"
    echo "   body (first 5 lines):"
    echo "${body}" | head -5 | sed 's/^/   | /'
    echo ""
    return 0
  fi

  local out
  out=$(gh issue create \
    --repo "${REPO}" \
    --milestone "${milestone_title}" \
    --title "${title}" \
    --label "$(IFS=,; echo "${labels[*]}")" \
    --body "${body}" 2>&1) || {
    echo "❌ Failed to create ${title}: ${out}" >&2
    return 1
  }

  # gh issue create пишет URL вида https://github.com/owner/repo/issues/N
  local num
  num=$(echo "${out}" | grep -oE '/issues/[0-9]+' | grep -oE '[0-9]+' || echo "?")
  echo "✅ Created: ${title} → #${num}"
  return 0
}

# ---------------------------------------------------------------------------
# AV-2: package skeleton src/rob_box_supervisor/
# ---------------------------------------------------------------------------
create_issue \
  "[AV-2] rob_box_supervisor: package skeleton (setup.py / pytest.ini / smoke-import)" \
  "agent:backend" \
  "type:infrastructure" \
  "priority:medium" \
  "needs-e2e" \
  "$(cat <<'EOF'
## Контекст

Каркас нового ROS2-пакета \`src/rob_box_supervisor/\` по образцу
\`src/rob_box_voice/\`. Чистая инфраструктурная задача (TDD: smoke-import).

Источники истины:
- \`src/rob_box_voice/package.xml\`, \`setup.py\`, \`setup.cfg\`, \`pytest.ini\` (образец)
- \`docs/adr/0028-avatar-supervisor.md\` §4.6 (целевая структура пакета)
- \`docs/plans/2026-08-24-avatar-decomposition.md\` §4 (AV-2 acceptance)

## Acceptance criteria

- [ ] \`src/rob_box_supervisor/package.xml\` создан по образцу voice, deps
      \`rclpy\`, \`std_msgs\`, \`geometry_msgs\`, exec_depend \`python3-msgpack\`.
- [ ] \`src/rob_box_supervisor/setup.py\` — entry_point
      \`supervisor_node = rob_box_supervisor.supervisor_node:main\`.
- [ ] \`src/rob_box_supervisor/setup.cfg\` + \`pytest.ini\` (\`testpaths = test/unit\`).
- [ ] \`src/rob_box_supervisor/rob_box_supervisor/__init__.py\` (пустой, \`__version__\`).
- [ ] \`test/unit/test_import.py::test_import_module\` — \`import rob_box_supervisor\` → OK.
- [ ] \`pytest src/rob_box_supervisor/test/unit -q\` → GREEN.
- [ ] \`black --check --line-length 120 src/rob_box_supervisor\` → PASS.
- [ ] \`flake8 src/rob_box_supervisor\` → PASS (max-complexity=10).

## Branch

\`feature/av-2-supervisor-package-skeleton\` (от \`feature/avatar\`).

## Commit

\`\`\`bash
git commit -m "feat(supervisor): package skeleton + smoke-import (TDD)"
\`\`\`

## Definition of Done

- [ ] PR открыт в \`feature/avatar\`, прошёл \`agent-flow-merge-gate\`.
- [ ] \`pytest -v\` raw-вывод в PR-описании.
- [ ] \`black --check\` + \`flake8\` raw-вывод в PR-описании.

## Связанные

- Родительский план: \`docs/plans/2026-08-24-avatar-decomposition.md\`
- Блокирует: AV-3, AV-4, AV-5, AV-6
EOF
)"

# ---------------------------------------------------------------------------
# AV-3: FSM ModeManager (чистая логика, TDD)
# ---------------------------------------------------------------------------
create_issue \
  "[AV-3] rob_box_supervisor: FSM ModeManager off/telegram_active/avatar_present/mixed" \
  "agent:backend" \
  "type:functional" \
  "priority:high" \
  "no-e2e-required" \
  "$(cat <<'EOF'
## Контекст

State-machine режимов аватара (ADR-0028 §4.1) — чистая Python-логика, без ROS.
Блокируется AV-2, блокирует AV-6.

Источники истины:
- \`docs/adr/0028-avatar-supervisor.md\` §4.1 (mermaid-stateDiagram, таблица режимов)
- \`docs/adr/0028-avatar-supervisor.md\` §6 Q1 (fail-safe на рестарт dialogue_node)
- \`docs/plans/2026-08-24-avatar-decomposition.md\` §4 (AV-3 acceptance)

## Acceptance criteria

- [ ] \`src/rob_box_supervisor/rob_box_supervisor/core/fsm.py\` — класс
      \`ModeManager\` с методами \`transition(event: Event) -> Mode | ConflictError\`.
- [ ] Таблица переходов = копия ADR-0028 §4.1 mermaid-stateDiagram.
- [ ] Режимы: \`off\`, \`telegram_active\`, \`avatar_present\`, \`mixed\`.
- [ ] Тесты \`test/unit/core/test_fsm.py\` (≥ 10 кейсов):
  1. \`off → telegram_active\` (\`telegram_acquire_floor\`)
  2. \`off → avatar_present\` (\`quest_acquire_floor\`)
  3. \`telegram_active → mixed\` (\`quest_acquire_floor teleop only\`)
  4. \`avatar_present → mixed\` (\`telegram_acquire_voice_floor\`)
  5. \`mixed → off\` (\`both_release\`)
  6. \`telegram_active → off\` (timeout 30 с, fake clock)
  7. Конфликт: попытка \`off → avatar_present\` при held \`voice_floor\` → \`ConflictError\`
  8. \`* → off\` всегда разрешён (escape hatch)
  9. Идемпотентность: повторный acquire от того же клиента — no-op
  10. Неизвестное событие → \`ValueError\`
- [ ] \`pytest -v src/rob_box_supervisor/test/unit/core/test_fsm.py\` → GREEN (raw-evidence).
- [ ] \`black --check --line-length 120\` + \`flake8\` → PASS.

## Branch

\`feature/av-3-supervisor-fsm\` (от \`feature/avatar\`).

## Commit

\`\`\`bash
git commit -m "feat(supervisor): FSM ModeManager off/telegram_active/avatar_present/mixed (TDD)"
\`\`\`

## Definition of Done

- [ ] PR в \`feature/avatar\`, raw-evidence (\`pytest -v\` с полным -v, не "тесты прошли").
- [ ] \`flake8\` + \`black\` raw-вывод.

## Связанные

- Блокируется: AV-2
- Блокирует: AV-6
EOF
)"

# ---------------------------------------------------------------------------
# AV-4: LockManager teleop_floor / voice_floor (чистая логика, TDD)
# ---------------------------------------------------------------------------
create_issue \
  "[AV-4] rob_box_supervisor: LockManager teleop_floor/voice_floor + 500ms dead-man" \
  "agent:backend" \
  "type:functional" \
  "priority:high" \
  "no-e2e-required" \
  "$(cat <<'EOF'
## Контекст

Менеджер floor-ов — кто держит «право руля» и «право голоса». Чистая логика,
TDD, без ROS. ADR-0028 §4.2 + §6 Q4 (dead-man 500 мс).

Источники истины:
- \`docs/adr/0028-avatar-supervisor.md\` §4.2 (LockManager)
- \`docs/adr/0028-avatar-supervisor.md\` §6 Q4 (dead-man 500 мс)
- \`docs/plans/2026-08-24-avatar-decomposition.md\` §4 (AV-4 acceptance)

## Acceptance criteria

- [ ] \`src/rob_box_supervisor/rob_box_supervisor/core/locks.py\` — класс
      \`LockManager\` с методами:
      - \`acquire(client_id, floor) -> None | ConflictError\`
      - \`release(client_id, floor) -> None\`
      - \`heartbeat(client_id, floor, now_ms) -> None\`
      - \`holder(floor) -> client_id | None\`
- [ ] Два независимых floor-а: \`teleop_floor\`, \`voice_floor\`.
- [ ] Dead-man: \`heartbeat\` не вызывался > 500 мс → \`holder\` возвращает \`None\`.
- [ ] Тесты \`test/unit/core/test_locks.py\` (≥ 8 кейсов):
  1. acquire → holder; release → None
  2. acquire(telegram, teleop), acquire(quest, voice) — оба держат (разные floors)
  3. acquire(quest, voice), acquire(telegram, voice) → второго \`ConflictError\`
  4. heartbeat refreshes; no heartbeat 500 мс → auto-release
  5. release by wrong client_id → \`PermissionError\`
  6. heartbeat by wrong client_id → \`PermissionError\`
  7. release уже-released floor — no-op (идемпотентность)
  8. \`holder(None)\` → \`ValueError\`
- [ ] \`pytest -v src/rob_box_supervisor/test/unit/core/test_locks.py\` → GREEN.
- [ ] \`black\` + \`flake8\` чисто.

## Branch

\`feature/av-4-supervisor-lock-manager\` (от \`feature/avatar\`).

## Commit

\`\`\`bash
git commit -m "feat(supervisor): LockManager teleop_floor/voice_floor + 500ms dead-man (TDD)"
\`\`\`

## Definition of Done

- [ ] PR в \`feature/avatar\`, raw-evidence (\`pytest -v\` с -v).

## Связанные

- Блокируется: AV-2
- Блокирует: AV-6
EOF
)"

# ---------------------------------------------------------------------------
# AV-5: /avatar/state msgpack schema (чистая логика, TDD)
# ---------------------------------------------------------------------------
create_issue \
  "[AV-5] rob_box_supervisor: /avatar/state msgpack schema + serialization round-trip" \
  "agent:backend" \
  "type:functional" \
  "priority:medium" \
  "no-e2e-required" \
  "$(cat <<'EOF'
## Контекст

Контракт \`/avatar/state\` (msgpack-encoded в \`std_msgs/String\` для совместимости).
Без публикации в ROS, только dataclasses + pack/unpack. ADR-0028 §4.3.

Источники истины:
- \`docs/adr/0028-avatar-supervisor.md\` §4.3 (\`AvatarState.msg\`)
- \`docs/architecture/SYSTEM_OVERVIEW.md\` §5.4 (таблица /avatar/state)
- \`docs/plans/2026-08-24-avatar-decomposition.md\` §4 (AV-5 acceptance)

## Acceptance criteria

- [ ] \`src/rob_box_supervisor/rob_box_supervisor/core/state.py\` — dataclasses:
      - \`AvatarState(mode, teleop_floor, voice_floor, last_event, since_ms, version)\`
      - \`FloorState(client_id, since_ms, last_heartbeat_ms)\`
      - \`AvatarEvent(timestamp_ms, client_id, kind, args)\`
- [ ] Функции \`pack(state: AvatarState) -> bytes\` (msgpack) и
      \`unpack(data: bytes) -> AvatarState\` (round-trip + version check).
- [ ] Тесты \`test/unit/core/test_state.py\` (≥ 6 кейсов):
  1. Round-trip \`AvatarState\` (все поля заполнены).
  2. Round-trip с None floors (off-режим).
  3. Forward-compat: payload c version+1 → либо raise, либо игнор новых полей
     (зафиксировать выбор в docstring).
  4. Backward-compat: payload от старого клиента без \`last_event\` → defaults.
  5. Размер типичного payload (off-режим) ≤ 200 байт (для 1 Hz публикации).
  6. msgpack из \`pack\` десериализуется внешним \`msgpack.unpackb\` (контракт).
- [ ] \`pytest -v\` GREEN, \`black\` + \`flake8\` чисто.

## Branch

\`feature/av-5-avatar-state-msgpack-schema\` (от \`feature/avatar\`).

## Commit

\`\`\`bash
git commit -m "feat(supervisor): /avatar/state msgpack schema + serialization round-trip (TDD)"
\`\`\`

## Definition of Done

- [ ] PR в \`feature/avatar\`, raw-evidence (\`pytest -v\`).

## Связанные

- Блокируется: AV-2
- Блокирует: AV-6
EOF
)"

# ---------------------------------------------------------------------------
# AV-6: supervisor_node.py в monitor-режиме
# ---------------------------------------------------------------------------
create_issue \
  "[AV-6] rob_box_supervisor: supervisor_node.py в monitor-режиме (Phase 1 safe deploy)" \
  "agent:backend" \
  "type:functional" \
  "priority:high" \
  "needs-e2e" \
  "$(cat <<'EOF'
## Контекст

ROS2-нода, которая принимает сервисы AcquireFloor/ReleaseFloor/SetAvatarMode,
публикует \`/avatar/state\`, но в Phase 1 НЕ меняет \`twist_mux\` inputs и
НЕ правит \`dialogue_node\` параметры. Это позволяет задеплоить supervisor
без blast radius — включаем \`mode:=active\` только после закрытия AV-7+AV-8+AV-10.

Источники истины:
- \`docs/adr/0028-avatar-supervisor.md\` §4.5 (monitor-режим, детально)
- \`docs/adr/0028-avatar-supervisor.md\` §4.3 (ROS 2 API)
- \`docs/architecture/SYSTEM_OVERVIEW.md\` §5.4 (\`/avatar/state\` контракт)
- \`src/rob_box_voice/.../dialogue_node.py:415\` — топик \`/voice/dialogue/state\`
  (НЕ \`/voice/state\` — расхождение ADR-0027 #2)
- \`docs/plans/2026-08-24-avatar-decomposition.md\` §4 (AV-6 acceptance)

## Acceptance criteria

- [ ] \`src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py\` —
      ROS2-нода (\`rclpy\`, Zenoh session через \`ZENOH_SESSION_CONFIG_URI\`).
- [ ] Параметр \`mode\` (default \`"monitor"\`). В monitor-режиме:
  - Публикует \`/avatar/state\` (latched, \`transient_local\`) из агрегатора.
  - Принимает сервисы \`AcquireFloor\`, \`ReleaseFloor\`, \`SetAvatarMode\`,
    пишет в лог, отвечает \`success=true, applied=false,
    reason="supervisor_in_monitor_mode"\`.
  - НЕ меняет \`twist_mux\` inputs, НЕ правит \`dialogue_node\` параметры.
- [ ] Агрегатор собирает из \`/odom\`, \`/device/snapshot\`,
      \`/voice/dialogue/state\`.
- [ ] \`dead_man_trips_total{client_id}\` счётчик — в \`/avatar.state.last_event\`
      (ADR-0028 §6 Q4).
- [ ] Unit-тесты с mock rclpy и fake aggregator: heartbeat трип → лог,
      \`/avatar/state\` обновляется (raw-evidence).
- [ ] \`black\` + \`flake8\` чисто.

## Branch

\`feature/av-6-supervisor-node-monitor\` (от \`feature/avatar\`).

## Commit

\`\`\`bash
git commit -m "feat(supervisor): supervisor_node.py in monitor mode (Phase 1 safe deploy)"
\`\`\`

## Definition of Done

- [ ] PR в \`feature/avatar\`, raw-evidence (\`pytest -v\` с mock rclpy).
- [ ] Деплой на dev-стенде (опционально для этой карточки, основное —
      AV-9 с docker-сервисом).

## Связанные

- Блокируется: AV-2, AV-3, AV-4, AV-5
- Блокирует: AV-9
EOF
)"

# ---------------------------------------------------------------------------
# AV-7: voice_input_mode в dialogue_node._declare_params (закрыть расхождение #1)
# ---------------------------------------------------------------------------
create_issue \
  "[AV-7] dialogue_node: добавить параметр voice_input_mode (закрыть расхождение ADR-0027 #1)" \
  "agent:backend" \
  "type:functional" \
  "priority:high" \
  "no-e2e-required" \
  "$(cat <<'EOF'
## Контекст

ADR-0027 §3.4 описывает 4 режима \`voice_input_mode\`, но параметра ещё нет
в \`dialogue_node._declare_params()\`. Без него supervisor (ADR-0028) не может
переключать режимы — каждый клиент должен знать текущий сам.

Источники истины:
- \`docs/adr/0027-meta-quest-ar-control.md\` §3.4 (4 режима + default \`respeaker\`)
- \`docs/plans/2026-08-24-meta-quest-telepresence.md\` строки 487 (\`voice_input_mode\` в \`_declare_params()\`)
- \`src/rob_box_voice/.../dialogue_node.py:646\` — метод \`_declare_params\` (нет параметра, \`grep\` пуст)
- \`src/rob_box_voice/config/dialogue_node.yaml\` — обновить ключ
- \`docs/plans/2026-08-24-avatar-decomposition.md\` §4 (AV-7 acceptance)

## Acceptance criteria

- [ ] В \`src/rob_box_voice/rob_box_voice/dialogue_node.py\` метод \`_declare_params\`
      добавлен параметр:
      \`\`\`python
      self.declare_parameter("voice_input_mode", "respeaker")  # respeaker | quest_passthrough | quest_ttts | quest_stt | quest_llm_formalize
      \`\`\`
- [ ] \`src/rob_box_voice/config/dialogue_node.yaml\` — добавлен ключ
      \`voice_input_mode: respeaker\`.
- [ ] Stub-обработчик через \`add_on_set_parameters_callback\`: при изменении
      параметра пишется \`self.get_logger().info(...)"\"voice_input_mode changed to {mode}"\`
      (без реальной логики Phase 2 — она в отдельном worker-issue).
- [ ] Регрессионный unit-тест \`test/unit/test_voice_input_mode_param.py\`:
      читает YAML, проверяет наличие ключа, default = \"respeaker\".
- [ ] Существующие тесты voice-пайплайна остаются GREEN.
- [ ] \`pytest -v src/rob_box_voice/test/unit -q\` → GREEN.
- [ ] \`black\` + \`flake8\` чисто.

## Branch

\`feature/av-7-voice-input-mode-param\` (от \`feature/avatar\`).

## Commit

\`\`\`bash
git commit -m "feat(voice): declare voice_input_mode param in dialogue_node (ADR-0027 §3.4)"
\`\`\`

## Definition of Done

- [ ] PR в \`feature/avatar\`, raw-evidence (\`pytest -v\`).

## Связанные

- Независимая карточка (можно делать параллельно с AV-2..AV-6).
- Блокирует: AV-10 (Telegram refactor использует параметр через supervisor).
EOF
)"

# ---------------------------------------------------------------------------
# AV-8: расширить meta-quest-api.md frame-типами 0x30-0x33 (docs-only)
# ---------------------------------------------------------------------------
create_issue \
  "[AV-8] meta-quest-api.md: добавить frame-типы 0x30-0x33 для supervisor client API" \
  "agent:architect" \
  "type:functional" \
  "priority:high" \
  "no-e2e-required" \
  "$(cat <<'EOF'
## Контекст

Расширение wire-протокола Quest для клиентского API supervisor. 4 новых
frame-типа, JSON-CMD wrapper для HTTP/REST клиентов, новые error codes.
docs-only карточка — без кода.

Источники истины:
- \`docs/adr/0028-avatar-supervisor.md\` §4.4 (клиентский API, frame-типы)
- \`docs/architecture/meta-quest-api.md\` §3 (существующая таблица frame types)
- \`docs/plans/2026-08-24-avatar-decomposition.md\` §4 (AV-8 acceptance)

## Acceptance criteria

- [ ] В \`docs/architecture/meta-quest-api.md\` §3 (таблица frame types)
      добавлены 4 строки:
  - \`0x30 SET_MODE\` — client → supervisor, msgpack \`{client_id, mode}\`
  - \`0x31 ACQUIRE_FLOOR\` — client → supervisor, \`{client_id, floor: teleop|voice}\`
  - \`0x32 RELEASE_FLOOR\` — client → supervisor, аналогично
  - \`0x33 STATE_UPDATE\` — supervisor → client, msgpack \`{state: <packed AvatarState>}\`
- [ ] В §5 (\`JSON_CMD\`) добавлен wrapper для тех же 4 команд (HTTP/REST клиенты,
      backward compat через JSON).
- [ ] В §11 (versioning) добавлен bump \`subprotocol = robbox-quest-v2\`.
      Зафиксировано: server проверяет \`subprotocol\` и отвечает
      \`ERROR{PROTOCOL_VERSION}\` для v1 клиентов (либо поддерживает обе
      версии — решение фиксируется в комментарии, реализация — AV-10).
- [ ] В §8 (error codes) добавлены:
  - \`FLOOR_HELD\` — другой клиент держит floor
  - \`MODE_CONFLICT\` — FSM отклонила переход
- [ ] \`git diff --stat docs/architecture/meta-quest-api.md\` → ≥ 30 строк,
      ≤ 100 (raw-evidence в PR).

## Branch

\`feature/av-8-meta-quest-frame-types-supervisor\` (от \`feature/avatar\`).

## Commit

\`\`\`bash
git commit -m "docs(meta-quest-api): extend frame types 0x30-0x33 for supervisor client API"
\`\`\`

## Definition of Done

- [ ] PR в \`feature/avatar\`, raw-evidence (\`git diff --stat\` + \`gh run view\` lint).
- [ ] Mermaid-stateDiagram из ADR-0028 §4.1 скопирован в meta-quest-api.md
      (cross-reference).

## Связанные

- Независимая (можно делать параллельно с AV-2..AV-7).
- Блокирует: AV-10 (Telegram refactor ссылается на новые frame-типы).
EOF
)"

# ---------------------------------------------------------------------------
# AV-9: docker-сервис rob_box_supervisor (Phase 1 monitor)
# ---------------------------------------------------------------------------
create_issue \
  "[AV-9] docker: сервис rob_box_supervisor на Vision Pi (Phase 1 monitor)" \
  "agent:devops" \
  "type:infrastructure" \
  "priority:medium" \
  "needs-e2e" \
  "$(cat <<'EOF'
## Контекст

Деплой supervisor на Vision Pi в monitor-режиме. По образцу \`voice-assistant\`
сервиса из \`docker/vision/docker-compose.yaml\`. Без blast radius — только
публикует \`/avatar/state\`, не вмешивается.

Источники истины:
- \`docs/adr/0028-avatar-supervisor.md\` §4.6 (деплой)
- \`docs/development/DOCKER_STANDARDS.md\` (правила Docker)
- \`docker/vision/docker-compose.yaml:170\` (образец \`voice-assistant\`)
- \`docker/vision/voice_assistant/Dockerfile\` (образец app-layer Dockerfile)
- \`docker/vision/scripts/voice_assistant/\` (образец start-скрипта)
- \`docs/plans/2026-08-24-avatar-decomposition.md\` §4 (AV-9 acceptance)

## Acceptance criteria

- [ ] \`docker/vision/supervisor/Dockerfile\` создан по образцу
      \`docker/vision/voice_assistant/Dockerfile\`. ❌ НЕ \`COPY config/\`,
      ❌ НЕ \`COPY scripts/\` (DOCKER_STANDARDS.md).
- [ ] \`docker/vision/supervisor/start_supervisor.sh\` — запуск
      \`ros_with_namespace.sh rob_box_supervisor supervisor_node\`.
- [ ] \`docker/vision/docker-compose.yaml\` — сервис \`supervisor\` добавлен
      по образцу \`voice-assistant\`:
  - \`network_mode: host\`
  - \`ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5\`
  - \`RMW_IMPLEMENTATION=rmw_zenoh_cpp\`
  - \`volumes: ['./config:/config:ro', './scripts:/ros_scripts:ro']\`
  - \`depends_on: zenoh-router-vision (healthy)\`
- [ ] \`docker build\` без ошибок (raw-evidence в PR).
- [ ] \`docker compose up -d supervisor\` → \`docker logs supervisor\`
      показывает \`mode: monitor\`, \`/avatar/state\` публикуется
      (raw-evidence: последние 30 строк лога в PR-описании).

## Branch

\`feature/av-9-supervisor-docker-service\` (от \`feature/avatar\`).

## Commit

\`\`\`bash
git commit -m "infra(supervisor): docker service on Vision Pi (Phase 1 monitor)"
\`\`\`

## Definition of Done

- [ ] PR в \`feature/avatar\`, raw-evidence (\`docker build\` + \`docker logs\`).
- [ ] Сервис протестирован на dev-стенде (Vision Pi).

## Связанные

- Блокируется: AV-6 (supervisor_node в monitor-режиме).
- Блокирует: AV-11 (e2e mixed mode).
EOF
)"

# ---------------------------------------------------------------------------
# AV-10: рефакторинг rob_box_telegram на клиентский API супервизора
# ---------------------------------------------------------------------------
create_issue \
  "[AV-10] rob_box_telegram: рефакторинг на клиентский API супервизора" \
  "agent:backend" \
  "refactor" \
  "priority:medium" \
  "needs-e2e" \
  "$(cat <<'EOF'
## Контекст

Telegram-бот сейчас сам публикует \`cmd_vel_web\` (priority 50 в twist_mux) и
дёргает TTS-канал, минуя общий state. Это race condition с Quest (ADR-0028 §1.2).
После рефакторинга Telegram становится клиентом supervisor — шлёт AcquireFloor,
получает STATE_UPDATE, гасит кнопки если floor не его.

Источники истины:
- \`docs/adr/0028-avatar-supervisor.md\` §1.2 (что болит — race conditions)
- \`docs/adr/0028-avatar-supervisor.md\` §4.4 (client API)
- \`docs/adr/0028-avatar-supervisor.md\` §6 Q3 (UX Telegram когда avatar_present)
- \`src/rob_box_telegram/.../telegram_node.py:65-66\` — текущие публикации
- \`docs/plans/2026-08-24-avatar-decomposition.md\` §4 (AV-10 acceptance)

## Acceptance criteria

- [ ] \`src/rob_box_telegram/.../telegram_node.py\` — публикация
      \`cmd_vel_web\` ЗАМЕНЯЕТСЯ на service-call \`AcquireFloor{client_id=\"telegram\",
      floor=\"teleop\"}\` перед каждой командой движения. Если
      \`success=false\` (другой клиент держит floor) — UI Telegram гасит
      кнопки движения (UX по ADR-0028 §6 Q3).
- [ ] Публикация TTS оборачивается в \`AcquireFloor{client_id=\"telegram\",
      floor=\"voice\"}\` → TTS-канал → \`ReleaseFloor\`.
- [ ] Heartbeat \`teleop_heartbeat\` (10 Гц) шлётся, пока держится \`teleop_floor\`.
- [ ] Подписка на \`/avatar/state\` (latched) — кнопки движения в Telegram
      дизейблятся, если \`state.teleop_floor != \"telegram\"\`.
- [ ] Существующие тесты Telegram-команд остаются GREEN (адаптация под mock
      service-call).
- [ ] Новый тест: \`test/unit/test_telegram_supervisor_client.py\` — verify
      acquire → publish → release sequence (mock rclpy services).
- [ ] \`pytest -v\` GREEN, \`black\` + \`flake8\` чисто.

## Branch

\`feature/av-10-telegram-supervisor-client\` (от \`feature/avatar\`).

## Commit

\`\`\`bash
git commit -m "refactor(telegram): route move/tts commands through supervisor client API"
\`\`\`

## Definition of Done

- [ ] PR в \`feature/avatar\`, raw-evidence (\`pytest -v\`).
- [ ] Деплой и smoke-test на dev-стенде.

## Связанные

- Блокируется: AV-7 (voice_input_mode нужен), AV-8 (frame-типы),
  AV-6 (supervisor принимает сервисы).
- Блокирует: AV-11 (e2e mixed).
EOF
)"

# ---------------------------------------------------------------------------
# AV-11: e2e mixed-mode (Quest teleop + Telegram voice одновременно)
# ---------------------------------------------------------------------------
create_issue \
  "[AV-11] e2e: Avatar mixed-mode (Quest teleop + Telegram voice одновременно)" \
  "agent:e2e-runner" \
  "type:testing" \
  "priority:high" \
  "needs-e2e" \
  "$(cat <<'EOF'
## Контекст

Сквозной e2e mixed-режима: Quest teleop + Telegram voice одновременно.
Quest Wi-Fi fail → Telegram подхватывает teleop (graceful handover).
Это acceptance-критерий всей Avatar-фичи.

Источники истины:
- \`docs/adr/0028-avatar-supervisor.md\` §1.3 (северная звезда)
- \`docs/adr/0028-avatar-supervisor.md\` §6 Q4 (dead-man handover)
- \`docs/development/E2E_GUIDE.md\` (общие правила e2e)
- \`docs/plans/2026-08-24-avatar-decomposition.md\` §4 (AV-11 acceptance)

## Acceptance criteria

Сценарий (Vision Pi + Main Pi + dev-машина):
- [ ] 1. Поднять \`rob_box_quest\` + \`rob_box_supervisor\` (\`mode:=active\`)
        + \`rob_box_telegram\`.
- [ ] 2. Открыть WebXR-клиент в браузере dev-машины, авторизоваться PIN.
- [ ] 3. Зажать grip на контроллере, дать twist \`linear=0.3\` → робот едет.
- [ ] 4. Из Telegram одновременно послать \`/say привет\` → робот произносит
        через TTS голосом робота (НЕ из ReSpeaker).
- [ ] 5. Скриншот/лог: \`docker logs supervisor\` показывает
        \`mode=mixed, teleop_floor=quest, voice_floor=telegram\`.
- [ ] 6. Разорвать Wi-Fi на dev-машине (или закрыть вкладку) → через ≤ 500 мс
        робот safe-stop, Telegram получает STATE_UPDATE
        \`{floors.teleop: none}\`, кнопки движения в Telegram разблокированы.
- [ ] 7. Из Telegram послать \`/forward\` → робот едет, \`/avatar/state\` →
        \`mode=telegram_active, teleop_floor=telegram\`.
- [ ] Raw-evidence в PR:
  - \`pytest -v\` (все unit-тесты supervisor/telegram/quest GREEN).
  - \`docker logs\` (последние 50 строк supervisor/quest/telegram).
  - Скриншоты mixed-state и post-failover.
- [ ] \`agent-flow-merge-gate\` проверяет raw-evidence блок.

## Branch

\`feature/av-11-avatar-mixed-e2e\` (от \`feature/avatar\`).

## Commit

Без нового кода — результаты в issue-комментарий / PR-описание (raw-evidence).

## Definition of Done

- [ ] PR в \`feature/avatar\`, raw-evidence (логи + скриншоты).
- [ ] Все 7 шагов сценария пройдены, e2e отчёт в PR-описании.

## Связанные

- Блокируется: AV-9 (docker), AV-10 (Telegram refactor).
- Финальная карточка эпика.
EOF
)"

echo ""
echo "✅ All 10 issue-черновиков processed (AV-2 ... AV-11)."
echo "📄 План: docs/plans/2026-08-24-avatar-decomposition.md"
echo "🔗 Branch: feature/av-2-supervisor-package-skeleton etc. (создаются воркерами)"
