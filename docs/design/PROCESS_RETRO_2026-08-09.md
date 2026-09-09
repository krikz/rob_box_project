# Retrospective: agent-flow аномалии 09.08 — решения и приоритеты

| Поле | Значение |
|------|----------|
| Документ | `docs/design/PROCESS_RETRO_2026-08-09.md` |
| Статус | Принято (architect), фиксы скриптов применены 09.08 |
| Дата | 2026-08-09 |
| Автор | architect (Hermes) |
| Связанное | `docs/design/AGENT_FLOW_PROPOSAL.md`, канбан `t_c55ed042`, скрипты `~/.hermes/scripts/agent-flow-*.sh` |

---

## 0. TL;DR

Прогон 09.08 вскрыл 15 аномалий процесса. Часть уже была починена до ретро
(4, 5, 11, 13, 3), часть — починена в рамках этой карточки (1, 8, 9, 14),
остальное — карточки-фиксы для backend/devops с приоритетами.

**Приоритеты (итог):**

| # | Аномалия | Приоритет | Решение | Статус |
|---|----------|-----------|---------|--------|
| 12 | sqlite «cannot start a transaction within a transaction» — память робота не пишется | **P0** | Карточка backend: SQLiteVoiceMemory, один connection + to_thread | 🔴 карточка |
| 2 | Бюджет 90/90 — воркер не коммитил промежуточно, работа висела в worktree | **P0** | WIP-коммиты в контракте воркера + triage max_runtime | 🟡 карточка |
| 7 | Воркер не верифицирует фичу после e2e (#1077 SUCCESS, а профиль не создаётся) | **P0** | e2e-доклад должен включать acceptance-проверку, а не только workflow success | 🟡 карточка |
| 1 | worktree add failed — done/archived карточки держат ветку → вечный block | P1 | free_stale_worktrees в merge-gate при unblock + разовая чистка | ✅ починено |
| 3 | rclpy Humble: declare_parameter('mix_channels', []) → BYTE_ARRAY, краш audio_node | P1 | live-фикс + коммит 049164e8 (непустой дефолт) | ✅ починено |
| 8 | e2e-контракт не валидируется: voice_file не существует → scp fail через 20 мин | P1 | fail-fast пред-проверка voice_file в e2e-process до build | ✅ починено |
| 9 | e2e-контракт: воркер писал ## e2e в PR body, а читается issue body; volume 150 vs 125 | P1 | fallback на PR body + документирование контракта | ✅ починено |
| 4 | Ложный deploy FAILED (gh run view conclusion пустой → FAILED при success) | P2 | retry ×3 чтения conclusion | ✅ починено |
| 5 | e2e-process молча выходил (set -euo pipefail + grep exit 1) | P2 | sed-конвертация \n | ✅ починено |
| 6 | Карточка не просыпалась после e2e (блок без причины) | P2 | ручной unblock юзером; фикс — см. #7 acceptance + #10 | 🟡 |
| 10 | respawn_guarded recent_success: ручной done→ready через SQL не создаёт событие | P2 | штатный путь через kanban unblock; карточка devops | 🟡 карточка |
| 11 | MiniMax 429 — воркеры падали, fallback deepseek не был настроен | P2 | fallback_providers в профиле architect | ✅ починено |
| 13 | Handoff-рекурсия: тестовый корень плодил «Handoff: Phase 2» детей | P2 | архивация корня + защита handoff.sh | ✅ починено |
| 14 | Дубликаты карточек от triage (#1076 → t_89eaf282 + t_1ecbd260) | P2 | идемпотентность по issue-number в triage | ✅ починено |
| 15 | Первый захват речи теряет начало фразы («роберт» вместо «робот») | P2 | карточка backend (STT/audio) | 🔴 карточка |

---

## 1. Что уже было починено до ретро (проверено по коду/логам)

- **#3 rclpy**: коммит `049164e8` «fix(voice 1076): непустой дефолт mix_channels»
  уже в ветке `z-{agent}/1076-...` и в `z-{e2e}/test-round-5`.
- **#4 ложный FAILED**: в `e2e-process.sh` уже был polling-цикл, но conclusion
  читался однократно — гонка `completed` → пустой conclusion. Усилено retry ×3.
- **#5 молчаливый выход**: `body_real` уже конвертировался `sed 's/\\n/\n/g'`,
  что и было фиксом для `grep exit 1` под `set -euo pipefail`.
- **#11 MiniMax 429**: профиль architect уже имеет `fallback_providers`
  (deepseek) — событие unblock «architect fallback deepseek added».
- **#13 handoff-рекурсия**: корень `t_2340e062` заархивирован, `handoff.sh`
  скипает `^Handoff:` и `handoff-parent:`.

## 2. Что починено в рамках этой карточки (09.08, скрипты)

### 2.1. #1 worktree add failed — free_stale_worktrees перед unblock (merge-gate)

**Факт:** старые done/archived карточки держат ту же ветку
(`z-{agent}/1050-...`), респавн падает «git worktree add failed», карточка
навсегда виснет в blocked (анблок → респавн → снова fail).

**Фикс в `agent-flow-merge-gate.sh`** (CI-red path, Q21): перед `kanban unblock`
вызывается `free_stale_worktrees_for "$task_id"` — освобождает worktree'ы на той
же ветке от старых карточек. Ранее функция вызывалась только в MERGED-cleanup.

**Разовая чистка:** освобождены worktree'ы archived-карточек
`t_c72bf606`, `t_19c356b6`, `t_dff30081` (ветки не в open PR).
`t_9435a3c5` — worktree-регистрация снята (prune), физическая папка осталась
(root-owned файлы из Docker-воркера) — на процесс не влияет, ветка свободна.

### 2.2. #8 e2e-контракт — fail-fast пред-проверка voice_file

**Факт:** `voice_file=spoy_new.ogg` не существовал → `ensure_voice_file.sh`
падал через 20 мин сборки (scp fail), время потрачено впустую.

**Фикс в `agent-flow-e2e-process.sh`:** после merge в round-ветку, ДО запуска
BUILD-цепочки, проверяется `git ls-files --error-unmatch "$e2e_voice_file"`:
- файл есть → ок (ensure_voice_file скопирует);
- файла нет и `voice_text` пуст → fail-fast: коммент в issue + skip (без 20-мин
  build/deploy впустую);
- файла нет, но `voice_text` задан → лог, ensure_voice_file сгенерит на лету.

### 2.3. #9 e2e-контракт — fallback на PR body

**Факт:** воркер писал блок `## e2e` в PR body, а e2e-process читает body issue
(#1077) → параметры терялись, шли дефолты.

**Фикс в `agent-flow-e2e-process.sh`:** если в issue body блока `## e2e` нет,
читается body PR (по head-ветке) и параметры извлекаются оттуда же. Приоритет:
issue body > PR body > env/дефолты.

### 2.4. #4 ложный FAILED — retry чтения conclusion

**Фикс в `agent-flow-e2e-process.sh`:** в `wait_workflow` и в финальном ожидании
verdict conclusion перечитывается до 3 раз с паузой 5s (гонка: `status=completed`
но conclusion ещё пустой → success ошибочно считался FAILURE).

### 2.5. #14 дубликаты triage — идемпотентность по issue-number

**Факт:** issue #1076 получил ДВЕ карточки от triage: `t_89eaf282` (ready) и
`t_1ecbd260` (archived). Маркер `kanban: t_` в комментариях мог потеряться при
переключении меток (e2e:rejected → needs-e2e) → дубль.

**Фикс в `agent-flow-triage.sh`:** перед созданием карточки проверяется
`kanban list` на существование НЕ-archived карточки с `issue: #<N>` в body.
Если есть — skip. (Дополнительно к маркеру в комментариях.)

---

## 3. Решения и карточки-фиксы

### P0

#### #12 sqlite «cannot start a transaction within a transaction» (backend)
**Факт:** `SQLiteVoiceMemory.append_turn` падает — память робота НЕ пишется
(turns теряются). Конкурентный доступ к одному соединению через `to_thread`.
**Решение:** карточка backend: переиспользование соединения thread-safe
(connection per thread / lock / WAL), ретрай на `OperationalError: cannot start
a transaction within a transaction`.
**Trade-off:** минимальный фикс — обернуть запись в `threading.Lock`; правильный
— отдельное соединение на поток. Сложность низкая, benefit высокий (память —
ядро фичи спикеров #1077).

#### #2 бюджет 90/90 — воркер не коммитит промежуточно (процесс)
**Факт:** t_9435a3c5 — вся работа (18 файлов, 496 flake8) висела в worktree,
воркер не коммитил WIP; при исчерпании бюджета работа пропала бы (rescue вручную).
**Решение:** карточка (agent-flow/контракт): воркер обязан коммитить WIP каждые
~15-20 мин (или при достижении половины max_runtime), triage может ставить
`--max-runtime` выше для крупных задач. В SOUL/карточке — явное правило
«коммить промежуточно, не копить всё в worktree».
**Trade-off:** WIP-коммиты чуть засоряют историю, но дают страховку от потери
работы при краше/бюджете.

#### #7 воркер не верифицирует фичу после e2e (процесс + backend)
**Факт:** #1077 e2e SUCCESS, а профиль спикера не создаётся (vosk fallback →
tag=None) — никто не проверил acceptance. e2e-доклад сообщает только
verdict workflow, не проверяет поведение фичи.
**Решение (две части):**
1. **Процесс (e2e-process/контракт):** e2e-доклад должен включать acceptance-
   проверку (по артефактам: ASR-транскрипт, логи, результат фичи), а не только
   «workflow success». Если acceptance не проверяем — вердикт `UNVERIFIED`, не
   `SUCCESS`.
2. **Backend:** карточка на доработку #1077: проверить, что профиль спикера
   реально создаётся при yandex-speaker_tag (не только при vosk-пути).
**Trade-off:** acceptance-проверка в e2e-process требует парсинга артефактов —
дороже, но без неё e2e SUCCESS ничего не гарантирует.

### P1

#### #1 worktree add failed (доделка) — devops
Остаток: диспетчер при спавне сам не чистит stale worktree'ы; сейчас чистит
merge-gate (при unblock) и e2e-process (перед unblock). Предложение devops:
добавить `free_stale_worktrees_for` в точку создания worktree в диспетчере
(или в triage перед create).

#### #8/#9 e2e-контракт — документирование (techwriter/PM)
Фиксы в скриптах применены; нужно зафиксировать контракт `## e2e` в
AGENT_FLOW_PROPOSAL.md: блок пишется в **body ISSUE** (не PR), voice_file —
существующий путь в репо, volume в разумных пределах (125-150).

### P2

#### #10 respawn_guarded recent_success (devops)
Ручной возврат done→ready через SQL не создаёт событие → диспетчер не берёт.
Решение: штатный путь `kanban unblock` (создаёт событие) или прямой вызов
respawn-функции. Карточка devops.

#### #15 первый захват речи теряет начало (backend)
«роберт» вместо «робот» — wake word обрезается. Карточка backend (STT/audio):
проверить VAD-детекцию начала фразы / буферизацию первого чанка.

---

## 4. Метрики эффективности (что меряем после фиксов)

| Метрика | До (09.08) | Цель |
|---------|-----------|------|
| Карточек, застрявших в blocked из-за worktree | ≥2 (1050, 1077) | 0 |
| Бюджет-потерь (работа в worktree без коммита) | 2 (t_9435a3c5, t_0c0a98ac) | 0 |
| e2e SUCCESS без верификации acceptance | 1 (#1077) | 0 |
| Ложных FAILED (пустой conclusion) | 1 | 0 |
| Дублей карточек от triage | 1 (#1076) | 0 |

---

## 5. Файлы, изменённые в рамках ретро

- `~/.hermes/scripts/agent-flow-merge-gate.sh` (+ синхронизированы копии в
  `profiles/agent-flow/`, `profiles/architect/`) — free_stale_worktrees перед unblock.
- `~/.hermes/scripts/agent-flow-e2e-process.sh` (+ копии) — fail-fast voice_file,
  fallback на PR body, retry conclusion ×3.
- `~/.hermes/scripts/agent-flow-triage.sh` (+ копия в `profiles/agent-flow/`) —
  идемпотентность по issue-number.
- Worktree'ы archived-карточек освобождены (разовая чистка).
