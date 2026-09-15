# DevOps On-Call Channel — Setup Runbook

> **Назначение**: ручной + полуавтоматический setup отдельного Telegram-канала
> `#devops-oncall`, через который будут идти stale-candidate / priority:high
> алерты (issue #2394, PM-default от 2026-09-15).
>
> **Контекст**: PM-sync завершился kanban `t_870273be` (PR #2577 merged в
> develop, `docs/p0-voice-bugs-decisions.md` §7). Следующий шаг —
> **owner-setup** нового канала (kanban `t_d215c9e0`, эта карточка).
> Реальный alerter-скрипт — отдельная карточка `t_6927a4c5` follow-up.

## TL;DR

```bash
# 1. В Telegram-клиенте (вручную, владелец):
#    - Создать канал: New Channel → name = "DevOps On-Call"
#      (username опционально; description — опционально).
#    - Channel Settings → Administrators → Add Administrator →
#      найти бота @<имя_вашего_hermes_бота> (тот, чей токен в .env
#      архитектора) → Promote to Admin с правом «Post Messages».
#    - Зайти в канал и отправить ОДНО сообщение (например "ping"),
#      чтобы бот увидел чат в getUpdates.

# 2. На хосте (полу-автоматически):
bash <repo>/scripts/agent_flow/setup-devops-oncall.sh \
    --chat-name "DevOps On-Call" \
    --env-file /home/builder/.env.249 \
    --profile architect \
    --dry-run                            # сначала сухой прогон
bash <repo>/scripts/agent_flow/setup-devops-oncall.sh \
    --chat-name "DevOps On-Call" \
    --env-file /home/builder/.env.249 \
    --profile architect \
    --apply                             # реальная запись
# В /home/builder/.env.249 появятся строки:
#   DEVOPS_ALERT_CHAT_ID=-100xxxxxxxxxx
#   DEVOPS_ALERT_CHAT_NAME=DevOps On-Call

# 3. Проверить (вручную, в Telegram):
#    - Бот @<имя_бота> уже должен быть в канале как admin.
#    - Тест-нотификация появится автоматически после notify-subscribe
#      на kanban-таску t_d215c9e0.
```

---

## Зачем отдельный канал (а не существующий PM-handoff)

| | PM-handoff (`chat_id=495039871`) | Новый `#devops-oncall` |
|---|---|---|
| Назначение | хэндоффы по задачам (PM рутина) | on-call алерты по инцидентам |
| Получатели | PM (krikz) | on-call devops + Шифу (owner) |
| Шум | высокий (все complete'ы) | низкий (только алерты) |
| Latency | не критична | <60 мин SLA (priority:high) |

PM-default (PR #2577): алерты stale-candidate НЕ ДОЛЖНЫ перегружать
PM-handoff канал — отдельный поток.

## Что нужно сделать владельцу

### Шаг 1: Создать канал (в Telegram-клиенте)

1. Открыть Telegram → меню → **New Channel**.
2. Name: `DevOps On-Call` (можно `#devops-oncall` или иное — но **имя
   должно совпадать** с тем, что передаётся в `--chat-name` скрипта).
3. Description (опционально): «On-call alerts for krikz/rob_box_project
   stale-candidate + priority:high|critical. See
   docs/runbooks/devops-oncall-channel.md».
4. Private vs Public: **Private** (только invited участники). Это снижает
   риск случайных подписок + спам-ботов.
5. Invite участников:
   - **Бот Hermes** (тот, чей `TELEGRAM_BOT_TOKEN` в `architect/.env`)
     — обязательно; promote to Admin.
   - Владелец репо (krikz) — для visibility.
   - Шифу (GOODWORKRINKZ) — по желанию; или PM может тегать в issues.

### Шаг 2: Promote бота до Admin (в Telegram-клиенте)

1. Открыть канал → Channel Info → Administrators → Add Administrator.
2. Найти бота по @username (см. BotFather / `architect/.env`).
3. Promote; права: **Post Messages** обязательно. Остальные — по вкусу
   (но «Ban Users» и «Add Users» боту не нужны → off).
4. Сохранить.

### Шаг 3: «Разбудить» бота (в Telegram-клиенте)

Telegram Bot API имеет ограничение: `getUpdates` возвращает только чаты,
в которых бот видит сообщения после того, как его туда добавили.

1. Зайти в созданный канал.
2. Отправить **одно** сообщение (например `ping` или `/start@<bot_username>`).
3. Подождать 5-10 секунд (чтобы Telegram API зарегистрировал).

После этого `getUpdates` вернёт этот чат с правильным `chat.id`.

### Шаг 4: Запустить helper-скрипт (на хосте 249 или другой)

```bash
ssh 10.1.1.249  # или где живёт cron

cd /home/builder/rob_box_project
bash scripts/agent_flow/setup-devops-oncall.sh \
    --chat-name "DevOps On-Call" \
    --env-file /home/builder/.env.249 \
    --profile architect \
    --dry-run
# Проверяем, что:
#   1. Токен найден в architect/.env.
#   2. chat_id найден через getUpdates.
#   3. env-файл будет обновлён (chmod 600, DEVOPS_ALERT_CHAT_ID=...).

bash scripts/agent_flow/setup-devops-oncall.sh \
    --chat-name "DevOps On-Call" \
    --env-file /home/builder/.env.249 \
    --profile architect \
    --apply
```

### Шаг 5: Обновить issue #2394

```bash
gh issue comment 2394 --repo krikz/rob_box_project --body "$(cat <<'EOF'
✅ Owner-setup завершён.

- Канал: `#devops-oncall` (Private)
- chat_id: -100xxxxxxxxxx (зафиксирован в /home/builder/.env.249 как DEVOPS_ALERT_CHAT_ID)
- Бот: @<bot_username> (admin в канале)
- Helper-скрипт: scripts/agent_flow/setup-devops-oncall.sh
- Runbook: docs/runbooks/devops-oncall-channel.md

Следующий шаг — alerter-скрипт (kanban t_6927a4c5 follow-up) с целевым CHAT_ID.
EOF
)"
```

### Шаг 6: Закрыть kanban-карточку

После всех шагов → `kanban complete t_d215c9e0` с summary:
«Канал создан, ID зафиксирован в .env.249, runbook + helper-скрипт в PR #XXXX».

---

## Что НЕ нужно делать

- **НЕ дублировать TELEGRAM_BOT_TOKEN** в `devops/.env` или `pm/.env`.
  Это сломает single-owner архитектуру (ретро 12.08 `t_5af222ea`):
  два getUpdates-консьюмера → один из них постоянно в retry-loop.
  Единственный профиль с активным токеном — `architect` (или тот,
  который PM/architect явно назначил).
- **НЕ коммитить `.env.249`** в репо. Это **секрет** уровня «low»
  (позволяет писать в конкретный канал, но не даёт доступ к бот-токену),
  но всё равно хранение — vault / host-only / `git-crypt` или подобное.
  В репо — **только** `setup-devops-oncall.sh` (без значений) и этот runbook.
- **НЕ создавать канал как Public.** Public-канал может быть найден
  через глобальный search Telegram → спам-боты начнут постить; бот не
  умеет защищаться от этого. Private — только по invite.
- **НЕ использовать существующий PM-handoff (`495039871`)** для алертов.
  Это PM-рутина, не on-call. PM-default от 2026-09-15 явно это запрещает.

## Что может пойти не так

| Симптом | Причина | Что делать |
|---|---|---|
| `chat_id для '#devops-oncall' не найден в getUpdates` | Бот не добавлен в канал, или не было сообщений после добавления | Проверить Шаг 2+3; повторить через 5-10 сек |
| `TELEGRAM_BOT_TOKEN не найден ни в одном .env` | Все профили в disable-режиме (после ретро t_5af222ea) | Проверить `architect/.env` — там должен быть активный токен; см. ADR-0022 § 6 |
| `notify-subscribe завершился с ошибкой` | task уже complete, или профиль devops не имеет токена | Это нормально — нотификация ушла через architect gateway (он же держит токен) |
| Сообщения в канале видны PM-handoff | Не тот chat_id прописан | Сверить `DEVOPS_ALERT_CHAT_ID` в `.env.249` с фактическим `chat.id` через `curl ... getChat` |

## Связанные артефакты

- Issue: https://github.com/krikz/rob_box_project/issues/2394
- PM-sync kanban: `t_870273be` (завершён)
- PM-sync docs PR: #2577 (`docs/p0-voice-bugs-decisions.md` § 4 q3, § 7)
- ADR-0022 § 6 (single-owner telegram): `docs/adr/0022-process-e2e-done-gates.md`
- Ретро single-owner: `t_5af222ea` (12.08.2026) — закомментированы токены в devops/pm/.env
- Alerter follow-up: kanban `t_6927a4c5` → follow-up `t_6927a4c5`
  (зависит от `DEVOPS_ALERT_CHAT_ID`)
- Helper-скрипт: `scripts/agent_flow/setup-devops-oncall.sh`
- Существующая PM-handoff инфра: `scripts/agent_flow/agent-flow-handoff.sh:92`
