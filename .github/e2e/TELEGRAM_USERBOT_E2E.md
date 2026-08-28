# Telegram e2e через тестовый userbot (L3, issue #1196)

Полный e2e-тест Telegram-канала rob_box-бота: **чат → бот → LLM → ответ в чат**.
В отличие от голосового e2e (`e2e_voice_test.sh`, инъекция в `/voice/stt/result`),
здесь сообщение отправляется через реальный Telegram Bot API **от лица
тестового user-аккаунта** (MTProto, Telethon). Это единственный способ
автоматически проверить полный цикл, потому что (проверено 13.08):

- бот не может сам себе отправлять сообщения — Telegram API возвращает
  `User_bot_to_bot_disabled`;
- боты в группах не видят сообщения от других ботов — нужен user-аккаунт;
- спамить автотестами общий чат Shizza неудобно — нужна отдельная тестовая
  группа.

## Предусловия (нужны от товарища Шифу)

1. **Тестовый user-аккаунт** (НЕ личный аккаунт Шифу) — номер + api_id/api_hash
   с [my.telegram.org](https://my.telegram.org), + session (первый вход
   подтверждается кодом из Telegram).
2. **Тестовая группа**, где состоят rob_box-бот и тестовый user. chat_id
   группы (обычно отрицательный, для супергрупп с префиксом `-100...`)
   добавляется в `TELEGRAM_ALLOWED_USERS` бота.
3. Скрипт запускается на билд-машине 249 (или в CI runner'е) с доступом в
   интернет к Telegram MTProto.

## Секреты (env, НЕ в репо)

| Переменная | Описание |
|---|---|
| `TG_TEST_API_ID` | api_id с my.telegram.org |
| `TG_TEST_API_HASH` | api_hash с my.telegram.org |
| `TG_TEST_SESSION` | строка сессии Telethon или путь к `.session` файлу |
| `TG_TEST_CHAT_ID` | chat_id тестовой группы (например `-1001234567890`) |
| `TG_BOT_USERNAME` | username бота (например `rob_box_test_bot`) |
| `TG_TEST_WAIT_ANSWER` | сек ожидания ответа (default 60) |
| `TG_TEST_VERBOSE` | `1` = печатать сырой диалог |

Сессию после первого запуска сохраняйте в защищённое место (GitHub Secrets /
.env на 249), не коммитьте.

## Сценарии

```bash
# 1. Текст с wake word → ответ бота в чат
TG_TEST_API_ID=... TG_TEST_API_HASH=... TG_TEST_SESSION=... TG_TEST_CHAT_ID=... \
python3 .github/e2e/telegram_userbot_e2e.py --scenario wake --text "Робот, спой песенку"

# 2. Текст БЕЗ wake word → LLM отвечает (после фикса потоков #1195)
python3 .github/e2e/telegram_userbot_e2e.py --scenario no-wake --text "продолжай"

# 3. Голосовое сообщение в чат → распознавание → ответ
python3 .github/e2e/telegram_userbot_e2e.py --scenario voice --file /tmp/cmd.ogg
```

Exit codes: `0` = PASS, `1` = FAIL, `2` = SKIP (нет секретов/telethon),
`3` = нет ответа за таймаут.

## Контракт для e2e-process

Когда тестовый аккаунт готов, e2e-process может гонять telegram-сценарии по
RUN_NOW. Контракт в issue (блок `## e2e`):

```
## e2e
scenario: telegram-userbot
telegram_text: "Робот, спой песенку про енотика"   # или voice_text для голосового e2e
telegram_scenario: wake                             # wake | no-wake | voice
telegram_wait: 90
llm: deepseek
tts: minimax-male-qn-qingse
stt: yandex
```

До готовности аккаунта работают L1 (юниты в CI) + L2 (эхо-проверка в
`e2e_voice_test.sh --check-tg-echo`) + ручной smoke.
