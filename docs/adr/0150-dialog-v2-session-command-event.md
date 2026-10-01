# ADR-0150: Новый виток диалоговой системы (dialog v2) — сессия с одним владельцем, команда вместо свободного ответа, фраза об исполнении из события

| Поле | Значение |
|---|---|
| Статус | Proposed (на утверждение товарищу Шифу). Только дизайн и план поставки, код не меняется |
| Дата | 2026-10-01 |
| Автор | Claude Code (шисюн), по брифу товарища Шифу `docs/research/dialog-v2/00-brief.md` |
| Issue | эпик — завести при принятии (аналог #3312 для музыки); входные карточки: #3266, #3271, #3297, #3145, #3144, #3108, #3109, #3000, #2754, #3269, #3296 (все open на 01.10) |
| Основание | `docs/research/dialog-v2/01…06` (в git); аудиты CI run 36918223549 (статика, 01.10) и 36689419795 (рантайм, 30.09); код сверен на этом worktree, HEAD `61fd84ff7` + коммит материалов `9d3d3fcbe` |
| Родители | ADR-0148 (код решает, LLM говорит — обязателен), ADR-0149 (музыка v2: владелец плеера, события `started/rejected`, флаг `music_engine`), ADR-0141, ADR-0131 (`utterance_id`), ADR-0102/0103 («Повод»), ADR-0083 (`build_agent`), ADR-0051 (ТАРС), ADR-0145 (бюджет классов), ADR-AF-0013 (мелкие PR), ADR-0018 (честный FAIL) |
| Заменяет после приёмки | ADR-0084 целиком; ADR-0021 целиком (R1–R5); ADR-0066 §2–§3 (pause/resume без владельца → типизированный `DialogControl` с полем `set_by` и TTL); ADR-0143 (ретраи Bug B/C как норма → режим команд без `tool_choice`); ADR-0129-dj §2 (штамп персоны в промпт → кадр сцены); ADR-0140 остаток; ADR-0065 §2 (списки вейк-слов в `dialogue_text.py` → `rob_box_dialog.knowledge`). Подробно — §13 |

---

## Оглавление

0. Что проверено, что нет; 0.1 расхождения материалов с кодом
1. Проблема: классы отказов и семь корней
2. Решение: границы, модули, владельцы
3. Поток одной реплики (как будет)
4. Что решает код, что — LLM
5. Исполнение, событие, фраза
6. Речь, перебивание, тишина
7. Сессия, сцены, персоны, история, память
8. Провайдеры: возможности и честная деградация
9. Интерфейс с музыкой (ADR-0149), Telegram, ТАРС
10. Типы сообщений: уход от JSON в `std_msgs/String`
11. Миграция: strangler за флагом `dialog_engine`, план PR
12. Таблица костылей на удаление
13. Какие ADR замещаются; что поправить в CONTEXT.md
14. Приёмка числами (A1–A16)
15. Что сохранить из старого
16. Риски и откат
17. Альтернативы
18. Вопросы к товарищу Шифу
19. Что не проверено

---

## 0. Что проверено, что нет

- Прочитаны целиком: `00-brief.md`, `01-bug-archaeology.md` (488 строк), `02-current-architecture.md` (398), `03-kludge-inventory.md` (286), `04-adr-landscape.md` (223), `05-oss-landscape.md` (402), `06-architecture-audit-digest.md` (185), ADR-0148, ADR-0149, `AGENTS.md`.
- **Сверено мной по коду на HEAD `61fd84ff7`** (grep/Read, 24 утверждения; ниже каждое помечено «(проверено)»):

| # | Утверждение | Результат на HEAD |
|---|---|---|
| V1 | `dialogue_node.py` 9 954 строки; `dialogue_guards.py` 3 063; `music_guard.py` 876; `tts_node.py` 7 316; `agent_core.py` 2 690 | `wc -l` — совпало |
| V2 | `re.compile` в `dialogue_guards.py` = 34, `skill_router.py` = 27, `speak_helpers.py` = 15 | совпало; в `music_guard.py` — **0** (см. §0.1) |
| V3 | Меток `Bug A–F`: 113 в `dialogue_node.py`, 57 в `dialogue_guards.py`; ссылок `#NNNN` в `dialogue_node.py` 705 (144 уникальных) | совпало |
| V4 | `DEFAULT_SYNTHETIC_RETRIES = 2` (`dialogue_node.py:7861`) против `DEFAULT_MAX_SYNTHETIC_RETRIES = 3` (`core/turn.py:36`) | совпало |
| V5 | `_MAX_TOOL_ITERATIONS = 8` (`agent_core.py:87`), цикл `:1598` | совпало |
| V6 | `skill_tool_narrowing: false` в обоих yaml (`docker/vision/config/voice_assistant/dialogue_node.yaml:112`, `src/rob_box_voice/config/dialogue_node.yaml:43`) | совпало |
| V7 | `music_engine: "v1"` (`mcp_server.yaml:12`), выбор движка `mcp_server.py:299-327` | совпало |
| V8 | «Принял.» на пустой ответ — `dialogue_node.py:8508` | совпало |
| V9 | `/voice/current_dialogue_id`: подписчики `tts_node.py:1308`, `tools/dialogue.py:109`; писателя нет, `dialogue_node.py:9456` только присваивает атрибут | совпало |
| V10 | `DEFAULT_WAKE_WORDS` — 26 вариантов, включая «бот», «роб», «рома», «робота» (`dialogue_text.py:31-75`); `is_silence_command` — подстрока `any(cmd in text_lower …)` (`:215-218`); `stt_node.py:226-240` — запасная копия с подстрочным матчем и «пустой список = пропускать всё» | совпало |
| V11 | Писатели `/voice/tts/request`: `tools/dialogue.py:99`, `tools/say.py:61`, `telegram_node.py:206`, `stt_node.py:605`, супервизор через `GRIP_TTS_REQUEST_TOPIC` (`supervisor_node.py:397`); рантайм добавляет `joystick_control_node` | 5–6 писателей, совпало |
| V12 | Писатели `/voice/tts/control`: `audio_node.py:152`, `stt_node.py:587`, `dialogue_node.py:848`, `quest_node.py:1752` | 4, совпало |
| V13 | `/voice/dj_mode` пишут `dialogue_node.py:1051,5274` и `tools/music.py:7055`; два владельца DJ-флага: `DJState` (`dj_mode.py:157`) и `MusicManager._dj_mode_enabled` (`music.py:714`) | совпало |
| V14 | `/voice/stt/result` пишут `stt_node.py:578` и `telegram_node.py:168`; telegram также пишет `/voice/dialogue/response`, `/avatar/command`, `/voice/tts/request`, `/voice/sound/stop` (`telegram_node.py:168-223`) | совпало |
| V15 | `check_silence_timeout` (`dialogue_state_machine.py:456`) в prod не вызывается; докстринг `:6` обещает «SILENCED → IDLE (after timeout)» | совпало — SILENCED бессрочен |
| V16 | `IGNORE_STOP_MS:700` (`dialogue_node.py:9639`, `tts_node.py:1843`) | совпало |
| V17 | Таймаут тула 10 с, `_LONG_TOOL_TIMEOUTS = {}` (`executors/ros_mcp.py:25,48`) | совпало |
| V18 | 10 флагов `_*_retry_used` в `dialogue_node.py`; 11 `build_*_retry_prompt` в `dialogue_guards.py`; `PHANTOM_ACTION_VERB_STEMS :2074`, `detect_phantom_action_claim :2216`, `detect_universal_action_claim :1830`, `is_metalanguage_babble :719` | совпало |
| V19 | `_SILENT_DONE_MARKERS` (`agent_core.py:440`, проверки `:1241,1265`) | совпало |
| V20 | `master_prompt_compact.txt` 60 490 байт, 45 упоминаний `RULE #`; промпт ссылается на гард (`:228`) | совпало |
| V21 | Каталог тулов: 66 записей, 60 видимых LLM (`python -c` через `rob_box_core.tool_catalog`) | совпало |
| V22 | `architecture/ownership.yml` — `nodes: {}`, `topics: {}`, `capabilities: {}`, `features: {}` | пуст, совпало |
| V23 | Пакетов сообщений в `src/`: `rob_box_perception_msgs`, `rob_box_supervisor_msgs`, `robot_sensor_hub_msg`; голосовых `*_msgs` нет; `rob_box_core/utterance.py` уже даёт dataclass `Utterance{text, sink, priority, voice, language, emotion, speech_id}` | совпало |
| V24 | ADR-0084 «Статус: accepted»; ADR-0021 «Статус: proposed» (18.08); `CONTEXT.md:109-116` «Ход» ссылается на `TurnGuards`; файлов `docs/adr/015*` нет — номер 0150 свободен | совпало |

- Дополнительно проверено: `SttAdmission` — 12 шагов (`stt_admission.py:570-840`), `MediaRouter` держит фиксированные тексты `*_TEXT/*_FAIL_TEXT` (`media_router.py:165-176`), `HealthAwareFallbackLLM` ловит квоту подстроками `2056/1008/token plan` (`health.py:163-168,540`), `slice_policy.yaml:137-143` разрешает 5 несуществующих тулов, `voice_memory_adapter.py:47-49`: **«Shifu directive 2026-09-02 forbids persisting dialogue turns in production»** — это ограничение на дизайн памяти (§7.4, §18 В8).
- **Ничего не запускалось** ни локально, ни на роботе. Все величины задержек, долей и частот — из тел issue, памяти проекта и материалов 01–06; где я их цитирую, стоит «(из материалов)».

### 0.1 Расхождения материалов с кодом (верю коду)

| Утверждение в материалах | Что на `61fd84ff7` | Следствие |
|---|---|---|
| 03 §0, 01 §0: меток `TEMP(ADR-0148)` в `src` — 0 | `grep "TEMP(ADR"` даёт **2**: `club_transition.py:233`, `dj_mode.py:85` (оба — страховка #3136 на музыкальном пути); точная строка `TEMP(ADR-0148)` не матчится, потому что в коде `TEMP(ADR-0148, #3136/…)`. 04 §4 это и имел в виду | На диалоговом пути меток 0 — вывод 03 остаётся; гард моратория (§11, PR-0) должен искать `TEMP(ADR-0148` без закрывающей скобки |
| ADR-0149 §5.3: «число `re.compile` в `dialogue_guards.py` (34) и `music_guard.py` должно падать» | в `music_guard.py` `re.compile` = 0; там регексов нет, есть бюджеты ретраев | метрика для `music_guard.py` — размер файла и число вердиктов, не `re.compile` (§14 A8) |
| 00-brief §Ограничения: «ADR-0013 — малые PR» | `docs/adr/0013-respeaker-dsp-tuning-mix-ch0.md` — про ReSpeaker; правило PR — `AF-0013-incremental-delivery-over-big-bang.md` (ADR-0149 §0.1 это уже отмечал) | ссылаюсь на ADR-AF-0013 |
| 02 §2.2: `/voice/tts/request` — 5 писателей в коде | по `create_publisher` с литералом — 3; ещё 2 через константы (`say.py:61`, `supervisor_node.py:397`); рантайм (06 §2) — 5, включая `joystick_control_node` с Main Pi | число подтверждено, но статический аудит пропускает константы — гард владельцев (§10.3) должен считать по рантайм-снимку |
| 02 §4 «История»: «живого вызова `save_turn` нет» | вызовы `voice_memory.py:33,36` — в докстринге-примере; `VoiceMemoryAdapter.save_turn` — deprecated-заглушка по директиве Шифу 02.09 | история не персистится **намеренно**; v2 это уважает (§7.4), вопрос В8 |
| 02 §3.1 п.22, 03 T7: «8 писателей `next_transition_at`» | не пересчитывал — музыкальный путь, его меняет #3312 | в этом ADR не используется |
| 05 §1–§2: `Pipecat ... issue #5305`, `LiveKit FallbackAdapter` — «(поиск)», «не подтверждено» | не проверял; беру из 05 только форму паттерна, не детали реализации | §4, §6 ссылаются на паттерн, не на API |

---

## 1. Проблема: классы отказов и семь корней

Товарищ Шифу слышит: «робот сказал, что записал, а не записал», «растерялся — бит не запустился» при играющей музыке, «читает звёздочки и system», молчит минуту, после «хватит» глух до утра, называет незнакомца Борисом, говорит голосом вчерашнего диджея. Материалы сводят это к десяти классам (01 §1) и семи корням (03 §3). Коротко, с цифрами (из материалов, если не сказано «проверено»):

| Класс отказа (01 §1) | Цифры | Open на 01.10 |
|---|---|---|
| 1. LLM не вызвала тул / выдумала успех | ≥ 10 итераций заплаток с 02.2026 по 01.10; детекторы под каждую предметную область (навигация → музыка → поиск → память → пресеты → ТАРС) | #3266, #3271, #3297 |
| 2. Гарды и синтетические ретраи ломают диалог сами | 14 типов ретраев, 10 флагов `_retry_used` (проверено); на одну Bug C — 13+ issue; до 8 вызовов LLM за ~70 с на фразу (#1881); 27 возможных вызовов на фразу (1 + 8 итераций) × 3 (проверено V4–V5) | #3144 |
| 3. Служебный текст и разметка уходят в TTS | strip-фильтр расширяли ≥ 6 раз; регрессия ×193/час (#2558); 5 копий набора done-маркеров (проверено V19) | #3269, #3296 |
| 4. Тишина, задержки, пустой ответ | 33 с – 3 мин; «Принял.» вместо честного отказа (проверено V8) | #3265 |
| 5. Молчаливая деградация облаков | дефолтный LLM-провайдер менялся ≥ 7 раз; `tool_choice` трижды туда-обратно; MiniMax молча игнорирует `tool_choice` (ADR-0143) | — |
| 6. Wake-word и допуск | 26 вариантов вейка, включая «бот», «роб», «рома» (проверено V10); три дрейфующие копии списка | — |
| 7. Барж-ин, отмена, гонки | STOP шлют 4 ноды (проверено V12); отмена хода не доходит до HTTP-стрима (#1280); `/voice/current_dialogue_id` никто не пишет (проверено V9); окно `IGNORE_STOP_MS:700` вместо id (проверено V16) | — |
| 8. Утечки состояния | SILENCED без TTL (проверено V15); марафон 29→30.09: 1/12 актов зелёный; историю чистили ≥ 25 коммитов | #3145, #3000 |
| 9. Идентичность | ~25 issue за три дня 22–24.09; `utterance_id` чинили 5 раз за 2 дня | #2754 |
| 10. Дубли реализаций | 5 детекторов «стоп», 4 — «громче», ~18 списков музыкальных тулов, 17 топиков с несколькими писателями (06 §2); `ownership.yml` пуст (проверено V22) | #3108, #3109 |

Темп: `dialogue_node.py` — 353 fix-коммита из 526; август 274 и сентябрь 272 fix-коммита по диалоговым каталогам (01 §15). `DialogueNode` WMC 1 103 при лимите 80, 210 методов, TCC 0.04 (02 §1.1, 06 §4).

### 1.1 Семь корней (03 §3, принимаю как есть)

- **К1.** LLM одним свободным ответом выбирает намерение, параметры и фразу об успехе.
- **К2.** Fire-and-forget тулы: `speak_text`, `set_voice`, `set_tts_provider`, `set_dj_mode`, `play_sound`, `play_animation`, `register_speaker` возвращают успех в момент публикации в топик (02 §6.1); планировщик отдаёт `{"status": "queued"}`.
- **К3.** Состояние без единственного владельца и событийного контракта: два владельца DJ-флага и голоса TTS, один SILENCED на двух логических писателей без поля «кто поставил», 17 топиков с несколькими писателями, JSON в `String` без схем.
- **К4.** Два канала речи (свободный текст и `speak_text`) плюс протокол done-маркера.
- **К5.** Контекст — смесь текста и разметки (`[Spkr:…]`, `[TG]`, `[URGENT_BACKLOG]`, `[CRITICAL]`), ~70k токенов на ход при `skill_tool_narrowing: false`.
- **К6.** Решение «принять ответ» — после генерации, эвристиками по тексту, с синтетическими ретраями (`_handle_result`, CC 82).
- **К7.** Божественный узел и размазанное знание без механизма удаления.

ADR-0148 §4 постановил: музыка — первая волна (ADR-0149), `DialogueNode` — следующая, на тех же принципах. Этот ADR — её дизайн.

---

## 2. Решение: границы, модули, владельцы

### 2.1 Принципы (ADR-0148, применённые к диалогу)

1. **Понимание — команда, не ответ.** LLM на каждом ходе выдаёт **одну команду из закрытого перечня** (`Say | Act | Ask | Pass`) со структурными полями. Код её валидирует и исполняет. Паттерн — Rasa CALM (05 §6: command generator → `StartFlow/SetSlot/Clarify/ChitChat`), не ReAct-агент.
2. **Один канал речи.** Личность говорит **только** текстом команды `Say` (или `Ask`). Тул `speak_text` для Личности исчезает; done-маркеров нет; служебное в речь попасть не может, потому что служебное в контексте идёт структурой, а не текстом (§7.3). Пропадает весь К4.
3. **Фразу об исполнении строит код из события владельца ресурса.** `Act` → исполнитель → событие (`started/finished/rejected/timeout`) → шаблон из каталога фраз (`rob_box_dialog/phrases/ru.yaml`, как HA `responses/<lang>` и OVOS `.dialog`, 05 §3–§4). LLM не утверждает, что действие произошло — ей просто не дают слова в этом месте (LiveKit `say()` + `StopResponse`, 05 §2).
4. **Сессия — один объект с одним владельцем.** `Session` (режим тишины с TTL и `set_by`, персона, голос «запрошен/применён», активная сцена, текущий собеседник, окно адресации) живёт в `dialog_node`, пишет только код, LLM видит read-only снимок (Letta `read_only` block, OVOS Session, 05 §4, §7). Сцены — стек кадров с явным `pop` и возвратом слотов (Rasa dialogue stack, 05 §6).
5. **Тулы видны по возможности** (SayCan «can» от кода, 05 §8): в SILENCED нет `Say`-действий с динамиками, без моторов нет движения, в v1-музыке нет `dj_set`. Фильтр — функция `available_tools(session, capabilities)`, не правило в промпте.
6. **Деградация громкая и типизированная.** Каждый провайдер публикует возможности и здоровье (Wyoming `describe/info` + коды ошибок стадий HA, 05 §3, §5); смена провайдера/голоса озвучивается шаблоном ровно один раз и видна в `Session.applied`.
7. **В историю пишется только то, что озвучено** (OpenAI `truncate` до `audio_end_ms`, Pipecat `TTSTextFrame`, 05 §1, §9), плюс структурные результаты действий. Инструкции и маркеры в истории не живут.
8. **Знание — один модуль** `rob_box_dialog.knowledge` (вейк-слова, команды тишины, глаголы громкости, каналы громкости, классы действий и их таймауты, шаблоны фраз). Остальные импортируют; копии удаляются с grep-доказательством (память `no-two-implementations`).
9. **Никаких регексов по тексту ответа LLM и синтетических ретраев** в v2. Невалидная команда → `Ask`-шаблон из кода («Не понял, повтори»), один раз, без второго вызова LLM. Это честный FAIL вместо «Принял.».

### 2.2 Пакеты и модули

```
src/rob_box_dialog/                      # НОВЫЙ пакет. Чистый Python: без rclpy, без сети, без LLM-клиентов.
  rob_box_dialog/
    knowledge.py        # одна таблица: WAKE_WORDS (из wake_words.yaml), SILENCE/UNSILENCE, VOLUME_WORDS,
                        # VOLUME_CHANNELS, ACTION_CLASSES{query, action, terminal} с таймаутами и
                        # interruptible, SCENE_KINDS, PHRASE_KEYS. Генерирует фрагменты промпта и схем.
    session.py          # Session (frozen snapshot + owner-методы), SceneFrame, стек сцен, TTL; события SessionChanged
    address.py          # AddressPolicy: адресовано ли роботу (вейк | окно адресации | подтверждённый собеседник)
    grammar.py          # Tier-1: закрытая грамматика команд (расширение media_command_grammar): стоп, тишина/выход,
                        # громкость по каналу, голос, новая сессия, диджей, да/нет на вопрос о личности
    command.py          # Command = Say | Act | Ask | Pass (frozen dataclasses) + parse/validate из tool_call или JSON
    tools_view.py       # available_tools(session, capabilities) -> узкий срез каталога (имена + схемы из rob_box_core)
    context.py          # build_context(session, turn_log, facts) -> структурные блоки для LLM (read-only), окно спикинг-истории
    execute.py          # Executor: Act -> ActionRequest; ожидание ActionEvent владельца; ActionResult{status, …}
    phrases/ru.yaml     # каталог фраз по ключу результата: {action_class}.{status}[.{reason}] с вариантами и слотами
    respond.py          # phrase_from_result(result, session) -> Utterance; speech_for(Say|Ask)
    turn_log.py         # TurnLog: события хода (turn_started, addressed, command, action_*, spoken, interrupted, error)
                        #   с turn_id/epoch; проекция в окно LLM; хранение — в памяти (§7.4, В8)
    interrupt.py        # InterruptPolicy: решение по classes/epoch — отменить речь, отменить LLM-стрим, не трогать действие
    degrade.py          # ProviderState -> решение режима (tool_calls | json_command), фраза деградации один раз на смену

src/rob_box_voice/rob_box_voice/
    dialog_node.py      # НОВЫЙ тонкий хост v2 (≤ 600 строк, WMC ≤ 80): ROS-подписки/публикации, таймеры, вызовы rob_box_dialog
    core/stt_admission.py   # ОСТАЁТСЯ: 12 шагов допуска; WakeWordStep → address.py; MediaCommandStep → grammar.py
    core/media_command_grammar.py, media_router.py  # ОСТАЮТСЯ как Tier-1 для музыки; грамматика расширяется, не копируется
    tts_node.py         # становится ВЛАДЕЛЬЦЕМ РЕЧИ: один вход SpeechRequest, события SpeechEvent, отмена по speech_id/epoch
    stt_node.py         # STT + ProviderState(stt); STOP не шлёт — шлёт InterruptRequest
    command_node.py     # движение по /dialog/intent, а не по /voice/stt/result напрямую (адресация — у диалога)

src/rob_box_dialog_msgs/                 # НОВЫЙ пакет сообщений (§10)
src/rob_box_harness/rob_box_harness/core/agent_core.py   # остаётся как LLM-клиент с тул-циклом ≤ 2 итераций «query»;
                                                          # окно истории и скиллы переезжают в rob_box_dialog (§7)
src/rob_box_supervisor/                  # ТАРС строится тем же rob_box_dialog с AgentSpec(operator) (§9.3)
src/rob_box_telegram/                    # канальный адаптер DialogInput/DialogOutput (§9.2)
```

Бюджет ADR-0145 (WMC ≤ 80, методов ≤ 40) — без исключений для новых классов. Каждый модуль `rob_box_dialog` — ≤ 5 публичных функций.

### 2.3 Владельцы состояния (заполняет `architecture/ownership.yml`, §10.3)

| Состояние | Владелец (единственный писатель) | Как публикуется | Кто читает |
|---|---|---|---|
| Сессия: эпоха, стек сцен, тишина (TTL, `set_by`), персона, голос `requested/applied`, собеседник, окно адресации | `dialog_node` (`rob_box_dialog.session`) | latched `/dialog/session` (`SessionState`), событие при каждом изменении | tts_node (голос), command_node (адресация), led_node, arbiter, quest, telegram, supervisor |
| Ход: `turn_id`, эпоха, стадия (`addressed → understood → executing → speaking → done/cancelled`) | `dialog_node` (`turn_log`) | `/dialog/turn_event` (`TurnEvent`) | ТАРС-журнал, e2e-харнесс, метрики |
| Речь робота: очередь, что звучит, сколько символов озвучено | **tts_node** | `/voice/speech/event` (`SpeechEvent`: `queued/started/progress/finished/cancelled`, `speech_id`, `spoken_chars`) | dialog_node (история только по `spoken_chars`), audio_node, stt_node, animation |
| Вход речи: сегмент, текст, провайдер STT, `utterance_id` | stt_node | `/voice/stt/utterance` (типизируется в PR-2; `utterance_id` по ADR-0131) | dialog_node — **единственный** читатель; command_node и telegram напрямую не читают |
| Собеседник (биометрия) | speaker_id_node | `/voice/speaker/result` (ADR-0131 join по `utterance_id`) | dialog_node; mcp_server читает снимок сессии, а не свой `EncounterSeam` |
| Музыка: играет/трек/DJ | `PlayerOwner` в mcp_server (ADR-0149 §2.3) | latched `/voice/music/state`, `/voice/music/event` | dialog_node (контекст, кадр сцены `DJ`, фраза по `started/rejected`) |
| Провайдеры: здоровье и возможности | каждая нода — за свой вид: tts_node (TTS), stt_node (STT), dialog_node (LLM) | latched `/voice/providers/state` (`ProviderState[]`) | dialog_node (`degrade.py`), quest/панель ТАРС |
| Floor / режим аватара | арбитр (как сегодня — образец) | `/avatar/state` | без изменений |
| Громкость | три канала остаются у трёх владельцев (tts_node, `PlayerOwner`, sound_node), **но решение «какой канал»** — у `grammar.py` по состоянию сессии/музыки | через действия `set_volume(channel=…)` | — |

Удаляются как топики без владельца: `/voice/current_dialogue_id` (заменён `speech_id`+эпохой в `SpeechRequest`), `/voice/dialogue/response` (заменён `SpeechRequest`), `/voice/tts/request` и `/voice/tts/control` (заменены `SpeechRequest`/`SpeechCancel`), `/dialogue/control` + `/dialogue/control_ack` (заменены сервисом `DialogControl`), `/voice/dj_mode` как писатель диалога (владелец — музыка v2), `/harness/task_events` (заменён `/dialog/turn_event`).

---

## 3. Поток одной реплики (как будет)

Пример: «Робби, запомни, что Саша любит зелёный чай» (ровно #2755: «Записала…» при `tools=[]`).

```mermaid
flowchart TD
  MIC[ReSpeaker] --> AN[audio_node: VAD, сегмент]
  AN -- "/audio/speech_audio" --> STT[stt_node: minimax→yandex→vosk<br/>+ ProviderState(stt)]
  AN -- "/audio/speech_audio" --> SID[speaker_id_node]
  STT -- "/voice/stt/utterance (utterance_id, text, stt_provider)" --> ADM
  SID -- "/voice/speaker/result (utterance_id)" --> ADM
  STT -- "InterruptRequest, если речь робота звучит" --> INT

  subgraph DN["dialog_node (тонкий хост) + rob_box_dialog"]
    ADM["SttAdmission (12 шагов, как есть)<br/>WakeWordStep → address.py"] -->|"не адресовано"| BL[бэклог/drop]
    ADM -->|"адресовано"| T1{"Tier-1 grammar.py<br/>стоп · тишина · громкость · голос ·<br/>новая сессия · диджей · да/нет"}
    T1 -- "распознано" --> EXE
    T1 -- "нет" --> CTX["context.py: снимок Session (read-only блок),<br/>окно озвученного, факты о собеседнике,<br/>available_tools(session, caps) ≈ 6–12 схем"]
    CTX --> LLM{{"LLM: ОДНА команда<br/>Say | Act | Ask | Pass<br/>(tool_call, либо JSON при provider.no_tool_choice)"}}
    LLM --> VAL["command.parse+validate<br/>невалидно → Ask-шаблон, 0 ретраев"]
    VAL -- "Say/Ask" --> SP
    VAL -- "Act(remember, {person, fact})" --> EXE["execute.py: ActionRequest →<br/>владелец ресурса (mcp/память/плеер/tts)"]
    EXE -- "ActionEvent done/rejected/timeout" --> RES["respond.py: phrase_from_result<br/>«Запомнил: Саша любит зелёный чай»<br/>/ «Не смог записать — память недоступна»"]
    EXE -- "query-тул (search_web, get_time)" --> LLM
    RES --> SP["SpeechRequest(speech_id, epoch, text, interruptible)"]
    INT["interrupt.py: по эпохе —<br/>отменить речь и стрим, действие по классу"] --> SP
    SP --> LOG["turn_log: spoken только по SpeechEvent.spoken_chars"]
  end

  SP -- "/voice/speech/request" --> TTS["tts_node — владелец речи:<br/>очередь, провайдер, отмена по speech_id/epoch,<br/>ProviderState(tts)"]
  TTS -- "/voice/speech/event" --> DN
  TTS --> SPK[динамики]
  EXE -- "/mcp/execute (ActionRequest)" --> MCP[mcp_server: тулы, PlayerOwner]
  MCP -- "/mcp/result + /voice/music/event" --> EXE
```

Точки решения — в целевой форме (против таблицы 02 §3.1):

| Решение | v1 (02 §3.1) | v2 |
|---|---|---|
| 1–6 речь/эхо/провайдер STT/ТАРС-или-личность | код в audio/stt | **без изменений**, плюс `ProviderState(stt)` |
| 7 адресовано роботу | вейк на каждой фразе, список из 26 искажений | `address.py`: вейк **или** окно адресации N с после ответа робота **или** подтверждённый собеседник в активной сцене (OVOS converse, HA `continue_conversation`, 05 §3–§4; параметры — В2) |
| 8–9 тишина | подстрока «хватит», без TTL | `grammar.py` по границам слов; кадр сцены `Silence(ttl, set_by)` |
| 10 движение/стоп | command_node мимо допуска | только после допуска: `dialog_node` публикует `/dialog/intent`, command_node исполняет |
| 12 медиа-команды | `MediaRouter`, при лишнем слове `to_llm` | остаётся Tier-1; при промахе LLM получает **узкий** `request_music/dj_set` (ADR-0149 §5.1) и тоже не сочиняет успех |
| 14 кто говорит, спросить ли имя | код (после #2888) | код; вопрос — `Ask`-шаблон из `phrases`, ответ «да/нет» — Tier-1 |
| 15 скилл | 27 регексов `skill_router` + LLM `load_skill` | скилл = сцена из грамматики/команды; `tools_view` даёт срез; `skill_router.py` удаляется |
| **16 что сказать и какие тулы** | **LLM, свободно** | **LLM — одна команда из перечня**; параметры — по узкой схеме с enum |
| 17–18 порядок/очередь тулов | код | код (`ACTION_CLASSES`); одно действие на ход, кроме `query` |
| **19 принять/переспросить/замьютить** | **эвристики по тексту, 14 ретраев** | **валидатор команды до исполнения; 0 ретраев** |
| 20 сказать целиком/первое предложение | `decide_turn_speech` | `respond.py`: длина `Say` ограничена схемой (`max_length`), не промптом |
| 21 провайдер TTS | tts_node | tts_node + `ProviderState(tts)` + фраза деградации |
| 22 DJ-переход | таймер + LLM | **музыка v2** (ADR-0149), диалог только слушает события |

---

## 4. Что решает код, что — LLM

### 4.1 Узкий слот LLM: команда

```python
# rob_box_dialog/command.py (frozen dataclasses; схема генерируется из них для tool_call и для JSON-режима)
class Say:   text: str                      # ≤ 280 символов (В5); одна-две фразы; никаких действий
class Ask:   text: str; expects: Literal["yes_no", "name", "free"]   # робот задаёт вопрос; окно адресации открывается
class Act:   tool: str; args: Mapping[str, Any]                      # ровно один тул из available_tools(session)
class Pass:  reason: Literal["not_addressed", "nothing_to_say", "unclear"]  # молчать — честный исход

Command = Say | Ask | Act | Pass
```

- **Форма:** если `ProviderState(llm).capabilities.tool_choice == true` — четыре команды подаются как тулы с `tool_choice="required"`, и `Act` несёт вложенный вызов узкого тула. Если провайдер игнорирует `tool_choice` (MiniMax, ADR-0143) — **JSON-режим**: системный блок требует один JSON-объект по схеме, парсер `command.parse(text)` берёт первый валидный объект; текст вне JSON отбрасывается, не озвучивается. Режим выбирает `degrade.py` по `ProviderState`, не промпт.
- **Валидатор** (`command.validate`): тул ∈ `available_tools`; аргументы по JSON-схеме каталога (`rob_box_core.tool_catalog`), enum/границы в схеме (громкость 0–100, голос из реестра) — poka-yoke (05 §10); `Say.text` без управляющих последовательностей (структурно: поле строки, не разметка). Невалидно → `Ask("Не понял, повтори")` из шаблона. **Одна** попытка LLM на ход; второго вызова по этой причине нет.
- **Query-цикл:** для тулов класса `query` (поиск, время, погода, память-чтение, `get_*`) результат возвращается LLM, которая снова выдаёт **одну команду** (обычно `Say`). Максимум 2 итерации (Letta `MaxCountPerStepToolRule`, 05 §7). Итого ≤ 3 вызова LLM на фразу; `_MAX_TOOL_ITERATIONS = 8` и 14 ретраев уходят.
- **Терминальность:** `Act` класса `action` терминален (Letta `TerminalToolRule`): после него LLM слова не получает; фраза — из события. `Act` класса `query` не терминален.

### 4.2 Что остаётся коду (таблица границ)

| Решение | Владелец v2 | Форма |
|---|---|---|
| Адресовано ли роботу | `address.py` | вейк по границам слов (одна таблица), окно адресации, подтверждённый собеседник |
| Стоп, тишина, выход из тишины, громкость, голос, новая сессия, диджей вкл/выкл, да/нет | `grammar.py` | закрытая грамматика; **работает в любом состоянии**, включая окно адресации и SILENCED (урок HA 883012, 05 §3) |
| Какой канал громкости | `grammar.py` по снимку музыки/речи | «громче» при играющей музыке → музыка; при речи робота → TTS; иначе — последний активный канал |
| Какие тулы видны | `tools_view.py` | функция от `Session` × `ProviderState` × `music_engine`; срез ≈ 6–12 схем |
| Принять команду | `command.validate` | до исполнения |
| Таймаут действия, можно ли прервать | `knowledge.ACTION_CLASSES` | навигация 120 с/непрерываемо; музыка ≤ 6 с (ADR-0149 A2); память 3 с; поиск 15 с/прерываемо |
| Фраза об исполнении | `respond.py` | шаблон по `(class, status, reason)` |
| Что в истории | `turn_log.py` | только озвученное + структурные результаты |
| Кто говорит сейчас / чей ход | `dialog_node` (эпоха) | эпоха в каждом `SpeechRequest`/`ActionRequest`; устаревшая эпоха → отбрасывается владельцем |
| Спросить ли имя | код (ADR-0131/0139, как сегодня) | `Ask(expects="name")` из шаблона |
| Деградация | `degrade.py` | режим LLM, фраза один раз на смену |

### 4.3 Что остаётся LLM

Смысл и речь: понять фразу в контексте, выбрать команду, сформулировать `Say`/`Ask`, выбрать значения из перечислений (`mood`, `genre`, `voice`), ответить на вопрос по результату `query`. Персона и стиль — в системном промпте, который **генерируется** из `knowledge` + `Session.snapshot` (имена тулов, единицы, списки голосов — не вручную; урок HA #182006/#182568, 05 §3). Целевой объём: системный блок ≤ 4k токенов + снимок ≤ 1k + схемы ≤ 12 тулов ≤ 4k + окно ≤ 3k → **≤ 12k токенов** против ~70k (02 §5).

---

## 5. Исполнение, событие, фраза

### 5.1 Классы действий (`knowledge.ACTION_CLASSES`)

| Класс | Примеры | Результат | Фраза | Прерываемо |
|---|---|---|---|---|
| `query` | `search_web`, `get_current_time`, `memory_search`, `get_music_state` | `data` → обратно LLM (≤ 2 итераций) | LLM `Say` | да |
| `action.local` | `remember(person, fact)`, `set_volume(channel)`, `set_voice`, `forget_session` | `ActionEvent` от владельца ≤ 3 с | шаблон | нет (мгновенно) |
| `action.media` | `request_music`, `dj_set` (ADR-0149 §5.1), `play_sound`, `play_animation` | `/voice/music/event started|rejected` (≤ 6 с) / `SpeechEvent`-подобное от sound_node | шаблон по событию | `dj_set stop` — да; старт — нет |
| `action.motion` | `navigate_to_waypoint`, `stop_motion` | Nav2 result (до 120 с) | шаблон «Приехал к…/Не доехал: …» | **нет** (LiveKit `disallow_interruptions`, 05 §2); «стой» — отдельная Tier-1 команда |
| `action.identity` | `register_speaker`, `merge_speaker` | событие speaker_id_node (`registered`, `speaker_id`) | шаблон | нет |

### 5.2 Контракт исполнителя

`execute.run(act, session) -> ActionResult`:

```python
@dataclass(frozen=True)
class ActionResult:
    turn_id: str; epoch: int; tool: str; cls: ActionClass
    status: Literal["done", "rejected", "timeout", "cancelled", "pending"]
    reason: str | None          # ключ из phrases (например "quota", "not_found", "no_motors", "busy")
    data: Mapping[str, Any]     # слоты для шаблона: title, bpm, voice, person, fact, place, seconds
    event_ref: str | None       # id события владельца (track_id, speech_id, nav goal_id)
```

- Для fire-and-forget тулов v1 (К2) владелец **обязан** ответить событием: `set_voice` → tts_node публикует `SpeechEvent{kind: voice_changed, applied: …}` или `rejected{reason}`; `register_speaker` → speaker_id_node публикует `registered`; `play_sound` → sound_node `started/finished`. Нет события к дедлайну класса → `timeout` → шаблон «Не дождался …» — честно (A3, A11). Тул, который возвращает `success=True` без события владельца, в v2 не регистрируется (гард `seam_without_consumer.py` расширяется проверкой «тул класса action имеет источник события», PR-7).
- `pending` используется только для `action.motion`: код сразу говорит шаблон «Еду к …» (это обещание, не отчёт; ключ `motion.started`), а по результату — «Приехал» / «Не доехал: …».
- **Идемпотентность по `turn_id`**: повторный `ActionRequest` с тем же `turn_id` владельцем отбрасывается (ADR-0149 I6) — повторный запуск сета из #2948 структурно невозможен.

### 5.3 Как событие возвращается в диалог

Все события владельцев — типизированные топики (§10): `/voice/music/event`, `/voice/speech/event`, `/voice/speaker/event`, `/voice/sound/event`, результат Nav2 через action-клиент в mcp_server → `/mcp/result`. `execute.py` сопоставляет событие с ожиданием по `(turn_id | event_ref)`; совпадений нет → игнор + `WARNING stale_event`. Событие не приходит → таймер класса → `timeout`. Это закрывает «гард проверял факт вызова, а не успех» (#2949): успех = событие.

### 5.4 Каталог фраз

`rob_box_dialog/phrases/ru.yaml`: ключ `{class}.{status}[.{reason}]`, 1–3 варианта на ключ (против роботизированности, OVOS), слоты `{title}`, `{voice}`, `{person}`, `{seconds}`. Тест: каждый ключ, который может породить `execute.py`, присутствует (`test_phrases_cover_results`). `MediaRouter.*_TEXT` (`media_router.py:165-176`, проверено) переезжают сюда; «Принял.», «Что-то я задумался», «Я тут растерялся — бит не запустился» удаляются. Пустой/невалидный ответ LLM → `understand.invalid` («Не понял, повтори, пожалуйста») — всегда честно о причине. Длинные ответы (`Say` > лимита) обрезаются **схемой**, не промптом: валидатор отклоняет, LLM видит ошибку схемы в JSON-режиме один раз… нет — **без ретрая**: длинный `Say` озвучивается до первой границы предложения в лимите, остаток — в лог `WARNING say_truncated` (A13 измеряет).

---

## 6. Речь, перебивание, тишина

### 6.1 Владелец речи — tts_node

- Один вход `/voice/speech/request` (`SpeechRequest`: `speech_id`, `turn_id`, `epoch`, `source ∈ {dialog, operator, telegram, system}`, `sink`, `priority`, `interruptible`, `text`, `voice?`, `emotion?`). Все шесть сегодняшних писателей (`/voice/tts/request`, V11) идут через клиент `rob_box_core.speech_client.say(Utterance, …)`; `Utterance` уже есть (`rob_box_core/utterance.py:119`, проверено) и становится полезной нагрузкой `SpeechRequest`.
- Один канал отмены `/voice/speech/cancel` (`SpeechCancel`: `speech_id | epoch`), **один писатель — dialog_node**. `IGNORE_STOP_MS:700` (T1) исчезает: отмена по эпохе не трогает реплику новой эпохи.
- События `/voice/speech/event`: `queued, started{speech_id, text_len}, progress{spoken_chars}` (по чанкам), `finished`, `cancelled{spoken_chars}`, `voice_changed{requested, applied, provider}`, `rejected{reason}`. Диалог пишет в историю `text[:spoken_chars]` (п. 7 §2.1).
- Нормализация текста для TTS (markdown, цифры, латиница) — только в tts_node, одно место (R14 класс (в) → законная функция владельца). `SYSTEM_TEMPLATE_REGURGITATE` в tts_node остаётся **только** как последняя страховка с меткой `TEMP(ADR-0148` и метрикой срабатываний → 0 за марафон → удалить (R9).

### 6.2 Политика перебивания (`interrupt.py`)

| Сигнал | Кто решает | Действие |
|---|---|---|
| Фраза с вейком во время речи робота | `address.py` → `interrupt.py` | `SpeechCancel(epoch)` → речь ≤ 300 мс; стрим LLM текущего хода отменяется по эпохе (отмена доходит до HTTP-клиента: `AgentCore.cancel(epoch)` — закрыть `httpx` stream, #1280); действие класса `action.motion`/`identity` **не отменяется**, `query` — отменяется (Pipecat `cancel_on_interruption`, 05 §1) |
| Фраза без вейка во время речи робота | `address.py` | игнор, если нет окна адресации; в окне — как с вейком |
| «Стой/стоп» (Tier-1) | `grammar.py` | `stop_motion` + `SpeechCancel` + `dj_set(stop)` по снимку — что сейчас активно |
| Эхо/короткий шум | audio/stt как сегодня (грейс 2.5 с, T2) | оставляем, помечаем `TEMP(ADR-0148`; ложное прерывание с возобновлением речи (LiveKit `resume_false_interruption`) — **не в этом витке** (§17 Г) |
| Новая сессия | `grammar.py` | `Session.reset()` → эпоха+1 → все владельцы отбрасывают запросы старой эпохи |

Писателей STOP становится 1 (было 4, V12). stt_node вместо STOP публикует `/dialog/interrupt_request{utterance_id}` — диалог решает за ≤ 20 мс (адресация уже посчитана на тексте первого сегмента; если STT отдаёт текст поздно — используется детекция вейка по первому сегменту, как сегодня `stt_node.py:2034`). Если замер покажет задержку > 300 мс до тишины — stt_node получает право прямой отмены **только** по эпохе из latched `/dialog/session` (решение после PR-9, §16).

### 6.3 Тишина

- `Silence` — кадр сцены: `{set_by: user|operator, ttl_s, reason}`. Голосовое «хватит/помолчи» → `set_by=user, ttl=600` (В3); операторская пауза (`DialogControl.hold`) → `set_by=operator, ttl=∞ до resume`. **`resume` оператора снимает только операторский кадр**; пользовательская тишина кончается по TTL, по «говори/отвечай» (Tier-1, по границам слов) или по новой сессии. Это закрывает вопрос 6 из 03 §2.
- В SILENCED Tier-1 команды работают (стоп, громкость, выход); `available_tools` не содержит ничего, что звучит; `Pass(not_addressed)` не озвучивается; «Останавливаюсь» от command_node через диалог тоже молчит (вопрос 20, 03 §2).
- `check_silence_timeout` в DSM удаляется вместе с DSM: состояние — в `Session`.

---

## 7. Сессия, сцены, персоны, история, память

### 7.1 Session

```python
@dataclass(frozen=True)
class SessionSnapshot:
    session_id: str; epoch: int; started_at: float
    scenes: tuple[SceneFrame, ...]          # стек: Silence | Persona | DJ | OperatorHold | Identity(ask)
    voice: VoiceState                       # requested, applied, provider  (одно место вместо D6)
    speaker: SpeakerRef | None              # person_id, name, confidence, since
    address_window_until: float | None      # окно адресации без вейка
    music: MusicStateRef                    # копия latched-снимка плеера (читатель, не владелец)
    providers: Mapping[str, ProviderHealth] # llm/stt/tts: ok|degraded|quota|auth|down
```

Пишет только `dialog_node` через owner-методы `session.push_scene / pop_scene / set_voice_applied / set_speaker / reset`. Каждое изменение → latched `/dialog/session` + запись в `turn_log`. LLM получает `SessionSnapshot` как **структурный read-only блок** (не текст с маркерами): «кто передо мной», «какой голос звучит», «что играет», «какая сцена».

### 7.2 Сцены как кадры стека

| Кадр | Push | Pop | Что возвращается при pop |
|---|---|---|---|
| `Silence(set_by, ttl)` | Tier-1 «хватит» / `DialogControl.hold` | TTL, Tier-1 «говори», `resume` того же `set_by`, reset | речь разрешена |
| `Persona(name, voice)` | `Act(set_persona)` / `dj_set start` (DJ-персона) | явная команда, конец DJ (`/voice/music/event idle`), reset | голос и промпт персоны прошлого кадра (#3000, акт 4 марафона) |
| `DJ(set_id)` | `/voice/music/event started{dj.enabled}` | `idle`/`finished`/`dj_set stop` | срез тулов без `dj_set(next…)`; окно адресации закрывается |
| `OperatorHold` | `DialogControl.hold(set_by=operator)` | `DialogControl.resume`, таймаут супервизора | — |
| `Identity(ask, person)` | код задал «X, это ты?» | ответ да/нет (Tier-1), таймаут 15 с | — |

`Session.reset()` (новая сессия, 23 фразы U8 → грамматика) выталкивает все кадры, увеличивает эпоху, очищает окно LLM, **не трогает** долгую память о людях и состояние плеера (музыка продолжает играть; «забудь всё» ≠ «выключи музыку» — #2835/#3217 решаются тем, что это две разные Tier-1 команды).

### 7.3 История для LLM

- Источник — `turn_log` текущей сессии; проекция: `user: <текст озвученной человеком фразы>` (без `[Spkr:]`, `[TG]`, `[URGENT_BACKLOG]` — эти факты в снимке сессии и в структурном поле `source`), `assistant: Say.text[:spoken_chars]`, `tool: ActionResult` структурой. Отвергнутые команды, невалидный JSON, служебные ошибки в окно не попадают (#3145 закрыт по построению).
- Окно: ≤ 12 ходов или ≤ 3k токенов; сброс по `reset()`; при смене `Persona` — явный `context_strategy` кадра: `RESET` для DJ-персоны (Pipecat Flows, 05 §1), `APPEND` для остальных.
- Бэклог неадресованных фраз (сегодня `[URGENT_BACKLOG]`) — отдельное поле снимка `recent_unaddressed: [{text, speaker, age_s}]`, ограниченное 3 записями и 120 с.

### 7.4 Память о людях и турнах

- Директива Шифу 02.09 (`voice_memory_adapter.py:47-49`, проверено): турны в prod не персистятся. v2 держит `turn_log` **в памяти процесса** с ретенцией сессии; на диск — только агрегаты метрик и события без текста (`turn_id, stage, status, latency_ms, llm_calls`) для приёмки §14. Решение о тексте — В8.
- Факты о людях: только через `Act(remember(person_id, fact))` → владелец памяти (mcp_server, одна БД `harness_voice.db`, одна таблица, из которой читает `memory_search` — закрывает #2793) → событие `saved{fact_id}` → фраза «Запомнил: …». LLM не пишет в память сама (урок отравленной галереи, 05 §7). Чтение — `query`-тул `memory_search(person_id)` с фильтром по собеседнику (#1770).
- ТАРС-журнал (#3297): записи получают `ts` и `ttl`; в контекст идут только записи моложе 30 мин плюс сжатая сводка «за сутки» с явной датой — структурно, не текстом «не отвечает».

---

## 8. Провайдеры: возможности и честная деградация

- `ProviderState` (latched `/voice/providers/state`, писатели — по виду, §2.3): `{name, kind: llm|stt|tts, health: ok|degraded|quota|auth|down, since, capabilities: {tool_choice, ssml, streaming, voices[]}, last_error_code}`. Классы ошибок — enum, не подстроки (`health.py:163-168` подстроки `2056/1008/token plan` переезжают в адаптер MiniMax как единственное место, где они допустимы; наружу — enum).
- `degrade.py` решает: режим команд (`tool_calls` vs `json_command`), порядок провайдеров (`HealthAwareFallbackLLM` остаётся как транспорт), и **одну** фразу на смену состояния: «Облачный голос недоступен, говорю запасным» / «Поиск сейчас недоступен» — по шаблону, не из LLM; повтор той же фразы не чаще 1 раза в 10 мин.
- `VoiceState.requested ≠ applied` всегда видно в снимке; «каким голосом ты говоришь» отвечает Tier-1 из снимка, не LLM.
- Бюджет времени хода — у `dialog_node`: `TURN_DEADLINE_S` (В5, предлагаю 12 с на LLM-ход). Истёк → `SpeechCancel` текущих ожиданий, фраза `understand.timeout` («Не успел подумать — повтори»), в лог `provider_latency{provider, p}`. Ретраи транспорта (`RetryPolicy`) считаются внутри дедлайна; MiniMax/DeepSeek-таймауты (90/30 с) становятся `min(provider_timeout, deadline_left)`.
- Параметры, которые объявлены и не читаются (`llm_timeout_sec`, `agent_max_turns`, `<provider>.*`, 03 §1.6) — удаляются из yaml и кода в PR-4 (grep: `declare_parameter("llm_timeout_sec"` = 0).

---

## 9. Интерфейс с музыкой (ADR-0149), Telegram, ТАРС

### 9.1 Музыка — только интерфейс (не перепроектируется)

- Диалог **читает** latched `/voice/music/state` и `/voice/music/event` (владелец `PlayerOwner`, ADR-0149 §2.3) и **вызывает** узкие тулы `request_music`/`dj_set` (§5.1 ADR-0149) через `Act` класса `action.media`; Tier-1 медиа-команды идут `MediaRouter → PlayerOwner` без LLM (ADR-0149 I19).
- Фраза об успехе — из `started{track_id, title, bpm}` / `rejected{reason}` по шаблонам `media.started`, `media.rejected.{reason}`; при `music_engine: v1` событий нет → `request_music` в `available_tools` отсутствует, работает только `MediaRouter` v1 с его `*_TEXT` (перенесёнными в `phrases`). Диалог v2 **не вызывает** `MusicGuard`, `DJModeController.tick`, `dj_set_boundary` ни при каком флаге; их удаление — в PR-13…15 эпика #3312, не здесь.
- Кадр `DJ` сцены создаётся/снимается по событиям плеера; второй DJ-флаг в диалоге (`DJState.enabled`) не существует. `/voice/dj_mode` диалог не пишет (владелец — музыка).
- Снимок музыки в контексте LLM — один блок из `/voice/music/state` (ADR-0149 §5.2).

### 9.2 Telegram — канал, не вторая голова

telegram_node перестаёт писать `/voice/stt/result`, `/voice/dialogue/response`, `/voice/tts/request`, `/voice/sound/stop` (V14). Вместо этого: `/dialog/input` (`DialogInput{source: telegram, chat_id, text, user_ref}`) и подписка `/dialog/output` (`DialogOutput{source, chat_id, text, audio_ref?}`). `address.py` для `source=telegram` — всегда адресовано; `Session` одна на робота, но `SpeechRequest.sink` для Telegram — `telegram` (не динамики), озвучка — по запросу (`Act(speak_aloud)` или оператор). `/avatar/command` → супервизор остаётся (операторский канал, не диалог).

### 9.3 avatar-supervisor (ТАРС) — тот же движок, другая спецификация

- `AvatarSupervisor` строит `rob_box_dialog` с `AgentSpec(operator)`: своя грамматика Tier-1 (операторские команды), свой срез тулов (`operator.*` по `slice_policy.yaml` — после удаления 5 призраков D17), те же `Command`, `Executor`, `phrase_from_result`, `SpeechRequest(source=operator, sink=headset|speakers)`. Второй `AgentCore` остаётся как LLM-клиент, но без собственной истории/гардов (#3298/#3305: `say` либо регистрируется в каталоге как `action.local` с событием `SpeechEvent`, либо удаляется — В7).
- `/dialogue/control` + `control_ack` → сервис `/dialog/control` (`DialogControl.srv`: `hold|resume|reset`, `set_by`, `ttl_s` → `ok, session_epoch, scenes[]`). Ack структурный, читатель — вызывающий.
- ТАРС-журнал по §7.4; `operator_system_prompt.txt` генерируется из `knowledge` + реестра тулов (имена не хардкодятся — #3298).

---

## 10. Типы сообщений: уход от JSON в `std_msgs/String`

### 10.1 Пакет `rob_box_dialog_msgs`

| Тип | Поля (кратко) | Заменяет |
|---|---|---|
| `msg/SpeechRequest` | `speech_id, turn_id, epoch, source, sink, priority, interruptible, text, voice, language, emotion, stamp` | `/voice/tts/request`, `/voice/dialogue/response` (String JSON) |
| `msg/SpeechCancel` | `speech_id, epoch, reason` | `/voice/tts/control` STOP/IGNORE_STOP_MS |
| `msg/SpeechEvent` | `kind {queued,started,progress,finished,cancelled,voice_changed,rejected}, speech_id, turn_id, epoch, spoken_chars, text_len, voice_requested, voice_applied, provider, reason, stamp` | `/voice/tts/state`, `/voice/tts/finished`, `/voice/tts/batch_*`, `/voice/tts/current_voice`, `/voice/tts/provider_state` (часть) |
| `msg/SessionState` | `session_id, epoch, scenes[] (SceneFrame), voice_requested, voice_applied, speaker_id, speaker_name, address_window_until, stamp` | `/voice/dialogue/state` (строка), `/voice/dialogue/barge_in_policy` |
| `msg/SceneFrame` | `kind, set_by, ttl_until, payload_json (только для persona/dj: имя, set_id)` | — |
| `msg/TurnEvent` | `turn_id, epoch, stage, status, reason, llm_calls, latency_ms, source, stamp` | `/harness/task_events`, `/dialogue/control_ack` |
| `msg/ActionRequest` / `msg/ActionResult` | `turn_id, epoch, tool, args_json, cls, timeout_s` / `turn_id, tool, status, reason, data_json, event_ref` | `/mcp/execute`, `/mcp/result` (String JSON) |
| `msg/ProviderState` (+ `ProviderStateArray`) | `name, kind, health, since, tool_choice, ssml, streaming, voices[], last_error_code` | `/voice/tts/provider_state`, `~/.rob_box/llm_health.json` как единственный источник |
| `msg/DialogInput` / `msg/DialogOutput` | `source, channel_ref, text, user_ref, stamp` / `source, channel_ref, text, speech_id` | telegram → `/voice/stt/result`, `/voice/dialogue/response` |
| `msg/Intent` | `turn_id, epoch, kind {stop_motion, move, …}, args_json` | command_node ← `/voice/stt/result` |
| `srv/DialogControl` | req `action {hold,resume,reset}, set_by, ttl_s` → resp `ok, epoch, scenes[]` | `/dialogue/control` + `control_ack` |

Правило: поле `*_json` допускается только для **открытых** словарей аргументов тулов (схему проверяет каталог), всё остальное — типизированные поля. QoS: события — RELIABLE KEEP_LAST 10; снимки — latched (TRANSIENT_LOCAL depth 1). `utterance_id` (ADR-0131) остаётся в `/voice/stt/utterance`, который типизируется в тот же пакет (`msg/Utterance`) в PR-2 вместе с `speaker/result` (`msg/SpeakerResult`), чтобы `semantic_topic_name_review` (06 §3) получил контракт.

### 10.2 Переход

Типизированный топик вводится **вместе** с его потребителем в одном PR (гард `seam_without_consumer.py`); старый `String`-топик удаляется в том же PR, когда все писатели переведены; если писатель — внешний пир (quest), старый топик живёт до PR quest с записью в `seam_allowlist.json` и датой. Два контракта одного состояния параллельно — только внутри одного PR-окна (ADR-AF-0013), не как режим.

### 10.3 `architecture/ownership.yml`

PR-2 заполняет `topics:` для всех `/dialog/*`, `/voice/speech/*`, `/voice/providers/*`, `/voice/stt/*`, `/voice/speaker/*`, `/voice/music/*` (из ADR-0149) и `nodes:` для dialog_node/tts_node/stt_node/speaker_id_node/mcp_server/telegram/supervisor с полем `owner` и `writers_allowed`. Новый гард `scripts/lint/ownership_check.py`: (а) каждый топик из статического инвентаря в этих префиксах имеет владельца; (б) рантайм-снимок `L: Architecture Audit` не содержит `multiple_writers` вне `writers_allowed`. Сегодняшние 17 multi-writer — в `runtime-baseline.json` как долг с номером PR, который его гасит (§11).

---

## 11. Миграция: strangler за флагом `dialog_engine`, план PR

**Флаг** `dialog_engine: "v1" | "v2"` — ROS-параметр launch-файла `voice_assistant_headless.launch.py` (общий yaml `docker/vision/config/voice_assistant/*.yaml`, дубль в `src/.../config` синхронизируется тестом, как `test_music_engine_flag.py`). `v1`: запускается `dialogue_node`, tts/stt/command работают по старым топикам через **адаптер совместимости** в tts_node (`SpeechRequest` ↔ старый JSON; один модуль, удаляется в PR-15). `v2`: запускается `dialog_node`, `dialogue_node` не стартует, старые топики диалога не создаются. Дефолт `v1` до приёмки §14.

Правила каждого PR: ≤ ~600 изменённых строк (ADR-AF-0013); проходит `cc_budget.py`, `class_budget.py`, `seam_without_consumer.py`, `cc_budget_refs.py`, `dialogue_skip_reasons.py`, `validate_adr_namespace.sh`, новый `ownership_check.py`; **ни один PR не правит старый путь, кроме удаления** (ADR-0148 §2.2); каждый PR удаляет то, что заменил, с grep-критерием в описании; raw-вывод pytest/CI/логов обязателен (AGENTS.md). Параллельная работа с эпиком #3312: PR-3…PR-7 не трогают `src/rob_box_mcp_tools/tools/music.py`, `core/dj_mode.py`, `core/music_guard.py` — только перестают их вызывать под `v2`.

| PR | Что делает | Что удаляет (grep-критерий) | Как проверить | Метрика приёмки |
|---|---|---|---|---|
| **PR-0** | этот ADR; `scripts/dialog/turn_metrics.py` (из логов: вызовов LLM на фразу, STT→первый звук, доля `Принял/растерялся`, маркеры в TTS), `scripts/dialog/honesty_audit.py` (фраза об успехе ↔ событие ≤ 2 с); baseline по логам марафона 29→30.09 → `docs/dialog/baseline_2026-10-01.md`; гард моратория `scripts/lint/temp_adr0148_check.py` (новый `re.compile`/ретрай в диалоговых файлах без `TEMP(ADR-0148` → fail); завести эпик и карточки | мёртвое: `prompts/master_prompt.txt`, `master_prompt_simple.txt` (D10, 57 КБ), `core/voice_command_handler.py` (D11), `startup_greeting_node.py`, `action_server/http_server.py` (D15), мёртвые функции D12 — `ls`/`grep -rn <имя>` = 0 | гарды docs; скрипты на логах марафона (вывод в PR) | baseline A1–A16 зафиксирован |
| **PR-1** | пакет `rob_box_dialog`: `knowledge.py` (вейк из `wake_words.yaml`, тишина, громкость, `ACTION_CLASSES`), `command.py` + валидатор + генерация схем, `session.py` (кадры, TTL, эпоха), `phrases/ru.yaml` + `respond.phrase_from_result` | копии знания: `command_parser.py:97,136-141,319`, `command_node.py:81`, `dialogue_node.py:587` (вейк), `dialogue_state_machine.py:431-435`, `dialogue_helpers.py:79-93` (громкость), `stt_node.py:226-240` запасная копия — `grep -rn '"хватит"\|"робокс"\|"громче"' src --include=*.py` вне `knowledge.py`/`media_command_grammar.py`/тестов = 0 | unit: 1000 случайных команд → валидатор; `phrases` покрывают все `(class,status)`; `class_budget` без новых больших | A8 (копии знания: 5→1, 4→1, 3→1) |
| **PR-2** | `rob_box_dialog_msgs` (§10.1) + `ownership.yml` заполнен + `scripts/lint/ownership_check.py`; `/voice/stt/utterance` и `/voice/speaker/result` типизируются с потребителем dialogue_node (v1) | старые String-версии этих двух топиков — `grep -rn '"/voice/stt/utterance"' src` = 0 | colcon build; `ownership_check` зелёный; `ros2 topic info -v` на роботе (raw) | A7 (ownership заполнен) |
| **PR-3** | tts_node — владелец речи: `SpeechRequest/Cancel/Event`, отмена по `speech_id`/эпохе, `rob_box_core.speech_client`; все писатели (`tools/dialogue.py`, `say.py`, telegram, stt_node, supervisor grip, joystick) → клиент; адаптер совместимости для v1 | `IGNORE_STOP_MS` (T1), подписки `/voice/current_dialogue_id` (D16), второй писатель `/voice/tts/batch_complete` — `grep -rn "IGNORE_STOP_MS\|current_dialogue_id" src --include=*.py` = 0; `create_publisher(... "/voice/tts/request"` вне `speech_client.py` = 0 | робот: 20 реплик из 3 источников; `SpeechEvent.spoken_chars` при отмене; рантайм-аудит `multiple_writers` по `/voice/tts/*` = 0 | A6 (−5 multi-writer), A12 (отмена ≤ 300 мс) |
| **PR-4** | `ProviderState` от tts/stt/dialogue(v1 тоже публикует); `degrade.py`; `set_voice/set_tts_provider` → запрос + событие `voice_changed`; фраза деградации один раз | `VoiceStateStore` (D6), строка «Голос установлен» (`tools/dialogue.py:1464`), мёртвые параметры `llm_timeout_sec`/`agent_max_turns`/`<provider>.*` (03 §1.6) — `grep -rn "Голос установлен\|llm_timeout_sec\|agent_max_turns" src` = 0; подстроки квоты вне адаптера MiniMax = 0 | робот: подложить мёртвый ключ Yandex/MiniMax → одна фраза деградации, `ProviderState.health=quota` (raw лог) | A11 |
| **PR-5** | адресация: `address.py` (вейк по границам слов + окно адресации + подтверждённый собеседник), `stt_node` → `interrupt_request` вместо STOP, command_node ← `/dialog/intent` (через допуск) | писатели STOP в stt_node/audio_node (`grep -rn '"/voice/tts/control"' src` = 1 — адаптер v1), подстрочный `any(w in text_lower` в stt_node = 0, `DEFAULT_WAKE_WORDS` определён один раз | робот: «работает» не будит; «робот, хватит» → один ответ (A16); окно адресации: «громче» через 5 с после ответа принимается | A15, A16 |
| **PR-6** | понимание: `grammar.py` (Tier-1 расширена: тишина/выход с TTL, громкость по каналу, голос, новая сессия, да/нет), `tools_view.available_tools`, `context.build_context`, JSON-режим команд при `no_tool_choice`; подключается в **dialogue_node v1** только как замена `skill_router` + `skill_tool_narrowing` (чтобы срез заработал до v2 — ADR-0148 §2.4) | `skill_router.py` (27 `re.compile`), ключ `skill_tool_narrowing` из yaml, `_MUSIC_STOP_OVERRIDES`, U8 23 фразы, U9, T10 `MEDIA_ACTION_BACKING_S` — `grep -rn "skill_tool_narrowing\|MEDIA_ACTION_BACKING_S\|_MUSIC_STOP_OVERRIDES" src` = 0 | unit на грамматике (корпус фраз из issue #1292, #2897, #2971, `363a2d6e9`); робот: 30 фраз, лог `tools_visible=N` ≤ 12 | A5 (токены ≤ 12k на вызов) |
| **PR-7** | исполнение и ответ: `execute.py` (классы, таймауты, ожидание события), `respond.py`; событийные ответы владельцев: speaker_id `registered`, sound_node `started/finished`; `speak_text` скрыт от Личности; `seam_without_consumer` проверяет «action-тул имеет событие» | в v1-ноде: `detect_phantom_action_claim`, `detect_universal_action_claim`, `ACTION_CLAIM_RULES`, S4–S7, S12, `_SILENT_DONE_MARKERS` ×5, R11, фолбэки `:1986,2413,2590,2639` — `re.compile` в `dialogue_guards.py` 34 → ≤ 15; `_retry_used` 10 → ≤ 3; `grep -rn "_SILENT_DONE_MARKERS\|done_marker" src` = 0 | эталонный набор 60 фраз (из #2755, #2780, #2949, #3266, #3297 и ещё 55 по 01 §2) на роботе: `honesty_audit.py` | A3 = 0/60 |
| **PR-8** | `dialog_node` v2 за флагом: тонкий хост, `turn_log`, окно истории из озвученного, бюджет хода `TURN_DEADLINE_S`; launch по флагу | при `v2`: `dialogue_node` не стартует; `/voice/dialogue/response`, `/voice/dialogue/state`, `/harness/task_events`, `/dialogue/control*` не создаются (`ros2 topic list` на роботе, raw) | робот `v2`: сценарии `voice_core` марафона (акты 1–3); `turn_metrics.py` | A1 (p50=1, p100≤3), A2, A4 (WMC dialog_node ≤ 80), A13, A14 |
| **PR-9** | `interrupt.py`: отмена по эпохе до HTTP-стрима (`AgentCore.cancel`), классы прерываемости, «стой» по снимку | D14 мёртвое VAD-прерывание (`llm_processing`, `interrupt_agent_loop`), ветка `barge_in_policy=classify`, `/voice/dialogue/barge_in_policy` — grep = 0 | робот: 20 перебиваний; 0 тулов старого хода после перебивания (лог `stale_epoch`) | A12 |
| **PR-10** | сцены: `Silence(TTL,set_by)`, `Persona`, `DJ` по событиям музыки, `OperatorHold`; `DialogControl.srv`; `Session.reset()` | `DialogueStateMachine` (SILENCED/`check_silence_timeout`), `_pause_reason/_paused_at_ms`, `dj_set_boundary.py` (ADR-0129-dj штамп), `/dialogue/control` + `_ack` — `grep -rn "DialogueStateMachine\|check_silence_timeout\|control_ack" src --include=*.py` = 0 | марафон 12 актов (`run_night_marathon.sh`): «хватит» в акте 8 кончается по TTL; голос из акта 4 не доживает до акта 5 | A9, A10 (≥ 11/12) |
| **PR-11** | память: `remember`/`memory_search` на одной таблице с `person_id`; снимок сессии вместо маркеров в тексте; ТАРС-журнал с `ts/ttl` | R13 (`_HISTORY_MARKER_RE`, `_SPEAKER_TAG_RE`, `_META_PREFIX_RE` в `speak_helpers.py`), `discard_last_reply`, `[URGENT_BACKLOG]`/`[TG]`/`[Spkr:` в `dialogue_node.py` — `grep -rn "URGENT_BACKLOG\|\[Spkr:" src --include=*.py` = 0; `voice_memory.db` путь = 0 упоминаний | робот: «запомни/что помнишь» ×10, два собеседника; `honesty_audit` | A3 (память), A13 |
| **PR-12** | Telegram-канал: `DialogInput/Output`, `sink=telegram` | писатели telegram в `/voice/stt/result`, `/voice/dialogue/response`, `/voice/tts/request`, `/voice/sound/stop` — `grep -n create_publisher telegram_node.py` без `/voice/` и без `/avatar/command` = 0 | Telegram-сессия «Клод …» (память `telegram-bot-audio-and-klod-channel`): 10 сообщений, raw логи | A6 (−4 multi-writer) |
| **PR-13** | ТАРС на `rob_box_dialog` с `AgentSpec(operator)`; `say` в каталог с событием или удаление (В7); `slice_policy` без призраков; `_utterance_fallback.py` удалён (D2) | `sup/_utterance_fallback.py`, 5 призраков `slice_policy.yaml:137-143`, собственная история/гарды супервизора — `grep -rn "_utterance_fallback\|dialogue_pause\b" src` = 0 | шлем/Telegram: 20 операторских команд, `honesty_audit` на журнале (#3297 сценарий) | A3 (ТАРС), A6 |
| **PR-14** | приёмка §14 целиком (марафон 12 актов + эталонный набор 60 + Telegram + ТАРС) с raw → решение Шифу → `dialog_engine: v2` по умолчанию | — | все A1–A16 в PR с raw, run_id | — |
| **PR-15…17** | удаление старого пути тремя PR: (а) `dialogue_node.py`, `dialogue_guards.py`, `turn.py`, `turn_speech*.py`, `speak_helpers.py` регексы, адаптер совместимости в tts_node, старые String-топики диалога; (б) промпты: `master_prompt_compact.txt` → генерируемый из `knowledge` (`RULE #` = 0), скиллы `voice-tts/player/navigation/core/memory` → схемы; `_tool_catalog_data.py` без скрытых копий; (в) тесты на номера issue и текст промпта, baseline-ы `cc_budget`/`class_budget`/`seam`/`runtime-baseline` | `wc -l dialogue_node.py` → файла нет; `grep -rn "Bug [A-F]" src --include=*.py` = 0 (вне музыки до #3312 PR-13); `grep -oE "#[0-9]{3,4}" src/rob_box_dialog` ≤ 20; `grep -c "RULE #" prompts/` = 0 | pytest -v полный; CI run_id; рантайм-аудит без `multiple_writers` repo-scope | A4, A6 = 0, A8 |

**Первые PR можно начать без решений Шифу**: PR-0, PR-1, PR-2 не меняют поведение на роботе и не зависят от §18. PR-3 зависит от В1 (один канал речи — да/нет для ТАРС-`say`) только в части `say.py`.

---

## 12. Таблица костылей на удаление (группы из 03 §1, класс по ADR-0148, PR)

| Группа (03) | Что | Класс | Чем заменено | PR |
|---|---|---|---|---|
| R1, S1, P1(часть) | babble-гард и ретрай, `BABBLE_BANNED_OPENERS` | (а) | команда `Act`/`Say` — «делать или говорить» решено структурой | PR-7 (отключение в v1), PR-15 (удаление) |
| R2 | `is_planning_narration` → мьют | (б) | один канал речи: озвучивается только `Say.text` | PR-8, PR-15 |
| R3, R4, R5, S4, S5, S7, P1 | `ACTION_CLAIM_RULES`, `_ACTION_VERBS_*`, `PHANTOM_*`, ретраи | (а) | фраза об успехе из события (§5) | PR-7 |
| R6, S6, P3 | «не знаю мелодии» без поиска, Bug F | (а) | `search.find` (ADR-0149) как `query`; результат структурный | PR-7 (отключение), #3312 |
| R7, S2 | код Renardo в речи | (г) после ADR-0149 | LLM кода не пишет | #3312 PR-13 |
| R8, S3, P2 | `HALLUCINATED_MIDI_RE` | (б)→(г) | нет поля для нот в схеме | #3312 PR-6/13 |
| R9, S9, P5 | эхо `<system>` + ретрай; отказ в tts_node | (в)/(г) | ретрай — удалить (PR-7); отказ tts_node остаётся с `TEMP(ADR-0148` и счётчиком; удалить при 0 за марафон | PR-7, PR-15 |
| R10, S8, P7 | вызов тула текстом, `_PSEUDO_TOOL_CALL_RE` ×3 слоя | (в) → локализовано | JSON-режим команд: парсер в **одном** месте (`command.parse`) — это и есть адаптер провайдера | PR-6, PR-15 |
| R11, P6 | done-маркеры ×5 | (б)+(г) | нет протокола завершения: команда терминальна | PR-7 |
| R12 | `startswith(("[SYSTEM",…))` | (г) | ретраев с `[CRITICAL]` нет | PR-7 |
| R13, вопрос 28 | `_HISTORY_MARKER_RE`, `_SPEAKER_TAG_RE`, `_META_PREFIX_RE` | (а) | снимок сессии структурой | PR-11 |
| R14 | снятие markdown ×2 | (в) → одно место | только tts_node | PR-3 |
| R15, U7, P10 | выдуманная лирика после музыки, `_LYRICS_KEYWORDS` | (б) | у `request_music` нет текста песни в схеме; `Act` терминален | PR-7 |
| R16 | аргументы вызова в тексте `speak_text` | (б)/(в) | `speak_text` у Личности нет | PR-7 |
| U1 | `media_command_grammar` | образец (а) | остаётся, расширяется | PR-6 |
| U2 | `skill_router` 27 регексов | (а)/(г) | сцена из грамматики/команды, `tools_view` | PR-6 |
| U3, U4 | стоп ×5, громче ×4 | (г) | `knowledge` + `grammar` | PR-1, PR-6 |
| U5 | `MUSIC_GUARD_KEYWORDS`, `TOOL_REQUEST_PATTERNS` | (а) | намерение, распознанное кодом, код и исполняет (Tier-1); иначе гарду не на чём стоять | PR-7 |
| U6 | `_MUSIC_FALLBACK_KEYWORDS` → топ-трек на пустой ответ | (в)→(г) | пустой ответ → честный `Ask` | PR-7 |
| U8, U9 | фразы «новая сессия», да/нет личности | (а) | Tier-1 грамматика | PR-6 |
| S10, S11 | Bug C/B музыкальные ретраи | (а)+(г) | роутер + ADR-0149 события | PR-7 (не вызывается в v2), #3312 |
| S12 | `tool_skipped` ретрай | (а) | `query`-тулы по команде; время/поиск — `Act` | PR-7 |
| S13 | `truncated_args` | (в)→(б) | схемы ≤ 12 тулов, `max_tokens` не режет | PR-6 |
| S14 | silent_response / pseudo-call | (в) | `command.parse`; пусто → `Ask` без ретрая | PR-7 |
| P4, P13, P14, P11, P12 | правила промпта, продублированные кодом | (а)/(б)/(г) | промпт генерируется из `knowledge`; клампы в схеме; правило о порядке — код | PR-15(б) |
| P8, P9 | DJ-правила промпта | (б)/(а) | ADR-0149 | #3312 |
| 1.5 входные фразы | `_MUSIC_STOP_OVERRIDES`, `_should_force_dj_off_for_stop_command`, `_PREFIXES` STT-маркеров | (г)/(а)/(б) | грамматика; STT публикует статус полем (`Utterance.status`) | PR-6, PR-2 |
| 1.5 подставные фразы | «Принял.», «растерялся», «попробую ещё раз», «Секунду, ставлю трек», «Готово, играю.» | (в)/(а)/(г) | `phrases/ru.yaml` по событию | PR-7, PR-8 |
| 1.6 флаги | `skill_tool_narrowing`, `barge_in_policy`, `e2e_session_reset_token`/`e2e_mode` в prod-ноде, `faq_mode_enabled` ×2, мёртвые параметры, два бюджета ретраев | (б)/(г) | срез по возможности; одна политика; e2e через `DialogControl.reset`; один флаг; удалить | PR-6, PR-9, PR-10, PR-4, PR-8 |
| T1 | `IGNORE_STOP_MS:700` | (а) | отмена по `speech_id`/эпохе | PR-3 |
| T2–T4 | грейс эхо 2.5 с, sleep USB, Silero wait | (в) | остаются с `TEMP(ADR-0148` и измерением | PR-0 (метки) |
| T5 | цепочка таймеров приветствия | (а) | `SpeechEvent`/`ProviderState(tts).ok` как событие готовности | PR-8 |
| T6 | `TurnSpeechGate`, фальшивый батч `issue-2874-held-turn-speech` | (г) | решение до генерации, удерживать нечего | PR-8, PR-15 |
| T7–T9, T11 | DJ-таймеры | (а)+(г) | ADR-0149 | #3312 |
| T10 | `MEDIA_ACTION_BACKING_S=120` | (г) | — | PR-6 |
| D1 | music v1/v2 | (в) | #3312 | — |
| D2 | `_utterance_fallback.py` | (г) | `rob_box_core.utterance` | PR-13 |
| D3, D4 | зеркала `TurnState`, ветка `NUDGE` | (г) | — | PR-15 |
| D5 | два владельца DJ-флага | (а) | владелец — плеер; диалог читает | PR-10 (диалог), #3312 (mcp) |
| D6 | два владельца голоса | (а) | `VoiceState` в сессии + `voice_changed` от tts_node | PR-4 |
| D7, D8 | списки тулов ×18, знание ×N | (б)/(г) | `knowledge`, флаги каталога | PR-1, PR-7, #3312 |
| D9 | две launch-конфигурации | (г) | одна (`voice_assistant_headless`) | PR-8 |
| D10–D15 | мёртвые промпты, модули, функции, VAD-прерывание, greeting, action_server | (г) | — | PR-0, PR-9 |
| D16 | мёртвые топики | (а)/(г) | §2.3 | PR-3, PR-8 |
| D17 | призраки `slice_policy` | (г) | — | PR-13 |

Метрика прогресса в каждом PR: `re.compile` в `dialogue_guards.py` (34 → 0 к PR-15), `_retry_used` (10 → 0), `Bug [A-F]` в диалоговых файлах (170 → 0), `#NNNN` в `src/rob_box_dialog` (≤ 20), `TEMP(ADR-0148` в диалоговых файлах (0 → ≤ 5 помеченных страховок → 0).

---

## 13. Какие ADR замещаются; что поправить в CONTEXT.md

| ADR | Статус сейчас (проверено V24 / 04 §1) | Решение |
|---|---|---|
| **0084** TurnGuards bridge | accepted; оркестратор удалён #3327 | **Superseded** этим ADR при принятии: гардов хода нет, есть валидатор команды до исполнения |
| **0021** декомпозиция `dialogue_node` (R1–R5) | proposed с 18.08; узел ×2.4 | **Superseded**: узел не декомпозируется, а заменяется `dialog_node` + `rob_box_dialog`; R5 «issue-ссылка в каждом фиксе» **отменяется** (породил 705 ссылок, 01 §15; противоречит 0148); 0021-r1 (ratchet) остаётся инструментом |
| **0066** `/dialogue/control` pause/resume | без поля статуса | §2–§3 superseded: `DialogControl.srv` с `set_by`, TTL, структурный ack; остальное — в §7.2 |
| **0143** MiniMax игнорирует `tool_choice` → ретраи остаются | принято | Superseded: режим JSON-команд по `ProviderState`; ретраев нет |
| **0129-dj** смена персоны чистит историю и штампует промпт | proposed, ревизия 01.10 | Superseded §2: `Persona` — кадр сцены с `context_strategy`; штампа-негации в промпте нет |
| **0140** «ты диджей X» ретрай `set_dj_mode` | частично заменён #3134 | остаток superseded ADR-0149 + §9.1 |
| **0065** вейк-слова SSoT в коде | accepted | amended: SSoT — `rob_box_dialog.knowledge` (читает `wake_words.yaml`), `dialogue_text.py` → реэкспорт до PR-15 |
| **0102/0103** «Повод», `may_speak` | proposed, внедрено | **остаются**: `OccasionGate` — внутри `address.py`/`respond.py` как решение «можно ли заговорить» для синтетических поводов |
| **0131** `utterance_id` | accepted | остаётся; типизируется в `msg/Utterance` (PR-2) |
| **0083** `build_agent(spec)` | proposed | остаётся: `AgentSpec(personality|operator)` конфигурирует `rob_box_dialog` |
| **0037** слои памяти | proposed | частично superseded §7.4 (турны не персистятся по директиве 02.09; факты — одна таблица) |
| **0055/0128** одна БД / границы E2E | proposed/accepted | 0055 подтверждается (`harness_voice.db`); e2e-изоляция — через `DialogControl.reset` + `e2e_db_path`, флаги `e2e_mode` из prod-ноды уходят (PR-8) |
| **0093** ring неизвестных дикторов | proposed, не внедрён | не трогается; `Session.speaker` оставляет место для `transient_label` |
| **0148** | proposed | родитель; §2.4 «включить `skill_tool_narrowing`» исполняется PR-6 как `tools_view` |
| **0149** | proposed | родитель; интерфейс §9.1 |

Дубли номеров (`0021` ×3, `0129` ×3, `0024`/`0068`, `0026`/`0069`, `0027`/`0070`) — разобрать в PR-0 по ADR-0148 §2.4 (пометить старые копии «см. …»), иначе ссылки этого ADR на 0021/0129 двусмысленны.

**CONTEXT.md:**
- «Ход» (`:109-116`): убрать `TurnGuards`/«бюджет ретраев»; новое определение: «одна попытка понять и исполнить адресованную фразу: команда LLM из закрытого перечня, валидатор, одно действие, фраза из результата; владеет эпохой и дедлайном; ретраев не имеет».
- Добавить: **Сессия** (владелец `dialog_node`, эпоха, стек сцен), **Сцена/кадр** (TTL, `set_by`, что возвращается при pop), **Команда** (`Say | Ask | Act | Pass`), **Результат действия** (`ActionResult`, статус, событие владельца), **Владелец речи** (tts_node, `speech_id`), **Адресация** (вейк | окно | собеседник), **Возможность** (`ProviderState`, `available_tools`).
- «Срез» (`:77`): срез = функция от сессии и возможностей, а не статический список скилла.
- «AgentCore» (`:89`): «тул-цикл до 8 итераций» → «LLM-клиент; цикл ≤ 2 `query`-итераций под управлением `rob_box_dialog`».
- «Пауза» (`:134`): кадр `OperatorHold`, снимает только оператор; пользовательская тишина — отдельный кадр с TTL.
- «Вейк-слово» (`:45`): источник — `rob_box_dialog.knowledge`; адресация шире вейка.

---

## 14. Приёмка числами

Прогоны: (П1) ночной марафон 12 актов `scripts/e2e/run_night_marathon.sh` + `gen_night_marathon.py` на роботе, фразы инъекцией в `/voice/stt/utterance`, `tts_provider=minimax` (память `e2e-auto-tts-picks-unmeasured-yandex`), отдельная БД (ADR-0128); (П2) эталонный набор честности — 60 фраз из issue классов 1–3 (01 §2–§4), `scripts/dialog/honesty_audit.py`; (П3) Telegram-сессия 10 сообщений; (П4) ТАРС 20 операторских команд через шлем/Telegram; (П5) статика/рантайм-аудит `G/L: Architecture Audit`; (П6) `turn_metrics.py` по логам П1–П4. Пороги — мои предложения, не решения Шифу (В5).

| # | Критерий | Порог | Сейчас (01.10) | Как мерить |
|---|---|---|---|---|
| A1 | вызовов LLM на одну адресованную фразу | p50 = 1, p100 ≤ 3; Tier-1 фразы — 0 | до 27 возможных; 8 за ход в #1881 (из материалов) | `turn_metrics.py` по `TurnEvent.llm_calls` (П1, П6) |
| A2 | STT → первый звук робота | Tier-1: p50 ≤ 1.0 с; LLM-ход: p50 ≤ 4 с, p95 ≤ 8 с | 15–50 с в инцидентах (#3153, #2767; из материалов); baseline PR-0 | лог `🎤 STT` → `SpeechEvent.started` |
| A3 | выдуманных успехов на эталонном наборе | 0/60; и 0 за марафон | #2755, #2780, #2949, #3266, #3297 open (из материалов) | `honesty_audit.py`: каждая фраза с ключом `*.done` имеет `ActionEvent done` ≤ 2 с до неё; каждая `rejected` → фраза `*.rejected` |
| A4 | размер хоста | `dialog_node`: WMC ≤ 80, методов ≤ 40, ≤ 600 строк; `dialogue_node.py` удалён (PR-15) | `DialogueNode` WMC 1 103, 210 методов, 9 954 строки (проверено V1) | `radon cc`, `class_budget.py`, `wc -l` |
| A5 | токены контекста на вызов LLM | p50 ≤ 12k, p100 ≤ 20k | ≈ 70k (из материалов 02 §5) | `estimate_tokens` в логе `TurnEvent` (П6) |
| A6 | топики диалога с несколькими писателями (repo-scope) | 0 вне `writers_allowed` | 17 (06 §2, рантайм 30.09) | `L: Architecture Audit` → `runtime-findings.json multiple_writers` (П5) |
| A7 | `ownership.yml` | все топики префиксов `/dialog|/voice|/avatar` имеют `owner`; CI `ownership_check` зелёный | пуст (проверено V22) | `ownership_check.py` (П5) |
| A8 | заплатки | `re.compile` в `dialogue_guards.py` 34 → 0 (файл удалён); `_retry_used` 10 → 0; `Bug [A-F]` 170 → 0 в диалоговых файлах; копии знания: стоп 5→1, громче 4→1, вейк 3→1; `TEMP(ADR-0148` в диалоге ≤ 5 → 0 | 34 / 10 / 170 / 5,4,3 / 0 (проверено V2, V3, V18) | grep в PR-описании |
| A9 | тишина по TTL | «хватит» → речь возвращается через TTL без фраз человека; операторский `resume` не снимает пользовательскую тишину | бессрочно (проверено V15); марафон акт 8: 11 drop (из материалов) | П1 акт 8 + `SessionState.scenes` |
| A10 | утечки между актами | марафон ≥ 11/12 актов зелёных; голос/персона/скилл после `reset` равны дефолту | 1/12 (память `night-marathon-state-leaks-between-acts`) | П1, `SessionState` между актами |
| A11 | честная деградация | при убитом ключе провайдера: ровно 1 фраза деградации, `ProviderState.health` меняется ≤ 10 с, 0 молчаливых подмен голоса | молча (память `voice-stack-degrades-silently`) | сценарий PR-4 на роботе, лог |
| A12 | перебивание | речь останавливается ≤ 300 мс после вейка; 0 действий старой эпохи после перебивания; история содержит только `text[:spoken_chars]` | хвост доигрывает, `IGNORE_STOP_MS` (проверено V16) | П1 + 20 перебиваний, лог `stale_epoch` |
| A13 | служебный текст в речи | 0 вхождений `[`, `<`, `done`, `function_calls`, `Мнение` в `SpeechEvent.started.text` за марафон; `say_truncated` ≤ 2 % | ×193/час в #2558 (из материалов) | grep по логу TTS (П1) |
| A14 | пустой/невалидный ответ | 0 «Принял.»; `understand.invalid` ≤ 10 % ходов; 0 ретраев | 19 пустых за 50 мин (#1253, из материалов) | `TurnEvent.status=invalid` (П6) |
| A15 | адресация | 0 пробуждений на корпусе «работает/робота-диджея» (из #1292, #2971); в окне адресации «громче» принято ≥ 9/10 | ложные wake (из материалов) | unit-корпус + П1 |
| A16 | двойная обработка | «робот, хватит» → ровно 1 `SpeechRequest` | 2 («Хорошо, молчу» + «Останавливаюсь», вывод из кода 02 §3.2) | П1 лог |

Необходимое условие сверх чисел — прослушивание товарищем Шифу живого диалога (10 фраз из эталонного набора) и его вердикт «не врёт, не молчит, не читает мусор».

---

## 15. Что сохранить из старого (переносится, не переписывается)

| Что | Где | Куда |
|---|---|---|
| Допуск реплики, 12 шагов | `core/stt_admission.py:449-840`, `stt_admission_host.py` | как есть; `WakeWordStep` → `address.py`, `SilenceCommandStep`/`MediaCommandStep` → `grammar.py` |
| Грамматика медиа-команд и роутер | `core/media_command_grammar.py`, `media_router.py` | расширяется (U1 — образец) |
| `Utterance`, `Sink`, валидация полей TTS | `rob_box_core/utterance.py:55-220` | полезная нагрузка `SpeechRequest` |
| `utterance_id`, join «кто сказал» | `core/utterance_id.py`, `utterance_speaker.py`, `utterance_binding.py` (ADR-0131) | как есть, типизированные сообщения |
| «Повод» | `core/occasion.py:40-228` | внутри `address.py`/`respond.py` |
| Каталог тулов и генератор схем | `rob_box_core/tool_catalog.py`, `tools/gen_tool_catalog.py` | источник схем для `tools_view` и `command` |
| LLM-клиент, провайдеры, здоровье, ретраи транспорта | `harness/core/agent_core.py` (часть), `harness/providers/*`, `health.py` | `AgentCore` как клиент; `HealthAwareFallbackLLM` под `degrade.py` |
| Швы идентичности/встречи | `harness/identity/base.py`, `harness/encounter/*` | читатель — `Session.speaker` |
| Эпоха сессии | `core/session_epoch.py:56-114` | `Session.epoch` (переносится, DJ-специфика убирается) |
| Арбитр floor | `sup/core/locks.py`, `fsm.py` | без изменений — образец владельца |
| STT-каскад и провайдеры | `stt_node.py:1446`, `stt_fallback.py` | + `ProviderState(stt)` |
| Pregenerate TTS | `scheduler/pregen/*` (ADR-0092) | внутри владельца речи |
| Марафон и инъекция | `scripts/e2e/run_night_marathon.sh`, `gen_night_marathon.py` | П1 |

---

## 16. Риски и откат

| Риск | Смягчение | Откат |
|---|---|---|
| JSON-режим команд у MiniMax даёт невалидные объекты чаще, чем ретраи v1 | A14 измеряет долю `invalid`; при > 10 % — DeepSeek первым для LLM-ходов (В4); парсер берёт первый валидный объект | `dialog_engine: v1` |
| Задержка отмены речи через `interrupt_request` > 300 мс | замер в PR-9; план Б — прямая отмена по эпохе из latched сессии в stt_node | то же |
| Окно адресации без вейка ловит чужую речь | по умолчанию только при подтверждённом собеседнике (биометрия/лицо ≥ порога), окно 8 с, выключается флагом (В2) | выключить окно |
| Строгая терминальность `Act` делает ответ «сухим» (нет живой реплики после действия) | шаблоны с вариантами; опционально `Say`-комментарий после `query`; для `action` — только если Шифу попросит (В6) | — |
| Два контракта речи в переходный период (адаптер в tts_node) | один модуль, удаляется PR-15(а); `seam_allowlist` с датой | — |
| Параллельный эпик #3312 меняет `music.py`/`dj_mode.py` | PR-3…13 не трогают эти файлы; удаление музыкальных гардов — у #3312 | — |
| `develop` force-push (память проекта) | каждый PR от свежего `origin/develop`, merge-base до `gh pr create` | — |
| Гарды CI (`cc_budget`, `class_budget`, `seam`) красные на develop после мержа соседа (память `green-pr-ci-lies-about-guards`) | гонять гарды локально перед пушем | — |
| Директива Шифу о турнах в БД | `turn_log` в памяти; на диск — метрики без текста (В8) | — |
| ТАРС теряет функции при переезде на общий движок | PR-13 — последний функциональный; до него супервизор работает по-старому | флаг на уровне супервизора (`dialog_engine` читает и он) |

---

## 17. Альтернативы

| Вариант | Суть | + | − | Вердикт |
|---|---|---|---|---|
| **А. Продолжать декомпозицию `DialogueNode` по ADR-0021** | выносить методы в `core/*`, держать гарды | без флага и второго узла | с 18.08 узел ×2.4; гарды и ретраи остаются — К1, К2, К4, К6 не закрываются; 73 fix : 24 feat с 01.09 (01 §15) | **отклонено** |
| **Б. Полный агент (ReAct) с лучшей моделью и `tool_choice`** | заменить провайдера, оставить свободный ответ | мало кода | MiniMax без `tool_choice` (ADR-0143), кошелёк общий с музыкой (ADR-0149 В1); фраза об успехе всё равно у LLM — К1 не закрыт | **отклонено** |
| **В. Команда из закрытого перечня + код исполняет + фраза из события** (этот ADR) | Rasa CALM + HA responses + LiveKit `say/StopResponse` + SayCan «can» | закрывает К1–К6 структурно; работает без `tool_choice`; ≤ 3 вызова LLM | речь после действий «шаблонная»; нужен новый пакет и сообщения | **выбран** |
| **Г. Realtime-модель речь-в-речь (OpenAI Realtime-класс)** | убрать STT/TTS, прерывания на сервере | латентность, барж-ин «из коробки» | русский голос робота, персоны, локальная музыка, Pi + ReSpeaker 16 кГц; нет контроля действий; стоимость | **отложено**; паттерны `truncate`/`create_response=false` взяты |
| **Д. Pipecat/LiveKit как рантайм** | заменить ROS-контур фреймворком | готовые turn-стратегии | второй рантайм рядом с ROS2/Zenoh, своя шина без наших владельцев, `#5305`-класс дыр (05 §1) | **отклонено**; берём паттерны (системная полоса прерываний, `cancel_on_interruption`, `context_strategy`) |
| **Е. Отдельный процесс/контейнер для `dialog_node`** | изоляция падений | падение голоса не роняет диалог | ещё одна граница сети между Pi (06 §6 п.10); сейчас всё в одном контейнере | **отложено** до стабилизации v2 |

---

## 18. Вопросы к товарищу Шифу

- **В1. Один канал речи.** (а) Личность говорит только текстом команды `Say`, `speak_text` исчезает (этот ADR); (б) оставить `speak_text` как единственный канал и запретить свободный текст. **Рекомендация: (а)** — работает без `tool_choice`, убирает done-маркеры и 3 слоя парсинга псевдовызовов. Для ТАРС `say` — см. В7.
- **В2. Окно адресации без вейк-слова.** (а) выключено — вейк на каждой фразе, как сейчас; (б) 8 с после ответа робота, только для подтверждённого собеседника (биометрия ≥ 0.72 или свежее лицо ADR-0139); (в) 8 с для любого голоса. **Рекомендация: (б)** за флагом, включать после A15 на корпусе; (в) — риск отвечать на чужую речь (класс 6).
- **В3. TTL пользовательской тишины.** 10 мин / 30 мин / до новой сессии. **Рекомендация: 10 мин**, плюс выход по «говори/отвечай» и по новой сессии; операторский `resume` пользовательскую тишину не снимает.
- **В4. LLM для ходов диалога.** (а) MiniMax первым в JSON-режиме команд (общий кошелёк, ADR-0149 В1); (б) DeepSeek первым (есть `tool_choice`, проба баланса), MiniMax — фолбэк. **Рекомендация: (а) до замера A14 в PR-8; если `invalid` > 10 % — (б).**
- **В5. Пороги приёмки** A1 (≤ 3), A2 (4 с / 8 с), A5 (12k), A10 (11/12), A14 (10 %), `Say ≤ 280` символов, `TURN_DEADLINE_S = 12` — утвердить или изменить.
- **В6. Реплика после действия.** (а) только шаблон из события (сухо, честно); (б) шаблон + короткий `Say`-комментарий LLM вторым вызовом с результатом в контексте (живее, +1 вызов, риск приукрасить). **Рекомендация: (а)** по умолчанию, (б) — флаг, выключен (как `hype_line` в ADR-0149 В2).
- **В7. ТАРС `say` / `speak_text`.** (а) `say` регистрируется как `action.local` с событием `SpeechEvent` и остаётся у ТАРС (оператор просит робота сказать фразу вслух); (б) удалить, ТАРС говорит только `Say` в наушник, озвучка динамиками — через `DialogOutput(sink=speakers)`. **Рекомендация: (а)** — операторский сценарий «скажи гостям …» реален (память `speak-through-robot-needs-ssml`).
- **В8. Текст турнов на диске.** Директива 02.09 запрещает персист турнов. (а) `turn_log` только в памяти, на диск — метрики без текста (этот ADR); (б) разрешить текст турнов в отдельной e2e-БД для приёмки (ADR-0128), в проде — нет. **Рекомендация: (б)** — иначе `honesty_audit.py` работает только по логам контейнера.
- **В9. Срез удаления относительно #3312.** Можно ли в PR-7/PR-15 этого эпика удалять `music_guard.py`/DJ-ветки `dialogue_guards.py`, или ждать PR-13…15 эпика #3312? **Рекомендация: ждать #3312** — v2 их просто не вызывает; удаление одним владельцем, без гонок между сессиями.
- **В10. Telegram.** (а) канал `DialogInput/Output` (этот ADR, PR-12); (б) оставить как есть (пишет в голосовые топики), пометить долгом. **Рекомендация: (а)** — иначе A6 = 0 недостижим и «Клод …» остаётся второй головой.
- **В11. Персист сессии при рестарте контейнера.** (а) сессия в памяти, рестарт = новая сессия (просто, честно); (б) latched-снимок восстанавливается из `/data/dialog_session.json`. **Рекомендация: (а)**; музыка переживает рестарт по ADR-0149 своим снимком.

---

## 19. Что не проверено

- Ничего не запускалось: все задержки, доли и частоты — из тел issue, памяти проекта и материалов 01–06; baseline строится в PR-0 по логам марафона 29→30.09, которых я не открывал.
- Не проверял на роботе задержку маршрута `stt_node → interrupt_request → dialog_node → SpeechCancel → tts_node` (§6.2 допускает план Б).
- Не проверял, какую долю фраз MiniMax отдаёт валидным JSON без `tool_choice` — порог A14 стартовый.
- Не проверял тела ~280 issue, классификацию 01 §1 принимаю как есть; номера open-issue в шапке — по 01 (на 01.10).
- Не проверял паттерны опенсорса по первоисточникам: Pipecat `#5305`, LiveKit `FallbackAdapter`, детали `resume_false_interruption` — в 05 помечены «(поиск)»; в этом ADR от них зависит только форма (§6.2 ложные прерывания отложены).
- Не пересчитывал писателей `next_transition_at` и состояние `rob_box_music` — музыкальный путь не мой; интерфейс §9.1 взят из ADR-0149 §2.3/§5.1 как контракт.
- Число 60 для эталонного набора честности — моё; состав набора собирается в PR-0 из тел issue классов 1–3 и утверждается Шифу вместе с порогами (В5).
- Объём `rob_box_dialog_msgs` (§10.1) — предложение; полевой состав уточняется в PR-2 по фактическим полям JSON-сообщений v1 (`02 §2`), которые я читал только по материалам, а не по каждому `json.dumps`.
