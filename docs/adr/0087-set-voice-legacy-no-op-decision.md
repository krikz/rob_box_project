# ADR-0087: AV-28 `set_voice` legacy no-op — удалить или оставить заглушку под `/dialogue/control`

| Поле | Значение |
|---|---|
| Статус | **Proposed** (2026-09-09), диспатч из архитектурного обзора (retrospect-09.09) |
| Дата | 2026-09-09 |
| Автор | architect (Hermes Agent), карточка `t_b602b329` (issue #2267) |
| Контекст | После PR #2255 (voice-vr 21, whitelist consolidation, удаление `voice_preset`/`voice_output_language` параметров из `dialogue_node`) путь `set_voice` (mode=style) из шлема **честно no-op**: ws_server всё ещё принимает чип, `Bridge.set_voice_preset/language` публикуют в `/avatar/set_voice_preset|language`, супервизор их валидирует по whitelist, отвечает ack — и больше ничего. Параллельно живёт **работающий** канал `voice_pipeline` (тот же чип шлёт `sendVoicePipeline()` сразу после `sendStyleChange()`), который и делает реальный эффект через `_on_grip_voice_pipeline → self._pipeline_preset`. На каждый клик чипа уходит **две** команды: одна бесполезная (`set_voice` legacy), одна рабочая (`voice_pipeline`). ADR-0080 §1.6 явно документировал «осознанная заглушка под будущее расширение `/dialogue/control`». С тех пор ни одной карточки под `/dialogue/control` не появилось. |
| Затрагивает | `src/rob_box_quest/webxr_client/src/main.ts` (`sendStyleChange`, `sendVoicePipeline`, обработчики `preset`/`lang`/`voice_mode`), `src/rob_box_quest/rob_box_quest/server/ws_server.py` (`_json_cmd_set_voice`, `_json_cmd_set_voice_style`, `_json_cmd_set_voice_provider`, `_json_cmd_voice_pipeline`, table-dispatcher), `src/rob_box_quest/rob_box_quest/server/ws_server.py` (тестовый mock `Bridge.set_voice_preset/language`), `src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py` (`_on_set_voice_preset`, `_on_set_voice_language`, `_apply_voice_preset`, `_apply_voice_language`, `_AV28_PRESET_IDS`/`_AV28_LANGUAGES`), `src/rob_box_supervisor/test/unit/test_supervisor_node.py` (тесты на `_apply_voice_*` и `Bridge.set_voice_preset/language` стабы) |
| Родители | ADR-0080 §1.6 (мотивация: supervisor не пишет в чужие ROS-параметры), ADR-0085 (TARS2 metrics — пример «удалить, а не оставлять заглушку» для аналогичного кейса), ADR-0018 (честный FAIL — формализация «написано но не подключено»), ADR-0014 (процесс закрытия), ADR-AF-0013 (incremental delivery — маленький PR лучше большого) |
| Связанные | PR #2255 (закрыл foreign-param writes в супервизоре — создал этот no-op), issue #2138 (предыдущая путаница между `set_voice` picker и `set_voice` style, теперь уже корректно разведено), PR #2219 (voice-vr 10, table-dispatcher с терминальной UNKNOWN_COMMAND), ADR-0086 (EventBus/ReflexLayer: «удалить, не оставлять мёртвый код» — тот же класс решения) |

> **TL;DR.** Legacy `set_voice` (mode=style) сегодня **честно мёртв** на стороне
> супервизора (whitelist + log + ack, без побочного эффекта), при этом на
> стороне клиента по-прежнему шлётся на каждый выбор чипа стиля/языка.
> Рекомендация архитектора: **удалить** весь AV-28-ветви `set_voice` cmd-пути
> из клиента и сервера (вариант (a)), оставив только `voice_pipeline` (этот
> канал уже работает и покрывает тот же сценарий). Заглушка «на будущее
> `/dialogue/control`» ADR-0080 не была реализована ни одной карточкой за
> ~1 неделю; по правилам ADR-0086 / ADR-0018 мёртвый код без владельца
> с большей вероятностью вернётся «честным no-op» в проде, чем будет
> подключён.

---

## 1. Контекст: что проверено по коду

Все ссылки на develop @ `e635a8c9`. Метод — чтение исходников + `grep` по
сигнатурам, без выполнения (юнит-тесты ниже приложены).

### 1.1 Топология сегодня: три канала под одной панелью

Один клик по чипу стиля/языка в шлеме (`onPipelineAction` → case `"preset"` /
`"lang"`, `main.ts:510-539`) выполняет **два** вызова и **никакой третий**:

| # | Канал | Функция | Что делает |
|---|---|---|---|
| 1 | `set_voice` (legacy AV-28) | `sendStyleChange(preset?, language?)` (`main.ts:305-316`) | Шлёт JSON_CMD `set_voice` с `voice_id:""` и `preset`/`language` из снимка панели |
| 2 | `voice_pipeline` (рабочий AV-28, шаг 4б) | `sendVoicePipeline()` (`main.ts:336-358`) | Шлёт JSON_CMD `voice_pipeline` с `llm_enabled`/`preset`/`language` |

Канал 1 проходит через `ws_server._json_cmd_set_voice` (`ws_server.py:2778-2809`)
→ `_json_cmd_set_voice_style` (`ws_server.py:2812-2832`) → `bridge.set_voice_preset(...)`/
`set_voice_language(...)` (`ws_server.py:484-505`, `ws_server.py:709-717`)
→ publish в ROS `/avatar/set_voice_preset|language` → супервизор
`_on_set_voice_preset|language` (`supervisor_node.py:1458-1523`) → `_apply_voice_preset|language`
→ whitelist + log + ack.

Канал 2 проходит через `ws_server._json_cmd_voice_pipeline` (`ws_server.py:2857-2868`)
→ `bridge.publish_voice_pipeline(...)` (`ws_server.py:507-519`) → publish в
`/avatar/voice_pipeline` → супервизор `_on_grip_voice_pipeline` (`supervisor_node.py:2681-2719`)
→ `self._pipeline_preset` (`supervisor_node.py:605`), `_pipeline_language`,
`_pipeline_llm_enabled` → `grip_pipeline` (`supervisor_node.py:2774-2781`,
`:2781`) применяет к LLM-формализации грипа.

**Канал 1 ничего не меняет в поведении грипа** (документировано явно,
`supervisor_node.py:1465-1494`, `ws_server.py:476-505`). Канал 2 — единственный,
через который чип «применяется».

### 1.2 Не нарушение ADR, но интерфейс шире реализации

ADR-0080 §1.6 фиксировал проблему «супервизор перестаёт писать чужие
ROS-параметры — этот путь уже honest no-op, но не удалён». Этот ADR
формализовал намерение «оставить заглушку, пока не появится карточка под
`/dialogue/control`» — §2.7 «до explicit-контракта».

Прошло ~7 дней (`e635a8c9` от 2026-09-09, ADR-0080 от 2026-09-08). В этом окне:

- 0 issue и 0 kanban-карточек упоминают `/dialogue/control`,
- `git log origin/develop --grep='dialogue/control'` — 0 коммитов после ADR-0080,
- `scripts/agent_flow/` не добавил ни gate, ни lint на наличие мёртвых
  Bridge-методов (поиск `Bridge` в `scripts/lint/`: 0 совпадений).

Это класс решений «оставить заглушку, чтобы не плодить регрессий», и ADR-0086
(«EventBus/ReflexLayer: удалить, а не подключать») за неделю до этого
сформулировал правило в явном виде:

> Если код не имеет владельца, активной карточки и проверяющего теста на
> продовом пути — он **мёртвый**, и через несколько месяцев возрождается
> в виде «написано, но не подключено» (ADR-0018). Лучше удалить и
> пере-создать при появлении владельца.

### 1.3 Лишний wire-трафик

На каждый клик чипа уходит одна лишняя команда по WebSocket и одна
лишняя ROS-публикация. На headless-харнессе в e2e это малозаметно
(миллисекунды), но ADR-AF-0021 (CC-бюджет) считает **каждое** ветвление
в `dialogue_node` и `_on_json_cmd` — а `_json_cmd_set_voice_style` это
именно то место, где чип-стиль проходит через 4 guard'а (`mode` валидация,
rate-limit, preset/language validate, ack), ни один из которых не
защищает продовое поведение, потому что продового поведения нет.

### 1.4 Тестовая сеть держит канал живым

Тесты на ws_server и supervisor **существуют и зелёные** на legacy-пути:
- `test_ws_server_voice.py:590-626` (`test_set_voice_routes_preset_and_language_to_bridge`,
  `test_set_voice_preset_only_does_not_touch_language`) — проверяют,
  что `bridge.set_voice_preset_calls`/`set_voice_language_calls` инкрементнулись.
- `test_supervisor_node.py:320-339, 395-465` — проверяют `_apply_voice_preset/language`
  whitelist + mode='active' → `applied=True`.
- `test_supervisor_node.py:423-436` — feed через `_on_set_voice_preset|language` моки.

Все эти тесты **закрывают контракт «функция вызвана, ack ушёл»**, но
**ни один** не проверяет, что после apply меняется **что-то реальное**
(например, что `_pipeline_preset` изменился, или что новый стиль речи
влияет на `grip_pipeline`). Это и есть «написано, но не подключено»
ADR-0018 — тесты зелёные, прод не меняется.

---

## 2. Принятое решение: **вариант (a) — удалить**

### 2.1 Что удаляется

| Слой | Файл | Что удалить / изменить |
|---|---|---|
| Клиент | `webxr_client/src/main.ts:305-316` | Удалить `sendStyleChange(...)` целиком. |
| Клиент | `webxr_client/src/main.ts:510-524` (case `"preset"`) | Убрать `try { sendStyleChange(action.preset); } ... sendVoicePipeline()`. Оставить `modeManager.setCurrentPreset(...)`, `bridge.voicePipeline.setCurrentPreset(...)`, `ensureLlmFormalize()`, `sendVoicePipeline()`. |
| Клиент | `webxr_client/src/main.ts:525-539` (case `"lang"`) | Симметрично. |
| Клиент | `webxr_client/src/main.ts:286-304` | Удалить комментарий-предупреждение про #2138 (issue закрыт, ловушки больше нет). |
| Сервер | `ws_server.py:2778-2809` (`_json_cmd_set_voice`) | Убрать `is_style_request`/`mode` логику и второй путь. Оставить только `_json_cmd_set_voice_provider` (это **рабочий** канал AV-27 picker). Альтернатива: вообще переименовать cmd в `set_voice_provider`, но это уже отдельная карточка. |
| Сервер | `ws_server.py:2812-2832` (`_json_cmd_set_voice_style`) | Удалить целиком. |
| Сервер | `ws_server.py:476-505` (`Bridge.set_voice_preset`, `set_voice_language`) | Удалить методы и большой docstring. |
| Сервер | `ws_server.py:709-717` (`NoOpBridge.set_voice_preset`, `set_voice_language`) | Удалить методы. |
| Сервер | `ws_server.py:2896-2912` (table-dispatcher, mapping `set_voice`) | Оставить один путь: `set_voice → _json_cmd_set_voice_provider`. |
| Супервизор | `supervisor_node.py:1455-1523` (`_AV28_*`, `_on_set_voice_preset`, `_on_set_voice_language`, `_apply_voice_preset`, `_apply_voice_language`) | Удалить целиком. |
| Супервизор | `supervisor_node.py:1444-1454` (блок-комментарий «голос и управление из шлема») | Сократить до одной строки «AV-28 legacy path удалён; см. ADR-0087». |
| Тесты | `test_ws_server_voice.py:43-135` (mock Bridge с `set_voice_preset_calls`/`set_voice_language_calls`) | Удалить поля и методы. |
| Тесты | `test_ws_server_voice.py:574-624` (`test_set_voice_routes_preset_and_language_to_bridge`, `test_set_voice_preset_only_does_not_touch_language`) | Удалить. |
| Тесты | `test_supervisor_node.py:299-339, 393-466` (тесты на `_on_set_voice_preset|language` и `_apply_voice_preset|language`) | Удалить; `_apply_set_voice` (если это другой путь, см. тест 467+) — оставить. |

**Что НЕ удаляется** (живой код):

- `ws_server._json_cmd_set_voice_provider` (`ws_server.py:2835-2854`) — рабочий
  путь смены голоса из TTS picker.
- `ws_server._json_cmd_voice_pipeline` (`ws_server.py:2857-2868`) — рабочий
  путь смены стиля/языка грипа.
- `Bridge.set_voice(voice_id, preset)` (`ws_server.py:670-703`) — рабочий
  путь picker; пишет в `/voice/tts/set_voice` через супервизор.
- `Bridge.publish_voice_pipeline` (`ws_server.py:507-519`) — рабочий путь грипа.
- `supervisor._on_grip_voice_pipeline` (`supervisor_node.py:2681-2719`),
  `self._pipeline_preset/language/llm_enabled`, `grip_pipeline`.

### 2.2 Альтернатива — вариант (b), **схлопнуть**

Объединить чип-стиль и TTS picker в один cmd `voice_pipeline`, удалив
`set_voice` для style-режима, но **оставив** сам `set_voice` cmd в
протоколе как зарезервированный на будущее (через терминальную
`UNKNOWN_COMMAND` ветку, ADR-0080 §1.2 уже добавил её в `_on_json_cmd`).
Тогда:

- Сервер — `ws_server.py` сохраняет `set_voice` в JSON_CMD dispatcher, но
  не имеет обработчика; клиент его **никогда** не шлёт → живой мёртвый
  слот в протоколе.
- Плюсы: сохраняет имя `set_voice` для будущего `/dialogue/control`
  (которого пока нет).
- Минусы: класс мёртвого кода, ADR-0086 явно предостерегает.

Рекомендация архитектора — (a). Если Шифу захочет (b), это допустимо,
но тогда добавить явный ADR-link на эту карточку с пометкой
«`set_voice` зарезервирован под `/dialogue/control`, issue #TBD».

### 2.3 Почему (a), а не (b) — trade-off

| Критерий | (a) удалить | (b) схлопнуть |
|---|---|---|
| Размер diff | ~250 строк удалено, 0 добавлено | ~30 строк удалено, 0 добавлено |
| CC-бюджет (ADR-0021) | `ws_server` `dialogue_node` −1 ветвление в каждом | без изменений |
| ADR-0018 «написано, но не подключено» | больше не применимо | сохраняется в виде «зарезервированный cmd» |
| ADR-0086 «удалить, если нет владельца» | выполнено | не выполнено |
| Цена возврата, когда появится `/dialogue/control` | ~250 строк восстановить из git history | 0 |
| Цена оставления в проде как мёртвого кода | 0 | +1 «ловушка» в протоколе для будущего разработчика |

ADR-0086 уже закрепил «удалить» как правило. ADR-0085 (TARS2 metrics
client-side) — аналогичный кейс (метрики писались сервером, переехали
на клиент, серверная запись удалена) — прошёл по варианту (a) и Шифу
принял. Стоимость возврата низкая: `git log -p origin/develop -- ws_server.py`
восстановит всё за 5 минут.

### 2.4 Когда НЕ применять (a)

- В течение 1-2 карточек появилась **конкретная** задача под
  `/dialogue/control` с владельцем и acceptance-критериями. В этом
  случае вариант (b) оправдан, а карточка под `/dialogue/control`
  примет `set_voice`-cmd как контракт. Сегодня таких карточек нет.

---

## 3. Последствия

### 3.1 Положительные

- Чип стиля/языка в шлеме шлёт **одну** команду (`voice_pipeline`), а не две.
- WS-server и супервизор теряют 2 обработчика + 4 guard'а в каждом;
  CC `_json_cmd_set_voice` снижается с 13 до ≤8 (вариант (a)).
- Тестовая сеть перестаёт «защищать» мёртвый контракт. Тесты на
  `voice_pipeline` (которые зелёные на рабочем пути) уже покрывают
  сценарий чипа.
- ADR-0080 §1.6 остаётся валидным в части «supervisor не пишет в чужие
  параметры»; §2.7 «explicit contract» переформулируется: вместо
  «до `/dialogue/control`» — «если появится, пересоздать cmd с явным
  контрактом через новый ADR».

### 3.2 Отрицательные

- `git blame` для `set_voice` cmd теперь указывает только на AV-27 picker
  (бывшая `_json_cmd_set_voice_provider`). Если будущему разработчику
  понадобится стиль через тот же cmd — будет сюрприз. Контрмера:
  ADR-link в шапке файла + явная проверка в `voice_pipeline` ack (он
  уже сигналит, что пресет применился).

### 3.3 Нейтральные

- Лишний wire-трафик (миллисекунды) — был виден только в headless-харнессе.
- DOC-комментарии на удаляемых функциях ценны как знание — переносятся
  в ADR-0080 §2.7 (обновлённый) и в ADR-0087 (этот).

---

## 4. План реализации варианта (a)

Все правки идут в одной ветке `z-{agent}/2267-av-28-set-voice-legacy-no-op`,
одним PR. Порядок снизу-вверх по зависимостям:

1. **Супервизор**: удалить `_AV28_PRESET_IDS`, `_AV28_LANGUAGES`,
   `_on_set_voice_preset`, `_on_set_voice_language`, `_apply_voice_preset`,
   `_apply_voice_language` и подписки на `/avatar/set_voice_preset|language`.
   Запустить `pytest src/rob_box_supervisor/test/unit/test_supervisor_node.py -v`.
2. **Сервер**: удалить `_json_cmd_set_voice_style`, `Bridge.set_voice_preset`,
   `Bridge.set_voice_language`, `NoOpBridge.set_voice_preset/language`,
   сократить `_json_cmd_set_voice` до только provider-ветки.
   Запустить `pytest src/rob_box_quest/test/unit/server/test_ws_server_voice.py -v`.
3. **Тесты**: удалить тестовые случаи, перечисленные в §2.1, и
   `set_voice_preset_calls`/`set_voice_language_calls` поля mock'а.
   Перезапустить тесты — должны быть зелёные.
4. **Клиент**: удалить `sendStyleChange`, убрать его вызовы из
   `case "preset"` / `case "lang"`. Сократить комментарий про #2138.
   Запустить `cd src/rob_box_quest/webxr_client && npx vitest run tests/voice_pipeline_panel.test.ts tests/tts_picker_*.test.ts tests/mode_manager.test.ts`.
5. **Линти**: запустить `bash scripts/lint/check_cc_budget.sh` и
   `bash scripts/lint/check_grep_invariants.sh` (если существуют) —
   убедиться, что `_json_cmd_set_voice` вписался в CC≤15.
6. **CI**: запустить `gh pr checks <N>` (после push), все checks зелёные.
7. **Issue**: добавить ссылку на ADR-0087 в issue #2267 (эта карточка
   закрывается после merge PR).

### 4.1 Чеклист DoD (для исполнителя)

- [ ] `set_voice` cmd в ws_server имеет только provider-ветку (стиль удалён).
- [ ] Чип в шлеме шлёт **только** `voice_pipeline`, не `set_voice`.
- [ ] Тесты `test_ws_server_voice.py` и `test_supervisor_node.py` зелёные
      без legacy-кейсов.
- [ ] CC-бюджет не превышен (`scripts/lint/check_cc_budget.sh` PASS).
- [ ] CI на PR зелёный.
- [ ] ADR-0080 §2.7 обновлён (одна строка вместо блока).
- [ ] e2e: голосовые кейсы `voice_pipeline_*` зелёные (они и до этого
      зелёные, регрессии быть не должно).
- [ ] `gh pr view <N> --json files --jq '.files | length'` ≤ 12 (по
      правилам AF-0013).

### 4.2 Что делать НЕ надо

- Не менять контракт AV-27 picker (`Bridge.set_voice` через
  `_json_cmd_set_voice_provider`) — это **работающий** путь.
- Не трогать `/voice/tts/set_voice` (отдельный топик супервизора для
  picker'а, см. ADR-0080 §2.7 и тест `test_apply_set_voice_publishes_to_tts_set_voice_topic`).
- Не удалять `voice_pipeline` cmd, `Bridge.publish_voice_pipeline`,
  `_on_grip_voice_pipeline`, `self._pipeline_preset/language/llm_enabled`,
  `grip_pipeline` — это **рабочий** путь.
- Не переименовывать `set_voice` cmd в `set_voice_provider` — это
  ломает протокол для всех живых клиентов (TARS panel) и относится
  к отдельной карточке, если нужна.

---

## 5. Открытые вопросы (для Шифу)

Перед merge Шифу подтверждает одно из:

1. **(a) удалить** — план §4 без изменений.
2. **(b) схлопнуть** — удалить `sendStyleChange` и
   `_json_cmd_set_voice_style`, **оставить** пустой `set_voice`-слот в
   JSON_CMD dispatcher для будущего `/dialogue/control`. В этом случае:
   - в `ws_server.py` добавить один комментарий «`set_voice` зарезервирован
     под `/dialogue/control`, см. ADR-0087»;
   - открыть issue `#TBD` под этот контракт (если Шифу захочет —
     это требование варианта (b));
   - добавить `set_voice` в `KNOWN_BLOCKER_SIGNATURES` e2e-process, чтобы
     не было ложного «меру работает» в smoke.
3. **(c) отложить** — не закрывать карточку #2267, поставить `blocked`
   с причиной «появился конкретный план под `/dialogue/control`», перевести
   карточку в `ready` через N дней. **Не рекомендую** — ADR-0086 уже
   показал, что отложенный мёртвый код возвращается «no-op без владельца».

---

## 6. Связанные источники

- `git log --grep='voice-vr 21' origin/develop` — PR #2255
  (whitelist consolidation + честный no-op).
- `grep -n "set_voice_preset\|set_voice_language" src/rob_box_quest/rob_box_quest/server/ws_server.py src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py` — все
  точки удаления.
- `git diff origin/develop...<branch> --stat` после применения (a) —
  ожидаемо `~250 строк −N строк` без новых файлов.
- ADR-0086 (EventBus/ReflexLayer: удалить) — аналогичный класс решения,
  Шифу принял.
- ADR-0085 (TARS2 metrics client-side) — аналогичный класс (миграция
  → удаление), Шифу принял.
- ADR-AF-0021 (CC-бюджет) — `_json_cmd_set_voice_style` ложится под
  уменьшение CC.
- ADR-0080 §1.6, §2.7 — исходное обоснование no-op и рекомендация
  «до explicit-контракта» (контракт не появился, ADR-0087 закрывает
  это).