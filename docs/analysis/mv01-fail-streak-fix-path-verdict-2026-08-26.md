# MV01 Fail-Streak: архитектурный вердикт по fix-path (2026-08-26)

| Поле | Значение |
|------|----------|
| Автор | backend (skill `architecture-doc-review`) |
| Цель | Архитектурный обзор трёх fix-path'ов (Fix #1 / Fix #2 / Fix #3) для e2e voice fail-streak round-227..231 на сценарии `mv01_set_voice_alena` после deploy #1568 (commit ba7715e8, 25.08 12:38). Выбрать рекомендуемый path и оформить его в PR. **НЕ править e2e-acceptance** (корректен). |
| Связано | issue #1219 (multi-voice canon), PR #1568 / ba7715e8 (TTS state-aware STOP), PR #1636 / aa1612fe (repro-тест, MERGED 25.08 17:40), PR #1565 (music mutex, OPEN stale с 23.08), PR #1618 / 86291372 (voice_input_mode param, ADR-0027 §3.4), ADR-0027 (systemic wake-gate / Meta Quest), t_f29f15bc (research), t_5660d153 (retro done) |
| Дата проверки | 2026-08-26 (UTC+2) |
| Worktree | `z-backend/t_5a7b6fe3-mv01-fix-path-verdict` (develop @ 65b471d6) |
| **Вердикт** | **REQUEST_CHANGES** (см. §9) |

## 1. Резюме (TL;DR)

Fail-streak **не вызван регрессией в коде после #1568** — он вызван **семантическим
несоответствием между e2e acceptance и production-конфигом**: acceptance
mv01 требует, чтобы `set_voice` РЕАЛЬНО сменил голос (`voice_changed: true`),
но на роботе TTS-провайдер = `minimax`, а запрашиваемое имя `alena` принадлежит
yandex-каталогу. `SetVoiceTool` корректно возвращает `voice_unavailable`
(см. `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py:988-1004`),
а `voice_unavailable` НЕ публикует в `/voice/tts/current_voice` — отсюда
`voice_changed=false` в логах e2e.

**Корневая причина — конфигурационная, а не кодовая.** Все три fix-path'а
лечат симптом, но с разной ценой и разной долгосрочной устойчивостью:

| Path | Цена | Долгосрочная устойчивость | Рекомендация |
|---|---|---|---|
| Fix #1 (prompt rule) | минимальная, чисто текст + 1 unit-тест | НИЗКАЯ — LLM всё равно будет галлюцинировать «Алёну» при смене промпта / модели | **Отклонить как самостоятельный fix**; оставить как guardrail |
| Fix #2 (provider switch на yandex) | однострочная правка YAML, devops-deploy | ВЫСОКАЯ — это ВОССТАНАВЛИВАЕТ первоначальный (pre-#1568) контракт, в котором acceptance писался | **РЕКОМЕНДУЕТСЯ как primary fix** (revert конфига) |
| Fix #3 (ADR §3.4 voice_input_mode) | уже сделано (PR #1618), но не решает наш баг — `voice_input_mode` это про source входа, а не про выбор голоса | нулевая для нашего кейса | **Отклонить как ложную атрибуцию**; ADR-0027 §3.4 не нуждается в апдейте под этот bug |

Дополнительно: требуется `kanban_create` для sub-карточки на **rebase PR #1565**
(он open с 23.08, stale-branch guard может его заблокировать) — это out-of-scope
для архитектурного verdict'а, но указано в acceptance карточки t_5a7b6fe3.

## 2. Верифицированные факты (file:line citations)

Все цитаты — на develop @ 65b471d6. Проверены grep'ом в worktree.

### 2.1 Конфиг и реальный провайдер

- `docker/vision/config/voice_assistant/tts_node.yaml:10` — `provider: minimax`
  (это YAML, который монтируется на роботе в `/config/voice_assistant/tts_node.yaml`
  и читается `voice_assistant_headless.launch.py:51, 119`).
- `docker/vision/config/voice_assistant/voice_assistant_headless.launch.py:114-124`
  — tts_node грузит YAML **БЕЗ launch-override provider** (в отличие от
  `src/rob_box_voice/launch/voice_assistant.launch.py:150`, где есть
  `LaunchConfiguration('provider')`). ⇒ **реальный провайдер на роботе =
  то, что в смонтированном YAML = `minimax`**.
- `.env.example:84` — дефолтный провайдер проекта = **yandex**;
  `provider=minimax` (ROS-параметр) — opt-in.
- `src/rob_box_voice/rob_box_voice/tts_node.py:385` — `declare_parameter("provider", "minimax")`
  совпадает с YAML по умолчанию. Нет чтения `provider` из ENV (только
  `minimax_api_key` / `minimax_group_id` через ENV fallback, см.
  `tts_node.py:29-31` комментарий).

⇒ На роботе сейчас `provider=minimax`. Чтобы переключить — нужно править
либо YAML, либо добавлять launch-arg в headless launch (которого нет).

### 2.2 Реестр голосов (источник истины)

- `src/rob_box_voice/rob_box_voice/tts_voice_registry.py:41-44` —
  `yandex` = `anton, alena, filipp, jane, omazh, zahar, ermil, madirus, arina, kostya, rush`.
- `src/rob_box_voice/rob_box_voice/tts_voice_registry.py:45-53` —
  `minimax` = `Russian_ReliableMan, Russian_HandsomeChildhoodFriend,
  Russian_AttractiveGuy, Russian_Bad-temperedBoy, Russian_BrightHeroine,
  Russian_AmbitiousWoman, Russian_CrazyQueen, Russian_PessimisticGirl,
  male-qn-qingse, female-shaonv`.
- `tts_voice_registry.py:62-66` — DEFAULT_VOICES: `yandex=anton`, `minimax=male-qn-qingse`.

⇒ **alena — yandex-only**, не существует в minimax. `SetVoiceTool` валидирует
против `voices_for(provider)` (см. `dialogue.py:988`).

### 2.3 SetVoiceTool — корректное поведение

- `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py:988-1004`:
  ```python
  known = voices_for(provider)
  if known and voice_clean not in known:
      return MCPToolResult(
          success=False,
          data={"error": "voice_unavailable", ...},
          message=f"Голос '{voice_clean}' недоступен у провайдера '{provider}'..."
      )
  ```
- `dialogue.py:1006-1010` — лог `[set_voice] voice='...' provider=...` пишется
  ТОЛЬКО при успешном выполнении.
- `dialogue.py:1012-1017` — публикация в `/voice/tts/current_voice` ТОЛЬКО
  при успешном выполнении.

⇒ На `voice_unavailable`: success=False, лога нет, топика нет, **voice не
меняется**, e2e-скрипт (см. `e2e_voice_test.sh:1115-1129` regex
`\[set_voice\] voice='...'`) находит 0 совпадений → `voice_changed=false`
→ acceptance FAIL. **Это by design**, не bug.

### 2.4 Acceptance в voice_core_suite_v1.json

- `.github/e2e/scenarios/voice_core_suite_v1.json:31-45`:
  - `label: mv01_set_voice_alena`
  - `text: "Робот, говори голосом Алены"`
  - `acceptance.expected_tool_calls: ["set_voice"]`
  - `acceptance.voice_changed: true`
  - `_comment`: «Не валидируем КОНКРЕТНЫЙ голос — проверяем что set_voice
    РЕАЛЬНО сменил голос с дефолта (voice != default)».

⇒ Acceptance **корректен**: он не требует именно `alena`. Требует, чтобы
`set_voice` вызвался и сменил голос с дефолта (любой валидный для текущего
провайдера). Acceptance **писался в эпоху yandex-дефолта**, поэтому
выбор имени «Алены» был естественен для LLM.

### 2.5 Repro-тест (PR #1636, MERGED)

`src/rob_box_mcp_tools/test/test_tools/test_mv01_set_voice_alena_repro.py` —
4 теста, **все 4 PASSED на develop @ 65b471d6** (запущены
2026-08-26 02:30 локально):
- `test_set_voice_alena_rejected_on_minimax_provider` — подтверждает §2.3.
- `test_set_voice_alena_lacks_log_for_voice_changed_check` — объясняет
  механизм e2e fail (regex не находит log).
- `test_speak_text_alena_silently_falls_back_on_minimax` — гипотеза #3
  подтверждена: speak_text(voice="alena") МОЛЧА фоллбечит на default
  (male-qn-qingse), `voice_fell_back=True`, `voice_used="male-qn-qingse"`.
- `test_set_voice_minimax_voice_succeeds_and_logs` — контр-тест:
  `set_voice("female-shaonv")` работает, лог пишется. ⇒ Если LLM выберет
  minimax-валидный голос (например, `female-shaonv` по аналогии с «Алёна»),
  acceptance пройдёт.

⇒ **Гипотезы карточки #2/#3 полностью подтверждены** живым кодом.

### 2.6 Master prompt — текущее состояние

`src/rob_box_voice/prompts/master_prompt_compact.txt:79-108` — RULE #VOICE:
- Учит вызывать `set_voice(voice="...")` ПЕРЕД `speak_text`.
- Учит брать имена ТОЛЬКО из `[TTS] voices: ...` (строка 87-89).
- Содержит gender-mapping на **minimax** голоса (`Russian_BrightHeroine`,
  `Russian_ReliableMan` и т.д., строки 92-97).
- Объясняет `voice_unavailable` fallback (строки 105-108): «pick a
  different voice from the SAME `[TTS] voices: ...` list».
- Явно запрещает выдумывать «alena, tatjana» (строки 101-102).

⇒ RULE #VOICE **уже говорит LLM не выдумывать alena**. Но LLM
(MiniMax-M3) всё равно **галлюцинирует «Алёну»**, потому что в обучающих
данных «alena» — естественное имя женского голоса для русскоязычного TTS.
Это типичный случай «prompt rule vs training data bias» — нельзя гарантировать
compliance без структурного ограничения.

### 2.7 PR #1565 (music mutex) — stale?

`gh pr view 1565 --json state,mergeable,statusCheckRollup`:
- state: OPEN
- mergeable: MERGEABLE
- последний update: 2026-08-25 21:29
- CI: все checks зелёные на 2026-08-24 14:58

⇒ PR #1565 **технически не stale**, но `z-{agent}/1561-...` head-refName
паттерн указывает на устаревший `agent-flow` naming (после введения
`t_<id>-<slug>` для worker-веток). Если за этим PR не стоит активный
воркер — он может застрять в merge-gate после очередного stale-scan.
**Это вне scope архитектурного verdict'а** — должно идти отдельной
sub-карточкой (см. §9 «Что не входит в verdict»).

### 2.8 ADR-0027 и `voice_input_mode` — путаница в карточке

- `docs/adr/0027-meta-quest-ar-control.md:208-212` — `voice_mode` UI-кнопка
  маппится на параметр `dialogue_node` `voice_input_mode`.
- `voice_input_mode ∈ {respeaker, quest_passthrough, quest_ttts, quest_stt,
  quest_llm_formalize}` — это **SOURCE голосового входа** (ReSpeaker / Meta
  Quest passthrough / TTTS / STT / LLM formalize).
- `src/rob_box_voice/rob_box_voice/dialogue_node.py:817` —
  `self.declare_parameter("voice_input_mode", "respeaker")`.
- `src/rob_box_voice/test/unit/node/test_voice_input_mode_param.py` —
  regression test (PASSED).

⇒ `voice_input_mode` **НЕ имеет отношения к выбору голоса TTS**. Карточка
`t_5a7b6fe3` ошибочно атрибутирует Fix #3 этому ADR. **Fix #3 как
«структурное решение через voice_input_mode» не существует.** Если
автор имел в виду «расширить ADR-0027 параграфом про фильтрацию списка
голосов по провайдеру», то это новая фича, не реализованная в PR #1618.

## 3. BLOCKING

*Нет BLOCKING находок.* Acceptance корректен (§2.4), код корректен (§2.3),
тесты репро есть (§2.5). Проблема — конфигурационная (§2.1).

## 4. WARNINGS

### W1. Fix #2 (provider switch) сформулирован неточно в карточке

Карточка пишет: «`TTS_PROVIDER=yandex` → тогда alena/anton/zahar снова
доступны». **Такой ENV-переменной не существует.** Провайдер берётся
ТОЛЬКО из ROS-параметра `provider` (§2.1):
- `tts_node.yaml: provider: <value>` — но это YAML на хосте (build-time).
- `voice_assistant_headless.launch.py:119` — НЕ override-ит provider.

Реальные способы переключения:
1. **Правка `docker/vision/config/voice_assistant/tts_node.yaml:10`**
   `provider: minimax` → `provider: yandex`, потом devops-deploy
   (re-mount YAML). Требует YANDEX_API_KEY в ENV на роботе.
2. **Добавить launch-override в headless launch.py**, аналогично
   `voice_assistant.launch.py:142-155`, и перезапустить контейнер с
   `-p provider:=yandex` (требует правки launch.py + devops-deploy).
3. **Hot-reload через ROS parameter set** (если поддержан) — менее
   надёжно, без рестарта контейнера.

**Подтверждено в:** `tts_node.py:385`, `voice_assistant_headless.launch.py:114-124`,
`voice_assistant.launch.py:145-155`, `.env.example:84`.

⇒ Карточка должна быть обновлена: либо devops-карточка на YAML fix,
либо backend-карточка на добавление launch-override.

### W2. Fix #1 (prompt rule) — низкая долгосрочная устойчивость

Текущая RULE #VOICE уже запрещает выдумывать «alena/татьяна» (строки
101-102) и явно говорит «выбирай из [TTS] voices:». Дополнительное
правило «если пользователь говорит «Алёна», мапь на `female-shaonv` /
`Russian_BrightHeroine`» — это **эвристика для конкретной LLM
(MiniMax-M3)**, которая может:
- сломаться при смене модели (DeepSeek / GPT / другая MiniMax версия);
- противоречить rule на «не выдумывать» (LLM увидит конфликт);
- разрастись в хардкод маппинга для всех известных имён (антон/захар/
  алёна/filipp/ermil...) → десятки правил.

⇒ Рекомендую Fix #1 как **guardrail в дополнение к Fix #2**, не вместо.
Конкретный patch: одна строка в RULE #VOICE типа

```
- Если юзер просит голос по ИМЕНИ (Алёна, Антон, Захар) и активный
  провайдер его НЕ содержит, скажи «этот голос недоступен у меня сейчас»
  + предложи аналогичный из текущего списка ([TTS] voices:). НЕ вызывай
  set_voice с yandex-именем на minimax-провайдере — получишь voice_unavailable.
```

+ unit-тест по образцу `test_issue_1219_set_voice_rule.py:188-199`
(`test_voice_rule_teaches_voice_enumeration`).

### W3. PR #1565 (music mutex) — риск застрять в merge-gate

PR #1565 OPEN с 23.08, head-refName по старому `z-{agent}/...` паттерну.
В merge-gate есть stale-branch scan (`agent-flow-merge-gate.sh:271-`),
который **не блокирует** напрямую ветки с этим префиксом (см. §3.0
ADR-0030 / `t_a2cd5753-stale-branch-gate` PR), но **может** пометить
`stale-conflicting` если будет force-push на develop. На карточку это
влияет косвенно: e2e-process может попытаться триггернуть round на
#1565 и заблокироваться.

⇒ Создать sub-карточку на rebase+merge PR #1565 (devops), **отдельным
kanban-create** из этой ретро-карточки. Не блок для основного fix-path.

### W4. Fix #3 (voice_input_mode) — ложная атрибуция в карточке

Автор карточки написал: «Fix #3 ... использует уже декларированный
voice_input_mode в dialogue_node (PR #1618, 86291372) — LLM через system
prompt получает список доступных голосов текущего провайдера». Это
**не соответствует реализации** (§2.8):
- PR #1618 (commit 86291372) добавил параметр `voice_input_mode` в
  `dialogue_node.declare_parameter` для Meta Quest UI — это про SOURCE.
- Никакой «[TTS] voices отфильтрованные по провайдеру» через
  `voice_input_mode` не передаётся. Фильтрация делается в
  `tts_voice_registry.py:voices_for(provider)`, и результат уже
  попадает в `[TTS] voices:` строку через `format_tts_context`.

⇒ Если Fix #3 имелся в виду как «усилить фильтрацию на стороне
dialogue_node / сделать голоса через параметр dialogue_node» — это
новая фича. ADR-upd не нужен, потому что §3.4 не покрывает этот кейс.
Если Fix #3 имелся в виду как «поменять провайдер в dialogue_node.yaml
по аналогии с voice_input_mode» — то это по сути тот же Fix #2,
только на уровне `dialogue_node.tool_provider`, что неправильно
(tool_provider — это не TTS-провайдер, см. `voice_assistant.launch.py:62-68`).

## 5. SUGGESTIONS

### S1. Долгосрочно: добавить `voice_catalog_for_provider` в LLM-контекст

Сейчас `[TTS] voices: ...` (см. `tts_voice_registry.py:126-149`) уже
правильно провайдер-зависим. Но если убрать мапинг-эвристику из RULE
#VOICE и **полностью полагаться на `[TTS] voices:`**, теряется
семантическая связь «женский голос → какие ID подходят». Компромисс:

- Оставить RULE #VOICE: «бери только из [TTS] voices:».
- Убрать hardcoded gender-mapping (строки 92-97), заменить на
  «отфильтруй [TTS] voices: по полу по префиксу `female-`/`male-`/
  семантике имени (BrightHeroine/AmbitiousWoman/ReliableMan)».
- Это автоматически адаптируется к смене провайдера.

**Рекомендую для ADR-0027 §3.5 как future work** — НЕ блокирует текущий fix.

### S2. Hot-reload `provider` через ROS param callback

`tts_node.py:517` читает `provider` один раз в `_declare_params` →
самотестирование. Если бы был callback на изменение (`add_on_set_parameters_callback`),
можно было бы переключать провайдера через `ros2 param set /tts_node provider yandex`
без devops-deploy. Сложность средняя, требует проверки teardown старого провайдера.

**Не блокер, отдельная фича.**

### S3. Stale-PR scan для PR #1565

PR #1565 имеет MERGEABLE + CI зелёный, но может застрять. Из этой
ретро-карточки создать sub-карточку для devops-воркера: «rebase PR #1565
на develop, force-push, триггернуть e2e round». Это out-of-scope для
архитектурного verdict'а, но требует action.

### S4. Кросс-проверка `voice_input_mode` ↔ Fix #3

Если архитектор подтвердит, что Fix #3 — это всё-таки добавить
`voice_input_mode` в `tts_voice_registry` (как «filter mode» для списка
голосов), то это будет naming-collision с существующим параметром
(«source» vs «filter»). Рекомендую в таком случае назвать новый параметр
`voice_filter_mode` или `voice_availability_strategy`, и явно описать
семантику в ADR-0027 §3.5.

## 6. Сравнение fix-path'ов (decision matrix)

| Критерий | Fix #1 (prompt) | Fix #2 (provider switch) | Fix #3 (voice_input_mode/новый ADR) |
|---|---|---|---|
| Что меняется | master_prompt_compact.txt + 1 unit-тест | docker/vision/.../tts_node.yaml (1 строка) или launch.py | ADR-0027 §3.4→§3.5 + code + tests |
| Требует devops-deploy | нет | ДА | ДА |
| Требует LLM-compliance | ДА (фундаментально) | нет | нет |
| Устойчивость к смене LLM | низкая | высокая | высокая |
| Устойчивость к смене провайдера | низкая | нулевая (revert) | высокая |
| Решает mv01 на текущем develop | Частично (LLM может опять промахнуться) | ДА, детерминированно | ДА, но не реализовано |
| Решает mv02/mv03 | Частично | ДА | ДА |
| Back-compat для других e2e | риск (новые правила могут сломать mix) | нулевой риск (возврат к исходному контракту) | нулевой риск |
| Соответствие original design (issue #1219) | Частично | ДА (acceptance писался под yandex) | Требует нового дизайна |
| Долгосрочная поддержка | требует ревью при смене LLM | требует мониторинга | требует реализации |

## 7. Рекомендация (одна строка)

**Fix #2 как primary, Fix #1 как guardrail, Fix #3 отклонить.**

Детальный план:
1. **Сейчас (PR #XYZ от devops):** правка
   `docker/vision/config/voice_assistant/tts_node.yaml:10` — `provider:
   yandex`. Зависимость: YANDEX_API_KEY + YANDEX_FOLDER_ID в ENV робота
   (проверить заранее, иначе tts_node упадёт).
2. **Сейчас (PR #ABC от backend):** добавить в RULE #VOICE 1 правило
   про «голос по имени → если недоступен, скажи честно». + 1 unit-тест
   по образцу `test_issue_1219_set_voice_rule.py`.
3. **Параллельно (sub-карточка от devops):** rebase PR #1565 на develop,
   force-push, триггернуть e2e round.
4. **Future work (отдельная фича):** ADR-0027 §3.5 — «voice availability
   strategy: как LLM по `[TTS] voices:` понимает, какие голоса есть
   для её провайдера».

## 8. Что нужно проверить перед merge Fix #2 (DevOps pre-merge checklist)

- [ ] YANDEX_API_KEY / YANDEX_FOLDER_ID доступны в ENV на 10.1.1.21
      (ssh ros2@10.1.1.21 'env | grep -i yandex')
- [ ] tts_node на yandex стартует без TTSAuthError
      (docker logs voice-assistant --since 30s | grep -i auth)
- [ ] tts_node показывает `[tts_node] provider=yandex` в первых 5 сек
      (а не fallback на silero из-за фолбека)
- [ ] Маленький smoke: ssh ros2@10.1.1.21, сказать «робот, скажи привет»,
      услышать голос (не молчание)
- [ ] e2e round-232..N показывает mv01 OK

## 9. VERDICT

**REQUEST_CHANGES**

*Обоснование:* Все три fix-path'а в карточке описаны неточно (W1, W4) —
ни один из них нельзя брать как есть. Гипотеза карточки верна (§2.5,
repro-тест подтверждает), но карточка смешивает конфигурационный fix
(Fix #2) с кодовыми изменениями (Fix #1, #3) и ошибочно атрибутирует
Fix #3 к `voice_input_mode`. Прежде чем реализовывать, нужно:
(а) уточнить формулировки fix-path'ов (кто делает: devops / backend /
кодер ADR-апдейта);
(б) создать sub-карточку для rebase PR #1565 (W3);
(в) явно зафиксировать в карточке, что Fix #2 НЕ «TTS_PROVIDER=yandex
в ENV» (такой переменной нет, см. W1), а правка YAML или launch-override.

После этих уточнений — переход к **APPROVE_WITH_WARNINGS** (W1 устранён,
W3 закрыт sub-карточкой, W4 явно отклонён).

## 10. Что не входит в этот verdict (out-of-scope)

- **Правка `voice_core_suite_v1.json`** — acceptance корректен, e2e-зона
  ответственности devops. Не трогаем.
- **Rebase PR #1565** — отдельная sub-карточка (см. W3).
- **Реализация Fix #2 (правка YAML)** — после re-approve карточки,
  отдельной devops-карточкой.
- **Реализация Fix #1 (правка prompt)** — после re-approve карточки,
  отдельной backend-карточкой.

---

## Приложение A: гиперссылки и команды воспроизведения

| Что | Команда |
|---|---|
| Локальный прогон repro-теста | `PYTHONPATH=src/rob_box_voice:src/rob_box_harness:src/rob_box_llm:src/rob_box_core:src/rob_box_mcp_tools python3 -m pytest src/rob_box_mcp_tools/test/test_tools/test_mv01_set_voice_alena_repro.py -v` |
| Локальный прогон prompt-contract | `PYTHONPATH=src/rob_box_voice:... python3 -m pytest src/rob_box_voice/test/unit/test_issue_1219_set_voice_rule.py -v` |
| Проверить реальный провайдер на роботе | `ssh ros2@10.1.1.21 'docker exec voice-assistant bash -c "ros2 param get /tts_node provider"'` |
| Список последних e2e runs | `gh run list --workflow="L-E2E Voice Test.yml" --limit 10 --json databaseId,conclusion,createdAt,headBranch` |
| Состояние PR #1565 | `gh pr view 1565 --json state,mergeable,statusCheckRollup` |

## Приложение B: glossary

- **MV01** — сценарий `mv01_set_voice_alena` в `voice_core_suite_v1.json:31`.
- **acceptance** — поле `voice_changed: true` в JSON, проверяется e2e-скриптом.
- **voice_unavailable** — ошибка от `SetVoiceTool` при попытке установить
  голос, не входящий в `voices_for(active_provider)`.
- **[TTS] voices: ...** — строка LLM-контекста, формируется через
  `format_tts_context()` в `tts_voice_registry.py:126-149`.

## Приложение C: changelog этого документа

- 2026-08-26: initial draft (backend, worktree
  `z-backend/t_5a7b6fe3-mv01-fix-path-verdict`, develop @ 65b471d6).
