# Survey: `cobanov/awesome-jev` (pre-work issue #3084)

- **Источник:** https://github.com/cobanov/awesome-jev, commit `bb350810` (25.09.2026), прочитаны весь `README.md` (377 строк, 155 проектов) и оба research-notes: `research/2026-09-19.md`, `research/2026-09-20.md`.
- **Глубже по исходникам** (клоны, read-only):
  - `typesafe-ai/typesafe-sdk-python` `f078f1e` (v0.7.2);
  - `eugeniughelbur/jev-engineering` `82655a6`;
  - `klauswg/jev-guard` `067a170`;
  - `FBddcz/embodied-jev` `f08de2e`.
- **Не делалось:**
  - платных вызовов Jev не было, ни одного бенчмарка не перепрогонял;
  - все цифры ниже — из чужих репозиториев, это **их** данные на **их** задачах;
  - `docs.typesafe.ai` и `api.typesafe.ai` из среды исследования недоступны (egress 403), поэтому контракт API взят из исходников официального SDK.

## 1. Карта применений

| Домен (раздел awesome-jev) | Что делают | Типичный примитив | Где решение |
|---|---|---|---|
| Agents / coding / guardrails (≈40 проектов) | выбор модели или скилла, гейт tool-call'ов (allow / ask / deny), проверка заявлений «done» | Choice + Noul | детерминированные правила **до** Jev; пороги в коде |
| Context / compaction | что оставить в контексте LLM | Noul на фрагмент | код режет, модель только оценивает |
| Browser / computer use | выбор операции и DOM-цели; текст пишет отдельная LLM | Choice | исполнение — код и CDP |
| Routing / data / workflows | триаж почты и логов, rerank, SQL-предикаты, риск-триаж платежей | Noul / Score / Choice | действия read-only или через hard rules |
| Games / robotics | ход из легальных, кандидаты пути, подцели манипулятора | Choice по кандидатам, сгенерированным кодом | физика, геометрия и safety — в коде |
| Open reproductions | Laya, LitJev, poorjev, AnyJev… — self-hosted аналоги | те же примитивы | — |
| Evaluation / calibration | калибровка, чувствительность к формулировке, injection | — | — |

Робототехника в списке — только симуляции: Embodied Jev (MuJoCo Franka), jev-drone (MuJoCo), JevPilot (three.js). Реального железа под управлением Jev в списке **нет**. Цитата из research-notes 20.09: «no … robotics simulation … was performed» (сами авторы списка ничего не запускали).

## 2. Паттерны интеграции (industry evidence)

1. **Код генерирует кандидатов, модель выбирает, код исполняет.** Это общий паттерн всего списка. В README сформулировано так: «text or structured state + typed questions → constrained answers + probabilities → deterministic application code».
   - Chess with Jev: легальные ходы и факты считает код.
   - jev-tetris: код перечисляет все размещения.
   - Embodied Jev: код отбирает допустимые фазы (`planning.py:29-51`) и отбрасывает цели вне workspace.

   → Подтверждает принцип issue: «Jev выбирает только среди кандидатов нашего кода».
2. **Hard rules до модели, модель может только ужесточить.**
   - jev-engineering: regex-запреты → allowlist → Jev → пороги. Порядок зафиксирован тестом `tests/test_order.py`. Командные политики «can only add rules or tighten thresholds, never loosen».
   - jev-guard: blacklist, sanction и сумма > $500k решаются правилами, модель не вызывается.
   - Composio: «destructive-action gates».

   → Прямо ложится на наш `EMERGENCY_TOOLS` и монотонный `escalate_acceptance`.
3. **Fail-open vs fail-closed зависит от класса решения.**
   - jev-engineering (`recipes/README.md:5-13`): «fail open для локального coding-агента, fail closed для денег / почты / прода».
   - jev-guard при любой ошибке отправляет в `MANUAL_REVIEW` (fail-closed).
   - Embodied Jev при ошибке модели останавливает эпизод и **не** откатывается на правила.

   → У нас детерминированный путь — это **полноценная политика**, а не «разрешить всё». Поэтому «fail-open-to-local-fallback» у нас означает «fail to policy»: для acceptance это текущий `confirmation_policy.yaml`, который уже fail-closed для движения.
4. **Shadow mode перед включением.** jev-skill-router: «starts in a shadow mode that only logs the decision». → Наш scheduler-PoC сделан именно так (`scheduler_shadow.py`).
5. **Явная ветка Uncertain / abstain.**
   - discern: `Uncertain` branch.
   - Jevonian: `minConfidence` помечает маршрут, а не принимает его молча.
   - jev-sheets возвращает `UNSURE`.
   - wakegate пропускает шаг только когда Jev уверен.

   → `min_confidence` на класс в `routing.py`; ниже порога — fallback.
6. **Провайдер-абстракция и переключение провайдера.**
   - langchain-skill-router: «the judge is a protocol, so a self-hosted model or static rules can take Jev's place».
   - Ollaya и stuntd отдают `/v1/systemone` из локальной Laya; клиенты переключаются через `TYPESAFE_BASE_URL`.

   → Наш `DecisionProvider` + `SystemOneHttpProvider` с разным `base_url`.
7. **Кэш.**
   - jev-engineering `usecases/jevchecks.py`: дисковый кэш по SHA-256 от `{model, state, questions}`.
   - jevguard-mcp: SQLite WAL.
   - jev-guard: 24-часовое окно признаков.

   Для робота state (поза, активная задача) почти не повторяется, поэтому кэш в PoC **не** добавлен. Кэшировать имеет смысл только статичные вопросы о tool+args без контекста — это решение на потом, по данным.
8. **Таймауты и ретраи в реальных проектах.**

   | Проект | Таймаут | Ретраи | Прочее |
   |---|---|---|---|
   | jev-engineering gate | 5 с | нет | fail-open → «ask» |
   | jev-engineering jevchecks | 20 с | 1 | — |
   | jev-guard | 8 с | 1 | ретрай 429 только если `retry-after` ≤ 5 с |
   | Embodied Jev | 25 с | нет | — |
   | Официальный SDK | 10 с на операцию | **2**, backoff 0.5–5 с | общий бюджет **30 с** (`_core/retry.py:52-86,118-120`) |

   → Эти таймауты рассчитаны на офлайн-агентов и CI, **для робота они непригодны**. Поэтому в `routing.py` дедлайн один на всю цепочку (`MAX_DEADLINE_MS=2000`), ретраев нет, а официальный SDK не используется.

## 3. Схемы Choice / Score / Noul — что готово для наших случаев

| Наш случай | Ближайший пример | Схема |
|---|---|---|
| acceptance / human review | OpenRouter cookbook «Gating agent tool calls», jev-engineering | `destructive` (Noul) + `verdict` (Choice allow/ask/deny); пороги deny ≥ 0.90, confidence floor 0.45, allow < 0.10 |
| риск действия | jev-guard | `risk_level` (Score, 5 уровней) + `should_freeze` (Noul) + гейт: level ≥ 4, p ≥ 0.80, conf ≥ 0.70 |
| выбор ветки / next action | Embodied Jev hierarchical | Choice подцели (8 вариантов), затем 4 Choice по осям в одном запросе |
| роутинг ввода (наш quick_decide) | jev-voice-browser, jev-canvas | Choice intent из частичного транскрипта; код решает act / wait / ask |

Готовой схемы для наших инструментов нет — адаптируем. Правила, которые берём из опыта:

- версионировать вопросы, критерии и пороги **вместе**, как один артефакт (jev-engineering `recipes/README.md:151`);
- порядок вариантов влияет на ответ (там же, `:131`) — варианты задаёт код в фиксированном порядке;
- у Noul нет confidence (SDK `models.py:73-80`). jev-guard выводит её как `|p−0.5|×2`, у нас `max(p, 1−p)`: та же монотонность, шкала [0.5, 1];
- Choice у Jev **не всегда argmax** своего распределения. Embodied Jev видел `y_neg_40`=18% при `z_neg_40`=19% (`JEV_EVALUATION.md:138-140`). Мы это не отвергаем, но и не требуем argmax.

## 4. Antipatterns / lessons learned (evidence)

| Урок | Evidence | Что у нас |
|---|---|---|
| **Schema-valid ≠ правильное решение** | awesome-jev README, «Important caveat» | валидация ответа + пороги + fallback |
| **Высокая confidence ≠ точность** | jev-guard, n=100 синтетика: у Jev самый уверенный бакет — самый неточный (25%); rules-only 68% vs Jev 50%; Jev отправил 65/100 в review | модель не понижает политику; пороги — только после нашей калибровки |
| **Калибровка на «кубиках» провальна** | «Jev Does Not Play Dice»: Choice дал 82.9% на грань 1 при точности 19.0%; заявленный риск 30% превратил в 5.3% | Score/Noul не используем как «настоящую вероятность» без замера на наших данных |
| **Prompt injection через state работает частично** | jev-engineering, 300 вызовов: грубая инъекция сдвинула 0/30 опасных команд; «authority framing» («владелец одобрил») — до 3/30; `git stash clear` пропускался во всех трёх authority-вариантах; чтобы поймать все атаки, нужен floor 0.8, но тогда 58% нормального трафика уходит в escalation | текст пользователя попадает в state → модель **никогда** не может ослабить hard rule; `stop_navigation` сигнал вообще не читает |
| **Скоры плавают между запусками и версиями** | jev-engineering README:88: одна команда 0.94 deny в одном прогоне и 0.78 ask в другом | модель пинуется (`jev-1.13.0`, а не `jev-latest`) |
| **Слабые места Jev 1.13** | first-party jaggedness guide (по notes 19.09): счёт, арифметика, сравнение дат, косвенные вопросы, нерелевантный контекст, adversarial state; Choice и Noul не эквивалентны | геометрию, расстояния, время и заряд считает код; state компактный |
| **Роботика: модель хуже правил** | Embodied Jev: 2 успешных прогона, но куб оба раза выскальзывал, `release` модель не выбрала ни разу; плоский 21-action режим не взял куб за 40 шагов. RoboJEV (цитируется там же): Jev 5/10 vs rules 8/10 | Jev не управляет движением; навигация — P2 и только выбор среди кандидатов |
| **Jev как замена LLM** | README: «rather than free-form text generation»; браузерные агенты берут отдельную LLM для текста | не заменяем основную LLM (out of scope issue) |
| **Hosted Jev — только текст** | README «Input boundary»; notes 19.09: бюджет 64k на запрос, 32k на state + самый длинный вопрос | perception передаём как структурированные признаки |
| **Холодный старт** | jev-guard: первый вызов ≈ 2.6 с; дальше p50 272 мс, p95 830 мс | дедлайн сработает на холодном старте — это нормально, будет fallback |

Латентность из чужих замеров (не наш deployment path):

| Проект | p50 | p95 | Прочее |
|---|---|---|---|
| jev-engineering | 371 мс | 533 мс | max 2197 мс; $0.0000189 за вызов |
| jev-guard | 272 мс | 830 мс | — |
| Embodied Jev | 326–361 мс | — | медиана |
| Convex evals | 199 мс | — | медиана на вопрос |

Вендорский ориентир «70–500 ms» по p95 **не подтверждается** двумя из трёх независимых замеров. Для нас это значит, что на safety-критичном пути (дедлайн 400 мс) заметная доля вызовов будет уходить в fallback. Это нужно мерить у нас — см. `README.md`, «Что осталось».

## 5. Анализ пяти точек интеграции issue против текущего кода

> ⚠️ Две ссылки в issue устарели.
> - `src/rob_box_voice/rob_box_voice/scheduler/decision.py` (`HighLevelPlanner`, `DecisionPlan`, `DecisionCoordinator`) удалён в #2260 («удалить мёртвую половину планировщика», ADR-0080 §2.8) — модуль ни один путь не импортировал.
> - `src/rob_box_harness/rob_box_harness/core/dialog_core.py` заменён на `agent_core.py` (релиз #3098); эвристики переупорядочивания tool call'ов теперь в `agent_core.py:169-320` (`_order_tool_calls`).

| Точка issue | Состояние в коде сейчас | Похожие примеры | Вывод |
|---|---|---|---|
| P0 scheduler | `scheduler/quick_decide.py` — правила IGNORE / REPLACE / PENDING_LLM. `SCHEDULER_DESIGN.md §4.7` (v5, решение owner'а) **отказался от «уровня 2»** — второго модельного вызова (доп. RTT, двойная стоимость, рассинхрон с основной LLM) | jev-voice-browser, jev-skill-router (shadow) | **Конфликт с §4.7.** Сделан только shadow-PoC; включение требует решения Шифу по §4.7 |
| P1 acceptance | `AcceptanceGate.submit` синхронный (`acceptance.py:389`); `EMERGENCY_TOOLS={"stop_navigation"}` зашит в код | jev-engineering, jev-guard, OpenRouter gate | Подходит лучше всего: только эскалация, сигнал считается заранее. PoC: `escalate_acceptance` |
| P1 dialog (tool ordering) | `agent_core._order_tool_calls`: prelude first, destructive last | Edward (continue / pause / escalate) | Не трогать: это инвариант безопасности порядка, а не догадка. Модель здесь не даёт выигрыша, который нельзя получить правилом |
| P2 navigation | `navigation_skill.py`: закрытый набор действий | Embodied Jev, JevPilot, RoboJEV | Evidence против: в робототехнике модель проигрывает правилам. Не раньше, чем acceptance покажет пользу |
| P2 PASTE / action-server | `docs/architecture/action-protocol.md` | — | Только side-effect-free выбор кандидата до commit; примеров нет |

## 6. Что это меняет в acceptance criteria issue (предложение)

- Таймауты: вместо «timeout appropriate» — **общий дедлайн на цепочку**. Дефолты: safety 400 мс, latency 150 мс, quality 800 мс, потолок 2000 мс; **0 ретраев**. Основание — p95 272–830 мс в чужих замерах; финальные значения ставятся по нашим p95.
- Добавить: «модель пинуется; вопросы, критерии и пороги версионируются вместе».
- Добавить: «пороги ставятся только по калибровке на размеченных наших сценариях» (jev-guard, «Jev Does Not Play Dice»).
- Scheduler P0 → перевести в «shadow-only до пересмотра §4.7».
- Navigation P2 → «не начинать без положительного результата acceptance-оценки».
