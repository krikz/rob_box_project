# Доказательства с vision-pi (02.09.2026, вечер)

Собрано на живом роботе после того, как товарищ Шифу задеплоил `develop`
с этим change'ем (образ
`10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-dev`, контейнер
поднят `2026-09-02T17:12:22Z`). Доступ — по ключу через катану
(`ssh -J ros2@ros2-katana-gf66-11ud ros2@10.1.1.21`); прямой маршрут из
сети воркера закрыт (`kex_exchange_identification: Connection closed`
на 10.1.1.11/.20/.21).

## 1. До деплоя: на роботе не было НИЧЕГО из change'а

Снято тем же вечером, пока крутился прежний образ
(`voice-assistant-humble-test`, собран 16:41Z):

```
phase0 metric (voice_llm_prompt_tokens в metrics.py) : 0
skills_enabled в dialogue_node.py                    : 0
маркеры SKILL-MOVE в master_prompt_compact.txt       : 0
prompts/skills/                                      : 5 старых мёртвых файлов
                                                       (faq/memory/music/navigation/status_skill_prompt.txt)
curl localhost:9100/metrics                          : только python_* и process_*,
                                                       ни одной voice_*
```

Мастер-промпт на роботе был **байт в байт** наш дореформенный файл:

```
sha256 = 77d34a3ef0f191dde5d9b95556cd42b32cbeb01d676eaf168162fec9f4989b57
размер = 38 973 байта (LF)
```

Тот же хэш даёт наш `master_prompt_compact.txt` до разметки после
нормализации CRLF→LF. Значит локальные замеры `tiktoken` (10 838 tok)
описывают ровно то, что читала LLM на роботе, а не абстракцию.

## 2. После деплоя: change приехал целиком

```
phase0 metric   : 2
skills_enabled  : 13
SKILL-MOVE      : 10   (пять открывающих + пять закрывающих)
prompts/skills/ : 13 файлов (composer, core, dj, expression, knowledge,
                  mapping, memory, navigation, player, renardo-library,
                  scheduler, voice-tts + ещё не удалённый music_skill_prompt)
```

Модуль импортируется внутри контейнера:

```
$ python3 -c "from rob_box_core.prompt_sections import render_prompt, MARKER_PREFIX; ..."
prompt_sections OK, marker= <<<
```

Лог ноды при старте:

```
[dialogue_node] ✅ Prompt loaded: master_prompt_compact.txt (34552 bytes)
[dialogue_node] ℹ️ skills_enabled=false — доменные скиллы выключены
                  (Move A не активен, поведение как до change'а)
```

Дефолтная конфигурация, как и требует Migration Plan. Ни одного
`❌ Разметка секций мастер-промпта сломана`, ни одного `Traceback`,
ни одного упоминания `prompt_sections` в ошибках — `grep -ic` дал 0.

## 3. Главное обещание фазы 5, проверенное НА РОБОТЕ

Раскол обязан быть невидимым при выключенном флаге. Проверка прогнана
внутри контейнера, на том самом файле, который читает нода:

```
rendered(off) sha256 : 77d34a3ef0f191dde5d9b95556cd42b32cbeb01d676eaf168162fec9f4989b57
ожидался (до change) : 77d34a3ef0f191dde5d9b95556cd42b32cbeb01d676eaf168162fec9f4989b57
СОВПАЛ
chars off/on         : 32154 25286
  секция -> composer  : 4950 симв
  секция -> dj        : 5753 симв
  секция -> navigation:  424 симв
  секция -> player    :  708 симв
```

То есть при `skills_enabled=false` LLM на роботе получает промпт,
побайтово равный дореформенному. Это не рассуждение и не юнит-тест на
синтетике — это хэш файла, отрендеренного тем же кодом, который сейчас
работает на роботе.

## 4. Baseline (задача 0.5) — день 0

Снято во время сета, запущенного товарищем Шифу:

```
voice_llm_prompt_tokens_count{estimated="false",provider="health-aware-fallback",skill="none"}  18
voice_llm_prompt_tokens_sum  {estimated="false",provider="health-aware-fallback",skill="none"}  483155

bucket le=24000 :  0
bucket le=28000 : 13
bucket le=32000 : 18
bucket le=+Inf  : 18
```

Что из этого следует:

* **среднее 26 842 tok на обращение** — оценка из `design.md` (~27 100)
  подтвердилась на живом трафике;
* p50 попадает в интервал (24 000; 28 000], p95 — в (28 000; 32 000];
  ни одного обращения ниже 24 000 токенов;
* `estimated="false"` у **всех** 18 — значит задача 0.1 работает:
  primary-провайдер (minimax) действительно отдаёт `usage`, и цифры не
  оценочные, а провайдерские. Серий с `estimated="true"` нет вообще;
* `skill="none"` у всех — Move A выключен, как и задумано.

Задача 0.5 требует **семи дней**. Это день 0: метрика подтверждённо
работает и уже пишет реальные значения. Снимать через неделю — тем же
`curl`, разницу считать по этим числам.

## 5. Чего здесь НЕТ

* 5.5 и 8.3-8.5 (e2e до/после) — не прогонялись: включать
  `skills_enabled=true` до окончания недели baseline запрещено самим
  Migration Plan, а без включения «после» не существует.
* 7.4 — не мерилось, см. условия входа в `tasks.md` 7.4.
* Навигационный сценарий вообще требует отдельного разрешения: он
  двигает робота физически.
