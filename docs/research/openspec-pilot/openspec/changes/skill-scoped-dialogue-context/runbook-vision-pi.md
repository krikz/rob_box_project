# Ранбук: что осталось сделать на vision-pi

Шесть задач change'а не закрываются с рабочей станции: 0.5 (baseline),
5.5 (e2e до/после раскола), 7.4 (prefix-cache), 8.3-8.5 (приёмка). Всем им
нужен живой робот. Воркер до робота не дотянулся — `ssh` из его окружения
заблокирован политикой, а гадать хост живого робота он не стал.

Здесь — точная последовательность команд и **что именно приложить в
карточку**, чтобы каждая задача закрывалась доказательством, а не словами
(ADR-0018). Все факты ниже сверены по репозиторию, ничего не выдумано:

* контейнер — `voice-assistant` (`docker/vision/docker-compose.yaml:197`);
* метрики — `prometheus_client` на порту из `metrics_port`, по умолчанию
  **9100**, путь `/metrics` (`dialogue_node.py`, параметр объявлен со
  значением 9100; в живом конфиге — `metrics_port: 9100`);
* гистограмма — `voice_llm_prompt_tokens` с метками `provider`, `skill`,
  `estimated` (`rob_box_voice/observability/metrics.py`);
* живой конфиг ноды — `docker/vision/config/voice_assistant/dialogue_node.yaml`,
  он монтируется в контейнер как `/config` (`docker-compose.yaml:252`), а
  НЕ `src/rob_box_voice/config/dialogue_node.yaml` (тот едет в образ);
* флаги `skills_enabled` и `skill_tool_narrowing` в живой конфиг
  **добавлены** этим change'ем, оба `false`.

Хост из `docs/guides/VISION_PI_SETUP.md`: `ros2@vision-pi.local`
(или `ros2@10.1.1.11` по Ethernet / `10.1.1.21` по WiFi).

---

## 0.5 — baseline, 7 дней без единого изменения флагов

Фаза 0 уже в ветке, флаги выключены — робот в дефолтной конфигурации, и
это ровно то состояние, которое надо мерить.

```bash
# 1. Метрика вообще экспортируется?
curl -s localhost:9100/metrics | grep voice_llm_prompt_tokens | head -20

# 2. Снимок распределения (bucket'ы: 8k, 16k, 24k, 32k, 48k, 64k, +Inf)
curl -s localhost:9100/metrics | grep '^voice_llm_prompt_tokens_bucket'

# 3. Доля оценочных значений — если estimated="true" преобладает,
#    значит usage не приходит от провайдера и цифры грубые (задача 0.3).
curl -s localhost:9100/metrics | grep 'estimated='
```

Если Prometheus уже скребёт робота (см. `docs/guides/MONITORING_SYSTEM.md`),
p50/p95/p99 берутся запросом, а не глазами по бакетам:

```promql
histogram_quantile(0.50, sum by (le) (rate(voice_llm_prompt_tokens_bucket[7d])))
histogram_quantile(0.95, sum by (le) (rate(voice_llm_prompt_tokens_bucket[7d])))
histogram_quantile(0.99, sum by (le) (rate(voice_llm_prompt_tokens_bucket[7d])))
sum by (skill) (rate(voice_llm_prompt_tokens_count[7d]))
```

**В карточку:** сырой вывод `curl` (или скрин панели) + три квантиля +
разбивка по `skill`. Без разбивки по доменам сравнение в 8.4 будет
бессмысленным: выигрыш Move A зависит от того, какая доля ходов вообще
доменная.

---

## Включение Move A (шаг 2 Migration Plan)

Только после 0.5. Одна строка в живом конфиге плюс рестарт:

```bash
ssh ros2@vision-pi.local
cd ~/rob_box_project/docker/vision   # путь деплоя
sed -i 's/^    skills_enabled: false/    skills_enabled: true/' \
    config/voice_assistant/dialogue_node.yaml
docker compose restart voice-assistant

# Проверка, что нода увидела флаг и раскол сработал:
docker logs voice-assistant --tail 200 | grep -E "skills_enabled|🧩|Prompt loaded"
```

Ожидаемые строки в логе:

* `🧩 Загружено фрагментов скиллов: N (…)` — файлы прочитаны;
* `🧩 Доменные секции мастер-промпта уехали во фрагменты: composer (+…),
  dj (+…), navigation (+…), player (+…); системный промпт X → Y симв` —
  фаза 5 отработала;
* НЕ должно быть `❌ Разметка секций мастер-промпта сломана` — это значит,
  что в образ приехал промпт с битыми маркерами, и секции остались на
  позиции 0 (правила при этом не потеряны, но выигрыша нет).

Откат — `skills_enabled: false` и рестарт. Никаких миграций данных.

---

## 5.5 и 8.3 — e2e голосом, до и после

Прогнать ОДИН И ТОТ ЖЕ список фраз дважды: при `skills_enabled: false` и
при `true`. Между прогонами — чистая память (иначе повторится грабля из
предыдущей итерации: старые ходы в `local_chat.db`, где просьбы о музыке
отвечены словами без вызовов, работают как few-shot и модель продолжает
именно их — механизм из `dialog_core.py:681`).

Сценарии (по одному на домен из задачи 8.3):

| домен | фраза | что считается успехом |
|-------|-------|------------------------|
| composer | «сыграй жёсткий барабанный бит» | вызван `execute_music_code` или `compose_music`, звук пошёл |
| dj | «будь диджеем, устрой вечеринку» | вызван `set_dj_mode(enabled=true, plan=…)` |
| player | «включи трек из библиотеки» | `gen_list_library` → `gen_play_from_library` |
| navigation | «поезжай на кухню» | `list_waypoints` → `navigate_to_waypoint` |

```bash
docker logs voice-assistant --tail 50          # после КАЖДОГО сценария
docker logs voice-assistant | grep -E "tools:|tool_calls|🧩"
```

**В карточку:** последние 50 строк лога на каждый сценарий, оба прогона.
Провалом считается ответ словами без вызова инструмента — это регрессия,
а не «модель решила поговорить».

---

## 8.5 — регрессия #1403 («нет такой функции»)

Сценарий, где пред-роутер заведомо промахивается, а инструмент нужен.
Например, просьба о музыке, сформулированная не музыкальными словами:
«сделай так, чтобы было повеселее, как на вечеринке».

Успех: модель вызывает `load_skill`, получает фрагмент tool-результатом и
**в том же ходу** вызывает нужный инструмент. Провал: «у меня нет такой
функции».

```bash
docker logs voice-assistant | grep -E "load_skill|нет такой функции"
```

Метрика промахов роутера (задача 3.7) — это доля `source="llm"` в
`voice_skill_activation_total{skill, source}`; `source="router"` —
сработал пред-роутер, `source="miss"` — LLM запросила несуществующий
домен:

```bash
curl -s localhost:9100/metrics | grep voice_skill_activation_total
```

---

## 7.4 — влияние Move B на prefix-cache

Move B (`skill_tool_narrowing: true`) меняет `tools=` при каждой смене
домена и потому ломает кэшируемый префикс. Мерить надо ДВЕ вещи:

1. Латентность ответа: p50/p95 при `false` против `true` — по одинаковому
   набору фраз, не меньше 20 ходов на каждое состояние.
2. Долю попаданий в кэш — и вот тут в репозитории дырка, которую надо
   закрыть ДО замера. `DeepSeekProvider._usage_from`
   (`rob_box_llm/providers/deepseek.py:381`) вытаскивает из ответа только
   `prompt_tokens` / `completion_tokens` / `total_tokens`. Поля
   `prompt_cache_hit_tokens` / `prompt_cache_miss_tokens`, которые
   DeepSeek возвращает, отбрасываются — то есть сегодня доля попаданий не
   доезжает вообще ниоткуда, ни от primary (minimax её не отдаёт), ни от
   deepseek. Либо расширить `_usage_from` и добавить метрику, либо честно
   мерить только латентность и записать в карточку, что кэш не измерен.

**Условие входа, найденное фазой 5 (см. `tasks.md` §5 и
`test_prompt_skill_sections.py::_KNOWN_FOREIGN_MENTIONS`):** инструкции DJ
требуют `execute_music_code` и `search_samples`, которых у скилла `dj` в
каталоге нет. При включённом сужении диджей получит инструкцию, которую
нечем исполнить. Решить ДО включения Move B — иначе 7.4 померит не
prefix-cache, а этот дефект.

---

## 8.4 — сравнение с baseline

Те же три квантиля, что в 0.5, снятые за сопоставимый период после
включения Move A, плюс разбивка по `skill`. Ожидание по локальному замеру
`tiktoken`: системный промпт 10 838 → 8 543 tok, ход целиком — от −775
(composer) до −2 295 (недоменные ходы). Если на роботе выигрыш меньше
предсказанного, первое, что надо проверить, — доля ходов, на которых
скилл вообще активировался: `sum by (skill) (…_count)`.

**В карточку:** обе выборки рядом, конкретные числа. «Стало меньше» без
p50/p95 задачу не закрывает.
