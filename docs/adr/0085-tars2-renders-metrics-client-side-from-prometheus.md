# ADR-0085: TARS 2 рисует метрики сам из Prometheus, а не показывает Grafana

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-09-08 |
| Issue | [#2184](https://github.com/krikz/rob_box_project/issues/2184) («TARS 2 metrics panel: `show_metrics` не доходит до клиента»), PR [#2185](https://github.com/krikz/rob_box_project/pull/2185) |
| Контекст | `show_metrics` починили на уровне slice-guard и регистрации в MCP (первая итерация #2185), но на экране TARS 2 оператор всё равно не видел ни одной цифры: тракт заканчивался URL'ом Grafana, который клиент рисовал текстом. |
| Связанные | ADR-0018 (честный FAIL лучше красивого PASS), ADR-0052 §2.2 (slice guard), ADR-0074 §4.0 (размеры панелей Captain Bridge), ADR-0075 (MultiThreadedExecutor в supervisor) |

> **Перенумерован 2026-09-09 из ADR-0080 в ADR-0085.** Номер 0080 оказался
> занят дважды: этот документ влит 08.09 (PR #2185), а
> `0080-voice-and-headset-control-eight-seams.md` — 09.09 (PR #2221).
> `validate_adr_namespace.sh` пропустил обоих: он сверялся с
> `origin/develop` и не видел PR в полёте. Переименован этот, потому что
> на него нет ни одной внешней ссылки, тогда как на второй ссылаются 63
> места в коде и документации и 23 карточки по §-разделам. Порядок мержа
> здесь уступает связности ссылок: цель ADR-AF-0030 — однозначность,
> а не старшинство.

## TL;DR

Панель TARS 2 больше не показывает ссылку на Grafana. `avatar_supervisor`
сам запрашивает Prometheus (`/api/v1/query_range`) или Loki
(`/loki/api/v1/query_range`), публикует **ряды точек** в
`/avatar/tars/panel_data`, а Quest-клиент рисует по ним график на своей
canvas-текстуре. Grafana остаётся ссылкой «доглядеть с ноутбука»
(Explore-URL), а не источником картинки.

## Почему не Grafana

Показать саму Grafana на экране внутри VR не получается ни одним из трёх
очевидных способов — проверено на стенде (katana + Vision Pi, 08.09.2026):

| Способ | Что мешает |
|---|---|
| `<iframe>` в 3D-сцене | В immersive-WebXR браузер не проецирует DOM/overlay на плоскость в мире; Three.js не владеет DOM-контекстом iframe. Недостижимо в принципе, а не «пока не сделано». |
| PNG через `/render/d-solo/...` | Плагин `grafana-image-renderer` не установлен. |
| Любой HTTP к Grafana без креды | Анонимный доступ выключен: `/api/health` → 200, `/api/search` → 401. |

Плюс URL, который собирался до этой карточки, был мёртв дважды:
`http://prometheus.lan/grafana/...` — хост `prometheus.lan` не резолвится
(`getent hosts` пусто), reverse-proxy `docker/vision/Caddyfile` из docstring'а
не существует; дашборда `d/prometheus-overview` в Grafana нет, а параметр
`?query=` дашборд игнорирует — его понимает только Explore.

Prometheus и Loki, наоборот, отдают JSON по HTTP без авторизации и без
плагинов. Нарисовать ряд точек на canvas — ровно та же техника, которой уже
работают камерные панели (`video_panel.ts`). Поэтому данные берём у
источника, а рендер делаем на клиенте.

## Тракт

```
LLM tool call show_metrics(query)
  → ShowMetricsTool (mcp_server, контейнер voice-assistant)
      publish /avatar/tars/panel_request {request_id, query, datasource}
      и ЖДЁТ ответа (до SHOW_METRICS_TIMEOUT, 8 s)
  → TarsPanelDispatcher (avatar_supervisor)
      MetricsSource.query_range() / query_logs()  ← HTTP к Prometheus/Loki
      publish /avatar/tars/panel_url   {request_id, url, status, error}   ← legacy
      publish /avatar/tars/panel_data  {request_id, status, query, note,
                                        summary, series[], lines[], available[],
                                        url, error}
  → QuestNode._on_tars_panel_data → JSON_EVENT tars_panel_data (WebSocket)
  → tars2Panel.setPanelData() → график на canvas-текстуре TARS 2
```

Порядок публикации `panel_url` → `panel_data` **значим**: клиент обрабатывает
события в порядке прихода, а `setPanelUrl` сбрасывает нарисованные данные
(ссылка их не несёт). Обратный порядок затирал бы только что показанный
график.

## Решения, зафиксированные этой карточкой

### 1. Тул ждёт результат, а не «выстрелил и забыл»

`ShowMetricsTool` подписывается на `/avatar/tars/panel_data` (своя
`ReentrantCallbackGroup`; `mcp_server` крутится на `MultiThreadedExecutor`,
см. ADR-0075) и блокируется на `threading.Event` до 8 s. Event
регистрируется **до** публикации запроса — иначе быстрый ответ уходит в
никуда.

Причина — ADR-0018. Раньше тул возвращал «Опубликовал запрос… Quest откроет
панель», и ТАРС бодро рапортовал оператору об открытой панели, даже когда
метрики не существует, Prometheus недоступен или супервизор лежит.
`execution_type` поэтому `MEDIUM`, а не `FAST`.

### 2. Запрос резолвится по живому каталогу метрик

LLM генерирует PromQL по памяти и промахивается: в логах #2184 на «покажи
дрейф CPU» приехало `rate(network_latency_ms[5m])` — такой метрики нет и не
было. `MetricsSource.resolve_query()` сверяет имена с
`/api/v1/label/__name__/values` (кэш 60 s) и чинит: алиас из каталога
`QUERY_ALIASES` (включая русские слова — «память», «загрузка», «задержка»),
иначе близкое имя через `difflib` (cutoff 0.75). Подмена всегда
проговаривается в `note` — оператор должен знать, что смотрит не на своё
выражение.

Алиас-**выражение** (например `rate(a[5m]) / rate(b[5m])`) заменяет запрос
**целиком**, а не подставляется в позицию имени: подстановка внутрь
`rate(X[5m])` давала `rate(rate(a[5m]) / rate(b[5m])[5m])`, на что живой
Prometheus отвечает 400 (поймано при проверке на стенде).

### 3. Три исхода, а не два

`ok` / `empty` / `error` — разные состояния и на панели, и в ответе LLM.
`empty` («запрос выполнился, данных нет») сопровождается списком реально
существующих метрик; без этого оператор читает пустой экран как поломку
тракта.

### 4. Адрес мониторинга задаётся явно

Prometheus/Loki/Grafana подняты на build-машине (katana, `10.1.1.249`,
host-сеть), робот — на Vision Pi, тоже host-сеть. Docker-DNS между ними нет,
поэтому дефолты `http://prometheus:9090` / `http://loki:3100` не резолвились
с робота вообще — молча не работали ещё `container_status` и
`read_logs(source=loki)`. `MONITORING_HOST` / `PROMETHEUS_URL` / `LOKI_URL` /
`GRAFANA_URL` заданы в `docker/vision/docker-compose.yaml`.

## Что осталось за рамками (нужно поднять отдельно)

* **cAdvisor не запущен** ни на Main Pi, ни на Vision Pi — оба сервиса под
  `profiles: ["monitoring"]`, оба таргета в Prometheus `down`. Поэтому
  общесистемного CPU/RAM/диска в Prometheus нет, и «CPU» резолвится в
  `process_cpu_seconds_total` самих voice-нод.
* **Loki пуст**: `promtail` на Vision Pi тоже под профилем `monitoring` и не
  поднят — `/loki/api/v1/labels` не возвращает ни одного лейбла. Ветка
  `datasource=loki` работает и отвечает честным `empty`, но данных в ней не
  будет, пока promtail не запущен.
* Провиженные дашборды Grafana ссылаются на datasource UID `Prometheus` /
  `Loki`, а реальные UID — `PBFA97CFB590B2093` / `P8E80F9AEF21F6940`.
  Explore-URL мы поэтому строим по **имени** datasource, но сами дашборды
  этим сломаны.
* `otel-collector` на katana в crash-loop; в развёрнутом `prometheus.yml`
  таргет `localhost:8888` вместо `8889` (в репозитории уже 8889 — на стенде
  конфиг устарел).
