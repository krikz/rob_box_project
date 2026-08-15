# ADR-0017: Zenoh Router SPOF — решение для лабораторной среды (SL-2, issue #835)

Status: accepted (infrastructure, milestone M2)
Date: 2026-08-15
Deciders: devops (kanban t_b7bdcb3b)

## Context

В текущей архитектуре Rob Box (см. `docs/architecture/ZENOH_CLOUD_NAMESPACES.md`)
используются два Zenoh-роутера (`eclipse/zenoh:1.6.2`):

- **Main Pi** (`docker/main/`): контейнер `zenoh-router`, слушает
  `tcp/10.1.1.10:7447`, подключается к облачному роутеру
  `zenoh.robbox.online:7447`. Все ROS-ноды Main Pi подключаются к нему
  через session config (`ZENOH_SESSION_CONFIG_URI`), всё межнодовое
  взаимодействие Main Pi и весь трафик Vision Pi → облако идёт через него.
- **Vision Pi** (`docker/vision/`): контейнер `zenoh-router-vision`, слушает
  `tcp/10.1.1.11:7447`, подключается к роутеру Main Pi
  (`tcp/10.1.1.10:7447#iface=eth0`). ROS-ноды Vision Pi подключены к нему.

Таким образом, роутер Main Pi является **единой точкой отказа (SPOF)**:

- если контейнер `zenoh-router` падает, все ROS-ноды Main Pi теряют роутер
  (`connect.timeout_ms: { peer: -1 }`, `exit_on_failure: { peer: false }` —
  ноды остаются живы, но без обмена данными);
- Vision Pi теряет uplink к Main Pi и облаку (retry с backoff 1–4 c);
- облако теряет робота целиком.

Это зафиксировано в TECH_DEBT.md как **SL-2** («Один Zenoh router = SPOF»,
medium, defer:M2): «Redundancy возможна; low priority для lab env».

## Варианты

### A. Полная redundancy: второй активный роутер на Main Pi
Поднять второй экземпляр `zenohd` на другом порту (например 7448) и настроить
ноды на несколько endpoints (`connect.endpoints: ["tcp/10.1.1.10:7447",
"tcp/10.1.1.10:7448"]`).

- ✅ Полное устранение SPOF на уровне процесса
- ❌ Два роутера на одном хосте не защищают от отказа Pi/сети
- ❌ Дублирование трафика discovery, лишняя сложность для lab
- ❌ Требует пересборки/переконфигурации всех нод (session config)

### B. Failover-роутер на Vision Pi (второй хост)
Поднять ещё один роутер на Vision Pi и включить его в `connect.endpoints`
нод Main Pi как fallback.

- ✅ Защита от отказа роутера Main Pi
- ❌ Vision Pi сам зависит от Main Pi (питание/сеть общие в lab)
- ❌ Существенная переделка session config'ов и проверка failover-поведения
  rmw_zenoh_cpp (не гарантировано, что ноды переключатся прозрачно)
- ❌ Сложность несоразмерна пользе для лабораторной среды

### C. Status quo + observability (выбрано)
Оставить одиночный роутер Main Pi (и Vision Pi) как есть, но закрыть
главную «слепую зону» — **отсутствие алертов** при падении роутера:

- healthcheck и `restart: unless-stopped` уже настроены в обоих compose
  (контейнер сам перезапустится при падении процесса);
- **добавить Prometheus alert rules** на недоступность роутера
  (`container_last_seen` из cAdvisor) — оператор узнает о проблеме в
  течение пары минут, даже если auto-restart не сработал;
- зафиксировать решение и процедуру восстановления в runbook/ADR.

- ✅ Нулевой риск для работающей системы
- ✅ Дёшево: только конфиг мониторинга
- ✅ Покрывает реальный сценарий lab: роутер упал → алерт → руками
  `docker compose up -d zenoh-router` (или авто-restart уже поднял)
- ❌ SPOF остаётся — приемлемо для lab (см. Trade-offs)

## Decision

Принять вариант **C**: одиночный Zenoh router остаётся для лабораторной
среды (M2), полная redundancy откладывается. Для снижения риска:

1. Добавить Prometheus alert rules (`ZenohRouterMainDown`,
   `ZenohRouterVisionDown`) в `docker/monitoring/config/prometheus_rules.yml`
   и подключить их через `rule_files` в `docker/monitoring/config/prometheus.yml`.
2. Смонтировать файл правил в `docker/monitoring/docker-compose.yaml`.
3. Зафиксировать процедуру диагностики/восстановления (см. раздел
   «Runbook» ниже) и обновить `TECH_DEBT.md` (SL-2 → addressed/observed).
4. Если в будущем потребуется настоящая redundancy (несколько роботов,
   продакшн-эксплуатация) — пересмотреть вариант B с тестом failover на
   стенде; до этого момента single-router остаётся осознанным решением.

## Runbook (падение zenoh-router)

Диагностика:

```bash
# На Main Pi
docker ps -a --filter name=zenoh-router
docker logs zenoh-router --tail 50
curl -sf http://10.1.1.10:8000/@/local/router && echo OK

# На Vision Pi
docker ps -a --filter name=zenoh-router-vision
curl -sf http://10.1.1.11:8000/@/local/router && echo OK
```

Восстановление (auto-restart обычно уже поднял контейнер — проверьте):

```bash
# Main Pi
docker compose -f docker/main/docker-compose.yaml up -d zenoh-router
# Vision Pi (если его роутер потерял uplink)
docker compose -f docker/vision/docker-compose.yaml up -d zenoh-router
```

Проверка после восстановления:

```bash
curl -sf http://10.1.1.10:8000/@/local/router
curl -sf http://10.1.1.10:8000/@/router/local/status
# ROS-ноды переподключатся автоматически (retry 1–4 c)
```

## Trade-offs

- Для лабораторной среды (1 робот, 2 Pi, контролируемая сеть) отказ
  роутера — редкое событие, и его последствия ограничены: ноды живы,
  контейнер перезапускается автоматически. Полная redundancy добавила бы
  сложность конфигурации и риск непрозрачного failover rmw_zenoh_cpp без
  заметной пользы.
- Главный недостаток status quo — «тихий» отказ: без алерта можно не
  заметить, что робот «оглох». Именно это закрывает данное изменение.

## Consequences

- При падении `zenoh-router` оператор получит Prometheus-алерт
  (severity: critical) в течение ~2 минут.
- Инфраструктура мониторинга получает первую группу alert rules
  (файл `prometheus_rules.yml`), куда в будущем добавляются остальные.
- Конфигурация роутеров и session config'ов НЕ меняется — деплой на робота
  не требуется, изменения только на monitoring-хосте.

## Verification

- `python3 scripts/testing/test_monitoring_config.py` (расширен: проверяет
  новый prometheus.yml + prometheus_rules.yml).
- `docker compose -f docker/monitoring/docker-compose.yaml config` — валидность
  compose после добавления volume.
- e2e (после merge): `curl http://<monitoring>:9090/api/v1/rules` показывает
  `ZenohRouterMainDown`/`ZenohRouterVisionDown` (state=inactive) при живом
  роботе; smoke-голосовая команда через Zenoh проходит (робот отвечает).
