# Хендофф — единый движок сборки, разбор apt-cacher-ng на katana, 2026-09-28

> **Кто писал:** Claude, облачная сессия, **без доступа к katana**. Всё, что
> ниже помечено «на katana», — гипотезы и команды для проверки, не факты.
> **Следующая сессия:** будет с доступом к katana. Её задача — §4.
> **Честность (ADR-0018 / AGENTS.md):** у каждого «зелёно» стоит run/job id.
> §5 — то, что НЕ проверено.

---

## 1. Что сделано (не дублирую — ссылки)

- **PR:** [krikz/rob_box_project#3100](https://github.com/krikz/rob_box_project/pull/3100) → `develop`,
  ветка `claude/compassionate-hopper-uc56yt`, коммит `6416c92` (+ этот файл).
  Причина исходного падения, дизайн и raw-вывод проверок — в описании PR.
- **Коротко:** run [36351952819](https://github.com/krikz/rob_box_project/actions/runs/36351952819)
  упал с `host-gateway is not supported by the docker-container driver`:
  composite делал `docker buildx use`, это прилипало к раннеру, а
  `L-Build Base Images.yml` жил на своём inline buildx. Теперь одна реализация
  сборки — `scripts/build/buildx_build.sh` (`--builder`, без `buildx use`,
  подмена `host-gateway` на IP шлюза всегда). Её зовут composite (все L-Build)
  и локальный `scripts/build/build.py`. Базы вписаны в `docker/build-manifest.yaml`
  (`base_images`). Анализ локально — `scripts/ci/run_workflow_job.py`,
  цели в `Makefile`. Документация — `docs/CI_CD_PIPELINE.md` §«Локальная разработка и сборка».
- **Комментарии в PR с разбором прогона:**
  [attempt 1](https://github.com/krikz/rob_box_project/pull/3100#issuecomment-5870408396),
  [attempt 2](https://github.com/krikz/rob_box_project/pull/3100#issuecomment-5870697030).

## 2. Проверочный прогон 36421581380 (`L: Build All Services` на `6416c92`)

Запущен Шифу. Снимок на **13:22 UTC 28.09**, attempt 2 ещё шёл.

| job | итог | evidence |
|---|---|---|
| base ros2-zenoh | ✅ 29 мин (холодный) | job 108925437071, `✅ Built ros2-zenoh`, digest `sha256:268bf01f…` |
| base depthai | ✅ 33 мин | job 108925437017 |
| base rtabmap | ✅ 40 мин | job 108925436916 |
| base pcl | ❌ attempt 1 → ✅ attempt 2 (9 мин) | 108936203116 (apt 503) → 108940610231 |
| main rtabmap | ✅ **15 с** — registry-кеш на новом пути работает | 108945295801 |
| main perception | ❌ apt-cacher-ng 503 | 108945295791 |
| main lslidar, ros2-control, teleop, twist-mux, nav2, robot-state-publisher | ⏳ шли | — |
| vision (10 сервисов) | ⏳ led-matrix, supercollider стартовали, остальные в очереди | — |

- **Исходная ошибка не повторилась ни в одном job'е.**
- **Перезапуск упавших job'ов уже потрачен** (`rerun_failed_jobs` в 13:06).
  Повторно этот прогон НЕ перезапускать, нужен новый `workflow_dispatch`
  после починки apt-cacher-ng.
- Итог attempt 2 я не видел. Первым делом: `list_workflow_jobs 36421581380 filter=latest`.

## 3. Открытая проблема: apt-cacher-ng отдаёт 503

Два разных job'а в двух попытках упали на одном и том же:

```
Err:221 http://ports.ubuntu.com/ubuntu-ports jammy-updates/main arm64 libsoup2.4-common ...
  503  Connection closed, check DlMaxRetries [IP: 172.17.0.1 3142]          (pcl, attempt 1)
W: Failed to fetch http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-arm64/Packages.gz
  503  Connection closed, check DlMaxRetries [IP: 172.17.0.1 3142]          (perception, attempt 2)
E: Unable to locate package ros-humble-control-msgs
```

- `503 … check DlMaxRetries` формирует **сам apt-cacher-ng**: он не смог
  скачать файл со своего upstream. Сеть buildkit → прокси работает: через
  `172.17.0.1:3142` прошло 507 MB.
- В Dockerfile'ах уже стоят `Acquire::Retries "3"` и `Pipeline-Depth "0"`
  (см. `docker/base/Dockerfile.pcl:16-26`), apt ретраил.
- Прокси — контейнер `build-apt-cache` (`sameersbn/apt-cacher-ng:3.7.4`), см.
  `docker/build/docker-compose.yaml:34`, руководство `docker/build/APT_CACHER_NG_GUIDE.md`.

**Гипотезы в порядке проверки:**

1. **Кончается диск.** В `docs/plans/2026-09-21-cicd-pipeline-handoff.md` §7:
   «Registry на katana: 62.5 ГБ, диск 96%». Этот PR **добавляет** registry-кеш
   `mode=max` для четырёх баз (`rob_box_base:<имя>-buildcache`, это
   промежуточные стадии, сотни МБ–ГБ каждая). Если кеш apt-cacher-ng
   (`docker/build/data/apt-cache`) на том же разделе, запись в него может
   падать, и ошибка маскируется под 503. **Если это подтвердится, это
   побочный эффект PR**: нужен GC registry или вынос кеша, и это надо явно
   записать в PR.
2. **Upstream недоступен или троттлит** (`ports.ubuntu.com`, `packages.ros.org`)
   под параллельной нагрузкой: 4 холодные базы, потом 8+ сервисов сразу.
3. **Лимиты apt-cacher-ng**: `DlMaxRetries`, `NetworkTimeout`, число потоков.
   Конфиг, по гайду, `docker/build/config/acng.conf`, в compose он закомментирован.

## 4. Что сделать на katana (порядок)

Доступ описан в `docs/plans/2026-09-21-cicd-pipeline-handoff.md` §9 (там же
оговорка: `docker buildx ls` на хосте врёт про CI, раннеры — контейнеры).

```bash
# 1. Диск (гипотеза 1)
df -h / /var/lib/docker "$(docker inspect build-apt-cache -f '{{range .Mounts}}{{.Source}} {{end}}')"
curl -s localhost:5000/v2/krikz/rob_box_base/tags/list      # есть ли *-buildcache
du -sh <каталог registry> <каталог apt-cache>

# 2. Логи apt-cacher-ng в окне 12:55–13:20 UTC 28.09
docker logs --since 2026-09-28T12:50:00Z build-apt-cache 2>&1 | grep -i -E 'error|fail|503|retr|space' | tail -50
docker exec build-apt-cache sh -c 'tail -200 /var/log/apt-cacher-ng/apt-cacher.err'

# 3. Upstream с хоста и через прокси
curl -sI http://ports.ubuntu.com/ubuntu-ports/dists/jammy/InRelease | head -1
curl -sI http://packages.ros.org/ros2/ubuntu/dists/jammy/InRelease | head -1
curl -sI -x http://127.0.0.1:3142 http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-arm64/Packages.gz | head -1

# 4. Конфиг
docker exec build-apt-cache grep -E -i 'DlMaxRetries|NetworkTimeout|MaxConThreads|MaxDlSpeed' /etc/apt-cacher-ng/acng.conf

# 5. Билдеры в раннер-контейнерах (НЕ на хосте): должен быть robbox-<hostname>,
#    остаток старого `buildx use` (current=robbox-*) теперь безвреден.
for c in $(docker ps --format '{{.Names}}' | grep -i runner); do echo "== $c"; docker exec "$c" docker buildx ls | head -5; done
```

После починки: новый `workflow_dispatch` `L: Build All Services` на ветке PR
(`ref=claude/compassionate-hopper-uc56yt`). Ждём, что базы пройдут быстро
(registry-кеш уже записан), а сервисы соберутся полностью. Результат с run_id
отписать в PR #3100.

## 5. НЕ проверено

- Полный зелёный прогон `L: Build All Services` на ветке PR: Vision Pi не
  завершился ни разу, Main — частично.
- Что **базы** на втором прогоне берут registry-кеш. Для сервисов это видно
  (main rtabmap за 15 с), для баз прогона ещё не было.
- `scripts/build/build.py` реальной arm64-сборкой не гонялся (только
  `--dry-run`). Движок `buildx_build.sh` гонялся вживую в x86-песочнице
  (docker-container билдер + локальный registry), см. описание PR.
- Деплой образов с этой ветки на робота не делался.

## 6. Мелочи на потом (в PR не входят)

- Имена base-job'ов длинные (в них видны параметры матрицы): добавить
  `name: build-${{ matrix.name }}` в четыре job'а `L-Build Base Images.yml`,
  как у Vision Pi.
- `L-Build-All-Services.yml`: shellcheck SC2162 (`read` без `-r`), вне гейта actionlint.
- Не унифицированы: `.actrc`, `scripts/build/quick_docker_test.sh`,
  `scripts/testing/test_docker_local_arm64.sh`, `scripts/build/patch_*.sh`.

## 7. Процесс

- PR не мёржить — только Шифу.
- Подписка на события PR #3100 была в старой сессии. Новой сессии нужно
  подписаться заново (`subscribe_pr_activity krikz/rob_box_project 3100`).
- Если PR красный, отвечает за него владелец ветки: чинить и пушить в
  `claude/compassionate-hopper-uc56yt`, без force-push.

## 8. Suggested skills

- `handoff`: в конце сессии обновить этот файл или написать следующий.
- `code-review`: если по итогам §3 в PR добавится GC registry или правка
  apt-конфигурации.
- `run`: для локальной проверки `make build …` / `make lint`, если сессия
  будет на машине с docker.
