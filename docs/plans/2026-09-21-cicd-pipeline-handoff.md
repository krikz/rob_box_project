# Хендофф — конвейер сборки и доставки, 2026-09-21

> **Кто писал:** Claude Opus 5. Доступ к стенду katana (по Tailscale,
> `100.71.58.103`) и к Vision Pi (через katana) был, к Main Pi — не
> использовался.
> **База:** `develop` @ `a3014bda5`, ветка `feature/cicd-pipeline-redesign`,
> 50 коммитов сверху, `+11111 / −2270`.
> **Честность (ADR-0018):** у каждого «зелёно» ниже стоит run_id или прогон
> с числами. §6 перечисляет ровно то, что НЕ проверялось, и это не мелочи —
> деплоя на робота не было ни одного.

---

## 1. Что это было

Четыре плана от 15.09 (`docs/plans/2026-09-15-*.md`) доведены до кода.
Сверх планов — два решения владельца, принятых по ходу, каждое с ADR.

| план | статус |
|---|---|
| `builder-runtime-seam` | Этапы 0-5 сделаны. Общий builder-образ для пар сознательно НЕ делался — аргумент за него переписан по факту замеров (§3.2) |
| `service-manifest` | Phase 1-3 сделаны. Phase 4 (`L-Build Single Service.yml`) вне объёма |
| `resource-pack` | Этапы 1-4 сделаны, плюс сверх плана Renardo-сэмплы и Vosk для telegram-bot. Этап 5 (уборка) ждёт N успешных деплоев |
| `image-versions-seam` | вердикт «не переоткрывать ADR-0094» подтверждён; §7.2-7.5 закрыты, §7.1 сознательно не трогался |

Новые ADR: **0125** (Ресурсный пак) и **0126** (Renardo-сэмплы, заменяет
ADR-0111 §2.1).

## 2. Главный результат — в секундах

Полная сборка обоих Pi (18 сервисов), `L: Build All Services`, run
`35642624956`:

| | холодная | с кешем |
|---|---|---|
| **весь стек, Main + Vision** | — | **9 м 07 с** |
| `voice-base` | 40 м 07 с | **59 с** |
| `ros2-control` | 9 м 13 с (до починки вис 68+ мин) | **55 с** |
| `nav2` | 19 м 38 с | 43 с |
| `rtabmap` / `perception` / `lslidar` | 0:51 / 10:19 / 8:47 | 50 с / 49 с / 64 с |

До 21.09 registry-кеш не писался **вообще** — в каждом job'е стояло
`Cache: … (from only)`. Почему — §3.1.

## 3. Три ловушки, которые нашёл только стенд

### 3.1. Кеш был подключён и ничего не делал

`--cache-to` молча пропускался: на дефолтном драйвере buildx `docker`
экспорт кеша не поддерживается вовсе, и `ignore-error=true` не спасает —
это проверка фич драйвера, она срабатывает до старта сборки.

Переключить драйвер «на стенде» нельзя: раннеры на katana — это
**контейнеры** (`myoung34/github-runner`, восемь штук, docker.sock с хоста),
у каждого свой `~/.docker`. Билдер теперь заводится шагом в самом
`.github/actions/l-build-service` — единственное место, которое переживает
и пересоздание раннера, и правку хоста.

Дальше выяснилось, что `--add-host=host.docker.internal:host-gateway` —
фича демона docker, и `docker-container` её не знает: первый прогон с новым
билдером уронил все восемь job'ов за две минуты. Лечится подменой на
реальный IP шлюза bridge-сети.

### 3.2. Экономия размера от шва оказалась на порядок меньше обещанной

`dpkg -l` внутри `ros:humble-ros-base`: `ament-cmake`,
`rosidl-default-generators`, `nav-msgs` **уже там**. Для
`perception`/`vision-hailo` шов не убирает с уровня apt ничего (842 → 833 МБ).
У `ros2_control` ≈ ноль: 59 МБ моделей `rob_box_description` пересекают шов
в любом случае.

Ценность шва — изоляция (сетевой `git clone` у lslidar, закрытые артефакты
у vision-hailo) и корректность, а не байты. Продавать его как экономию
нельзя. Подробности — `2026-09-15-builder-runtime-seam.md` §13.

### 3.3. `apt` под qemu висел 68 минут на нуле CPU

```
PID      ELAPSED   TIME      COMMAND
3822692  4075      00:00:00  apt-get         ← 68 минут, НОЛЬ секунд CPU
ss -tnp: CLOSE-WAIT Recv-Q=1  172.17.0.1:42884 → 172.17.0.1:3142
```

Причина — HTTP-конвейер: и apt, и apt-cacher-ng держат `Pipeline-Depth 10`,
метод apt под qemu-user застревает на закрытии соединения. Слова
`Pipeline-Depth` в репозитории не было ни в одном из 11 мест, где пишется
`02proxy`. После `Acquire::http::Pipeline-Depth "0"` тот же шаг — **124.7 с**.

**Метод, который стоит унести с собой:** под эмуляцией «висит» и «медленно
работает» выглядят одинаково. Различает их только `TIME`, а не `ELAPSED`.

## 4. Что ещё изменилось, кроме планов

- **Три копии `verify_in_registry`** (оба L-Build + Single Service) сведены в
  `scripts/ci/verify-image-in-registry.sh`. Они уже разъехались на строку
  сообщения — синхронность держалась ни на чём.
- **Phantom-теги удалены** (`MICRO_ROS_AGENT_TAG`, `RTABMAP_SYNC_TAG`) вместе
  с источником — мёртвой веткой `PI_TYPE=vision` у rtabmap в Single Service.
  Сторож `check_image_versions_usage.sh` подключён hard gate'ом (ADR-0094 §3.2
  ждал этого с момента написания).
- **Три места, где тесты есть, а CI их не гонял**, подключены в
  `G-Lint Code.yml`: `test_l_build_service_composite.py`, `tests/unit/docker/`,
  новый `test_verify_image_in_registry.py`. Плюс `actionlint` hard gate'ом на
  двух build-workflow.
- **Образ `voice-resources` удалён целиком** (ADR-0126): сэмплы едут
  Ресурсным паком в `/opt/rob_box/samples` и приходят bind-mount'ом.
- **CRLF ломал парсер манифеста молча** — на Pi (mawk) он разбирал одну
  запись из десяти и продолжал. На Windows баг не воспроизводится, негативный
  контроль гоняется только в linux-контейнере.

## 5. Что проверено, с чем сверяться

| проверка | результат |
|---|---|
| `L: Build All Services` run `35642624956` | **зелёный**, 18 сервисов, обе `update-image-versions` |
| `L: Build Vision Pi Services` run `35636286658` | зелёный, 11 сервисов (до удаления voice-resources) |
| `L: Build Main Pi Services` run `35635018766` | зелёный, 8 сервисов |
| `G: Lint Code` run `35631181914` | зелёный, гейты: 23+28+94+22+7+39 passed |
| локальный guard-набор | **234 passed, 2 skipped** |
| `check_image_versions_usage.sh` | exit 0 |
| Vision Pi, модели | Vosk+Silero разложены, второй прогон `no-op … sha256 совпал` |
| Vision Pi, сэмплы | 885 МБ мигрированы из volume в `/opt/rob_box/samples`, хук → `OK no-op`, ноль сетевых запросов |
| Vision Pi, HEF | sha256 `retinaface` на роботе совпал с манифестом |

Команда, воспроизводящая локальный набор:

```bash
python -m pytest scripts/ci/tests/ tests/unit/test_workflow_refactor_acceptance.py \
  tests/unit/docker tests/vision -q -o addopts="" -p no:cacheprovider
bash scripts/ci/check_image_versions_usage.sh
```

## 6. Что НЕ проверялось

1. **Деплой на робота не запускался ни разу.** Значит не подтверждено:
   что `voice-assistant` видит `/models` через bind-mount, что `stt_node` и
   `tts_node` поднимаются с моделями с хоста, что музыка играет из
   `/opt/rob_box/samples`. Пока это не прогнано, Этап 3 ресурсного пака и
   ADR-0126 готовыми считать нельзя.
2. **Ветка `HAILO_INSTALL_BINDING=whl`** — вендорского пака при сборке не
   было, блок перенесён байт-в-байт, но в рантайм-стадии ни разу не исполнен.
3. **Реальное скачивание сэмплов** с `collections.renardo.org` — тесты
   гоняют локальный HTTP-сервер, на роботе сработал no-op по маркеру.
4. **`size_bytes: 629145600`** у `renardo-samples` — оценка из ADR-0111 §6,
   не измерение (фактический каталог на роботе — 885 МБ).
5. **Main Pi** в этой работе по ssh не трогался.

## 7. Очередь — решения владельца

| что | почему ждёт человека |
|---|---|
| **Деплой на робота** | без него §6.1 висит |
| **Registry на katana: 62.5 ГБ**, диск 96% | каждый SHA-тег хранится вечно, GC нет; на старые теги могут ссылаться `.image-versions` прошлых деплоев |
| **Старый volume `vision_renardo_samples`** (885 МБ) | оставлен как страховка; удалять только после того, как музыка заиграет с хоста: `docker volume rm vision_renardo_samples` |
| **Размер ветки** | 50 коммитов, +11111 строк при лимите AF-0013 в 3000 на PR. Чисто независимых кусков нет: чистка phantom-тегов опирается на Phase 3, кеш и билдер живут в одном файле — получится стек из 3-4 PR, мержащихся по порядку |

## 8. Очередь — техническая

- **Phase 4**: `L-Build Single Service.yml` не просто дублирует список
  сервисов — он **не использует композит вовсе**, у него свой
  `docker buildx build`. Поэтому кеш, билдер и подмена host-gateway на
  одиночные сборки не распространяются.
- **Общий builder-образ** для пар `vesc_nexus`↔`ros2_control` и
  `perception`↔`vision-hailo`: оценивать как экономию CI-времени (повторная
  компиляция rosidl — минуты под qemu), не размера.
- **`torch.hub`-фолбэк silero** удалён вместе со ступенью Dockerfile и ничем
  не заменён — если оба файловых пути отвалятся, `tts_node` полезет в сеть в
  рантайме (`resource-pack` §13.5).
- **Этап 5 ресурсного пака**: удалить `download_yolov8n_hef.sh` и
  `download_retinaface_hef.sh` (193 строки) после N успешных деплоев.
- **Два эталона sha256 закрытых артефактов утрачены физически** — на Vision Pi
  каталога `/opt/rob_box/vendor` нет, пакеты установлены, `.deb` не сохранены.
  Аргумент к зеркалу (`resource-pack` §4.3).
- **Устаревшая проза**: `docs/CI_CD_PIPELINE.md` упоминает `build-oak-d` как
  отдельный job; `VOICE_ASSISTANT_DOCKER.md:17` и
  `DOCKER_BUILD_OPTIMIZATION.md` называют Vosk «bundled in the image».

## 9. Как проверить состояние за пять минут

```bash
# 1. Гварды локально (см. §5) — должно быть 234 passed
# 2. Что реально лежит на роботе
ssh ros2@10.1.1.21 'ls /opt/rob_box/models /opt/rob_box/samples'
# 3. Идемпотентность пака: второй прогон обязан быть no-op без сети
ssh ros2@10.1.1.21 'sudo bash ~/rob_box_project/docker/vision/scripts/resource_pack/apply_resource_pack.sh --dry-run'
# 4. Кеш живой? В логе любого build-job'а должно быть "(from + to, mode=max"
gh run view <run_id> --log --job <job_id> | grep "Cache:"
# 5. Драйвер билдера на раннере — НЕ смотреть с хоста katana:
#    docker buildx ls на хосте покажет default/docker и соврёт про CI.
```

**Доступ:** katana — `ros2@100.71.58.103` (Tailscale) или
`ros2@ros2-katana-gf66-11ud`, пароль `open`, с Windows через
`plink`/`pscp`. Vision Pi — `10.1.1.21` через katana.
Подробности — `docs/deployment/` и заметки в ADR-0125 §7.
