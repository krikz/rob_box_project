# Вендорный пак HailoRT — как получить, как разложить, как засеять новый хост

Закрытые артефакты HailoRT (`.deb`, `.whl`, `.so`) **не лежат в git и не
зеркалируются никуда**. Канонический экземпляр живёт в `/opt/rob_box/vendor/`
на build-хосте. Этот документ — единственная инструкция, как его собрать
самому и как починить ситуацию, когда пак недоступен.

Решение не размножать эти файлы принято владельцем проекта 2026-09-16:
артефакты Hailo Developer Zone выдаются по аккаунту, и проект не создаёт
новых способов их распространения. Разбор альтернатив (приватный GHCR,
git-lfs, release-ассеты) — `docs/plans/2026-09-15-resource-pack.md` §4.3,
вариант **(б)**.

> Следствие, которое надо принимать осознанно: `/opt/rob_box/vendor` на
> katana — единственная копия. При потере диска пак восстанавливается
> только повторным заходом в Developer Zone по этой инструкции.

---

## 1. Состав пака

Версия HailoRT, на которую сегодня закреплён проект — **4.24.0**
(`scripts/setup/setup_node.sh:708`, `docker/vision/vision-hailo/Dockerfile:123`).

| Файл | Откуда берётся | Кому нужен |
|---|---|---|
| `hailort-pcie-driver_4.24.0_all.deb` | Developer Zone, скачивание | **Vision Pi**, хостовый DKMS-драйвер |
| `hailort_4.24.0_arm64.deb` | Developer Zone, скачивание | **Vision Pi** (`hailortcli`) и как источник `.so` |
| `hailort-4.24.0-cp310-cp310-linux_aarch64.whl` | Developer Zone, скачивание | **build-хост**, Python-биндинг `hailo_platform` в образ |
| `libhailort.so.4.24.0` | **извлекается вручную** из `hailort_4.24.0_arm64.deb` | **build-хост**, рантайм-библиотека в образ |

Четвёртый файл — самый неочевидный: wheel содержит только Python-обвязку,
но не саму библиотеку (`docker/vision/vision-hailo/Dockerfile:115`).
Скрипта, который бы её извлекал, в проекте нет — шаг ручной, см. §3.

---

## 2. Скачивание из Developer Zone

1. Завести аккаунт на <https://hailo.ai/developer-zone/> (регистрация
   обязательна, анонимной ссылки на эти файлы не существует — именно
   поэтому они не скачиваются скриптом).
2. Раздел **Software Downloads → HailoRT**, выбрать версию **4.24.0**
   и платформу **Hailo-8 / aarch64**.
3. Скачать три файла из таблицы §1 (те, что помечены «Developer Zone,
   скачивание»).

Если берёшь другую версию HailoRT — менять придётся не только имена
файлов, но и `hailo_version` в `scripts/setup/setup_node.sh:708`, и имена
в `docker/vision/vision-hailo/Dockerfile:123-125`, и в обоих
build-workflow (`L-Build Vision Pi Services.yml:512`,
`L-Build Single Service.yml:489`). Версия зашита в пяти местах — это
известный техдолг, его закрывает манифест ресурсов из
`docs/plans/2026-09-15-resource-pack.md`.

---

## 3. Извлечение `libhailort.so.4.24.0` из `.deb`

Ручной шаг, выполняется один раз на любой Linux-машине (не обязательно
arm64 — распаковка архива, не установка):

```bash
mkdir -p /tmp/hailort-extract
dpkg-deb -x hailort_4.24.0_arm64.deb /tmp/hailort-extract
find /tmp/hailort-extract -name 'libhailort.so*'
```

Забирать нужно файл с полной версией в имени — `libhailort.so.4.24.0`,
а не симлинк `libhailort.so`. Образ делает симлинк сам
(`docker/vision/vision-hailo/Dockerfile:125`).

---

## 4. Раскладка по хостам

### 4.1 Build-хост (katana)

Нужны два файла — wheel и `.so`:

```bash
sudo mkdir -p /opt/rob_box/vendor
sudo cp hailort-4.24.0-cp310-cp310-linux_aarch64.whl /opt/rob_box/vendor/
sudo cp libhailort.so.4.24.0 /opt/rob_box/vendor/
sudo chmod 0644 /opt/rob_box/vendor/*
```

Отсюда их забирает шаг `Fetch HailoRT wheel + lib` перед сборкой
`vision-hailo` (`L-Build Vision Pi Services.yml:507-522`). Есть fallback
на `/tmp/` — хостовый `/tmp` проброшен в раннеры как `/tmp:/tmp`; это
аварийный путь, канонический — `/opt/rob_box/vendor`.

### 4.2 Vision Pi

Нужны оба `.deb`:

```bash
sudo mkdir -p /opt/rob_box/vendor
sudo cp hailort-pcie-driver_4.24.0_all.deb /opt/rob_box/vendor/
sudo cp hailort_4.24.0_arm64.deb /opt/rob_box/vendor/
```

Дальше их ставит `setup_hailo_ai_hat()` в `scripts/setup/setup_node.sh:707`
(DKMS-сборка драйвера + `hailortcli`). Функция сама проверяет наличие
железки через `lspci` и молча выходит, если AI HAT не воткнут.

---

## 5. Проверка

```bash
# Build-хост: оба файла на месте
ls -la /opt/rob_box/vendor/hailort-4.24.0-cp310-cp310-linux_aarch64.whl \
       /opt/rob_box/vendor/libhailort.so.4.24.0

# Vision Pi: драйвер поднялся и видит чип
lspci | grep -i hailo
hailortcli scan          # ожидается устройство hailo8
```

`hailortcli scan`, возвращающий `hailo8` — это acceptance Phase 1 по
ADR-0089 §9. Импорт `hailo_platform` в acceptance **не входит**: без
биндинга нода работает в stub-режиме с логом (ADR-0099 §2.2).

### Контрольные суммы

Эталонные sha256 в проекте пока **не зафиксированы** — ни для одного из
четырёх файлов. Снять их надо один раз, при следующем контакте с
Developer Zone, и записать в манифест ресурсов, когда он появится:

```bash
sha256sum hailort-pcie-driver_4.24.0_all.deb \
          hailort_4.24.0_arm64.deb \
          hailort-4.24.0-cp310-cp310-linux_aarch64.whl \
          libhailort.so.4.24.0
```

До тех пор «тот ли это файл» проверяется только по имени и версии.

---

## 6. Что происходит, когда пака нет

| Где | Поведение сегодня | Что делать |
|---|---|---|
| **Vision Pi**, `setup_node.sh:731-738` | Печатает предупреждение с перечнем недостающих `.deb` и путём, куда их положить, и выходит с кодом 0 — setup продолжается | Выполнить §2 и §4.2, перезапустить setup |
| **Build-хост**, `L-Build Vision Pi Services.yml:517-519` | `::error::<файл> не найден ни в /opt/rob_box/vendor, ни в /tmp` и **`exit 1`** — job падает, вся сборка vision не проходит | Выполнить §2, §3 и §4.1 на katana |
| **Рантайм** в собранном образе без биндинга | Нода стартует в stub-режиме и пишет причину в лог, restart-loop'а нет (`docker/vision/scripts/vision-hailo/start_vision_hailo.sh`, ADR-0099 §2.2) | Пересобрать образ после засева build-хоста |

Жёсткое падение сборки — сознательное: образ `vision-hailo` без Hailo
бесполезен, и узнать об этом лучше в CI, чем найти stub на роботе.

---

## 7. Засев нового build-хоста

Автоматики нет — процедура ручная, и это принятая цена решения не
зеркалировать вендорные файлы:

1. §2 — скачать три файла из Developer Zone (нужен аккаунт).
2. §3 — извлечь `libhailort.so.4.24.0` из `.deb`.
3. §4.1 — разложить wheel и `.so` в `/opt/rob_box/vendor/`.
4. §5 — проверить наличие и снять sha256.
5. Прогнать сборку `vision-hailo` через `L-Build Single Service.yml`
   и убедиться, что шаг `Fetch HailoRT wheel + lib` прошёл.

---

## Связанное

- ADR-0089 — AI HAT+ Hailo-8 deployment, фазы и acceptance
- ADR-0099 — стратегия установки биндинга, capability-honest stub
- ADR-0018 — честная деградация вместо тихого падения
- `docs/plans/2026-09-15-resource-pack.md` — манифест ресурсов, §4 про
  закрытые артефакты
- `scripts/setup/setup_node.sh:707` — `setup_hailo_ai_hat()`
- `docker/vision/vision-hailo/Dockerfile:113-131` — установка биндинга
