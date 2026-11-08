# Рут Яндекс Станции Лайт: ADB + SSH (runbook)

Дата: 2026-11-07. Выполнено на нашем железе (Станция Лайт 1-го поколения).
Опирается на исследование [2026-11-03-yandex-station-lite.md](../research/2026-11-03-yandex-station-lite.md).

## Итоговое состояние (к чему пришли)

| Что | Значение |
|---|---|
| Модель | Яндекс Станция Лайт 1 (`yandexmicro`, YNDX-00025), серийник `L002KBA0056SXP` |
| Прошивка | `0.369.1.46.4778262442.20261013.297` (уязвимая; откат с заводской `factory-4`) |
| IP колонки | `10.1.1.13` (DHCP от Cudy) |
| ADB root | `adb connect 10.1.1.13:5555` → `uid=0(root)` |
| SSH root | `ssh -p 2222 root@10.1.1.13` (ключ `~/.ssh/id_ed25519`, вход и без `-i`) |
| OTA | отключено (`yandex-noupdate.sh`: ключ OTA подменён, апдейтер заглушён) |
| Переживает перезагрузку | да: dropbear автозапуском `S99zdropbear`, маркеры ADB `chattr +i` |

ВНИМАНИЕ: фабричный сброс колонки стирает `/data` (dropbear, маркеры) — рут придётся ставить заново этим же runbook'ом.

## Топология сети (это важно — из-за неё дважды ходили не туда)

- Колонка и ПК-стенд — **в сети Cudy** `10.1.1.0/24` (роутер `10.1.1.1`).
- `FriendlyWrt` (`192.168.2.1`) — выше по цепочке (его WAN `192.168.1.100`), на нём **PassWall2**.
- Телефон с «Домом с Алисой» — в сети FriendlyWrt (`192.168.2.0/24`), приложение привязки — на телефоне, а Wi-Fi для колонки вводится вручную (сеть Cudy).
- **Ловушка 1:** редирект DNS на FriendlyWrt до колонки НЕ доходит — колонка берёт DNS у Cudy, а Cudy резолвит своим апстримом, не FriendlyWrt. Проверка: `nslookup quest.rob_box.local 10.1.1.1` до редиректа даёт `NXDOMAIN`.
- **Ловушка 2:** PassWall2 (`dns_redirect=1`) перехватывает DNS своей подсети; `server=/домен/...` в системном dnsmasq FriendlyWrt для клиентов LAN не срабатывает, пока PassWall2 включён. Точечно это чинится через PassWall2 `dns_hosts`, но для рута мы в итоге обошлись без FriendlyWrt вообще.

## Зачем вариант 2, а не 1

Вариант 1 (внешний unlock-сервер `179.254.167.126`) на сброшенной Лайт 1 **не работает**: заводской образ `factory-4` после `/register` ждёт откат прошивки, а сервер его не отдаёт (в исследовании помечено «не проверялось»). Использовали **вариант 2 — локальный стенд lite-kit**, который отдаёт `.46` и проверен именно для этого случая.

## Что нужно на ПК

- `adb` (platform-tools), `python` 3.8+.
- lite-kit: `git clone https://github.com/krevetkqqq/lite-kit C:\lite-kit`.
- dropbear (armhf, статический) из `Marker284/yandex-station-binaries`, каталог `bin/`.
- `Marker284/yandex-station-extended` — `contrib/yandex-noupdate.sh`.

## Порядок

### 1. Сброс колонки

Отключить питание → включить → **сразу** в момент подсветки зажать паузу (▶‖) на 6–10 с, пока не станет жёлто-зелёной → отпустить, пульсирует голубым (ждёт настройки).

### 2. Прошивка и lite-kit на ПК

```powershell
# прошивка .46 для yandexmicro (зеркало из install-firmware.ps1 бывает мёртвым — качай с официального URL из metadata.json)
curl.exe -L -o "C:\lite-kit\firmware\yandexmicro\0.369.1.46.4778262442.20261013.297.zip" `
  "https://quasar.s3.yandex.net/yandexmicro/ota/user/770338e8-a517-4642-a442-d26be2632873/0.369.1.46.4778262442.20261013.297.zip"
# sha256 = 74d4e8fae375b2ad1757761e01caf6be0209d216328ce1a942dbef38c8f20d11

cd C:\lite-kit
.\start.ps1        # UAC (админ): правила брандмауэра + порты 53/80/443; сам печатает IP стенда
```

Сервер слушает 53 (DNS), 80, 443. В логе `log\server.log`. Строка «в firmware/ нет .zip» — это про старую плоскую раскладку; раскладка `firmware/<platform>/<версия>.zip` отдаётся нормально.

### 3. DNS на роутере, который обслуживает колонку

В web-морде **Cudy** (`10.1.1.1`) поставить DNS локальной сети = **IP ПК-стенда** (`10.1.1.37`). Проверка с ПК: `nslookup quasar.yandex.net 10.1.1.1` должен вернуть IP стенда.

### 4. Привязка

В «Доме с Алисой»: «+» → «Устройство с Алисой» → до экрана Wi-Fi → ввести сеть Cudy и пароль → нажать «Подключиться». Колонка сходит на lite-kit: `/register` → `check_updates` → скачает `.46` (66,8 МБ) → поставит и перезагрузится. Признак в `log\server.log`: `>>> [OTA] файл отдан ПОЛНОСТЬЮ (66814464 б)`.

### 5. ADB root

```powershell
adb connect 10.1.1.13:5555
adb devices            # device
adb shell id           # uid=0(root)
```

### 6. Закрепить (пока ADB жив)

```powershell
# маркеры ADB + защита от удаления
adb shell 'mkdir -p /data/quasar && touch /data/quasar/activate_adb_quasmodrom /data/quasar/activate_network_adb && chattr +i /data/quasar/activate_adb_quasmodrom /data/quasar/activate_network_adb'

# отключить OTA (скрипт перезалить с LF! — см. грабли)
adb push contrib\yandex-noupdate.sh /data/opt/yandex-noupdate.sh
adb shell sh /data/opt/yandex-noupdate.sh        # «updatesd убран / applier заглушён / ota key подменён»
adb shell reboot

# после перезагрузки — dropbear на 2222
adb shell mkdir -p /data/opt/bin /data/opt/etc/dropbear
adb push dropbear    /data/opt/bin/dropbear
adb push dropbearkey /data/opt/bin/dropbearkey
adb shell chmod +x /data/opt/bin/dropbear /data/opt/bin/dropbearkey
adb shell mount -o remount,rw /
adb push S99zdropbear /etc/init.d/S99zdropbear     # скрипт ниже
adb shell chmod +x /etc/init.d/S99zdropbear
adb shell 'sync; mount -o remount,ro /'
adb shell '/etc/init.d/S99zdropbear start'         # сеанс «повиснет» — это нормально, прерви

# ключ входа
adb shell mount -o remount,rw /
adb shell 'mkdir -p /root/.ssh && echo "<публичный ключ>" > /root/.ssh/authorized_keys && chmod 700 /root/.ssh && chmod 600 /root/.ssh/authorized_keys'
adb shell 'sync; mount -o remount,ro /'
```

`S99zdropbear` (LF, не CRLF):

```sh
#!/bin/sh
# Autostart dropbear SSH on :2222 (delete this file to remove).
DB=/data/opt/bin/dropbear
DIR=/data/opt/etc/dropbear
HK=$DIR/hk_ed25519
LOG=$DIR/dropbear.log
start() {
  [ -x "$DB" ] || { echo "$(date) no $DB" >> "$LOG"; return 1; }
  [ -s "$HK" ] || /data/opt/bin/dropbearkey -t ed25519 -f "$HK" >> "$LOG" 2>&1 || return 1
  pidof dropbear >/dev/null 2>&1 && return 0
  "$DB" -r "$HK" -p 2222 -B 2>>"$LOG"
}
case "$1" in
  start)   start ;;
  stop)    killall dropbear 2>/dev/null ;;
  restart) killall dropbear 2>/dev/null; sleep 1; start ;;
esac
exit 0
```

Проверка: `ssh -p 2222 root@10.1.1.13 id` → `uid=0(root)`; после reboot — снова.

### 7. Откат DNS и остановка стенда

1. Вернуть DNS на Cudy как было (авто/провайдерский).
2. Остановить lite-kit: процесс `pythonw` (equasar_server) запущен elevated — из обычной консоли не убивается, убивать через UAC (`Start-Process powershell -Verb RunAs` → `Stop-Process`).
3. **Перезагрузить Cudy** — у него остаётся закешированный DNS (quasar → старый unlock-сервер), после reboot всё встаёт на место.

## Вторая колонка: что отличалось (2026-11-08)

Колонка `10.1.1.141` (серийник `LN00000000000020401900000cbd5cce`) — другая ревизия железа, чем первая
(`10.1.1.13`, Amlogic). Расхождения, которые пришлось учитывать:

| Что | Первая (`10.1.1.13`) | Вторая (`10.1.1.141`) |
|---|---|---|
| Заводская прошивка | `0.5.1.20…factory-4` | `0.4.1.11…factory-3` (2022) — стенд отдаёт `.46` так же |
| `/data` | больше | **89 МБ всего**, свободно 77 МБ до установки |
| `uname -m` | `aarch64` | `aarch64`, но userland 32-бит: `/lib/ld-linux-armhf.so.3`, `lib32 -> lib` — armhf-бинарники подходят |
| Модели KWS | `rbx/kws`, `rbx/kws_real` (метки `🔑 kws` в логе) | нет — вейк работает по словам из `RBX_WAKE`, метки не печатаются |
| `getprop` | нет | нет (`-sh: getprop: not found`) — скрипты, которые его зовут, получат пустоту |

Особенности busybox на этой прошивке: `tar` **без `-z`** (только `tar cf - | gzip -c`), `netstat` без `-p`,
`timeout` нет, `scp` нет (файлы гонять `ssh … 'cat > file'` или потоком `tar | ssh 'tar xf -'`).

Установка хоста и всё про ключи/настройки — [station-igor-install.md](station-igor-install.md).

## Грабли (всё поймано руками)

- **CRLF после `git clone` на Windows**: скрипты, залитые через `adb push`, падают на busybox с `illegal option -`. Чинить: `[IO.File]::ReadAllText(...).Replace("`r`n","`n")` и заливать с LF.
- **`adb shell '/etc/init.d/S99zdropbear start'` не завершается** — dropbear уходит в фон, не закрыв stdout. Прервать сеанс (Ctrl+C), сервер продолжает работать.
- **`stop.ps1` не гасит сервер из не-админской консоли** (сервер elevated, `server.pid` пустой) — убивать PID через UAC.
- **«Нет доступа к интернету» на колонке** в начале привязки — бывает; проверка `HEAD /generate_204` в логе lite-kit должна дать 204, дальше прогон идёт сам.
- **PassWall2 перехватывает DNS**: `server=` в системном dnsmasq не виден клиентам LAN, пока `dns_redirect=1`. После отката PassWall2 нужно перезапустить/выключить, иначе остаётся устаревший ответ из его sing-box.
- **Стенд lite-kit захлёбывается флудом (08.11)**: его встроенный DNS-сервер обрабатывал UDP-запросы **в одном потоке**, а первый апстрим `1.1.1.1` не отвечал — каждое незнакомое имя блокировало цикл до 6 с. Cudy льёт на стенд все имена → очередь росла, `quasar.*` умирали по таймауту, и Cudy отвечал провайдерским DNS. Лечение в локальной копии: поток на запрос, порядок апстримов `77.88.8.8, 8.8.8.8, 1.1.1.1`, таймаут 1,2 с.
- **`Internet Connection Sharing` (svchost `SharedAccess`) держит UDP :53** рядом со стендом (его поднимает Hyper-V «Default Switch»). Пока ICS жив, UDP стенда может «умереть» при его рестарте: проверять `Get-NetUDPEndpoint -LocalPort 53`, при конфликте — перезапустить стенд.
- **DHCP-раздача DNS = IP стенда надёжнее, чем DNS на Cudy**: Cudy гоняет два резолвера (наш стенд и провайдерский), и `quasar.yandex.net` через раз уходил в интернет — в логе стенда видно, что `.ru`/`scbh` он спрашивает, а `.net` нет. `Preferred DNS = <IP стенда>` в DHCP (Alternate пусто) убирает гонку: клиент спрашивает стенд напрямую, Cudy в резолве не участвует. Проверка: `ipconfig /renew` → `DNS Servers: 10.1.1.37`.
- **Перезагрузка Cudy не помогает, если DNS прописан в DHCP**: перезагрузка чистит только кэш резолвера; саму раздачу проверять после `ipconfig /renew`.

## Ссылки

- Установка и обновление хоста Игорька (ключи, настройки, две колонки, 429): [station-igor-install.md](station-igor-install.md)
- Исследование: [docs/research/2026-11-03-yandex-station-lite.md](../research/2026-11-03-yandex-station-lite.md)
- hackalice: https://hackalice.me/start/root/ , https://hackalice.me/start/ssh/
- lite-kit: https://github.com/krevetkqqq/lite-kit
- yandex-station-extended: https://github.com/Marker284/yandex-station-extended
- dropbear: https://github.com/Marker284/yandex-station-binaries
