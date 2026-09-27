# Hailo-8 DKMS root cause — `/dev/hailo0` пропадает после ребута (#3089, #3090)

Дата: 2026-09-27

## TL;DR

- Деплой `L-Deploy and Verify` на develop/staging падал на двух местах: голый `sudo` по non-TTY ssh и backtick command substitution в heredoc. Починено, деплой зелёный (run `36338971222`, 5m24s).
- На Vision Pi найден root cause «флапающего» Hailo-драйвера: `unattended-upgrades` ставит новый kernel, а `linux-headers-raspi` **не установлен** → DKMS не может пересобрать `hailo_pci.ko` под новый kernel → после ребута `/dev/hailo0` пропадает → vision-hailo не стартует.
- Текущее состояние робота: драйвер восстановлен и **персистентен** (модуль на диске + modalias), все контейнеры `healthy`.

---

## 1. Что чинили (деплой)

Падавшие раны и причины:

| Run | Шаг | Причина | Fix |
|---|---|---|---|
| `36337848209` | Reinstall Systemd Units + Health Timer (#3018) | голый `sudo` по non-TTY ssh (`sudo: a terminal is required`) + backtick command substitution в heredoc `setup_autostart` (`docker compose ps` → `no configuration file provided: not found`) | `781d5f824` |
| `36338793006` | Ensure Hailo PCIe driver (#3090) | голый `sudo bash scripts/setup/ensure_hailo_driver.sh` | `cc551ef79` |
| `36337117989` | Start Containers | `error gathering device information while adding custom device "/dev/hailo0": no such file or directory` | recovery через `ensure_hailo_driver.sh` (#3092) |

Изменения (develop):
- `781d5f824` — `reinstall_vision_pi_systemd.sh`: кэширование sudo-креденшелов через `sudo -v` (`SUDO_PASSWORD=open`); экранированы backticks в heredoc `setup_autostart`; workflow передаёт `SUDO_PASSWORD` + `echo "open" | sudo -S systemctl daemon-reload`; регресс-тест `test_setup_autostart_heredoc_has_no_command_substitution`.
- `cc551ef79` — `Ensure Hailo PCIe driver`: `sudo bash` → `echo 'open' | sudo -S bash`.

Верификация: [run 36338971222](https://github.com/krikz/rob_box_project/actions/runs/36338971222) — все шаги ✔.

---

## 2. Что нашли на роботе (Vision Pi, 10.1.1.21, через Katana)

| Проверка | Результат |
|---|---|
| kernel | `6.8.0-1065-raspi` |
| PCI | `0000:01:00.0 ... Hailo-8 [1e60:2864]` ✅ |
| `/dev/hailo0` | есть, `crw-rw-rw-` (создан 21:01, в момент деплоя) ✅ |
| `lsmod` | `hailo_pci 126976 6` загружен ✅ |
| `dkms status` | `hailo_pci/4.24.0, 6.8.0-1065-raspi, aarch64: installed` ✅ |
| модуль на диске | `/lib/modules/6.8.0-1065-raspi/updates/dkms/hailo_pci.ko.zst` (собран 21:00) ✅ |
| `modinfo -F filename hailo_pci` | `/lib/modules/6.8.0-1065-raspi/updates/dkms/hailo_pci.ko.zst` ✅ |
| `modules.alias` | `alias pci:v00001E60d00002864... hailo_pci` → авто-загрузка после ребута ✅ |
| containers | все `Up (healthy)`: vision-hailo, vision-face, oak-d, voice-*, telegram-bot, quest… ✅ |

Load ~9.67 в момент захода — не restart-loop, а нормальная нагрузка (`vision_face` 33%, `camera_node` 25%, `zenohd` 19%, `mcp_server` 17%, `vision_hailo` 14%).

---

## 3. Root cause (это #3090)

Цепочка:

1. На Vision Pi включён **`unattended-upgrades`** (`APT::Periodic::Unattended-Upgrade "1"` в `/etc/apt/apt.conf.d/20auto-upgrades`) — сам ставит новые kernel-имиджи (`linux-image-raspi` → `linux-image-6.8.0-XXXX-raspi`).
2. Metapackage **`linux-headers-raspi` НЕ установлен** (`dpkg -l | grep linux-headers-raspi` → пусто). Значит headers для нового kernel не приезжают.
3. Kernel обновился `1064 → 1065-raspi`. DKMS postinst-хук (`/etc/kernel/postinst.d/dkms`) есть, но **без headers не может пересобрать `hailo_pci.ko`** под `6.8.0-1065-raspi`.
4. После ребута в `1065` модуль отсутствует → `/dev/hailo0` не создаётся → `docker compose up vision-hailo` падает на device-mount.

То есть это не «драйвер исчез», а «kernel обновился без headers → DKMS не достроил модуль». Каждый следующий unattended kernel-апгрейд будет повторять поломку, пока не поставлен `linux-headers-raspi`.

Recovery (уже работает): deploy-шаг `[Vision Pi] Ensure Hailo PCIe driver` → `scripts/setup/ensure_hailo_driver.sh` ставит `linux-headers-$(uname -r)` + `dkms autoinstall` + `modprobe` → возвращает `/dev/hailo0`.

---

## 4. Prevention (fix #3090, НЕ сделан)

Чтобы не повторялось после следующего kernel-апгрейда:

1. **Установить `linux-headers-raspi` на Vision Pi** — headers будут приезжать вместе с kernel, DKMS пересоберёт модуль сам.
2. Опционально: watchdog на `/dev/hailo0` после boot (alert при пропаже).

Реализация — через репозиторий (`scripts/setup/setup_node.sh` → `setup_hailo_ai_hat()` и/или `ensure_hailo_driver.sh`) и деплой, НЕ правками напрямую на роботе.

---

## 5. Итоги по issue

| Issue | Статус |
|---|---|
| #3097 (deploy tracker) | closed — деплой зелёный |
| #3095 (SSH permission denied) | closed — транзиент, не воспроизводится |
| #3089 (critical: vision не стартует) | closed — recovery + deploy-time prevention выполнены и проверены |
| #3090 (root cause, tech-debt) | **open** — root cause найден, prevention (`linux-headers-raspi` + watchdog) не реализован |
