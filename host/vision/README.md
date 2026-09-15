# ROBBOX Vision Pi — host-level systemd units

Хост-уровневые юниты и drop-in'ы для Vision Pi (10.1.1.21, RPi 5, 8 GB RAM).

Эти файлы НЕ копируются автоматически. Оператор устанавливает их вручную
**один раз** при первичной настройке или при миграции с dphys-swapfile на
zram.

## Содержимое

| Файл | Куда ставить | Зачем |
|---|---|---|
| `robbox-zram.service` | `/etc/systemd/system/robbox-zram.service` | Поднимает 4 GB zram-swap при загрузке |
| `ssh-memory-low.conf` | `/etc/systemd/system/ssh.service.d/10-robbox-memory-low.conf` | `MemoryLow=128M` для sshd |

## Установка вручную

```bash
# zram-swap
sudo cp host/vision/robbox-zram.service /etc/systemd/system/robbox-zram.service
sudo cp scripts/setup/setup_vision_pi_swap.sh /opt/rob_box_project/scripts/setup/setup_vision_pi_swap.sh
sudo chmod +x /opt/rob_box_project/scripts/setup/setup_vision_pi_swap.sh
sudo systemctl daemon-reload
sudo systemctl enable --now robbox-zram.service
sudo /opt/rob_box_project/scripts/setup/setup_vision_pi_swap.sh --status

# ssh MemoryLow
sudo mkdir -p /etc/systemd/system/ssh.service.d/
sudo cp host/vision/ssh-memory-low.conf /etc/systemd/system/ssh.service.d/10-robbox-memory-low.conf
sudo systemctl daemon-reload
sudo systemctl restart ssh.service
sudo systemctl show ssh | grep -E "MemoryLow|MemoryHigh"
```

## Установка автоматическая (рекомендуемая)

```bash
sudo bash scripts/setup/setup_vision_pi_swap.sh --auto
```

Скрипт `setup_vision_pi_swap.sh` идемпотентен — можно запускать повторно.

## Дополнительные override через `/etc/default/robbox-zram`

```bash
# Пример: уменьшить zram до 2 GB, использовать lz4 (быстрее, но хуже сжатие)
echo 'ROBBOX_ZRAM_SIZE_MB=2048' | sudo tee /etc/default/robbox-zram
echo 'ROBBOX_ZRAM_ALGO=lz4' | sudo tee -a /etc/default/robbox-zram
sudo systemctl daemon-reload
sudo systemctl restart robbox-zram.service
```

## Проверка

```bash
# zram активен?
swapon --show
cat /proc/swaps

# sshd получил MemoryLow?
systemctl show ssh | grep -E "MemoryLow|MemoryHigh"

# под нагрузкой ssh всё ещё отвечает?
time ssh pi@vision-pi 'echo ok'
```

## ADR / источник

- ADR-0111: `docs/adr/0111-vision-pi-zram-swap-and-container-limits.md`
- Issue #2621: Vision Pi уходит в недоступность — 8 ГБ без свопа