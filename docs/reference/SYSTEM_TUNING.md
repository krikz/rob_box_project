# Настройка системных параметров Raspberry Pi для ROS2 и DDS

## Проблема с размерами буферов сокетов

При запуске контейнеров вы можете увидеть предупреждение:
```
failed to increase socket send buffer size to at least 1048576 bytes, current is 425984 bytes
```

Это означает, что системные лимиты ядра Linux ограничивают размеры сетевых буферов.

## Решение 1: Уменьшенные буферы в CycloneDDS (уже применено)

Конфигурация уже оптимизирована под стандартные лимиты Pi:
- `SocketReceiveBufferSize: 256kB - 400kB`
- `SocketSendBufferSize: 256kB - 400kB`

Этого достаточно для сжатых изображений 720p @ 5fps.

## Решение 2: Увеличение системных лимитов (опционально)

Если требуется больше пропускной способности, увеличьте системные лимиты.

### На обеих Raspberry Pi выполните:

```bash
# Проверьте текущие значения
sysctl net.core.rmem_max
sysctl net.core.wmem_max
sysctl net.core.rmem_default
sysctl net.core.wmem_default

# Временное увеличение (до перезагрузки)
sudo sysctl -w net.core.rmem_max=8388608      # 8MB для receive
sudo sysctl -w net.core.wmem_max=4194304      # 4MB для send
sudo sysctl -w net.core.rmem_default=2097152  # 2MB default receive
sudo sysctl -w net.core.wmem_default=1048576  # 1MB default send
```

### Постоянная настройка (сохраняется после перезагрузки):

```bash
# Создайте/отредактируйте файл конфигурации
sudo nano /etc/sysctl.d/99-ros2-network.conf
```

Добавьте следующие строки:
```conf
# ROS2 DDS network buffer optimization
net.core.rmem_max = 8388608
net.core.wmem_max = 4194304
net.core.rmem_default = 2097152
net.core.wmem_default = 1048576
```

Сохраните файл (Ctrl+O, Enter, Ctrl+X) и примените:
```bash
sudo sysctl -p /etc/sysctl.d/99-ros2-network.conf
```

### После увеличения лимитов

Можете вернуть большие буферы в `cyclonedds.xml`:
```xml
<SocketReceiveBufferSize min="2MB" max="4MB"/>
<SocketSendBufferSize min="1MB" max="2MB"/>
```

## Проверка настроек

```bash
# Проверьте примененные значения
sysctl net.core.rmem_max net.core.wmem_max net.core.rmem_default net.core.wmem_default

# Перезапустите контейнеры
cd ~/rob_box_project/docker/vision   # или /main
./update_and_restart.sh

# Проверьте, что ошибок больше нет
docker logs oak-d | grep "failed to increase"
```

## Рекомендации

**Для стабильной работы с текущими настройками (256-400kB):**
- ✅ Достаточно для сжатых изображений JPEG/PNG
- ✅ Работает без изменения системы
- ✅ Подходит для 720p @ 5fps
- ✅ Минимальная нагрузка на Pi

**Увеличивайте буферы только если:**
- ❌ Видите потери пакетов в логах DDS
- ❌ Нестабильная передача изображений
- ❌ Нужно увеличить разрешение/FPS
- ❌ Множество одновременных топиков

## Мониторинг сети

```bash
# Проверка потерь пакетов
watch -n 1 'netstat -s | grep -i "packet receive errors"'

# Проверка использования буфера
ss -tuln

# Мониторинг пропускной способности
iftop -i wlan0  # или eth0
```

## Дополнительные оптимизации (опционально)

```bash
# Увеличить размер UDP буферов
sudo sysctl -w net.ipv4.udp_mem="102400 873800 16777216"
sudo sysctl -w net.ipv4.udp_rmem_min=16384
sudo sysctl -w net.ipv4.udp_wmem_min=16384

# Оптимизация TCP (если используется)
sudo sysctl -w net.ipv4.tcp_rmem="4096 87380 16777216"
sudo sysctl -w net.ipv4.tcp_wmem="4096 65536 16777216"
```

---

**Текущая конфигурация**: Оптимизирована под стандартные лимиты Pi (256-400kB буферы)  
**Рекомендация**: Сначала протестируйте с текущими настройками. Увеличивайте лимиты только при необходимости.
