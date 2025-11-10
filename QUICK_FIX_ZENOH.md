# 🔧 Краткое руководство: Исправление команд через REST API

## 📋 Проблема
Команды через `https://zenoh.robbox.online/robots/RBXU100001/0/cmd_vel_voice` не доходят до робота.

## 🔍 Причины

### 1. ❌ Неправильный режим облачного роутера
```json
// БЫЛО (неправильно):
{
  "mode": "peer"  // Не маршрутизирует между клиентами!
}

// ДОЛЖНО БЫТЬ:
{
  "mode": "router",  // Полная маршрутизация
  "routing": {
    "router": {
      "peers_failover_brokering": true  // Пересылка между peers
    }
  }
}
```

### 2. ❌ REST API не совместим с ROS топиками напрямую

**Проблема:** 
- REST API публикует на: `robots/RBXU100001/0/cmd_vel_voice`
- ROS подписывается на: `robots/RBXU100001/0/cmd_vel_voice/geometry_msgs::msg::dds_::Twist_/RIHS01_...`

**Ключи не совпадают** → сообщения не доставляются!

---

## ✅ Решение

### Шаг 1: Исправить облачный роутер (zenoh.robbox.online)

```bash
# 1. Остановить Zenoh
sudo systemctl stop zenoh-router

# 2. Скачать правильную конфигурацию
wget https://raw.githubusercontent.com/krikz/rob_box_project/main/docs/cloud/zenoh_router_config.json5 \
  -O /etc/zenoh/config.json5

# 3. Запустить Zenoh
sudo systemctl start zenoh-router

# 4. Проверить режим
curl http://localhost:8000/@/router/config | grep mode
# Должно быть: "mode":"router"
```

### Шаг 2: Установить мост на роботе

```bash
# На роботе (SSH: ros2@10.1.1.20)

# 1. Установить zenoh-python
pip3 install eclipse-zenoh

# 2. Скачать скрипт моста
cd ~/rob_box_project
git pull
cp scripts/zenoh_rest_bridge.py ~/

# 3. Запустить мост
source /opt/ros/humble/setup.bash
python3 ~/zenoh_rest_bridge.py

# Вы должны увидеть:
# ✅ Мост запущен и готов к работе!
# Подписка на Zenoh топик: cmd_vel_web_bridge
# Публикация в ROS топик: cmd_vel_voice
```

### Шаг 3: Протестировать

```bash
# С вашей машины отправить команду
curl -X PUT http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_web_bridge \
  -H "Content-Type: application/json" \
  -d '{"linear":{"x":0.1,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'

# Проверить логи моста на роботе
# Должно появиться: 📨 Получено и отправлено: linear=(0.10, 0.00, 0.00), ...

# Проверить логи twist_mux
docker logs -f twist-mux
# Должно появиться получение команды на cmd_vel_voice
```

---

## 📖 Полная документация

- **Анализ проблемы:** `docs/reports/ZENOH_CLOUD_CONFIG_ISSUE_2025-11-10.md`
- **Конфигурация облака:** `docs/cloud/README.md`
- **Диагностика:** `scripts/diagnose_zenoh_cloud.sh`

---

## 🎯 Systemd сервис для моста (рекомендуется)

Для автозапуска моста при старте робота:

```bash
# Создать сервис
sudo nano /etc/systemd/system/zenoh-rest-bridge.service
```

```ini
[Unit]
Description=Zenoh REST to ROS Bridge
After=network.target docker.service

[Service]
Type=simple
User=ros2
WorkingDirectory=/home/ros2
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_zenoh_cpp"
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash'
ExecStart=/usr/bin/python3 /home/ros2/zenoh_rest_bridge.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
# Включить и запустить
sudo systemctl enable zenoh-rest-bridge
sudo systemctl start zenoh-rest-bridge
sudo systemctl status zenoh-rest-bridge
```

---

## 🔍 Диагностика

Если не работает, запустите диагностику:

```bash
cd ~/rob_box_project/docker/main
../../scripts/diagnose_zenoh_cloud.sh
```

Скрипт проверит:
- ✅ ROBOT_ID
- ✅ Zenoh Router запущен
- ✅ twist-mux подписан
- ✅ Подключение к облаку
- ✅ Режим облачного роутера
- ✅ Namespace конфигурация

---

## 📊 Как это работает

```
1. Веб UI отправляет:
   PUT http://zenoh.robbox.online/robots/RBXU100001/cmd_vel_web_bridge
   {"linear": {"x": 0.1}, "angular": {"z": 0.0}}
   
   ↓

2. Облачный Zenoh Router (режим router):
   Маршрутизирует к роботу
   
   ↓

3. Zenoh REST Bridge на роботе:
   Получает JSON
   Конвертирует в ROS Twist
   
   ↓

4. ROS топик cmd_vel_voice:
   Публикует Twist с правильным типом и хашем
   
   ↓

5. twist_mux:
   Получает команду (приоритет 25)
   
   ↓

6. /diff_drive_controller/cmd_vel_unstamped:
   Команда доходит до моторов!
```

---

**Важно:** Используйте топик **`cmd_vel_web_bridge`**, а не `cmd_vel_voice`!

**Готово!** 🎉
