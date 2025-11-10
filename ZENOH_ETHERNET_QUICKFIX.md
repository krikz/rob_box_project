# Быстрое исправление: Zenoh через Ethernet

## 🎯 Суть проблемы

Zenoh трафик шёл через WiFi → перегрузка → ошибки транспорта

## ✅ Решение

Добавлен параметр `#iface=eth0` ко всем Zenoh endpoints → трафик идёт только через Gigabit Ethernet

## 📝 Что изменилось

### Vision Pi (`docker/vision/config/zenoh_router_config.json5`)

```diff
connect:
  endpoints: [
-   "tcp/10.1.1.10:7447"
+   "tcp/10.1.1.10:7447#iface=eth0"
  ]

listen:
  endpoints: [
-   "tcp/[::]:7447"
+   "tcp/[::]:7447#iface=eth0"
  ]
```

### Main Pi (`docker/main/config/zenoh_router_config.json5`)

```diff
listen:
  endpoints: [
-   "tcp/[::]:7447"
+   "tcp/[::]:7447#iface=eth0"
  ]
```

## 🚀 Развёртывание

```bash
# На Vision Pi
cd ~/rob_box_project
git pull origin main
cd docker/vision
docker-compose restart zenoh-router

# На Main Pi
cd ~/rob_box_project  
git pull origin main
cd docker/main
docker-compose restart zenoh-router
```

## ✓ Проверка

```bash
# Проверить что соединение через eth0 (10.1.1.x)
sudo netstat -tnp | grep 7447 | grep ESTABLISHED

# Vision Pi должен показать: 10.1.1.11:XXXXX → 10.1.1.10:7447
# Main Pi должен показать:   10.1.1.10:7447 ← 10.1.1.11:XXXXX
```

## 📚 Детали

См. полную документацию: `ZENOH_ETHERNET_INTERFACE_FIX_2025-11-10.md`

## ⚠️ Важно

- **eth0 должен быть активен** с IP 10.1.1.10 (Main) / 10.1.1.11 (Vision)
- WiFi остаётся для SSH и управления
- Изменения не требуют пересборки Docker образов
