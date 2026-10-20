# 🌐 Сетевая топология Rob Box

> Версия: 1.0 | Дата: 2026-02-19

---

## Содержание

1. [Физическая топология](#1-физическая-топология)
2. [IP-адресация](#2-ip-адресация)
3. [Zenoh топология](#3-zenoh-топология)
4. [Порты и конфигурация](#4-порты-и-конфигурация)
5. [SSH доступ](#5-ssh-доступ)

---

## 1. Физическая топология

Система использует **две независимые сети** для разделения потоков данных и управляющего трафика.
Узлов три: Main Pi, Vision Pi и Sound Pi ([ADR-0159](../adr/0159-sound-pi-third-node-voice-stack-and-sensor-hub.md)).

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'primaryColor':'#e8f4f8','primaryTextColor':'#000','primaryBorderColor':'#2c5282'}}}%%
graph TB
    subgraph Ethernet["📡 GIGABIT ETHERNET — данные ROS/Zenoh (1000 Mbps)"]
        VisionEth["Vision Pi (eth0)<br/>10.1.1.11"]
        MainEth["Main Pi (eth0)<br/>10.1.1.10"]
        SoundEth["Sound Pi (eth0)<br/>10.1.1.12"]
        VisionEth <==ROS/Zenoh==> MainEth
        SoundEth <==ROS/Zenoh==> MainEth
        SoundEth <==ROS/Zenoh==> VisionEth
    end

    subgraph WiFi["📶 WiFi — SSH, управление, мониторинг (100-300 Mbps)"]
        VisionWlan["Vision Pi (wlan0)<br/>10.1.1.21"]
        MainWlan["Main Pi (wlan0)<br/>10.1.1.20"]
        SoundWlan["Sound Pi (wlan0)<br/>10.1.1.22"]
        Router["Router / DHCP"]
        HostPC["Host PC<br/>10.1.1.5"]

        VisionWlan <-.SSH.-> Router
        MainWlan <-.SSH.-> Router
        SoundWlan <-.SSH.-> Router
        Router <-.-> HostPC
    end

    style Ethernet fill:#e8f4f8,stroke:#2c5282,stroke-width:3px
    style WiFi fill:#fff3cd,stroke:#856404,stroke-width:3px
    style VisionEth fill:#cce5ff,stroke:#004085,stroke-width:2px
    style MainEth fill:#cce5ff,stroke:#004085,stroke-width:2px
    style SoundEth fill:#cce5ff,stroke:#004085,stroke-width:2px
    style VisionWlan fill:#f8d7da,stroke:#721c24,stroke-width:2px
    style MainWlan fill:#f8d7da,stroke:#721c24,stroke-width:2px
    style SoundWlan fill:#f8d7da,stroke:#721c24,stroke-width:2px
```

**⚠️ ВАЖНО:**
- ROS 2 / Zenoh используют **ТОЛЬКО** Ethernet (`10.1.1.x`) — предотвращает перегрузку WiFi
- WiFi используется **ТОЛЬКО** для SSH, Foxglove и мониторинга

---

## 2. IP-адресация

| Устройство | Ethernet (eth0) | WiFi (wlan0) | Роль |
|------------|-----------------|--------------|------|
| **Main Pi** | `10.1.1.10` | `10.1.1.20` | Центральный узел: Nav2, RTAB-Map, VESC, Zenoh router |
| **Vision Pi** | `10.1.1.11` | `10.1.1.21` | Сенсоры и зрение: OAK-D, AprilTag, Hailo, распознавание лиц, LED-матрица |
| **Sound Pi** | `10.1.1.12` | `10.1.1.22` | Звук и оператор: ReSpeaker, voice-assistant, SuperCollider, ТАРС (avatar-supervisor), avatar-arbiter, Quest, Telegram-бот (PR-7); TTL-USB для ESP32-хаба |
| **Host PC** | — | `10.1.1.5` | Foxglove Studio, RViz2, SSH, мониторинг |
| **Monitoring** | — | `10.1.1.x` | Grafana + Prometheus (отдельная машина) |

---

## 3. Zenoh топология

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'primaryColor':'#e8f4f8','primaryTextColor':'#000','primaryBorderColor':'#2c5282'}}}%%
graph TB
    Cloud["☁️ zenoh.robbox.online:7447<br/>(Облако — опционально)"]

    MainRouter["Main Pi Zenoh Router<br/>10.1.1.10:7447<br/>mode: peer"]

    VisionRouter["Vision Pi Zenoh Router<br/>10.1.1.11:7447<br/>mode: client"]

    SoundRouter["Sound Pi Zenoh Router<br/>10.1.1.12:7447<br/>connect: Main + Vision"]

    MainNodes["Main Pi ROS2 узлы<br/>rtabmap · twist_mux<br/>nav2 · lslidar · perception"]

    VisionNodes["Vision Pi ROS2 узлы<br/>oak-d · apriltag<br/>vision-hailo · led-matrix"]

    SoundNodes["Sound Pi ROS2 узлы<br/>voice-assistant · supercollider<br/>supervisor · avatar-arbiter<br/>quest · telegram-bot"]

    Cloud <==TCP/TLS==> MainRouter
    MainRouter <==UDP/TCP==> VisionRouter
    MainRouter <==UDP/TCP==> SoundRouter
    VisionRouter <==UDP/TCP==> SoundRouter
    MainRouter --> MainNodes
    VisionRouter --> VisionNodes
    SoundRouter --> SoundNodes

    style Cloud fill:#cce5ff,stroke:#004085,stroke-width:2px
    style MainRouter fill:#d4edda,stroke:#28a745,stroke-width:3px
    style VisionRouter fill:#fff3cd,stroke:#856404,stroke-width:2px
    style SoundRouter fill:#fff3cd,stroke:#856404,stroke-width:2px
    style MainNodes fill:#e8f4f8,stroke:#2c5282,stroke-width:2px
    style VisionNodes fill:#e8f4f8,stroke:#2c5282,stroke-width:2px
    style SoundNodes fill:#e8f4f8,stroke:#2c5282,stroke-width:2px
```

### Режимы Zenoh

| Узел | Режим | Описание |
|------|-------|----------|
| **Main Pi** | `peer` | Центральный роутер, работает автономно |
| **Vision Pi** | `client` | Подключается к Main Pi, зависит от него |
| **Sound Pi** | `client` | Подключается к Main Pi и напрямую к Vision Pi (ADR-0159 §2.2, вариант Б) |
| **Cloud** | `peer` | Опционально — удалённый мониторинг |

### Переменные окружения

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG=/config/zenoh_session_config.json5
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

---

## 4. Порты и конфигурация

| Сервис | Хост | Порт | Протокол |
|--------|------|------|----------|
| Zenoh Router | Main Pi | 7447 | TCP/UDP |
| Zenoh Router | Vision Pi | 7447 | TCP/UDP |
| Zenoh Router | Sound Pi | 7447 | TCP/UDP |
| Zenoh Cloud | zenoh.robbox.online | 7447 | TCP/TLS |
| Grafana | Monitoring PC | 3000 | HTTP |
| Prometheus | Monitoring PC | 9090 | HTTP |
| SSH | Main Pi (wlan0) | 22 | TCP |
| SSH | Vision Pi (wlan0) | 22 | TCP |
| SSH | Sound Pi (wlan0) | 22 | TCP |

---

## 5. SSH доступ

```bash
# Vision Pi (WiFi)
sshpass -p 'open' ssh ros2@10.1.1.21

# Sound Pi (WiFi) — голосовой стек: docker logs voice-assistant и т. д.
sshpass -p 'open' ssh ros2@10.1.1.22

# Main Pi (WiFi)
sshpass -p 'open' ssh ros2@10.1.1.20

# Vision Pi (Ethernet — с другой Pi)
ssh ros2@10.1.1.11
```

---

**Связанные документы:**
- [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md) — общая архитектура (раздел 3)
- [SOFTWARE.md](SOFTWARE.md) — программные компоненты и Docker
- [guides/VISION_PI_NETWORK_SETUP.md](../guides/VISION_PI_NETWORK_SETUP.md) — настройка сети Vision Pi

**Навигация:** [← Архитектура](README.md) | [📚 Документация](../README.md)
