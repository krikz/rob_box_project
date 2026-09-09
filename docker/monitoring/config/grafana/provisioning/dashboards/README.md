# Demo Dashboards for Rob Box

## 📚 Quick Navigation

This directory contains 4 specialized Grafana dashboards for the Rob Box demo stand.

### 🎯 Dashboard Files

| Dashboard | File | Panels | Focus |
|-----------|------|--------|-------|
| **1. System Overview** | `demo_1_system_overview.json` | 10 | CPU/Memory gauges, load graphs, all errors |
| **2. Navigation** | `demo_2_navigation.json` | 7 | RTAB-Map, Nav2, motors, LiDAR (~18 nodes) |
| **3. Perception** | `demo_3_perception.json` | 5 | Cameras, sensors, perception (~10 nodes) |
| **4. Voice** | `demo_4_voice.json` | 5 | Voice assistant, LED, Zenoh (~10 nodes) |

### 📖 Documentation

| Document | Description |
|----------|-------------|
| [DEMO_DASHBOARDS.md](../DEMO_DASHBOARDS.md) | 📘 **START HERE** - Complete setup guide & troubleshooting |
| [NODE_MAPPING.md](../NODE_MAPPING.md) | 🗺️ Visual node distribution across dashboards |
| [DASHBOARD_PREVIEW.md](../DASHBOARD_PREVIEW.md) | 👁️ Visual mockups & demo scenarios |
| [VERIFICATION_SUMMARY.md](../VERIFICATION_SUMMARY.md) | ✅ Implementation summary & testing checklist |

## 🚀 Quick Start

### 1. Enable Monitoring (Both Pi)

```bash
# Main Pi
ssh ros2@10.1.1.20
cd ~/rob_box_project/docker/main
./scripts/enable_monitoring.sh

# Vision Pi
ssh ros2@10.1.1.21
cd ~/rob_box_project/docker/vision
./scripts/enable_monitoring.sh
```

### 2. Access Dashboards

**Standard mode:**
- Dashboard 1: http://10.1.1.10:3000/d/rob_box_demo_1
- Dashboard 2: http://10.1.1.10:3000/d/rob_box_demo_2
- Dashboard 3: http://10.1.1.10:3000/d/rob_box_demo_3
- Dashboard 4: http://10.1.1.10:3000/d/rob_box_demo_4

**Kiosk mode (4 monitors):**
```
http://10.1.1.10:3000/d/rob_box_demo_1?kiosk
http://10.1.1.10:3000/d/rob_box_demo_2?kiosk
http://10.1.1.10:3000/d/rob_box_demo_3?kiosk
http://10.1.1.10:3000/d/rob_box_demo_4?kiosk
```

## 🎨 Dashboard Layout

```
┌─────────────────────────┬─────────────────────────┐
│    🟨 Dashboard 1       │    🟩 Dashboard 2       │
│   System Overview       │      Navigation         │
│   (Main Monitor)        │      & Motion           │
├─────────────────────────┼─────────────────────────┤
│    🟦 Dashboard 3       │    🟥 Dashboard 4       │
│   Perception            │      Voice              │
│   & Sensors             │      & Interface        │
└─────────────────────────┴─────────────────────────┘
```

## 📊 What's Monitored

### Dashboard 1: System Overview & Errors
- CPU & Memory gauges (both Pi)
- CPU load graphs by container
- All errors from all containers

### Dashboard 2: Navigation & Motion
- **RTAB-Map** - SLAM mapping
- **Nav2** - Path planning & control (8 nodes)
- **ROS2 Control** - Motor control (3 nodes)
- **Twist Mux** - Command multiplexing
- **LSLIDAR** - 2D laser scanner
- **Robot State Publisher** - TF transforms

### Dashboard 3: Perception & Sensors
- **OAK-D Camera** - RGB-D + AprilTag detection
- **Ceiling Camera** - USB overhead view
- **micro-ROS Agent** - ESP32 sensor hub
- **Perception System** - Health, context, reflection (5 nodes)

### Dashboard 4: Voice & Interface
- **Voice Assistant** - Audio, STT, TTS, dialogue (7 nodes)
- **LED Matrix** - NeoPixel display
- **Zenoh Routers** - DDS communication (Main + Vision)

## ⚙️ Auto-Provisioning

These dashboards are automatically loaded by Grafana through the provisioning system.

**Config location:** `../dashboards.yaml`

The provisioning scans this directory every 10 seconds for new/updated dashboards.

## 🔍 Troubleshooting

**Dashboards not appearing?**
```bash
docker restart grafana
docker logs grafana | grep -i dashboard
```

**No data in panels?**
```bash
# Check Prometheus
curl http://10.1.1.10:9090/api/v1/targets

# Check Loki
curl http://10.1.1.10:3100/ready

# Check Promtail (Vision Pi)
docker logs promtail-vision | tail -20
```

**For detailed troubleshooting:** See [DEMO_DASHBOARDS.md](../DEMO_DASHBOARDS.md#устранение-неполадок)

## 📈 Statistics

| Metric | Count |
|--------|-------|
| Dashboards | 4 |
| Total Panels | 27 |
| Containers Monitored | 14 |
| ROS 2 Nodes | ~38 |
| Gauge Panels | 4 |
| Timeseries Panels | 2 |
| Logs Panels | 20 |

## 🎯 Use Cases

### Demo Stand Presentation
Use all 4 dashboards on separate monitors to show:
- Overall system health (Dashboard 1)
- Navigation in action (Dashboard 2)
- Sensor inputs (Dashboard 3)
- Voice interaction (Dashboard 4)

### Development & Debugging
- Focus on specific dashboard for targeted debugging
- Use Dashboard 1 for quick system overview
- Filter logs in specific dashboards for detailed analysis

### Remote Monitoring
- Access dashboards from any device on network
- Use kiosk mode for clean presentation
- Monitor multiple robots by changing IP address

## 🔗 Related Documentation

- [Main Monitoring Guide](../../docs/guides/MONITORING_SYSTEM.md)
- [Quick Reference](../../docs/MONITORING_QUICK_REF.md)
- [Agent Guide](../../docs/development/AGENT_GUIDE.md)

## 📝 Version Info

- **Created:** 2025-11-01
- **Version:** 1.0
- **Status:** Production Ready ✅
- **Grafana Version:** 10.2.0+
- **Schema Version:** 38

---

**For complete documentation, start with [DEMO_DASHBOARDS.md](../DEMO_DASHBOARDS.md)** 📘
