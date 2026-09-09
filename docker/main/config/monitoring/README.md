# Monitoring Configuration Files

This directory contains configuration files for the Rob Box monitoring stack.

## Files

### prometheus.yml
Prometheus configuration for scraping metrics from cAdvisor instances on both Pis.

**Key settings:**
- `scrape_interval: 15s` - how often to collect metrics
- Targets: Main Pi (localhost:8080) and Vision Pi (10.1.1.11:8080)

### loki-config.yaml
Loki log aggregation server configuration.

**Key settings:**
- `retention_period: 168h` (7 days) - how long to keep logs
- Storage: filesystem-based on `/tmp/loki`
- Ingestion limits to prevent OOM on Raspberry Pi

### promtail-config.yaml (Main Pi)
Promtail configuration for collecting logs from Main Pi Docker containers.

**Features:**
- Scrapes Docker containers with label `logging: "promtail"`
- Collects system logs from `/var/log/*.log`
- Sends logs to local Loki (localhost:3100)

### promtail-config.yaml (Vision Pi)
Promtail configuration for collecting logs from Vision Pi Docker containers.

**Features:**
- Scrapes Docker containers with label `logging: "promtail"`
- Collects system logs from `/var/log/*.log`
- Sends logs to Main Pi Loki (10.1.1.10:3100)

### grafana-datasources.yaml
Grafana datasource provisioning - auto-configures Loki and Prometheus.

**Configured datasources:**
- Loki (localhost:3100) - default for log queries
- Prometheus (localhost:9090) - for metrics

## Customization

### Changing retention period

Edit `loki-config.yaml`:
```yaml
limits_config:
  retention_period: 72h  # Change from 168h (7 days) to 72h (3 days)
```

### Adjusting scrape intervals

Edit `prometheus.yml`:
```yaml
global:
  scrape_interval: 30s  # Change from 15s to reduce CPU load
  evaluation_interval: 30s
```

### Filtering specific logs

Edit `promtail-config.yaml` to add filters:
```yaml
pipeline_stages:
  - docker: {}
  - match:
      selector: '{container="oak-d"}'
      drop: true  # Don't collect logs from oak-d
```

## Network Ports

| Service    | Port | Access       | Description              |
|------------|------|--------------|--------------------------|
| Grafana    | 3000 | Web UI       | Dashboard interface      |
| Prometheus | 9090 | Web UI, API  | Metrics interface        |
| Loki       | 3100 | API only     | Log ingestion endpoint   |
| cAdvisor   | 8080 | Web UI, API  | Container metrics        |
| Promtail   | 9080 | Internal     | Not exposed externally   |

## Security Notes

- Default Grafana credentials: `admin/robbox` - **CHANGE IN PRODUCTION**
- All services use `network_mode: host` for simplicity
- No authentication on Prometheus, Loki, cAdvisor - use firewall in production
- Promtail has read-only access to Docker socket

## See Also

- [Monitoring System Guide](../../../docs/guides/MONITORING_SYSTEM.md) - Complete user guide
- [Docker Standards](../../../docs/development/DOCKER_STANDARDS.md) - Project Docker conventions
