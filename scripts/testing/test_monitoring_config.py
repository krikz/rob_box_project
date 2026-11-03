#!/usr/bin/env python3
"""
Test script to validate monitoring configuration files.
Checks YAML syntax and required fields.
"""

import os
import sys
import yaml
import json
from pathlib import Path


def test_yaml_file(filepath, required_keys=None):
    """Test if YAML file is valid and contains required keys."""
    print(f"Testing {filepath}...")
    
    try:
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        
        if data is None:
            print(f"  ❌ File is empty")
            return False
            
        if required_keys:
            for key in required_keys:
                if key not in data:
                    print(f"  ❌ Missing required key: {key}")
                    return False
        
        print(f"  ✓ Valid")
        return True
        
    except yaml.YAMLError as e:
        print(f"  ❌ YAML syntax error: {e}")
        return False
    except FileNotFoundError:
        print(f"  ❌ File not found")
        return False
    except Exception as e:
        print(f"  ❌ Error: {e}")
        return False


def test_json_file(filepath):
    """Test if JSON file is valid."""
    print(f"Testing {filepath}...")
    
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        if data is None:
            print(f"  ❌ File is empty")
            return False
            
        print(f"  ✓ Valid")
        return True
        
    except json.JSONDecodeError as e:
        print(f"  ❌ JSON syntax error: {e}")
        return False
    except FileNotFoundError:
        print(f"  ❌ File not found")
        return False
    except Exception as e:
        print(f"  ❌ Error: {e}")
        return False


def test_otel_self_telemetry_prometheus_target_consistency(project_root):
    """Регресс-страховка issue #1730.

    Порт, на который otel-collector отдаёт self-metrics
    (service.telemetry.metrics.address), должен совпадать с target'ом
    job_name='otel-collector' в prometheus.yml. Иначе prometheus не будет
    scrape'ить self-telemetry → потеря health pipeline.

    Исторически дефолт otelcol v0.105 — :8888, но на Katana build-deepseek-proxy
    (LiteLLM) занимает его под свои метрики, что приводит к crash-loop otel-collector.
    Перенесли на :8889; этот тест закрепляет синхронность двух файлов.
    """
    cfg_path = project_root / "docker/monitoring/config/otel-collector.yaml"
    prom_path = project_root / "docker/monitoring/config/prometheus.yml"

    cfg = yaml.safe_load(cfg_path.read_text())
    prom = yaml.safe_load(prom_path.read_text())

    addr = cfg.get("service", {}).get("telemetry", {}).get("metrics", {}).get("address")
    assert addr, f"otel-collector.yaml: service.telemetry.metrics.address не задан ({cfg_path})"

    # '0.0.0.0:8889' → ('0.0.0.0', '8889')
    host, _, port = addr.rpartition(":")
    assert host and port, f"Некорректный telemetry address: {addr!r}"
    assert port.isdigit(), f"Порт не число: {port!r}"
    expected_target = f"localhost:{port}"

    actual_target = None
    for job in prom.get("scrape_configs", []):
        if job.get("job_name") == "otel-collector":
            for sc in job.get("static_configs", []):
                targets = sc.get("targets", [])
                if targets:
                    actual_target = targets[0]
                    break
        if actual_target:
            break

    assert actual_target, (
        f"prometheus.yml: job_name='otel-collector' не найден или пустой targets"
    )
    assert actual_target == expected_target, (
        f"Порт рассогласован: otel self-telemetry {addr!r} → {expected_target}, "
        f"но prometheus scrape-target = {actual_target!r}. "
        f"См. issue #1730."
    )
    print(f"  ✓ otel self-telemetry {addr} == prometheus target {actual_target}")


# Единственный источник правды «какой хост какие IP занимает» для scrape-целей (ADR-0159).
# Какой сервис на каком хосте — определяет compose-каталог docker/<host>/ (сервис объявлен
# в docker-compose.yaml именно этого каталога); тест сверяет с ним scrape-конфиг (#4285).
HOST_IPS = {
    "main-pi": {"10.1.1.10", "localhost"},
    "vision-pi": {"10.1.1.11"},
    "sound-pi": {"10.1.1.12", "10.1.1.22"},
}
HOST_COMPOSE_DIR = {"main-pi": "main", "vision-pi": "vision", "sound-pi": "sound"}


def test_prometheus_service_targets_match_host_layout(project_root):
    """Регресс #4285: цель сервиса скрейпится по адресу хоста, где сервис живёт.

    После ADR-0159 voice-assistant/telegram-bot/supercollider живут на Sound Pi, но живой
    Prometheus ходил на Vision (.11) → ТАРС говорил «лежат» про здоровые контейнеры.
    """
    prom = yaml.safe_load(
        (project_root / "docker/monitoring/config/prometheus.yml").read_text(encoding="utf-8"))
    checked = 0
    for job in prom.get("scrape_configs", []):
        for sc in job.get("static_configs", []):
            labels = sc.get("labels", {})
            service, host = labels.get("service"), labels.get("host")
            if not service or host not in HOST_COMPOSE_DIR:
                continue
            compose = yaml.safe_load((project_root / "docker" / HOST_COMPOSE_DIR[host]
                                      / "docker-compose.yaml").read_text(encoding="utf-8"))
            assert service in (compose.get("services") or {}), (
                f"job {job['job_name']!r}: service={service!r} не объявлен в "
                f"docker/{HOST_COMPOSE_DIR[host]}/docker-compose.yaml, но помечен host={host!r}")
            for target in sc.get("targets", []):
                ip = target.rsplit(":", 1)[0]
                assert ip in HOST_IPS[host], (
                    f"job {job['job_name']!r}: цель {target!r} не принадлежит host={host!r} "
                    f"(адреса {sorted(HOST_IPS[host])}) — #4285")
                checked += 1
    assert checked, "не нашлось ни одной цели с labels service+host — тест ничего не проверил"
    print(f"  ✓ {checked} scrape-целей сервисов соответствуют раскладке по хостам")


def test_monitoring_agents_deployed_and_loki_host_real(project_root):
    """Issue #3300: cadvisor/promtail сидят в profile 'monitoring', а деплой
    делал `compose up` без профиля и `docker rm -f` всего проекта — агенты
    пропадали навсегда. Плюс LOKI_HOST=monitoring-machine не резолвится.
    """
    wf = (project_root / ".github/workflows/L-Deploy and Verify.yml").read_text(encoding="utf-8")
    n = wf.count("compose --profile monitoring up -d cadvisor promtail")
    assert n >= 3, (
        f"L-Deploy and Verify.yml: шаг поднятия агентов мониторинга найден {n} раз, "
        f"нужен на всех трёх Pi (Main + Vision + Sound, ADR-0159). См. issue #3300."
    )
    for pi in ("main", "vision", "sound"):
        env = (project_root / f"docker/{pi}/.env").read_text(encoding="utf-8")
        loki = [l.split("=", 1)[1].strip() for l in env.splitlines() if l.startswith("LOKI_HOST=")]
        assert loki and loki[0] != "monitoring-machine", (
            f"docker/{pi}/.env: LOKI_HOST={loki!r} — плейсхолдер не резолвится, "
            f"promtail не достучится до Loki. См. issue #3300."
        )
    print("  ✓ деплой поднимает profile monitoring; LOKI_HOST задан реальным адресом")


def main():
    """Run all monitoring configuration tests."""
    print("=" * 60)
    print("Monitoring Configuration Validation")
    print("=" * 60)
    print()
    
    # Скрипт лежит в scripts/testing/ → корень репозитория на 3 уровня выше.
    project_root = Path(__file__).parent.parent.parent
    results = []
    
    # Test Main Pi configurations
    print("Main Pi configurations:")
    print("-" * 60)
    
    results.append(test_yaml_file(
        project_root / "docker/main/config/monitoring/prometheus.yml",
        required_keys=["global", "scrape_configs"]
    ))
    
    results.append(test_yaml_file(
        project_root / "docker/main/config/monitoring/loki-config.yaml",
        required_keys=["server", "schema_config", "limits_config"]
    ))
    
    results.append(test_yaml_file(
        project_root / "docker/main/config/monitoring/promtail-config.yaml",
        required_keys=["server", "clients", "scrape_configs"]
    ))
    
    results.append(test_yaml_file(
        project_root / "docker/main/config/monitoring/grafana-datasources.yaml",
        required_keys=["apiVersion", "datasources"]
    ))

    # Актуальный стек мониторинга (docker/monitoring/) — Prometheus с alert rules
    # (ADR-0017, SL-2): prometheus.yml подключает rule_files, rules-файл содержит
    # алерты на падение zenoh-router (Main/Vision Pi).
    results.append(test_yaml_file(
        project_root / "docker/monitoring/config/prometheus.yml",
        required_keys=["global", "rule_files", "scrape_configs"]
    ))

    results.append(test_yaml_file(
        project_root / "docker/monitoring/config/prometheus_rules.yml",
        required_keys=["groups"]
    ))

    print()
    
    # Test Vision Pi configurations
    print("Vision Pi configurations:")
    print("-" * 60)
    
    results.append(test_yaml_file(
        project_root / "docker/vision/config/monitoring/promtail-config.yaml",
        required_keys=["server", "clients", "scrape_configs"]
    ))
    
    print()
    
    # Test docker-compose files
    print("Docker Compose files:")
    print("-" * 60)
    
    results.append(test_yaml_file(
        project_root / "docker/main/docker-compose.yaml",
        required_keys=["services"]
    ))
    
    results.append(test_yaml_file(
        project_root / "docker/vision/docker-compose.yaml",
        required_keys=["services"]
    ))
    
    print()
    
    # Check if scripts exist and are executable
    print("Management scripts:")
    print("-" * 60)
    
    scripts = [
        "docker/main/scripts/enable_monitoring.sh",
        "docker/main/scripts/disable_monitoring.sh",
        "docker/vision/scripts/enable_monitoring.sh",
        "docker/vision/scripts/disable_monitoring.sh",
    ]
    
    for script in scripts:
        path = project_root / script
        if path.exists() and os.access(path, os.X_OK):
            print(f"  ✓ {script}")
            results.append(True)
        else:
            print(f"  ❌ {script} (missing or not executable)")
            results.append(False)

    print()
    print("Consistency checks:")
    print("-" * 60)

    # Issue #1730 — синхронизация otel-collector self-telemetry и prometheus target.
    try:
        test_otel_self_telemetry_prometheus_target_consistency(project_root)
        results.append(True)
    except AssertionError as e:
        print(f"  ❌ {e}")
        results.append(False)
    except Exception as e:
        print(f"  ❌ Unexpected error: {e}")
        results.append(False)

    try:
        test_prometheus_service_targets_match_host_layout(project_root)
        results.append(True)
    except AssertionError as e:
        print(f"  ❌ {e}")
        results.append(False)

    try:
        test_monitoring_agents_deployed_and_loki_host_real(project_root)
        results.append(True)
    except AssertionError as e:
        print(f"  ❌ {e}")
        results.append(False)
    except Exception as e:
        print(f"  ❌ Unexpected error: {e}")
        results.append(False)

    print()
    print("=" * 60)
    
    # Summary
    passed = sum(results)
    total = len(results)
    
    print(f"Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("✓ All tests passed!")
        return 0
    else:
        print(f"❌ {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
