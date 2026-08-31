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
