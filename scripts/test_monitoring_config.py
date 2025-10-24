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


def main():
    """Run all monitoring configuration tests."""
    print("=" * 60)
    print("Monitoring Configuration Validation")
    print("=" * 60)
    print()
    
    project_root = Path(__file__).parent.parent
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
