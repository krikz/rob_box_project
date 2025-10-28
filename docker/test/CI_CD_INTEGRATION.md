# CI/CD Integration for x86_64 Testing

**Purpose:** Integrate x86_64 test environment into GitHub Actions for automated smoke testing.

---

## 🎯 Overview

This document describes how to integrate the x86_64 test environment into CI/CD workflows for automated testing.

**Benefits:**
- ✅ Automated smoke testing on every PR
- ✅ Catch integration issues early
- ✅ Validate ROS2 communication
- ✅ No need for physical Raspberry Pi in CI

---

## 📝 GitHub Actions Workflow Example

### smoke-test.yml

Create `.github/workflows/smoke-test.yml`:

```yaml
name: Smoke Test on x86_64

on:
  pull_request:
    branches:
      - develop
      - main
  push:
    branches:
      - develop
      - main
  workflow_dispatch:

jobs:
  smoke-test:
    name: Run smoke tests on x86_64
    runs-on: ubuntu-latest
    
    steps:
      - name: Checkout code
        uses: actions/checkout@v4
      
      - name: Set up QEMU for ARM64 emulation
        uses: docker/setup-qemu-action@v3
        with:
          platforms: linux/arm64
      
      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3
      
      - name: Log in to GitHub Container Registry
        uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}
      
      - name: Prepare test environment
        run: |
          cd docker/test
          cp .env.example .env
          
          # Use latest images from GHCR
          echo "IMAGE_TAG=latest" >> .env
      
      - name: Start core services
        run: |
          cd docker/test
          
          # Start only working services (no hardware dependencies)
          docker compose -f docker-compose-x86-test.yaml up -d \
            zenoh-router \
            robot-state-publisher \
            twist-mux \
            perception
          
          # Wait for services to initialize
          sleep 10
      
      - name: Check service health
        run: |
          cd docker/test
          
          # Check that services are running
          docker compose -f docker-compose-x86-test.yaml ps
          
          # Run health check script
          ./scripts/check_health.sh
      
      - name: Run smoke tests
        run: |
          cd docker/test
          ./scripts/smoke_test.sh
      
      - name: Verify Zenoh connectivity
        run: |
          # Check Zenoh REST API
          curl -f http://localhost:8000/@/local/router
          
          # Check publishers/subscribers
          curl -s http://localhost:8000/@/local/publisher | jq
          curl -s http://localhost:8000/@/local/subscriber | jq
      
      - name: Collect logs on failure
        if: failure()
        run: |
          cd docker/test
          docker compose -f docker-compose-x86-test.yaml logs > smoke-test-logs.txt
      
      - name: Upload logs as artifact
        if: failure()
        uses: actions/upload-artifact@v4
        with:
          name: smoke-test-logs
          path: docker/test/smoke-test-logs.txt
          retention-days: 7
      
      - name: Stop services
        if: always()
        run: |
          cd docker/test
          docker compose -f docker-compose-x86-test.yaml down
```

---

## 🔧 Advanced Workflow: Full Service Test

For testing ALL services (including hardware-dependent ones):

### full-smoke-test.yml

```yaml
name: Full Smoke Test (All Services)

on:
  workflow_dispatch:  # Manual trigger only
  schedule:
    - cron: '0 2 * * *'  # Daily at 2 AM

jobs:
  full-smoke-test:
    name: Test all services including hardware-dependent
    runs-on: ubuntu-latest
    
    steps:
      - name: Checkout code
        uses: actions/checkout@v4
      
      - name: Set up QEMU
        uses: docker/setup-qemu-action@v3
        with:
          platforms: linux/arm64
      
      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3
      
      - name: Log in to GHCR
        uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}
      
      - name: Prepare environment
        run: |
          cd docker/test
          cp .env.example .env
          echo "IMAGE_TAG=latest" >> .env
      
      - name: Start ALL services
        run: |
          cd docker/test
          docker compose -f docker-compose-x86-test.yaml up -d
          sleep 15
      
      - name: Check core services
        run: |
          cd docker/test
          
          # These should be running
          for service in zenoh-router robot-state-publisher twist-mux perception; do
            echo "Checking $service..."
            if ! docker compose -f docker-compose-x86-test.yaml ps $service | grep -q "Up"; then
              echo "ERROR: $service should be running but is not"
              exit 1
            fi
          done
      
      - name: Check hardware-dependent services
        run: |
          cd docker/test
          
          # These should exit (no hardware)
          for service in oak-d lslidar led-matrix voice-assistant micro-ros-agent vesc-nexus; do
            echo "Checking $service (expected to exit)..."
            status=$(docker compose -f docker-compose-x86-test.yaml ps $service 2>/dev/null | tail -n +2)
            
            if echo "$status" | grep -q "Up"; then
              echo "WARNING: $service is running (unexpected, but not critical)"
            else
              echo "OK: $service exited as expected (no hardware)"
            fi
          done
      
      - name: Verify Zenoh topics
        run: |
          # Check that expected topics exist
          pubs=$(curl -s http://localhost:8000/@/local/publisher | jq -r 'keys[]' | grep -c '^rt/' || echo 0)
          echo "Found $pubs publishers"
          
          if [ "$pubs" -lt 3 ]; then
            echo "ERROR: Expected at least 3 publishers, found $pubs"
            exit 1
          fi
      
      - name: Generate test report
        run: |
          cd docker/test
          
          echo "# Smoke Test Report" > smoke-test-report.md
          echo "Date: $(date)" >> smoke-test-report.md
          echo "" >> smoke-test-report.md
          
          echo "## Service Status" >> smoke-test-report.md
          docker compose -f docker-compose-x86-test.yaml ps >> smoke-test-report.md
          
          echo "" >> smoke-test-report.md
          echo "## Zenoh Statistics" >> smoke-test-report.md
          curl -s http://localhost:8000/@/local/router | jq >> smoke-test-report.md
      
      - name: Upload test report
        uses: actions/upload-artifact@v4
        with:
          name: full-smoke-test-report
          path: docker/test/smoke-test-report.md
          retention-days: 30
      
      - name: Cleanup
        if: always()
        run: |
          cd docker/test
          docker compose -f docker-compose-x86-test.yaml down
```

---

## 🎨 Integration Tests with ROS2

For more advanced testing with ROS2 CLI:

### integration-test.yml

```yaml
name: Integration Tests

on:
  pull_request:
    paths:
      - 'src/**'
      - 'docker/main/**'
      - 'docker/vision/**'

jobs:
  integration-test:
    name: ROS2 Integration Tests
    runs-on: ubuntu-latest
    
    steps:
      - name: Checkout code
        uses: actions/checkout@v4
      
      - name: Set up QEMU
        uses: docker/setup-qemu-action@v3
        with:
          platforms: linux/arm64
      
      - name: Install ROS2 Humble
        run: |
          sudo apt update
          sudo apt install -y software-properties-common
          sudo add-apt-repository universe
          sudo apt update && sudo apt install -y curl gnupg lsb-release
          
          # Add ROS2 repository
          sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg
          
          echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
            http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
            sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
          
          sudo apt update
          sudo apt install -y ros-humble-ros-base ros-humble-rmw-zenoh-cpp
      
      - name: Start test environment
        run: |
          cd docker/test
          cp .env.example .env
          ./scripts/start_test_env.sh
      
      - name: Test ROS2 topics
        run: |
          source /opt/ros/humble/setup.bash
          export RMW_IMPLEMENTATION=rmw_zenoh_cpp
          export ZENOH_SESSION_CONFIG_URI="$(pwd)/docker/test/config/zenoh_session_config.json5"
          
          # Wait for topics to appear
          sleep 5
          
          # List topics
          ros2 topic list
          
          # Check specific topics exist
          ros2 topic list | grep -q "/robot_description" || exit 1
          ros2 topic list | grep -q "/tf" || exit 1
          ros2 topic list | grep -q "/tf_static" || exit 1
          
          # Check topic types
          ros2 topic info /robot_description
          ros2 topic info /tf
      
      - name: Test ROS2 services
        run: |
          source /opt/ros/humble/setup.bash
          export RMW_IMPLEMENTATION=rmw_zenoh_cpp
          export ZENOH_SESSION_CONFIG_URI="$(pwd)/docker/test/config/zenoh_session_config.json5"
          
          # List services
          ros2 service list
      
      - name: Cleanup
        if: always()
        run: |
          cd docker/test
          ./scripts/stop_test_env.sh
```

---

## 📊 Monitoring Integration

For integration with monitoring/observability:

```yaml
      - name: Check Prometheus metrics
        run: |
          # If monitoring is enabled
          if docker ps | grep -q prometheus; then
            curl -f http://localhost:9090/-/healthy
            
            # Query some metrics
            curl -s 'http://localhost:9090/api/v1/query?query=up' | jq
          fi
```

---

## 🔍 Best Practices

### 1. Test Scope

**DO:**
- ✅ Test service startup
- ✅ Test ROS2 communication
- ✅ Test Zenoh connectivity
- ✅ Test topic publishing/subscribing
- ✅ Validate configuration loading

**DON'T:**
- ❌ Test hardware-dependent functionality
- ❌ Performance benchmarking
- ❌ Load testing (too slow in QEMU)

### 2. Performance

```yaml
# Limit resources to avoid CI timeouts
services:
  rtabmap:
    # ...
    deploy:
      resources:
        limits:
          cpus: '2'
          memory: 2G
```

### 3. Caching

```yaml
# Cache Docker layers for faster builds
- name: Cache Docker layers
  uses: actions/cache@v3
  with:
    path: /tmp/.buildx-cache
    key: ${{ runner.os }}-buildx-${{ github.sha }}
    restore-keys: |
      ${{ runner.os }}-buildx-
```

### 4. Parallel Testing

```yaml
strategy:
  matrix:
    service: [zenoh-router, robot-state-publisher, twist-mux, perception]
```

---

## 📈 Metrics and Reporting

Track test results over time:

```yaml
- name: Generate metrics
  run: |
    {
      echo "test_timestamp=$(date +%s)"
      echo "test_duration_seconds=$SECONDS"
      echo "services_running=$(docker ps -q | wc -l)"
      echo "zenoh_publishers=$(curl -s http://localhost:8000/@/local/publisher | jq 'keys | length')"
      echo "zenoh_subscribers=$(curl -s http://localhost:8000/@/local/subscriber | jq 'keys | length')"
    } | tee test-metrics.txt

- name: Upload metrics
  uses: actions/upload-artifact@v4
  with:
    name: test-metrics
    path: test-metrics.txt
```

---

## 🚨 Notifications

Notify on failures:

```yaml
- name: Notify on failure
  if: failure()
  uses: actions/github-script@v7
  with:
    script: |
      github.rest.issues.createComment({
        issue_number: context.issue.number,
        owner: context.repo.owner,
        repo: context.repo.repo,
        body: '❌ Smoke tests failed. Check the logs for details.'
      })
```

---

## 🎯 Summary

The x86_64 test environment is fully compatible with GitHub Actions and can be integrated into CI/CD pipelines for:

1. **PR Validation** - Smoke test on every PR
2. **Daily Builds** - Full test suite nightly
3. **Integration Tests** - ROS2 communication validation
4. **Performance Monitoring** - Track test execution time

All tests run without physical hardware, using QEMU emulation for ARM64 services.

---

**Created:** October 28, 2025  
**Version:** 1.0.0
