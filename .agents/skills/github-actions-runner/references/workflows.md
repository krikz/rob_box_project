# Workflows Reference — rob_box_project

Репозиторий: `krikz/rob_box_project`  
Файлы: `.github/workflows/`

---

## G-Build All Services.yml

**Название:** "G: Build All Services"  
**Триггеры:** `push` в `main`, `workflow_dispatch`, `schedule` (03:00 UTC)  
**Параметры:** без параметров  
**Что делает:** Сборка базовых образов + Main Pi + Vision Pi на GitHub Actions  
**Registry:** `ghcr.io/krikz/rob_box`  

```bash
gh workflow run "G-Build All Services.yml" --repo krikz/rob_box_project
```

---

## G-Build Base Images.yml

**Название:** "G: Build Base Images"  
**Триггеры:** `workflow_dispatch`, `workflow_call`  
**Параметры:** без параметров  
**Образы:** `ros2-zenoh`, `rtabmap`, `depthai`, `pcl`  
**Registry:** `ghcr.io/krikz/rob_box_base`  

```bash
gh workflow run "G-Build Base Images.yml" --repo krikz/rob_box_project
```

---

## G-Build Main Pi Services.yml

**Название:** "G: Build Main Pi Services"  
**Триггеры:** push в `develop`/`copilot/**`, `workflow_dispatch`, `workflow_call`  
**Параметры:** без параметров  
**Сервисы:** `robot-state-publisher`, `rtabmap`, `twist-mux`, `micro-ros-agent`, `ros2-control`, `nav2`, `lslidar`, `perception`  
**Platform:** `linux/arm64`  

```bash
gh workflow run "G-Build Main Pi Services.yml" --repo krikz/rob_box_project
```

---

## G-Build Vision Pi Services.yml

**Название:** "G: Build Vision Pi Services"  
**Триггеры:** push в `develop`/`feature/**`/`copilot/**`, `workflow_dispatch`, `workflow_call`  
**Параметры:** без параметров  
**Сервисы:** `oak-d`, `lslidar`, `led-matrix`, `voice-assistant`, `perception`, `apriltag`  
**Platform:** `linux/arm64`  

```bash
gh workflow run "G-Build Vision Pi Services.yml" --repo krikz/rob_box_project
```

---

## G-Lint Code.yml

**Название:** "G: Lint Code"  
**Триггеры:** push в `develop`/`main` (`.py`, `.yml`, `Dockerfile`), `pull_request`, `workflow_dispatch`  
**Параметры:** без параметров  
**Проверки:** black, isort, flake8 для Python; hadolint для Dockerfile; yamllint для YAML  

```bash
gh workflow run "G-Lint Code.yml" --repo krikz/rob_box_project
```

---

## G-Run Tests.yml

**Название:** "G: Run Tests"  
**Триггеры:** push в `develop`/`main` (изменения `src/**`), `pull_request`, `workflow_dispatch`  
**Параметры:** без параметров  
**Что делает:** Unit tests для ROS 2 пакетов на Ubuntu 22.04  

```bash
gh workflow run "G-Run Tests.yml" --repo krikz/rob_box_project
```

---

## G-Validate Docker Compose.yml

**Название:** "G: Validate Docker Compose"  
**Триггеры:** push/PR в `develop`/`main` (изменения `docker-compose.yaml`), `workflow_dispatch`  
**Параметры:** без параметров  
**Что проверяет:** `docker/main/docker-compose.yaml` и `docker/vision/docker-compose.yaml`  

```bash
gh workflow run "G-Validate Docker Compose.yml" --repo krikz/rob_box_project
```

---

## L-Build All Services.yml

**Название:** "L: Build All Services"  
**Runner:** `self-hosted` (локальный build machine)  
**Триггеры:** только `workflow_dispatch`  
**Registry:** `localhost:5000/krikz/rob_box`  

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `build_base_images` | boolean | `false` | Сначала собрать базовые образы |
| `push_to_registry` | boolean | `true` | Пушить в localhost:5000 |

```bash
gh workflow run "L-Build All Services.yml" \
  -f build_base_images=false \
  -f push_to_registry=true \
  --repo krikz/rob_box_project
```

---

## L-Build Base Images.yml

**Название:** "L: Build Base Images"  
**Runner:** `self-hosted`  
**Триггеры:** `workflow_dispatch`, `workflow_call`  
**Образы:** `ros2-zenoh`, `rtabmap`, `depthai`, `pcl`  
**Registry:** `localhost:5000/krikz/rob_box_base` + `ghcr.io/krikz/rob_box_base`  

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `push_to_registry` | boolean | `true` | Пушить в localhost:5000 |

```bash
gh workflow run "L-Build Base Images.yml" \
  -f push_to_registry=true \
  --repo krikz/rob_box_project
```

---

## L-Build Main Pi Services.yml

**Название:** "L: Build Main Pi Services"  
**Runner:** `self-hosted`  
**Триггеры:** `workflow_dispatch`, `workflow_call`  
**Сервисы:** `robot-state-publisher`, `rtabmap`, `twist-mux`, `micro-ros-agent`, `ros2-control`, `nav2`, `lslidar`, `perception`  

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `push_to_registry` | boolean | `true` | Пушить в localhost:5000 |

```bash
gh workflow run "L-Build Main Pi Services.yml" \
  -f push_to_registry=true \
  --repo krikz/rob_box_project
```

---

## L-Build Vision Pi Services.yml

**Название:** "L: Build Vision Pi Services"  
**Runner:** `self-hosted`  
**Триггеры:** `workflow_dispatch`, `workflow_call`  
**Сервисы:** `oak-d`, `led-matrix`, `ceiling-camera`, `voice-assistant`, `apriltag`, `zenoh-router`  

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `push_to_registry` | boolean | `true` | Пушить в localhost:5000 |

```bash
gh workflow run "L-Build Vision Pi Services.yml" \
  -f push_to_registry=true \
  --repo krikz/rob_box_project
```

---

## L-Build Single Service.yml

**Название:** "L: Build Single Service"  
**Runner:** `self-hosted`  
**Триггеры:** только `workflow_dispatch`  
**Назначение:** Собрать один конкретный сервис без запуска всего pipeline  

| Параметр | Тип | Обязателен | Варианты |
|----------|-----|-----------|---------|
| `branch` | choice | ✅ | `main`, `develop`, `release/v1.0.0` |
| `pi_type` | choice | ✅ | `main`, `vision`, `base` |
| `service` | choice | ✅ | см. ниже |
| `push_to_registry` | boolean | — | default: `true` |
| `create_issue_on_failure` | boolean | — | default: `true` |

**Доступные сервисы по pi_type:**

| `pi_type=main` | `pi_type=vision` | `pi_type=base` |
|----------------|-----------------|----------------|
| robot-state-publisher | oak-d | ros2-zenoh |
| rtabmap | led-matrix | depthai |
| twist-mux | ceiling-camera | pcl |
| micro-ros-agent | voice-assistant | |
| ros2-control | apriltag | |
| nav2 | zenoh-router | |
| lslidar | | |
| perception | | |

```bash
# Пример: собрать voice-assistant из develop
gh workflow run "L-Build Single Service.yml" \
  -f branch=develop \
  -f pi_type=vision \
  -f service=voice-assistant \
  -f push_to_registry=true \
  --repo krikz/rob_box_project

# Пример: собрать rtabmap из main
gh workflow run "L-Build Single Service.yml" \
  -f branch=main \
  -f pi_type=main \
  -f service=rtabmap \
  --repo krikz/rob_box_project
```

---

## L-Deploy and Verify.yml

**Название:** "L: Deploy and Verify"  
**Runner:** `self-hosted` (нужен доступ к 10.1.1.x сети и Raspberry Pi)  
**Триггеры:** только `workflow_dispatch`  
**Что делает:**  
- Останавливает контейнеры на Main Pi (10.1.1.10) и Vision Pi (10.1.1.11)  
- Обновляет код из выбранной ветки  
- Подтягивает свежие Docker образы  
- Запускает контейнеры  
- Проверяет логи и ROS 2 топики  
- Создаёт GitHub issue при ошибках  

| Параметр | Тип | Обязателен | Варианты |
|----------|-----|-----------|---------|
| `environment` | choice | ✅ | `production`, `staging`, `test` |
| `registry_source` | choice | ✅ | `github`, `local`, `skip` |
| `branch` | choice | — | `main`, `develop`, `feature/test` |
| `dry_run` | boolean | — | default: `false` |

**Маппинг environment → IMAGE_TAG:**
- `production` → `latest` (из `main`)
- `staging` → `dev` (из `develop`)
- `test` → `test` (из `feature/test`)

**Маппинг registry_source:**
- `github` → `ghcr.io/krikz/rob_box` (G-Build workflows)
- `local` → `localhost:5000/krikz/rob_box` (L-Build workflows, в 10-20x быстрее pull)
- `skip` → не загружать образы, использовать существующие

```bash
# Production деплой
gh workflow run "L-Deploy and Verify.yml" \
  -f environment=production \
  -f registry_source=github \
  --repo krikz/rob_box_project

# Staging деплой из develop с локальным registry
gh workflow run "L-Deploy and Verify.yml" \
  -f environment=staging \
  -f registry_source=local \
  --repo krikz/rob_box_project

# Dry-run (без реального деплоя)
gh workflow run "L-Deploy and Verify.yml" \
  -f environment=test \
  -f registry_source=github \
  -f dry_run=true \
  --repo krikz/rob_box_project
```

---

## Полезные ссылки

- Все workflows: https://github.com/krikz/rob_box_project/actions
- CI/CD документация: `docs/CI_CD_PIPELINE.md`
- Deployment Guide: `docs/DEPLOYMENT_WORKFLOW.md`
