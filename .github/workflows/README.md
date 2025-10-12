# GitHub Actions Workflows

Автоматизированные CI/CD pipelines для rob_box_project.

## Workflows Overview

### Автоматический merge

1. **`auto-merge-feature-to-develop.yml`** - Feature → Develop
   - Триггер: Push в `feature/**`, `feat/**`
   - Действия: Собрать изменённые сервисы → Мердж в develop → Удалить feature ветку
   - Tags: `*-humble-dev`

2. **`auto-merge-to-main.yml`** - Develop → Main
   - Триггер: Push в `develop`
   - Действия: Собрать ВСЕ сервисы → Мердж в main → Создать release tag
   - Tags: `*-humble-latest`

### Сборка образов

3. **`build-vision-services.yml`** - Vision Pi services
   - Services: oak-d, lslidar, apriltag, led-matrix, voice-assistant
   - Platform: linux/arm64
   - Registry: ghcr.io/krikz/rob_box

4. **`build-main-services.yml`** - Main Pi services
   - Services: micro-ros-agent, zenoh-router
   - Platform: linux/arm64
   - Registry: ghcr.io/krikz/rob_box

5. **`build-base-images.yml`** - Base Docker images
   - Images: ros2-zenoh-humble
   - Platform: linux/arm64

6. **`build-all.yml`** - Orchestrator
   - Вызывает все build workflows
   - Используется в auto-merge

### Валидация

7. **`validate-docker-compose.yml`** - Docker Compose validation
   - Проверка синтаксиса docker-compose.yaml

## Quick Reference

### Feature Branch Workflow

```bash
# 1. Создать feature ветку
git checkout -b feature/my-feature

# 2. Разработка + commit
git add .
git commit -m "feat: add new feature"

# 3. Push → автоматический merge в develop
git push origin feature/my-feature

# ✅ GitHub Actions: build → merge → delete branch
```

### Production Release

```bash
# Push в develop → автоматический merge в main
git push origin develop

# ✅ GitHub Actions: build all → merge to main → release tag
```

### Manual Trigger

```bash
# Ручной запуск workflow
gh workflow run build-vision-services.yml --ref develop

# Или через GitHub UI:
# Actions → Build Vision Pi Services → Run workflow
```

## Tag Convention

| Branch | Docker Tag | Use Case |
|--------|-----------|----------|
| `main` | `humble-latest` | Production |
| `develop` | `humble-dev` | Development/Testing |
| `feature/*` | `humble-dev` | After auto-merge |
| SHA | `humble-<sha>` | Specific version |

## Workflow Triggers

| Workflow | Trigger | When |
|----------|---------|------|
| auto-merge-feature-to-develop | Push to `feature/**` | After feature commit |
| auto-merge-to-main | Push to `develop` | After develop update |
| build-vision-services | Push to `docker/vision/**` | Vision services changed |
| build-main-services | Push to `docker/main/**` | Main services changed |
| build-all | Called by other workflows | Full build |

## Permissions

Workflows require these permissions:

```yaml
permissions:
  contents: write        # For git operations
  packages: write        # For ghcr.io push
  pull-requests: write   # For PR operations
```

## Documentation

Полная документация: [`../docs/CI_CD_PIPELINE.md`](../docs/CI_CD_PIPELINE.md)

## Monitoring

- **GitHub Actions:** https://github.com/krikz/rob_box_project/actions
- **Docker Registry:** https://github.com/krikz?tab=packages
- **Logs:** Each workflow run provides detailed logs

## Contact

For workflow issues, check:
1. GitHub Actions logs
2. `docs/CI_CD_PIPELINE.md` - troubleshooting section
3. Create issue with `ci/cd` label
