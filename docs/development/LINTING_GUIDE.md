# 🧹 Code Quality & Linting Setup

**Purpose:** Автоматическая проверка качества кода перед коммитом и в CI/CD.

**Impact:**
- 🐛 Меньше багов в production
- 📏 Единый стиль кода
- ⚡ Быстрый code review
- 🔒 Автоматический поиск секретов

---

## 📋 Table of Contents

- [Quick Start](#quick-start)
- [GitHub Actions (CI/CD)](#github-actions-cicd)
- [Manual Linting](#manual-linting)
- [Fixing Common Issues](#fixing-common-issues)
- [Configuration Files](#configuration-files)

---

## ⚡ Quick Start

Локальных git-хуков (pre-commit) в репо нет: проверки гоняет CI (G-Lint Code).
Перед пушем запусти линтеры вручную, см. [Manual Linting](#manual-linting).

---

## 🤖 GitHub Actions (CI/CD)

### Когда запускается

**Workflow:** `.github/workflows/G-Lint Code.yml`

**Triggers:**
- Push в `feature/**`, `develop`, `main` (если изменились `.py`, `.yaml`, `.yml`, `Dockerfile`)
- Pull Request в `develop` или `main`
- Manual trigger (Actions → Lint Code → Run workflow)

### Что проверяет

| Linter | Проверяет | Время |
|--------|-----------|-------|
| **Black** | Python formatting | ~30s |
| **Flake8** | Python code style | ~45s |
| **isort** | Import sorting | ~20s |
| **yamllint** | YAML syntax/style | ~15s |
| **hadolint** | Dockerfile best practices | ~20s |
| **shellcheck** | Shell script issues | ~25s |

**Total:** ~2-3 minutes

### Как читать результаты

**GitHub Actions → Lint Code → Summary:**
```
📊 Lint Results Summary

| Check              | Status  |
|--------------------|---------|
| Python Linting     | success |
| YAML Linting       | success |
| Dockerfile Linting | success |
| Shell Scripts      | success |
```

**⚠️ Important:** Linting jobs используют `continue-on-error: true`, поэтому workflow НЕ УПАДЕТ при ошибках. Но проблемы будут подсвечены в логах.

**Why `continue-on-error`?**
- Не блокируем срочные fixes
- Можем улучшать код постепенно
- Фокус на critical issues (Docker builds, tests)

### Где смотреть проблемы

1. **GitHub PR page:**
   - Зелёная галочка ✅ = нет критических проблем
   - Желтый warning ⚠️ = есть lint issues (кликни для деталей)

2. **Actions logs:**
   ```
   Python Linting → Black formatting check
   └─ src/rob_box_voice/rob_box_voice/audio_node.py:42: Line too long (125 > 120)
   ```

3. **Summary section:**
   - Quick Fixes команды
   - Список всех проблем

---

## 🛠️ Manual Linting

Линтеры можно запускать вручную (локальных git-хуков в репо нет, проверки гоняет CI).

### Python

```bash
# Install tools
pip install black flake8 isort

# Format code (изменит файлы)
black src/rob_box_voice/ src/rob_box_animations/ tools/animation_editor/

# Sort imports (изменит файлы)
isort src/rob_box_voice/ src/rob_box_animations/ tools/animation_editor/

# Check code quality (только проверка)
flake8 src/rob_box_voice/ \
  --max-line-length=120 \
  --extend-ignore=E203,W503,E501 \
  --statistics

# Check formatting (только проверка)
black --check --diff src/rob_box_voice/
isort --check-only --diff src/rob_box_voice/
```

### YAML

```bash
# Install
pip install yamllint

# Check all YAML
yamllint .github/workflows/ docker/ src/*/config/ src/*/launch/
```

### Dockerfile

```bash
# Install (Linux)
wget -O /tmp/hadolint https://github.com/hadolint/hadolint/releases/download/v2.12.0/hadolint-Linux-x86_64
chmod +x /tmp/hadolint
sudo mv /tmp/hadolint /usr/local/bin/hadolint

# Check Dockerfile
hadolint docker/main/teleop/Dockerfile

# Check all Dockerfiles
find docker/ -name "Dockerfile" -exec hadolint {} +
```

### Shell Scripts

```bash
# Install (Ubuntu/Debian)
sudo apt-get install shellcheck

# Check script
shellcheck scripts/setup/setup_vision_pi.sh

# Check all scripts
find scripts/ -name "*.sh" -exec shellcheck {} +
```

---

## 🔧 Fixing Common Issues

### Black: Line too long

**Problem:**
```
src/rob_box_voice/rob_box_voice/audio_node.py:42: Line too long (125 > 120)
```

**Fix:**
```bash
black src/rob_box_voice/rob_box_voice/audio_node.py
```

Black автоматически разобьёт длинные строки.

---

### Flake8: F401 imported but unused

**Problem:**
```
src/rob_box_voice/rob_box_voice/audio_node.py:10:1: F401 'time' imported but unused
```

**Fix:** Удали неиспользуемый import:
```python
# Before
import time  # ← не используется
import numpy as np

# After
import numpy as np
```

---

### Flake8: E302 expected 2 blank lines

**Problem:**
```
src/rob_box_voice/rob_box_voice/audio_node.py:25:1: E302 expected 2 blank lines, found 1
```

**Fix:** Добавь пустую строку перед определением класса/функции:
```python
# Before
import numpy as np
class AudioNode:  # ← только 1 пустая строка
    pass

# After
import numpy as np


class AudioNode:  # ← 2 пустые строки
    pass
```

---

### isort: Imports incorrectly sorted

**Problem:**
```
ERROR: Imports are incorrectly sorted and/or formatted.
```

**Fix:**
```bash
isort src/rob_box_voice/rob_box_voice/audio_node.py
```

isort автоматически отсортирует imports в правильном порядке:
1. Standard library
2. Third-party packages
3. Local imports

---

### yamllint: Line too long

**Problem:**
```
docker/main/docker-compose.yaml:42:121: [warning] line too long (130 > 120 characters)
```

**Fix:** Разбей длинную строку:
```yaml
# Before
command: ros2 launch rob_box_bringup robot.launch.py use_sim_time:=false robot_description:=/robot_description

# After
command: >
  ros2 launch rob_box_bringup robot.launch.py
  use_sim_time:=false
  robot_description:=/robot_description
```

---

### hadolint: DL3059 Multiple consecutive RUN instructions

**Problem:**
```
docker/main/teleop/Dockerfile:10 DL3059 Multiple consecutive RUN instructions. Consider consolidation.
```

**Fix:** Объедини RUN commands:
```dockerfile
# Before
RUN apt-get update
RUN apt-get install -y curl
RUN apt-get clean

# After
RUN apt-get update && \
    apt-get install -y curl && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
```

---

### shellcheck: SC2086 Double quote to prevent globbing

**Problem:**
```
scripts/setup/setup_vision_pi.sh:15:10: SC2086 Double quote to prevent globbing and word splitting.
```

**Fix:** Добавь кавычки:
```bash
# Before
cp $CONFIG_FILE /etc/

# After
cp "$CONFIG_FILE" /etc/
```

---

## 📁 Configuration Files

### `.yamllint.yml`
**Location:** `/home/ros2/rob_box_project/.yamllint.yml`

**Purpose:** Правила для yamllint

**Key settings:**
```yaml
rules:
  line-length:
    max: 120        # ← максимальная длина строки
  indentation:
    spaces: 2       # ← 2 пробела для отступов
```

---

### `.hadolint.yaml`
**Location:** `/home/ros2/rob_box_project/.hadolint.yaml`

**Purpose:** Правила для Dockerfile линтера

**Key settings:**
```yaml
ignored:
  - DL3008  # ← игнорируем pinning apt versions (ROS специфично)
```

---

### `.github/workflows/G-Lint Code.yml`
**Location:** `/home/ros2/rob_box_project/.github/workflows/G-Lint Code.yml`

**Purpose:** GitHub Actions workflow для CI/CD линтинга

**Key settings:**
```yaml
jobs:
  python-lint:
    steps:
      - name: Check code formatting with Black
        run: black --check --diff src/
        continue-on-error: true  # ← не блокируем workflow
```

---

## 📊 Metrics & Success Criteria

**Phase 2 Goals (from AI_DEVELOPMENT_REVIEW.md):**

| Metric | Before | Target | Current |
|--------|--------|--------|---------|
| CI failures (lint) | N/A | <5% | TBD |
| Code style consistency | 60% | 95% | TBD |
| Time to fix lint issues | 30 min | 5 min | TBD |

---

## 🎓 Best Practices

### ✅ DO:

1. **Fix lint issues in separate commits:**
   ```bash
   git commit -m "style: format code with black"
   git commit -m "feat: add new feature"
   ```

2. **Review lint warnings in PR:**
   - Yellow ⚠️ in GitHub Actions → click → fix

### ❌ DON'T:

1. **Don't ignore lint warnings in CI:**
   - Even with `continue-on-error`, fix issues eventually

2. **Don't commit formatting-only changes mixed with features:**
   ```bash
   # BAD: mixed commit
   git commit -m "Add feature + format all files"
   
   # GOOD: separate commits
   git commit -m "style: format code with black"
   git commit -m "feat: add new feature"
   ```

---

## 🔗 Related Documentation

- [AGENT_GUIDE.md](./AGENT_GUIDE.md) - AI development guide
- [AI_TROUBLESHOOTING_CHECKLIST.md](./AI_TROUBLESHOOTING_CHECKLIST.md) - Debug checklist
- [AI_DEVELOPMENT_REVIEW.md](./AI_DEVELOPMENT_REVIEW.md) - Phase 2 roadmap
- [CI_CD_PIPELINE.md](../CI_CD_PIPELINE.md) - GitHub Actions workflows

---

## 🆘 Troubleshooting

### Black/Flake8 conflicts

Black и Flake8 могут конфликтовать на некоторых правилах. Наш конфиг уже настроен:

```bash
# flake8 игнорирует:
# E203: whitespace before ':'
# W503: line break before binary operator
# E501: line too long (black handles this)
```

---

**Last Updated:** October 16, 2025 (Phase 2, Item 3-4)
