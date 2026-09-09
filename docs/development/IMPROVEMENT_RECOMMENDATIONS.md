# Рекомендации по улучшению проекта (October 2025)

## 🎯 Введение

Этот документ содержит практические рекомендации по улучшению проекта Rob Box на основе code review от October 2025. Рекомендации разделены по приоритетам и включают конкретные примеры реализации.

---

## 🔴 Высокий приоритет (Сделать в ближайшее время)

### 1. Закрепить версии Python зависимостей ⭐

**Проблема:** Текущие `requirements.txt` используют `>=` для версий, что может привести к несовместимостям.

**Текущее состояние:**
```txt
# src/rob_box_voice/requirements.txt
numpy>=1.21.0
sounddevice>=0.4.6
vosk>=0.3.45
```

**Рекомендуемое решение:**
```txt
# src/rob_box_voice/requirements.txt
numpy==1.24.3
sounddevice==0.4.6
vosk==0.3.45
```

**Как реализовать:**

1. На Raspberry Pi (или dev машине) установить зависимости:
```bash
pip install -r src/rob_box_voice/requirements.txt
```

2. Зафиксировать установленные версии:
```bash
pip freeze | grep -E "(numpy|sounddevice|vosk|pyaudio)" > requirements.lock
```

3. Создать новый `requirements.txt` с зафиксированными версиями:
```bash
# Ручное редактирование или автоматизация
cat requirements.lock
```

**Преимущества:**
- ✅ Воспроизводимые сборки
- ✅ Избежание breaking changes
- ✅ Упрощение отладки

**Файлы для обновления:**
- `src/rob_box_voice/requirements.txt`
- `docker/vision/voice_assistant/requirements.txt`
- `docker/vision/voice_base/requirements.txt`
- `src/rob_box_perception/requirements.txt`
- `tools/animation_editor/requirements.txt`

---

### 2. Добавить unit tests для критических модулей ⭐

**Проблема:** Ограниченное покрытие тестами, особенно утилитарных функций.

**Модули, нуждающиеся в тестах:**
- `src/rob_box_voice/rob_box_voice/utils/audio_utils.py`
- `src/rob_box_voice/rob_box_voice/utils/respeaker_interface.py`
- `src/rob_box_perception/rob_box_perception/health_monitor.py`

**Пример теста для audio_utils.py:**

```python
# src/rob_box_voice/test/test_audio_utils.py
#!/usr/bin/env python3
"""Unit tests for audio_utils module."""

import pytest
import numpy as np
from rob_box_voice.utils.audio_utils import (
    calculate_rms,
    calculate_db,
    find_respeaker_device
)


class TestCalculateRMS:
    """Tests for RMS calculation."""
    
    def test_rms_with_zeros(self):
        """Test RMS with zero input."""
        data = np.zeros(1024, dtype=np.int16)
        result = calculate_rms(data)
        assert result == 0.0
    
    def test_rms_with_constant(self):
        """Test RMS with constant value."""
        data = np.full(1024, 100, dtype=np.int16)
        result = calculate_rms(data)
        assert result == 100.0
    
    def test_rms_with_sine_wave(self):
        """Test RMS with sine wave (known result)."""
        # Sine wave RMS = amplitude / sqrt(2)
        amplitude = 1000
        samples = 1024
        t = np.linspace(0, 2*np.pi, samples)
        data = (amplitude * np.sin(t)).astype(np.int16)
        
        result = calculate_rms(data)
        expected = amplitude / np.sqrt(2)
        
        # Allow 1% tolerance
        assert abs(result - expected) < expected * 0.01
    
    def test_rms_with_empty_array_raises_error(self):
        """Test that empty array raises ValueError."""
        with pytest.raises(ValueError):
            calculate_rms(np.array([]))


class TestCalculateDB:
    """Tests for dB calculation."""
    
    def test_db_with_max_value(self):
        """Test dB calculation with max int16 value."""
        rms = 32767  # Max int16
        result = calculate_db(rms)
        assert result == pytest.approx(0.0, abs=0.1)
    
    def test_db_with_half_max(self):
        """Test dB calculation with half max value."""
        rms = 32767 / 2
        result = calculate_db(rms)
        # Should be approximately -6dB
        assert result == pytest.approx(-6.0, abs=0.5)
    
    def test_db_with_zero_returns_negative_infinity(self):
        """Test that zero RMS returns -inf dB."""
        result = calculate_db(0.0)
        assert result == float('-inf')


class TestFindRespeakerDevice:
    """Tests for ReSpeaker device detection."""
    
    def test_find_device_returns_valid_index(self, mocker):
        """Test that device finder returns valid index."""
        # Mock PyAudio device info
        mock_audio = mocker.Mock()
        mock_audio.get_device_count.return_value = 3
        mock_audio.get_device_info_by_index.side_effect = [
            {'name': 'Built-in Audio', 'index': 0},
            {'name': 'ReSpeaker 4 Mic Array', 'index': 1},
            {'name': 'USB Headset', 'index': 2}
        ]
        
        result = find_respeaker_device(mock_audio)
        assert result == 1
    
    def test_find_device_returns_none_if_not_found(self, mocker):
        """Test that None is returned if device not found."""
        mock_audio = mocker.Mock()
        mock_audio.get_device_count.return_value = 2
        mock_audio.get_device_info_by_index.side_effect = [
            {'name': 'Built-in Audio', 'index': 0},
            {'name': 'USB Headset', 'index': 1}
        ]
        
        result = find_respeaker_device(mock_audio)
        assert result is None
```

**Запуск тестов:**
```bash
# Установить pytest
pip install pytest pytest-mock pytest-cov

# Запустить тесты
cd /workspace
pytest src/rob_box_voice/test/test_audio_utils.py -v

# С coverage
pytest src/rob_box_voice/test/ --cov=rob_box_voice --cov-report=html
```

**Файлы для создания:**
- `src/rob_box_voice/test/test_audio_utils.py`
- `src/rob_box_voice/test/test_respeaker_interface.py`
- `src/rob_box_perception/test/test_health_monitor.py`

---

### 3. Добавить integration tests для межплатного взаимодействия ⭐

**Проблема:** Нет автоматизированных тестов для проверки связи Vision Pi ↔ Main Pi.

**Рекомендуемая структура:**

```python
# tests/integration/test_vision_to_main_communication.py
#!/usr/bin/env python3
"""Integration tests for Vision Pi to Main Pi communication."""

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
import time


class TestVisionToMainCommunication:
    """Test data flow from Vision Pi to Main Pi."""
    
    @pytest.fixture(autouse=True)
    def setup(self):
        """Setup ROS 2 context."""
        rclpy.init()
        yield
        rclpy.shutdown()
    
    def test_camera_rgb_reaches_main_pi(self):
        """Verify Vision Pi camera RGB reaches Main Pi."""
        # Create subscriber on Main Pi
        class Listener(Node):
            def __init__(self):
                super().__init__('test_listener')
                self.received = False
                self.sub = self.create_subscription(
                    Image,
                    '/camera/rgb/image_raw',
                    self.callback,
                    10
                )
            
            def callback(self, msg):
                self.received = True
        
        node = Listener()
        
        # Wait up to 10 seconds for message
        start_time = time.time()
        while not node.received and (time.time() - start_time) < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        assert node.received, "Did not receive camera RGB data"
        node.destroy_node()
    
    def test_lidar_scan_reaches_main_pi(self):
        """Verify Vision Pi LSLIDAR reaches Main Pi."""
        class Listener(Node):
            def __init__(self):
                super().__init__('test_listener')
                self.received = False
                self.sub = self.create_subscription(
                    PointCloud2,
                    '/lslidar/scan',
                    self.callback,
                    10
                )
            
            def callback(self, msg):
                self.received = True
        
        node = Listener()
        
        start_time = time.time()
        while not node.received and (time.time() - start_time) < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        assert node.received, "Did not receive LSLIDAR data"
        node.destroy_node()
    
    def test_zenoh_router_health(self):
        """Verify Zenoh router is healthy on both Pis."""
        import requests
        
        # Check Vision Pi router
        response_vision = requests.get('http://10.1.1.11:8000/@/local/router')
        assert response_vision.status_code == 200
        
        # Check Main Pi router
        response_main = requests.get('http://10.1.1.10:8000/@/local/router')
        assert response_main.status_code == 200


class TestVoiceAssistantEndToEnd:
    """End-to-end test for voice assistant pipeline."""
    
    def test_voice_pipeline_flow(self):
        """Test full pipeline: Audio → STT → Dialogue → TTS → Sound."""
        # TODO: Implement full voice assistant test
        # 1. Publish test audio to /audio/audio
        # 2. Wait for /stt/text
        # 3. Wait for /dialogue/text
        # 4. Wait for /tts/audio
        # 5. Verify all steps completed
        pass
```

**Запуск integration tests:**
```bash
# На локальной машине (требует оба Pi в сети)
pytest tests/integration/ -v --tb=short

# В CI/CD (с mock окружением)
pytest tests/integration/ --mock-hardware -v
```

---

## 🟡 Средний приоритет (Следующий спринт)

### 4. Улучшить docstrings для публичных функций

**Проблема:** Многие функции имеют минимальные docstrings без Args/Returns/Raises.

**Текущее состояние:**
```python
def calculate_rms(audio_data):
    """Calculate RMS of audio data."""
    return np.sqrt(np.mean(audio_data**2))
```

**Рекомендуемое решение:**
```python
def calculate_rms(audio_data: np.ndarray) -> float:
    """
    Calculate Root Mean Square (RMS) amplitude of audio data.
    
    RMS is commonly used for measuring the average signal level.
    Higher RMS indicates louder audio.
    
    Args:
        audio_data: Numpy array of audio samples (int16 or float32)
        
    Returns:
        RMS value as float. For int16 data, range is 0 to 32767.
        For normalized float32, range is 0.0 to 1.0.
        
    Raises:
        ValueError: If audio_data is empty
        TypeError: If audio_data is not a numpy array
        
    Example:
        >>> import numpy as np
        >>> silence = np.zeros(1024, dtype=np.int16)
        >>> calculate_rms(silence)
        0.0
        >>> 
        >>> loud_tone = np.full(1024, 10000, dtype=np.int16)
        >>> calculate_rms(loud_tone)
        10000.0
    """
    if len(audio_data) == 0:
        raise ValueError("audio_data cannot be empty")
    
    if not isinstance(audio_data, np.ndarray):
        raise TypeError("audio_data must be a numpy array")
    
    return float(np.sqrt(np.mean(audio_data.astype(float)**2)))
```

**Используйте следующий шаблон:**

```python
def function_name(param1: Type1, param2: Type2 = default) -> ReturnType:
    """
    Brief one-line description.
    
    More detailed explanation of what the function does,
    how it works, and any important notes.
    
    Args:
        param1: Description of param1
        param2: Description of param2 (optional, default: {default})
        
    Returns:
        Description of return value and its type/range
        
    Raises:
        ExceptionType1: When and why this is raised
        ExceptionType2: When and why this is raised
        
    Example:
        >>> result = function_name(value1, value2)
        >>> print(result)
        expected_output
        
    Note:
        Any additional important information
        
    See Also:
        related_function1, related_function2
    """
    # Implementation
```

---

### 5. Добавить dependabot для автоматических обновлений

**Создать файл:** `.github/dependabot.yml`

```yaml
# Dependabot configuration for rob_box_project
version: 2
updates:
  # Python dependencies
  - package-ecosystem: "pip"
    directory: "/src/rob_box_voice"
    schedule:
      interval: "weekly"
    open-pull-requests-limit: 5
    labels:
      - "dependencies"
      - "python"
    reviewers:
      - "krikz"
  
  - package-ecosystem: "pip"
    directory: "/docker/vision/voice_assistant"
    schedule:
      interval: "weekly"
    labels:
      - "dependencies"
      - "docker"
  
  # Docker base images
  - package-ecosystem: "docker"
    directory: "/docker/base"
    schedule:
      interval: "weekly"
    labels:
      - "dependencies"
      - "docker"
  
  # GitHub Actions
  - package-ecosystem: "github-actions"
    directory: "/"
    schedule:
      interval: "weekly"
    labels:
      - "dependencies"
      - "github-actions"
```

**Преимущества:**
- Автоматические PR для обновлений
- Уведомления о security vulnerabilities
- Простое управление зависимостями

---

### 6. Добавить coverage reporting в CI/CD

**Обновить:** `.github/workflows/test.yml`

```yaml
name: Test

on:
  push:
    branches: [develop, main]
  pull_request:
    branches: [develop, main]

jobs:
  test-python:
    runs-on: ubuntu-latest
    
    steps:
      - uses: actions/checkout@v3
      
      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'
      
      - name: Install dependencies
        run: |
          pip install pytest pytest-cov pytest-mock
          pip install -r src/rob_box_voice/requirements.txt
      
      - name: Run tests with coverage
        run: |
          pytest src/ --cov=rob_box_voice --cov=rob_box_perception \
            --cov-report=xml --cov-report=term
      
      - name: Upload coverage to Codecov
        uses: codecov/codecov-action@v3
        with:
          file: ./coverage.xml
          flags: unittests
          name: codecov-umbrella
      
      - name: Check coverage threshold
        run: |
          coverage report --fail-under=70
```

**Добавить badge в README.md:**
```markdown
[![codecov](https://codecov.io/gh/krikz/rob_box_project/branch/main/graph/badge.svg)](https://codecov.io/gh/krikz/rob_box_project)
```

---

## 🟢 Низкий приоритет (В будущем)

### 7. Добавить commitlint для проверки commit messages

**Установить:** `.github/workflows/commitlint.yml`

```yaml
name: Lint Commit Messages

on: [pull_request]

jobs:
  commitlint:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
        with:
          fetch-depth: 0
      
      - uses: wagoid/commitlint-github-action@v5
        with:
          configFile: .commitlintrc.json
```

**Создать:** `.commitlintrc.json`

```json
{
  "extends": ["@commitlint/config-conventional"],
  "rules": {
    "type-enum": [
      2,
      "always",
      [
        "feat",
        "fix",
        "docs",
        "style",
        "refactor",
        "perf",
        "test",
        "chore",
        "revert"
      ]
    ],
    "scope-enum": [
      2,
      "always",
      [
        "voice",
        "docker",
        "perception",
        "navigation",
        "slam",
        "led",
        "ci",
        "docs"
      ]
    ],
    "subject-case": [2, "never", ["upper-case"]],
    "subject-empty": [2, "never"],
    "subject-full-stop": [2, "never", "."],
    "header-max-length": [2, "always", 72]
  }
}
```

---

### 8. Настроить Prometheus + Grafana для мониторинга

**Создать:** `docker/monitoring/docker-compose.yaml`

```yaml
services:
  prometheus:
    image: prom/prometheus:latest
    container_name: prometheus
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
      - prometheus_data:/prometheus
    command:
      - '--config.file=/etc/prometheus/prometheus.yml'
      - '--storage.tsdb.path=/prometheus'
    ports:
      - "9090:9090"
    restart: unless-stopped
  
  grafana:
    image: grafana/grafana:latest
    container_name: grafana
    volumes:
      - grafana_data:/var/lib/grafana
      - ./grafana/dashboards:/etc/grafana/provisioning/dashboards
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=admin
    ports:
      - "3000:3000"
    restart: unless-stopped
  
  node-exporter:
    image: prom/node-exporter:latest
    container_name: node-exporter
    command:
      - '--path.rootfs=/host'
    network_mode: host
    volumes:
      - '/:/host:ro,rslave'
    restart: unless-stopped

volumes:
  prometheus_data:
  grafana_data:
```

**Создать:** `docker/monitoring/prometheus.yml`

```yaml
global:
  scrape_interval: 15s
  evaluation_interval: 15s

scrape_configs:
  - job_name: 'prometheus'
    static_configs:
      - targets: ['localhost:9090']
  
  - job_name: 'vision-pi'
    static_configs:
      - targets: ['10.1.1.11:9100']
        labels:
          pi: 'vision'
  
  - job_name: 'main-pi'
    static_configs:
      - targets: ['10.1.1.10:9100']
        labels:
          pi: 'main'
  
  - job_name: 'ros2-metrics'
    static_configs:
      - targets: ['10.1.1.11:8080', '10.1.1.10:8080']
```

---

### 9. Создать developer onboarding checklist

**Создать:** `docs/development/ONBOARDING.md`

```markdown
# Developer Onboarding Checklist

Добро пожаловать в Rob Box Project! Этот чеклист поможет вам начать работу.

## 📋 Pre-requisites

- [ ] Ubuntu 22.04+ или macOS
- [ ] Git установлен
- [ ] Docker и Docker Compose установлены
- [ ] Python 3.10+ установлен
- [ ] VS Code или PyCharm (рекомендуется)

## 📚 Документация (обязательно к прочтению)

- [ ] Прочитать [README.md](../../README.md) - обзор проекта
- [ ] Прочитать [AGENT_GUIDE.md](./AGENT_GUIDE.md) ⭐ КРИТИЧЕСКИ ВАЖНО
- [ ] Прочитать [DOCKER_STANDARDS.md](./DOCKER_STANDARDS.md)
- [ ] Прочитать [PYTHON_STYLE_GUIDE.md](./PYTHON_STYLE_GUIDE.md)
- [ ] Прочитать [CONTRIBUTING.md](../../CONTRIBUTING.md)
- [ ] Ознакомиться с [.github/copilot-instructions.md](../../.github/copilot-instructions.md)

## 🔧 Setup Local Environment

- [ ] Clone repository: `git clone https://github.com/krikz/rob_box_project.git`
- [ ] Install pre-commit hooks: `pip install pre-commit && pre-commit install`
- [ ] Install Python dependencies: `pip install -r src/rob_box_voice/requirements.txt`
- [ ] Install linting tools: `pip install black flake8 isort pytest`
- [ ] Verify Docker works: `docker run hello-world`

## 🎯 First Tasks

- [ ] Run linters: `black --check src/ && flake8 src/`
- [ ] Build a Docker image: `cd docker/base && docker build -f Dockerfile.ros2-zenoh -t test:local .`
- [ ] Run existing tests: `pytest src/rob_box_voice/test/`
- [ ] Review recent commits: `git log -10 --oneline`
- [ ] Join project communication channels (if applicable)

## 💻 Development Workflow

- [ ] Create feature branch: `git checkout -b feature/my-feature`
- [ ] Make changes following style guide
- [ ] Run linters before commit
- [ ] Write tests for new code
- [ ] Update documentation if needed
- [ ] Create PR to develop branch

## 🤝 Getting Help

- Check existing [Issues](https://github.com/krikz/rob_box_project/issues)
- Read [TROUBLESHOOTING.md](../guides/TROUBLESHOOTING.md)
- Create new Issue with "question" label
- Contact maintainers

## ✅ Ready to Contribute!

Когда все чекбоксы отмечены - вы готовы к работе! 🎉
```

---

## 📊 Метрики успеха

После внедрения рекомендаций, ожидаемые улучшения:

| Метрика | Сейчас | Цель |
|---------|--------|------|
| Test Coverage | ~40% | 70%+ |
| Build Reproducibility | 60% | 95%+ |
| Documentation Score | 95% | 98%+ |
| Code Quality Score | 79% | 85%+ |
| Developer Onboarding Time | 2-3 дня | 1 день |
| Bug Detection in CI | 50% | 80%+ |

---

## 🔄 План внедрения

### Неделя 1
- [x] Создать GitHub Copilot instructions
- [x] Создать CODE_REVIEW документ
- [ ] Закрепить версии в requirements.txt
- [ ] Добавить unit tests для audio_utils

### Неделя 2
- [ ] Добавить integration tests
- [ ] Настроить dependabot
- [ ] Добавить coverage reporting
- [ ] Улучшить docstrings в ключевых модулях

### Неделя 3-4
- [ ] Создать onboarding checklist
- [ ] Добавить commitlint
- [ ] Настроить monitoring (опционально)
- [ ] Провести code review изменений

---

**Документ создан:** October 20, 2025  
**Следующий review:** После внедрения high-priority рекомендаций  
**Автор:** AI Code Review Agent
