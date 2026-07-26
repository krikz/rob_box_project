# Testing Patterns

**Analysis Date:** 2026-05-15

## Test Framework

**Runner:** pytest ≥ 6.0, invoked via `colcon test` or directly.

**Config:** `src/rob_box_voice/pytest.ini` (most complete config in the project).

**Key addopts:**
```ini
addopts =
    --verbose
    --strict-markers
    --tb=short
    --color=yes
    -ra
```

**Assertion library:** Standard pytest assertions + `unittest.TestCase.assert*` (in older tests).

**Additional plugins:**
- `pytest-cov` — coverage reporting
- `pytest-mock` — mock utilities

**Run commands:**
```bash
# All packages (from workspace root)
source install/setup.bash
colcon test
colcon test-result --verbose

# Specific package
colcon test --packages-select rob_box_voice
colcon test --packages-select rob_box_voice rob_box_animations rob_box_perception

# Only unit tests (fast)
colcon test --packages-select rob_box_voice --pytest-args -m unit

# Skip hardware and API tests
colcon test --packages-select rob_box_voice --pytest-args "-m 'not hardware and not api'"

# Direct pytest (from package dir)
cd src/rob_box_voice
pytest test/ -v

# With coverage
pytest --cov=rob_box_voice --cov-report=html
pytest --cov=rob_box_voice --cov-report=term-missing
```

---

## Test File Organization

**Total test files:** 79 (across all packages)

**Per-package layout:** `test/` directory at package root.

**Simple packages** (`led_matrix_driver`, `rob_box_perception`):
```
src/rob_box_perception/
└── test/
    ├── __init__.py
    ├── test_copyright.py       # ament_copyright linter test
    ├── test_flake8.py          # ament_flake8 linter test
    ├── test_pep257.py          # ament_pep257 linter test
    ├── test_health_monitor.py  # unit tests (unittest.TestCase style)
    ├── test_reflection_node.py
    ├── test_context_aggregator.py
    ├── test_time_provider.py
    ├── test_startup_greeting_node.py
    ├── test_memory_integration.py
    ├── test_summarization.py
    └── unit/core/
        ├── test_event_detector.py
        ├── test_memory_manager.py
        └── test_prompt_formatter.py
```

**Complex packages** (`rob_box_voice`):
```
src/rob_box_voice/
├── pytest.ini
└── test/
    ├── __init__.py
    ├── test_copyright.py, test_flake8.py, test_pep257.py  # ament lint
    ├── test_dialogue_node.py, test_command_node.py, ...    # legacy node tests
    └── unit/
        ├── test_compositor_prompt.py
        ├── test_voice_base_requirements.py
        ├── node/
        │   ├── conftest.py                  # rclpy full mock (sys.modules)
        │   ├── test_pure_methods.py
        │   ├── test_agent_loop.py
        │   └── test_faq_event_mode.py
        ├── core/
        │   ├── test_dialogue_manager.py
        │   ├── test_conversation_history.py
        │   ├── test_command_parser.py
        │   ├── test_speech_formatter.py
        │   ├── test_voice_command_handler.py
        │   ├── test_faq_loader.py
        │   ├── test_faq_store.py
        │   ├── test_music_stack_validation.py
        │   ├── test_music_runtime_assets.py
        │   └── test_sc_only_custom_synthdefs.py
        └── llm/
            ├── test_provider_manager.py
            ├── test_tool_call_executor.py
            └── test_streaming_handler.py
```

**`rob_box_mcp_tools` layout:**
```
src/rob_box_mcp_tools/
└── test/
    ├── conftest.py           # MockLogger, MockNode fixtures
    ├── test_base.py
    ├── test_registry.py
    ├── test_waypoint_store.py
    ├── test_mcp_server.py
    ├── test_llm_integration.py
    ├── unit/core/
    │   └── test_tool_call_accumulator.py
    └── test_tools/
        ├── test_music.py
        ├── test_animation.py
        └── test_mapping.py
```

**File naming:** `test_<module_name>.py` — mirrors source module name.

---

## Test Structure

### Style 1: pytest classes (preferred for new tests)

Used in `src/rob_box_voice/test/unit/` — no `unittest.TestCase` inheritance:

```python
#!/usr/bin/env python3
"""Unit tests for DialogueManager"""

import pytest
from rob_box_voice.core.dialogue_manager import DialogueManager, DialogueState


class TestDialogueManagerWakeWords:
    """Test wake word detection"""

    def test_is_wake_word_detects_default_words(self):
        manager = DialogueManager()
        assert manager.is_wake_word('робок привет') is True

    def test_is_wake_word_ignores_non_wake_words(self):
        manager = DialogueManager()
        assert manager.is_wake_word('привет') is False
```

See: `src/rob_box_voice/test/unit/core/test_dialogue_manager.py`

### Style 2: `unittest.TestCase` with `rclpy.init/shutdown`

Used for tests that instantiate real ROS 2 nodes (integration-leaning):

```python
import unittest
import rclpy
from rob_box_perception.health_monitor import HealthMonitor


class TestHealthMonitor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 once for all tests in this class"""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = HealthMonitor()

    def tearDown(self):
        self.node.destroy_node()

    def test_node_initialization(self):
        self.assertEqual(self.node.get_name(), 'health_monitor')
        self.assertIsNotNone(self.node.rosout_sub)
```

See: `src/rob_box_perception/test/test_health_monitor.py`

---

## Test Markers

Defined in `src/rob_box_voice/pytest.ini`. `--strict-markers` enforced — undeclared markers cause errors.

| Marker | Purpose | Speed |
|--------|---------|-------|
| `@pytest.mark.unit` | No external dependencies | <1s |
| `@pytest.mark.integration` | May require ROS 2 services | 1–5s |
| `@pytest.mark.hardware` | Requires physical devices | Manual |
| `@pytest.mark.slow` | Takes >1 second | Slow |
| `@pytest.mark.api` | Calls external APIs (DeepSeek, Yandex) | Slow + network |

**Ament lint markers** (in `test_flake8.py`, `test_pep257.py`, `test_copyright.py`):
- `@pytest.mark.flake8`
- `@pytest.mark.pep257`
- `@pytest.mark.copyright`
- `@pytest.mark.linter`

---

## Mocking

### Strategy 1: Full `sys.modules` ROS mock via `conftest.py`

The most complete approach — used for `DialogueNode` tests (which import heavy ROS 2 + OpenAI deps). The `conftest.py` installs mocks into `sys.modules` **before any import**.

`src/rob_box_voice/test/unit/node/conftest.py` provides:
- `FakeNode` — minimal `rclpy.node.Node` stub with `get_logger()`, `declare_parameter()`, `create_publisher()`, etc.
- All ROS 2 message types mocked as `MagicMock`
- No actual ROS 2 sockets opened — runs in plain CI

```python
# conftest.py installs this BEFORE test imports:
class FakeNode:
    def __init__(self, name, **kwargs):
        self._name = name
        self._logger = MagicMock()

    def get_logger(self):
        return self._logger

    def declare_parameter(self, name, default=None):
        return MagicMock()

    def create_publisher(self, *args, **kwargs):
        return MagicMock()

    def create_subscription(self, *args, **kwargs):
        return MagicMock()

    def create_timer(self, *args, **kwargs):
        return MagicMock()
```

### Strategy 2: `conftest.py` fixture-based mocks (without sys.modules)

Used in `src/rob_box_mcp_tools/test/conftest.py` — provides `MockLogger`, `MockNode` fixtures:

```python
class MockLogger:
    def __init__(self):
        self.info_messages = []
        self.warning_messages = []
        self.error_messages = []

    def info(self, msg: str):
        self.info_messages.append(msg)
```

### Strategy 3: `@patch` decorator for external dependencies

```python
from unittest.mock import Mock, patch, MagicMock

@patch('requests.post')
def test_api_call(mock_post):
    mock_response = Mock()
    mock_response.status_code = 200
    mock_response.json.return_value = {'result': 'success'}
    mock_post.return_value = mock_response
    result = call_api()
    assert result == 'success'

@patch('pyaudio.PyAudio')
def test_audio_device(mock_pyaudio):
    mock_stream = Mock()
    mock_pyaudio.return_value.open.return_value = mock_stream
    node = AudioNode()
    assert node.stream is not None
```

### Mocking ROS publishers directly

```python
def test_publisher_called():
    mock_publisher = Mock()
    node.publisher = mock_publisher
    node.publish_message("test")
    mock_publisher.publish.assert_called_once()
```

### Mock streaming API chunks

Pattern from `src/rob_box_voice/test/unit/node/test_agent_loop.py`:

```python
def _make_chunk(content=None, tool_calls_delta=None, finish_reason=None, usage=None):
    """Create mock OpenAI streaming API chunk."""
    chunk = MagicMock()
    chunk.choices[0].delta.content = content
    chunk.choices[0].delta.tool_calls = tool_calls_delta
    chunk.choices[0].finish_reason = finish_reason
    chunk.usage = None
    return chunk
```

---

## Fixtures

**pytest fixtures** — used in newer tests. Defined in `conftest.py` or inline.

```python
@pytest.fixture
def sample_audio_data():
    """Provide sample audio data for tests"""
    import numpy as np
    return np.random.randint(-32768, 32767, 1024, dtype=np.int16)

@pytest.fixture
def sample_providers():
    """Sample provider configurations for testing"""
    return {
        "deepseek": ProviderConfig(
            name="DeepSeek",
            base_url="https://api.deepseek.com",
            model="deepseek-chat",
            env_var="LLM_API_KEY",
            fallback_env="DEEPSEEK_API_KEY",
        ),
    }
```

See: `src/rob_box_voice/test/unit/llm/test_provider_manager.py`

---

## Ament Lint Tests

Every ROS 2 package includes ament lint tests in `test/`:

**`test_flake8.py`** — runs `ament_flake8` on the package source:
```python
from ament_flake8.main import main_with_errors
import pytest

@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=['.'])
    assert rc == 0, f'Found {len(errors)} code style errors: {errors}'
```

**`test_pep257.py`** — runs `ament_pep257` docstring linter:
```python
from ament_pep257.main import main
@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
```

**`test_copyright.py`** — checks copyright headers. Currently `@pytest.mark.skip` in most packages because headers are not yet added to generated source files.

---

## Coverage

**Configuration** (in `src/rob_box_voice/pytest.ini`):
```ini
[coverage:run]
source = rob_box_voice
omit =
    */test/*
    */__pycache__/*
    */build/*
    */install/*

[coverage:report]
exclude_lines =
    pragma: no cover
    def __repr__
    raise AssertionError
    raise NotImplementedError
    if __name__ == .__main__.:
    if TYPE_CHECKING:
    @abstractmethod
```

**Current coverage** (from `coverage.json` / `htmlcov/`):
- **13%** overall (179 covered / 1331 total statements)
- Coverage report stored in `htmlcov/` (generated locally, not committed)
- Key gaps: `dialogue_node.py` (1111 stmts, 0% covered), `tts_node.py` (423 stmts, 0%), `audio_node.py` (216 stmts, 0%)
- Best covered: `health_monitor.py` (80 stmts, 25 covered = ~31%), `startup_greeting_node.py` (82 stmts, 19 covered = ~23%)

**Goals** (from `docs/development/TESTING_GUIDE.md`):
| Component | Target |
|-----------|--------|
| Core nodes | >80% |
| Utilities | >70% |
| Integration | >60% |

**Generate reports:**
```bash
pytest --cov=rob_box_voice --cov-report=html   # → htmlcov/index.html
pytest --cov=rob_box_voice --cov-report=term-missing
pytest --cov=rob_box_voice --cov-fail-under=80
```

---

## CI Integration

### `G-Run Tests.yml` (`.github/workflows/G-Run Tests.yml`)

**Triggers:** Push/PR to `develop` or `main` when `src/**/*.py` files change.

**Jobs:**

| Job | Runner | When |
|-----|--------|------|
| `unit-tests` | ubuntu-22.04 | Every push/PR |
| `integration-tests` | ubuntu-22.04 | Only on `develop`/`main` |
| `test-summary` | ubuntu-latest | After unit-tests |

**Unit tests job steps:**
1. Setup ROS 2 kilted (`ros-tooling/setup-ros@v0.7`)
2. Install `pytest pytest-cov pytest-mock`
3. `colcon build --packages-up-to rob_box_voice rob_box_animations rob_box_perception`
4. `colcon test --packages-select rob_box_voice rob_box_animations rob_box_perception --return-code-on-test-failure`
5. Upload coverage artifacts (retained 30 days)

**Integration tests:** Run only on `develop`/`main`, use `pytest -m integration`.

### `G-Integration Tests.yml` (`.github/workflows/G-Integration Tests.yml`)

Docker-based integration tests with QEMU arm64 emulation on `ubuntu-latest`. Triggered after successful `G-Build Vision Pi Services`. Tests run against the actual Docker image with mock LLM server.

**Test infrastructure:**
- `docker/vision/test/mock_llm/` — mock LLM server
- `docker/vision/test/ollama/` — Ollama test instance
- `docker/vision/test/scenario_runner/` — scenario executor
- `docker/vision/test/docker-compose.test.yml` — test composition

---

## Test Writing Guidelines

### One assertion per test
```python
# BAD
def test_everything():
    node = AudioNode()
    assert node.rate == 16000
    assert node.publisher is not None

# GOOD — separate tests
def test_sample_rate():
    node = AudioNode()
    assert node.rate == 16000

def test_publisher_created():
    node = AudioNode()
    assert node.publisher is not None
```

### Test names describe behavior
```python
def test_is_wake_word_detects_default_words(): ...
def test_enable_silence_sets_silenced_state(): ...
def test_remove_wake_word_strips_prefix(): ...
```

### Test `core/` modules without ROS

The `core/` subdirectory pattern in `rob_box_voice` and `rob_box_mcp_tools` separates business logic from ROS. Tests for `core/` modules require **no ROS mocking at all**:

```python
# No rclpy, no conftest needed:
from rob_box_voice.core.dialogue_manager import DialogueManager

def test_is_wake_word():
    manager = DialogueManager()
    assert manager.is_wake_word('робок привет') is True
```

### Async testing

For streaming/async patterns (from `test_agent_loop.py`):
```python
from concurrent.futures import TimeoutError as FuturesTimeoutError

def test_timeout_handling():
    # Use direct call, not asyncio.run for sync-wrapped async
    with pytest.raises(FuturesTimeoutError):
        node._execute_with_timeout(slow_func, timeout=0.1)
```

---

*Testing analysis: 2026-05-15*
