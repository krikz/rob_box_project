# 🧪 Testing Guide

**Purpose:** Comprehensive guide for writing and running tests in rob_box_project.

**Why test:**
- 🐛 Catch bugs before production
- 📝 Document expected behavior
- 🔄 Enable safe refactoring
- ⚡ Fast feedback during development

---

## 📋 Table of Contents

- [Quick Start](#quick-start)
- [Test Structure](#test-structure)
- [Writing Tests](#writing-tests)
- [Running Tests](#running-tests)
- [GitHub Actions CI](#github-actions-ci)
- [Coverage Reports](#coverage-reports)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

---

## ⚡ Quick Start

### Run all tests in workspace

```bash
cd ~/voice_ws
source install/setup.bash

# Run tests for all packages
colcon test

# View results
colcon test-result --verbose
```

### Run tests for specific package

```bash
# Voice assistant tests
colcon test --packages-select rob_box_voice

# Animation tests
colcon test --packages-select rob_box_animations

# With pytest arguments
colcon test --packages-select rob_box_voice \
  --pytest-args -v -s
```

### Run specific test file

```bash
cd src/rob_box_voice
pytest test/test_audio_node.py -v
```

### Run with coverage

```bash
cd src/rob_box_voice
pytest --cov=rob_box_voice --cov-report=html
# Open htmlcov/index.html in browser
```

---

## 📁 Test Structure

### Directory Layout

```
src/rob_box_voice/
├── rob_box_voice/          # Source code
│   ├── audio_node.py
│   ├── dialogue_node.py
│   └── ...
├── test/                   # Tests directory
│   ├── __init__.py
│   ├── test_audio_node.py      # Unit tests for audio_node
│   ├── test_dialogue_node.py   # Unit tests for dialogue_node
│   ├── test_integration.py     # Integration tests
│   └── fixtures/               # Test data
│       ├── sample_audio.wav
│       └── mock_responses.json
├── pytest.ini              # pytest configuration
└── package.xml             # ROS2 package manifest
```

### Test Categories

| Category | Marker | Purpose | Speed |
|----------|--------|---------|-------|
| **Unit** | `@pytest.mark.unit` | Test individual functions/classes | Fast (<1s) |
| **Integration** | `@pytest.mark.integration` | Test multiple components together | Medium (1-5s) |
| **Hardware** | `@pytest.mark.hardware` | Require physical devices | Slow + manual |
| **API** | `@pytest.mark.api` | Call external APIs | Slow + network |

---

## ✍️ Writing Tests

### Unit Test Template

```python
#!/usr/bin/env python3
"""
Unit tests for <NodeName>
Description of what this node does
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import rclpy
from rclpy.node import Node

# Import node to test
from rob_box_voice.my_node import MyNode


class TestMyNode(unittest.TestCase):
    """Test suite for MyNode"""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 context once for all tests"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 context"""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures before each test"""
        self.node = MyNode()

    def tearDown(self):
        """Clean up after each test"""
        self.node.destroy_node()

    def test_node_creation(self):
        """Test that node can be created"""
        self.assertIsInstance(self.node, Node)
        self.assertEqual(self.node.get_name(), 'my_node')

    def test_publisher_creation(self):
        """Test that publishers are created"""
        publishers = self.node.get_publisher_names_and_types_by_node(
            'my_node', 'rob_box_voice'
        )
        self.assertIn('/my_topic', [pub[0] for pub in publishers])

    @patch('external_library.function')
    def test_with_mocked_dependency(self, mock_function):
        """Test with mocked external dependency"""
        mock_function.return_value = "mocked_value"
        result = self.node.call_external_function()
        self.assertEqual(result, "expected_result")
        mock_function.assert_called_once()


if __name__ == '__main__':
    unittest.main()
```

### Using pytest markers

```python
import pytest

@pytest.mark.unit
def test_fast_unit_test():
    """Fast test, no external dependencies"""
    assert 1 + 1 == 2

@pytest.mark.integration
def test_ros_nodes_communication():
    """Test communication between nodes"""
    # Requires ROS2 running
    pass

@pytest.mark.hardware
def test_audio_device():
    """Test with real audio device"""
    # Requires ReSpeaker USB
    pass

@pytest.mark.api
def test_deepseek_api():
    """Test with real DeepSeek API"""
    # Requires DEEPSEEK_API_KEY
    pass

@pytest.mark.slow
def test_long_running():
    """Test that takes >1 second"""
    import time
    time.sleep(2)
    pass
```

### Mocking external dependencies

```python
from unittest.mock import Mock, patch, MagicMock

# Mock API call
@patch('requests.post')
def test_api_call(mock_post):
    mock_response = Mock()
    mock_response.status_code = 200
    mock_response.json.return_value = {'result': 'success'}
    mock_post.return_value = mock_response
    
    # Your test code
    result = call_api()
    assert result == 'success'

# Mock ROS2 publisher
def test_publisher():
    mock_publisher = Mock()
    node.publisher = mock_publisher
    
    node.publish_message("test")
    
    mock_publisher.publish.assert_called_once()

# Mock hardware device
@patch('pyaudio.PyAudio')
def test_audio_device(mock_pyaudio):
    mock_stream = Mock()
    mock_pyaudio.return_value.open.return_value = mock_stream
    
    # Your test code
    node = AudioNode()
    assert node.stream is not None
```

### Testing with fixtures

```python
import pytest
import numpy as np

@pytest.fixture
def sample_audio_data():
    """Provide sample audio data for tests"""
    return np.random.randint(-32768, 32767, 1024, dtype=np.int16)

@pytest.fixture
def mock_deepseek_response():
    """Provide mock DeepSeek API response"""
    return {
        'choices': [
            {'message': {'content': 'Привет! Как дела?'}}
        ]
    }

def test_audio_processing(sample_audio_data):
    """Test using audio fixture"""
    processed = process_audio(sample_audio_data)
    assert processed.shape == sample_audio_data.shape

def test_dialogue_response(mock_deepseek_response):
    """Test using API response fixture"""
    response = parse_deepseek_response(mock_deepseek_response)
    assert response == "Привет! Как дела?"
```

---

## 🏃 Running Tests

### Local Development

#### Run all tests
```bash
cd ~/voice_ws
colcon test
colcon test-result --verbose
```

#### Run specific package
```bash
colcon test --packages-select rob_box_voice
```

#### Run only unit tests (fast)
```bash
colcon test --packages-select rob_box_voice \
  --pytest-args -m unit
```

#### Run with verbose output
```bash
colcon test --packages-select rob_box_voice \
  --event-handlers console_direct+ \
  --pytest-args -v -s
```

#### Run specific test
```bash
# Using colcon
colcon test --packages-select rob_box_voice \
  --pytest-args test/test_audio_node.py::TestAudioNode::test_node_creation

# Using pytest directly
cd src/rob_box_voice
pytest test/test_audio_node.py::TestAudioNode::test_node_creation -v
```

#### Run tests matching pattern
```bash
# All tests with "audio" in name
pytest -k audio -v

# All tests with "dialogue" in name
pytest -k dialogue -v
```

#### Skip slow tests
```bash
colcon test --packages-select rob_box_voice \
  --pytest-args "-m 'not slow'"
```

#### Skip hardware/API tests
```bash
colcon test --packages-select rob_box_voice \
  --pytest-args "-m 'not hardware and not api'"
```

### Coverage

#### Generate HTML coverage report
```bash
cd src/rob_box_voice
pytest --cov=rob_box_voice --cov-report=html
firefox htmlcov/index.html  # or xdg-open
```

#### Terminal coverage report
```bash
pytest --cov=rob_box_voice --cov-report=term-missing
```

#### Check minimum coverage
```bash
pytest --cov=rob_box_voice --cov-fail-under=80
```

---

## 🤖 GitHub Actions CI

### When tests run

**Workflow:** `.github/workflows/test.yml`

**Triggers:**
- Push to `feature/**`, `develop`, `main` (when `.py` files change)
- Pull Request to `develop` or `main`
- Manual trigger (Actions → Run Tests → Run workflow)

### What's tested

| Job | Runs | When |
|-----|------|------|
| **unit-tests** | All unit tests | Every push/PR |
| **integration-tests** | Integration tests | Only on develop/main |

### Viewing results

1. **GitHub Actions tab:**
   ```
   Actions → Run Tests → Latest workflow
   ```

2. **PR checks:**
   - Green ✅ = all tests passed
   - Red ❌ = tests failed (click for logs)

3. **Artifacts:**
   - Coverage reports uploaded for 30 days
   - Download: Actions → Run Tests → Artifacts → coverage-report

### Local simulation

```bash
# Simulate CI environment
docker run -it --rm -v $(pwd):/workspace ubuntu:22.04

# Inside container:
apt-get update
apt-get install -y software-properties-common curl
add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list
apt-get update
apt-get install -y ros-humble-desktop python3-colcon-common-extensions

# Run tests
cd /workspace
source /opt/ros/humble/setup.bash
colcon test
```

---

## 📊 Coverage Reports

### Reading coverage reports

**HTML Report (recommended):**
```bash
pytest --cov=rob_box_voice --cov-report=html
firefox htmlcov/index.html
```

**Features:**
- Color-coded: Green = covered, Red = not covered
- Click files to see line-by-line coverage
- Sort by coverage percentage

**Terminal Report:**
```
---------- coverage: platform linux, python 3.10.12 ----------
Name                          Stmts   Miss  Cover   Missing
-----------------------------------------------------------
rob_box_voice/audio_node.py      45      5    89%   23-27
rob_box_voice/dialogue_node.py   67     12    82%   45-56, 89
-----------------------------------------------------------
TOTAL                           112     17    85%
```

### Coverage goals

| Component | Target | Current | Status |
|-----------|--------|---------|--------|
| **Core nodes** | >80% | TBD | 🟡 |
| **Utilities** | >70% | TBD | 🟡 |
| **Integration** | >60% | TBD | 🟡 |

### Improving coverage

1. **Find uncovered code:**
   ```bash
   pytest --cov=rob_box_voice --cov-report=term-missing
   ```

2. **Focus on:**
   - Error handling branches
   - Edge cases
   - Complex logic

3. **Don't focus on:**
   - Simple getters/setters
   - `__repr__` methods
   - `if __name__ == '__main__'`

---

## 📖 Best Practices

### ✅ DO:

1. **Write tests first (TDD):**
   ```python
   # Write test
   def test_process_audio():
       data = np.zeros(1024)
       result = process_audio(data)
       assert result.shape == (1024,)
   
   # Then implement
   def process_audio(data):
       return data  # Initial implementation
   ```

2. **Test one thing per test:**
   ```python
   # BAD: Testing multiple things
   def test_everything():
       node = AudioNode()
       assert node.rate == 16000
       assert node.publisher is not None
       assert node.process_audio(data) is not None
   
   # GOOD: Separate tests
   def test_sample_rate():
       node = AudioNode()
       assert node.rate == 16000
   
   def test_publisher_created():
       node = AudioNode()
       assert node.publisher is not None
   
   def test_audio_processing():
       node = AudioNode()
       data = np.zeros(1024)
       assert node.process_audio(data) is not None
   ```

3. **Use descriptive test names:**
   ```python
   # BAD
   def test1():
       pass
   
   # GOOD
   def test_audio_node_publishes_16khz_pcm_data():
       pass
   ```

4. **Mock external dependencies:**
   ```python
   @patch('requests.post')  # Mock API
   @patch('pyaudio.PyAudio')  # Mock hardware
   def test_with_mocks(mock_pyaudio, mock_requests):
       pass
   ```

5. **Clean up in tearDown:**
   ```python
   def tearDown(self):
       self.node.destroy_node()
       rclpy.shutdown()
   ```

### ❌ DON'T:

1. **Don't test external libraries:**
   ```python
   # BAD: Testing numpy
   def test_numpy():
       assert np.array([1, 2, 3]).shape == (3,)
   
   # GOOD: Test your code that uses numpy
   def test_audio_conversion():
       result = convert_audio(np.array([1, 2, 3]))
       assert result.dtype == np.float32
   ```

2. **Don't use sleep() in tests:**
   ```python
   # BAD
   def test_async_operation():
       start_operation()
       time.sleep(5)  # Slow!
       assert operation_complete()
   
   # GOOD: Use mocks or timeout
   def test_async_operation():
       with timeout(1):
           result = wait_for_operation()
       assert result is True
   ```

3. **Don't commit test data to git (use fixtures):**
   ```python
   # BAD: Large test files in git
   test/data/audio_sample_100MB.wav
   
   # GOOD: Generate in fixture
   @pytest.fixture
   def sample_audio():
       return np.random.random(16000)
   ```

4. **Don't skip tests without reason:**
   ```python
   # BAD
   @pytest.mark.skip
   def test_something():
       pass
   
   # GOOD
   @pytest.mark.skip(reason="Waiting for API v2 release")
   def test_api_v2_feature():
       pass
   ```

---

## 🐛 Troubleshooting

### Tests fail locally but pass in CI

```bash
# Check Python version
python3 --version  # Should match CI (3.10)

# Check ROS2 sourced
echo $ROS_DISTRO  # Should be 'humble'

# Clean build
cd ~/voice_ws
rm -rf build/ install/ log/
colcon build --packages-select rob_box_voice
colcon test --packages-select rob_box_voice
```

### ImportError: No module named 'rob_box_voice'

```bash
# Build package first
colcon build --packages-select rob_box_voice

# Source workspace
source install/setup.bash

# Then run tests
colcon test --packages-select rob_box_voice
```

### Test hangs forever

```bash
# Kill hanging tests
pkill -9 -f pytest

# Add timeout to test
@pytest.mark.timeout(5)  # Fail after 5 seconds
def test_something():
    pass
```

### "No tests collected"

```bash
# Check test file name starts with test_
ls test/test_*.py

# Check function names start with test_
grep "def test_" test/test_audio_node.py

# Run with verbose to see discovery
pytest --collect-only -v
```

### ROS2 context already initialized

```python
# Use setUpClass/tearDownClass for ROS init
@classmethod
def setUpClass(cls):
    if not rclpy.ok():
        rclpy.init()

@classmethod
def tearDownClass(cls):
    if rclpy.ok():
        rclpy.shutdown()
```

### Coverage report empty

```bash
# Check source path in pytest.ini
[coverage:run]
source = rob_box_voice  # Should match package name

# Run from package directory
cd src/rob_box_voice
pytest --cov=rob_box_voice
```

---

## 📚 Resources

### Documentation
- [pytest docs](https://docs.pytest.org/)
- [ROS2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [unittest mock](https://docs.python.org/3/library/unittest.mock.html)

### Related Files
- [LINTING_GUIDE.md](./LINTING_GUIDE.md) - Code quality guide
- [AI_TROUBLESHOOTING_CHECKLIST.md](./AI_TROUBLESHOOTING_CHECKLIST.md) - Debug checklist
- [.github/workflows/test.yml](../../.github/workflows/test.yml) - CI test workflow

### Example Tests
- `src/rob_box_voice/test/test_audio_node.py` - Unit test example
- `src/rob_box_voice/test/test_dialogue_node.py` - API mocking example

---

## 📈 Metrics

**Phase 2 Goals (from AI_DEVELOPMENT_REVIEW.md):**

| Metric | Before | Target | Current |
|--------|--------|--------|---------|
| Test coverage | 0% | >70% | TBD |
| Tests passing | N/A | 100% | TBD |
| CI test time | N/A | <5 min | TBD |

**Track progress:**
```bash
# Run tests and save results
colcon test --packages-select rob_box_voice
colcon test-result --all | tee test_results.log

# Check coverage
pytest --cov=rob_box_voice --cov-report=term | grep TOTAL
```

---

**Last Updated:** October 16, 2025 (Phase 2, Item 5)
