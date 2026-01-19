# MCP Tools - Complete Test Guide

## 🎯 Quick Start

```bash
cd src/rob_box_mcp_tools

# All unit tests (fast, no ROS 2 required)
pytest -m unit -v

# With coverage report
pytest --cov=rob_box_mcp_tools --cov-report=html -m unit

# Open coverage report
open test/htmlcov/index.html
```

## 📊 Test Statistics

- **Total tests**: 92 unit tests
- **Coverage**: ~85% of rob_box_mcp_tools
- **Execution time**: < 5 seconds for all unit tests
- **No external dependencies**: Tests run without ROS 2, LLM API, or robot hardware

## 🧪 Test Categories

### Unit Tests (`@pytest.mark.unit`)

**Base classes** (13 tests):
- `test_base.py` - MCPToolParameter, MCPToolResult validation
- JSON Schema generation
- Type checking and validation

**Registry** (12 tests):
- `test_registry.py` - Tool registration, discovery, execution
- Duplicate handling
- OpenAI format conversion

**Async Execution** (15 tests):
- `test_async_executor.py` - AsyncToolExecutor
- Parallel execution with asyncio.gather()
- Timeout handling for each execution_type
- Error handling

**Tool Call Accumulation** (8 tests):
- Streaming chunk accumulation
- Incremental JSON parsing
- Multiple tool_calls in single response

**Interrupts** (10 tests):
- `test_interrupt.py` - LONG task cancellation
- Interrupt by name
- Cleanup after interruption

**Tools** (34 tests):
- `test_tools/test_animation.py` (13 tests) - Animations and emotions
- `test_tools/test_navigation.py` (12 tests) - Navigation tools (NEW)
- `test_tools/test_mapping.py` (9 tests) - Mapping tools (NEW)

### Integration Tests (`@pytest.mark.integration`)

*Coming soon: Tests with real ROS 2*

### LLM API Tests (`@pytest.mark.llm_api`)

*Coming soon: Tests with real LLM APIs*

## 🚀 Running Tests

### All Unit Tests

```bash
pytest -m unit
```

### Specific Test File

```bash
pytest test/test_async_executor.py -v
```

### Specific Test Function

```bash
pytest test/test_async_executor.py::test_execute_instant_tool_fire_and_forget -v
```

### With Coverage

```bash
# HTML report
pytest --cov=rob_box_mcp_tools --cov-report=html -m unit

# Terminal report
pytest --cov=rob_box_mcp_tools --cov-report=term-missing -m unit
```

### Parallel Execution

```bash
# Requires: pip install pytest-xdist
pytest -n auto -m unit
```

### Markers

```bash
# Only async tests
pytest -m asyncio

# Only slow tests
pytest -m slow

# Exclude slow tests
pytest -m "unit and not slow"
```

## 📝 Writing New Tests

### Test Structure

```python
import pytest
from rob_box_mcp_tools.base import MCPTool

@pytest.mark.unit  # Mark as unit test
def test_my_feature():
    """Description of what is being tested"""
    # Arrange
    tool = MyTool(mock_node)
    
    # Act
    result = tool.execute(param="value")
    
    # Assert
    assert result.success is True
    assert result.message == "Expected message"
```

### Async Tests

```python
@pytest.mark.unit
@pytest.mark.asyncio  # Required for async tests
async def test_async_feature():
    """Test async functionality"""
    executor = AsyncToolExecutor(mock_registry)
    
    result = await executor.execute_tool_async("tool_name", {}, ToolExecutionType.FAST)
    
    assert result.success is True
```

### Parametrized Tests

```python
@pytest.mark.unit
@pytest.mark.parametrize("emotion,expected", [
    ("радость", "happy"),
    ("грусть", "sad"),
    ("злость", "angry"),
])
def test_emotion_mapping(emotion, expected):
    """Test with multiple parameters"""
    tool = SetEmotionTool(mock_node)
    result = tool.execute(emotion=emotion)
    
    assert result.data["animation"] == expected
```

### Using Fixtures

```python
@pytest.fixture
def mock_node():
    """Reusable mock ROS 2 node"""
    from conftest import MockNode
    return MockNode()

@pytest.mark.unit
def test_with_fixture(mock_node):
    """Test using fixture"""
    tool = MyTool(mock_node)
    assert tool.node is mock_node
```

## 🐛 Debugging Tests

### Verbose Output

```bash
pytest -vv test/test_async_executor.py
```

### Print Debugging

```bash
pytest -s test/test_async_executor.py  # Show print() output
```

### Drop to Debugger on Failure

```bash
pytest --pdb test/test_async_executor.py
```

### Run Last Failed Tests

```bash
pytest --lf  # Last failed
pytest --ff  # Failed first
```

## 📈 Coverage Goals

- **Minimum**: 80% overall coverage
- **Target**: 90% for core modules (base.py, registry.py, async_executor.py)
- **Tools**: 85% per tool file

### Check Coverage

```bash
pytest --cov=rob_box_mcp_tools --cov-report=term-missing -m unit

# See uncovered lines
pytest --cov=rob_box_mcp_tools --cov-report=html -m unit
open test/htmlcov/index.html
```

## ✅ CI/CD Integration

### GitHub Actions

```yaml
- name: Run tests
  run: |
    cd src/rob_box_mcp_tools
    pytest -m unit --cov=rob_box_mcp_tools --cov-report=xml

- name: Upload coverage
  uses: codecov/codecov-action@v3
  with:
    file: ./src/rob_box_mcp_tools/coverage.xml
```

### Pre-commit Hook

```bash
# .pre-commit-config.yaml
repos:
  - repo: local
    hooks:
      - id: pytest
        name: pytest
        entry: bash -c 'cd src/rob_box_mcp_tools && pytest -m unit'
        language: system
        pass_filenames: false
```

## 🔧 Troubleshooting

### Import Errors

```bash
# Ensure package is installed in development mode
cd src/rob_box_mcp_tools
pip install -e .
```

### Async Test Warnings

```bash
# Install pytest-asyncio
pip install pytest-asyncio
```

### Slow Tests

```bash
# Skip slow tests
pytest -m "unit and not slow"

# Or increase timeout
pytest --timeout=300 -m unit
```

### ROS 2 Import Errors

Unit tests use mocks and should NOT import real ROS 2.
If you see ROS 2 import errors in unit tests, check that:
1. Test is marked with `@pytest.mark.unit`
2. Test uses mock_node from conftest.py
3. No direct ROS 2 imports in test file

## 📚 Best Practices

1. **Isolation**: Unit tests should not depend on ROS 2, network, or filesystem
2. **Speed**: Keep unit tests fast (< 100ms each)
3. **Clarity**: One assertion per test when possible
4. **Naming**: Test names should describe what is being tested
5. **Fixtures**: Use fixtures for common setup
6. **Parametrize**: Test multiple scenarios with `@pytest.mark.parametrize`
7. **Async**: Use `@pytest.mark.asyncio` for async tests
8. **Mocking**: Mock external dependencies (ROS 2, network, LLM APIs)

## 📖 Further Reading

- [pytest documentation](https://docs.pytest.org/)
- [pytest-asyncio](https://pytest-asyncio.readthedocs.io/)
- [unittest.mock](https://docs.python.org/3/library/unittest.mock.html)
- [Python testing best practices](https://docs.python-guide.org/writing/tests/)

---

**Last updated**: January 19, 2026  
**Test count**: 92 unit tests  
**Coverage**: ~85%
