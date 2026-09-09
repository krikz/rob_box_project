# Python Testing Patterns

Comprehensive guide to implementing robust testing strategies in Python using
pytest, fixtures, mocking, parameterization, and test-driven development practices.

## When to Use This Skill

Use this skill when:
- Writing unit tests for Python code (`src/rob_box_*/test/`)
- Setting up test suites for ROS2 nodes with mock topics
- Implementing pytest fixtures for `VoiceMemory`, `MCPTools`, `dialogue_node`
- Testing async code (agent cycle, LLM streaming, VAD)
- Working on TASK-035, TASK-037, TASK-041 (CI/CD tests)

## Core Concepts

### Test Types
- **Unit Tests**: Test individual functions/classes in isolation
- **Integration Tests**: Test interaction between components
- **Functional Tests**: Test complete features end-to-end

### Test Structure (AAA Pattern)
- **Arrange**: Set up test data and preconditions
- **Act**: Execute the code under test
- **Assert**: Verify the results

## Fundamental Patterns

### Pattern 1: Basic pytest Tests

```python
import pytest

def test_something():
    """Test description."""
    result = my_function(input)
    assert result == expected

def test_raises_on_invalid():
    with pytest.raises(ValueError, match="expected message"):
        my_function(invalid_input)
```

### Pattern 2: Fixtures for Setup and Teardown

```python
import pytest
from typing import Generator

@pytest.fixture
def db_session() -> Generator:
    """Fixture that provides connected database."""
    session = create_session()
    yield session
    session.close()

@pytest.fixture(scope="session")
def app_config():
    """Session-scoped fixture - created once per test session."""
    return {"database_url": "sqlite:///:memory:", "debug": True}

@pytest.fixture(scope="module")
def ros_node(app_config):
    """Module-scoped: set up ROS2 mock node once per module."""
    node = MockNode(app_config)
    yield node
    node.destroy()
```

### Pattern 3: Parameterized Tests

```python
import pytest

@pytest.mark.parametrize("input,expected", [
    ("hello", True),
    ("", False),
    (None, False),
])
def test_is_valid(input, expected):
    assert is_valid(input) == expected

# With custom IDs
@pytest.mark.parametrize("value,expected", [
    pytest.param(1, True, id="positive"),
    pytest.param(0, False, id="zero"),
    pytest.param(-1, False, id="negative"),
])
def test_is_positive(value, expected):
    assert (value > 0) == expected
```

### Pattern 4: Mocking with unittest.mock

```python
from unittest.mock import Mock, patch, MagicMock

def test_ros_topic_unavailable():
    """Mock ROS topic to test graceful error handling."""
    with patch("rclpy.node.Node.create_publisher") as mock_pub:
        mock_pub.side_effect = Exception("Topic unavailable")
        tool = NavigateToWaypointTool(node=MockNode())
        result = tool.execute({"x": 1.0, "y": 2.0})
        assert result.success is False
        assert "unavailable" in result.message.lower()

@patch("rob_box_voice.core.llm_adapter.LLMAdapter.call")
def test_agent_cycle_timeout(mock_llm):
    """Test timeout handling without real LLM."""
    mock_llm.side_effect = TimeoutError("LLM timeout")
    with pytest.raises(TimeoutError):
        agent.run("tell me a story")
```

### Pattern 5: Testing Async Code

```python
import pytest
import asyncio
from unittest.mock import AsyncMock

@pytest.mark.asyncio
async def test_async_dialogue():
    """Test async dialogue node response."""
    result = await dialogue_node.process_request("hello")
    assert result is not None

@pytest.mark.asyncio
async def test_mcp_tool_async():
    """Test async MCP tool execution."""
    mock_topic = AsyncMock()
    mock_topic.publish.return_value = None
    tool = AnimationTool(publisher=mock_topic)
    result = await tool.execute({"animation": "police"})
    assert result.success is True
    mock_topic.publish.assert_called_once()
```

### Pattern 6: Monkeypatch for Environment Variables

```python
def test_ollama_url_default(monkeypatch):
    monkeypatch.delenv("OLLAMA_BASE_URL", raising=False)
    assert get_ollama_url() == "http://localhost:11434"

def test_ollama_url_custom(monkeypatch):
    monkeypatch.setenv("OLLAMA_BASE_URL", "http://10.1.1.11:11434")
    assert get_ollama_url() == "http://10.1.1.11:11434"
```

### Pattern 7: Temporary Files and Directories

```python
def test_voice_memory_db(tmp_path):
    """Test VoiceMemory with temporary database."""
    db_path = tmp_path / "voice_memory.db"
    memory = VoiceMemory(db_path=str(db_path))
    memory.save_turn(role="user", content="hello")
    turns = memory.get_recent_turns(limit=5)
    assert len(turns) == 1
    assert turns[0]["content"] == "hello"
```

## Advanced Patterns

### Pattern 8: conftest.py — Shared Fixtures

```python
# src/rob_box_voice/test/conftest.py
import pytest
from unittest.mock import MagicMock

@pytest.fixture(scope="session")
def mock_ros_node():
    """Shared mock ROS2 node for all tests."""
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    return node

@pytest.fixture
def mock_ollama_embedder():
    """Mock Ollama embedder — no real Ollama needed."""
    embedder = MagicMock()
    embedder.embed.return_value = [0.1] * 768  # 768-dim vector
    return embedder

@pytest.fixture
def voice_memory(tmp_path, mock_ollama_embedder):
    """VoiceMemory with temp DB and mock embedder."""
    from rob_box_voice.core.voice_memory import VoiceMemory
    return VoiceMemory(
        db_path=str(tmp_path / "test.db"),
        embedder=mock_ollama_embedder
    )
```

### Pattern 9: Testing Retry/Resilience

```python
from unittest.mock import Mock

def test_llm_retries_on_connection_error():
    """Test LLM adapter retries on transient failures."""
    client = Mock()
    client.chat.side_effect = [
        ConnectionError("timeout"),
        ConnectionError("timeout"),
        {"content": "hello"},
    ]
    adapter = LLMAdapter(client=client, max_retries=3)
    result = adapter.call(messages=[])
    assert result == {"content": "hello"}
    assert client.chat.call_count == 3

def test_graceful_degradation_without_ollama(monkeypatch):
    """Test VoiceMemory works with FTS5 only when Ollama is down."""
    monkeypatch.setenv("OLLAMA_BASE_URL", "http://localhost:99999")
    memory = VoiceMemory(db_path=":memory:")
    results = memory.search("test query")
    assert isinstance(results, list)  # FTS5 fallback works
```

### Pattern 10: Test Markers

```python
import pytest

@pytest.mark.slow
def test_stress_50_iterations():
    """50 iteration stress test — marked as slow."""
    for i in range(50):
        result = agent.run(f"request {i}")
        assert result is not None

@pytest.mark.integration
def test_mcp_server_tools():
    """Integration test requiring running MCP server."""
    pass

@pytest.mark.skipif(
    not os.path.exists("/dev/snd"),
    reason="Requires audio hardware"
)
def test_vad_detection():
    pass
```

## Test Design Principles

### One Behavior Per Test

```python
# BAD - multiple behaviors
def test_voice_memory():
    memory.save_turn(role="user", content="hello")
    memory.save_turn(role="assistant", content="world")
    turns = memory.get_recent_turns(5)
    assert len(turns) == 2
    facts = memory.search("hello")
    assert len(facts) > 0

# GOOD - focused
def test_save_turn_stores_user_message():
    memory.save_turn(role="user", content="hello")
    turns = memory.get_recent_turns(1)
    assert turns[0]["content"] == "hello"
    assert turns[0]["role"] == "user"

def test_search_returns_relevant_results():
    memory.save_turn(role="user", content="моя кошка живет на кухне")
    results = memory.search("кухня")
    assert len(results) > 0
```

## Testing Best Practices

### Test Naming Convention

```python
# Pattern: test_<unit>_<scenario>_<expected_outcome>
def test_navigate_to_waypoint_with_nav2_unavailable_returns_error():
    ...

def test_memory_save_with_valid_fact_stores_in_db():
    ...

def test_dialogue_node_on_interrupt_stops_cleanly():
    ...
```

### Coverage Reporting

```bash
# Run with coverage
pytest --cov=src/rob_box_voice --cov-report=term-missing tests/

# Fail if below threshold
pytest --cov=src/rob_box_mcp_tools --cov-fail-under=80 tests/
```

### pytest.ini Configuration

```ini
[pytest]
testpaths = test
python_files = test_*.py
addopts = -v --strict-markers --tb=short
markers =
    slow: marks tests as slow (>5s)
    integration: marks integration tests requiring ROS2
    unit: marks unit tests (no hardware/network required)
```

## CI/CD Integration

```yaml
# .github/workflows/G-Run Tests.yml
- name: Run unit tests
  run: |
    pytest src/rob_box_voice/test/ \
      --cov=src/rob_box_voice \
      --cov-report=xml \
      -m "not integration" \
      --tb=short

- name: Run MCP tools tests
  run: |
    pytest src/rob_box_mcp_tools/test/ \
      --cov=src/rob_box_mcp_tools \
      --cov-fail-under=80
```

## Best Practices Summary

1. **Write tests alongside code** — TASK-037 requires ≥80% coverage
2. **One assertion per test** when possible
3. **Use fixtures** for VoiceMemory, mock ROS nodes, temp DBs
4. **Mock external dependencies** — Ollama, ROS topics, LLM
5. **Parametrize** to cover edge cases without duplication
6. **Test error paths** — graceful degradation is critical
7. **Mark slow/integration tests** to run selectively in CI
8. **Async tests** with `pytest-asyncio` for dialogue_node, MCP calls
9. **No real hardware** in unit tests — mock everything
10. **Run in CI** on every push via GitHub Actions
