# Code Review Skill

Comprehensive code review capabilities for Rob Box project — ROS2 Python nodes,
MCP tools, Docker configurations, and agent architecture.

## When to Use This Skill

Use this skill when:
- Reviewing Python code in `src/rob_box_*/` before merge
- Auditing Docker files in `docker/main/` or `docker/vision/`
- Code reviewing MCP tools (`src/rob_box_mcp_tools/`)
- Reviewing PRs on `feature/agent` → `develop` → `main`
- Checking that TASK-035/036/037 acceptance criteria are met

## Review Approach

For each piece of code reviewed, provide:

### Summary
- Overall quality assessment (1–5)
- Key findings count by severity
- Recommended priority areas

### Critical Issues (if any)
- **Issue**: Clear description
- **Location**: File and line number  
- **Impact**: Why this matters
- **Severity**: Critical / High / Medium / Low
- **Fix**: Code example

### Findings by Category

## 1. Security Analysis

Check for:
- Hardcoded secrets, passwords, API keys in code or Dockerfiles
- `ENV API_KEY=secret` in Dockerfile → ❌ use runtime env or Docker secrets
- Unsanitized inputs passed to `subprocess`, `eval`, `exec`
- Sensitive data in logs (`self.get_logger().info(f"token={token}")`)
- No authentication on exposed ports/endpoints

**Rob Box specific:**
```python
# ❌ BAD - API key in code
DEEPSEEK_API_KEY = "sk-abc123..."

# ✅ GOOD - from environment
DEEPSEEK_API_KEY = os.environ.get("DEEPSEEK_API_KEY")
if not DEEPSEEK_API_KEY:
    self.get_logger().error("DEEPSEEK_API_KEY not set")
```

## 2. Performance Review

Check for:
- Algorithm efficiency — O(n²) loops that could be O(n)
- Blocking calls in async context (sync `requests` inside `async def`)
- Missing `await` on async operations
- Unnecessary DB queries in loops (N+1 problem)
- Memory leaks — objects accumulated without cleanup
- ThreadPoolExecutor without proper timeout/cleanup

**Rob Box specific:**
```python
# ❌ BAD - blocking call in async agent cycle
def process_llm_response(self, response):
    time.sleep(2)  # blocks event loop

# ✅ GOOD - non-blocking
async def process_llm_response(self, response):
    await asyncio.sleep(0)  # yield control
```

```python
# ❌ BAD - memory leak in agent cycle
self.history.append(turn)  # grows unbounded

# ✅ GOOD - bounded history
self.history.append(turn)
if len(self.history) > MAX_HISTORY:
    self.history = self.history[-MAX_HISTORY:]
```

## 3. Code Quality

Check for:
- SOLID principles violations
- Functions > 50 lines (split into smaller)
- Cyclomatic complexity > 10
- Missing type hints on public methods
- `print()` instead of `self.get_logger().info()`
- Magic numbers without named constants
- Inconsistent naming (snake_case for functions, PascalCase for classes)

**Rob Box Python standards:**
```python
# ❌ BAD
def process(x, data, flag=True):
    print(f"processing {x}")
    if flag:
        return data[0]

# ✅ GOOD
def process_voice_command(
    command: str,
    context: DialogueContext,
    use_memory: bool = True
) -> Optional[str]:
    """Process voice command with optional memory context.
    
    Args:
        command: Raw transcribed voice command.
        context: Current dialogue context.
        use_memory: Whether to include memory context.
    
    Returns:
        LLM response string, or None if processing failed.
    """
    self.get_logger().info(f"Processing command: {command[:50]}")
    ...
```

## 4. Docker Review

Check for (see `docs/development/DOCKER_STANDARDS.md`):
- ❌ `COPY config/` or `COPY scripts/` in Dockerfile
- ❌ Missing `network_mode: host`
- ❌ Missing `depends_on: zenoh-router`
- ❌ `latest` tag on base image
- ✅ Volumes for config: `./config:/config:ro`

```dockerfile
# ❌ BAD
COPY config/zenoh_config.json5 /config/
COPY scripts/start.sh /scripts/

# ✅ GOOD - use volumes in docker-compose.yaml
# volumes:
#   - ./config:/config:ro
#   - ./scripts:/scripts:ro
```

## 5. ROS2 Patterns

Check for:
- `print()` instead of `self.get_logger().info()`
- Missing `rclpy.shutdown()` in cleanup
- QoS profile mismatches (publisher vs subscriber)
- Blocking calls in ROS callbacks (no `async def` in ROS2 callbacks normally)
- Missing `try/except` around topic pub/sub

```python
# ❌ BAD
class MyNode(Node):
    def callback(self, msg):
        print(msg.data)  # should use logger
        result = requests.get(url)  # blocking HTTP in callback!

# ✅ GOOD
class MyNode(Node):
    def callback(self, msg: String) -> None:
        self.get_logger().info(f"Received: {msg.data}")
        self._executor.submit(self._fetch_async, msg.data)
```

## 6. Maintainability

Check for:
- Missing docstrings on public methods
- Functions > 50 lines — should be split
- Hard dependencies instead of dependency injection
- Missing error handling for external calls (LLM, DB, ROS topics)
- Test coverage — new code should have tests

```python
# ❌ BAD - hard dependency
class DialogueNode(Node):
    def __init__(self):
        self.llm = DeepSeekLLM()  # hard-coded LLM

# ✅ GOOD - injectable
class DialogueNode(Node):
    def __init__(self, llm_adapter: LLMAdapterBase):
        self.llm = llm_adapter  # testable with mock
```

## Review Template

```markdown
## Code Review: <PR/file name>

**Overall Quality**: X/5
**Findings**: N critical, N high, N medium, N low

### Critical Issues
None / [list with file:line and fix examples]

### High Priority
None / [list]

### Medium Priority  
None / [list]

### Suggestions (Low)
None / [list]

### Checklist
- [ ] No COPY config/ in Dockerfiles
- [ ] network_mode: host in all services
- [ ] No print() — only self.get_logger()
- [ ] Type hints on all public methods
- [ ] black (120) + isort (black) + flake8 pass
- [ ] Tests written for new code
```

## Quick Severity Guide

| Severity | Examples |
|----------|---------|
| **Critical** | Secrets in code, crash on None, deadlock |
| **High** | Memory leak, blocking main thread, no error handling |
| **Medium** | Missing type hints, functions > 50 lines, no docstrings |
| **Low** | Naming conventions, extract constants, minor refactor |
