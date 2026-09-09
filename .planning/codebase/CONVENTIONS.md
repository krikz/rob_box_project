# Coding Conventions

**Analysis Date:** 2026-05-15

## Toolchain

| Tool | Version | Purpose |
|------|---------|---------|
| **black** | 24.4.2 | Auto-formatter — opinionated, no debates |
| **flake8** | 7.0.0 | Linter — style and code quality |
| **isort** | 5.13.2 | Import sorter |
| **yamllint** | 1.35.1 | YAML syntax/style |
| **hadolint** | latest | Dockerfile linting |
| **shellcheck** | latest | Shell script linting |

Config lives in CI (`.github/workflows/G-Lint Code.yml`). No root `setup.cfg` or `pyproject.toml` — linting config is passed as CLI flags.

---

## Formatting (Black)

**Line length:** 120 characters — `black --line-length 120` (default in the project, enforced in CI).

**Quotes:** Black enforces double quotes for strings. Single quotes only when string contains double quotes.

**Trailing commas:** Black adds trailing commas in multi-line structures automatically.

**Apply:**
```bash
black .                    # format all Python
black --check --diff .     # check only (CI mode)
```

---

## Naming Conventions

**Files:**
- Python modules: `snake_case.py` (e.g., `audio_node.py`, `health_monitor.py`)
- Script executables start with `#!/usr/bin/env python3`

**Variables and functions:** `snake_case`
```python
audio_sample_rate = 16000
def process_audio_data(raw_data): ...
```

**Constants:** `SCREAMING_SNAKE_CASE`
```python
MAX_BUFFER_SIZE = 1024
DEFAULT_SAMPLE_RATE = 16000
```

**Classes:** `PascalCase`
```python
class AudioProcessor: ...
class ConversationHistory: ...
```

**ROS 2 node classes:** `<Purpose>Node(Node)` — PascalCase with `Node` suffix
```python
class AudioNode(Node): ...
class DialogueNode(Node): ...
class HealthMonitor(Node): ...   # exception: no "Node" suffix if purpose is clear
```

**ROS 2 node name string:** `snake_case` inside `super().__init__()`
```python
super().__init__('audio_node')
super().__init__('health_monitor')
```

**Private / protected:**
```python
self.public_var = 1          # public
self._protected_var = 2      # protected (internal, subclass-accessible)
self.__private_var = 3       # private (name mangling)
```

---

## Import Organization

**Order (isort profile=black):**
1. Standard library (`os`, `sys`, `typing`, `threading`, `time`, ...)
2. Third-party packages (`rclpy`, `numpy`, `torch`, `grpc`, ...)
3. Local package imports (`from rob_box_voice.audio_processor import ...`)

**Preferred style:** Absolute imports over relative.
```python
# Preferred
from rob_box_voice.audio_processor import AudioProcessor

# Allowed (sparingly)
from .audio_processor import AudioProcessor
```

**Star imports:** Forbidden (`from module import *`).

**Lazy imports:** Used when import is expensive or hardware-dependent:
```python
try:
    import pyaudio
    HAS_PYAUDIO = True
except ImportError:
    HAS_PYAUDIO = False
```

**Apply:**
```bash
isort . --profile black       # sort all imports
isort --check-only --diff .   # check only (CI mode)
```

---

## ROS 2 Node Patterns

### Node Initialization (`__init__`)

Standard `__init__` pattern from `src/rob_box_voice/rob_box_voice/audio_node.py` and `src/rob_box_perception/rob_box_perception/health_monitor.py`:

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # 1. Declare and read parameters
        self.declare_parameter('sample_rate', 16000)
        self.sample_rate = self.get_parameter('sample_rate').value

        # 2. State variables
        self.my_state = []

        # 3. QoS profiles (if needed)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # 4. Publishers
        self.pub = self.create_publisher(MsgType, '/topic', qos)

        # 5. Subscriptions
        self.sub = self.create_subscription(MsgType, '/topic', self.callback, 10)

        # 6. Timers
        self.timer = self.create_timer(5.0, self.on_timer)

        # 7. Log readiness
        self.get_logger().info('MyNode started')
```

### Entry Point

Every ROS 2 node module ends with this pattern (from `src/rob_box_voice/rob_box_voice/audio_node.py`):
```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Logging

**Use `self.get_logger()` — never `print()`.**

```python
self.get_logger().info('Node started')
self.get_logger().warning('Device not found, using fallback')
self.get_logger().error(f'Failed to process: {e}')
self.get_logger().debug(f'Received chunk: {len(data)} bytes')
```

Emoji-prefixed log messages are common and encouraged for readability:
```python
self.get_logger().info('🏥 Health Monitor запущен')
self.get_logger().info('✓ AudioNode остановлен')
```

### Parameters

Always declare parameters with defaults before reading:
```python
self.declare_parameter('device_index', -1)   # -1 = auto-detect
self.device_index = self.get_parameter('device_index').value
```

### QoS Profiles

Use explicit QoS for audio/sensor topics:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

audio_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)
```

---

## Error Handling

**Standard pattern:** `except Exception as e:` + `self.get_logger().error()`:
```python
try:
    result = self._do_something()
except Exception as e:
    self.get_logger().error(f'Failed: {e}')
    return None
```

**Bare `except:`** is used in some audio device code (`src/rob_box_voice/rob_box_voice/audio_node.py` lines 325–337) — avoid this in new code.

**Hardware fallback pattern** (from `src/rob_box_voice/rob_box_voice/tts_node.py`):
```python
try:
    from yandex.cloud.ai.tts.v3 import tts_pb2
    YANDEX_GRPC_AVAILABLE = True
except ImportError:
    YANDEX_GRPC_AVAILABLE = False
```

**noqa annotations** used sparingly:
- `# noqa: E722` — bare except in legacy audio code
- `# noqa: F811` — redefinition of function (tool definition pattern in dialogue_node)
- `# type: ignore[assignment,misc]` — for optional feature modules loaded dynamically

---

## Documentation (Docstrings)

**Style:** Google-style docstrings (from `docs/development/PYTHON_STYLE_GUIDE.md`).

**Module docstring** (at file top, after shebang):
```python
#!/usr/bin/env python3
"""
audio_node.py - AudioNode for ReSpeaker Mic Array capture.

Publishes: /audio/audio, /audio/vad, /audio/direction
"""
```

**Class docstring:**
```python
class AudioNode(Node):
    """
    ROS2 node for audio capture and processing.

    Attributes:
        sample_rate (int): Audio sample rate in Hz
        channels (int): Number of audio channels
    """
```

**Function docstring (public API):**
```python
def process_audio(raw_data: bytes, sample_rate: int = 16000) -> np.ndarray:
    """
    Convert raw audio bytes to normalized float32 array.

    Args:
        raw_data: Raw PCM audio data as bytes (int16 format)
        sample_rate: Audio sample rate in Hz

    Returns:
        Normalized audio data as float32 numpy array

    Raises:
        ValueError: If raw_data is empty
    """
```

**Comments:** Mix of Russian and English. Russian is dominant in inline comments and docstrings — acceptable and expected throughout the codebase.

---

## Type Hints

Used for public method signatures and instance variable annotations. `Optional` from `typing` is the most common annotation.

```python
from typing import Optional, Dict, List

self.model: Optional[Model] = None
self.stream: Optional[pyaudio.Stream] = None

def _recognize_yandex(self, audio_bytes: bytes) -> Optional[str]: ...
def read_parameter(self, param_name: str) -> Optional[int]: ...
```

**`@dataclass`** used in some modules (e.g., `src/rob_box_voice/rob_box_voice/llm/provider_manager.py` — `ProviderConfig` dataclass).

Type hints are **required for public API** (methods called from outside the class). Internal helpers may omit them.

---

## Module Structure

**Typical ROS 2 package layout:**
```
src/rob_box_<name>/
├── rob_box_<name>/       # Python package (importable)
│   ├── __init__.py
│   ├── <main>_node.py    # ROS 2 node class + main()
│   ├── core/             # Business logic (no ROS dependencies)
│   ├── utils/            # Shared helpers
│   └── skills/           # Optional skill modules
├── test/                 # Tests
├── scripts/              # Non-ROS executables / tools
├── setup.cfg             # colcon install config
├── package.xml           # ROS 2 manifest
└── pytest.ini            # pytest config (in rob_box_voice)
```

**`core/` subdirectory** (pattern in `rob_box_voice`, `rob_box_mcp_tools`): Pure Python business logic with **no ROS 2 dependencies** — makes unit testing without ROS possible.

---

## Linting Configuration (CI)

From `.github/workflows/G-Lint Code.yml`:

```bash
# Black check
black --check --diff --color src/rob_box_voice/ src/rob_box_animations/ ...

# Flake8
flake8 src/rob_box_voice/ ... \
    --max-line-length=120 \
    --extend-ignore=E203,W503,E501 \
    --exclude=venv,build,install,log,.git

# isort
isort --check-only --diff --color ... --profile black
```

**`continue-on-error: true`** in CI — lint failures do NOT block Docker builds. Violations are reported as warnings.

**Triggers:** Push/PR to `develop` or `main` when `.py`, `.yaml`, `.yml`, or `Dockerfile` files change.

---

## Anti-Patterns

### Bare `except:` (without exception type)
**What:** `except:` in `src/rob_box_voice/rob_box_voice/audio_node.py` lines 325–337.
**Why wrong:** Swallows `KeyboardInterrupt`, `SystemExit`, and all other exceptions.
**Do this instead:** `except Exception as e:` with `self.get_logger().error(f'...: {e}')`.

### `print()` in node code
**What:** Using `print(...)` instead of ROS logger.
**Why wrong:** Output bypasses ROS log system, not captured by `ros2 topic echo /rosout`.
**Do this instead:** `self.get_logger().info(...)`.

### Star imports
**What:** `from module import *`
**Why wrong:** Pollutes namespace, breaks isort, makes dependencies opaque.
**Do this instead:** Explicit imports.

---

*Convention analysis: 2026-05-15*
