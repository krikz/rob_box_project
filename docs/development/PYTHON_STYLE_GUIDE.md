# 📐 Python Coding Style Guide

**Purpose:** Единый стиль кодирования для rob_box_project на Python.

**Why follow style guide:**
- 🤝 Легче читать чужой код
- 🔍 Проще находить баги
- ⚡ Быстрее code review
- 🤖 Автоматизация через linters

**TL;DR:** Используй `black` для форматирования, `flake8` для проверки, `isort` для imports.

---

## 📋 Table of Contents

- [Quick Reference](#quick-reference)
- [Formatting (Black)](#formatting-black)
- [Naming Conventions](#naming-conventions)
- [Imports](#imports)
- [Documentation](#documentation)
- [Type Hints](#type-hints)
- [ROS2 Specific](#ros2-specific)
- [Common Patterns](#common-patterns)
- [Anti-Patterns](#anti-patterns)
- [Tools Setup](#tools-setup)

---

## ⚡ Quick Reference

### Code formatting

```bash
# Format all Python files
black .

# Check without modifying
black --check .

# Format specific file
black src/rob_box_voice/audio_node.py
```

### Import sorting

```bash
# Sort all imports
isort .

# Check without modifying
isort --check-only .
```

### Linting

```bash
# Check code quality
flake8 src/rob_box_voice/

# With detailed output
flake8 --statistics --show-source src/rob_box_voice/
```

---

## 🎨 Formatting (Black)

**We use Black** - автоматический code formatter. Споры о форматировании = закончены.

### Line Length

```python
# Maximum 120 characters per line
# Black автоматически разбивает длинные строки

# Before Black
very_long_function_call(argument1, argument2, argument3, argument4, argument5, argument6, argument7, argument8)

# After Black
very_long_function_call(
    argument1,
    argument2,
    argument3,
    argument4,
    argument5,
    argument6,
    argument7,
    argument8,
)
```

### Indentation

```python
# 4 spaces (Black handles this automatically)
def my_function():
    if condition:
        do_something()
        do_another_thing()
```

### Quotes

```python
# Black prefers double quotes for strings
message = "Hello, world!"  # ✅ Good
message = 'Hello, world!'  # ⚠️ Black will change to double quotes

# EXCEPT: Use single quotes when string contains double quotes
message = 'He said "Hello"'  # ✅ Good
```

### Trailing Commas

```python
# Black adds trailing commas for multi-line structures
my_list = [
    "item1",
    "item2",
    "item3",  # ✅ Trailing comma (Black adds this)
]

my_dict = {
    "key1": "value1",
    "key2": "value2",  # ✅ Trailing comma
}
```

---

## 🏷️ Naming Conventions

### Variables & Functions

```python
# snake_case for variables and functions
audio_sample_rate = 16000  # ✅ Good
audioSampleRate = 16000    # ❌ Bad (camelCase)
AUDIO_SAMPLE_RATE = 16000  # ❌ Bad (reserved for constants)

def process_audio_data(raw_data):  # ✅ Good
    pass

def ProcessAudioData(raw_data):  # ❌ Bad (PascalCase reserved for classes)
    pass
```

### Constants

```python
# SCREAMING_SNAKE_CASE for constants
MAX_BUFFER_SIZE = 1024      # ✅ Good
DEFAULT_SAMPLE_RATE = 16000  # ✅ Good
max_buffer_size = 1024      # ❌ Bad
```

### Classes

```python
# PascalCase for classes
class AudioProcessor:  # ✅ Good
    pass

class audio_processor:  # ❌ Bad
    pass

class Audio_Processor:  # ❌ Bad
    pass
```

### ROS2 Nodes

```python
# Node classes: <Purpose>Node (PascalCase)
class AudioNode(Node):  # ✅ Good
    pass

class DialogueNode(Node):  # ✅ Good
    pass

# Node names in ROS2: snake_case
def __init__(self):
    super().__init__('audio_node')  # ✅ Good node name
```

### Private/Protected

```python
class MyClass:
    def __init__(self):
        self.public_var = 1           # ✅ Public
        self._protected_var = 2       # ✅ Protected (hint: internal use)
        self.__private_var = 3        # ✅ Private (name mangling)
    
    def public_method(self):          # ✅ Public
        pass
    
    def _protected_method(self):      # ✅ Protected
        pass
    
    def __private_method(self):       # ✅ Private
        pass
```

---

## 📦 Imports

**We use isort** - автоматическая сортировка imports.

### Import Order

```python
# 1. Standard library
import os
import sys
from typing import List, Optional

# 2. Third-party packages
import numpy as np
import rclpy
from rclpy.node import Node

# 3. Local imports
from rob_box_voice.audio_processor import AudioProcessor
from rob_box_voice.utils import convert_format
```

### Import Style

```python
# ✅ Good: Explicit imports
from rob_box_voice.audio_processor import AudioProcessor, AudioConfig

# ✅ Good: Import module for many items
import rob_box_voice.utils as utils

# ❌ Bad: Star imports (unless explicitly needed)
from rob_box_voice import *

# ❌ Bad: Importing unused modules
import time  # but never use it
```

### Relative Imports

```python
# For same package imports
# ✅ Good: Absolute imports (preferred)
from rob_box_voice.audio_processor import AudioProcessor

# ⚠️ OK: Relative imports (use sparingly)
from .audio_processor import AudioProcessor
from ..utils import helper_function
```

### Conditional Imports

```python
# Put at module level if possible
try:
    import pyaudio
    HAS_PYAUDIO = True
except ImportError:
    HAS_PYAUDIO = False

# Use in function if import is expensive
def use_expensive_library():
    import expensive_ml_library  # Lazy import
    return expensive_ml_library.process()
```

---

## 📝 Documentation

### Module Docstrings

```python
#!/usr/bin/env python3
"""
Audio processing module for rob_box voice assistant.

This module handles audio capture, processing, and publishing to ROS2 topics.
Uses ReSpeaker USB microphone array for multi-channel audio input.

Example:
    >>> from rob_box_voice.audio_node import AudioNode
    >>> node = AudioNode()
    >>> rclpy.spin(node)
"""

import rclpy
from rclpy.node import Node
```

### Class Docstrings

```python
class AudioNode(Node):
    """
    ROS2 node for audio capture and processing.
    
    Captures audio from ReSpeaker USB device, processes raw PCM data,
    and publishes to /audio/pcm topic for downstream STT processing.
    
    Attributes:
        sample_rate (int): Audio sample rate in Hz (default: 16000)
        channels (int): Number of audio channels (default: 6 for ReSpeaker)
        chunk_size (int): Buffer size in frames (default: 1024)
        audio_stream: PyAudio stream object
    
    Example:
        >>> node = AudioNode()
        >>> rclpy.spin(node)
    """
    
    def __init__(self):
        super().__init__('audio_node')
        self.sample_rate = 16000
```

### Function Docstrings

```python
def process_audio(raw_data: bytes, sample_rate: int = 16000) -> np.ndarray:
    """
    Convert raw audio bytes to numpy array and normalize.
    
    Takes raw PCM bytes from audio device, converts to float32 array,
    and normalizes values to [-1.0, 1.0] range.
    
    Args:
        raw_data: Raw PCM audio data as bytes (int16 format)
        sample_rate: Audio sample rate in Hz (default: 16000)
    
    Returns:
        Normalized audio data as float32 numpy array
    
    Raises:
        ValueError: If raw_data is empty or invalid format
        
    Example:
        >>> raw = b'\\x00\\x01\\x00\\x02'
        >>> processed = process_audio(raw)
        >>> print(processed.dtype)
        float32
    """
    if not raw_data:
        raise ValueError("raw_data cannot be empty")
    
    # Convert bytes to int16 array
    audio_array = np.frombuffer(raw_data, dtype=np.int16)
    
    # Normalize to [-1.0, 1.0]
    normalized = audio_array.astype(np.float32) / 32768.0
    
    return normalized
```

### Inline Comments

```python
# ✅ Good: Explain WHY, not WHAT
# Use STFT for frequency domain analysis (better noise reduction)
spectrogram = librosa.stft(audio_data)

# ❌ Bad: Stating the obvious
# Convert audio to spectrogram
spectrogram = librosa.stft(audio_data)

# ✅ Good: Complex logic explanation
# ReSpeaker has 6 channels: [0-3]=mics, [4]=playback, [5]=loopback
# We only need mic channels for voice capture
mic_channels = audio_data[:, 0:4]

# ✅ Good: TODO/FIXME/HACK markers
# TODO(username): Add support for 8-channel devices
# FIXME: Memory leak when stream is not properly closed
# HACK: Workaround for PyAudio bug #123
```

---

## 🔤 Type Hints

**Use type hints** - помогают IDE и документируют код.

### Basic Types

```python
from typing import List, Dict, Optional, Tuple, Union

def process_samples(
    samples: List[int],
    config: Dict[str, str],
    timeout: Optional[float] = None
) -> Tuple[np.ndarray, int]:
    """Process audio samples with configuration."""
    pass

# Variables can also have type hints
sample_rate: int = 16000
audio_data: Optional[np.ndarray] = None
```

### ROS2 Types

```python
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image

class MyNode(Node):
    def __init__(self) -> None:
        super().__init__('my_node')
        
        # Type hint for publisher
        self.publisher: rclpy.publisher.Publisher = self.create_publisher(
            String,
            '/topic_name',
            10
        )
    
    def callback(self, msg: String) -> None:
        """Process incoming message."""
        self.get_logger().info(f'Received: {msg.data}')
```

### Complex Types

```python
from typing import Callable, Any, TypeVar

# Type variable
T = TypeVar('T')

def map_transform(
    data: List[T],
    transform: Callable[[T], T]
) -> List[T]:
    """Apply transform to each element."""
    return [transform(item) for item in data]

# Union types
def process_input(data: Union[str, bytes, np.ndarray]) -> np.ndarray:
    """Accept multiple input types."""
    if isinstance(data, str):
        return np.array([ord(c) for c in data])
    elif isinstance(data, bytes):
        return np.frombuffer(data, dtype=np.uint8)
    else:
        return data
```

---

## 🤖 ROS2 Specific

### Node Structure

```python
#!/usr/bin/env python3
"""Audio processing node."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AudioNode(Node):
    """Audio capture and processing node."""
    
    def __init__(self) -> None:
        """Initialize audio node."""
        super().__init__('audio_node')
        
        # Declare parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('device_index', 0)
        
        # Get parameters
        self.sample_rate = self.get_parameter('sample_rate').value
        self.device_index = self.get_parameter('device_index').value
        
        # Create publisher
        self.audio_pub = self.create_publisher(
            String,
            '/audio/pcm',
            10
        )
        
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Audio node initialized')
    
    def timer_callback(self) -> None:
        """Timer callback to publish audio."""
        msg = String()
        msg.data = 'audio_data'
        self.audio_pub.publish(msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = AudioNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Publisher/Subscriber

```python
# ✅ Good: Type hints and clear names
self.audio_publisher: rclpy.publisher.Publisher = self.create_publisher(
    Audio,  # Message type
    '/audio/pcm',  # Topic name
    10  # QoS depth
)

# ✅ Good: Subscriber with typed callback
self.text_subscriber = self.create_subscription(
    String,
    '/dialogue/text',
    self.text_callback,
    10
)

def text_callback(self, msg: String) -> None:
    """Handle incoming text messages."""
    self.get_logger().info(f'Received: {msg.data}')
```

### Logging

```python
# ✅ Good: Use ROS2 logger
self.get_logger().debug('Detailed debug info')
self.get_logger().info('Normal operation')
self.get_logger().warn('Warning message')
self.get_logger().error('Error occurred')
self.get_logger().fatal('Critical error')

# ❌ Bad: Using print()
print('This is a message')  # Don't do this in ROS nodes

# ✅ Good: Format strings
self.get_logger().info(f'Sample rate: {self.sample_rate} Hz')

# ✅ Good: Lazy evaluation for expensive operations
if self.get_logger().get_effective_level() <= 10:  # DEBUG level
    expensive_debug_info = compute_expensive_debug_info()
    self.get_logger().debug(f'Debug: {expensive_debug_info}')
```

---

## 🎯 Common Patterns

### Error Handling

```python
# ✅ Good: Specific exceptions
try:
    audio_device = open_audio_device(device_index)
except IOError as e:
    self.get_logger().error(f'Failed to open audio device: {e}')
    raise RuntimeError(f'Audio device {device_index} not available') from e

# ✅ Good: Context managers for cleanup
with open('/tmp/audio.wav', 'wb') as f:
    f.write(audio_data)

# ✅ Good: Finally for cleanup
stream = None
try:
    stream = audio.open()
    process_audio(stream)
finally:
    if stream:
        stream.close()
```

### Configuration

```python
# ✅ Good: Use ROS2 parameters
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare with defaults
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('timeout', 5.0)
        
        # Get values
        self.sample_rate = self.get_parameter('sample_rate').value
        
        # Parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameters_callback)
    
    def parameters_callback(self, params):
        """Handle parameter changes."""
        for param in params:
            if param.name == 'sample_rate':
                self.sample_rate = param.value
                self.get_logger().info(f'Sample rate updated: {param.value}')
        return SetParametersResult(successful=True)
```

### Resource Management

```python
# ✅ Good: Cleanup in destroy
class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_node')
        self.stream = self.audio.open()
    
    def destroy_node(self):
        """Clean up resources before shutdown."""
        if hasattr(self, 'stream') and self.stream:
            self.stream.stop_stream()
            self.stream.close()
        super().destroy_node()
```

---

## ⚠️ Anti-Patterns

### Don't Do These

```python
# ❌ Bad: Global variables
audio_data = []  # Global mutable state

def process():
    global audio_data
    audio_data.append(new_data)

# ✅ Good: Use class attributes or pass as arguments
class AudioProcessor:
    def __init__(self):
        self.audio_data = []
    
    def process(self, new_data):
        self.audio_data.append(new_data)
```

```python
# ❌ Bad: Bare except
try:
    process_audio()
except:  # Catches KeyboardInterrupt, SystemExit!
    pass

# ✅ Good: Specific exceptions
try:
    process_audio()
except (IOError, ValueError) as e:
    logger.error(f'Processing failed: {e}')
```

```python
# ❌ Bad: Mutable default arguments
def add_sample(sample, buffer=[]):  # DANGER!
    buffer.append(sample)
    return buffer

# ✅ Good: Use None and create inside function
def add_sample(sample, buffer=None):
    if buffer is None:
        buffer = []
    buffer.append(sample)
    return buffer
```

```python
# ❌ Bad: String concatenation in loops
result = ""
for item in items:
    result += str(item) + ", "

# ✅ Good: Join or list comprehension
result = ", ".join(str(item) for item in items)
```

---

## 🛠️ Tools Setup

### Install tools

```bash
pip install black flake8 isort mypy
```

### Run tools

```bash
# Format code
black src/rob_box_voice/

# Sort imports
isort src/rob_box_voice/

# Check style
flake8 src/rob_box_voice/

# Type checking
mypy src/rob_box_voice/
```

### Pre-commit hook

Already configured in `.pre-commit-config.yaml`:

```bash
# Install pre-commit
pip install pre-commit
pre-commit install

# Now black, isort, flake8 run automatically on git commit
```

### IDE Integration

**VS Code (settings.json):**
```json
{
    "python.formatting.provider": "black",
    "python.formatting.blackArgs": ["--line-length=120"],
    "python.linting.enabled": true,
    "python.linting.flake8Enabled": true,
    "python.linting.flake8Args": [
        "--max-line-length=120",
        "--extend-ignore=E203,W503"
    ],
    "editor.formatOnSave": true,
    "[python]": {
        "editor.codeActionsOnSave": {
            "source.organizeImports": true
        }
    }
}
```

**PyCharm:**
- Settings → Tools → Black → Enable
- Settings → Editor → Code Style → Python → Imports → Use isort

---

## 📚 References

- [PEP 8 – Style Guide for Python Code](https://peps.python.org/pep-0008/)
- [Black Code Style](https://black.readthedocs.io/en/stable/the_black_code_style/current_style.html)
- [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)
- [ROS 2 Python Style Guide](https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html)

---

## 🔗 Related Documentation

- [LINTING_GUIDE.md](./LINTING_GUIDE.md) - Linter setup and usage
- [TESTING_GUIDE.md](./TESTING_GUIDE.md) - Testing best practices
- [.pre-commit-config.yaml](../../.pre-commit-config.yaml) - Automated checks

---

**Last Updated:** October 16, 2025 (Phase 3, Item 1)
