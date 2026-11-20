"""
Core utilities for the perception package.

After W10 the perception package is LLM-free, so this layer only
exposes pure-Python helpers (event detection, robot speech markers).
``PromptFormatter`` was deleted in W10 along with the summarization
pipeline that used it; ``MemoryManager`` was deleted as a dead extract
with zero runtime importers (issue #3869). This ``__init__`` is no
longer published.
"""
