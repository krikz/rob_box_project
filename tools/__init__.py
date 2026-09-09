"""Tooling directory: developer/test utilities (mock servers, capture harnesses, etc.).

This module exists so that submodules (e.g. ``tools.audio_capture_harness``,
``tools.mock_minimax_server``) are importable as ``tools.X.Y`` Python
modules. The individual scripts under ``tools/`` also work as
top-level ``python3 tools/<script>.py`` invocations — adding this
file does not affect that path because script execution with
``__name__ == '__main__'`` is independent of the package layout.
"""
