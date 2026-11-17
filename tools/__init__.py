"""Tooling directory: developer/test utilities (generators, architecture audit, benches).

This module exists so that submodules (e.g. ``tools.gen_tool_catalog``,
``tools.architecture_audit``) are importable as ``tools.X.Y`` Python
modules. The individual scripts under ``tools/`` also work as
top-level ``python3 tools/<script>.py`` invocations — adding this
file does not affect that path because script execution with
``__name__ == '__main__'`` is independent of the package layout.
"""
