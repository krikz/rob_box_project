"""Bundled data files for the rob_box_mcp_tools package.

The package is a regular Python package (not a namespace package)
so that :func:`importlib.resources.files` can locate YAML assets
under ``rob_box_mcp_tools.data`` consistently across Python 3.10+.

The single file currently living here is ``slice_policy.yaml`` — the
sender→slice→tool allowlist consumed by
:class:`rob_box_mcp_tools.slice_authority.ToolSliceAuthority` (see
ADR-0052, issue #1998 §6.2).
"""
