"""Bundled data files for the rob_box_harness core package.

The package is intentionally a regular Python package (not a namespace
package) so that :func:`importlib.resources.files` can locate YAML
assets under ``core/data/`` consistently across Python 3.10+. The
single file currently living here is ``confirmation_policy.yaml`` —
the catalog used by :class:`rob_box_harness.core.confirmation_policy
.ToolConfirmationPolicy`.
"""