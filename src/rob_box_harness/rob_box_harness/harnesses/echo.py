"""``EchoHarness`` — the first built-in dummy harness.

Round-trips the input through the LLM and returns the LLM's
response verbatim. The harness delegates the actual loop to
:func:`rob_box_harness.harnesses._base.run_request_response_loop`;
its own ``step`` is a one-liner that supplies the identity
post-processor.

The harness:

  * stores the user message in the memory store,
  * calls :class:`LLMProvider.complete`,
  * if the response includes tool calls, executes them and
    feeds the results back to the LLM,
  * stores the final assistant response in the memory store,
  * dispatches an :class:`EchoEffect` so the side-effect bus
    is exercised.
"""

from __future__ import annotations

from typing import Any, Mapping

from rob_box_harness.harness import Harness
from rob_box_harness.harnesses._base import run_request_response_loop


class EchoHarness(Harness[Mapping[str, Any]]):
    """A dummy harness that echoes the LLM's response."""

    name = "echo"

    async def step(self, input_data: Any) -> str:
        """Run a single turn: ask the LLM, dispatch an EchoEffect, return."""
        return await run_request_response_loop(self, input_data)


__all__ = ["EchoHarness"]
