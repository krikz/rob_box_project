"""``UpperHarness`` — a dummy harness that uppercases the LLM's response.

Identical to :class:`EchoHarness` except for the post-processing
step: the assistant text is uppercased before being returned and
dispatched. Lives as a separate harness to demonstrate that
adding a new harness is a 30-line affair — the framework's
lifecycle, ports, registry, and runner do not need to change.
"""

from __future__ import annotations

from typing import Any, Mapping

from rob_box_harness.harness import Harness
from rob_box_harness.harnesses._base import run_request_response_loop


class UpperHarness(Harness[Mapping[str, Any]]):
    """A dummy harness that uppercases the LLM's response."""

    name = "upper"

    async def step(self, input_data: Any) -> str:
        """Run a single turn: ask the LLM, uppercase, echo, return.

        The uppercase post-processor is the only thing that
        distinguishes this harness from :class:`EchoHarness`; the
        shared loop is in :func:`run_request_response_loop`.
        """
        return await run_request_response_loop(
            self, input_data, post_process=str.upper
        )


__all__ = ["UpperHarness"]
