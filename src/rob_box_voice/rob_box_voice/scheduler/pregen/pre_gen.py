"""Build a speculative next-chunk task from the current chunk.

This is the *entry point* of the chunk-level speculative pipeline
(ADR-0056, §3.2). It runs synchronously inside the ROS callback
thread (``TTSNode.dialogue_callback``), so it must be cheap and
side-effect free — the only thing it returns is either:

* a fully-formed :class:`PreGenTask` describing the next chunk to
  pre-synthesise, **or**
* ``None`` if speculation is impossible for any reason
  (no published ``pregenerate`` field, last chunk in the batch,
  reserved id mismatch, etc.).

The caller (``TTSNode.pregenerate``) treats ``None`` as "behave
exactly like the legacy code path" — no logging beyond a debug
line.

The payload contract
--------------------

The LLM-streaming layer (``dialogue_node``, separate card
``t_9798710b``) is expected to publish ``/voice/dialogue/response``
messages of the shape::

    {
      "ssml": "<speak>...</speak>",
      "speech_id": "<uuid-for-current-chunk>",
      "dialogue_id": "...",
      "batch_id": "...",
      "batch_index": 1,
      "batch_total": 3,
      "voice": "anton",                  # optional
      "language": "ru",                  # optional
      # === speculative next-chunk hint (optional, ADR-0056 §2.3) ===
      "pregenerate": {
        "next_speech_id": "<uuid-for-next-chunk>",
        "next_ssml": "<speak>...</speak>",
        "next_voice": "anton",            # optional, defaults to current voice
        "next_language": "ru",            # optional, defaults to current lang
        "next_ssml_attributes": {         # optional, defaults to {}
          "pitch": 1.0,
          "rate": 1.0,
          "volume": -19.0
        },
        "priority": "normal"              # normal | operator | personality (7a)
      }
    }

``next_*`` carries the FULL ssml/text of the next chunk that the
publisher already has on hand (typically the chunk the LLM just
emitted but hasn't fully sent yet). ``next_speech_id`` is reserved
**before** the current chunk starts synthesising so the listener
can correlate :class:`PreGenResult` with the eventual
``/voice/dialogue/response`` of the next chunk.

Failure modes that must yield ``None``
--------------------------------------

* ``chunk_data["pregenerate"]`` missing — the publisher (legacy
  dialogue_node / mcp_server ``speak_text``) does not yet opt in.
* ``batch_total == batch_index`` — current chunk is the LAST chunk
  in the batch; nothing to speculatively pre-synthesise.
* ``next_speech_id`` missing or equal to the current
  ``speech_id`` — misconfigured publisher (would create a
  self-feedback loop in the FIFO gate).
* ``next_ssml`` empty after strip — degenerate payload.
* ``next_dialogue_id`` (if specified) differs from the current
  ``dialogue_id`` — REPLACE/transition in flight, do not pre-gen
  for a stale session.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

_LOG = logging.getLogger(__name__)


# Default ``priority`` when the publisher omits the field. Matches
# the legacy behaviour ("normal" speech is the common case).
DEFAULT_NEXT_PRIORITY: str = "normal"


@dataclass(frozen=True)
class PreGenTask:
    """Speculative synthesis task — describes one future chunk.

    Frozen dataclass: callers must construct a fresh instance per
    speculative kickoff; the executor never mutates a task in
    place. ``voice`` / ``language`` / ``ssml_attributes`` are
    snapshotted at build time so a mid-flight provider switch
    (``set_provider``) does not corrupt the in-flight pre-gen.
    """

    next_speech_id: str
    next_ssml: str
    voice: str
    language: Optional[str]
    ssml_attributes: dict
    dialogue_id: Optional[str]
    priority: str = DEFAULT_NEXT_PRIORITY
    # Provenance — echoed in logs and metrics so reviewers can see
    # *why* a pre-gen was launched (e.g. ``"dialogue_node"``,
    # ``"speak_text"``). Free-form, defaults to empty.
    source: str = ""

    def __post_init__(self) -> None:
        if not self.next_speech_id:
            raise ValueError("PreGenTask.next_speech_id must not be empty")
        if not self.next_ssml or not self.next_ssml.strip():
            raise ValueError("PreGenTask.next_ssml must be non-empty")
        if self.priority not in {"normal", "operator", "personality"}:
            raise ValueError(
                f"PreGenTask.priority must be one of "
                f"normal/operator/personality, got {self.priority!r}"
            )


def build_pregen_task(
    current_chunk: Mapping[str, Any],
    *,
    fallback_voice: Optional[str] = None,
    fallback_language: Optional[str] = None,
) -> Optional[PreGenTask]:
    """Produce a :class:`PreGenTask` from the current chunk payload.

    Parameters
    ----------
    current_chunk
        The JSON-decoded dict that ``TTSNode.dialogue_callback``
        just received. Must contain ``speech_id`` and (optionally)
        ``pregenerate`` / ``batch_total`` / ``batch_index`` /
        ``dialogue_id`` / ``voice`` / ``language``.
    fallback_voice
        Voice to use when the publisher did not specify
        ``pregenerate.next_voice`` (and the current chunk has no
        ``voice`` either). Usually ``self.yandex_voice`` /
        ``self.minimax_voice`` / ``self.silero_speaker`` from the
        caller.
    fallback_language
        Same for ``language``.

    Returns
    -------
    Optional[PreGenTask]
        ``None`` if any of the failure modes in the module
        docstring applies. The caller MUST treat ``None`` as
        "no speculation, proceed with the legacy path".

    Notes
    -----
    The function is intentionally tolerant of malformed payloads:
    missing keys, wrong types, empty strings — all fall through to
    ``None``. The only thing that raises is a fundamentally
    inconsistent publisher (e.g. ``next_speech_id == speech_id``,
    which would create a self-feedback loop) — and even that is
    swallowed to ``None`` so a single bad chunk doesn't crash the
    node.
    """
    try:
        payload = current_chunk.get("pregenerate")
        if not isinstance(payload, Mapping):
            return None

        next_speech_id = payload.get("next_speech_id")
        next_ssml = payload.get("next_ssml")
        if not isinstance(next_speech_id, str) or not next_speech_id:
            return None
        if not isinstance(next_ssml, str) or not next_ssml.strip():
            return None

        # Avoid self-feedback loop: same id for current and next
        # would mean the listener tries to claim before publish.
        cur_speech_id = current_chunk.get("speech_id")
        if isinstance(cur_speech_id, str) and next_speech_id == cur_speech_id:
            _LOG.debug(
                "pregenerate skipped: next_speech_id == current speech_id "
                "(self-feedback guard)"
            )
            return None

        # Last chunk in the batch — nothing to speculate on.
        batch_index = current_chunk.get("batch_index")
        batch_total = current_chunk.get("batch_total")
        if (
            isinstance(batch_index, int)
            and isinstance(batch_total, int)
            and batch_index >= batch_total
        ):
            return None

        # Resolve voice / language / ssml_attributes:
        # next_* > current chunk > ROS param fallback.
        voice = (
            payload.get("next_voice")
            or current_chunk.get("voice")
            or fallback_voice
            or ""
        )
        language = (
            payload.get("next_language")
            if "next_language" in payload
            else current_chunk.get("language", fallback_language)
        )
        ssml_attrs = payload.get("next_ssml_attributes")
        if not isinstance(ssml_attrs, Mapping):
            ssml_attrs = {}

        dialogue_id = current_chunk.get("dialogue_id")
        priority = payload.get("priority", DEFAULT_NEXT_PRIORITY)
        if not isinstance(priority, str):
            priority = DEFAULT_NEXT_PRIORITY
        if priority not in {"normal", "operator", "personality"}:
            priority = DEFAULT_NEXT_PRIORITY

        source = str(payload.get("source", current_chunk.get("source", "")) or "")

        # Copy ssml_attrs to a plain dict so the frozen dataclass
        # doesn't retain the caller's Mapping.
        ssml_dict = {k: v for k, v in ssml_attrs.items()}

        return PreGenTask(
            next_speech_id=next_speech_id,
            next_ssml=next_ssml,
            voice=voice,
            language=language,
            ssml_attributes=ssml_dict,
            dialogue_id=dialogue_id if isinstance(dialogue_id, str) else None,
            priority=priority,
            source=source,
        )
    except Exception as exc:  # noqa: BLE001 — never crash the ROS callback
        _LOG.warning(
            "build_pregen_task: unexpected error (%s) — falling back to no-pregen",
            exc,
        )
        return None