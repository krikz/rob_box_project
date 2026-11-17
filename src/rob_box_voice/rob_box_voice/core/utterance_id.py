"""utterance_id.py — deterministic id for one speech utterance (issue #2829).

``stt_node`` and ``speaker_id_node`` both receive the exact same
``/audio/speech_audio`` (``AudioData``, raw PCM bytes, no header) message
from ``audio_node`` -- one message per accepted utterance (see
``audio_node.py`` around the ``speech_audio_pub.publish`` call: the whole
buffered utterance is published as a single ``AudioData``, never split or
re-chunked downstream before either node reads it).

ADR-0131 picked option (a): a hash of those PCM bytes, computed
independently by both nodes, over changing the message type or adding a
new topic from ``audio_node``. Both nodes call :func:`compute_utterance_id`
on the *same* ``bytes(msg.data)`` they already extract as their first line
of processing -- no wire change, no new topic, no coordination needed
between the two nodes for the id to match.

Why sha1[:12] and not the full digest: 12 hex chars (48 bits) makes
collisions between two *different* utterances desk-check negligible for a
single robot's session lifetime, while keeping log lines and JSON payloads
short. Do not shorten further -- see the birthday-bound note in the ADR.
"""

from __future__ import annotations

import hashlib

_DIGEST_LEN = 12


def compute_utterance_id(pcm_bytes: bytes) -> str:
    """Return the deterministic id for one utterance's raw PCM bytes.

    Empty input is a valid (if degenerate) utterance and still gets a
    stable id -- callers should not special-case it away, so this
    function does not raise for empty ``pcm_bytes``.
    """
    return hashlib.sha1(bytes(pcm_bytes)).hexdigest()[:_DIGEST_LEN]
