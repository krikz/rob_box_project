# tools/audio_capture_harness — AudioData capture/validation harness

Test-only Python harness that captures AudioData messages published by
``rob_box_voice/tts_node`` on the ``/voice/audio/speech`` topic, validates
the int16-LE mono @ 16 kHz contract, dumps the chunks into a single WAV
file, and verifies stream-level properties (duration, joints, time-to-first).

The harness lives **outside** ``rob_box_llm/`` and ``rob_box_voice/``
(it is a test tool under ``tools/``), so it does not import any of the
production nodes' own test scaffolding. It can be exercised against a
real ``rclpy`` subscriber (production path inside the project's ROS
Docker image) or against a JSON-lines pipe (the same wire format
``tts_audio_bench.scripts.real_subscriber`` produces), which is what
runs on minimal hosts that don't have ROS2 Humble installed.

## What it covers (acceptance gates)

| # | Gate                                                                          | How the harness proves it                                                                       |
|---|-------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------|
| 1 | Subscribes to ``/voice/audio/speech`` with QoS compatible with ``tts_node``.   | ``--transport ros2`` builds the same QoS profile (``best_effort`` or ``reliable`` + ``KEEP_LAST`` + configured depth) as ``tts_node``. |
| 2 | Validates the format — int16 LE mono @ 16 kHz.                                | ``validate_audio_chunk`` checks payload alignment, byte order, peak range; samples per channel and sample width come out-of-band via ``--expected-sample-rate``/``--expected-channels``/``--expected-sample-width``. |
| 3 | Dumps every received chunk to a single WAV file (in publish order).           | ``write_wav_from_chunks`` writes a contiguous WAV and round-trips the header. |
| 4 | Measures time-to-first-AudioData (TTFA) from harness spawn to first callback. | In ``--transport ros2`` mode: ``time.monotonic()`` is captured at ``tts_node`` spawn, and the first ROS callback records the elapsed wall clock into ``report.ttfa_s``. |
| 5 | After the publisher ends, validates the WAV duration and joins across chunks. | ``check_joints`` walks the concatenated int16 stream and flags any boundary discontinuity above the threshold (default 4000 int16 units); duration is checked against an optional ``--expected-duration-s`` ± ``--expected-duration-tol-s`` window. |
| 6 | Logs every subprocess it launches (stdout capture).                            | The harness appends each spawned command (``tts_node``, ``ros2 topic pub``, …) to ``report.commands`` and re-emits a shell-quoted line for each in the report. |

## Files

```
tools/audio_capture_harness/
├── __init__.py                       # package marker
├── audio_capture_harness.py          # main CLI / library (validation, WAV writer, transports)
├── acceptance.py                     # end-to-end runner: real TTSNode._publish_audio → harness
├── test_audio_capture_harness.py     # 23 unit + integration tests
└── README.md                         # this file
```

## Quick start

### 1. Unit / integration tests

```
python3 -m pytest tools/audio_capture_harness/test_audio_capture_harness.py
# 23 passed
```

These cover:

* ``validate_audio_chunk`` (int16-LE / mono / layout / peak range / alignment)
* ``check_joints`` (zero jumps on a DC stream, sine in range, big click detected, short streams OK)
* ``write_wav_from_chunks`` (header round-trip, blank chunks skipped, empty rejects)
* the ``--transport stdin`` CLI end-to-end (continuous streams pass, joints click fails, big-endian rejected, duration window enforced, ``ttfa_s`` recorded from ``publish_t_s``, heartbeats skipped)
* the ``--transport ros2`` CLI refusing to import when ``rclpy`` is missing

### 2. Acceptance run (real TTSNode → WAV → JSON)

The acceptance runner drives the **production** ``TTSNode._publish_audio``
(code path used by ``tts_node`` at runtime) and pipes the resulting
``AudioData`` frames through the harness:

```
PYTHONPATH=. python3 tools/audio_capture_harness/acceptance.py
# [acceptance] harness rc=0
# [acceptance] duration_s: 1.0000
# [acceptance] ttfa_s:     0.1
# [acceptance] joints_ok:  True
# [acceptance] header_ok:  True
# [acceptance] ok:         True
# [acceptance] wav: sr=16000 ch=1 sw=2 frames=16000
```

Artefacts are written to ``/tmp/acceptance_capture.wav`` and
``/tmp/acceptance_capture.json`` (configurable via ``--wav-out`` /
``--report-out``).

### 3. ROS2 mode inside the project's Docker image

On a host with ROS2 Humble + ``audio_common_msgs`` installed (the
project's own Docker image), ``--transport ros2`` becomes the
canonical mode:

```
python3 -m tools.audio_capture_harness.audio_capture_harness \
    --transport ros2 \
    --tts-node-cmd 'ros2 run rob_box_voice tts_node --ros-args \
        -p provider:=minimax \
        -p minimax_api_key:=*** -p minimax_group_id:=*** \
        -p minimax_base_url:="http://127.0.0.1:18080"' \
    --publish-topic /voice/tts/request \
    --publish-payload '{"ssml": "<speak>привет</speak>"}' \
    --minimax-base-url-env MINIMAX_BASE_URL \
    --expected-sample-rate 16000 \
    --expected-duration-s 1.0 --expected-duration-tol-s 0.5 \
    --wav-out /tmp/cap.wav --report-out /tmp/cap.json
```

The harness spawns ``tts_node`` as a subprocess, captures its stdout,
starts an ``rclpy`` subscriber, prints the synthesised text onto
``/voice/tts/request`` if ``--publish-payload`` is given, and exits 0
when every acceptance gate (WAV header, duration, joints, no format
violations, TTFA recorded) passes.

### 4. Stdin mode (no ROS2)

The same harness accepts frames over stdin, so the bench's
``real_subscriber`` (a separate Python process that consumes AudioData
over a stdio pipe) can drive it directly:

```
# A frame description is a base64'd int16 LE payload plus the out-of-band
# AudioInfo fields the production AudioData doesn't carry:
echo '{"frame_index":0,"publish_t_s":0.1,"sample_rate":16000,"channels":1,"sample_width":2,"layout":"little_endian","data_b64":"<base64>"}' \
| python3 -m tools.audio_capture_harness.audio_capture_harness \
    --transport stdin --wav-out /tmp/cap.wav --report-out /tmp/cap.json
```

## CLI reference

```
python3 -m tools.audio_capture_harness.audio_capture_harness --help
```

Highlights:

* ``--transport {ros2,stdin}`` — pick the frame transport (default ``stdin``).
* ``--qos-reliability {best_effort,reliable}`` — mirror of ``tts_node``'s
  ``audio_qos_reliability`` parameter (default ``best_effort``).
* ``--qos-depth N`` — mirror of ``tts_node``'s ``audio_qos_depth``
  parameter (default 10).
* ``--expected-sample-rate Hz`` / ``--expected-channels N`` /
  ``--expected-sample-width bytes`` — the contract the harness
  asserts on every chunk (default 16000 / 1 / 2).
* ``--expected-duration-s ± --expected-duration-tol-s`` — optional
  duration-window check after the stream ends.
* ``--joint-threshold int16`` — max adjacent-sample jump allowed at
  chunk boundaries (default 4000).
* ``--tts-node-cmd '<shell>'`` — only in ``--transport ros2`` mode:
  the harness spawns this subprocess, captures its stdout, and
  records the command in the JSON report.
* ``--publish-topic`` / ``--publish-payload`` — only in ``--transport
  ros2`` mode: the harness publishes ``std_msgs/String`` on the
  topic right after the subscriber is up.
* ``--min-msgs N`` / ``--capture-max-seconds S`` — idle-quiescence:
  when ``N`` frames have arrived and no new frame has been seen for
  ``S`` seconds, the harness stops.
* ``--deadline-s N`` — hard wall-clock deadline.

## JSON report shape

```jsonc
{
  "ok": true,
  "transport": "stdin",
  "wav_path": "/tmp/cap.wav",
  "frames": 1,
  "valid_chunks": 1,
  "bytes_total": 32000,
  "samples": 16000,
  "sample_rate": 16000,
  "channels": 1,
  "sample_width": 2,
  "ttfa_s": 0.1,
  "duration_s": 1.0,
  "duration_ok": true,
  "expected_duration_min_s": 0.95,
  "expected_duration_max_s": 1.05,
  "joints": {"ok": true, "max_jump": 771, "samples": 16000, "threshold": 4000},
  "header_ok": true,
  "header": {"ok": true, "sample_rate": 16000, "channels": 1, "sample_width": 2, "frames": 16000, "reason": null},
  "first_frame_at_s": 0.1,
  "last_frame_at_s": 0.1,
  "format_violations": [],
  "commands": []
}
```

## Why a separate test module

The project's ``tts_audio_bench/`` already covers the production
``tts_node`` audio pipeline against the ``MiniMax`` mock server, and
its ``scripts/real_subscriber.py`` is the canonical real subprocess
subscriber for ``/voice/audio/speech``. ``audio_capture_harness`` is a
**narrowly-scoped complement** focused on the capture/validation side:

* it formats the JSON report per acceptance run (TTFA, joints,
  duration, header round-trip) — ``tts_audio_bench`` reports scenarios
  in a different shape optimised for cross-validation;
* it offers the ``--transport ros2`` mode that
  ``tts_audio_bench`` deliberately does not (it runs on hosts without
  ROS2);
* it ships unit tests for the validation helpers that
  ``tts_audio_bench`` does not cover in isolation.

Both harnesses share the same wire contract (``uint8[] data`` for
``audio_common_msgs/AudioData``, int16 LE, mono, configured sample
rate), so the WAV file the bench's ``real_subscriber`` produces is
bit-equivalent to what ``audio_capture_harness`` writes when given the
same frame stream through its stdin transport.
