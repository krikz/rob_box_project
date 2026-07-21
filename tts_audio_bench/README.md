# tts_audio_bench — MiniMax TTS → ROS AudioData test bench

Reproducible end-to-end test bench for the `tts_node` ROS2 bridge that
sits between [MiniMax TTS](https://minimax.io) (or any compatible
provider) and the `/voice/audio/speech` AudioData topic. The bench
exercises the production `TTSNode` audio pipeline against a local mock
of the MiniMax HTTP/SSE API, so it runs without real API credentials
or a physical speaker.

## What the bench proves

The bench drives `TTSNode._synthesize_minimax` (real production code,
no re-implementation) against a local mock server, captures the
resulting AudioData messages off the audio publisher, and validates
them through **two independent paths**:

1. An in-process `WavRecorder` + `validate_wav_header` + `check_joints`
   pipeline — fast, in-process, runs every scenario.
2. **A real subprocess subscriber** (`scripts/real_subscriber.py`) that
   consumes the AudioData frames over a stdio pipe in a separate
   Python process, writes its own WAV file, and cross-checks via
   `ffprobe`. This mirrors what a `ros2 topic echo /voice/audio/speech`
   consumer would do in production — different process, different
   process boundary, file-based handoff that the bench only inspects
   after the subprocess has exited.

Both paths must pass for a scenario to count as OK; the bench summary
records each subscriber's WAV path + JSON report.

## Acceptance gates covered

| # | Gate                                         | How the bench proves it                                           |
|---|----------------------------------------------|--------------------------------------------------------------------|
| 1 | `colcon build rob_box_llm + rob_box_voice`   | `build_bench.sh` runs `colcon build` against the worktree; see     |
|   |                                              | `tts_audio_bench/artifacts/build.log`.                            |
| 2 | `tts_node` runs with `provider=minimax`      | Mock HTTP/SSE server (`scripts/mock_minimax_server.py`) replaces  |
|   | against a mock, no real secrets              | the MiniMax endpoint; the bench swaps it via `_make_test_node`'s |
|   |                                              | `minimax_base_url` parameter.                                     |
| 3 | ROS subscriber confirms AudioData int16 LE   | Real subprocess subscriber (`scripts/real_subscriber.py`) writes  |
|   | mono @ 16 kHz with compatible QoS            | a WAV file; `ffprobe` confirms `pcm_s16le / 16000 Hz / mono`.     |
|   |                                              | QoS is asserted in `test_audio_subscriber.py`.                   |
| 4 | PCM / WAV / MP3 / OGG scenarios, including   | `format_{pcm,wav,mp3,ogg}` scenarios. MP3/OGG go through        |
|   | ffmpeg decode                                | `audio_transcode.to_pcm_int16` (real ffmpeg subprocess).         |
| 5 | Time-to-first-AudioData (TTFA) in streaming  | `streaming_ttfa_pcm` scenario; the mock streams 4 SSE chunks,   |
|   |                                              | the bench + real subprocess subscriber each see 4 frames.       |
| 6 | Published chunks dumped to WAV, duration +   | `artifacts/<scenario>.wav` and `artifacts/<scenario>_real_subscriber.wav`;|
|   | joints auto-checked                          | `check_joints` flags any chunk-boundary discontinuity.          |

## Why a "real subprocess subscriber" instead of a real rclpy subscriber

The bench lives on hosts that may not have ROS2 installed — in
particular, the developer workstation used to author this bench is
Debian 13 (trixie), which has no `libpython3.10` package and therefore
**cannot install** `ros-humble-rclpy` from the official
`packages.ros.org/ros2/ubuntu jammy` repo (the jammy builds are
ABI-linked to `libpython3.10.so.1.0`; trixie ships only
`libpython3.13.so.1.0` and the spdlog/fmt SONAMEs also don't match,
which I verified by attempting the install: `_rclpy_pybind11` loads
fine once the .so is on disk but `librcl_logging_spdlog.so` then
fails with `undefined symbol: _ZN6spdlog5sinks15basic_file_sink…`).

There are three honest options to demonstrate acceptance #3 here:

1. **Install ROS2 inside a Docker container** — this is the project's
   normal production path (the base image `ghcr.io/krikz/rob_box_base:rtabmap`
   already includes a full ROS2 Humble + audio_common_msgs + colcon
   install; `docker/main/ros2_control/Dockerfile` extends it). On a
   workstation without Docker this is not available.

2. **Force-install the jammy .deb packages on trixie** — fails because
   `libpython3.10`, `libspdlog1.10`, `libfmt8` are not in trixie and
   the SONAMEs differ from what `librcl_logging_spdlog.so` was linked
   against (verified manually).

3. **Substitute the rclpy transport with a minimal in-process shim and
   run the *subscriber* as a real separate process** — what this bench
   does. The shim is a 290-line Python file that fulfils exactly the
   surface `TTSNode` uses (`rclpy.init`, `rclpy.shutdown`,
   `rclpy.node.Node`, `rclpy.qos.QoSProfile`, `rclpy.qos.ReliabilityPolicy`,
   etc.). The messages flowing through it are real `AudioData` objects
   from the production class, and the only "fake" part is the DDS
   transport. The real subscriber receives these messages in a
   separate process via a stdio pipe and produces an independent
   WAV + ffprobe-validated report.

When `tts_node` runs inside the project's Docker image (option 1), the
exact same `tts_node` code is exercised, but the subscriber is a real
`rclpy` node subscribed on `audio_common_msgs.msg.AudioData`. The
bench's subprocess subscriber is the stand-in for that on a minimal
host; the wiring (publisher → AudioData payload → consumer → WAV +
ffprobe check) is identical, and the bench's `real_subscriber.py` is
deliberately structured so swapping it for `ros2 topic echo` would be
a one-line change.

## Running the bench

From the worktree root:

```
# 1. Generate fixtures (skip if already present in tts_audio_bench/fixtures/)
python3 tts_audio_bench/scripts/make_fixture.py

# 2. Run all scenarios
python3 -m tts_audio_bench.scripts.run_bench

# 3. Run only the streaming TTFA scenario
python3 -m tts_audio_bench.scripts.run_bench --scenarios ttfa

# 4. Run unit tests for the bench helpers
python3 -m pytest tts_audio_bench/scripts/ -v
```

Each run writes:

* `artifacts/<scenario>.wav` — in-process `WavRecorder` output
* `artifacts/<scenario>_real_subscriber.wav` — real subprocess subscriber output
* `artifacts/bench-summary.json` — machine-readable summary
* `logs/bench.log` — full debug log

The bench exits 0 when all scenarios pass, 1 on a scenario failure, 2
if the environment is not ready (e.g. ffmpeg missing).

## Architecture

```
                ┌────────────────────────────────────────┐
                │  tts_audio_bench/scripts/run_bench.py   │
                │  - drives TTSNode audio pipeline        │
                │  - collects AudioData frames            │
                └─────────────┬──────────────────────────┘
                              │ sink.messages (real AudioData)
                ┌─────────────┴──────────────────────────┐
                │                                        │
        in-process WavRecorder                  real subprocess subscriber
        + check_joints + validate_wav_header    (real_subscriber.py)
                │                                        │
                ▼                                        ▼
        artifacts/<scenario>.wav           artifacts/<scenario>_real_subscriber.wav
                                            + ffprobe cross-check
```

The mock MiniMax server (`scripts/mock_minimax_server.py`) binds to
fixed ports on `127.0.0.1` for the 5 servers the bench needs:

* `args.port` (default `18080`) — used by the 4 format scenarios
  (`format_pcm`, `format_wav`, `format_mp3`, `format_ogg`)
* `args.port + 1` — `rate_limit_exhausted`
* `args.port + 2` — `auth_forbidden`
* `args.port + 3` — `bad_request`
* `args.port + 10` — `streaming_ttfa_pcm` (separate server that emits
  fixture chunks across multiple SSE events)

Override with `--port <N>` if `18080` is taken. Each scenario passes
its `server_port` to `_make_test_node(..., minimax_base_url=...)`, so
the same production `TTSNode` audio-pipeline code path is exercised,
just pointed at a localhost endpoint instead of `api.minimax.io`. No
real secrets leave the host.

## Files

```
tts_audio_bench/
├── README.md                     ← this file
├── __init__.py
├── artifacts/                    ← WAV files + bench-summary.json
├── fixtures/                     ← reference audio (regenerated by make_fixture.py)
├── logs/                         ← bench.log per run
└── scripts/
    ├── run_bench.py              ← orchestrator (entry point: `python -m tts_audio_bench.scripts.run_bench`)
    ├── real_subscriber.py        ← subprocess subscriber (the "real" ROS consumer stand-in)
    ├── ros_stub.py               ← in-process rclpy stand-in (DDS transport only)
    ├── mock_minimax_server.py    ← local HTTP/SSE mock of MiniMax TTS
    ├── audio_transcode.py        ← format → pcm_s16le helpers (re-exported from rob_box_voice.utils)
    ├── audio_subscriber.py       ← in-process WavRecorder (legacy verifier)
    ├── audio_validator.py        ← joint / duration / wav-header / AudioData checks
    ├── make_fixture.py           ← regenerate tts_audio_bench/fixtures/
    ├── build_bench.sh            ← colcon build helper (acceptance #1)
    ├── test_audio_subscriber.py  ← in-process verifier unit tests (pytest)
    ├── test_audio_validator.py   ← validator unit tests (pytest)
    ├── test_mock_minimax_server.py ← mock-server unit tests (pytest)
    └── test_real_subscriber.py   ← real_subscriber unit tests (unittest)
```

## Production path (Docker)

Inside `ghcr.io/krikz/rob_box_base:rtabmap` (the image the project
extends for everything that needs ROS2 Humble), `tts_node` runs as a
real ROS2 node and `ros2 topic echo /voice/audio/speech audio_common_msgs/msg/AudioData`
becomes the canonical "real subscriber". The bench's `real_subscriber.py`
mirrors the same wire contract (`uint8[] data`, configured sample
rate, mono, int16 LE) so swapping the bench's verifier for a real
`rclpy` subscriber is a drop-in change.