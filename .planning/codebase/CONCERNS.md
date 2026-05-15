# Codebase Concerns

**Analysis Date:** 2026-05-15

---

## Tech Debt

**Monolithic `dialogue_node.py` (2040 lines):**
- Issue: Single file handles agent loop, LLM provider switching, FAQ mode, DJ mode, event mode, conversation history, MCP tool calls, barge-in, and TTS queuing. Extremely hard to test in isolation.
- Files: `src/rob_box_voice/rob_box_voice/dialogue_node.py`
- Impact: 13.4% test coverage (the lowest in the project). Changes frequently introduce regressions.
- Fix approach: Extract agent loop, conversation history, and mode logic into separate classes. `dialogue_node.py` should be ~300-line orchestrator.

**Robot status MCP tool returns hardcoded stub data:**
- Issue: `get_robot_status` always returns `position: {x:0, y:0, theta:0}` and `battery: 85%`. No ROS topic subscription.
- Files: `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py` (line 440)
- Impact: LLM receives false positional and battery data. Robot tells users it is at origin when it is not.
- Fix approach: Subscribe to `/odom` and `/diagnostics` in the MCP server's ROS context to provide real data.

**`reflection_node.py` falls back to stub responses:**
- Issue: When `deepseek_client` is None (API key missing or env not set), all proactive thoughts and user responses return hardcoded strings like "У меня всё отлично!".
- Files: `src/rob_box_perception/rob_box_perception/reflection_node.py` (lines 668, 702, 747)
- Impact: Silent degradation — reflection appears to work but produces meaningless output. No warning is logged at startup.
- Fix approach: Log a `WARN` at node startup when no API key is found, and publish a diagnostic status so monitoring can detect it.

**`command_node.py` navigation and vision stubs:**
- Issue: Three commands — `get_current_position`, `detect_objects`, `follow_person` — are stubbed with `# TODO` and empty `pass`.
- Files: `src/rob_box_voice/rob_box_voice/command_node.py` (lines 343, 359, 370, 394)
- Impact: Voice commands for position reporting, object detection, and following silently do nothing.
- Fix approach: Implement position via `/tf` lookup or `/odom` subscriber; object detection via `/detections` topic; following via nav2 FollowPerson behavior.

**`async_executor.py` polls with busy-wait:**
- Issue: `_wait_for_result` loops with `await asyncio.sleep(0.05)` until `request_id` appears in `results_cache`. At 20 Hz polling, long-running tools waste CPU and inflate latency.
- Files: `src/rob_box_mcp_tools/rob_box_mcp_tools/async_executor.py` (lines 348–357)
- Impact: 50 ms minimum latency per tool call even for near-instant operations. Under load (multiple concurrent tools) polling wakes up the event loop at high frequency.
- Fix approach: Replace polling with `asyncio.Event`; set the event from the result callback instead of scanning the cache.

**`setup.py` placeholder metadata in `rob_box_perception`:**
- Issue: `description='TODO: Package description'` and `license='TODO: License declaration'` remain in production package manifest.
- Files: `src/rob_box_perception/setup.py` (lines 25–26)
- Impact: Cosmetic, but signals incomplete package metadata; breaks any automated license scanning.

**LED compositor uses hardcoded configuration:**
- Issue: LED compositor is launched without parameters and uses a hardcoded internal config; the driver uses YAML. Two separate configuration paths with no single source of truth.
- Files: `src/led_matrix_driver/launch/led_matrix_system_launch.py` (lines 6, 30)
- Impact: LED configuration changes require editing two different systems. Easy to get them out of sync.

---

## Known Bugs

**BUG-12: Unbounded local messages list within a single dialogue (TASK-043, pending):**
- Symptoms: Context grows without bound across many tool calls within one dialogue turn. After 15+ iterations in agent loop, token count balloons, causing timeouts.
- Files: `src/rob_box_voice/rob_box_voice/dialogue_node.py` (lines 1557–1562)
- Trigger: Long agentic dialogues where the robot calls many tools before speaking.
- Workaround: `_trim_history` runs at conversation end, not within a single agent run. Mid-run input list is unbounded.

**BUG-13: Tool results accumulate in conversation history (TASK-044, pending):**
- Symptoms: `function_call_output` items from earlier turns are kept in the trimmed 20-turn history. Large tool outputs (music, navigation) inflate every subsequent LLM request.
- Files: `src/rob_box_voice/rob_box_voice/dialogue_node.py` (lines 1637, 1718–1757)
- Trigger: Conversations involving music or mapping tools that return verbose JSON.
- Workaround: `_truncate_history_outputs` truncates to 200 chars per output, but does not remove them.

**BUG-16: LLM forgets system prompt / personality after 15+ iterations (TASK-046, pending):**
- Symptoms: After many dialogue turns, robot ignores persona rules and gives terse uncharacteristic answers.
- Files: `src/rob_box_voice/rob_box_voice/dialogue_node.py`
- Trigger: Long sessions (evening events) with 20+ user exchanges.
- Workaround: None currently. TASK-046 proposes periodic system reminder injection.

**TASK-047: barge-in regression uncovered by tests:**
- Symptoms: Regression introduced in commit `37527df` broke barge-in (speech interruption mid-TTS). No integration tests exist to catch it.
- Files: `src/rob_box_voice/rob_box_voice/tts_node.py`, `src/rob_box_voice/rob_box_voice/audio_node.py`
- Impact: User cannot interrupt the robot mid-speech; robot must finish before next input is accepted.

**BLE joystick blocked by kernel 6.14.0-raspi regression:**
- Symptoms: Bluetooth ExpressLRS joystick connects but HID service is hidden by kernel; `/dev/input/js0` is never created.
- Files: `docs/fixes/BLE_JOYSTICK_KERNEL_REGRESSION_2026-01-18.md`
- Trigger: Ubuntu 25.04 Plucky with kernel 6.14.0-raspi on Raspberry Pi 5.
- Workaround: No software workaround. Blocked on upstream kernel fix or kernel downgrade.

**VESC wheel jitter on stop (partially fixed):**
- Symptoms: Wheels twitched when stopping because 0 RPM commands continued at 50 Hz. Timeout mechanism added.
- Files: `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp`, `VESC_TIMEOUT_FIX_CHANGELOG.md`
- Current state: Fixed with 0.5s timeout, but timeout is only tracked per-joint; simultaneous multi-joint edge cases have not been tested.

---

## Security Considerations

**Hardcoded SSH password in CI/CD workflow:**
- Risk: SSH password `'open'` is plaintext in `.github/workflows/L-Deploy and Verify.yml` (20+ occurrences). Anyone with repo read access can extract the robot password.
- Files: `.github/workflows/L-Deploy and Verify.yml` (lines 250, 263, 281, 296, 312, 325, …)
- Current mitigation: Local runners only; repository is private.
- Recommendations: Move password to GitHub Actions secret (`${{ secrets.ROBOT_SSH_PASS }}`); prefer SSH key-based auth over password.

**Same hardcoded password in documentation:**
- Risk: `sshpass -p 'open'` appears in multiple docs files and fix guides, all committed to git history.
- Files: `docs/fixes/ZENOH_TRANSPORT_FIX_QUICKREF.md`, `docs/fixes/ZENOH_FIX_2025-11-10_MAXIMUM.md`, `docs/fixes/ZENOH_FIX_2025-11-10_DEPLOYMENT.md`, `docs/fixes/FIX_SOUND_REPETITION_ISSUE.md`
- Recommendations: Replace with placeholder `<ROBOT_PASSWORD>` in docs; rotate the actual password and store in secrets manager.

**Zenoh has no authentication:**
- Risk: Zenoh session and router configs have `password: null`. Any device on the 10.1.1.0/24 network can subscribe to all ROS topics including camera streams and motor commands.
- Files: `docker/vision/config/zenoh_session_config.json5` (line 789), `docker/main/config/zenoh_router_config.json5` (line 792)
- Current mitigation: Network is private (direct Ethernet link between Pi boards).
- Recommendations: Enable Zenoh TLS + username/password for production deployments. Acceptable for lab use on isolated network.

**Privileged Docker containers:**
- Risk: 5 services on Main Pi run `privileged: true` (micro-ros-agent, ros2-control, lslidar, perception, nav2-related services). Two services on Vision Pi also run privileged (oak-d, led-matrix).
- Files: `docker/main/docker-compose.yaml` (lines 68, 217, 255, 372, 395), `docker/vision/docker-compose.yaml` (lines 29, 286)
- Current mitigation: Required for CAN bus, SPI, and serial hardware access.
- Recommendations: Replace broad `privileged: true` with targeted `devices:` mounts and `cap_add: [SYS_RAWIO]` where possible; review whether all five main-Pi services genuinely need full privilege.

**`network_mode: host` for all containers:**
- Risk: All containers share the host network namespace. A compromised container has direct access to all host network interfaces.
- Files: `docker/vision/docker-compose.yaml`, `docker/main/docker-compose.yaml` (all services)
- Current mitigation: Required for Zenoh DDS (multicast discovery relies on host network).
- Recommendations: Acceptable given ROS 2 DDS requirements; document this as a known accepted risk.

**Unpinned `ollama/ollama:latest` image:**
- Risk: The Ollama container pulls `:latest` tag, making builds non-reproducible. A breaking upstream change could silently break local LLM inference.
- Files: `docker/vision/docker-compose.yaml` (line 325)
- Recommendations: Pin to a specific digest or version tag (e.g., `ollama/ollama:0.6.5`).

---

## Performance Bottlenecks

**Polling in `async_executor._wait_for_result`:**
- Problem: Busy-polls `results_cache` at 50 ms intervals instead of using `asyncio.Event`.
- Files: `src/rob_box_mcp_tools/rob_box_mcp_tools/async_executor.py` (lines 337–357)
- Cause: Designed for cross-thread result delivery; polling was chosen as simple solution.
- Improvement path: Use `asyncio.Event` per request; signal it from the thread that produces the result via `loop.call_soon_threadsafe`.

**`dialogue_node.py` context size grows within a single agent run:**
- Problem: During one user turn, `input_list` is built from full `_conversation` plus user input, then tool results are appended inline for each iteration. 20 tool calls × average 300-token results = 6,000+ tokens per second turn at iteration 15.
- Files: `src/rob_box_voice/rob_box_voice/dialogue_node.py` (lines 1557–1570)
- Cause: OpenAI Agents SDK accumulates all tool outputs in the run's message list; no mid-run trimming.
- Improvement path: Implement mid-run summarisation or cap tool result size before passing to SDK.

**`voice_memory.py` Ollama embedding calls are synchronous:**
- Problem: Embedding generation for memory retrieval (FAQs, long-term memory) is called synchronously within the async dialogue handler, blocking the event loop.
- Files: `src/rob_box_voice/rob_box_voice/core/voice_memory.py` (line 85)
- Cause: `httpx` client used directly; no async wrapper.
- Improvement path: Use `httpx.AsyncClient` and `await` the embedding calls; or run them in a thread pool via `asyncio.run_in_executor`.

---

## Fragile Areas

**`audio_node.py` bare `except:` clauses in shutdown:**
- Files: `src/rob_box_voice/rob_box_voice/audio_node.py` (lines 325, 331, 337)
- Why fragile: Three bare `except: pass` blocks swallow all errors including `SystemExit` and `KeyboardInterrupt` during shutdown. Real hardware errors (PyAudio stream still locked) are silently ignored.
- Safe modification: Replace with `except OSError` and log the error at `WARN` level before continuing shutdown.

**`calibrate_max_rpm.py` and `linearity_test.py` swallow all exceptions during motor runs:**
- Files: `src/vesc_nexus/src/vesc_nexus/tools/calibrate_max_rpm.py` (line 122), `src/vesc_nexus/src/vesc_nexus/tools/linearity_test.py` (line 123)
- Why fragile: Calibration tools run motors at increasing speeds; a CAN bus error that is silently swallowed could leave motors running at maximum speed.
- Safe modification: Catch `Exception`, send zero-velocity command, then re-raise or log critically.

**VESC hardware interface has no NaN/infinity guard on velocity commands:**
- Files: `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp` (write function, ~line 150+)
- Why fragile: If ros2_control passes a NaN command (possible during controller initialisation race), the VESC handler receives it without a validity check. VESC behaviour with NaN CAN frames is undefined.
- Safe modification: Add `std::isnan(cmd_velocities_[i]) || std::isinf(cmd_velocities_[i])` guard before `sendSpeed`; log `RCLCPP_ERROR` and skip the command.

**Zenoh IPs hardcoded in router configs:**
- Files: `docker/vision/config/zenoh_router_config.json5` (line 98), `docker/main/config/zenoh_router_config.json5` (line 100)
- Why fragile: `10.1.1.10` and `10.1.1.11` are hardcoded. Any network change (DHCP, test environment, second robot unit) requires manual config edits across multiple files.
- Safe modification: Parameterise via environment variables (`MAIN_PI_IP`, `VISION_PI_IP`) in docker-compose and substitute into config at container start via entrypoint script.

**`test_dialogue_node.py` is a skeleton with 13+ empty test methods:**
- Files: `src/rob_box_voice/test/test_dialogue_node.py` (lines 36–156)
- Why fragile: All test methods contain only a `# TODO` comment and `pass`. The test file appears in CI and passes (zero assertions), giving false confidence in dialogue node coverage.
- Safe modification: Either implement the tests or mark them `@pytest.mark.skip(reason="…")` so coverage reporting is honest.

**`led_matrix_driver.py` exception in `clear_matrix` during shutdown may leave LEDs lit:**
- Files: `src/led_matrix_driver/led_matrix_driver/led_matrix_driver.py` (lines 50, 94, 105)
- Why fragile: If SPI write fails during shutdown (e.g., device removed), the `except Exception as e` only logs and continues. LEDs may remain in last animation state.
- Safe modification: Retry clear once, then force a hardware reset signal if available.

---

## Scaling Limits

**`docker/build/` directory contains 8 full runner workspace copies:**
- Current state: `docker/build/data/runner{1..8}/_work/rob_box_project/` — 8 complete copies of the repository for local GitHub Actions runners.
- Limit: Disk usage scales linearly with runner count × repo size. At ~50 MB per workspace and growing, this is already several hundred MB of duplication.
- Scaling path: Use shared read-only bind mounts for runners, or move runners to ephemeral Docker containers.

**Single Zenoh router on Main Pi is a single point of failure:**
- Current capacity: One `zenoh-router` container bridges all DDS traffic between both Pi boards.
- Limit: If the Main Pi reboots, Vision Pi loses all ROS topic visibility, including TTS and camera pipeline.
- Scaling path: Configure Vision Pi with a local router and bidirectional connection so that either side can act as primary.

**LLM providers are external APIs with no offline fallback:**
- Current capacity: DeepSeek and Qwen APIs (cloud). Ollama is available locally but `enable_fallback: False` by default.
- Limit: Any internet outage or API outage silences the voice assistant entirely.
- Scaling path: Set `enable_fallback: True` and configure Ollama as the offline fallback; test fallback path in CI.

---

## Dependencies at Risk

**`ollama/ollama:latest` (unpinned):**
- Risk: Upstream Ollama releases (model format changes, API breaking changes) can silently break local inference after `docker pull`.
- Impact: Local TTS fallback and FAQ embeddings break without warning.
- Migration plan: Pin to a specific version tag; add a `docker pull` + smoke-test step in CI before deployment.

**`DeepSeek` and `Qwen` external LLM APIs:**
- Risk: Both providers have experienced API outages and rate limit changes. `dialogue_node.py` raises `RuntimeError` if no API key is present.
- Impact: Complete voice assistant failure if either provider has an outage and fallback is disabled.
- Migration plan: Enable `enable_fallback: True`; add health-check probe to CI that verifies Ollama local model is loaded before deployment.

**`Silero TTS` model runtime download:**
- Risk: Silero model is downloaded at container startup from a PyTorch Hub URL. If the CDN is unavailable, the TTS container crashes on startup.
- Files: `src/rob_box_voice/rob_box_voice/tts_node.py`
- Impact: No speech output until model loads; startup fails if offline.
- Migration plan: Pre-bake the Silero model into the Docker image during CI build.

**`pyaudio` + ALSA on Raspberry Pi 5 with Ubuntu 25.04:**
- Risk: ALSA driver layer and PyAudio versioning are tightly coupled to the kernel. Ubuntu 25.04 Plucky uses a newer kernel (6.14.0-raspi) that already caused one regression (BLE joystick).
- Files: `src/rob_box_voice/rob_box_voice/audio_node.py`, `src/rob_box_voice/rob_box_voice/utils/audio_utils.py`
- Migration plan: Pin Ubuntu base image to 24.04 LTS in Dockerfiles to avoid unstable kernel regression exposure.

---

## Missing Critical Features

**Robot position integration in MCP tools:**
- Problem: `get_robot_status` returns `x:0, y:0, theta:0` always. Navigation and exploration voice commands cannot report actual position.
- Blocks: Any voice command that references robot location ("where are you?", "go back to where you were").

**Object detection and person following (Phase 6):**
- Problem: `detect_objects` and `follow_person` commands in `command_node.py` are empty stubs.
- Files: `src/rob_box_voice/rob_box_voice/command_node.py` (lines 359, 370)
- Blocks: Interactive vision-based voice commands.

**Automated integration tests for agent cycle (TASK-041, pending):**
- Problem: The recursive agent loop (`dialogue_node.py`) has no automated end-to-end tests in CI. Regressions (like the barge-in regression in commit `37527df`) are discovered in production.
- Blocks: Safe iteration on LLM prompt changes and tool additions.

**System reminders for long LLM sessions (BUG-16, TASK-046, pending):**
- Problem: After 15+ dialogue turns, the LLM loses awareness of persona rules because the system prompt is only in the first message and gets pushed out of the effective context window.
- Blocks: Reliable multi-hour event operation.

---

## Test Coverage Gaps

**`dialogue_node.py` — 13.4% coverage:**
- What's not tested: Agent loop logic, provider switching, conversation trimming, barge-in handling, FAQ mode, DJ mode, event mode activation.
- Files: `src/rob_box_voice/rob_box_voice/dialogue_node.py`
- Risk: Any refactor or prompt change can break core voice assistant behaviour undetected.
- Priority: High

**`vesc_nexus` — zero pytest unit tests:**
- What's not tested: Motor velocity calculations, gear ratio application, timeout logic, NaN handling, CAN error recovery.
- Files: `src/vesc_nexus/src/vesc_nexus/src/vesc_system_hardware_interface.cpp`, `src/vesc_nexus/src/vesc_nexus/include/vesc_nexus/`
- Risk: Motor control regressions are only caught by running on physical hardware. Safety-critical path.
- Priority: High

**`rob_box_animations` — zero pytest unit tests:**
- What's not tested: `frame_renderer.py`, `animation_player.py`, `animation_loader.py`.
- Files: `src/rob_box_animations/rob_box_animations/`
- Risk: Animation regressions (LED state corruption, player loop bugs) are undetected.
- Priority: Medium

**`rob_box_teleop` — zero pytest unit tests:**
- What's not tested: Teleoperation command mapping, safety limits.
- Files: `src/rob_box_teleop/`
- Risk: Safety-critical teleop changes have no automated regression check.
- Priority: Medium

**`test_dialogue_node.py` — 13 skeleton test methods (all `pass`):**
- What's not tested: DialogueNode instantiation, LLM call routing, error handling, queue processing, context tracking.
- Files: `src/rob_box_voice/test/test_dialogue_node.py` (lines 36–156)
- Risk: CI reports passing tests but provides zero coverage validation. False confidence.
- Priority: High

**`test_audio_node.py` — 7 skeleton test methods (all `pass`):**
- What's not tested: PyAudio device selection, VAD threshold behavior, stream error recovery.
- Files: `src/rob_box_voice/test/test_audio_node.py` (lines 32–87)
- Priority: Medium

---

*Concerns audit: 2026-05-15*
