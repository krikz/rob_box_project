# Key Interfaces and APIs — Voice Persistent Nodes

Source: `src/rob_box_voice/rob_box_voice/`
Extracted: 2026-07-17

---

## Architecture Overview

7 ROS2 nodes forming the voice pipeline:

```
AudioNode → STTNode → DialogueNode → TTSNode → SoundNode
    ↓                      ↓              ↓
LEDNode              CommandNode     Animations
```

---

## 1. AudioNode (`audio_node.py`)

**Class**: `AudioNode(Node)` — ReSpeaker Mic Array v2.0 audio capture + VAD/DoA.

### ROS2 Publishers
- `/audio/audio` (`AudioData`, BEST_EFFORT) — raw audio stream (16kHz mono)
- `/audio/speech_audio` (`AudioData`, BEST_EFFORT) — detected speech segments
- `/audio/vad` (`Bool`) — Voice Activity Detection (true = speech)
- `/audio/direction` (`Int32`) — Direction of Arrival (0–359°)
- `/audio/state` (`String`) — node state: `ready`, `running`, `error_no_device`, `error_stream`, `stopped`
- `/voice/tts/control` (`String`) — sends STOP to interrupt TTS

### ROS2 Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sample_rate` | int | 16000 | Audio sample rate (Hz) |
| `channels` | int | 1 | Audio channels |
| `chunk_size` | int | 1024 | Buffer chunk size |
| `vad_threshold` | float | 3.5 | VAD sensitivity (dB) |
| `publish_rate` | int | 10 | VAD/DoA poll rate (Hz) |
| `device_index` | int | -1 | Audio device (-1 = auto) |
| `device_name` | str | "ReSpeaker 4 Mic Array" | Device name filter |
| `speech_continuation` | float | 1.5 | Speech continuation time (s) |
| `speech_prefetch` | float | 0.5 | Pre-speech buffer (s) |
| `speech_min_duration` | float | 0.3 | Min speech duration (s) |
| `speech_max_duration` | float | 15.0 | Max speech duration (s) |

### Public Methods
- `initialize_hardware()` — connect ReSpeaker USB + open PyAudio stream
- `audio_callback(in_data, frame_count, time_info, status)` — PyAudio stream callback
- `check_vad_and_doa()` — timer: read VAD/DoA from ReSpeaker, publish speech segments
- `publish_state(state: str)` — publish node state
- `list_available_devices()` — log all PyAudio devices
- `shutdown()` — graceful cleanup (stream + ReSpeaker + PyAudio)

---

## 2. STTNode (`stt_node.py`)

**Class**: `STTNode(Node)` — Speech-to-Text: Yandex gRPC v3 (primary) + Vosk (fallback).

### ROS2 Subscribers
- `/audio/speech_audio` (`AudioData`, BEST_EFFORT) — speech segments from AudioNode
- `/voice/tts/state` (`String`) — TTS state for echo suppression

### ROS2 Publishers
- `/voice/stt/result` (`String`) — recognised text
- `/voice/stt/state` (`String`) — node state: `ready`, `recognizing`
- `/voice/tts/control` (`String`) — sends STOP on wake word detection (barge-in)

### ROS2 Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_path` | str | `/models/vosk-model-small-ru-0.22` | Vosk model path |
| `sample_rate` | int | 16000 | Audio sample rate |
| `yandex_api_key` | str | "" | Yandex Cloud API key |
| `yandex_language` | str | `ru-RU` | Recognition language |
| `yandex_model` | str | `general` | Yandex STT model |
| `eou_profile` | str | `balanced` | End-of-utterance: `fast` / `balanced` / `patient` |
| `aec_mode` | str | `hardware` | Echo cancellation: `software` / `hardware` |
| `wake_words` | list | `[робок, робот, роббокс]` | Wake words for TTS interruption |

### EOU Profiles (End of Utterance)
- `fast` — HIGH classifier, 700ms max pause
- `balanced` — DEFAULT classifier, 1200ms max pause
- `patient` — DEFAULT classifier, 2000ms max pause

### Public Methods
- `initialize_yandex()` — connect Yandex gRPC STT v3 channel
- `initialize_vosk()` — load Vosk model from disk
- `speech_audio_callback(msg)` — main: recognize speech segment (Yandex → Vosk fallback)
- `tts_state_callback(msg)` — track robot speaking state for echo suppression
- `_recognize_yandex(audio_bytes) → str|None` — Yandex gRPC streaming recognition
- `_recognize_vosk(audio_bytes) → str|None` — Vosk offline recognition
- `publish_result(text)` — publish recognised text + trigger TTS stop on wake word
- `publish_state(state)` — publish node state

---

## 3. DialogueNode (`dialogue_node.py`)

**Class**: `DialogueNode(Node)` — Central voice dialogue agent (OpenAI Agents SDK).

### ROS2 Subscribers
- `/voice/stt/result` (`String`) — recognised speech from STTNode
- `/audio/vad` (`Bool`) — VAD for barge-in tracking
- `/voice/tts/finished` (`String`) — TTS completion signal (JSON `{speech_id, success}`)
- `/voice/sound/state` (`String`) — sound playback state (for `ready` sync)
- `/voice/dj_mode` (`String`) — DJ mode commands

### ROS2 Publishers
- `/voice/dialogue/response` (`String`) — JSON chunks with SSML for TTS
- `/voice/dialogue/state` (`String`) — current state: `IDLE` / `LISTENING` / `DIALOGUE` / `SILENCED`
- `/voice/sound/trigger` (`String`) — trigger sound effects
- `/voice/tts/control` (`String`) — STOP command for TTS interruption

### ROS2 Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `provider` | str | `deepseek` | LLM provider: `deepseek` / `mimo` |
| `api_key` | str | "" | API key override |
| `base_url` | str | "" | Base URL override |
| `model` | str | "" | Model override |
| `temperature` | float | 0.7 | LLM temperature |
| `max_tokens` | int | 500 | Max response tokens |
| `system_prompt_file` | str | `master_prompt_compact.txt` | System prompt template |
| `history_max_turns` | int | 20 | Max conversation turns kept |
| `agent_max_turns` | int | 20 | Max agent tool-call loop turns |
| `dialogue_timeout` | float | 300.0 | Inactivity timeout (s) |
| `wake_words` | list | `[робок, робот, роббокс]` | Wake words |
| `enable_mcp_tools` | bool | True | Enable MCP tool adapter |
| `enable_fallback` | bool | False | Enable fallback model on errors |
| `fallback_model` | str | "" | Fallback model name |
| `llm_timeout_sec` | float | 90.0 | LLM request timeout |
| `verbose_llm` | bool | True | Verbose LLM logging |
| `faq_mode_enabled` | bool | False | Event FAQ mode |
| `faq_event_config_file` | str | "" | FAQ event YAML path |
| `history_excluded_tools` | list | `[handle_navigation]` | Tools excluded from history |

### Supported LLM Providers
| Provider | Base URL | Default Model | Fallback Model | Env Vars |
|----------|----------|---------------|----------------|----------|
| `deepseek` | `https://api.deepseek.com/v1` | `deepseek-v4-flash` | `deepseek-v4-flash` | `DEEPSEEK_API_KEY`, `LLM_API_KEY` |
| `mimo` | `https://api.xiaomimimo.com/v1` | `mimo-v2.5-pro` | `mimo-v2.5` | `MIMO_API_KEY`, `LLM_API_KEY` |

### Agent Tool Functions (Flat Mode — 30 tools)

**Output tools** (locked, sequential):
- `speak_text(text, animation="neutral")` — speak via TTS, blocks until playback done
- `play_sound(sound)` — play sound effect, blocks until done
- `play_animation(animation, duration=3.0)` — trigger LED animation

**Memory tools**:
- `memory_context(limit=10)` — load recent conversation context
- `memory_save(fact, category="general")` — save persistent fact about user
- `memory_search(query, limit=5)` — search long-term memory

**FAQ tools**:
- `faq_search(query, limit=3)` — search event FAQ database

**Status tools**:
- `get_current_time()` — current date/time
- `get_robot_status()` — battery, sensors, navigation state
- `get_battery_level()` — battery percentage

**Navigation tools**:
- `navigate_to_waypoint(waypoint)` — navigate to saved waypoint (blocks until arrival)
- `navigate_to_coordinates(x, y, theta=0.0)` — navigate to map coordinates
- `move_direction(direction, distance=0.5)` — move: `вперёд`, `назад`, `налево`, `направо`
- `list_waypoints()` — list all saved waypoints
- `save_waypoint(name)` — save current position as waypoint
- `delete_waypoint(name)` — delete a waypoint
- `clear_waypoints()` — delete all waypoints
- `get_current_pose()` — get robot (x, y, theta)

**Voice control**:
- `set_volume(action)` — volume: `louder`, `quieter`, `max`, `normal`
- `set_pitch(pitch)` — voice pitch: 0.5–2.0

**Music/Synthesis tools**:
- `search_samples(query, pack="0_foxdot_default", case="lower")` — search Renardo samples
- `execute_music_code(code, pattern_name="p1")` — run Renardo/SuperCollider code
- `stop_music(pattern_name="")` — stop music playback
- `set_vibe_preset(preset)` — music vibe: `chill`, `energetic`, `ambient`, `jazz`, `dark`
- `get_music_state()` — current music state

**DJ mode tools**:
- `set_dj_mode(enabled, next_transition_sec=0, theme="")` — toggle autonomous DJ
- `list_tracks(tag="", min_rating=0)` — list saved tracks
- `save_track(name, title="", description="", tags="", rating=0, notes="")` — save track
- `load_track(name)` — load and play saved track
- `delete_track(name)` — delete track

### Skill Sub-Agents (Compositor Mode — 5 skills)

When `USE_SKILLS=true`:

- `handle_music` — MusicSkill: execute_music_code, stop_music, set_vibe_preset, get_music_state, search_samples, set_dj_mode
- `handle_navigation` — NavigationSkill: navigate_to_waypoint, move_direction, waypoint CRUD
- `handle_memory` — MemorySkill: memory_save, memory_search, memory_context
- `handle_status` — StatusSkill: battery, time, robot status, volume, pitch
- `handle_faq` — FAQSkill: event FAQ search (loaded only when FAQ mode enabled)

### Dialogue States (`DialogueManager`)
- `IDLE` — waiting for wake word
- `LISTENING` — wake word detected, listening for command
- `DIALOGUE` — processing user request (LLM running)
- `SILENCED` — "помолчи" mode, only unsilence commands accepted

### Key Internal Methods
- `_load_system_prompt() → str` — load prompt template from package share
- `_build_agent(model_override="")` — (re)build OpenAI Agents SDK Agent
- `_make_tools() → list` — create flat-mode @function_tool list
- `_make_output_tools() → list` — create output tools for compositor mode
- `_build_skills(model) → list` — instantiate skill sub-agents
- `_cancel_run(reason)` — cancel in-progress agent run (barge-in)
- `_on_stt(msg)` — STT result handler: wake word gate → start agent
- `_on_vad(msg)` — VAD speech flag (barge-in tracking only)

---

## 4. TTSNode (`tts_node.py`)

**Class**: `TTSNode(Node)` — Text-to-Speech: Yandex gRPC v3 (primary, anton voice) + Silero v5 (fallback).

### ROS2 Subscribers
- `/voice/dialogue/response` (`String`) — JSON chunks with SSML from DialogueNode
- `/voice/tts/request` (`String`) — TTS requests from reflection_node etc.
- `/voice/tts/control` (`String`) — STOP command
- `/voice/current_dialogue_id` (`String`) — dialogue session tracking (stale request filtering)

### ROS2 Publishers
- `/voice/audio/speech` (`AudioData`) — synthesized speech audio
- `/voice/tts/state` (`String`) — state: `ready`, `synthesizing`, `playing`, `stopped`
- `/voice/tts/finished` (`String`) — JSON `{speech_id, success, error}`

### ROS2 Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `provider` | str | `yandex` | TTS provider: `yandex` / `silero` |
| `yandex_api_key` | str | "" | Yandex Cloud API key |
| `yandex_voice` | str | `anton` | Original ROBBOX voice |
| `yandex_speed` | float | 1.0 | Speech speed (0.1–3.0) |
| `silero_speaker` | str | `baya` | Silero speaker: `aidar` / `baya` / `kseniya` / `xenia` / `eugene` |
| `silero_sample_rate` | int | 48000 | Silero output rate |
| `silero_put_accent` | bool | True | Auto stress mark placement |
| `silero_put_yo` | bool | True | Auto ё placement |
| `silero_put_stress_homo` | bool | True | Homograph stress disambiguation |
| `silero_put_yo_homo` | bool | True | Homograph ё disambiguation |
| `chipmunk_mode` | bool | True | Chipmunk voice effect |
| `pitch_shift` | float | 1.0 | Playback rate multiplier |
| `normalize_text` | bool | True | Text normalization before TTS |
| `volume_db` | float | -3.0 | Output volume (dB) |

### Public Methods
- `initialize_audio_device()` — setup ALSA default → dmix_respeaker
- `_load_silero_model()` — lazy-load Silero v5 from /models/silero/v5_ru.pt
- `control_callback(msg)` — handle STOP command
- `_interrupt_playback()` — stop current playback + discard dialogue ID
- `_on_new_dialogue_id(msg)` — update current dialogue session
- `dialogue_callback(msg)` — process JSON chunk: extract SSML, synthesize, play
- `_extract_text_from_ssml(ssml) → str` — strip XML tags
- `_parse_ssml_attributes(ssml) → dict` — extract pitch/rate from `<prosody>`
- `parameters_callback(params)` — runtime parameter changes

### SSML Support
SSML `<prosody>` tag attributes:
- `pitch`: `+10%`, `-10%`, `high`, `low`, `medium`, or float multiplier
- `rate`: float speed multiplier

---

## 5. SoundNode (`sound_node.py`)

**Class**: `SoundNode(Node)` — Sound effects playback from `sound_pack/`.

### ROS2 Subscribers
- `/voice/sound/trigger` (`String`) — trigger name (e.g. `thinking`, `cute`, `talk`)

### ROS2 Publishers
- `/voice/sound/state` (`String`) — state: `ready`, `playing_<name>`, `error_no_dir`, `error_no_sounds`
- `/animations/trigger` (`String`, optional) — animation trigger for LED matrix sync

### ROS2 Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sound_pack_dir` | str | `/ws/sound_pack` | Sound pack directory |
| `volume_db` | float | -12.0 | Global volume (dB) |
| `trigger_animations` | bool | True | Auto-trigger LED animations |
| `animation_topic` | str | `/animations/trigger` | Animation trigger topic |

### Trigger → Sound Resolution (4-tier)
1. `trigger_map` from `sound_catalog.json`
2. Direct filename match (legacy)
3. Random group selection (`talk`, `cute`, `confused`, `drip`, `work`, `talk_beep`)
4. Fuzzy substring match

### Public Methods
- `initialize_audio_device()` — ALSA default → dmix_respeaker
- `load_sounds()` — load all MP3 from sound_pack + catalog.json
- `trigger_callback(msg)` — handle sound trigger
- `select_sound(trigger) → str|None` — resolve trigger to sound name
- `play_sound_thread(sound_name, trigger)` — playback in background thread
- `cleanup_playback_noise()` — stop stream + USB stabilization delay
- `trigger_animation(trigger)` — publish matching animation trigger
- `publish_state(state)` — publish state
- `parameters_callback(params)` — runtime volume changes (reloads sounds)

---

## 6. LEDNode (`led_node.py`)

**Class**: `LEDNode(Node)` — 12× RGB LED ring on ReSpeaker.

### ROS2 Subscribers
- `/voice/state` (`String`) — voice assistant state: `idle`, `listening`, `thinking`, `speaking`, `error`
- `/audio/direction` (`Int32`) — DoA from AudioNode
- `/voice/animation/request` (`String`) — animation sync: `police_lights`, `ambulance`, `fire_truck`, `road_service`

### ROS2 Services
- `/voice/set_auto_led` (`SetBool`) — enable/disable auto LED mode

### ROS2 Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `brightness` | int | 16 | LED brightness (0–31) |
| `auto_mode` | bool | True | Auto mode (follow voice state) |
| `colors.idle` | list | `[0,0,0]` | Idle color (RGB) |
| `colors.listening` | list | `[0,255,0]` | Listening color (green) |
| `colors.thinking` | list | `[0,0,255]` | Thinking color (blue) |
| `colors.speaking` | list | `[0,255,255]` | Speaking color (cyan) |
| `colors.error` | list | `[255,0,0]` | Error color (red) |

### PixelRingLite Commands (USB HID)
| Method | CMD | Description |
|--------|-----|-------------|
| `trace()` | 0 | DoA tracking |
| `mono(r,g,b)` | 1 | Solid color |
| `listen()` | 2 | Directional glow |
| `speak()` | 3 | Rotating animation |
| `think()` | 4 | Pulsing animation |
| `spin()` | 5 | Fast rotation |
| `set_brightness(n)` | 0x20 | Brightness 0–31 |
| `set_color_palette(c1,c2)` | 0x21 | Dual-color palette |
| `set_volume(n)` | 0x23 | Volume indicator 0–12 |

### LED-Matrix Sync (Ring → Matrix)
Animation name → ring alternating colors:
- `police_lights` — blue ↔ red, 0.3s
- `ambulance` — red ↔ white, 0.25s
- `fire_truck` — red ↔ orange, 0.2s
- `road_service` — orange ↔ yellow, 0.35s

### Public Methods
- `initialize_hardware()` — connect ReSpeaker LED + set initial trace mode
- `state_callback(msg)` — handle voice state change
- `direction_callback(msg)` — handle DoA change
- `animation_request_callback(msg)` — sync ring with LED-matrix animation
- `set_auto_led_callback(request, response)` — ROS2 service handler
- `set_mode_manual(mode)` — manual control: `off`, `trace`, `listen`, `think`, `speak`, `spin`
- `shutdown()` — stop animations, turn off LEDs

---

## 7. CommandNode (`command_node.py`)

**Class**: `CommandNode(Node)` — Voice command recognition and execution via Nav2.

### ROS2 Subscribers
- `/voice/stt/result` (`String`) — recognised speech from STTNode
- `/voice/dialogue/state` (`String`) — dialogue state (to avoid interfering)

### ROS2 Publishers
- `/voice/command/intent` (`String`) — `intent_type:confidence` (e.g. `navigate:0.85`)
- `/voice/command/feedback` (`String`) — user feedback messages
- `/cmd_vel_voice` (`Twist`) — velocity commands (twist_mux priority 25)

### ROS2 Actions
- `/navigate_to_pose` (`NavigateToPose`) — Nav2 navigation action client

### ROS2 Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `confidence_threshold` | float | 0.7 | Minimum confidence to execute |
| `enable_navigation` | bool | True | Enable Nav2 navigation |
| `enable_follow` | bool | False | Enable follow mode (Phase 6) |
| `enable_vision` | bool | False | Enable vision commands (Phase 6) |

### Intent Types (`IntentType`)
- `NAVIGATE` — navigate to waypoint, direction
- `STOP` — stop movement
- `FOLLOW` — person following
- `STATUS` — status request
- `MAP` — map operations
- `VISION` — object detection
- `UNKNOWN` — pass to dialogue_node

### Direction Commands (relative Nav2 goals)
- `налево` / `влево` — rotate 90° left
- `направо` / `вправо` — rotate 90° right
- `вперед` / `вперёд` — move 1m forward
- `назад` — move 1m backward

### Public Methods
- `stt_callback(msg)` — parse command, classify intent, execute
- `execute_command(command)` — dispatch by intent type
- `handle_navigate(command)` — waypoint or direction navigation
- `handle_stop(command)` — cancel all Nav2 goals
- `handle_direction(direction)` — relative Nav2 goal
- `handle_status(command)` — status stub
- `send_nav2_goal(x, y, theta)` — absolute Nav2 goal (map frame)
- `send_relative_nav2_goal(x, y, theta)` — relative Nav2 goal (base_link frame)
- `publish_intent(command)` — publish intent:confidence
- `publish_feedback(text)` — publish user feedback

---

## 8. Core Libraries

### DialogueManager (`core/dialogue_manager.py`)
**Class**: `DialogueManager`

States: `IDLE` → `LISTENING` → `DIALOGUE` → `IDLE` (+ `SILENCED`)

| Method | Description |
|--------|-------------|
| `is_wake_word(text) → bool` | Check wake word presence |
| `has_wake_word(text) → bool` | Alias for is_wake_word |
| `remove_wake_word(text) → str` | Strip wake word from text |
| `is_silence_command(text) → bool` | Check: "помолчи" etc. |
| `is_unsilence_command(text) → bool` | Check: "говори" etc. |
| `should_respond(text) → bool` | State-aware respond decision |
| `transition_state(new_state)` | Change state + update timestamp |
| `enable_silence(duration=300.0)` | Enter SILENCED for N seconds |
| `disable_silence()` | Return to IDLE |
| `is_silenced() → bool` | Check + auto-expire |
| `check_timeout() → bool` | Check dialogue timeout → IDLE |
| `add_query(query)` | Accumulate query |
| `should_process_queries() → bool` | Check accumulation timeout |
| `get_accumulated_queries() → list` | Get + clear query queue |
| `reset()` | Full state reset |

### CommandParser (`core/command_parser.py`)
**Class**: `CommandParser`

| Method | Description |
|--------|-------------|
| `parse(text) → Command` | Parse full command |
| `remove_wake_word(text) → str` | Strip wake word |
| `classify_intent(text) → Command` | Pattern match → intent + entities |
| `add_pattern(intent, pattern, entity_type)` | Register custom pattern |
| `get_patterns(intent) → list` | Get patterns for intent |

### VoiceMemory (`core/voice_memory.py`)
**Class**: `VoiceMemory`

SQLite + FTS5 + optional sqlite-vec/Ollama vector search.

| Method | Description |
|--------|-------------|
| `save_turn(role, content, ...)` | Persist user/assistant turn |
| `load_recent_turns(limit, exclude_current_session) → list` | Load recent history |
| `search(query, limit=5) → list` | Hybrid FTS5+vector search |
| `save_fact(fact, category) → int` | Save persistent user fact |
| `search_facts(query, limit=5) → list` | Search persistent facts |
| `get_stats() → dict` | DB statistics (turns, sessions) |

### FAQStore (`core/faq_store.py`)
**Class**: `FAQStore`

SQLite-backed FAQ with Ollama embeddings.

| Method | Description |
|--------|-------------|
| `replace_items(event_id, items) → int` | Replace FAQ for event |
| `search(query, event_id, limit=3) → list` | Semantic FAQ search |

---

## 9. MCP Tool Adapter (`rob_box_mcp_tools.llm_adapter`)

**Class**: `LLMToolCallAdapter`

Bridges DialogueNode tool calls to ROS2 services/actions.

| Method | Description |
|--------|-------------|
| `execute_tool_call_sync(tool_name, params, timeout=10.0) → dict` | Synchronous MCP tool execution |
| `execute_tool_call_async(tool_name, params) → Future` | Async MCP tool execution |

---

## 10. Skills Architecture

5 sub-agents (OpenAI Agents SDK), each with domain prompt + tool subset:

| Skill | Tool Name | Tools | agent_max_turns | temperature |
|-------|-----------|-------|-----------------|-------------|
| MusicSkill | `handle_music` | execute_music_code, stop_music, set_vibe_preset, get_music_state, search_samples, set_dj_mode, save_dj_set_plan, save_dj_persona, save_dj_theme | 10 | 0.85 |
| NavigationSkill | `handle_navigation` | navigate_to_waypoint, move_direction, list_waypoints, save_waypoint, delete_waypoint, clear_waypoints, get_current_pose | default | default |
| MemorySkill | `handle_memory` | memory_save, memory_search, memory_context | default | default |
| StatusSkill | `handle_status` | get_robot_status, get_battery_level, get_current_time, set_volume, set_pitch | default | default |
| FAQSkill | `handle_faq` | faq_search (direct, no MCP) | default | 0.2 |

---

## Inter-Node Communication Summary

```
┌─────────────┐    /audio/vad          ┌──────────────┐
│  AudioNode  │ ─────────────────────→ │ DialogueNode │
│             │    /audio/speech_audio  │              │
└──────┬──────┘                        └──────┬───────┘
       │                                      │
       │  /audio/speech_audio                 │  /voice/dialogue/response (JSON+SSML)
       ↓                                      ↓
┌─────────────┐    /voice/stt/result    ┌─────────────┐
│   STTNode   │ ──────────────────────→ │   TTSNode   │
│             │                         │             │
└─────────────┘                         └──────┬──────┘
       │                                      │
       │  /voice/stt/result                   │  /voice/tts/finished
       ↓                                      ↓
┌──────────────┐                       ┌─────────────┐
│ CommandNode  │                       │ SoundNode   │
│              │                       │ (effects)   │
└──────────────┘                       └─────────────┘
                                              ↑
                                     /voice/sound/trigger
                                              │
┌─────────────┐    /voice/state       ┌──────┴───────┐
│   LEDNode   │ ←─────────────────── │ DialogueNode │
└─────────────┘                       └──────────────┘

Control paths:
  STTNode    → /voice/tts/control (STOP)   → TTSNode (barge-in on wake word)
  AudioNode  → /voice/tts/control (STOP)   → TTSNode (VAD-based interrupt)
  DialogueNode → /voice/tts/control (STOP) → TTSNode (new dialogue ID)
  TTSNode    → /voice/tts/finished         → DialogueNode (awaited by speak_text)
  SoundNode  → /voice/sound/state (ready)  → DialogueNode (awaited by play_sound)
```

---

*Generated from source: `src/rob_box_voice/rob_box_voice/`*  
*7 nodes, ~45 ROS2 topics/services/actions, ~30 agent tools, 5 skill sub-agents, 3 core libraries*
