# Changelog

Все значимые изменения в проекте Rob Box документируются в этом файле.

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.0.0/),
и этот проект придерживается [Semantic Versioning](https://semver.org/lang/ru/).

## [v1.0.0-1-bfe71bf-humble-rc1] - 2026-08-06

Первый релиз v1.0.0 на ROS 2 Humble. RC (release candidate) — обкатка после
выставок и ДОДов. Build `bfe71bf` (short SHA от `origin/main`).

**Что включено:** состояние `main` (HEAD `bfe71bfb`) — рабочая лошадка,
которую гоняли на ДОДах и выставках. Стабильный baseline для hotfix'ов.

**НЕ включено:** `feature/harness-p0-foundation` (MiniMax LLM-интеграция,
llm_streaming, fallback'и, DJ-план, agent-flow proposal) — в активной
разработке, требует стабилизации. Дойдёт в `develop` → `main` после.

**Теги:** `v1.0.0-1-bfe71bf-humble-rc1` (annotated, не подписан)

**Платформа:** ROS 2 Humble (`humble`) — единственная поддерживаемая.

**Hotfix-процедура:**
1. `git checkout v1.0.0-1-bfe71bf-humble-rc1`
2. Создать ветку `hotfix/<short-name>` от тега
3. Fix → push → PR в этот тег (или в новый RC)
4. Новый релиз как `v1.0.0-2-<build>-humble-rc1` (build++)

## [v1.0.0-1-ab8b6b8-humble-rc1] - 2026-08-07

**Второй RC1** после `v1.0.0-1-bfe71bf-humble-rc1`. Build `ab8b6b8` (short SHA от `origin/main`).

**Включено:**
- Полная локальная сборка всех образов на self-hosted Katana (`L-Build All Services`)
- Push всех собранных образов в `ghcr.io/krikz/rob_box` (+ `rob_box_base` для base-images)
- Миграция с `G-Build All Services` (GitHub-hosted QEMU) на `L-Build All & Push to GHCR` (self-hosted)
- Обновление `.image-versions.{main,vision}.latest` после успешной сборки
- DJ-план через `set_dj_mode(plan=...)` + лимит переходов 8→30 (в feature, **не задеплоен**)
- Фильтр озвучки `[CRITICAL`, `[ПОЛИТИКА`, `[ROOT` + промпт-запрет (в feature, **не задеплоен**)

**НЕ включено:** `feature/harness-p0-foundation` — DJ-план, фильтр, fallback LLM MiniMax→DeepSeek,
llm_streaming, agent-flow. В активной разработке, требует стабилизации.

**Baseline:** `0.1.2` → `ab8b6b8` — **658 коммитов** (без автоматических `ci: SHA tags`).

**Платформа:** ROS 2 Humble (`humble`) — единственная поддерживаемая.

**Теги:** `v1.0.0-1-ab8b6b8-humble-rc1` (annotated, не подписан)

### Reverts (откаты)

- Revert "fix(06.08): (1) G-build rtabmap QEMU segfault workaround (apt retry + --no-install-recommends + ldconfig); (2) L-build DOCKER_TAG handling для refs/tags/* (теперь latest вместо local → .image-versions.latest обновляется). Issue #1041"
- Revert "fix(06.08 G-build): параметризовать APT_PROXY build-arg для всех 4 base-images — Dockerfile.rtabmap:10 ждёт APT_PROXY для apt-cacher. Раньше G-build передавал пустую строку → apt-get напрямую → медленнее/нестабильно. Теперь G-build использует тот же путь что и Katana local build (APT_PROXY=http://10.1.1.5:3142). Issue #1041"
- Revert "fix(06.08 G-build): объединить два build-args блока в один (BASE_IMAGE + APT_PROXY) — GH parser ругался на duplicate"
- Revert "fix(06.08 G-build): убрать хардкод http://10.1.1.5:3142 — GitHub Actions runner изолирован от локальной сети. APT_PROXY теперь пустая строка по умолчанию (на self-hosted runner'е Katana apt-cacher настраивается локально через setup_buildx_with_apt_cache.sh). Issue #1041"
- Revert "revert(06.08): откатить QEMU segfault workaround в Dockerfile.rtabmap — apt-get retry/--no-install-recommends/ldconfig не помогли (QEMU крашится на libc-bin post-install в pdal). Возвращаем оригинал. Issue #1041"
- Revert "feat(rtabmap): add static TF publisher for RGB optical-frame alias"

### Fixes (212 коммитов)

**Voice (40):**
- fix(llm): update LLM provider and model for improved performance
- fix(voice): speak fallback when LLM returns empty response
- fix(voice): add tool_choice=required for MiMo compatibility
- fix(llm): increase HTTP timeout 30→120s for reasoning model
- fix(llm): add traceback logging to LLM error handlers
- fix(llm): increase max_tokens 4096→8192 for reasoning models
- fix(llm): switch from deprecated deepseek-chat to deepseek-v4-flash
- fix(llm): switch from deprecated deepseek-chat to deepseek-v4-flash
- fix(voice): stop music after rap/poem spoken-word performance
- fix(voice): add flat ROS params for FAQ mode override
- fix(voice): fix FAQStore migrations path + add handle_faq to compositor routing
- fix(voice): strengthen imperial march playback
- fix(voice): use file synthdefs for custom wrappers
- fix(voice): retry renardo initialization
- fix(voice): ban unsafe sample letters
- fix(voice): preload pianovel at startup
- fix(voice): harden foxdot startup and prompts
- fix(voice): correct synth preload gating
- fix(voice): template foxdot synth path at startup
- fix(voice): repair foxdot runtime loading
- fix(voice): unify music prompt architecture
- fix(voice): make added vocals subtle and local in DJ mode
- fix(voice): expand Imperial March prompt guidance
- fix(voice): DJ theme passed to handle_music + reduce START plan to 5-8 tracks
- fix(voice): default DJ name is 'ДиДжей РОббокс' when no persona set
- fix(voice): save_dj_theme() restarts DJ mode immediately + play music now
- fix(voice): auto-stop DJ mode when transition count exceeds plan length
- fix(voice): correct FoxDot docs - bass/saw exist, scale= in >> works; add effects palette, chords, advanced patterns
- fix(voice): forbid bass/saw synths, fix scale= in >> calls, fix [xx] sub-patterns with short dur
- fix(voice): force handle_music for DJ mode, forbid direct speak_text without music
- fix(voice): trigger DJ mode when user gives persona+theme, not just explicit party request
- fix(voice): fix search_samples path /renardo_samples → /root/.config/renardo/samples
- fix(voice/dj): route all DJ transitions through MusicSkill (handle_music)
- fix(voice/music): freeAll SC nodes before Clock.clear() to prevent overflow
- fix(voice/dj): expand synth/scale/sample palette diversity
- fix(voice): add ros-humble-rtabmap-msgs to voice-assistant image
- fix(voice): strip [выполнено через: ...] prefix in speak_text before TTS
- fix(voice): compositor nav+music compound request orchestration
- fix(voice): revert MusicSkill max_tokens 500→800
- fix(voice): fix slow music inference (45-60s → ~15-20s)
- fix(voice): show LLM wait_for timeout in startup log
- fix(voice): truncate history outputs + increase llm timeout + music short summary
- fix(voice): use SDK to_input_list for conversation history
- fix(voice): force tool calls on every waypoint request — anti-pattern-completion rule
- fix(voice): add save/delete verification loop to NavigationSkill prompt
- fix(voice): prohibit list_waypoints before save_waypoint in NavigationSkill prompt
- fix(voice): force NavigationSkill to call tools (tool_choice=required)
- fix(voice): improve anti-hallucination guard in NavigationSkill
- fix(voice): add anti-hallucination guard to NavigationSkill

**Tests (24):**
- fix(test): remove trailing space from empty_message test — пробел не фильтруется
- fix(test): remove mem_limit, causes container startup failure on build machine
- fix(test): restore wake_words list, empty string broke empty_message filter
- fix(test): add SCENARIO_PATTERN to limit CI runs, prevent OOM
- fix(test): increase OVERALL_TIMEOUT to 300s, speed up listening rescue 20→10s
- fix(test): add 'робок' wake word prefix to all scenario STT texts
- fix(test): switch to deepseek-v4-flash model in proxy and voice-test
- fix(test): enable MCP tools so dialogue_node can call speak_text
- fix(test): mount scenarios and results via /tmp/ like config
- fix(test): mount ./config to scenario-runner (needs zenoh session config)
- fix(test): use pgrep for healthcheck, add full container log dump on failure
- fix(test): use dot instead of source in healthcheck (dash compat)
- fix(test): disable Zenoh shared memory in test session config
- fix(test): remove wake_words param with empty string value
- fix(test): source ROS 2 setup in voice-test healthcheck (ros2 not in PATH)
- fix(test): bypass ROS 2 Humble --params-file bug, use -p params instead
- fix(test): use local Ollama instead of DeepSeek API for voice-test
- fix(test): use ollama list instead of curl for healthcheck (curl not in ollama image)
- fix(test): increase ollama healthcheck timeout to 20min for first model pull
- fix(test): use zenoh port 17447 to avoid conflict with production (7447)
- fix(test): use zenoh REST port 18001 — 8000/8001 both occupied on host
- fix(test): use zenoh REST port 8001 to avoid conflict with production (8000)
- fix(test): mount whole config dir instead of single file for zenoh
- fix(test): use ollama port 11435 to avoid conflict with production + cleanup zenoh artifact dir

**RTABMAP / Mapping (18+5):**
- fix(mapping): improve error handling in StartMappingTool for service readiness and timeouts
- fix(mapping): make optimize_map execute real rtabmap services
- fix(rtabmap): relax localization links
- fix(rtabmap): use presynced rgbd stream
- fix(rtabmap): disable optimize-from-graph-end in configs
- fix(rtabmap): stop reanchoring map on startup
- fix(rtabmap): add depth frame tf alias
- fix(mapping): narrow start_mapping trigger — don't fire on roleplay/location mentions
- fix(rtabmap): always start in localization mode, remove state file dependency
- fix(mapping): set_mode_mapping BEFORE LoadDatabase to prevent rtabmap crash
- fix(mapping): use database_path instead of path in LoadDatabase.Request
- fix(rtabmap): reduce MaxNodes 200→100 to prevent Zenoh transport overload
- fix(rtabmap): add cloud_output_voxels=0.1 to prevent Zenoh transport overflow
- fix(rtabmap): switch optimizer GTSAM→g2o, robust=true
- fix(rtabmap): remove --delete_db_on_start — preserve map between restarts
- fix(rtabmap): add explicit camera_info_topic for rs_compat
- fix(rtabmap): switch to compressed transport to reduce Zenoh eth0 bandwidth
- fix(rtabmap): patch rtabmap.launch.py at startup to inject Grid/Sensor=2
- fix(rtabmap): update YAML to use /rtabmap/rtabmap: full node path
- fix(rtabmap): add --ros-args --params-file to override Grid/Sensor via ROS params
- fix(rtabmap): replace broken args:= with cfg:= INI file for Grid/Sensor=2
- fix(rtabmap): fix static_transform_publisher syntax for ROS 2 Humble
- fix(rtabmap): add static TF alias camera_rgb_camera_optical_frame→camera_color_optical_frame
- fix(rtabmap-sync): update topic remapping to /camera/camera/ double namespace

**Docker / CI (13+12):**
- fix(ci): use ros2 user's PulseAudio socket (uid 1000)
- fix(ci): runs-on: self-hosted (match existing runner labels)
- fix(ci): handle Docker daemon unavailable on self-hosted runner
- fix(ci): only remove zenoh config dir artifact, not the actual file
- fix(ci): escape dot in sed patterns — BRE . matches any char
- fix(ci): copy config to /tmp/ so Docker daemon on host can see it
- fix(ci): remove duplicate led_matrix_driver, update lint paths to ros2leds
- fix(docker): revert python 3.14 to 3.11 in voice_resources and mock_llm
- fix(docker): restore ubuntu 24.04 in supercollider (not a ROS package, no Humble constraint)
- fix(docker): revert ubuntu from 24.04 to 22.04 in supercollider (ROS2 Humble requires 22.04)
- fix(docker): add --retries 5 --timeout 120 to pip upgrade step
- fix(docker): add --retries 5 --timeout 120 to all pip install commands
- fix(docker): add pip --retries 5 --timeout 120 to prevent PyPI network failures
- fix(docker): split pip upgrade into separate layer before COPY requirements
- fix(docker): split pip upgrade into separate layer before COPY requirements
- fix(ci): configure dependabot for github-actions + pip + docker ecosystems
- fix(ci): narrow MSDO scanning to trivy,checkov,bandit for Docker/IaC+Python
- fix(docker): remove --symlink-install for production image to avoid editable mode issues
- fix(docker): inject additional parameters for improved localization and optimization
- fix(docker): run vision zenoh router with sh
- fix(docker): update paths in Dockerfile and .dockerignore for voice resources
- fix(docker): align main runtime conventions
- fix(ci): git pull --rebase before push to handle parallel jobs race
- fix(ci): add contents: write permission to build workflows
- fix(ci): add SHA tagging + .image-versions update to Vision/Main build workflows

**Telegram / Oak-D (7+7):**
- fix(telegram): use set_reaction instead of react for 👀 emoji
- fix(telegram): NoneType tool_calls, eyes reaction, message debounce
- fix(telegram): preserve newlines in /repl multiline code
- fix(telegram): fix compressedDepth header skip — find PNG sig instead of fixed 4 bytes
- fix(telegram): add Pillow + numpy to image deps for photo_depth/photo_map
- fix(telegram): fix camera topic rs_compat rename + add photo_depth + photo_map
- fix(oak-d): rename left→infra1, right→infra2 for rs_compat
- fix(oak-d): rename rgb→color, stereo→depth for rs_compat parameter sections
- fix(oak-d): correct resolution to 640x400 with i_output_isp: false
- fix(oak-d): i_rs_compat renames rgb→color, update all remappings
- fix(oak-d): fix param names rs_compat→i_rs_compat, color→rgb, depth→stereo sections
- fix(oak-d): fix namespace mismatch + clean config yaml + tune sync params
- fix(telegram): add retry loop on polling crash (Timed out recovery)
- fix(oak-d): migrate to depthai_ros_driver_v3 for arm64 support

**Docs (6):**
- fix(docs): task 01-02-03 — ROADMAP.md add telegram-bot/MCP-tools/teleop, mark voice+internal as implemented
- fix(docs): task 01-02-02 — SYSTEM_OVERVIEW.md full service tables (12+11), apriltag note, monitoring separate machine
- fix(docs): tasks 01-01-01 to 01-01-04 — fix ubuntu@/ssh/paths/service names + SOFTWARE.md new packages
- fix(docs): task 01-01-08 — README.md add telegram-bot/MCP-tools, fix Pi specs, fix SLAM description
- fix(docs): task 01-01-07 — DOCUMENTATION_STRUCTURE.md add all src packages table
- fix(docs): task 01-01-05 — HARDWARE.md LED count + joystick BLE kernel bug note fix(docs): task 01-01-06 — TROUBLESHOOTING.md Zenoh/VESC/LiDAR/Perception cases

**Прочие:**
- fix(L-Build All & Push to GHCR): убрать docker buildx imagetools push — удалён в buildx 0.21+. Теперь imagetools create сам пушит когда target указывает на registry
- fix(L-Build All & Push to GHCR): использовать secrets.CR_PAT вместо GITHUB_TOKEN — GITHUB_TOKEN не имеет прав push в rob_box_base (403 Forbidden), CR_PAT с packages:write работает
- fix(L-Build All & Push to GHCR): base-images пушим в ghcr.io/krikz/rob_box_base (отдельный package), main+vision в ghcr.io/krikz/rob_box — иначе 403 Forbidden на push в чужой package
- fix(L-Build All & Push to GHCR): push_to_registry boolean вместо string — startup_failure из-за несоответствия типа input (referenced workflow ожидает boolean, передавал string 'true')
- fix(L-Build Vision Pi for Deploy): jq -sc (compact output) для GITHUB_OUTPUT — GH парсер требует однострочный JSON, иначе 'Invalid format'
- fix(06.08 G-build): build-voice-resources context . → docker/vision — Dockerfile копирует voice_assistant/download_samples.py, который лежит в docker/vision/, не в корне. Issue #1041
- fix(06.08 G-build): убрать хардкод http://10.1.1.5:3142 — GitHub Actions runner изолирован от локальной сети. APT_PROXY теперь пустая строка по умолчанию (на self-hosted runner'е Katana apt-cacher настраивается локально через setup_buildx_with_apt_cache.sh). Issue #1041
- fix(06.08 G-build): объединить два build-args блока в один (BASE_IMAGE + APT_PROXY) — GH parser ругался на duplicate
- fix(06.08 G-build): параметризовать APT_PROXY build-arg для всех 4 base-images — Dockerfile.rtabmap:10 ждёт APT_PROXY для apt-cacher. Раньше G-build передавал пустую строку → apt-get напрямую → медленнее/нестабильно. Теперь G-build использует тот же путь что и Katana local build (APT_PROXY=http://10.1.1.5:3142). Issue #1041
- fix(06.08): (1) G-build rtabmap QEMU segfault workaround (apt retry + --no-install-recommends + ldconfig); (2) L-build DOCKER_TAG handling для refs/tags/* (теперь latest вместо local → .image-versions.latest обновляется). Issue #1041
- fix(runner): increase dialogue_node wait to 120s, catch RCLError gracefully
- fix(dialogue): implement history_excluded_tools filtering + memory limits
- fix(mimo): disable thinking mode for reliable tool calls
- fix(dialogue): change tool_choice to "auto" for MiMo compatibility
- fix(03.1): resolve plan-checker blockers — mark Open Questions RESOLVED, fix truth inflation, add VALIDATION.md
- fix(config): move model/provider to flat params in voice_assistant.yaml
- fix(compose): revert supercollider tag to match build format
- fix(config): disable event mode (open day 2026) on robot
- fix(deploy): stop sourcing .image-versions for local registry; fix supercollider compose
- fix(compose): use IMAGE_TAG as default instead of hardcoded latest
- fix(build): fix 3 failing Docker builds on ARM64 cross-compile
- fix(04-plan): исправлен порядок подпланов — сначала скиллы/инструкции, потом миграция данных
- fix(packages): remove TODO from setup.py/package.xml, fix emails, add COLCON_IGNORE
- fix(supercollider): update JACK configuration to use nperiods=3 for improved audio handling
- fix(dialogue): update anti-escalation rules and hardware constraints for music playback
- fix(dialogue): clear conversation history after each FAQ event mode turn
- fix(faq): strip Russian stopwords and add morphological prefix in FTS search
- fix(prompt): lengthen imperial march lead
- fix(prompt): clamp music continuation patterns
- fix(apriltag): correct tag size in configuration for accuracy
- fix(vision): align apriltag camera pipeline
- fix(nav2): simplify local obstacles and reduce inflation
- fix(supercollider): clean up additional JACK SHM files before starting
- fix(control): align diff drive odometry mode
- fix(start_zenoh): improve cleanup function and update user instructions
- fix(nav2_params): update robot_base_frame to base_footprint for consistency across configurations
- fix(nav2_params): update observation sources to include cloud obstacles for improved navigation
- fix(deploy): filter startup log false positives
- fix(deploy): reduce false positive log issues
- fix(zenoh): add costmap drop qos to prevent nav2 transport closure
- fix(dj): no-pause transitions + synth rotation anti-repeat
- fix(dj): add anti-escalation rules to prevent runaway amp/dur progression
- fix(dj): preserve party theme across auto-transitions
- fix(voice/dj): route all DJ transitions through MusicSkill (handle_music)
- fix(voice/music): freeAll SC nodes before Clock.clear() to prevent overflow
- fix(voice/dj): expand synth/scale/sample palette diversity
- fix(dj): prevent cacophony by adding Clock.clear() + layer/BPM limits
- fix(dj): make set_dj_mode mandatory in music skill prompt
- fix(music): add track library tools to MusicSkill (USE_SKILLS mode)
- fix(music): remove inline DDL/bootstrap from Python, mount migrations/ in Docker
- fix(music): wire track library tools to LLM + fix Docker bootstrap
- fix(zenoh): prevent transport closure on large rtabmap publishes
- fix(zenoh): cover all rtabmap/** topics with congestion_control=drop
- fix(teleop): reduce max_angular_speed from 5.0 to 1.0 rad/s for improved control
- fix(zenoh): set congestion_control=drop for rtabmap cloud topics
- fix(perception): filter guess_from_tf noise + startup grace period in health_monitor
- fix(dialogue): increase max_tokens from 800 to 2000 to prevent JSON truncation
- fix(vision): fix rtabmap-sync relative remaps - remove leading / to match namespace
- fix(mcp): auto-infer new_location from map_name, remove required=True
- fix(perception): remove duplicate startup greeting from reflection_node
- fix(mcp): make new_location required in start_mapping to force explicit LLM choice
- fix(mcp): fix Empty NameError in start_mapping and improve new_location hints
- fix(mcp): lazy import rtabmap_msgs — Vision Pi doesn't have rtabmap package
- fix(perception): fix DeepSeek 400 error and health status case mismatch
- fix(joystick): correct axis_angular_z mapping and adjust max_angular_speed
- fix(navigation): remove as_tool mess; navigation never times out in Telegram
- fix(joystick): correct axis_angular_z mapping for Yaw control
- fix(nav2): increase transform_tolerance to 1.0 + tf_buffer_duration 30s — fix LiDAR TF drops in costmap
- fix(nav2): inflation_radius 0.30→0.55 — must be >= inscribed_radius 0.451
- fix(rviz): enable Use rainbow option in Visualization Manager
- fix(ceiling-camera): reduce fps 10→5, res 1280x720→640x480
- fix(rtabmap-sync): update topic remapping to /camera/camera/ double namespace
- fix(depthai): pin to v2.12.2-humble instead of broken humble-latest
- fix(depthai): use humble-arm64-latest for Raspberry Pi arm64 support

### Features (71 коммитов)

- feat(L-Build All & Push to GHCR): полная сборка всех образов локально на Katana + push в ghcr.io с SHA-тегами. Триггеры: push в main, workflow_dispatch, schedule 0 0 * * *. Стиль как у L-Build All Services — собирает base (rtabmap/ros2-zenoh/depthai/pcl) + main (9 сервисов) + vision (7 сервисов), потом пушит в ghcr.io через docker buildx imagetools и обновляет .image-versions.latest
- feat(L-Build Vision Pi for Deploy): новый workflow для сборки Vision Pi сервисов на self-hosted Katana (native arm64, без QEMU). Триггеры: push в main (с path filter на docker/vision, src), workflow_dispatch (ручной), schedule 0 0 * * * (полночь UTC). Логика: проверяет наличие SHA-тега в ghcr.io через docker buildx imagetools inspect — собирает только отсутствующие. После сборки обновляет docker/vision/.image-versions.latest (как L-Build Vision Pi Services). Готово для последующего деплоя через L-Deploy and Verify.yml --ref main.
- feat(test): DeepSeek proxy via LiteLLM for integration tests
- feat(voice): switch LLM provider from deepseek to mimo-v2.5-pro
- feat(llm): replace Qwen with MiMo (Xiaomi) as LLM provider
- feat(voice): P0 error handling — APITimeoutError/RateLimitError/APIStatusError + fallback model
- feat(03.1): execute research phase — validate RESEARCH.md, pin openai versions, update ROADMAP
- feat(llm): make model configurable via ROS params for telegram + voice
- feat(workflow): add 'local' environment option and update IP addresses for vision and main Pi
- feat(04): migrate tasks.json → GitHub Issues; remove migration artifacts
- feat(github): create label taxonomy (17 labels) and milestones (M1/M2/M3)
- feat(gsd): migrate skills and instructions to GitHub Issues workflow
- feat(04-plan): Phase 4 GitHub Issues Integration — CONTEXT.md + PLAN.md (3 sub-plans)
- feat(docs): plan 01-03 — package READMEs audit and update
- feat(guidelines): add Karpathy guidelines for coding best practices
- feat(docker-expert): add comprehensive Docker patterns and review guidelines for optimization, security, and performance
- feat(skills): enhance RLM context rules for various skills to optimize document usage and retrieval
- feat(skills): refine context rules and task instructions across multiple SKILL documents
- feat(documentation): update context strategies in planning and implementation documents
- feat(dialogue): add event FAQ prefetching and related instructions for performance requests
- feat(requirements): add openpyxl to requirements.txt for XLSX FAQ support
- feat(voice): add event_mode.yaml to .gitignore
- feat(voice): add volume control parameter to TTS node configuration
- feat(voice): add README for event FAQ file configuration
- feat(voice): enable FAQ mode in dialogue configuration
- feat(voice): add faq event mode
- feat(docs): add FoxDot runtime repair design and implementation plans
- feat(voice): refine stranger things arrangement
- feat(voice): add imperial march sc-only palette
- feat(voice): add sc-only retro synth palette
- feat(camera): add compatibility aliases for depthai_ros_driver color frame naming
- feat(rtabmap): add static TF publisher for RGB optical-frame alias
- feat(tests): update tests for lidar-only mode and remove camera dependencies
- feat(rtabmap): update parameters for LiDAR-only mode and remove unused TF publishers
- feat(nav2): optimize controller parameters for smoother navigation and improved trajectory handling
- feat(mcp): add recommended executor thread count for containerized runtime
- feat(docker): optimize RTAB-Map configuration for stable LiDAR SLAM
- feat(music): enhance music tool registration with error handling
- feat(mapping): optimize service client creation with callback groups
- feat(mapping): enhance FinishMappingTool with publish_map functionality and timeout handling
- feat(telegram): add repl music commands
- feat(deploy): add cleanup steps for unused Docker artifacts on Vision and Main Pi
- feat(nav2_params): optimize local costmap configuration for improved obstacle detection
- feat(download): enhance sample pack downloading with improved error handling and concurrency
- feat(deploy): deduplicate auto-generated issues
- feat(docker): optimize vision build and layout
- feat(voice): add DJ persona memory via save_dj_persona() tool
- feat(music): real instrument → Renardo synth mapping
- feat(dj): agent-planned set + live plan editing
- feat(dj): theme-adaptive music parameters
- feat(music): add 3 new DJ examples from live coding session (D/E/F)
- feat(music): add Stranger Things theme + Acid Techno recipe to music prompt
- feat(music): add pro production techniques from live coder analysis
- feat(dj): DJ ROB-BOX persona, set progression, MC crowd engagement
- feat(dj): add party theme support to DJ mode
- feat(music): enhance DJ mode with next_transition_sec parameter for smoother transitions
- feat(music): add SetDjModeTool and integrate DJ mode functionality
- feat(music): add TrackLibrary + 4 MCP tools for music media library
- feat(perception): expose mapping_mode in PerceptionEvent
- feat(mapping): implement mapping lifecycle FSM
- feat(mapping): enhance OptimizeMapTool with service warmup and timeout handling
- feat(navigation): add optimize_map tool and update mapping scenario instructions
- feat(voice): log history_excluded_tools at startup
- feat(voice): generic history exclusion per skill-tool via config
- feat(voice): translate all active prompts to English + skill-creator improvements
- feat(skills): add python-expert skill
- feat(ci): per-service SHA-pinned image tags
- feat(skills): add senior-devops skill (CI/CD, IaC, containers, cloud)
- feat(rtabmap): fix DB growth, add OptimizeMapTool + LoadMapTool
- feat(rviz): add RobotModel, GlobalPlan, LocalPlan, GoalPose, GlobalCostmap, LocalCostmap
- feat(rtabmap): add OAK-D RGB-D integration for 3D SLAM

### Performance (6 коммитов)

- perf(oak-d): reduce camera fps 15 -> 10 to lower rtabmap CPU and network load
- perf(camera): raise OAK-D + ceiling camera from 5fps to 15fps
- perf(rtabmap): limit WM to 500 nodes, BinDataKept=false, Kp/MaxFeatures=200 — 200x speedup
- perf(oak-d): reduce resolution 640x360 + fps 5 to cut relay bandwidth 10x
- perf(oak-d): enable H.264 on-chip encoding + lazy publishers
- perf(oak-d): reduce camera FPS 15→10 to lower network load

### Refactoring (6 коммитов)

- refactor(L-Build All & Push to GHCR): переделать на 3 параллельных workflow_call (base + main + vision) как в L-Build All Services, потом push в ghcr.io. Раньше был 1 job с последовательной сборкой всех 20 сервисов — медленно. Теперь параллельно (до 8 одновременно) на self-hosted Katana.
- refactor(test): per-scenario timeout_s instead of global OVERALL_TIMEOUT
- refactor(deploy): reorganize Docker cleanup steps for Vision and Main Pi
- refactor(docker): normalize main rtabmap startup path
- refactor(music): TrackLibrary JSON→SQLite, bootstrap track in migration
- refactor(mapping): remove file persistence from MappingState, pure in-memory FSM

### Build / Dependencies (17 коммитов)

- build(deps): bump the python-deps group across 5 directories with 8 updates
- build(deps): bump the github-actions group across 1 directory with 2 updates
- build(deps): bump torch from 2.12.0 to 2.13.0 in /src/rob_box_voice
- build(deps): bump torch
- build(deps): bump the python-deps group across 6 directories with 10 updates
- build(deps): bump the python-deps group across 5 directories with 9 updates
- build(deps): bump the python-deps group across 6 directories with 10 updates
- build(deps): bump the github-actions group across 1 directory with 2 updates
- build(deps): bump the python-deps group across 5 directories with 9 updates
- build(deps): bump actions/checkout in the github-actions group
- build(deps): bump the python-deps group across 5 directories with 7 updates
- build(deps): bump the python-deps group across 6 directories with 23 updates
- build(deps): bump ubuntu
- build(deps): bump actions/cache in the github-actions group
- build(deps): bump the docker-base-images group across 3 directories with 2 updates
- build(deps): bump the python-deps group across 3 directories with 4 updates
- build(deps): bump the github-actions group with 11 updates

### Documentation (41 коммитов)

- docs(changelog): v1.0.0-1-bfe71bf-humble-rc1 — first release candidate from main (stable, post-ДОД baseline)
- docs(03.1.1): execution SUMMARY + advance to Phase 4
- docs(03.1.1): create phase plan — P0 error handling + OPENAI_LOG
- docs(03.1.1): capture phase context — apply P0 error handling from 03.1 research
- docs(03.1): add v1→v2 migration impact analysis — feature-by-feature with Rob Box action items (P0/P1/P2)
- docs(state): Phase 03.1 complete — advance to Phase 4
- docs(03.1): add execution SUMMARY — Phase 03.1 complete
- docs(03.1): create phase plan — validate research, pin versions, document decision
- docs(research): Phase 03.1 - OpenAI Agents SDK vs Anthropic Claude SDK agentic features comparison
- docs(state): record phase 03.1 context session
- docs(03.1): capture phase context
- docs(04): add research and 3 execution plans for phase 4
- docs(04): create phase plans 04-01, 04-02, 04-03
- docs(04): research phase - GitHub Issues integration
- docs(04): capture phase 4 context via discuss-phase
- docs(03-complete): Phase 3 SUMMARY.md, STATE.md complete — all 7 tasks done
- docs(03-05b): tasks.json — add TASK-051 and TASK-052 (stub implementation tasks)
- docs(03-05a): add # STUB: markers to system.py and command_node.py (4 locations)
- docs(03-04): DIALOGUE_NODE_REFACTORING.md — decomposition strategy 6 components, 2040→300 lines target
- docs(03-03): COVERAGE_REPORT.md — 14/19 modules below 50%, 6 modules at 0%
- docs(03-02): STATIC_ANALYSIS_REPORT.md — flake8 4288 violations, black 159 files, isort 136 files
- docs(03-01): TECH_DEBT.md — 30 concerns with severity/disposition; tasks.json — add BG-5 (TASK-049) and BG-6 (TASK-050)
- docs(03): fix plan — task IDs, 74 methods, dependency warning (checker pass)
- docs(03): create phase 3 plan — code quality review (CQ-01..CQ-06)
- docs(phase-3): research + validation strategy for Phase 3
- ... и ещё 16 коммитов с доками

### Chore (9 коммитов)

- chore(workflows): удалить L-Build Vision Pi Services for Deploy — дублировал L-Build All & Push to GHCR (тот же функционал, но покрывает все 20 сервисов вместо только vision)
- chore(workflows): удалить G-Build All Services.yml — G-build на GitHub-hosted с QEMU регулярно падает (cache flake, ldconfig segfault, rcutils not found). Вся сборка теперь идёт через L-Build (self-hosted Katana, native arm64). G-Auto-merge to Main обновлён — использует L-Build All Services вместо G. README обновлён.
- chore(state): phase 4 COMPLETE — Milestone 1 fully executed
- chore(state): phase 4 plan-phase complete — ready to execute
- chore(state): update STATE.md — phase 4 discuss complete, ready for plan-phase
- chore(planning): mark Phase 1 complete in STATE.md
- chore(repo): sync current workspace changes
- chore(git): ignore local worktrees
- chore(skills): merge skills from develop branch

### Tests (2 коммитов)

- test(supercollider): cover JACK shm cleanup
- test(deploy): cover docker cleanup workflow

## [Unreleased]

### MiniMax TTS-провайдер

#### Добавлено

* `examples/tts_minimax_example.py` — end-to-end пример получения провайдера
  через registry/factory, синтеза PCM и записи валидного mono WAV модулем
  `wave`. Конфигурация читается из ENV, включая custom base URL и локальный
  лимит конкурентности.
* API reference и Getting Started обновлены для bytes API, поддерживаемых
  голосов и форматов `pcm_22050`, `pcm_24000`, `wav`; добавлены инструкции по
  запуску примера, проверке WAV и ссылка на ADR-0001.

### [PR #907] — MiniMax LLM-интеграция в `rob_box_llm` (text + tools + vision)

> Ветка `feature/harness-p0-foundation` → `develop`. Один feature branch,
> внутри несколько фаз (M0 + M1 + M4, см. `architecture/minimax-provider.md`).
> TTS и image generation намеренно не входят — это отдельные адаптеры
> (TTS-фаза уже смержена через PR #907/ADR-0007).

#### Добавлено

* **`MiniMaxProvider`** — OpenAI-compatible адаптер существующего
  `LLMProvider` (`MiniMax-M3`, `https://api.minimax.io/v1`). Наследуется
  от общего `_OpenAICompatibleProvider` и переиспользует маппинг SDK
  исключений на типизированный `errors.ProviderError`.
* **Мультимодальный `LLMMessage.content`**: backward-compatible расширение
  до `str | tuple[MessagePart, ...]`. Новые value objects
  `TextPart`/`ImagePart` сериализуются в OpenAI `image_url` content
  blocks (URL pass-through / bytes → base64-data-URL).
* **`ProviderCapabilities`** + `capabilities_for(model)` —
  capability introspection для безопасного fallback и fail-fast gate
  до сетевого вызова. `image_input` сужается до vision-capable моделей
  (`*M3*`, `*M2-vision*`, `*vision*`).
* **`MINIMAX_MAX_IMAGE_BYTES = 10 MB`** — инженерный default для image
  payload; единая точка правки, юнит-тесты на превышение лимита.
* **`MiniMaxRedactedLogFilter`** — utility для гарантированного
  вычёркивания `MINIMAX_API_KEY` из log records.
* **Маппинг `base_resp.status_code`** — HTTP 200 c прикладной ошибкой
  MiniMax превращается в `AuthError`/`RateLimitError`/`ContentFilterError`/
  `ProviderError` через общий `_post_process_response` hook.
* **`MiniMaxProvider.thinking` (per-call override)** — default
  `{type: disabled}` (latency-sensitive); переопределяется через
  `settings.extra` для agent mode.
* **35 новых unit-тестов** (`test_minimax_provider.py`): fake SDK,
  text/tool/error/thinking/image-validation, vision off для не-vision моделей.
  85 зелёных в `rob_box_llm` итого.
* **Документация:**
  * `docs/guides/MINIMAX.md` — пользовательский гайд по text+vision
    провайдеру (API key, env, factory YAML, capabilities, troubleshooting).
  * `docs/guides/examples/minimax_llm.yaml` — копируемый шаблон
    `llm.providers` для registry/factory.
  * `src/rob_box_llm/README.md` — обновлён: добавлена таблица
    text+vision провайдеров с явными `name` / `base_url` / capabilities.
  * `.env.example` — секция `LLM ПРОВАЙДЕРЫ` (отдельно от TTS-блока).
  * `architecture/minimax-provider.md` — обзорный проектный документ.
  * `docs/adr/0002-minimax-provider.md` — capability-segregated ADR.

#### Изменено

* `src/rob_box_llm/rob_box_llm/__init__.py` — публичные экспорты
  `MiniMaxProvider`, `TextPart`, `ImagePart`, `MessagePart`,
  `MessageContent`, `ProviderCapabilities` (semver-minor: 0.1.0 → 0.2.1).
* `src/rob_box_llm/rob_box_llm/errors.py` — добавлены
  `CapabilityUnavailableError` и обновлён docstring иерархии.

#### Не входит в PR #907

* Реестр провайдеров / factory / fallback decorator — фаза M2, отдельная
  Kanban-задача (после PR #907).
* Миграция `DialogueNode` и Telegram `LLMChat` на новый `LLMProvider` —
  фаза M3, отдельная Kanban-задача. Текущие legacy-пути сохранены.
* Image generation через MiniMax — отложено (YAGNI), до подтверждённого
  consumer (потенциально Telegram media tool).

## [Март 2026] — PR #572: Integrate MCP tools, enhance documentation, and improve test coverage

> Ветка `feature/agent-skills` → `develop` | 566 коммитов | +68840 / -2670 строк

### 🎉 Добавлено

#### Voice Assistant — переписан на OpenAI Agents SDK
- **Полный рефакторинг `dialogue_node`** на OpenAI Agents SDK: Compositor + 4 суб-агента (Navigation, Memory, Music, Status)
- **Compositor паттерн** — главный агент делегирует задачи специализированным скилам через `FunctionTool`
- **Streaming + barge-in** — `run_streamed()` с мгновенным прерыванием через VAD-прерывание
- **Generation guard** — защита от stale потоков, публикующих TTS в новый диалог
- **Параметр `verbose_llm`** — полное логирование входа/выхода LLM через ROS 2 параметр
- **`get_current_time` MCP tool** — время через инструмент вместо инъекции в промт (KV cache)
- **Sentence splitting** — Yandex TTS: длинные тексты режутся по предложениям и озвучиваются чанками
- **SSML-only** — унификация формата ответа TTS, убран plain text
- **Бэрдж-ин grace период** — 5с после STT для предотвращения отклика на комнатное эхо
- **Многослойная музыкальная стратегия** — instant drums первым, добавление слоёв чанками

#### Telegram Bot (новый сервис)
- **`rob_box_telegram`** — операторский интерфейс через Telegram: команды, фото с потолочной камеры, TTS через бота
- CI/CD: telegram-bot добавлен во все Vision Pi workflows (build, deploy, single-service)
- Docker: Dockerfile без `--symlink-install`, корректный `chmod +x` стартового скрипта

#### Навигация
- **Динамический CRUD вейпоинтов** через SQLite — добавление/удаление/просмотр точек на лету
- **Go-speak-return миссии** — робот едет к вейпоинту, произносит фразу, возвращается
- **Fail-fast на ошибки Nav2** — голосовое сообщение об ошибке вместо молчания
- **Anti-hallucination tool markers** + защита от ложных навигационных команд в промте
- **Замена `rclpy.spin_until_future_complete` → `threading.Event`** в NavigationSkill (устранение дедлока)
- **`ReentrantCallbackGroup`** для Nav2 action client — устранение дедлока

#### Музыка (Renardo + SuperCollider)
- **Renardo MCP tool** — real-time генерация музыки через FoxDot/SuperCollider
- **SuperCollider в Docker Compose** — отдельный сервис с JACK/ALSA dmix
- **`music_max_amp` параметр** — ограничение амплитуды в LLM-генерируемом коде
- **Загрузка сэмплов Renardo** в Dockerfile (pitchglitch vocal)
- **Трёхшаговый стоп музыки** — корректная очистка SuperCollider synths
- **Гайдлайны Imperial March, Christmas Tree, waltz** — точные MIDI ноты и BPM
- **Monkey-patch для `spack`** в `getBufferFromSymbol`

#### Калибровка моторов
- **Серия калибровок `wheel_separation`**: 0.39 → 0.81 → 0.97 → 1.05 → 1.09 → 1.11 (на основе тестов вращения на 360°)
- **Калибровки `gear_ratio`**: 2.3 → 2.16 → 2.26 → 2.17
- **Circular footprint** для Nav2 (skid-steer вращение), `robot_radius` 0.35 → 0.45
- **`max_rps` калибровка**: 6.5 → 10 → 12.2 (через `move_test.py`)
- Документация: процедуры тестирования и калибровки моторов Rob Box

#### Телеуправление
- **Миграция BLE → SBUS serial protocol** для ExpressLRS джойстика
- **Авто-определение порта SBUS** при USB-реподключении
- Кнопки: остановка публикации нулей при дизарме (twist_mux timeout)

#### Тестирование
- **Docker-compose интеграционная тест-среда** для `dialogue_node` (scenario runner)
- **Mock MCP сервер** в scenario-runner с реальными инструментами
- **5 интеграционных сценариев**: barge-in A/B/C, rapid_messages, memory_context
- **`wait_for_idle()` через state topic** (замена `wait_for_quiet()` на sleep)
- **`voice_memory.db` фикстура** (85 turns, 1 fact) из SQL seed через python3
- **Переход с Ollama → DeepSeek API** в тестах
- **Self-hosted Pi runner** для нативных arm64 интеграционных тестов
- Юнит-тесты: `health_monitor` 19%→84%, `sound_node` 53%→85%, `led_node` 62%→90%, `reflection_node` 28%→75%, `context_aggregator` 43%→69%, `command_node` 48%→75%
- LED animation синхронизация и тесты (`led_node`)

#### Agent Skills & Документация
- `.agents/skills/context-engineering/SKILL.md` — методология Research→Design→Plan→Implement
- `.agents/skills/github-actions-runner/SKILL.md` — запуск CI/CD через UI и CLI
- `.agents/skills/docker-expert/SKILL.md` — Docker best practices, оптимизация, безопасность
- `.agents/skills/mcp-builder/SKILL.md` — создание MCP серверов (FastMCP/SDK)
- `.agents/skills/skill-creator/SKILL.md` — руководство по созданию skills
- `.agents/skills/motor-testing/SKILL.md` — тестирование моторов и одометрия
- `.agents/skills/zenoh-dev-setup/SKILL.md` — Zenoh для dev-машины
- `.agents/github-copilot/code-review-specialist/SKILL.md` — code review методология
- `.agents/github-copilot/python-testing-patterns/SKILL.md` — pytest best practices
- VS Code prompt files (`.github/copilot-instructions.md`, `.claude/commands/`) для Context Engineering
- ROADMAP.md + ROADMAP_SIMPLE.md — реализованные и планируемые фичи
- Zenoh dev-машина: setup документация + калибровочный скрипт

#### CI/CD
- Integration Tests workflow на self-hosted Pi runner (native arm64)
- `feat/*` ветки добавлены во все Single Service workflows
- SSH hardening в deploy workflow (предотвращение connection hangs)
- Cleanup stale `*.db` директорий перед checkout

### 🐛 Исправлено

#### Критические баги Voice Assistant
- **BUG-17** — silent retry при LLM timeout до порога счётчика ошибок
- **BUG-18** — `pending_queries` не обрабатываются после interrupt
- **BUG-19** — deadlock в `_recreate_llm_client` при вызове `client.close()` (py-spy диагноз)
- **BUG-20** — stale `_continue_after_tool_calls` озвучивает ответ в новый диалог
- **`Message.get()` bug** в non-streaming пути
- **`MaxTurnsExceeded`** обработка из OpenAI Agents SDK
- **Orphaned tool messages** — очистка истории от осиротевших tool_calls (HTTP 400)
- **Stale `pending_queries`** — очистка перед `listen_for_response`
- **`httpx.Client` пересоздание** после каждого LLM timeout
- **Параллельный `speak_text` стоп** при cancel run
- **Конкурентность `speak_text`** — `asyncio.Lock` для сериализации

#### MCP / Navigation
- **Nav2 action client deadlock** — `ReentrantCallbackGroup`
- **`self._node → self.node`** в mapping skill
- **Non-blocking service calls** — убран `spin_until_future_complete`
- **`set_vibe_preset`** — параметр `preset` → `preset_name` (MCP API)
- **`memory_save`** — параметр `content` → `fact`
- **`registry.execute`** — параметр `name` → `tool_name`
- **`GetCurrentTimeTool`** не был зарегистрирован в mcp_server
- **BEST_EFFORT QoS** для `/mcp/*` топиков (снижение задержки Zenoh)

#### Audio / SuperCollider
- **ALSA `dmix`** вместо `hw` device index — параллельное воспроизведение
- **JACK shm cleanup** — удаление stale SHM файлов при старте (предотвращение "default server already active")
- **jackd2 debconf hang** при установке supercollider-language
- **`shm_size: 256m`** для JACK Bus error
- **`jack_connect`** — подключение scsynth outputs к ALSA playback

#### Telegram
- **Ceiling camera topic** исправлен на `/ceiling_camera/image_raw/compressed`
- **f-strings для RcutilsLogger** (нет поддержки printf `%s`)
- **JSON с SSML** в tts_node вместо plain text

#### Конфигурация
- `docker-compose.yaml` — YAML multi-line python oneliners исправлены
- `wake_words: [""]` вместо `[]` — ROS 2 не может парсить пустой список

### ♻️ Рефакторинг
- **`_continue_after_tool_calls`**: рекурсия → итеративный while
- **`ConversationHistory`** вынесен в core слой
- **`VoiceCommandHandler`, `CommandParser`, `SpeechFormatter`, `DialogueManager`** — extraction в core
- **`ProviderManager`, `ToolCallExecutor`, `StreamingHandler`** — extraction в LLM layer
- **`MemoryManager`, `PromptFormatter`, `EventDetector`** — extraction в perception core
- **`ToolCallAccumulator`** вынесен в core layer
- `master_prompt_compact.txt` компактизация (555→128 линий, reverт до рабочей версии)
- Реорганизация `scripts/` и `tools/` в логическую структуру
- `.gitignore` — игнорировать `.agents/` skills директории

### 📚 Документация
- BUG-11, BUG-12..BUG-16 описания с анализом и fix-кодом
- BUG-18 — pending_queries hang after interrupt
- BUG-19 — deadlock с py-spy диагнозом
- Транскрипция видео для анализа
- AI HAT+ 26 TOPS анализ и актуализация ROADMAP
- CONTRIBUTING.md — таблица именования веток + SemVer правила

---

## [Февраль 2026]

### Добавлено
- PRD.md — Product Requirements Document с 34 задачами, milestones и acceptance criteria (19 февраля 2026)
- 11 специализированных AI-агентов в `docs/development/agents/`: navigation, backend, voice, frontend, devops, docs, git, security, scenarios, structure, **diagnostics** (19 февраля 2026)
- `docs/development/agents/diagnostics-agent.md` — агент удалённой диагностики: SSH-диагностика контейнеров, ROS 2 топиков, здоровья сервисов (19 февраля 2026)
- tasks.json — структурированный список задач с приоритетами и test_steps (19 февраля 2026)
- progress.md — лог выполнения задач агентами (19 февраля 2026)
- `docs/architecture/NETWORK_TOPOLOGY.md` — отдельный документ сетевой топологии (19 февраля 2026)
- SKILLS-001: добавлены `When to Apply` секции во все 12 agent guides (19 февраля 2026)
- SKILLS-002: оценка 4 пакетов skills.sh — ROS2 ниша пуста, внешние skills не установлены (19 февраля 2026)
- SKILLS-003: установлены 2 quality skills — `python-testing-patterns` и `code-review-specialist` (20 февраля 2026)

### Исправлено
- **TASK-048**: Double timeout hang (BUG-10 + BUG-15) — `_continue_after_tool_calls` переписан с рекурсии на итеративный while-цикл, устранено зависание 120с (20 февраля 2026)
- **TASK-035**: Серия фиксов стабильности agent mode (20 февраля 2026):
  - dmix `asound.conf` — параллельный TTS+sound через ALSA dmix
  - PlaySoundTool INSTANT — убран `time.sleep()`, fire-and-forget
  - ThreadPoolExecutor hang — `shutdown(wait=False)` в `_ask_llm_streaming` и `_continue_after_tool_calls`
  - interrupt_agent_loop — STT STOP прерывает цикл на итерации 2+
  - GetCurrentTimeTool — убрана инъекция времени в system_prompt (KV cache miss)
  - Убран preload past_turns — устранена "каша" с предыдущими сессиями
  - `conversation_history.clear()` при wake word из IDLE
  - TIME_CONTEXT_MARKER — исправлен поиск маркера для вставки времени
  - speak_text animation — добавлены псевдонимы neutral/excited/confused в enum

### Изменено
- Реорганизация документации: 10 файлов из корня `docs/` перемещены в соответствующие категории (27 февраля 2026)
- Архивировано 19 устаревших/дублирующих файлов документации (27 февраля 2026)
- Исправлены все ссылки на "Raspberry Pi 4" → "Raspberry Pi 5" (11 файлов) (27 февраля 2026)
- Консолидированы мелкие категории документации: `operations/` → `guides/`, `optimization/` → `development/` (27 февраля 2026)

## [Январь 2026]

### Добавлено
- `sound_catalog.json` — каталог из 51+ звуковых эффектов с метаданными (тип, теги, длительность)
- Загрузка звуков из `sound_catalog.json` в dialogue_node с логикой выбора по контексту (30 января 2026)
- `GetSoundInfoTool` — MCP инструмент для получения информации о доступных звуках (30 января 2026)
- Логирование использования токенов LLM API (документ: [TOKEN_USAGE_LOGGING.md](docs/fixes/TOKEN_USAGE_LOGGING.md), [TOKEN_USAGE_RU.md](docs/fixes/TOKEN_USAGE_RU.md)) (29 января 2026)
- MCP инструменты для провайдера DeepSeek (29 января 2026)

### Исправлено
- **Бесконечный цикл анимаций** — race condition в `llm_adapter.py`: Event теперь регистрируется ДО publish + добавлен лимит MAX_ITERATIONS=10 ([документ](docs/fixes/ANIMATION_LOOP_FIX.md)) (29 января 2026)
- **Синхронизация TTS и анимаций** — предотвращение смешивания анимаций между сеансами диалога ([документ](docs/fixes/ANIMATION_TTS_FIX.md)) (28 января 2026)
- **Scary story + системные звуки** — robot не молчит после "расскажу историю", устранено смешивание контекстов ([документ](docs/fixes/SCARY_STORY_FIX.md)) (29 января 2026)
- **QoS mismatch в deepseek_adapter** — несоответствие QoS настроек вызывало timeout ошибки ([документ](docs/fixes/QOS_MISMATCH_FIX.md)) (29 января 2026)
- **DeepSeek connection pool** — отключён httpx connection pooling для предотвращения idle timeout ([документ](docs/fixes/DEEPSEEK_CONNECTION_POOL_FIX.md)) (30 января 2026)
- **DeepSeek reasoner** — исправлен режим reasoner для корректной работы стриминга ([документ](docs/fixes/DEEPSEEK_REASONER_FIX.md)) (29 января 2026)
- **Stream timeout в dialogue_node** — добавлен таймаут для API stream соединения, устранено зависание (30 января 2026)
- **ThreadPoolExecutor deadlock** — устранён вложенный deadlock при создании stream (30 января 2026)
- **Повторение ответов LLM** — добавлены ограничения на повторения в промте ([документ](docs/fixes/PROMPT_REPETITION_FIX.md)) (29 января 2026)
- **Остановка agent cycle** — улучшены stopping conditions после tool calls, увеличен max_iterations с таймаутом (30 января 2026)
- **TTS ошибка устройства** — publish error state и сообщение при недоступности аудио устройства (30 января 2026)
- Удалён `set_emotion`, заменён на `play_animation` в системных промтах (30 января 2026)
- Fallback для Qwen отключён в конфигурации dialogue_node (30 января 2026)

## [Октябрь–Декабрь 2025]

### Добавлено
- Документация ICP Odometry: `docs/architecture/ICP_ODOMETRY.md` (декабрь 2025)
- GUI интерфейс управления роботом `tools/robot_control_gui_simple.py` (ноябрь 2025)
- Параметр `enable_search` для Qwen API web-поиска в dialogue_node и reflection_node (ноябрь 2025)
- Голосовой ассистент rob_box_voice с DeepSeek, Vosk STT, Silero TTS
- LED анимации rob_box_animations для WS2812B матриц
- Интеграция Zenoh для распределённой связи между Vision Pi и Main Pi
- Docker контейнеры для всех сервисов
- RTAB-Map SLAM с OAK-D Lite камерой
- AprilTag детекция на Vision Pi
- Nav2 навигация с командным управлением
- Документация в docs/ по стандартам ROS 2
- Система мониторинга с Grafana, Prometheus, Loki (24 октября 2025)
  - Легковесный мониторинг на отдельной машине
  - cAdvisor и Promtail на обоих Raspberry Pi
  - Красивые Grafana дашборды с 20 панелями
  - Скрипты enable/disable для управления мониторингом
- Полная документация по Zenoh namespace и облачному подключению (23 октября 2025)
- Исследование практик маппинга для RTAB-Map (24 октября 2025)
- Time awareness в dialogue_node - робот теперь знает текущее время (24 октября 2025)
- dialogue_id для синхронизации TTS чанков между сеансами диалога (24 октября 2025)

### Изменено
- Миграция с ROS 2 topics на Zenoh pub/sub
- Переход на offline-first стратегию для STT/TTS
- Реорганизация Docker структуры по стандартам проекта
- Оптимизация сборки для Raspberry Pi 5
- Система накопления запросов в dialogue_node — все запросы отправляются одним пакетом в DeepSeek (таймаут 2.5с) (4 ноября 2025)
- Автоматический fallback между Qwen и DeepSeek в dialogue_node и reflection_node (ноябрь 2025)
- Провайдер LLM по умолчанию изменён на DeepSeek (ноябрь 2025)
- Перемещение perception и lslidar контейнеров с Vision Pi на Main Pi (24 октября 2025)
- Рефакторинг системы мониторинга — агенты на Pi, центральный стек на отдельной машине (24 октября 2025)
- Изменена стратегия CI/CD — создание PR вместо прямого auto-merge (23 октября 2025)
- Реорганизация скриптов и конфигов согласно DOCKER_STANDARDS.md (24 октября 2025)

### Исправлено
- USB питание на Vision Pi для OAK-D камеры
- Проблемы с контейнерами Vision Pi (config volumes, network_mode)
- Ошибки компиляции apriltag и lslidar драйверов в Docker
- TF трансформации — robot-state-publisher теперь использует Zenoh namespace wrapper (24 октября 2025)
- Порядок TTS чанков — предотвращение смешивания между сеансами диалога (24 октября 2025)
- Дублирование запусков тестов и линтинга в CI/CD (23 октября 2025)
- Предупреждение 'PerceptionEvent не найден' в voice-assistant (24 октября 2025)
- Orphaned workflow build-all-local.yml — добавлен placeholder с deprecation notice (28 октября 2025)
- Инвалидация кэша Docker образа robot-state-publisher (19 ноября 2025)
- Поворот лидара на 180° — корректная ориентация LSLIDAR N10 (20 ноября 2025)
- Ориентация колёс и осей (20 ноября 2025)
- Парсинг JSON для ответов Qwen и DeepSeek (ноябрь 2025)
- Chipmunk эффект TTS — восстановлен оригинальный голос ([документ](docs/fixes/CHIPMUNK_VOICE_FIX_SUMMARY.md))

## [0.1.0] - 2025-10-04

### Добавлено
- Первый релиз базовой системы
- URDF модель робота rob_box_description
- Базовые launch файлы rob_box_bringup
- Интеграция VESC моторных контроллеров vesc_nexus
- ESP32 сенсорный хаб robot_sensor_hub_msg
- LED драйверы ros2leds и led_matrix_driver

---

**Навигация:** [← Назад в README](README.md) | [📚 Документация](docs/README.md)
