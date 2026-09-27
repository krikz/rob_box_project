# ROB-BOX Feature & Subfeature Matrix

Date: 2026-09-27

## Evidence model
- Code — implementation exists.
- Tests — unit/integration evidence.
- E2E — acceptance/E2E evidence.
- Live — physical robot/runtime evidence.

Status: 🟢 Done; 🟡 Partial; 🟠 Skeleton; 🔴 Broken/Gap; 🔵 Design only; — not separately evaluated.

## 1. Voice Assistant
| Feature | Subfeature | Code | Tests | E2E | Live | Overall |
|---|---|:---:|:---:|:---:|:---:|:---:|
| Audio | capture / ReSpeaker / channels / DSP | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| STT | providers / fallback | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| STT | timeout / runtime reliability | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| Wake Gate | activation / preflight / handoff | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| Dialogue | node / FSM / turns | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Dialogue | conversation history / context | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| Dialogue | identity context | 🟡 | 🟢 | 🔴 | 🔴 | 🔴 |
| LLM | provider abstraction / fallback | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| LLM | tool calling | 🟢 | 🟢 | 🟡 | 🟡 | 🟡 |
| LLM | deterministic tool selection | 🟢 | 🟢 | 🔴 | 🔴 | 🔴 |
| TTS | abstraction / MiniMax | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| TTS | voice / prosody | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| TTS | SFX | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| ActionServer | ROS actions / HTTP adapter | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |

## 2. Identity
| Feature | Subfeature | Code | Tests | E2E | Live | Overall |
|---|---|:---:|:---:|:---:|:---:|:---:|
| Face | detection / embeddings / gallery | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Face | confidence / encounters | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Speaker | embeddings / DB / registration | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Speaker | confidence bands | 🟢 | 🟢 | 🟡 | 🟡 | 🟡 |
| Speaker | cold/warm startup | 🟢 | 🟢 | 🔴 | 🔴 | 🔴 |
| Identity | voice ↔ face mapping | 🟡 | 🟢 | 🔴 | 🔴 | 🔴 |
| Identity | unified identity key | 🔴 | 🔴 | 🔴 | 🔴 | 🔴 |
| Identity | persistent identity context | 🔴 | 🔴 | 🔴 | 🔴 | 🔴 |

## 3. Memory / Context
| Feature | Subfeature | Code | Tests | E2E | Live | Overall |
|---|---|:---:|:---:|:---:|:---:|:---:|
| Storage | SQLite / voice turns / facts | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Storage | FTS / vector index | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Save | memory_save | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Search | memory_search | 🟢 | 🟢 | 🟢* | 🔴 | 🔴 |
| Search | semantic / fact retrieval | 🟢 | 🟢 | 🟢* | 🔴 | 🔴 |
| Seam | save → search consistency | 🔴 | 🔴 | 🔴* | 🔴 | 🔴 |
| Session | history / reset / isolation | 🟢 | 🟢 | 🔴 | 🔴 | 🔴 |
| Context | robot context | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| Context | person / long-term context | 🟡 | 🟢 | 🔴 | 🔴 | 🔴 |

## 4. Perception
| Feature | Subfeature | Code | Tests | E2E | Live | Overall |
|---|---|:---:|:---:|:---:|:---:|:---:|
| Sensor bridge | ROS sensors / vision | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| Vision | face events | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Audio perception | speaker events | 🟢 | 🟢 | 🟡 | 🟡 | 🟡 |
| Context | aggregator / normalization | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| Context | person context | 🟡 | 🟢 | 🔴 | 🔴 | 🔴 |
| Health | node/container/runtime health | 🟢 | 🟢 | 🟡 | 🔴 | 🔴 |

## 5. Robot Control / Navigation
| Feature | Subfeature | Code | Tests | E2E | Live | Overall |
|---|---|:---:|:---:|:---:|:---:|:---:|
| Navigation | Nav2 integration | 🟢 | 🟢 | 🟢 | 🟡 | 🟢/🟡 |
| Navigation | goals / cancel / feedback | 🟢 | 🟢 | 🟢 | 🟡 | 🟢/🟡 |
| Actions | movement actions | 🟢 | 🟢 | 🟢 | 🟡 | 🟢/🟡 |
| Safety | action guards | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |

## 6. Music / Entertainment
| Feature | Subfeature | Code | Tests | E2E | Live | Overall |
|---|---|:---:|:---:|:---:|:---:|:---:|
| Playback | play / stop / pause / state | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Composition | RTTTL / compose_music | 🟢 | 🟢 | 🟡 | 🟡 | 🟡 |
| Composition | melody lookup / exact matching | 🟢 | 🟢 | 🔴 | 🔴 | 🔴 |
| Composition | genre/style | 🟢 | 🟢 | 🟡 | 🟡 | 🟡 |
| Synth | lead / bass / pad | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Synth | counter / drums | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| DJ | mode / transitions | 🟢 | 🟢 | 🟡 | 🟡 | 🟡 |
| DJ | speech behavior | 🟢 | 🟢 | 🔴 | 🔴 | 🔴 |
| Packs | samples / loops / genre patterns | 🟢 | 🟢 | 🔴 | 🔴 | 🔴 |

## 7. Telegram
| Feature | Subfeature | Code | Tests | E2E | Live | Overall |
|---|---|:---:|:---:|:---:|:---:|:---:|
| Bot | bot / auth / basic commands | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Voice | voice commands / dialogue | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Voice | music / robot actions | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| Faces | listing / management | 🟡 | 🔵 | 🔵 | 🔵 | 🔵 |
| Faces | suspicious / merge workflows | 🟡 | 🔵 | 🔵 | 🔵 | 🔵 |

## 8. Avatar / Meta Quest / Telepresence
| Feature | Subfeature | Code | Tests | E2E | Live | Overall |
|---|---|:---:|:---:|:---:|:---:|:---:|
| Supervisor | AvatarState / FloorState / AvatarEvent | 🟢 | 🟢 | — | — | 🟢 |
| Supervisor | ModeManager / LockManager | 🟢 | 🟢 | — | — | 🟢 |
| Supervisor | runtime node / orchestration | 🟠 | 🟡 | 🔴 | 🔴 | 🔴 |
| Quest | Docker / Zenoh / Caddy | 🟢 | 🟢 | 🟡 | 🔴 | 🔴 |
| Quest | quest_node | 🟡 | 🟡 | 🔴 | 🔴 | 🔴 |
| Quest | HTTPS/WSS / WebXR | 🟡 | 🟡 | 🔴 | 🔴 | 🔴 |
| Telepresence | video / camera / LiDAR | 🟡 | 🟡 | 🔴 | 🔴 | 🔴 |
| Teleoperation | Quest → robot / movement | 🔴 | 🔴 | 🔴 | 🔴 | 🔴 |
| Safety | teleop lock | 🟡 | 🟡 | 🔴 | 🔴 | 🔴 |
| Multi-client | voice / Telegram | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| Multi-client | Quest / shared session / identity | 🔴 | 🔴 | 🔴 | 🔴 | 🔴 |

## 9. Platform / Deployment
| Feature | Subfeature | Code | Tests | E2E | Live | Overall |
|---|---|:---:|:---:|:---:|:---:|:---:|
| ROS2 | nodes / topics / services / actions | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Docker | voice / vision | 🟢 | 🟢 | 🟢 | 🟡 | 🟢/🟡 |
| Docker | Quest | 🟢 | 🟢 | 🔴 | 🔴 | 🔴 |
| Zenoh | transport / cross-machine | 🟢 | 🟢 | 🟢 | 🟡 | 🟡 |
| Deployment | Docker / systemd | 🟢 | 🟢 | 🟡 | 🔴 | 🔴 |
| Recovery | restart / health timer / self-recovery | 🟢 | 🟢 | 🟡 | 🔴 | 🔴 |
| Reliability | memory budget / OOM resistance | 🟡 | 🟡 | 🔴 | 🔴 | 🔴 |

## 10. E2E / Acceptance
| Feature | Subfeature | Code | Tests | E2E | Live | Overall |
|---|---|:---:|:---:|:---:|:---:|:---:|
| Harness | scenario runner | 🟢 | 🟢 | 🟢 | 🟡 | 🟢/🟡 |
| Harness | acceptance / topic injection | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Harness | verdict engine | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Harness | wake-gate preflight | 🟢 | 🟢 | 🟢 | 🟢 | 🟢 |
| Validation | E2E isolation | 🟡 | 🟡 | 🔴 | 🔴 | 🔴 |
| Validation | semantic validation | 🟡 | 🟡 | 🔴 | 🔴 | 🔴 |
| Validation | cross-act cleanup | 🟡 | 🟡 | 🔴 | — | 🔴 |

## Critical seams
1. memory_save → memory_search — #2793.
2. face → dialogue → speaker identity — #3024.
3. LLM → required tool selection — #2406.
4. Avatar FSM model → actual Supervisor runtime.
5. Quest infrastructure → Quest application — #2318.
6. Source → physical robot deployment — #3018.
7. Formal E2E PASS → semantic correctness — #2793.
8. Voice stack → memory budget / OOM — #2676.

## Recommended long-term review model
Track every feature as:

Feature → Subfeature → Implementation → Tests → E2E → Live → Owner/Source of Truth

Also record canonical implementation, duplicate implementations, contract, consumers, proving E2E scenario, live evidence and known seams.