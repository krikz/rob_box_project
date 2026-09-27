# VisionEvent.msg — Single AI HAT+ inference event
#
# One message = one detection (face, object, person, gesture).
# Published by vision_hailo_node on /vision/hailo/events.
# Aggregated by context_aggregator_node into PerceptionEvent.vision_events_json.
#
# Empty bbox = scene-level event (e.g. "scene change", "no person detected").
#
# Coordinates are normalized to [0, 1] over the source image dimensions,
# so the same message can be rendered on top of any camera stream without
# rescaling. This is a deliberate choice over pixel-coords: downstream
# consumers (LLM context, TARS cockpit, Quest WebXR) all work with
# normalized fractions, not pixels.

# Timestamp
builtin_interfaces/Time stamp

# Source camera identifier ("oak-d", "mjpg-720p", "ceiling-camera").
# Sourced from hailo_models.yaml config; never free-form at runtime.
string source_camera

# Event type discriminator — Phase 1 keeps these four stable, Phase 2/3
# may extend (e.g. "gesture_wave", "scene_change").
# Allowed: "person" | "face" | "object" | "scene" | "stub"
# "stub" — событие выдумано заглушкой (hailo_enabled=false), кадра за ним
# нет. Потребители, решающие «показывать ли это Личности», обязаны его
# отсекать — ADR-0089 §2.2, предикат vision_hailo_loader.is_stub_event.
string event_type

# Detection class label (free-form, mirrors Hailo model zoo class names).
# Empty when event_type == "scene".
string class_name

# Detection class index inside the active HEF (negative = unknown / N/A).
int32 class_id

# Confidence [0.0, 1.0]. 0.0 means the producer did not compute one.
float32 confidence

# Normalized bounding box [0, 1]. Negative width/height = "no box".
# (cx, cy) = center, w/h = size.
float32 bbox_cx
float32 bbox_cy
float32 bbox_w
float32 bbox_h

# Approximate distance in meters (fused with OAK-D stereo depth when
# available). Negative or NaN = "depth unknown".
float32 distance_m

# Embedding reference id (set by face pipeline only, Phase 2).
# Empty for person/object events. Maps to /data/faces.db rowid.
string embedding_id

# Display name resolved from embedding (e.g. "Masha"). Empty for unknown.
string display_name

# Optional structured attributes (JSON object). Phase 2 face attributes
# (clothing, accessories) will land here. Empty in Phase 1.
string attributes_json
