# PerceptionEvent.msg - Unified perception event
# Агрегированный контекст от всех источников данных

# Timestamp
builtin_interfaces/Time stamp

# Vision context (JSON string from vision processing)
string vision_context

# Robot state
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
bool is_moving

# Sensors
float32 battery_voltage
float32 temperature

# AprilTags (detected IDs)
int32[] apriltag_ids

# System health
string system_health_status  # "healthy", "degraded", "critical"
string[] health_issues

# Time awareness (current time context)
string current_time_human     # Human-readable time: "2025-10-21 12:30:00"
string time_period            # "morning", "day", "evening", "night"
string time_context_json      # Full time context as JSON

# Internet connectivity
bool internet_available

# ROS2 nodes availability
string[] active_nodes
string[] failed_nodes
string[] missing_nodes

# Equipment health (cameras, LIDAR, motors, SLAM)
string equipment_summary_json

# Memory context (recent events summary)
string memory_summary

# Summarized history (from DeepSeek) - structured context
string speech_summaries         # Суммаризованные вопросы пользователя (JSON array)
string robot_response_summaries # Суммаризованные ответы робота (JSON array)
string robot_thought_summaries  # Суммаризованные внутренние мысли робота (JSON array)
string vision_summaries         # Суммаризованные визуальные наблюдения (JSON array)
string system_summaries         # Суммаризованные системные события (JSON array)
