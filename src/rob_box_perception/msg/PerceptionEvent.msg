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

# Memory context (recent events summary)
string memory_summary
