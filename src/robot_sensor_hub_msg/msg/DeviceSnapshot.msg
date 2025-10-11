# robot_sensor_hub_msg/DeviceSnapshot.msg
# Aggregated snapshot of all device readings

int64 timestamp           # timestamp in nanoseconds
DeviceData[] devices      # array of device data
