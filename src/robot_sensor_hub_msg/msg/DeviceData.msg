# robot_sensor_hub_msg/DeviceData.msg
# Simplified device data structure used in ESP32 firmware

uint8 device_type     # device type: 0=AHT30, 1=HX711, 2=FAN
uint8 device_id       # device ID (channel for AHT30, fan number, etc.)
uint8 data_type       # data type: 1=temp, 2=humidity, 3=weight, 4=speed, 5=rpm
float32 value         # measured value
uint8 error_code      # error code (0=ok, >0=error)
