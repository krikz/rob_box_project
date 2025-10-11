# robot_sensor_hub_msg/DeviceCommand.msg
# Command for device control

uint8 device_type      # device type: 0=AHT30, 1=HX711, 2=FAN
uint8 device_id        # device ID
uint8 command_code     # command: 0=SET_SPEED, 1=TARE_SCALE
float32 param_1        # command parameter 1
float32 param_2        # command parameter 2
