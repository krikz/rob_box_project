# Mock Data for Testing

This directory contains mock/fake data files for testing Rob Box services without real hardware.

## Purpose

When testing on x86_64 without actual Raspberry Pi hardware, some services need mock data:
- **RTAB-Map** needs camera images and LiDAR scans
- **Nav2** needs odometry and laser scans
- **Perception** needs sensor data

## Files

Mock data files will be stored here (not in git due to size):
- `fake_scan.bag` - Mock LiDAR scan data
- `fake_image.bag` - Mock camera images
- `fake_odom.bag` - Mock odometry data

## Generating Mock Data

To generate mock data for testing:

```bash
# Record from a real robot (on Raspberry Pi)
ros2 bag record /scan /camera/rgb/image_raw /odom -o test_data

# Or use simulation (if available)
ros2 launch rob_box_gazebo simulation.launch.py
ros2 bag record /scan /camera/rgb/image_raw /odom -o sim_data

# Copy to mock_data directory
scp ros2@10.1.1.21:~/test_data_0.db3 ./mock_data/fake_scan.bag
```

## Using Mock Data

Start a ROS2 bag player in a container:

```bash
# Play mock data
docker run --rm -it \
  --network host \
  -v $(pwd)/mock_data:/mock_data:ro \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -e ZENOH_SESSION_CONFIG_URI=/mock_data/zenoh_session_config.json5 \
  ros:humble \
  ros2 bag play /mock_data/fake_scan.bag --loop
```

This will make RTAB-Map and Nav2 work in the test environment.
