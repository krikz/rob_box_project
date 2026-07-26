# Scenario Runner использует тот же базовый образ что и voice-assistant
# (там уже есть ROS2 kilted + rclpy + все зависимости)
ARG BASE_IMAGE=ghcr.io/krikz/rob_box:voice-assistant-kilted-latest
FROM ${BASE_IMAGE}

RUN pip install --break-system-packages --no-cache-dir requests pyyaml

COPY runner.py /runner/runner.py
# Scenarios are mounted as volume at runtime (not baked into image)
# so they can be updated without rebuilding
COPY voice_memory.db /baked_db/voice_memory.db

# Source ROS2 при запуске
ENV PYTHONPATH=/opt/ros/kilted/lib/python3.10/site-packages

CMD ["python3", "/runner/runner.py"]
