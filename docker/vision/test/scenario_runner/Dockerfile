# Scenario Runner использует тот же базовый образ что и voice-assistant
# (там уже есть ROS2 Humble + rclpy + все зависимости)
ARG BASE_IMAGE=ghcr.io/krikz/rob_box:voice-assistant-humble-latest
FROM ${BASE_IMAGE}

RUN pip install --no-cache-dir requests pyyaml

COPY runner.py /runner/runner.py
COPY scenarios/ /scenarios/
# Verify scenarios were copied (debug)
RUN ls -la /scenarios/ && echo "scenarios copied OK"
COPY voice_memory.db /baked_db/voice_memory.db

# Source ROS2 при запуске
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages

CMD ["python3", "/runner/runner.py"]
