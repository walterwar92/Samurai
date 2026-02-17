FROM osrf/ros:humble-desktop

# ROS2 navigation packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies (CPU-only PyTorch + ultralytics)
ENV PIP_CONSTRAINT=""
RUN pip3 install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir \
    "ultralytics>=8.0" \
    "flask>=3.0" \
    flask-cors \
    flask-socketio \
    paho-mqtt \
    timm \
    && pip3 install --no-cache-dir "numpy<2"

# Build ROS2 workspace
WORKDIR /root/Samurai/ros_ws
ENV ROS_DOMAIN_ID=42

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]
