FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-rosdep \
    ros-humble-joy ros-humble-rosbag2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
CMD ["sleep", "infinity"]
