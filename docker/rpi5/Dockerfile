ARG ROS_IMAGE=osrf/ros:humble-desktop-full
FROM ${ROS_IMAGE}

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full \
    python3-colcon-common-extensions python3-rosdep \
    python3-opencv ros-humble-cv-bridge ros-humble-rosbag2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
CMD ["sleep", "infinity"]
