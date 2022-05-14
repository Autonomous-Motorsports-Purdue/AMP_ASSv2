FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop \
    && rm -rf /var/lib/apt/lists/*
