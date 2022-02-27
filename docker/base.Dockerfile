FROM ros:noetic-ros-base
SHELL ["/bin/bash", "-c"]
WORKDIR /amp_ws

RUN \
    apt update && \
    apt install -y \
        tmux \
        curl \
        wget \
        vim \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        mesa-utils \
        unzip \ 
    && apt upgrade -y && \
    rm -rf /var/lib/apt/list/*
