FROM ros:noetic-ros-base
SHELL ["/bin/bash", "-c"]
WORKDIR /amp_ws

COPY . .

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
RUN . /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths src -r -y && \
    catkin_make
RUN echo "source /amp_ws/devel/setup.bash" >> ~/.bashrc

