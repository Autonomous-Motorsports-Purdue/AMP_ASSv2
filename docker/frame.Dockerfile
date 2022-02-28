FROM amp-devel:noetic-desktop
SHELL ["/bin/bash", "-c"]
WORKDIR /amp_ws

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    tmux \
    curl \
    wget \
    vim \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    unzip \ 
    && rm -rf /var/lib/apt/list/* \
    && rosdep update

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
