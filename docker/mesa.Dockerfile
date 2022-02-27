FROM amp-base:default
SHELL ["/bin/bash", "-c"]
WORKDIR /amp_ws

COPY . .
RUN . /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths src -r -y && \
    catkin_make && echo "source /amp_ws/devel/setup.bash" >> ~/.bashrc

