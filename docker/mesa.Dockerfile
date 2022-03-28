FROM amp-devel:frame-desktop
SHELL ["/bin/bash", "-c"]
WORKDIR /amp_ws

COPY src ./src
COPY .catkin_workspace .

RUN apt-get update && \
    . /opt/ros/noetic/setup.bash && \
    rosdep install --rosdistro=$(echo ROS_DISTRO) --from-paths src -iry && \
    rm -rf /var/lib/apt/list/* && \
    catkin_make
RUN echo "source /amp_ws/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
