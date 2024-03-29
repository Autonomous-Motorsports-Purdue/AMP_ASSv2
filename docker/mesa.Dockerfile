FROM amp-devel:frame-desktop
SHELL ["/bin/bash", "-c"]
WORKDIR /amp_ws

COPY src ./src

RUN apt-get update && \
    rosdep update && \
    . /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths src -iry && \
    rm -rf /var/lib/apt/list/* && \
    catkin_make
RUN echo "source /amp_ws/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
