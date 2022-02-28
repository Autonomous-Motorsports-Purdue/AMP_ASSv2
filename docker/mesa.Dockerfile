FROM amp-devel:frame-desktop
SHELL ["/bin/bash", "-c"]
WORKDIR /amp_ws

COPY src ./src
COPY .catkin_workspace .

RUN . /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths src -r -y && \
    catkin_make
RUN echo "source /amp_ws/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
