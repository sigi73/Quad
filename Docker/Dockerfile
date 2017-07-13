FROM ubuntu:trusty

# setup keys
RUN apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8


# install ros packages
ENV ROS_DISTRO indigo
RUN apt-get update && apt-get install -y \
    ros-indigo-desktop ros-indigo-cv-bridge ros-indigo-cv-camera ros-indigo-opencv3 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

RUN mkdir /development
VOLUME /development
