FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 as nvidia
FROM ubuntu:18.04
LABEL maintainer="Emiliano Borghi"

ARG uid
ENV USER test

# Setup environment
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV \
  LANG=en_US.UTF-8 \
  DEBIAN_FRONTEND=noninteractive \
  TERM=xterm

# Dependencies
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        apt-transport-https \
        apt-utils \
        ca-certificates \
        gnupg2 \
        mesa-utils \
        software-properties-common \
        sudo \
        wget

# Create a user with passwordless sudo
RUN adduser --gecos "Development User" --disabled-password -u ${uid} ${USER}
RUN adduser ${USER} sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER root

# Add ROS keys
# https://discourse.ros.org/t/new-gpg-keys-deployed-for-packages-ros-org/9454
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Add Gazebo keys
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall

# Install ROS packages
RUN apt-get update && \
    apt-get install -y \
    ros-melodic-desktop \
    ros-melodic-imu-tools \
    ros-melodic-gmapping \
    ros-melodic-tf

RUN apt-get update && \
    apt-get install -y \
    gazebo9 \
    libgazebo9-dev

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-melodic-moveit ros-melodic-navfn ros-melodic-amcl \
    ros-melodic-base-local-planner ros-melodic-costmap-2d ros-melodic-teleop-twist-keyboard \
    ros-melodic-map-server ros-melodic-move-base \
    ros-melodic-gazebo-plugins \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-dwa-local-planner

# Installing OpenGL for nvidia-docker2
# https://stackoverflow.com/a/53823600
COPY --from=nvidia /usr/local /usr/local
COPY --from=nvidia /etc/ld.so.conf.d/glvnd.conf /etc/ld.so.conf.d/glvnd.conf

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

COPY 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

RUN apt-get update

# Finish ROS installation
RUN rosdep init
USER $USER
RUN rosdep update

# Install dependencies from source
# http://wiki.ros.org/catkin/Tutorials/using_a_workspace#Installing_Packages
ENV DEPS_WS=/deps_ws
RUN sudo mkdir -p ${DEPS_WS}/src
WORKDIR ${DEPS_WS}/src
RUN sudo git clone https://github.com/stevendes/turtlebot_warehouse
WORKDIR ${DEPS_WS}
USER ${USER}
RUN rosdep install --from-paths src --rosdistro=melodic -yi -r --os=ubuntu:bionic
USER root
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash; \
        catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic; \
        cd build; make install"
RUN sudo rm -r ${DEPS_WS}

# Automatically source ROS workspace
RUN echo ". /opt/ros/melodic/setup.bash" >> /home/${USER}/.bashrc
ENV WS_DIR "/catkin_ws"
ENV CATKIN_SETUP_BASH "${WS_DIR}/devel/setup.bash"
RUN echo "[[ -f ${CATKIN_SETUP_BASH} ]] && . ${CATKIN_SETUP_BASH}" >> /home/${USER}/.bashrc
USER root

# Workspace
RUN mkdir -p /catkin_ws/src/ && \
    chown -R $USER /catkin_ws

WORKDIR /catkin_ws/

RUN apt-get update

USER $USER
