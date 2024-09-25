##############################################################################
##                           1. stage: Base Image                           ##
##############################################################################
ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop as base

# Configure DDS
COPY dds_profile.xml /opt/misc/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/misc/dds_profile.xml

# Create user
ARG USER=fanuc_m16ib
ARG UID=1000
ARG GID=1000
ENV USER=$USER
RUN groupadd -g $GID $USER \
    && useradd -m -u $UID -g $GID --shell $(which bash) $USER

#install xacro and joint state publisher gui package (additional necessarity when not using the ur-package from ros)
USER root
RUN apt-get update && apt-get install -y ros-humble-xacro
RUN apt-get update && apt-get install -y ros-humble-joint-state-publisher-gui
USER ${USER}

# Setup workpace
USER $USER
RUN mkdir -p /home/$USER/ros2_ws/src
WORKDIR /home/$USER/ros2_ws


