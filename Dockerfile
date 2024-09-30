##############################################################################
##                           1. stage: Base Image                           ##
##############################################################################
ARG ROS_DISTRO=humble

# AMD64 Hardware
#FROM osrf/ros:$ROS_DISTRO-desktop as base

# ARM64 Hardware
FROM arm64v8/ros:$ROS_DISTRO AS base

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

# Setup workpace
USER $USER
RUN mkdir -p /home/$USER/ros2_ws/src
WORKDIR /home/$USER/ros2_ws


# install common ros2 pkgs
USER root
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rviz2
USER ${USER}


# install necessary packages for ros2 control
USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-lifecycle \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-std-srvs
USER $USER


# install necessary packages for moveit
USER root
RUN DEBIAN_FRONTEND=noninteractive \
	apt update && apt install -y  \
    ros-${ROS_DISTRO}-moveit  \
    ros-${ROS_DISTRO}-moveit-common  \
    ros-${ROS_DISTRO}-moveit-servo  \
    ros-${ROS_DISTRO}-moveit-ros-perception  \
    ros-${ROS_DISTRO}-joint-trajectory-controller  \
    ros-${ROS_DISTRO}-joint-state-broadcaster  \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-sensor-msgs-py  \
    ros-${ROS_DISTRO}-rqt-controller-manager
USER $USER


#install dependencies to calculate with affine transformations, use track-ik and python interface
USER root
RUN DEBIAN_FRONTEND=noninteractive \
    apt update && apt install -y  \
    ros-${ROS_DISTRO}-tf-transformations

RUN apt-get update && apt-get install -y pip
RUN apt-get update && apt-get install -y libnlopt*

RUN pip install transforms3d
RUN pip install scipy
RUN pip install numpy==1.23.5
USER $USER


#pick-ik setup
USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-pick-ik
USER $USER


#zivid setup
USER root
RUN mkdir -p /home/$USER/zivid
WORKDIR /home/$USER/zivid

RUN apt-get update && apt-get install -y wget \
    g++

RUN wget \
https://downloads.zivid.com/sdk/releases/2.13.1+18e79e79-1/u20/arm64/zivid_2.13.1+18e79e79-1_arm64.deb \
https://downloads.zivid.com/sdk/releases/2.13.1+18e79e79-1/u20/arm64/zivid-studio_2.13.1+18e79e79-1_arm64.deb \
https://downloads.zivid.com/sdk/releases/2.13.1+18e79e79-1/u20/arm64/zivid-tools_2.13.1+18e79e79-1_arm64.deb \
https://downloads.zivid.com/sdk/releases/2.13.1+18e79e79-1/u20/arm64/zivid-genicam_2.13.1+18e79e79-1_arm64.deb
RUN apt-get update && apt-get install ./*.deb -y

    #recommendet for CUDA permissions of the user "NvRmMemInitNvmap failed with Permission denied"
RUN usermod -aG sudo,video "$USER" 

WORKDIR /home/$USER/ros2_ws
USER $USER

# Copy src into src folder to build the workspace initially --> mounting overwrites this
COPY ./src /home/$USER/ros2_ws/src

# Build the workspace packages
RUN cd /home/$USER/ros2_ws && \
     . /opt/ros/$ROS_DISTRO/setup.sh && \
     colcon build

# Add built package to entrypoint by calling install/setup.bash
USER root
RUN sed -i 's|exec "\$@"|source "/home/'"${USER}"'/ros2_ws/install/setup.bash"\n&|' /ros_entrypoint.sh
USER $USER

CMD ["/bin/bash"]




