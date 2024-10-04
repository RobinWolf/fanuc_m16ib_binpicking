#!/bin/bash
##############################################################################
##                   Build the image, using Dockerfile                  ##
##############################################################################
ROS_DISTRO=humble

uid=$(eval "id -u")
gid=$(eval "id -g")

#pass some arguments and settings to the dev.Dockerfile while building the image (Dockerfile)

#--no-cache \
docker build \
  --build-arg ROS_DISTRO="$ROS_DISTRO" \
  --build-arg UID="$uid" \
  --build-arg GID="$gid" \
  -f Dockerfile \
  -t fanuc_m16ib_binpicking/"$ROS_DISTRO" .

##############################################################################
##                            Run the container                             ##
##############################################################################
SRC_CONTAINER=/home/fanuc_m16ib/ros2_ws/src
SRC_HOST="$(pwd)"/src                           
docker run \

docker run \
  --name fanuc_m16ib_binpicking \
  --rm \
  -it \
  --net=host \
  -e DISPLAY="$DISPLAY" \
  -v "$SRC_HOST":"$SRC_CONTAINER":rw \
  --gpus 'all,"capabilities=compute,utility,graphics"' \
  --runtime nvidia \
  --env-file .env\
  fanuc_m16ib_binpicking/"$ROS_DISTRO"

