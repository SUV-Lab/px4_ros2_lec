#!/bin/bash
PS_NAME=px4_ros2
SCRIPT_PATH="$(dirname "$(readlink -f "$0")")"
xhost +

echo "stopping and removing container"
docker stop $PS_NAME 2>/dev/null
docker rm $PS_NAME 2>/dev/null

if docker images | awk -v image_name="mdeagewt/px4_ros2" -v image_tag="1.0" '$1 == image_name && $2 == image_tag {found=1; exit} END {exit !found}'; then
  if command -v nvidia-smi &> /dev/null; then
    echo "run docker image using gpu"
    docker run -it --privileged --gpus all \
      -e DISPLAY=$DISPLAY \
      --env="QT_X11_NO_MITSHM=1" \
      -e NVIDIA_DRIVER_CAPABILITIES=all \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v /dev:/dev:rw \
      -v $SCRIPT_PATH/lab:/home/user/lab \
      -w /home/user \
      --hostname $(hostname) \
      --group-add dialout \
      --user user \
      --network host \
      --shm-size 4096m \
      --name $PS_NAME mdeagewt/px4_ros2:1.0 bash
  else
    echo "run docker image without using gpu"
    echo "if you have gpu or gazebo runs with black screen, install nvidia-driver and nvidia-docker"
    docker run -it --privileged \
      -e DISPLAY=$DISPLAY \
      --env="QT_X11_NO_MITSHM=1" \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v /dev:/dev:rw \
      -v $SCRIPT_PATH/lab:/home/user/lab \
      -w /home/user \
      --hostname $(hostname) \
      --group-add dialout \
      --user user \
      --network host \
      --shm-size 4096m \
      --name $PS_NAME mdeagewt/px4_ros2:1.0 bash
  fi
else
    echo "download docker image first using \"docker pull mdeagewt/px4_ros2:1.0\""
fi

