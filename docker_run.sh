#!/bin/bash
PS_NAME=px4_ros2

container_name="px4_ros2"
counter=1

if [ "$1" == "new" ]; then
	if docker ps -a --format "{{.Names}}" | grep -q "$container_name"; then
	    while true; do
		new_name="${container_name}_${counter}"
		if ! docker ps -a --format "{{.Names}}" | grep -q "$new_name"; then
		    container_name="$new_name"
		    break
		fi
		counter=$((counter + 1))
	    done
	fi
PS_NAME="${container_name}"
elif [ "$1" == "lab09" ]; then
	echo "stopping all containers"
	docker stop $(docker ps -aq)
	exit 0
elif [ "$1" == "remove" ]; then
	echo "stopping and removing all container"
	docker stop $(docker ps -a -q --filter "name=px4_ros2")
	docker rm $(docker ps -a -q --filter "name=px4_ros2")
	exit
else
	echo "stopping and removing container"
	docker stop $PS_NAME 2>/dev/null
	docker rm $PS_NAME 2>/dev/null
fi

echo "make container"
xhost +
if docker images | awk -v image_name="mdeagewt/px4_ros2" -v image_tag="1.0" '$1 == image_name && $2 == image_tag {found=1; exit} END {exit !found}'; then
  if command -v nvidia-smi &> /dev/null; then
    echo "run docker image using gpu"
    docker run -it --privileged --gpus all \
      -e DISPLAY=$DISPLAY \
      --env="QT_X11_NO_MITSHM=1" \
      -e NVIDIA_DRIVER_CAPABILITIES=all \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v /dev:/dev:rw \
      -w /home/user \
      --hostname $(hostname) \
      --group-add dialout \
      --user user \
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
      -w /home/user \
      --hostname $(hostname) \
      --group-add dialout \
      --user user \
      --shm-size 4096m \
      --name $PS_NAME mdeagewt/px4_ros2:1.0 bash
  fi
else
    echo "download docker image first using \"docker pull mdeagewt/px4_ros2:1.0\""
fi
