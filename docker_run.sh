#!/bin/bash
PS_NAME=px4_ros2

container_name="px4_ros2"
counter=1

notify_wsl() {
	echo "WSL2 DETECTED!"
	echo "YOU MUST HAVE AN WINDOWS OS WHICH MEETS FOLLOWING CONDITION FOR GUI:"
	echo ">>> Windows 10: Build 19044+ or newer   /   Windows 11"
	echo "ALSO, MAKE SURE THE WINDOWS HOST HAS GRAPHICS DRIVER WHICH SUPPORTS WSLg"
	echo ">>> INTEL GRAPHICS:"
	echo ">>> - INSTALL THIS ON WSL2: sudo apt-get install -y intel-opencl-icd"
	echo ">>> - INSTALL THIS ON WINDOWS: https://www.intel.com/content/www/us/en/download/19344/intel-graphics-windows-dch-drivers.html"
	echo ">>> NVIDIA GRAPHICS:"
	echo ">>> - INSTALL THE NEWEST NVIDIA DRIVER WITH WSLg SUPPORT"
	echo ">>> - YOU ALSO NEED NVIDIA-CONTAINER-TOOLKIT SAME AS THE LINUX ENVIRONMENT"
}

# publisher ==> packet capture and N publisher , last publisher default awake 20s it change $3
if [ "$1" == "server_talker" -o "$1" == "talker" ]; then
	set="$2"

	if [ -z "$3" ]; then
		time="20"
	else	
		time="$3"
	fi
	
	for var in $(seq 1 $set)
	do
		sum=$(expr $set - $var + 1)
		echo "Set Server_Talker $sum"
		PS_NAME="${container_name}_publisher_${sum}"
		if docker images | awk -v image_name="stmoon/px4_ros2" -v image_tag="lec" '$1 == image_name && $2 == image_tag {found=1; exit} END {exit !found}'; then
		  if [[ $(grep microsoft /proc/version) ]]; then
			notify_wsl
			if command -v nvidia-smi &> /dev/null; then
				# CASE: WSL2 WITH NVIDIA GPU
				echo "run docker image using gpu (WSL)"
				docker run -it --privileged --gpus all \
				-e DISPLAY=$DISPLAY \
				-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
				-d \
				--rm \
				-e QT_X11_NO_MITSHM=1 \
				-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
				-e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
				-e NVIDIA_DRIVER_CAPABILITIES=all \
				-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
				-v /mnt/wslg:/mnt/wslg \
				-v /usr/lib/wsl:/usr/lib/wsl \
				-v /dev:/dev:rw \
				-v /run/user/1000:/run/user/1000 \
				-w /home/user \
				--device=/dev/dxg \
				--device=/dev/dri/card0 \
				--hostname $(hostname) \
				--group-add dialout \
				--user user \
				--ipc host \
				--shm-size 4096m \
				--name $PS_NAME stmoon/px4_ros2:lec bash -c "sleep $time"
			else
				# CASE: WSL2 WITHOUT NVIDIA GPU
				echo "run docker image without nvidia gpu"
				echo "if you have gpu or gazebo runs with black screen, install nvidia-driver and nvidia-docker"
				docker run -it --privileged \
				-e DISPLAY=$DISPLAY \
				-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
				-d \
				--rm \
				-e QT_X11_NO_MITSHM=1 \
				-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
				-e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
				-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
				-v /mnt/wslg:/mnt/wslg \
				-v /usr/lib/wsl:/usr/lib/wsl \
				-v /dev:/dev:rw \
				-v /run/user/1000:/run/user/1000 \
				-w /home/user \
				--device=/dev/dxg \
				--device=/dev/dri/card0 \
				--hostname $(hostname) \
				--group-add dialout \
				--user user \
				--ipc host \
				--shm-size 4096m \
				--name $PS_NAME stmoon/px4_ros2:lec bash -c "sleep $time"
			fi
		  else
			echo "GENERIC LINUX SYSTEM DETECTED!"	  
			if command -v nvidia-smi &> /dev/null; then
				echo "run docker image using gpu"
				docker run -it --privileged --gpus all \
				-e DISPLAY=$DISPLAY \
				-d \
				--rm \
				--env="QT_X11_NO_MITSHM=1" \
				-e NVIDIA_DRIVER_CAPABILITIES=all \
				-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
				-v /dev:/dev:rw \
				-w /home/user \
				--hostname $(hostname) \
				--group-add dialout \
				--user user \
				--shm-size 4096m \
				--name $PS_NAME stmoon/px4_ros2:lec bash -c "sleep $time"
			else
				echo "run docker image without using gpu"
				echo "if you have gpu or gazebo runs with black screen, install nvidia-driver and nvidia-docker"
				docker run -it --privileged \
				-e DISPLAY=$DISPLAY \
				-d \
				--rm \
				--env="QT_X11_NO_MITSHM=1" \
				-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
				-v /dev:/dev:rw \
				-w /home/user \
				--hostname $(hostname) \
				--group-add dialout \
				--user user \
				--shm-size 4096m \
				--name $PS_NAME stmoon/px4_ros2:lec bash -c "sleep $time"
			fi
		  fi
		else
		    echo "download docker image first using \"docker pull stmoon/px4_ros2:lec\""
		fi
		
		if [ "$1" == "server_talker" ]; then
			if [ $sum -eq 1 ]; then
			    docker exec $PS_NAME bash -c 'source /opt/ros/humble/setup.bash && export ROS_DISCOVERY_SERVER=172.17.0.2:11811 && ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server'
			else
			    docker exec -d $PS_NAME bash -c 'source /opt/ros/humble/setup.bash && export ROS_DISCOVERY_SERVER=172.17.0.2:11811 && ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server'
			fi
		fi
		if [ "$1" == "talker" ]; then
			if [ $sum -eq 1 ]; then
			    docker exec $PS_NAME bash -c 'source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker --ros-args --remap __node:=Simple_talker'
			else
			    docker exec -d $PS_NAME bash -c 'source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker --ros-args --remap __node:=Simple_talker'
			fi
		fi
	done
exit
fi
# ----------------------------------------------------------------


# listener ==> Discovery Server and Server listener , Server listener is always awake 
if [ "$1" == "server_listener" -o "$1" == "listener" ]; then
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
PS_NAME1="${container_name}_S"
fi

if [ "$1" == "server_listener" -o "$1" == "listener" ]; then
# xhost +

	if [ "$1" == "server_listener" ]; then

		if docker images | awk -v image_name="stmoon/px4_ros2" -v image_tag="lec" '$1 == image_name && $2 == image_tag {found=1; exit} END {exit !found}'; then
		  if [[ $(grep microsoft /proc/version) ]]; then
			notify_wsl
			if command -v nvidia-smi &> /dev/null; then
				echo "run docker image using nvidia gpu"
				docker run -it --privileged --gpus all \
				-e DISPLAY=$DISPLAY \
				-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
				-d \
				-e QT_X11_NO_MITSHM=1 \
				-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
				-e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
				-e NVIDIA_DRIVER_CAPABILITIES=all \
				-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
				-v /mnt/wslg:/mnt/wslg \
				-v /usr/lib/wsl:/usr/lib/wsl \
				-v /dev:/dev:rw \
				-v /run/user/1000:/run/user/1000 \
				-w /home/user \
				--device=/dev/dxg \
				--device=/dev/dri/card0 \
				--hostname $(hostname) \
				--group-add dialout \
				--user user \
				--ipc host \
				--shm-size 4096m \
				--name $PS_NAME1 stmoon/px4_ros2:lec bash
			else
				echo "run docker image without using nvidia gpu"
				echo "if you have gpu or gazebo runs with black screen, install nvidia-driver and nvidia-docker"
				docker run -it --privileged \
				-e DISPLAY=$DISPLAY \
				-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
				-d \
				-e QT_X11_NO_MITSHM=1 \
				-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
				-e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
				-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
				-v /mnt/wslg:/mnt/wslg \
				-v /usr/lib/wsl:/usr/lib/wsl \
				-v /dev:/dev:rw \
				-v /run/user/1000:/run/user/1000 \
				-w /home/user \
				--device=/dev/dxg \
				--device=/dev/dri/card0 \
				--hostname $(hostname) \
				--group-add dialout \
				--user user \
				--ipc host \
				--shm-size 4096m \
				--name $PS_NAME1 stmoon/px4_ros2:lec bash
			fi
		  else
			echo "GENERIC LINUX SYSTEM DETECTED!"
			xhost +
			if command -v nvidia-smi &> /dev/null; then
				echo "run docker image using gpu"
				docker run -it --privileged --gpus all \
				-e DISPLAY=$DISPLAY \
				-d \
				--env="QT_X11_NO_MITSHM=1" \
				-e NVIDIA_DRIVER_CAPABILITIES=all \
				-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
				-v /dev:/dev:rw \
				-w /home/user \
				--hostname $(hostname) \
				--group-add dialout \
				--user user \
				--shm-size 4096m \
				--name $PS_NAME1 stmoon/px4_ros2:lec bash
			else
				echo "run docker image without using gpu"
				echo "if you have gpu or gazebo runs with black screen, install nvidia-driver and nvidia-docker"
				docker run -it --privileged \
				-e DISPLAY=$DISPLAY \
				-d \
				--env="QT_X11_NO_MITSHM=1" \
				-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
				-v /dev:/dev:rw \
				-w /home/user \
				--hostname $(hostname) \
				--group-add dialout \
				--user user \
				--shm-size 4096m \
				--name $PS_NAME1 stmoon/px4_ros2:lec bash
			fi
		  fi
		else
		    echo "download docker image first using \"docker pull stmoon/px4_ros2:lec\""
		fi
		docker exec -d $PS_NAME1 bash -c 'source /opt/ros/humble/setup.bash && fastdds discovery --server-id 0'
	fi


	if docker images | awk -v image_name="stmoon/px4_ros2" -v image_tag="lec" '$1 == image_name && $2 == image_tag {found=1; exit} END {exit !found}'; then
	  if [[ $(grep microsoft /proc/version) ]]; then
	  	notify_wsl
		if command -v nvidia-smi &> /dev/null; then
			echo "run docker image using nvidia gpu"
			docker run -it --privileged --gpus all \
			-e DISPLAY=$DISPLAY \
			-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
			-d \
			-e QT_X11_NO_MITSHM=1 \
			-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
			-e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
			-e NVIDIA_DRIVER_CAPABILITIES=all \
			-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-v /mnt/wslg:/mnt/wslg \
			-v /usr/lib/wsl:/usr/lib/wsl \
			-v /dev:/dev:rw \
			-v /run/user/1000:/run/user/1000 \
			-w /home/user \
			--device=/dev/dxg \
			--device=/dev/dri/card0 \
			--hostname $(hostname) \
			--group-add dialout \
			--user user \
			--ipc host \
			--shm-size 4096m \
			--name $PS_NAME stmoon/px4_ros2:lec bash
		else
			echo "run docker image without using gpu"
			echo "if you have gpu or gazebo runs with black screen, install nvidia-driver and nvidia-docker"
			docker run -it --privileged \
			-e DISPLAY=$DISPLAY \
			-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
			-d \
			-e QT_X11_NO_MITSHM=1 \
			-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
			-e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
			-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-v /mnt/wslg:/mnt/wslg \
			-v /usr/lib/wsl:/usr/lib/wsl \
			-v /dev:/dev:rw \
			-v /run/user/1000:/run/user/1000 \
			-w /home/user \
			--device=/dev/dxg \
			--device=/dev/dri/card0 \
			--hostname $(hostname) \
			--group-add dialout \
			--user user \
			--ipc host \
			--shm-size 4096m \
			--name $PS_NAME stmoon/px4_ros2:lec bash
		fi
	  else
		echo "GENERIC LINUX SYSTEM DETECTED!"
		xhost +
		if command -v nvidia-smi &> /dev/null; then
			echo "run docker image using gpu"
			docker run -it --privileged --gpus all \
			-e DISPLAY=$DISPLAY \
			-d \
			--env="QT_X11_NO_MITSHM=1" \
			-e NVIDIA_DRIVER_CAPABILITIES=all \
			-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-v /dev:/dev:rw \
			-w /home/user \
			--hostname $(hostname) \
			--group-add dialout \
			--user user \
			--shm-size 4096m \
			--name $PS_NAME stmoon/px4_ros2:lec bash
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
			--name $PS_NAME stmoon/px4_ros2:lec bash
		fi
	  fi
	else
	    echo "download docker image first using \"docker pull stmoon/px4_ros2:lec\""
	fi
	
	if [ "$1" == "server_listener" ]; then
		docker exec $PS_NAME bash -c 'source /opt/ros/humble/setup.bash && export ROS_DISCOVERY_SERVER=172.17.0.2:11811 && ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server'
	fi
	
	if [ "$1" == "listener" ]; then
		docker exec $PS_NAME bash -c 'source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp listener --ros-args --remap __node:=Simple_listener'
	fi
exit
fi


#-------------------------------------------



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
elif [ "$1" == "lab10" ]; then
	echo "stopping all containers"
	docker stop $(docker ps -aq)
	echo " "
	echo "--------------------"
	echo "    Lab 10 Ready"
	echo "--------------------"
	echo " "
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
# xhost +
if docker images | awk -v image_name="stmoon/px4_ros2" -v image_tag="lec" '$1 == image_name && $2 == image_tag {found=1; exit} END {exit !found}'; then
	if [[ $(grep microsoft /proc/version) ]]; then
		notify_wsl
		if command -v nvidia-smi &> /dev/null; then
			echo "run docker image using nvidia gpu"
			docker run -it --privileged --gpus all \
				-e DISPLAY=$DISPLAY \
				-e QT_X11_NO_MITSHM=1 \
				-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
				-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
				-e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
				-e NVIDIA_DRIVER_CAPABILITIES=all \
				-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
				-v /mnt/wslg:/mnt/wslg \
				-v /usr/lib/wsl:/usr/lib/wsl \
				-v /dev:/dev:rw \
				-v /run/user/1000:/run/user/1000 \
				-w /home/user \
				--device=/dev/dxg \
				--device=/dev/dri/card0 \
				--hostname $(hostname) \
				--group-add dialout \
				--network none \
				--user user \
				--ipc host \
				--shm-size 4096m \
				--name $PS_NAME stmoon/px4_ros2:lec bash
		else
			echo "run docker image without using nvidia gpu"
			echo "if you have gpu or gazebo runs with black screen, install nvidia-driver and nvidia-docker"
			docker run -it --privileged \
				-e DISPLAY=$DISPLAY \
				-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
				-e QT_X11_NO_MITSHM=1 \
				-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
				-e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
				-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
				-v /mnt/wslg:/mnt/wslg \
				-v /usr/lib/wsl:/usr/lib/wsl \
				-v /dev:/dev:rw \
				-v /run/user/1000:/run/user/1000 \
				-w /home/user \
				--device=/dev/dxg \
				--device=/dev/dri/card0 \
				--hostname $(hostname) \
				--group-add dialout \
				--network none \
				--user user \
				--ipc host \
				--shm-size 4096m \
				--name $PS_NAME stmoon/px4_ros2:lec bash
		fi
	else
		echo "GENERIC LINUX SYSTEM DETECTED!"	  
		xhost +
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
				--network none \
				--user user \
				--shm-size 4096m \
				--name $PS_NAME stmoon/px4_ros2:lec bash
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
				--network none \
				--user user \
				--shm-size 4096m \
				--name $PS_NAME stmoon/px4_ros2:lec bash
		fi
	fi


else
    echo "download docker image first using \"docker pull stmoon/px4_ros2:lec\""
fi
