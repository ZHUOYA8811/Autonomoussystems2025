#!/bin/bash

CONTAINER_NAME="jazzy-as3"
IMAGE_NAME="osrf/ros:jazzy-desktop"
WORKSPACE="$HOME/AS/autonomous_system-group05"

# 允许 docker 访问 X server
xhost +local:docker > /dev/null

# 检查容器是否存在
if [ "$(docker ps -a -q -f name=$CONTAINER_NAME)" ]; then
    # 容器存在
    if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        echo "Container is already running. Attaching..."
        docker exec -it $CONTAINER_NAME bash -c "
            source /opt/ros/jazzy/setup.bash && 
            cd /workspace && 
            if [ -f install/setup.bash ]; then source install/setup.bash; fi && 
            bash"
    else
        echo "Starting existing container..."
        docker start $CONTAINER_NAME
        docker exec -it $CONTAINER_NAME bash -c "
            source /opt/ros/jazzy/setup.bash && 
            cd /workspace && 
            if [ -f install/setup.bash ]; then source install/setup.bash; fi && 
            ros2 launch simulation simulation.launch.py"
    fi
else
    echo "Creating new container..."
    docker run -it \
        --name $CONTAINER_NAME \
        --network host \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $WORKSPACE:/workspace \
        $IMAGE_NAME \
        bash -c "
            source /opt/ros/jazzy/setup.bash && 
            cd /workspace && 
            bash"
fi
