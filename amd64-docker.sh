if [ "$#" -eq 2 ]; then
    docker build --file ./docker/amd64.Dockerfile --tag amp-ass:dev .
fi

if [ "$1" -eq 0 ]; then
    echo "These are the options:"
    echo "  - 1: Intel graphics"
    echo "  - 2: Nvidia graphics"
    echo "  - X 0: Build and run on 'X' graphics"
elif [ "$1" -eq 1 ]; then # Intel graphics
    DOCKER_ARGS="-it --rm --net=host \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --device="/dev/dri:/dev/dri" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --privileged"
elif [ "$1" -eq 2 ]; then # Nvidia graphics
    DOCKER_ARGS="-it --rm --net=host \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_DRIVER_CAPABILITIES=all" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --gpus all \
        --privileged"
fi

if [ "$1" -ne 0 ]; then
    xhost + local:docker > /dev/null
    docker run $DOCKER_ARGS amp-ass:dev bash
fi
