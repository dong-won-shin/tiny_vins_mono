#!/bin/bash

# Run script for Tiny VINS Mono Docker container with GUI support

set -e

echo "Setting up X11 forwarding for GUI applications..."

# Create X11 auth file for Docker
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Starting Tiny VINS Mono Docker container..."

# Run the container with GUI support
docker run -it --rm \
    --name tiny-vins-mono \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH:rw" \
    --volume="$(pwd)/../data:/workspace/tiny_vins_mono/data:rw" \
    --volume="$(pwd)/../logs:/workspace/tiny_vins_mono/logs:rw" \
    --volume="$(pwd)/../config:/workspace/tiny_vins_mono/config:rw" \
    --net=host \
    tiny-vins-mono:latest \
    /bin/bash

echo "Container stopped."