#!/bin/bash

# in order to be able to use this script install:
# pip install docker-run-cli
DIR="$(cd -P "$(dirname "$0")" && pwd)"
if docker ps --format '{{.Names}}' | grep -q "acdc"; then
    docker-run --name acdc
else
    docker-run --gpus all --volume $(dirname "$DIR"):/docker-ros/ws --image rwthika/acdc:ros2-cuda-tf --name acdc
fi