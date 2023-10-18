#!/bin/bash

# in order to be able to use this script install:
# pip install docker-run-cli
DIR="$(cd -P "$(dirname "$0")" && pwd)"
if docker ps --format '{{.Names}}' | grep -q "acdc"; then
    docker-run --name acdc
else
    docker-run --volume $(dirname "$DIR"):/home/rosuser/ws --image rwthika/acdc:ros2 --name acdc
fi