# in order to be able to use this script install:
# pip install docker-run-cli
DIR="$(cd -P "$(dirname "$0")" && pwd)"
docker-run --gpus all --volume $(dirname "$DIR"):/home/rosuser --image rwthika/acdc:ros2-cuda-tf