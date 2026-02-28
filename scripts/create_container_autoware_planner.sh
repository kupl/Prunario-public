SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
PROJECT_DIR=$SCRIPTPATH/..

DOCKER_IMAGE=autoware_universe_planner
DOCKER_FILE=$SCRIPTPATH/../docker/Dockerfile
# AUTOWARE_VERSION=latest

if [ ! "$(docker images -q ${DOCKER_IMAGE})" ]; then
    echo "${DOCKER_IMAGE} does not exist. Creating..."
    docker build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -f ${DOCKER_FILE} -t ${DOCKER_IMAGE} .
fi

rocker --nvidia \
       --privileged \
       --x11 \
       --user \
       --network host \
       --volume $PROJECT_DIR/autoware_planner:$HOME/autoware \
       --volume $PROJECT_DIR/data/carla_map:/$HOME/carla_map \
       -- ${DOCKER_IMAGE}
